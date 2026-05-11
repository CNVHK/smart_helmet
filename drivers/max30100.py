"""MAX30100 心率/血氧传感器驱动。

驱动负责：
1. 初始化 MAX30100 到 HR + SpO2 模式
2. 读取 FIFO 中 IR/RED 原始样本
3. 根据 IR 波形做简单峰值心率估算
4. 根据 red/IR 的 AC/DC 比值估算 SpO2

说明：当前算法是轻量工程版，适合先跑通链路和演示；医疗级精度需要更完整的滤波、
运动伪影处理、校准曲线和稳定窗口。
"""

import time

MAX30100_ADDR = 0x57

REG_INT_STATUS = 0x00
REG_INT_ENABLE = 0x01
REG_FIFO_WR_PTR = 0x02
REG_OVF_COUNTER = 0x03
REG_FIFO_RD_PTR = 0x04
REG_FIFO_DATA = 0x05
REG_MODE_CONFIG = 0x06
REG_SPO2_CONFIG = 0x07
REG_LED_CONFIG = 0x09
REG_REV_ID = 0xFE
REG_PART_ID = 0xFF

MODE_SPO2_HR = 0x03
MODE_RESET = 0x40

SPO2_CONFIG_50HZ_1600US = 0x43
LED_CONFIG_DEFAULT = 0x77

FIFO_DEPTH = 16
SAMPLE_RATE_HZ = 50
BUFFER_SIZE = 150


def _sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def _ticks_ms():
    """毫秒计时兼容函数。"""
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _ticks_diff(now, old):
    """计时差兼容函数。"""
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


class MAX30100:
    """MAX30100 心率/血氧传感器，输出 v1.2 vital 字段。"""

    def __init__(self, i2c, addr=MAX30100_ADDR, led_config=LED_CONFIG_DEFAULT):
        """保存 I2C 总线和配置。"""
        self.i2c = i2c
        self.addr = addr
        self.led_config = led_config
        self._available = i2c is not None
        self.ir_buffer = []
        self.red_buffer = []
        self.last_sample_ms = _ticks_ms()
        self.last_hr = None
        self.last_spo2 = None
        self.last_raw = {"ir": None, "red": None, "count": 0}

    def init(self):
        """初始化芯片和 FIFO。"""
        if self.i2c is None:
            self._available = False
            return False
        try:
            part_id = self._read_reg(REG_PART_ID)
            if part_id != 0x11:
                print("[MAX30100] unexpected part_id:", hex(part_id))
            self._reset()
            self._clear_fifo()
            self._write_reg(REG_INT_ENABLE, 0x00)
            self._write_reg(REG_SPO2_CONFIG, SPO2_CONFIG_50HZ_1600US)
            self._write_reg(REG_LED_CONFIG, self.led_config)
            self._write_reg(REG_MODE_CONFIG, MODE_SPO2_HR)
            self._clear_fifo()
            _sleep_ms(100)
            self._available = True
            return True
        except Exception as exc:
            print("[MAX30100] init failed:", exc)
            self._available = False
            return False

    def check(self):
        """返回设备是否在线。"""
        return self._available

    def read(self):
        """读取 FIFO 并输出心率、血氧和接触状态。"""
        if not self._available:
            return self._result(False, None, "max30100_not_available")
        try:
            samples = self._read_all_samples()
            self._append_samples(samples)
            hr = self._estimate_hr()
            spo2 = self._estimate_spo2()
            contact = self._estimate_contact()
            if hr is not None:
                self.last_hr = hr
            if spo2 is not None:
                self.last_spo2 = spo2
            data = {
                "hr": self.last_hr,
                "spo2": self.last_spo2,
                "hr_valid": 1 if self.last_hr is not None and contact else 0,
                "spo2_valid": 1 if self.last_spo2 is not None and contact else 0,
                "contact": contact,
                "raw": self.last_raw,
            }
            return self._result(True, data, None)
        except Exception as exc:
            print("[MAX30100] read failed:", exc)
            self._available = False
            return self._result(False, None, "max30100_read_failed")

    def _reset(self):
        """软复位芯片。"""
        self._write_reg(REG_MODE_CONFIG, MODE_RESET)
        for _ in range(20):
            if not (self._read_reg(REG_MODE_CONFIG) & MODE_RESET):
                return True
            _sleep_ms(10)
        return False

    def _clear_fifo(self):
        """清空 FIFO 指针和溢出计数。"""
        self._write_reg(REG_FIFO_WR_PTR, 0x00)
        self._write_reg(REG_OVF_COUNTER, 0x00)
        self._write_reg(REG_FIFO_RD_PTR, 0x00)

    def _fifo_available(self):
        """返回 FIFO 可读样本数。"""
        wr = self._read_reg(REG_FIFO_WR_PTR) & 0x0F
        rd = self._read_reg(REG_FIFO_RD_PTR) & 0x0F
        ovf = self._read_reg(REG_OVF_COUNTER) & 0x0F
        count = (wr - rd) & 0x0F
        return FIFO_DEPTH if ovf else count

    def _read_all_samples(self):
        """读取当前 FIFO 中所有样本。"""
        count = self._fifo_available()
        samples = []
        for _ in range(count):
            raw = self._read_regs(REG_FIFO_DATA, 4)
            ir = (raw[0] << 8) | raw[1]
            red = (raw[2] << 8) | raw[3]
            samples.append((ir, red))
        if count >= FIFO_DEPTH:
            self._write_reg(REG_OVF_COUNTER, 0x00)
        if samples:
            ir, red = samples[-1]
            self.last_raw = {"ir": ir, "red": red, "count": len(samples)}
        return samples

    def _append_samples(self, samples):
        """把新样本放入固定长度缓存。"""
        for ir, red in samples:
            self.ir_buffer.append(ir)
            self.red_buffer.append(red)
        if len(self.ir_buffer) > BUFFER_SIZE:
            self.ir_buffer = self.ir_buffer[-BUFFER_SIZE:]
            self.red_buffer = self.red_buffer[-BUFFER_SIZE:]

    def _estimate_contact(self):
        """根据 DC 强度和波动粗略判断是否接触。"""
        if len(self.ir_buffer) < 10:
            return 0
        recent_ir = self.ir_buffer[-30:]
        recent_red = self.red_buffer[-30:]
        ir_avg = sum(recent_ir) / len(recent_ir)
        red_avg = sum(recent_red) / len(recent_red)
        ir_span = max(recent_ir) - min(recent_ir)
        red_span = max(recent_red) - min(recent_red)
        return 1 if ir_avg > 1000 and red_avg > 1000 and (ir_span > 10 or red_span > 10) else 0

    def _estimate_hr(self):
        """用 IR 波形局部峰值估算心率。"""
        values = self.ir_buffer
        if len(values) < SAMPLE_RATE_HZ * 2:
            return None
        window = values[-BUFFER_SIZE:]
        mean = sum(window) / len(window)
        centered = [v - mean for v in window]
        abs_avg = sum([abs(v) for v in centered]) / len(centered)
        threshold = max(20, abs_avg * 0.6)
        peaks = []
        min_gap = int(SAMPLE_RATE_HZ * 0.35)
        last_peak = -min_gap
        for i in range(1, len(centered) - 1):
            if i - last_peak < min_gap:
                continue
            if centered[i] > threshold and centered[i] > centered[i - 1] and centered[i] >= centered[i + 1]:
                peaks.append(i)
                last_peak = i
        if len(peaks) < 2:
            return None
        intervals = []
        for i in range(1, len(peaks)):
            intervals.append(peaks[i] - peaks[i - 1])
        avg_interval = sum(intervals) / len(intervals)
        if avg_interval <= 0:
            return None
        hr = 60.0 * SAMPLE_RATE_HZ / avg_interval
        if 40 <= hr <= 200:
            return int(round(hr))
        return None

    def _estimate_spo2(self):
        """用 AC/DC 比值估算 SpO2。"""
        if len(self.ir_buffer) < SAMPLE_RATE_HZ * 2:
            return None
        ir = self.ir_buffer[-BUFFER_SIZE:]
        red = self.red_buffer[-BUFFER_SIZE:]
        ir_dc = sum(ir) / len(ir)
        red_dc = sum(red) / len(red)
        if ir_dc <= 0 or red_dc <= 0:
            return None
        ir_ac = max(ir) - min(ir)
        red_ac = max(red) - min(red)
        if ir_ac <= 0 or red_ac <= 0:
            return None
        ratio = (red_ac / red_dc) / (ir_ac / ir_dc)
        spo2 = 110.0 - 25.0 * ratio
        if 70 <= spo2 <= 100:
            return int(round(spo2))
        return None

    def _write_reg(self, reg, value):
        self.i2c.writeto_mem(self.addr, reg, bytes([value]))

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _read_regs(self, reg, length):
        return self.i2c.readfrom_mem(self.addr, reg, length)

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "max30100", "data": data, "error": error}
