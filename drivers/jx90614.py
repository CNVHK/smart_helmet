"""JX90614 / NSA2200 红外人体温度传感器驱动。

该模块不一定响应普通 I2C scan，当前按手册使用 7-bit 通配地址 0x7F：
1. CMD(0x30) 写 0x00 复位
2. CMD(0x30) 写 0x08 开启状态机/启动转换
3. 轮询 Data_Ready(0x02) bit3
4. 读取 To(0x10, 0x11, 0x12)，温度 T = T0 / 2^14
"""

import time

JX90614_DEFAULT_ADDR = 0x7F

REG_DATA_READY = 0x02
REG_TO0 = 0x10
REG_CMD = 0x30

CMD_RESET = 0x00
CMD_START = 0x08
READY_BIT = 0x08
TEMP_SCALE = 16384.0


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


class JX90614:
    """JX90614 人体温度传感器，输出 body_temp，单位摄氏度。"""

    def __init__(self, i2c, addr=JX90614_DEFAULT_ADDR):
        """保存 I2C 总线和设备地址。"""
        self.i2c = i2c
        self.addr = addr
        self._available = i2c is not None
        self.last_ready = 0

    def init(self):
        """用一次启动命令探测设备是否可通信。"""
        if self.i2c is None:
            self._available = False
            return False
        try:
            self._start_conversion()
            self._available = True
            return True
        except Exception as exc:
            print("[JX90614] init failed:", exc)
            self._available = False
            return False

    def check(self):
        """返回设备是否可用。"""
        return self._available

    def read(self):
        """读取人体温度并返回统一传感器字典。"""
        if not self._available:
            return self._result(False, None, "jx90614_not_available")
        try:
            self._start_conversion()
            if not self._wait_ready():
                return self._result(False, None, "jx90614_timeout")
            data = self._read(REG_TO0, 3)
            t0 = (data[0] << 16) | (data[1] << 8) | data[2]
            temp = t0 / TEMP_SCALE
            return self._result(True, {
                "body_temp": round(temp, 2),
                "body_temp_source": "infrared",
                "body_temp_raw": t0,
                "to_bytes": [data[0], data[1], data[2]],
            }, None)
        except Exception as exc:
            print("[JX90614] read failed:", exc)
            self._available = False
            return self._result(False, None, "jx90614_read_failed")

    def _start_conversion(self):
        """按手册复位 CMD 并开启状态机。"""
        self._write(REG_CMD, CMD_RESET)
        _sleep_ms(10)
        self._write(REG_CMD, CMD_START)
        _sleep_ms(10)

    def _wait_ready(self, timeout_ms=1000, interval_ms=20):
        """等待 Data_Ready bit3 置位。"""
        start = _ticks_ms()
        while _ticks_diff(_ticks_ms(), start) < timeout_ms:
            ready = self._read(REG_DATA_READY, 1)[0]
            self.last_ready = ready
            if ready & READY_BIT:
                return True
            _sleep_ms(interval_ms)
        return False

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg, n=1):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "jx90614", "data": data, "error": error}
