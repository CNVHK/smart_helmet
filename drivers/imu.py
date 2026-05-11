"""LIS2DH12 IMU 驱动。

基于用户已有 drivers/imu.py 改写为统一返回格式。LIS2DH12 只有三轴加速度，
没有陀螺仪，因此 gx/gy/gz 返回 None。
"""

import time

LIS2DH12_ADDR = 0x19
REG_WHO_AM_I = 0x0F
REG_CTRL_REG1 = 0x20
REG_CTRL_REG4 = 0x23
REG_OUT_X_L = 0x28
WHO_AM_I_VAL = 0x33

SCALE_2G = 0x00
SCALE_4G = 0x10
SCALE_8G = 0x20
SCALE_16G = 0x30

SENSITIVITY_G = {
    SCALE_2G: 0.001,
    SCALE_4G: 0.002,
    SCALE_8G: 0.004,
    SCALE_16G: 0.012,
}

def _sleep_ms(ms):
    """兼容 CPython 测试环境和 MicroPython 的毫秒延时。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class IMU:
    """LIS2DH12 三轴加速度计驱动，输出统一传感器字典。"""

    def __init__(self, i2c, addr=LIS2DH12_ADDR, scale=SCALE_2G):
        """保存 I2C 对象和量程配置。"""
        self.i2c = i2c
        self.addr = addr
        self.scale = scale
        self.sensitivity = SENSITIVITY_G.get(scale, SENSITIVITY_G[SCALE_2G])
        self.offset_g = {"ax": 0.0, "ay": 0.0, "az": 0.0}
        self._available = False

    def init(self):
        """检查芯片 ID 并配置 100Hz 高分辨率采样。"""
        try:
            who = self._read(REG_WHO_AM_I, 1)[0]
            if who != WHO_AM_I_VAL:
                self._available = False
                print("[IMU] WHO_AM_I mismatch:", who)
                return False
            self._write(REG_CTRL_REG1, 0x57)
            self._write(REG_CTRL_REG4, 0x88 | self.scale)
            _sleep_ms(20)
            self._available = True
            return True
        except Exception as exc:
            self._available = False
            print("[IMU] init failed:", exc)
            return False

    def check(self):
        """返回设备是否在线。"""
        return self._available

    def calibrate(self, samples=50):
        """静止零偏校准，az 默认扣除 1g 重力。"""
        sx = sy = sz = 0.0
        count = 0
        for _ in range(samples):
            raw = self._read_acc_g()
            if raw is not None:
                sx += raw["ax"]
                sy += raw["ay"]
                sz += raw["az"] - 1.0
                count += 1
            _sleep_ms(10)
        if count:
            self.offset_g = {"ax": sx / count, "ay": sy / count, "az": sz / count}
        return count > 0

    def read(self):
        """读取加速度，单位 g；陀螺仪字段保留为 None。"""
        if not self._available:
            return self._result(False, None, "imu_not_available")
        try:
            acc_g = self._read_acc_g()
            if acc_g is None:
                return self._result(False, None, "imu_read_failed")
            data = {
                "ax": round(acc_g["ax"] - self.offset_g["ax"], 3),
                "ay": round(acc_g["ay"] - self.offset_g["ay"], 3),
                "az": round(acc_g["az"] - self.offset_g["az"], 3),
                "gx": None,
                "gy": None,
                "gz": None,
            }
            return self._result(True, data, None)
        except Exception as exc:
            print("[IMU] read failed:", exc)
            return self._result(False, None, "imu_read_failed")

    @property
    def available(self):
        """兼容旧代码的在线状态属性。"""
        return self._available

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg, length):
        return self.i2c.readfrom_mem(self.addr, reg, length)

    def _read_acc_g(self):
        data = self._read(REG_OUT_X_L | 0x80, 6)
        x = self._to_signed((data[1] << 8) | data[0]) >> 4
        y = self._to_signed((data[3] << 8) | data[2]) >> 4
        z = self._to_signed((data[5] << 8) | data[4]) >> 4
        return {
            "ax": x * self.sensitivity,
            "ay": y * self.sensitivity,
            "az": z * self.sensitivity,
        }

    def _to_signed(self, value):
        return value - 65536 if value > 32767 else value

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "imu", "data": data, "error": error}
