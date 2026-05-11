"""AHT20 温湿度驱动，参考 Uniknect 官方示例。"""

import time

AHT20_ADDR = 0x38
CMD_INIT = b"\xBE\x08\x00"
CMD_TRIGGER = b"\xAC\x33\x00"
CMD_RESET = b"\xBA"


def _sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class AHT20:
    """AHT20 温湿度传感器，输出 temperature(℃) 和 humidity(%RH)。"""

    def __init__(self, i2c, addr=AHT20_ADDR):
        """保存 I2C 总线和地址。"""
        self.i2c = i2c
        self.addr = addr
        self._available = False

    def init(self):
        """复位并初始化 AHT20。"""
        try:
            self.i2c.writeto(self.addr, CMD_RESET)
            _sleep_ms(20)
            self.i2c.writeto(self.addr, CMD_INIT)
            _sleep_ms(10)
            self._available = True
            return True
        except Exception as exc:
            print("[AHT20] init failed:", exc)
            self._available = False
            return False

    def check(self):
        """返回设备在线状态。"""
        return self._available

    def read(self):
        """读取温湿度并返回统一字典。"""
        if not self._available:
            return {"ok": False, "sensor": "aht20", "data": None, "error": "aht20_not_available"}
        try:
            self.i2c.writeto(self.addr, CMD_TRIGGER)
            _sleep_ms(80)
            buf = bytearray(6)
            if hasattr(self.i2c, "readfrom_into"):
                self.i2c.readfrom_into(self.addr, buf)
            else:
                data = self.i2c.readfrom(self.addr, 6)
                for i in range(6):
                    buf[i] = data[i]
            humidity_raw = (buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4)
            temperature_raw = ((buf[3] & 0x0F) << 16) | (buf[4] << 8) | buf[5]
            humidity = humidity_raw * 100.0 / 0x100000
            temperature = temperature_raw * 200.0 / 0x100000 - 50.0
            if not (-40 <= temperature <= 85) or not (0 <= humidity <= 100):
                return {"ok": False, "sensor": "aht20", "data": None, "error": "aht20_value_invalid"}
            return {
                "ok": True,
                "sensor": "aht20",
                "data": {"temperature": round(temperature, 2), "humidity": round(humidity, 2)},
                "error": None,
            }
        except Exception as exc:
            print("[AHT20] read failed:", exc)
            return {"ok": False, "sensor": "aht20", "data": None, "error": "aht20_read_failed"}

