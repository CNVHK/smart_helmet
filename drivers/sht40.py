"""SHT40 温湿度驱动。"""

import time

SHT40_ADDR = 0x44
CMD_MEASURE_HIGH_PRECISION = 0xFD


def _sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class SHT40:
    """SHT40 温湿度传感器，输出 temperature(℃) 和 humidity(%RH)。"""

    def __init__(self, i2c, addr=SHT40_ADDR):
        """保存 I2C 总线和地址。"""
        self.i2c = i2c
        self.addr = addr
        self._available = False

    def init(self):
        """探测设备是否存在。"""
        try:
            self.i2c.writeto(self.addr, bytes([CMD_MEASURE_HIGH_PRECISION]))
            _sleep_ms(10)
            self.i2c.readfrom(self.addr, 6)
            self._available = True
            return True
        except Exception as exc:
            self._available = False
            print("[SHT40] init failed:", exc)
            return False

    def check(self):
        """返回设备在线状态。"""
        return self._available

    def read(self):
        """读取温湿度并返回统一字典。"""
        if not self._available:
            return self._result(False, None, "sht40_not_available")
        try:
            self.i2c.writeto(self.addr, bytes([CMD_MEASURE_HIGH_PRECISION]))
            _sleep_ms(10)
            data = self.i2c.readfrom(self.addr, 6)
            raw_t = (data[0] << 8) | data[1]
            raw_h = (data[3] << 8) | data[4]
            temperature = -45.0 + 175.0 * raw_t / 65535.0
            humidity = -6.0 + 125.0 * raw_h / 65535.0
            humidity = min(100.0, max(0.0, humidity))
            if not (-40 <= temperature <= 125):
                return self._result(False, None, "sht40_value_invalid")
            return self._result(True, {
                "temperature": round(temperature, 2),
                "humidity": round(humidity, 2),
            }, None)
        except Exception as exc:
            print("[SHT40] read failed:", exc)
            return self._result(False, None, "sht40_read_failed")

    def _crc8(self, data):
        """预留 CRC8 校验函数，当前读取流程不强制校验。"""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "sht40", "data": data, "error": error}

