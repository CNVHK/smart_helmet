"""GY302/BH1750 light sensor driver over I2C."""

import time


GY302_ADDR = 0x23
GY302_ALT_ADDR = 0x5C

CMD_POWER_ON = 0x01
CMD_RESET = 0x07
CMD_CONT_HIGH_RES = 0x10

DEFAULT_MTREG = 69
MEASUREMENT_TIME_MS = 180


def _sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class GY302:
    """GY302 module using the BH1750FVI ambient light sensor.

    read() returns lux from the actual I2C sensor data. The data payload keeps a
    "light" field for compatibility with the old ADC light sensor placeholder.
    """

    def __init__(self, i2c=None, addr=GY302_ADDR, auto_detect=True, mtreg=DEFAULT_MTREG):
        self.i2c = i2c
        self.addr = addr
        self.auto_detect = auto_detect
        self.mtreg = mtreg
        self._available = False

    def init(self):
        """Detect the module and power on the BH1750."""
        if self.i2c is None:
            self._available = False
            return False
        try:
            self.addr = self._select_addr()
            self._write_cmd(CMD_POWER_ON)
            _sleep_ms(10)
            self._write_cmd(CMD_RESET)
            _sleep_ms(10)
            self._write_cmd(CMD_CONT_HIGH_RES)
            _sleep_ms(MEASUREMENT_TIME_MS)
            self._available = True
            return True
        except Exception as exc:
            print("[GY302] init failed:", exc)
            self._available = False
            return False

    def check(self):
        return self._available

    def read(self):
        """Read the latest high-resolution measurement and return lux."""
        if not self._available:
            return self._result(False, None, "gy302_not_available")
        try:
            data = self.i2c.readfrom(self.addr, 2)
            raw = (data[0] << 8) | data[1]
            lux = self._raw_to_lux(raw)
            return self._result(True, {
                "light": round(lux, 2),
                "lux": round(lux, 2),
                "raw": raw,
            }, None)
        except Exception as exc:
            print("[GY302] read failed:", exc)
            self._available = False
            return self._result(False, None, "gy302_read_failed")

    def _select_addr(self):
        if not self.auto_detect or not hasattr(self.i2c, "scan"):
            return self.addr
        devices = self.i2c.scan()
        if self.addr in devices:
            return self.addr
        if GY302_ALT_ADDR in devices:
            return GY302_ALT_ADDR
        return self.addr

    def _write_cmd(self, cmd):
        self.i2c.writeto(self.addr, bytes([cmd & 0xFF]))

    def _raw_to_lux(self, raw):
        return raw / (1.2 * (self.mtreg / DEFAULT_MTREG))

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "gy302", "data": data, "error": error}
