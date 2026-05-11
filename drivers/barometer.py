"""BMP280 pressure sensor driver over I2C."""

import time

BMP280_ADDR = 0x76
BMP280_CHIP_ID = 0x58

REG_CALIB = 0x88
REG_CHIP_ID = 0xD0
REG_RESET = 0xE0
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_PRESS_MSB = 0xF7

RESET_VALUE = 0xB6

# osrs_t x1, osrs_p x4, normal mode.
CTRL_MEAS_NORMAL = (1 << 5) | (3 << 2) | 3

# standby 250 ms, IIR filter x4, SPI 3-wire disabled.
CONFIG_NORMAL = (3 << 5) | (2 << 2)

SEA_LEVEL_PRESSURE_PA = 101325.0


def _sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class Barometer:
    """BMP280 barometer.

    read() returns:
      temperature: Celsius
      pressure_pa: Pa
      pressure_hpa: hPa
      altitude_m: estimated altitude from standard sea-level pressure
    """

    def __init__(self, i2c=None, addr=BMP280_ADDR, sea_level_pa=SEA_LEVEL_PRESSURE_PA):
        self.i2c = i2c
        self.addr = addr
        self.sea_level_pa = sea_level_pa
        self._available = False
        self._calib = None
        self._t_fine = 0

    def init(self):
        """Detect BMP280, load calibration, and enter normal mode."""
        if self.i2c is None:
            self._available = False
            return False
        try:
            chip_id = self._read_u8(REG_CHIP_ID)
            if chip_id != BMP280_CHIP_ID:
                self._available = False
                print("[BMP280] unexpected chip id:", hex(chip_id))
                return False

            self._write_u8(REG_RESET, RESET_VALUE)
            _sleep_ms(5)
            self._read_calibration()
            self._write_u8(REG_CONFIG, CONFIG_NORMAL)
            self._write_u8(REG_CTRL_MEAS, CTRL_MEAS_NORMAL)
            _sleep_ms(10)
            self._available = True
            return True
        except Exception as exc:
            print("[BMP280] init failed:", exc)
            self._available = False
            return False

    def check(self):
        return self._available

    def read(self):
        """Read compensated temperature and pressure."""
        if not self._available:
            return self._result(False, None, "bmp280_not_available")
        try:
            raw_pressure, raw_temperature = self._read_raw()
            temperature = self._compensate_temperature(raw_temperature)
            pressure_pa = self._compensate_pressure(raw_pressure)
            if pressure_pa <= 0:
                return self._result(False, None, "bmp280_pressure_invalid")

            altitude_m = 44330.0 * (1.0 - (pressure_pa / self.sea_level_pa) ** 0.1903)
            return self._result(True, {
                "temperature": round(temperature, 2),
                "pressure_pa": round(pressure_pa, 2),
                "pressure_hpa": round(pressure_pa / 100.0, 2),
                "altitude_m": round(altitude_m, 2),
            }, None)
        except Exception as exc:
            print("[BMP280] read failed:", exc)
            self._available = False
            return self._result(False, None, "bmp280_read_failed")

    def _read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, REG_PRESS_MSB, 6)
        raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return raw_pressure, raw_temperature

    def _read_calibration(self):
        data = self.i2c.readfrom_mem(self.addr, REG_CALIB, 24)
        self._calib = {
            "dig_T1": self._u16(data, 0),
            "dig_T2": self._s16(data, 2),
            "dig_T3": self._s16(data, 4),
            "dig_P1": self._u16(data, 6),
            "dig_P2": self._s16(data, 8),
            "dig_P3": self._s16(data, 10),
            "dig_P4": self._s16(data, 12),
            "dig_P5": self._s16(data, 14),
            "dig_P6": self._s16(data, 16),
            "dig_P7": self._s16(data, 18),
            "dig_P8": self._s16(data, 20),
            "dig_P9": self._s16(data, 22),
        }
        if self._calib["dig_P1"] == 0:
            raise ValueError("invalid calibration data")

    def _compensate_temperature(self, adc_t):
        c = self._calib
        var1 = (((adc_t >> 3) - (c["dig_T1"] << 1)) * c["dig_T2"]) >> 11
        var2 = (((((adc_t >> 4) - c["dig_T1"]) * ((adc_t >> 4) - c["dig_T1"])) >> 12) * c["dig_T3"]) >> 14
        self._t_fine = var1 + var2
        return ((self._t_fine * 5 + 128) >> 8) / 100.0

    def _compensate_pressure(self, adc_p):
        c = self._calib
        var1 = self._t_fine - 128000
        var2 = var1 * var1 * c["dig_P6"]
        var2 = var2 + ((var1 * c["dig_P5"]) << 17)
        var2 = var2 + (c["dig_P4"] << 35)
        var1 = ((var1 * var1 * c["dig_P3"]) >> 8) + ((var1 * c["dig_P2"]) << 12)
        var1 = ((((1 << 47) + var1) * c["dig_P1"]) >> 33)
        if var1 == 0:
            return 0.0
        pressure = 1048576 - adc_p
        pressure = (((pressure << 31) - var2) * 3125) // var1
        var1 = (c["dig_P9"] * (pressure >> 13) * (pressure >> 13)) >> 25
        var2 = (c["dig_P8"] * pressure) >> 19
        pressure = ((pressure + var1 + var2) >> 8) + (c["dig_P7"] << 4)
        return pressure / 256.0

    def _read_u8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _write_u8(self, reg, value):
        self.i2c.writeto_mem(self.addr, reg, bytes([value & 0xFF]))

    def _u16(self, data, offset):
        return data[offset] | (data[offset + 1] << 8)

    def _s16(self, data, offset):
        value = self._u16(data, offset)
        return value - 65536 if value & 0x8000 else value

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "barometer", "data": data, "error": error}
