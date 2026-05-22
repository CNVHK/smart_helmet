"""ICM-20602 SPI IMU driver.

The driver keeps the same output shape as the existing IMU driver so the
algorithm and data packet layers do not need to know which IMU is installed.
Acceleration is reported in g and gyroscope values are reported in deg/s.
"""

import time


WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D
I2C_IF = 0x70
ACCEL_XOUT_H = 0x3B

READ_FLAG = 0x80
WHO_AM_I_VAL = 0x12

ACCEL_SENS_2G = 16384.0
GYRO_SENS_250DPS = 131.0


def _sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class ICM20602SPI:
    """SPI ICM-20602 6-axis IMU driver with unified sensor output."""

    def __init__(self, spi, cs_pin=None, cs_always_low=False):
        """Keep the SPI bus and chip-select pin."""
        self.spi = spi
        self.cs = cs_pin
        self.cs_always_low = bool(cs_always_low)
        self.offset = {
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "gx": 0.0,
            "gy": 0.0,
            "gz": 0.0,
        }
        self._available = False
        self.last_error = "not_initialized"
        if self.cs is not None and not self.cs_always_low:
            self.cs.value(1)

    def init(self):
        """Reset the chip, verify WHO_AM_I and configure +-2g / +-250dps."""
        if self.spi is None:
            self._available = False
            self.last_error = "spi_not_available"
            print("[ICM20602] SPI not available")
            return False
        if self.cs is None and not self.cs_always_low:
            self._available = False
            self.last_error = "cs_pin_not_available"
            print("[ICM20602] CS pin not available")
            return False
        try:
            who_before = self._read_reg(WHO_AM_I)
            print("[ICM20602] WHO_AM_I before init:", hex(who_before))

            self._write_reg(PWR_MGMT_1, 0x80)
            _sleep_ms(100)
            self._write_reg(PWR_MGMT_1, 0x01)
            _sleep_ms(10)
            self._write_reg(PWR_MGMT_2, 0x00)
            self._write_reg(SMPLRT_DIV, 0x09)
            self._write_reg(CONFIG, 0x01)
            self._write_reg(GYRO_CONFIG, 0x00)
            self._write_reg(ACCEL_CONFIG, 0x00)
            self._write_reg(ACCEL_CONFIG2, 0x03)
            self._write_reg(I2C_IF, 0x40)
            _sleep_ms(20)

            who = self._read_reg(WHO_AM_I)
            if who != WHO_AM_I_VAL:
                print("[ICM20602] WHO_AM_I mismatch:", hex(who))
                self._available = False
                self.last_error = "who_am_i_mismatch_" + hex(who)
                return False
            self._available = True
            self.last_error = None
            return True
        except Exception as exc:
            print("[ICM20602] init failed:", exc)
            self._available = False
            self.last_error = "imu_init_failed"
            return False

    def check(self):
        """Return whether the chip passed initialization."""
        return self._available

    def calibrate(self, samples=100):
        """Calibrate static offsets. Keep 1g on Z axis as gravity."""
        if not self._available:
            return False
        sums = {
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "gx": 0.0,
            "gy": 0.0,
            "gz": 0.0,
        }
        count = 0
        for _ in range(samples):
            data = self._read_motion()
            if data:
                sums["ax"] += data["ax"]
                sums["ay"] += data["ay"]
                sums["az"] += data["az"] - 1.0
                sums["gx"] += data["gx"]
                sums["gy"] += data["gy"]
                sums["gz"] += data["gz"]
                count += 1
            _sleep_ms(10)
        if not count:
            return False
        for key in sums:
            self.offset[key] = sums[key] / count
        return True

    def read(self):
        """Read acceleration and gyroscope data in unified dict format."""
        if not self._available:
            return self._result(False, None, self.last_error or "imu_not_available")
        try:
            raw = self._read_motion()
            if raw is None:
                return self._result(False, None, "imu_read_failed")
            data = {
                "ax": round(raw["ax"] - self.offset["ax"], 3),
                "ay": round(raw["ay"] - self.offset["ay"], 3),
                "az": round(raw["az"] - self.offset["az"], 3),
                "gx": round(raw["gx"] - self.offset["gx"], 3),
                "gy": round(raw["gy"] - self.offset["gy"], 3),
                "gz": round(raw["gz"] - self.offset["gz"], 3),
            }
            return self._result(True, data, None)
        except Exception as exc:
            print("[ICM20602] read failed:", exc)
            self._available = False
            self.last_error = "imu_read_failed"
            return self._result(False, None, "imu_read_failed")

    @property
    def available(self):
        """Compatibility status attribute."""
        return self._available

    def _select(self):
        if self.cs is not None and not self.cs_always_low:
            self.cs.value(0)

    def _deselect(self):
        if self.cs is not None and not self.cs_always_low:
            self.cs.value(1)

    def _write_reg(self, reg, value):
        self._select()
        try:
            self.spi.write(bytes([reg & 0x7F, value & 0xFF]))
        finally:
            self._deselect()

    def _read_reg(self, reg):
        data = self._read_regs(reg, 1)
        return data[0]

    def _read_regs(self, reg, length):
        self._select()
        try:
            tx = bytes([reg | READ_FLAG]) + bytes([0x00] * length)
            rx = bytearray(length + 1)
            if hasattr(self.spi, "write_readinto"):
                self.spi.write_readinto(tx, rx)
                return bytes(rx[1:])
            self.spi.write(bytes([reg | READ_FLAG]))
            return self.spi.read(length, 0x00)
        finally:
            self._deselect()

    def _read_motion(self):
        data = self._read_regs(ACCEL_XOUT_H, 14)
        if data is None or len(data) != 14:
            return None

        ax_raw = self._int16(data[0], data[1])
        ay_raw = self._int16(data[2], data[3])
        az_raw = self._int16(data[4], data[5])
        gx_raw = self._int16(data[8], data[9])
        gy_raw = self._int16(data[10], data[11])
        gz_raw = self._int16(data[12], data[13])

        return {
            "ax": ax_raw / ACCEL_SENS_2G,
            "ay": ay_raw / ACCEL_SENS_2G,
            "az": az_raw / ACCEL_SENS_2G,
            "gx": gx_raw / GYRO_SENS_250DPS,
            "gy": gy_raw / GYRO_SENS_250DPS,
            "gz": gz_raw / GYRO_SENS_250DPS,
        }

    def _int16(self, msb, lsb):
        value = (msb << 8) | lsb
        return value - 65536 if value & 0x8000 else value

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "imu", "data": data, "error": error}
