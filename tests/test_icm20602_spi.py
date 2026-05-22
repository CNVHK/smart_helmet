"""ICM-20602 SPI smoke test."""

import sys
import time

try:
    sys.path.append("..")
except Exception:
    pass

import config
from drivers.imu_icm20602_spi import ICM20602SPI, WHO_AM_I
from machine import SPI, Pin


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def make_pin(pin_id, mode=None, pin_name=None):
    ids = []
    if pin_name is not None:
        ids.append(pin_name)
    ids.append(pin_id)
    for item in ids:
        try:
            return Pin(item, mode) if mode is not None else Pin(item)
        except Exception as exc:
            print("pin failed:", item, exc)
    return None


def main():
    spi = SPI(
        config.SPI_ID,
        baudrate=config.SPI_BAUDRATE,
        bits=getattr(config, "SPI_BITS", 8),
        polarity=getattr(config, "SPI_POLARITY", 0),
        phase=getattr(config, "SPI_PHASE", 0),
    )
    cs = make_pin(
        getattr(config, "ICM20602_CS_PIN", None),
        Pin.OUT,
        getattr(config, "ICM20602_CS_PIN_NAME", None),
    )
    imu = ICM20602SPI(spi, cs)
    ok = imu.init()
    print("init:", ok)
    try:
        print("WHO_AM_I:", hex(imu._read_reg(WHO_AM_I)))
    except Exception as exc:
        print("WHO_AM_I read failed:", exc)

    while True:
        print(imu.read())
        sleep_ms(200)


main()
