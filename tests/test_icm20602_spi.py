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
        if isinstance(pin_name, (list, tuple)):
            ids.extend(pin_name)
        else:
            ids.append(pin_name)
    if pin_id is not None:
        ids.append(pin_id)
    for item in ids:
        try:
            return Pin(item, mode) if mode is not None else Pin(item)
        except Exception as exc:
            print("pin failed:", item, exc)
    return None


def make_spi():
    return SPI(
        config.SPI_ID,
        baudrate=config.SPI_BAUDRATE,
        bits=getattr(config, "SPI_BITS", 8),
        polarity=getattr(config, "SPI_POLARITY", 1),
        phase=getattr(config, "SPI_PHASE", 1),
    )


def main():
    print("spi id:", config.SPI_ID)
    print("spi baudrate:", config.SPI_BAUDRATE)
    print("spi mode:", getattr(config, "SPI_POLARITY", 1), getattr(config, "SPI_PHASE", 1))
    print("cs pin name:", getattr(config, "ICM20602_CS_PIN_NAME", None))

    spi = make_spi()
    cs = make_pin(
        getattr(config, "ICM20602_CS_PIN", None),
        Pin.OUT,
        getattr(config, "ICM20602_CS_PIN_NAME", None),
    )
    imu = ICM20602SPI(
        spi,
        cs,
        cs_always_low=bool(getattr(config, "ICM20602_CS_ALWAYS_LOW", False)),
    )
    print("WHO_AM_I before init:", hex(imu._read_reg(WHO_AM_I)))
    ok = imu.init()
    print("init:", ok)
    print("WHO_AM_I:", hex(imu._read_reg(WHO_AM_I)))

    while True:
        print(imu.read())
        sleep_ms(200)


main()
