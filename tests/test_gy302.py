"""GY302 real-board test: print lux from the I2C BH1750 sensor."""

import sys
import time

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from machine import I2C, Pin
from drivers.gy302 import GY302


def make_i2c():
    attempts = (
        lambda: I2C(config.I2C_ID, freq=config.I2C_FREQ),
        lambda: I2C(config.I2C_ID),
        lambda: I2C(config.I2C_ID, scl=Pin(config.I2C_SCL_PIN), sda=Pin(config.I2C_SDA_PIN), freq=config.I2C_FREQ),
        lambda: I2C(config.I2C_ID, scl=config.I2C_SCL_PIN, sda=config.I2C_SDA_PIN, freq=config.I2C_FREQ),
    )
    for attempt in attempts:
        try:
            return attempt()
        except Exception as exc:
            print("I2C init attempt failed:", exc)
    return None


def main():
    i2c = make_i2c()
    if i2c is None:
        print("I2C init failed")
        return
    devices = i2c.scan() if hasattr(i2c, "scan") else []
    print("I2C devices:", [hex(addr) for addr in devices])

    sensor = GY302(i2c, addr=config.GY302_ADDR)
    ok = sensor.init()
    print("GY302 init:", ok, "addr:", hex(sensor.addr))
    if not ok:
        print("GY302 not detected. Expected address:", hex(config.GY302_ADDR), "or", hex(config.GY302_ALT_ADDR))
        return

    while True:
        packet = sensor.read()
        data = packet.get("data") or {}
        print("lux={lux} raw={raw} packet={packet}".format(
            lux=data.get("lux"),
            raw=data.get("raw"),
            packet=packet,
        ))
        time.sleep(1)


main()
