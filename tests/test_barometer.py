"""BMP280 实机测试：直接读取板子上的气压传感器。"""

import sys
import time

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from machine import I2C
from drivers.barometer import Barometer


def main():
    i2c = I2C(config.I2C_ID, freq=config.I2C_FREQ)
    devices = i2c.scan() if hasattr(i2c, "scan") else []
    print("I2C devices:", [hex(addr) for addr in devices])

    sensor = Barometer(i2c, addr=config.BAROMETER_ADDR)
    ok = sensor.init()
    print("BMP280 init:", ok)
    if not ok:
        print("BMP280 not detected at:", hex(config.BAROMETER_ADDR))
        return

    while True:
        packet = sensor.read()
        print(packet)
        time.sleep(1)


main()
