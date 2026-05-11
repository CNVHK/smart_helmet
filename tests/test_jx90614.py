import time
import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import machine
import config
from drivers.jx90614 import JX90614


def main():
    """循环读取 JX90614 人体温度。"""
    i2c = machine.I2C(config.I2C_ID, freq=config.JX90614_I2C_FREQ)
    sensor = JX90614(i2c, addr=config.JX90614_ADDR)
    print("init:", sensor.init())
    while True:
        print(sensor.read())
        time.sleep(1)


main()
