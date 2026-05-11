"""SHT40 单项测试：打印温湿度。"""

import time
import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from machine import I2C
from drivers.sht40 import SHT40


def main():
    """初始化 SHT40 并循环打印数据。"""
    i2c = I2C(config.I2C_ID, freq=config.I2C_FREQ)
    devices = i2c.scan() if hasattr(i2c, "scan") else []
    print("I2C devices:", [hex(addr) for addr in devices])
    sensor = SHT40(i2c, addr=config.SHT40_ADDR)
    sensor.init()
    while True:
        print(sensor.read())
        time.sleep(1)


main()
