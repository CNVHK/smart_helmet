"""MAX30100 心率/血氧模块测试。"""

import sys
import time

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import machine
import config
from drivers.max30100 import MAX30100


def main():
    """循环读取 MAX30100 驱动输出。"""
    i2c = machine.I2C(config.I2C_ID, freq=100000)
    print("scan:", [hex(x) for x in i2c.scan()])
    sensor = MAX30100(i2c)
    print("init:", sensor.init())
    while True:
        print(sensor.read())
        time.sleep_ms(100)


main()
