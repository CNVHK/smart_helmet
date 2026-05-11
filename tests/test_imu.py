"""IMU 单项测试：打印 ax ay az gx gy gz。"""

import time
import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from machine import I2C
from drivers.imu import IMU


def main():
    """初始化 IMU 并循环打印数据，ax/ay/az 单位为 g。"""
    i2c = I2C(config.I2C_ID, freq=config.I2C_FREQ)
    imu = IMU(i2c)
    imu.init()
    while True:
        packet = imu.read()
        data = packet.get("data") or {}
        print("ax={ax} ay={ay} az={az} gx={gx} gy={gy} gz={gz}".format(**{
            "ax": data.get("ax"),
            "ay": data.get("ay"),
            "az": data.get("az"),
            "gx": data.get("gx"),
            "gy": data.get("gy"),
            "gz": data.get("gz"),
        }))
        time.sleep(1)


main()
