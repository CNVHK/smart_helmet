"""GPS 单项测试：打印 RMC 解析结果。"""

import time
import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from machine import UART
from drivers.gps_uart import GPSUART


def main():
    """初始化 GPS UART 并循环打印解析结果。"""
    uart = UART(config.GPS_UART_ID, config.GPS_BAUDRATE)
    gps = GPSUART(uart)
    gps.init()
    while True:
        print(gps.read())
        time.sleep(1)


main()
