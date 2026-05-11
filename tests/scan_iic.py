import machine
import time

i2c = machine.I2C(1, freq=100000)

while True:
    devices = i2c.scan()
    print([hex(addr) for addr in devices])
    time.sleep(1)