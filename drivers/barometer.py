"""BMP280 气压传感器简化驱动。

当前实现只完成在线探测和接口占位，避免未接设备影响主循环。后续可补全补偿公式。
"""


class Barometer:
    """气压传感器统一接口。"""

    def __init__(self, i2c=None, addr=0x76):
        """保存 I2C 总线和地址。"""
        self.i2c = i2c
        self.addr = addr
        self._available = False

    def init(self):
        """探测 BMP280 芯片 ID。"""
        if self.i2c is None:
            return False
        try:
            chip_id = self.i2c.readfrom_mem(self.addr, 0xD0, 1)[0]
            self._available = chip_id in (0x58, 0x60)
            return self._available
        except Exception as exc:
            print("[Barometer] init failed:", exc)
            self._available = False
            return False

    def check(self):
        """返回在线状态。"""
        return self._available

    def read(self):
        """读取气压；未补偿时返回不可用状态。"""
        if not self._available:
            return {"ok": False, "sensor": "barometer", "data": None, "error": "barometer_not_available"}
        return {"ok": False, "sensor": "barometer", "data": None, "error": "barometer_compensation_not_implemented"}

