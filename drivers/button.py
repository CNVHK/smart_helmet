"""按键驱动。"""


class Button:
    """GPIO 按键，默认低电平触发。"""

    def __init__(self, pin=None, active_low=True):
        """保存按键引脚和触发电平。"""
        self.pin = pin
        self.active_low = active_low
        self._available = pin is not None

    def init(self):
        """按键无需额外初始化。"""
        return self._available

    def check(self):
        """返回按键是否可用。"""
        return self._available

    def is_pressed(self):
        """读取按键状态。"""
        if not self._available:
            return False
        value = self.pin.value()
        return value == 0 if self.active_low else value == 1
