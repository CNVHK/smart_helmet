"""蜂鸣器报警驱动。"""

import time


def _sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class Buzzer:
    """有源蜂鸣器，按 warning level 执行本地报警。"""

    def __init__(self, pin=None):
        """保存 GPIO 引脚对象。"""
        self.pin = pin
        self._available = pin is not None

    def init(self):
        """关闭蜂鸣器。"""
        self.off()
        return self._available

    def check(self):
        """返回蜂鸣器是否可用。"""
        return self._available

    def on(self):
        """打开蜂鸣器。"""
        if self._available:
            self.pin.value(1)

    def off(self):
        """关闭蜂鸣器。"""
        if self._available:
            self.pin.value(0)

    def beep(self, count=1, on_ms=120, off_ms=120):
        """短响指定次数。"""
        for _ in range(count):
            self.on()
            _sleep_ms(on_ms)
            self.off()
            _sleep_ms(off_ms)

    def alert(self, level):
        """根据预警等级执行蜂鸣器策略。"""
        if level == "suspected":
            self.beep(1)
        elif level == "light":
            self.beep(2)
        elif level == "medium":
            self.beep(5, 80, 80)
        elif level == "severe":
            self.on()
            _sleep_ms(10000)
            self.off()
        elif level == "sos":
            self.on()

