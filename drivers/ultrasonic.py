"""HC-SR04 超声波测距驱动。"""

import time


def _sleep_us(us):
    """微秒延时兼容函数。"""
    if hasattr(time, "sleep_us"):
        time.sleep_us(us)
    else:
        time.sleep(us / 1000000.0)


def _ticks_us():
    """兼容获取微秒计时。"""
    return time.ticks_us() if hasattr(time, "ticks_us") else int(time.time() * 1000000)


def _ticks_diff(now, old):
    """兼容计时差值。"""
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


class Ultrasonic:
    """GPIO 触发式超声波测距，输出 front_cm。"""

    def __init__(self, trig=None, echo=None):
        """保存 trigger 和 echo 引脚对象。"""
        self.trig = trig
        self.echo = echo
        self._available = trig is not None and echo is not None

    def init(self):
        """设置初始电平。"""
        if self._available:
            self.trig.value(0)
        return self._available

    def check(self):
        """返回在线状态。"""
        return self._available

    def read(self, timeout_us=30000):
        """测距并返回厘米。"""
        if not self._available:
            return {"ok": False, "sensor": "ultrasonic", "data": None, "error": "ultrasonic_not_available"}
        try:
            self.trig.value(0)
            _sleep_us(2)
            self.trig.value(1)
            _sleep_us(10)
            self.trig.value(0)

            start_wait = _ticks_us()
            while self.echo.value() == 0:
                if _ticks_diff(_ticks_us(), start_wait) > timeout_us:
                    return {"ok": False, "sensor": "ultrasonic", "data": None, "error": "ultrasonic_timeout"}
            pulse_start = _ticks_us()
            while self.echo.value() == 1:
                if _ticks_diff(_ticks_us(), pulse_start) > timeout_us:
                    return {"ok": False, "sensor": "ultrasonic", "data": None, "error": "ultrasonic_timeout"}
            pulse = _ticks_diff(_ticks_us(), pulse_start)
            distance_cm = pulse / 58.0
            return {"ok": True, "sensor": "ultrasonic", "data": {"front_cm": round(distance_cm, 1)}, "error": None}
        except Exception as exc:
            print("[Ultrasonic] read failed:", exc)
            return {"ok": False, "sensor": "ultrasonic", "data": None, "error": "ultrasonic_read_failed"}

