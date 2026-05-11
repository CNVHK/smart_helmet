"""Quectel 官方 GNSS 驱动封装。"""

import time

try:
    import config
except ImportError:
    config = None


def _ticks_ms():
    """毫秒计时兼容函数。"""
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _ticks_diff(now, old):
    """计时差兼容函数。"""
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


class QuectelGNSS:
    """使用 quectel.GNSS 获取定位信息，输出统一 GPS 数据格式。"""

    def __init__(self):
        """尝试导入 quectel 模块。"""
        self._available = False
        self.gnss = None
        self.last_data = {"valid": False, "latitude": None, "longitude": None, "speed_kmh": None, "utc_time": None}
        self.last_read_ms = 0
        try:
            import quectel

            self.gnss = quectel.GNSS()
        except Exception as exc:
            print("[GNSS] quectel module not available:", exc)

    def init(self):
        """启动 GNSS。"""
        if self.gnss is None:
            return False
        try:
            self._available = bool(self.gnss.start())
            return self._available
        except Exception as exc:
            print("[GNSS] start failed:", exc)
            self._available = False
            return False

    def check(self):
        """返回 GNSS 是否已启动。"""
        return self._available

    def read(self):
        """读取官方定位字典并转换为统一字段。"""
        if not self._available:
            return {"ok": True, "sensor": "gps", "data": self.last_data, "error": None}
        interval = getattr(config, "GNSS_READ_INTERVAL_MS", 5000) if config else 5000
        now = _ticks_ms()
        if self.last_read_ms and _ticks_diff(now, self.last_read_ms) < interval:
            return {"ok": True, "sensor": "gps", "data": self.last_data, "error": None}
        self.last_read_ms = now
        try:
            loc = self.gnss.get_location()
            if loc:
                self.last_data = {
                    "valid": True,
                    "latitude": loc.get("latitude"),
                    "longitude": loc.get("longitude"),
                    "speed_kmh": loc.get("speed_kmh") or loc.get("speed"),
                    "utc_time": loc.get("utc_time") or loc.get("time"),
                }
            else:
                self.last_data["valid"] = False
            return {"ok": True, "sensor": "gps", "data": self.last_data, "error": None}
        except Exception as exc:
            # +CME ERROR: 516 通常表示当前无有效定位。保持上一帧数据，避免主循环报错。
            self.last_data["valid"] = False
            return {"ok": True, "sensor": "gps", "data": self.last_data, "error": None}

    def stop(self):
        """停止 GNSS。"""
        try:
            if self.gnss:
                self.gnss.stop()
        except Exception:
            pass
