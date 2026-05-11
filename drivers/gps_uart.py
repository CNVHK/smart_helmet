"""GPS UART NMEA 解析驱动。"""


class GPSUART:
    """读取 UART NMEA，优先解析 RMC 定位句。"""

    def __init__(self, uart=None):
        """保存 UART 对象；为空时降级运行。"""
        self.uart = uart
        self._available = uart is not None
        self.last_data = {"valid": False, "latitude": None, "longitude": None, "speed_kmh": None, "utc_time": None}

    def init(self):
        """GPS 无需额外初始化，检查 UART 是否存在。"""
        return self._available

    def check(self):
        """返回 UART 是否可用。"""
        return self._available

    def read(self):
        """读取一批 NMEA 数据并解析 RMC。"""
        if not self._available:
            return self._result(True, self.last_data, None)
        try:
            raw = self.uart.read()
            if not raw:
                return self._result(True, self.last_data, None)
            text = raw.decode("ascii", "ignore") if hasattr(raw, "decode") else str(raw)
            for line in text.splitlines():
                if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                    self.last_data = self.parse_rmc(line)
                    break
            return self._result(True, self.last_data, None)
        except Exception as exc:
            print("[GPS] read failed:", exc)
            return self._result(False, None, "gps_read_failed")

    def parse_rmc(self, sentence):
        """解析 RMC 语句，输出 decimal degree 坐标和 km/h 速度。"""
        try:
            body = sentence.split("*")[0]
            parts = body.split(",")
            valid = len(parts) > 6 and parts[2] == "A"
            latitude = self._nmea_to_decimal(parts[3], parts[4]) if valid else None
            longitude = self._nmea_to_decimal(parts[5], parts[6]) if valid else None
            speed_knots = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
            return {
                "valid": valid,
                "latitude": latitude,
                "longitude": longitude,
                "speed_kmh": round(speed_knots * 1.852, 2) if valid else None,
                "utc_time": parts[1] if len(parts) > 1 else None,
            }
        except Exception as exc:
            print("[GPS] parse failed:", exc)
            return {"valid": False, "latitude": None, "longitude": None, "speed_kmh": None, "utc_time": None}

    def _nmea_to_decimal(self, value, hemi):
        """把 ddmm.mmmm / dddmm.mmmm 转为十进制度。"""
        if not value:
            return None
        dot = value.find(".")
        deg_len = dot - 2
        degree = float(value[:deg_len])
        minute = float(value[deg_len:])
        result = degree + minute / 60.0
        if hemi in ("S", "W"):
            result = -result
        return round(result, 6)

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "gps", "data": data, "error": error}

