"""UART driver for MS60-3015S80M4-BSD / AT6010 BSD radar."""


try:
    import time
except ImportError:
    time = None


class MS60BSDRadar:
    """Parse AT6010 UART reports and expose BSD radar targets."""

    SEND_HEAD = 0x58
    RESP_HEAD = 0x59
    REPORT_HEAD = 0x5A

    TYPE_FULL = 0
    TYPE_MOTION = 3
    TYPE_REGION = 5
    TYPE_BSD = 7

    CMD_GET_DETECTION = 0x30
    CMD_GET_RADAR_STATE = 0xD0
    CMD_SET_RADAR_STATE = 0xD1
    CMD_SET_BAUDRATE = 0x19

    DET_FLAGS = (
        (0x01, "approaching"),
        (0x02, "leaving"),
        (0x04, "motion"),
        (0x08, "micro_motion"),
        (0x10, "presence"),
    )

    def __init__(self, uart=None, request_on_read=False, max_buffer=256):
        self.uart = uart
        self.request_on_read = request_on_read
        self.max_buffer = max_buffer
        self._available = uart is not None
        self._buffer = bytearray()
        self.last_response = None
        self.last_data = self._empty_data()

    def init(self):
        """Return whether the UART object is available."""
        return self._available

    def check(self):
        """Return whether the driver can access UART."""
        return self._available

    def read(self):
        """Read UART bytes, parse all complete frames and return latest data."""
        if not self._available:
            return self._result(False, None, "radar_not_available")
        try:
            if self.request_on_read:
                self.request_detection()
            raw = self._read_uart()
            if raw:
                self.feed(raw)
            return self._result(True, self.last_data, None)
        except Exception as exc:
            print("[MS60BSDRadar] read failed:", exc)
            return self._result(False, None, "radar_read_failed")

    def feed(self, raw):
        """Feed bytes into the parser. Useful for tests or external readers."""
        if raw is None:
            return []
        if isinstance(raw, str):
            raw = raw.encode()
        self._buffer.extend(raw)
        if len(self._buffer) > self.max_buffer:
            del self._buffer[: len(self._buffer) - self.max_buffer]
        return self._parse_buffer()

    def request_detection(self):
        """Ask the radar for the current detection information."""
        return self.send_command(self.CMD_GET_DETECTION)

    def get_radar_state(self):
        """Ask whether radar detection is enabled."""
        return self.send_command(self.CMD_GET_RADAR_STATE)

    def set_radar_state(self, enabled=True):
        """Enable or disable radar detection."""
        return self.send_command(self.CMD_SET_RADAR_STATE, [1 if enabled else 0])

    def set_baudrate(self, baudrate):
        """Send baudrate switch command. Reconfigure local UART separately."""
        value = int(baudrate)
        params = [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]
        return self.send_command(self.CMD_SET_BAUDRATE, params)

    def send_command(self, cmd, params=None):
        """Build and write an AT6010 control frame."""
        if not self._available or not hasattr(self.uart, "write"):
            return False
        frame = self.build_command(cmd, params)
        self.uart.write(frame)
        return True

    @classmethod
    def build_command(cls, cmd, params=None):
        """Return a 0x58 command frame with 16-bit little-endian checksum."""
        params = list(params or [])
        body = [cls.SEND_HEAD, cmd & 0xFF, len(params) & 0xFF] + [p & 0xFF for p in params]
        check = sum(body) & 0xFFFF
        body.append(check & 0xFF)
        body.append((check >> 8) & 0xFF)
        return bytes(body)

    @classmethod
    def build_report(cls, report_type, payload):
        """Build a 0x5A active report frame. Intended for tests."""
        data = [report_type & 0xFF] + [item & 0xFF for item in payload]
        body = [cls.REPORT_HEAD, len(data) & 0xFF] + data
        body.append(sum(body) & 0xFF)
        return bytes(body)

    def _read_uart(self):
        if hasattr(self.uart, "any"):
            count = self.uart.any()
            if not count:
                return b""
            return self.uart.read(count)
        return self.uart.read()

    def _parse_buffer(self):
        parsed = []
        while len(self._buffer) >= 3:
            pos = self._find_next_head()
            if pos < 0:
                self._buffer[:] = b""
                break
            if pos:
                del self._buffer[:pos]
            head = self._buffer[0]
            if head == self.REPORT_HEAD:
                item = self._parse_report_frame()
            elif head == self.RESP_HEAD:
                item = self._parse_response_frame()
            else:
                del self._buffer[0]
                continue
            if item is None:
                break
            parsed.append(item)
        return parsed

    def _parse_report_frame(self):
        length = self._buffer[1]
        total = 3 + length
        if len(self._buffer) < total:
            return None
        frame = bytes(self._buffer[:total])
        del self._buffer[:total]
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            return {"type": "bad_report_checksum", "frame": frame}
        payload = frame[2:-1]
        data = self.parse_report_payload(payload)
        if data is not None:
            self.last_data = data
        return {"type": "report", "data": data}

    def _parse_response_frame(self):
        length = self._buffer[2]
        total = 5 + length
        if len(self._buffer) < total:
            return None
        frame = bytes(self._buffer[:total])
        del self._buffer[:total]
        check = self._u16(frame, total - 2)
        if (sum(frame[:-2]) & 0xFFFF) != check:
            return {"type": "bad_response_checksum", "frame": frame}
        self.last_response = {
            "cmd": frame[1],
            "params": frame[3:-2],
            "ok": not frame[3:-2] or frame[3] == 0,
        }
        return {"type": "response", "data": self.last_response}

    def parse_report_payload(self, payload):
        """Parse active report payload where byte 0 is report type."""
        if not payload:
            return None
        report_type = payload[0]
        body = payload[1:]
        if report_type == self.TYPE_BSD:
            return self._parse_bsd(body)
        if report_type == self.TYPE_FULL:
            return self._parse_full(body)
        if report_type == self.TYPE_MOTION:
            return self._parse_motion(body)
        if report_type == self.TYPE_REGION:
            return self._parse_region(body)
        return {
            "detected": False,
            "type": report_type,
            "type_name": "unknown",
            "raw": bytes(body),
            "updated_ms": self._ticks_ms(),
        }

    def _parse_bsd(self, body):
        if len(body) < 4:
            return self._empty_data("bsd", error="bsd_payload_short")
        obj_num = min(self._u16(body, 0), 8)
        available = max(0, (len(body) - 4) // 4)
        count = min(obj_num, available)
        objects = []
        for index in range(count):
            offset = 4 + index * 4
            obj = {
                "range_m": self._s8(body[offset]),
                "angle_deg": self._s8(body[offset + 1]),
                "velocity_mps": self._s8(body[offset + 2]),
                "id": self._s8(body[offset + 3]),
            }
            objects.append(obj)
        nearest = self._nearest_object(objects)
        data = {
            "detected": count > 0,
            "type": self.TYPE_BSD,
            "type_name": "bsd",
            "obj_num": count,
            "objects": objects,
            "nearest": nearest,
            "nearest_m": nearest.get("range_m") if nearest else None,
            "nearest_cm": nearest.get("range_m") * 100 if nearest else None,
            "front_cm": nearest.get("range_m") * 100 if nearest else None,
            "updated_ms": self._ticks_ms(),
        }
        return data

    def _parse_full(self, body):
        if len(body) < 20:
            return self._empty_data("full", error="full_payload_short")
        data = self._parse_common_detection(body)
        data["type"] = self.TYPE_FULL
        data["type_name"] = "full"
        data["range_conf"] = body[14]
        data["angle_conf"] = body[15]
        data["frame_idx"] = self._u32(body, 16)
        return data

    def _parse_motion(self, body):
        if len(body) < 8:
            return self._empty_data("motion", error="motion_payload_short")
        data = self._parse_common_detection(body)
        data["type"] = self.TYPE_MOTION
        data["type_name"] = "motion"
        return data

    def _parse_region(self, body):
        if len(body) < 4:
            return self._empty_data("region", error="region_payload_short")
        obj_num = min(self._u32(body, 0), 3)
        available = max(0, (len(body) - 4) // 4)
        count = min(obj_num, available)
        objects = []
        for index in range(count):
            offset = 4 + index * 4
            range_mm = self._u16(body, offset)
            angle = self._s16(body, offset + 2)
            objects.append({"range_mm": range_mm, "range_cm": range_mm / 10.0, "angle_deg": angle})
        nearest = self._nearest_object(objects, "range_cm")
        return {
            "detected": count > 0,
            "type": self.TYPE_REGION,
            "type_name": "region",
            "obj_num": count,
            "objects": objects,
            "nearest": nearest,
            "nearest_cm": nearest.get("range_cm") if nearest else None,
            "front_cm": nearest.get("range_cm") if nearest else None,
            "updated_ms": self._ticks_ms(),
        }

    def _parse_common_detection(self, body):
        range_mm = self._u16(body, 2)
        angle = self._s16(body, 4)
        velocity = self._s16(body, 6)
        return {
            "detected": body[0] != 0,
            "det_result": body[1],
            "det_flags": self._flags(body[1]),
            "range_mm": range_mm,
            "range_cm": range_mm / 10.0,
            "front_cm": range_mm / 10.0,
            "angle_deg": angle,
            "velocity": velocity,
            "updated_ms": self._ticks_ms(),
        }

    def _find_next_head(self):
        report = self._buffer.find(bytes([self.REPORT_HEAD]))
        response = self._buffer.find(bytes([self.RESP_HEAD]))
        if report < 0:
            return response
        if response < 0:
            return report
        return report if report < response else response

    def _nearest_object(self, objects, key="range_m"):
        nearest = None
        for obj in objects:
            value = obj.get(key)
            if value is None or value < 0:
                continue
            if nearest is None or value < nearest.get(key):
                nearest = obj
        return nearest

    def _empty_data(self, type_name="bsd", error=None):
        data = {
            "detected": False,
            "type_name": type_name,
            "obj_num": 0,
            "objects": [],
            "nearest": None,
            "nearest_m": None,
            "nearest_cm": None,
            "front_cm": None,
            "updated_ms": self._ticks_ms(),
        }
        if error:
            data["error"] = error
        return data

    def _flags(self, value):
        return [name for bit, name in self.DET_FLAGS if value & bit]

    def _result(self, ok, data, error):
        return {"ok": ok, "sensor": "radar", "data": data, "error": error}

    def _ticks_ms(self):
        if time and hasattr(time, "ticks_ms"):
            return time.ticks_ms()
        if time:
            return int(time.time() * 1000)
        return 0

    @staticmethod
    def _u16(data, offset):
        return data[offset] | (data[offset + 1] << 8)

    @staticmethod
    def _u32(data, offset):
        return (
            data[offset]
            | (data[offset + 1] << 8)
            | (data[offset + 2] << 16)
            | (data[offset + 3] << 24)
        )

    @classmethod
    def _s16(cls, data, offset):
        value = cls._u16(data, offset)
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _s8(value):
        value &= 0xFF
        return value - 0x100 if value & 0x80 else value
