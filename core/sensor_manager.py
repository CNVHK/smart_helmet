"""Unified sensor manager."""

from core.data_filter import SensorFilterBank


class SensorManager:
    """Initialize, read and track all sensors."""

    def __init__(self, sensors=None):
        self.sensors = sensors or {}
        self.status = {}
        self.filters = SensorFilterBank(size=3)

    def init_all(self):
        for name, sensor in self.sensors.items():
            try:
                ok = sensor.init() if hasattr(sensor, "init") else True
                self.status[name] = bool(ok)
            except Exception as exc:
                print("[SensorManager] init failed:", name, exc)
                self.status[name] = False
        return self.status

    def read_all(self):
        result = {}
        for name, sensor in self.sensors.items():
            try:
                packet = sensor.read()
                ok = bool(packet.get("ok")) if isinstance(packet, dict) else False
                self.status[name] = ok or self.status.get(name, False)
                data = packet.get("data") if isinstance(packet, dict) else None
                result[name] = self.filters.update_dict(name, data) if ok else None
            except Exception as exc:
                print("[SensorManager] read failed:", name, exc)
                self.status[name] = False
                result[name] = None
        return result

    def get_status(self):
        return self.status
