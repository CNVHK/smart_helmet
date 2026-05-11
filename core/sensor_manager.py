"""统一传感器管理。"""

from core.data_filter import SensorFilterBank


class SensorManager:
    """初始化、读取和记录所有传感器状态。"""

    def __init__(self, sensors=None):
        """传入 name->sensor 对象字典。"""
        self.sensors = sensors or {}
        self.status = {}
        self.filters = SensorFilterBank(size=3)

    def init_all(self):
        """初始化所有传感器，单个失败不影响其他设备。"""
        for name, sensor in self.sensors.items():
            try:
                ok = sensor.init() if hasattr(sensor, "init") else True
                self.status[name] = bool(ok)
            except Exception as exc:
                print("[SensorManager] init failed:", name, exc)
                self.status[name] = False
        return self.status

    def read_all(self):
        """周期读取所有传感器，返回 name->data 的统一结构。"""
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
        """返回各传感器在线状态。"""
        return self.status

