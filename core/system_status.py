"""系统状态管理。"""


class SystemStatus:
    """维护电量、网络和运行状态。"""

    def __init__(self):
        """初始化默认状态。"""
        self.battery = None
        self.signal = None
        self.work_time = 0
        self.helmet_on = 1
        self.network = "offline"
        self.status = "normal"

    def set_network(self, network):
        """更新网络状态。"""
        self.network = network

    def set_status(self, status):
        """更新系统运行状态。"""
        self.status = status

    def to_dict(self):
        """导出 JSON 友好的系统状态。"""
        return {
            "battery": self.battery,
            "signal": self.signal,
            "work_time": self.work_time,
            "helmet_on": self.helmet_on,
            "network": self.network,
            "status": self.status,
        }
