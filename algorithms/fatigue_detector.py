"""疲劳检测接口占位。"""


class FatigueDetector:
    """后续接入疲劳检测模型时保持 update 接口稳定。"""

    def update(self, sensor_data):
        """当前不触发疲劳预警。"""
        return {"triggered": False}

