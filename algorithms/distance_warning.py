"""距离预警算法。"""

from config import DISTANCE_LIGHT_CM, DISTANCE_MEDIUM_CM, DISTANCE_SEVERE_CM
from core.warning_state import DISTANCE_WARNING_LIGHT, DISTANCE_WARNING_MEDIUM, DISTANCE_WARNING_SEVERE


class DistanceWarning:
    """根据前方距离输出接近预警。"""

    def update(self, distance_data):
        """输入距离数据，返回预警结果。"""
        if not distance_data or distance_data.get("front_cm") is None:
            return {"triggered": False}
        cm = distance_data["front_cm"]
        if cm <= DISTANCE_SEVERE_CM:
            return self._warning(DISTANCE_WARNING_SEVERE, "severe", "severe_distance_warning", cm)
        if cm <= DISTANCE_MEDIUM_CM:
            return self._warning(DISTANCE_WARNING_MEDIUM, "medium", "medium_distance_warning", cm)
        if cm <= DISTANCE_LIGHT_CM:
            return self._warning(DISTANCE_WARNING_LIGHT, "light", "light_distance_warning", cm)
        return {"triggered": False}

    def _warning(self, code, level, message, cm):
        return {
            "triggered": True,
            "code": code,
            "level": level,
            "category": "distance",
            "message": message,
            "need_sos": False,
            "raw": {"front_distance_cm": cm},
        }
