"""高温中暑预警算法。"""

from config import HEAT_LIGHT_C, HEAT_MEDIUM_C, HEAT_SEVERE_C
from core.warning_state import HEAT_WARNING_LIGHT, HEAT_WARNING_MEDIUM, HEAT_WARNING_SEVERE


class HeatDetector:
    """根据环境温度输出高温预警。"""

    def update(self, env_data):
        """输入环境数据，返回预警结果。"""
        if not env_data or env_data.get("temperature") is None:
            return {"triggered": False}
        temp = env_data["temperature"]
        if temp >= HEAT_SEVERE_C:
            return self._warning(HEAT_WARNING_SEVERE, "severe", "severe_heat_warning", temp)
        if temp >= HEAT_MEDIUM_C:
            return self._warning(HEAT_WARNING_MEDIUM, "medium", "medium_heat_warning", temp)
        if temp >= HEAT_LIGHT_C:
            return self._warning(HEAT_WARNING_LIGHT, "light", "light_heat_warning", temp)
        return {"triggered": False}

    def _warning(self, code, level, message, temp):
        return {
            "triggered": True,
            "code": code,
            "level": level,
            "category": "heat",
            "message": message,
            "need_sos": False,
            "raw": {"temperature": temp},
        }

