"""碰撞检测算法。"""

import math
from config import COLLISION_LIGHT_G, COLLISION_MEDIUM_G, COLLISION_SEVERE_G, COLLISION_JERK_G_S
from core.warning_state import COLLISION_LIGHT, COLLISION_MEDIUM, COLLISION_SEVERE, COLLISION_SUSPECTED


class CollisionDetector:
    """基于加速度峰值和 jerk 的简化碰撞检测。"""

    def __init__(self):
        """初始化上一帧加速度。"""
        self.last_acc_g = None

    def update(self, imu_data, dt_s=0.1):
        """输入 IMU 数据，输出预警结果。"""
        if not imu_data:
            return self._normal()
        ax = imu_data.get("ax")
        ay = imu_data.get("ay")
        az = imu_data.get("az")
        if ax is None or ay is None or az is None:
            return self._normal()
        acc_g = math.sqrt(ax * ax + ay * ay + az * az)
        jerk = 0.0
        if self.last_acc_g is not None and dt_s > 0:
            jerk = abs(acc_g - self.last_acc_g) / dt_s
        self.last_acc_g = acc_g

        if acc_g >= COLLISION_SEVERE_G:
            return self._warning(COLLISION_SEVERE, "severe", "severe_collision_detected", True, acc_g, jerk)
        if acc_g >= COLLISION_MEDIUM_G:
            return self._warning(COLLISION_MEDIUM, "medium", "medium_collision_detected", False, acc_g, jerk)
        if acc_g >= COLLISION_LIGHT_G:
            return self._warning(COLLISION_LIGHT, "light", "light_collision_detected", False, acc_g, jerk)
        if jerk >= COLLISION_JERK_G_S:
            return self._warning(COLLISION_SUSPECTED, "suspected", "collision_suspected", False, acc_g, jerk)
        return self._normal()

    def _warning(self, code, level, message, need_sos, acc_g, jerk):
        return {
            "triggered": True,
            "code": code,
            "level": level,
            "category": "collision",
            "message": message,
            "need_sos": need_sos,
            "raw": {"acc_peak_g": round(acc_g, 2), "gyro_peak_dps": None, "jerk_peak_g_s": round(jerk, 2)},
        }

    def _normal(self):
        return {"triggered": False}
