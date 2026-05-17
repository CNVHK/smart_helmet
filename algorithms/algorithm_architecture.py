"""
智能安全头盔算法总架构。

对应 4.30 安排：
1. 危险骑行状态预警算法：事故前预警
2. 碰撞识别算法：事故发生判断
3. 事故分级与响应算法：事故后处理
4. 疲劳识别算法：长时间骑行/作业风险
5. 高温中暑风险识别算法：环境与暴露风险
6. 综合安全评分算法：APP 首页和管理端展示

该文件负责算法编排，不替代 collision_detector.py 中已经完成的碰撞识别核心逻辑。
主程序只需要按 update() 的入参传入当前一帧数据，即可拿到统一结果。
"""

import math
import time
from collections import deque

try:
    from .collision_detector import HelmetCollisionDetector
except ImportError:
    from collision_detector import HelmetCollisionDetector


def _clamp(value, low=0, high=100):
    return max(low, min(high, value))


def _finite_or_default(value, default=0.0):
    """
    将传感器输入归一成可计算的浮点数。

    算法层不假设主控永远传入干净数据。这里统一过滤 None、NaN 和无穷值，
    避免单帧坏数据让 sqrt、atan2 或评分逻辑崩溃。
    """

    if value is None:
        return default
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _round_or_none(value, digits=2):
    if value is None:
        return None
    return round(value, digits)


def _make_deque(maxlen=None, iterable=()):
    """
    Create a deque on both CPython and MicroPython-style runtimes.

    MicroPython requires deque(iterable, maxlen), while CPython also supports
    unbounded deque(). This project runs on embedded targets, so keep all
    runtime windows bounded.
    """

    if maxlen is None:
        return deque(iterable)
    try:
        return deque(iterable, maxlen=maxlen)
    except TypeError:
        return deque(iterable, maxlen)


class RuntimeFeatureTracker:
    """
    从原始 IMU/GNSS 数据中提取跨模块共用的运行特征。

    4.27 会议要求算法侧提供函数给主程序调用，主程序只负责传入传感器数据。
    因此这里把疲劳和综合评分所需的派生特征也放在算法内部计算：
    - 头部低头持续时间
    - 1 分钟点头次数
    - 头部姿态稳定性
    - GNSS 速度波动
    - 粗略运动强度
    """

    def __init__(self, sample_rate=100, history_seconds=12):
        self.sample_rate = sample_rate
        self.maxlen = max(1, int(sample_rate * history_seconds))
        self.history = _make_deque(self.maxlen)
        self.low_head_seconds = 0.0
        self.last_low_head_time = None
        self.last_pitch = None
        self.last_pitch_trend = 0
        self.nod_count_1min = 0
        self.nod_events = _make_deque(max(10, int(sample_rate * 60)))

    def update(self, ax, ay, az, gx, gy, gz, gps_speed=None, timestamp=None):
        # 逻辑骨架：
        # 1. 清洗当前帧 IMU/GNSS 输入
        # 2. 由加速度估算俯仰角，并统计低头持续时间
        # 3. 根据俯仰角趋势变化统计 1 分钟点头次数
        # 4. 将当前帧写入 60 秒历史窗口
        # 5. 从最近 10 秒窗口计算速度波动、头部稳定性和运动强度
        # 6. 输出供预警、疲劳、高温和综合评分复用的运行特征
        ax = _finite_or_default(ax)
        ay = _finite_or_default(ay)
        az = _finite_or_default(az, 1.0)
        gx = _finite_or_default(gx)
        gy = _finite_or_default(gy)
        gz = _finite_or_default(gz)
        gps_speed = None if gps_speed is None else max(0.0, _finite_or_default(gps_speed))

        now = float(time.time() if timestamp is None else timestamp)
        pitch = self._estimate_pitch(ax, ay, az)
        gyro_norm = math.sqrt(gx ** 2 + gy ** 2 + gz ** 2)
        acc_norm = math.sqrt(ax ** 2 + ay ** 2 + az ** 2)

        low_head = abs(pitch) >= 35.0
        if low_head:
            if self.last_low_head_time is None:
                self.last_low_head_time = now
            self.low_head_seconds = now - self.last_low_head_time
        else:
            self.last_low_head_time = None
            self.low_head_seconds = 0.0

        self._update_nod_count(now, pitch)

        self.history.append((now, pitch, gyro_norm, acc_norm, gps_speed))

        stats = self._recent_stats(seconds=10, now=now)
        speed_variation = stats["speed_range"]
        avg_gyro = stats["avg_gyro"]
        head_stability = _clamp(1.0 - avg_gyro / 120.0, 0.0, 1.0)
        motion_intensity = _clamp((stats["acc_range"] / 2.0) + (avg_gyro / 240.0), 0.0, 1.0)

        return {
            "pitch_deg": round(pitch, 1),
            "low_head_seconds": round(self.low_head_seconds, 1),
            "nod_count_1min": self.nod_count_1min,
            "head_stability": round(head_stability, 2),
            "speed_variation": round(speed_variation, 2),
            "motion_intensity": round(motion_intensity, 2),
        }

    def _estimate_pitch(self, ax, ay, az):
        denominator = math.sqrt(ay ** 2 + az ** 2)
        return math.degrees(math.atan2(ax, denominator))

    def _update_nod_count(self, now, pitch):
        if self.last_pitch is None:
            self.last_pitch = pitch
            return

        diff = pitch - self.last_pitch
        trend = 1 if diff > 4.0 else -1 if diff < -4.0 else 0
        if self.last_pitch_trend == 1 and trend == -1 and abs(pitch) >= 18.0:
            self.nod_events.append(now)

        self.last_pitch_trend = trend or self.last_pitch_trend
        self.last_pitch = pitch

        while self.nod_events and now - self.nod_events[0] > 60.0:
            self.nod_events.popleft()
        self.nod_count_1min = len(self.nod_events)

    def _recent_stats(self, seconds, now):
        cutoff = now - seconds
        count = 0
        gyro_sum = 0.0
        acc_min = None
        acc_max = None
        speed_min = None
        speed_max = None

        for item in self.history:
            item_time, _pitch, gyro_norm, acc_norm, gps_speed = item
            if item_time < cutoff:
                continue
            count += 1
            gyro_sum += gyro_norm
            if acc_min is None or acc_norm < acc_min:
                acc_min = acc_norm
            if acc_max is None or acc_norm > acc_max:
                acc_max = acc_norm
            if gps_speed is not None:
                if speed_min is None or gps_speed < speed_min:
                    speed_min = gps_speed
                if speed_max is None or gps_speed > speed_max:
                    speed_max = gps_speed

        return {
            "avg_gyro": gyro_sum / count if count else 0.0,
            "acc_range": (acc_max - acc_min) if acc_min is not None and acc_max is not None else 0.0,
            "speed_range": (speed_max - speed_min) if speed_min is not None and speed_max is not None else 0.0,
        }

    def _recent_history(self, seconds):
        """返回最近 seconds 秒的历史窗口，避免多处重复切片和边界计算。"""

        count = min(len(self.history), max(1, int(self.sample_rate * seconds)))
        if count <= 0:
            return []
        result = []
        start = len(self.history) - count
        index = 0
        for item in self.history:
            if index >= start:
                result.append(item)
            index += 1
        return result

    def _range(self, values):
        if not values:
            return 0.0
        return max(values) - min(values)


class RidingRiskPreWarning:
    """
    事故前危险骑行状态预警。

    关注速度突降、头部剧烈晃动、角速度异常、急转弯/突然偏航等风险。
    输出 risk_pre_score、risk_pre_level、risk_pre_alert，用于事故前提醒。
    """

    def update(self, feature):
        # 逻辑骨架：
        # 1. 读取碰撞模块已经计算好的窗口峰值与运动状态
        # 2. 按速度突降、角速度、jerk、加速度分别累加风险分
        # 3. 将总分裁剪到 0~100
        # 4. 按分数映射预警等级、告警值和建议动作
        # 5. 返回可解释原因，方便 APP 展示和后续调参
        acc_peak = feature.get("acc_peak", 0.0)
        gyro_peak = feature.get("gyro_peak", 0.0)
        jerk_peak = feature.get("jerk_peak", 0.0)
        speed_drop = feature.get("speed_drop", 0.0)
        moving_state = feature.get("moving_state", "unknown")

        score = 0
        reasons = []

        if moving_state in ("moving", "high_speed"):
            score += 10

        if moving_state == "high_speed":
            score += 15
            reasons.append("high_speed")

        if speed_drop >= 3.0:
            score += 35
            reasons.append("speed_drop")
        elif speed_drop >= 1.5:
            score += 18
            reasons.append("speed_change")

        if gyro_peak >= 350:
            score += 30
            reasons.append("gyro_abnormal")
        elif gyro_peak >= 220:
            score += 18
            reasons.append("head_shake")

        if jerk_peak >= 30:
            score += 25
            reasons.append("impact_trend")
        elif jerk_peak >= 18:
            score += 12
            reasons.append("jerk_rise")

        if acc_peak >= 3.5:
            score += 20
            reasons.append("strong_acc")
        elif acc_peak >= 2.2:
            score += 10
            reasons.append("acc_fluctuation")

        score = _clamp(score)

        if score >= 70:
            level = "high"
            alert = 3
            action = "continuous_warning"
        elif score >= 40:
            level = "medium"
            alert = 2
            action = "beep_and_app_reminder"
        elif score >= 20:
            level = "low"
            alert = 1
            action = "record"
        else:
            level = "normal"
            alert = 0
            action = "record"

        return {
            "risk_pre_score": score,
            "risk_pre_level": level,
            "risk_pre_alert": alert,
            "risk_pre_action": action,
            "risk_pre_reasons": reasons or ["normal"],
        }


class FatigueDetector:
    """
    疲劳识别初版。

    当前支持 IMU 姿态特征和可选摄像头特征。没有摄像头数据时，也能通过连续使用时长、
    频繁点头、长时间低头和头部稳定性下降给出基础评分。
    """

    def update(
        self,
        usage_minutes=0,
        nod_count_1min=None,
        low_head_seconds=None,
        head_stability=None,
        eye_closed_seconds=None,
        blink_rate=None,
        night=False,
        speed_variation=None,
    ):
        # 逻辑骨架：
        # 1. 融合使用时长、夜间状态、IMU 姿态和可选摄像头特征
        # 2. 对点头、长时间低头、头部不稳、闭眼/眨眼异常分别加权
        # 3. 使用速度波动作为辅助项，避免单一疲劳信号误判
        # 4. 根据总分输出 normal/mild/warning/severe
        # 5. 保留 fatigue_reasons，支持端侧和服务端复盘
        score = 0
        reasons = []

        if usage_minutes >= 180:
            score += 25
            reasons.append("long_usage")
        elif usage_minutes >= 90:
            score += 12
            reasons.append("usage_rise")

        if night:
            score += 12
            reasons.append("night")

        nod_count_1min = nod_count_1min or 0
        low_head_seconds = low_head_seconds or 0
        head_stability = 1.0 if head_stability is None else head_stability
        speed_variation = speed_variation or 0.0

        if nod_count_1min >= 8:
            score += 28
            reasons.append("frequent_nod")
        elif nod_count_1min >= 4:
            score += 15
            reasons.append("nod")

        if low_head_seconds >= 20:
            score += 25
            reasons.append("long_low_head")
        elif low_head_seconds >= 8:
            score += 12
            reasons.append("low_head")

        if head_stability < 0.45:
            score += 20
            reasons.append("unstable_head")
        elif head_stability < 0.70:
            score += 10
            reasons.append("stability_down")

        if eye_closed_seconds is not None:
            if eye_closed_seconds >= 2.0:
                score += 35
                reasons.append("eye_closed")
            elif eye_closed_seconds >= 0.8:
                score += 18
                reasons.append("blink_slow")

        if blink_rate is not None and (blink_rate < 6 or blink_rate > 30):
            score += 10
            reasons.append("blink_abnormal")

        if speed_variation >= 2.5:
            score += 8
            reasons.append("speed_unstable")

        score = _clamp(score)

        if score >= 75:
            level = "severe"
            alert = 3
            action = "continuous_alarm"
        elif score >= 50:
            level = "warning"
            alert = 2
            action = "app_popup"
        elif score >= 25:
            level = "mild"
            alert = 1
            action = "local_reminder"
        else:
            level = "normal"
            alert = 0
            action = "none"

        return {
            "fatigue_score": score,
            "fatigue_level": level,
            "fatigue_alert": alert,
            "fatigue_action": action,
            "fatigue_reasons": reasons or ["normal"],
        }


class HeatRiskDetector:
    """
    高温中暑风险识别初版。

    优先使用体温；没有体温传感器时，使用环境温度、湿度、暴露/工作时长和运动强度估算。
    """

    def update(
        self,
        body_temp=None,
        env_temp=None,
        humidity=None,
        exposure_minutes=0,
        work_minutes=0,
        motion_intensity=0.0,
        strong_sun=False,
    ):
        # 逻辑骨架：
        # 1. 优先使用体温判断人体真实热负荷
        # 2. 体温缺失时，使用环境温度、湿度、暴露时长和运动强度估算风险
        # 3. 强日照作为叠加风险项，不单独触发高等级告警
        # 4. 总分裁剪后映射 safe/attention/warning/danger
        # 5. 输出建议动作：补水、休息降温或停止作业并通知
        score = 0
        reasons = []

        if body_temp is not None:
            if body_temp >= 39.0:
                score += 45
                reasons.append("body_temp_danger")
            elif body_temp >= 38.0:
                score += 30
                reasons.append("body_temp_warning")
            elif body_temp >= 37.3:
                score += 12
                reasons.append("body_temp_rise")

        if env_temp is not None:
            if env_temp >= 38:
                score += 35
                reasons.append("env_temp_danger")
            elif env_temp >= 33:
                score += 24
                reasons.append("env_temp_high")
            elif env_temp >= 30:
                score += 12
                reasons.append("env_temp_rise")

        if humidity is not None:
            if humidity >= 80:
                score += 16
                reasons.append("humidity_high")
            elif humidity >= 65:
                score += 8
                reasons.append("humidity_rise")

        if exposure_minutes >= 180 or work_minutes >= 180:
            score += 22
            reasons.append("long_exposure")
        elif exposure_minutes >= 60 or work_minutes >= 60:
            score += 10
            reasons.append("exposure_rise")

        if motion_intensity >= 0.75:
            score += 15
            reasons.append("high_intensity")
        elif motion_intensity >= 0.45:
            score += 8
            reasons.append("motion_intensity")

        if strong_sun:
            score += 10
            reasons.append("strong_sun")

        score = _clamp(score)

        if score >= 75:
            level = "danger"
            alert = 3
            action = "stop_work_and_notify"
        elif score >= 50:
            level = "warning"
            alert = 2
            action = "rest_and_cool_down"
        elif score >= 25:
            level = "attention"
            alert = 1
            action = "drink_water"
        else:
            level = "safe"
            alert = 0
            action = "none"

        return {
            "heat_score": score,
            "heat_level": level,
            "heat_alert": alert,
            "heat_action": action,
            "heat_reasons": reasons or ["safe"],
        }


class VitalSignRiskDetector:
    """Lightweight risk scoring for the currently available vital sensors."""

    def update(self, body_temp=None, heart_rate=None, spo2=None):
        score = 0
        reasons = []
        body_temp_alert = 0
        hr_alert = 0
        spo2_alert = 0

        if body_temp is not None:
            if body_temp >= 39.0:
                body_temp_alert = 3
                score += 35
                reasons.append("body_temp_danger")
            elif body_temp >= 38.0:
                body_temp_alert = 2
                score += 22
                reasons.append("body_temp_warning")
            elif body_temp >= 37.3:
                body_temp_alert = 1
                score += 10
                reasons.append("body_temp_rise")

        if heart_rate is not None:
            if heart_rate < 45 or heart_rate > 130:
                hr_alert = 3
                score += 30
                reasons.append("heart_rate_danger")
            elif heart_rate < 55 or heart_rate > 110:
                hr_alert = 2
                score += 18
                reasons.append("heart_rate_warning")
            elif heart_rate > 100:
                hr_alert = 1
                score += 8
                reasons.append("heart_rate_rise")

        if spo2 is not None:
            if spo2 < 90:
                spo2_alert = 3
                score += 35
                reasons.append("spo2_danger")
            elif spo2 < 94:
                spo2_alert = 2
                score += 22
                reasons.append("spo2_warning")
            elif spo2 < 96:
                spo2_alert = 1
                score += 10
                reasons.append("spo2_attention")

        score = _clamp(score)
        max_alert = max(body_temp_alert, hr_alert, spo2_alert)
        if max_alert >= 3:
            level = "danger"
            action = "notify_health_risk"
        elif max_alert == 2:
            level = "warning"
            action = "health_warning"
        elif max_alert == 1:
            level = "attention"
            action = "record_and_remind"
        else:
            level = "safe"
            action = "none"

        return {
            "vital_score": score,
            "vital_level": level,
            "vital_alert": max_alert,
            "body_temp_alert": body_temp_alert,
            "hr_alert": hr_alert,
            "spo2_alert": spo2_alert,
            "vital_action": action,
            "vital_reasons": reasons or ["safe"],
        }


class SafetyScoreEngine:
    """
    综合安全评分。

    分数越高越安全；碰撞/SOS 权重最高，其次是疲劳、高温和事故前风险。
    """

    def update(
        self,
        pre_warning,
        collision,
        fatigue,
        heat,
        vital=None,
        battery_level=None,
        signal_level=None,
        worn=True,
        location_valid=True,
    ):
        # 逻辑骨架：
        # 1. 以 100 分为基础分，碰撞/SOS 最高优先扣分
        # 2. 叠加事故前风险、疲劳、高温、设备电量/信号/佩戴/定位风险
        # 3. 严重碰撞、SOS、重度疲劳、高温危险直接强制 danger
        # 4. 根据剩余分数映射 safe/attention/danger
        # 5. 提取主风险类型和首页摘要文案
        score = 100
        risks = []

        collision_alert = collision.get("collision_alert", 0)
        if collision.get("need_sos"):
            score -= 70
            risks.append("sos")
        elif collision_alert >= 3:
            score -= 55
            risks.append("severe_collision")
        elif collision_alert == 2:
            score -= 35
            risks.append("collision")
        elif collision_alert == 1:
            score -= 15
            risks.append("light_collision")

        score -= pre_warning.get("risk_pre_alert", 0) * 8
        if pre_warning.get("risk_pre_alert", 0) > 0:
            risks.append("riding_risk")

        score -= fatigue.get("fatigue_alert", 0) * 12
        if fatigue.get("fatigue_alert", 0) > 0:
            risks.append("fatigue")

        score -= heat.get("heat_alert", 0) * 12
        if heat.get("heat_alert", 0) > 0:
            risks.append("heat")

        vital = vital or {}
        score -= vital.get("vital_alert", 0) * 10
        if vital.get("vital_alert", 0) > 0:
            risks.append("vital")

        if battery_level is not None and battery_level < 20:
            score -= 8
            risks.append("low_battery")

        if signal_level is not None and signal_level < 2:
            score -= 8
            risks.append("weak_signal")

        if not worn:
            score -= 20
            risks.append("not_worn")

        if not location_valid:
            score -= 12
            risks.append("location_invalid")

        score = _clamp(score)

        force_danger = (
            collision.get("need_sos")
            or collision_alert >= 3
            or fatigue.get("fatigue_alert", 0) >= 3
            or heat.get("heat_alert", 0) >= 3
            or vital.get("vital_alert", 0) >= 3
        )

        if force_danger or score < 50:
            status = "danger"
        elif score < 80:
            status = "attention"
        else:
            status = "safe"

        main_risk_type = risks[0] if risks and status != "safe" else "none"

        if status == "safe":
            summary = "当前状态正常"
        elif main_risk_type == "sos":
            summary = "已触发 SOS"
        elif main_risk_type == "severe_collision":
            summary = "检测到严重碰撞"
        elif main_risk_type == "collision":
            summary = "检测到明显碰撞"
        elif main_risk_type == "fatigue":
            summary = "存在疲劳风险"
        elif main_risk_type == "heat":
            summary = "存在高温风险"
        elif main_risk_type == "riding_risk":
            summary = "骑行状态异常"
        else:
            summary = "存在设备或定位风险"

        return {
            "safety_score": score,
            "risk_status": status,
            "main_risk_type": main_risk_type,
            "risk_summary": summary,
            "risk_items": risks,
        }


class AccidentResponseEngine:
    """
    事故分级与响应策略。

    这里只生成动作建议，不直接控制硬件。主程序可根据 device_actions 控制蜂鸣器、
    语音或灯光，根据 app_actions/cloud_actions 决定上报和展示。
    """

    def update(self, pre_warning, collision, fatigue, heat, safety):
        # 逻辑骨架：
        # 1. 碰撞和 SOS 优先生成事故后响应
        # 2. 若无事故，再处理事故前骑行预警
        # 3. 疲劳和高温可与事故响应叠加，补充设备、APP、云端动作
        # 4. danger 状态统一追加云端危险状态上报
        # 5. 对动作列表去重排序，保证上报内容稳定
        device_actions = []
        app_actions = []
        cloud_actions = []
        response_level = "none"
        sos_alert = 0

        collision_alert = collision.get("collision_alert", 0)
        if collision.get("need_sos") or collision_alert >= 3:
            response_level = "emergency"
            sos_alert = 2
            device_actions.extend(["continuous_beep", "status_light_flash"])
            app_actions.extend(["show_sos_countdown", "show_live_location"])
            cloud_actions.extend(["upload_accident_event", "notify_emergency_contact", "start_location_tracking"])
        elif collision_alert == 2:
            response_level = "accident_confirm"
            device_actions.append("beep_warning")
            app_actions.extend(["show_collision_confirm", "allow_cancel_sos"])
            cloud_actions.append("upload_accident_position")
        elif collision_alert == 1:
            response_level = "local_notice"
            device_actions.append("short_beep")
            app_actions.append("record_light_collision")
            cloud_actions.append("upload_light_event")

        if response_level == "none":
            if pre_warning.get("risk_pre_alert", 0) >= 3:
                response_level = "pre_warning"
                device_actions.append("continuous_beep")
                app_actions.append("remind_slow_down")
            elif pre_warning.get("risk_pre_alert", 0) == 2:
                response_level = "pre_warning"
                device_actions.append("short_beep")
                app_actions.append("show_riding_warning")

        if fatigue.get("fatigue_alert", 0) >= 2:
            device_actions.append("fatigue_beep")
            app_actions.append("show_fatigue_popup")
            if fatigue.get("fatigue_alert", 0) >= 3:
                cloud_actions.append("notify_manager_fatigue")

        if heat.get("heat_alert", 0) >= 2:
            device_actions.append("heat_beep")
            app_actions.append("show_heat_warning")
            if heat.get("heat_alert", 0) >= 3:
                cloud_actions.append("notify_manager_heat")

        if safety.get("risk_status") == "danger" and "upload_danger_status" not in cloud_actions:
            cloud_actions.append("upload_danger_status")

        return {
            "response_level": response_level,
            "sos_alert": sos_alert,
            "device_actions": sorted(set(device_actions)),
            "app_actions": sorted(set(app_actions)),
            "cloud_actions": sorted(set(cloud_actions)),
        }


class SmartHelmetAlgorithm:
    """
    面向主程序的一站式算法入口。

    update() 入参均为可选字段；暂未接入的传感器可以传 None 或不传。
    """

    def __init__(self, sample_rate=100):
        self.runtime_tracker = RuntimeFeatureTracker(sample_rate=sample_rate)
        self.collision_detector = HelmetCollisionDetector(sample_rate=sample_rate)
        self.pre_warning = RidingRiskPreWarning()
        self.fatigue_detector = FatigueDetector()
        self.heat_detector = HeatRiskDetector()
        self.vital_detector = VitalSignRiskDetector()
        self.safety_engine = SafetyScoreEngine()
        self.response_engine = AccidentResponseEngine()

    def update(
        self,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        gps_speed=None,
        timestamp=None,
        body_temp=None,
        heart_rate=None,
        spo2=None,
        env_temp=None,
        humidity=None,
        usage_minutes=0,
        exposure_minutes=0,
        work_minutes=0,
        motion_intensity=None,
        strong_sun=False,
        nod_count_1min=None,
        low_head_seconds=None,
        head_stability=None,
        eye_closed_seconds=None,
        blink_rate=None,
        night=False,
        speed_variation=None,
        battery_level=None,
        signal_level=None,
        worn=True,
        location_valid=True,
    ):
        # 逻辑骨架：
        # 1. 接收主程序当前一帧传感器数据
        # 2. RuntimeFeatureTracker 提取跨模块运行特征
        # 3. HelmetCollisionDetector 完成碰撞/摔倒/SOS 判断
        # 4. 事故前预警、疲劳、高温分别独立评分
        # 5. SafetyScoreEngine 汇总为安全分与主风险
        # 6. AccidentResponseEngine 生成设备、APP、云端响应动作
        # 7. 返回统一结构，供 MQTT/HTTP 上报和 APP 展示
        frame_time = time.time() if timestamp is None else timestamp
        runtime_feature = self.runtime_tracker.update(
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=gz,
            gps_speed=gps_speed,
            timestamp=frame_time,
        )

        collision = self.collision_detector.update(
            ax, ay, az, gx, gy, gz, gps_speed=gps_speed, timestamp=frame_time
        )
        collision["accident_type"] = self._classify_accident_type(collision)

        pre_warning = self.pre_warning.update(collision)

        fatigue = self.fatigue_detector.update(
            usage_minutes=usage_minutes,
            nod_count_1min=runtime_feature["nod_count_1min"] if nod_count_1min is None else nod_count_1min,
            low_head_seconds=runtime_feature["low_head_seconds"] if low_head_seconds is None else low_head_seconds,
            head_stability=runtime_feature["head_stability"] if head_stability is None else head_stability,
            eye_closed_seconds=eye_closed_seconds,
            blink_rate=blink_rate,
            night=night,
            speed_variation=runtime_feature["speed_variation"] if speed_variation is None else speed_variation,
        )

        heat = self.heat_detector.update(
            body_temp=body_temp,
            env_temp=env_temp,
            humidity=humidity,
            exposure_minutes=exposure_minutes,
            work_minutes=work_minutes,
            motion_intensity=runtime_feature["motion_intensity"] if motion_intensity is None else motion_intensity,
            strong_sun=strong_sun,
        )

        vital = self.vital_detector.update(
            body_temp=body_temp,
            heart_rate=heart_rate,
            spo2=spo2,
        )

        safety = self.safety_engine.update(
            pre_warning=pre_warning,
            collision=collision,
            fatigue=fatigue,
            heat=heat,
            vital=vital,
            battery_level=battery_level,
            signal_level=signal_level,
            worn=worn,
            location_valid=location_valid,
        )

        response = self.response_engine.update(
            pre_warning=pre_warning,
            collision=collision,
            fatigue=fatigue,
            heat=heat,
            safety=safety,
        )

        return {
            "version": "1.2",
            "timestamp": int(frame_time),
            "runtime_feature": runtime_feature,
            "pre_warning": pre_warning,
            "collision": collision,
            "fatigue": fatigue,
            "heat": heat,
            "vital": vital,
            "safety": safety,
            "response": response,
        }

    def _classify_accident_type(self, collision):
        if collision.get("collision_alert", 0) == 0:
            return "none"

        moving_state = collision.get("moving_state")
        fall_direction = collision.get("fall_direction")
        speed_drop = collision.get("speed_drop", 0.0)
        acc_peak = collision.get("acc_peak", 0.0)

        if moving_state == "high_speed" and (speed_drop >= 3.0 or acc_peak >= 4.0):
            return "high_speed_collision"
        if fall_direction not in ("upright", "unknown") and moving_state in ("static", "slow"):
            return "low_speed_fall"
        if fall_direction not in ("upright", "unknown"):
            return "road_unbalance_fall"
        return "external_collision"


def build_telemetry_event(device_id, result, gps=None):
    """
    构造统一 MQTT/HTTP 上报报文。

    服务器和 APP 可以直接读取 safety 作为首页状态，读取各子模块作为详情页数据。
    """

    payload = {
        "version": result.get("version", "1.2"),
        "device_id": device_id,
        "msg_type": "telemetry_algorithm",
        "timestamp": result["timestamp"],
        "runtime_feature": result["runtime_feature"],
        "safety": result["safety"],
        "response": result["response"],
        "pre_warning": result["pre_warning"],
        "collision": {
            "collision_alert": result["collision"]["collision_alert"],
            "collision_level": result["collision"]["collision_level"],
            "accident_type": result["collision"]["accident_type"],
            "fall_detected": result["collision"]["fall_detected"],
            "fall_direction": result["collision"]["fall_direction"],
            "need_sos": result["collision"]["need_sos"],
            "reason": result["collision"]["reason"],
        },
        "fatigue": result["fatigue"],
        "heat": result["heat"],
        "vital": result.get("vital"),
    }

    if gps:
        payload["gps"] = gps

    return payload


if __name__ == "__main__":
    algorithm = SmartHelmetAlgorithm(sample_rate=100)
    output = algorithm.update(
        ax=0.01,
        ay=0.02,
        az=1.0,
        gx=0.5,
        gy=0.3,
        gz=0.2,
        gps_speed=2.0,
        env_temp=32,
        humidity=70,
        usage_minutes=100,
        work_minutes=80,
        battery_level=86,
        signal_level=4,
    )
    print(build_telemetry_event("HLM-00123", output))
