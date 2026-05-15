"""
智能头盔碰撞检测算法模块
作者：ANNA
版本：V1.4
日期：4.27

功能：
1. 使用 6 轴 IMU 判断碰撞风险
2. 使用 GNSS 速度辅助判断运动状态和速度突降
3. 使用滑动窗口统计短时间峰值
4. 使用连续 N 次超过阈值降低误报
5. 碰撞后进入观察状态，判断是否摔倒静止
6. 根据三轴加速度方向估计摔倒方向
7. 输出适配 MQTT 事件报文的数据结构

单位：
1. IMU 加速度单位：g
2. IMU 角速度单位：deg/s
3. GNSS 速度单位：m/s

collision_alert：
0 = 正常
1 = 轻微震动/小冲击
2 = 明显碰撞
3 = 严重碰撞
"""

import time
import math
from collections import deque


def _finite_or_default(value, default=0.0):
    """
    将外部输入转换为安全浮点数。

    嵌入式传感器偶发通信异常时，可能出现 None、NaN、字符串或无穷值。
    碰撞算法对异常值非常敏感，因此在进入滤波器前先做统一清洗。
    """

    if value is None:
        return default
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _make_deque(maxlen=None, iterable=()):
    """
    Create a deque on both CPython and MicroPython-style runtimes.

    MicroPython requires deque(iterable, maxlen), while CPython accepts the
    maxlen keyword. Keep detector windows bounded for embedded targets.
    """

    if maxlen is None:
        return deque(iterable)
    try:
        return deque(iterable, maxlen=maxlen)
    except TypeError:
        return deque(iterable, maxlen)


class LowPassFilter:
    """
    一阶低通滤波器。

    低通滤波的作用：
    1. 降低 IMU 高频噪声
    2. 避免由于单点毛刺导致误触发
    3. 让加速度、角速度、GNSS 速度变化更平滑

    公式：
        filtered = alpha * new_value + (1 - alpha) * old_value

    alpha 含义：
        alpha 越大：越相信新数据，响应更快，但噪声更明显
        alpha 越小：越相信旧数据，数据更平滑，但响应更慢

    推荐：
        IMU：0.3 ~ 0.4
        GNSS 速度：0.2 ~ 0.4
    """

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        """
        更新滤波器。

        参数：
            new_value: 新输入数据，可以是 float，也可以是 None

        返回：
            滤波后的值

        说明：
            如果 new_value 为 None，说明当前传感器数据无效。
            此时不更新滤波器，直接返回上一次有效值。
        """

        if new_value is None:
            return self.value

        # 第一次输入时，没有历史值，直接使用当前值
        if self.value is None:
            self.value = new_value
        else:
            # 一阶低通滤波核心公式
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value

        return self.value


class HelmetCollisionDetector:
    """
    智能头盔碰撞检测核心类。

    输入：
        ax, ay, az: 三轴加速度，单位 g
        gx, gy, gz: 三轴角速度，单位 deg/s
        gps_speed: GNSS 地面速度，单位 m/s，可选

    输出：
        collision_alert: 碰撞预警等级，0~3
        fall_detected: 是否判断为摔倒，0/1
        fall_direction: 摔倒方向
        need_sos: 是否需要自动 SOS
    """

    def __init__(
        self,
        sample_rate=100,
        window_time=0.5,
        cooldown_time=2.0,
        observe_time=10.0
    ):
        """
        初始化检测器。

        参数：
            sample_rate:
                IMU 采样率，单位 Hz。
                例如 100Hz 表示每秒 100 个数据点。

            window_time:
                滑动窗口长度，单位秒。
                例如 0.5 秒表示保存最近 0.5 秒数据。

            cooldown_time:
                碰撞触发后的冷却时间，单位秒。
                用于避免一次碰撞连续触发多条报警。

            observe_time:
                碰撞后的观察时间，单位秒。
                用于判断碰撞后是否长时间静止，从而触发自动 SOS。
        """

        # 滑动窗口最大长度，例如 100Hz * 0.5s = 50 个采样点
        self.window_size = int(sample_rate * window_time)

        # 使用 deque 作为滑动窗口，超过 maxlen 会自动丢弃最旧数据
        self.window = _make_deque(self.window_size)

        self.cooldown_time = cooldown_time
        self.observe_time = observe_time

        # 六轴 IMU 低通滤波器
        # 加速度滤波器
        self.ax_filter = LowPassFilter(0.35)
        self.ay_filter = LowPassFilter(0.35)
        self.az_filter = LowPassFilter(0.35)

        # 角速度滤波器
        self.gx_filter = LowPassFilter(0.35)
        self.gy_filter = LowPassFilter(0.35)
        self.gz_filter = LowPassFilter(0.35)

        # GNSS 速度滤波器
        # GNSS 速度刷新率通常比 IMU 低，因此也需要平滑
        self.gps_speed_filter = LowPassFilter(0.3)

        # jerk 计算所需的历史加速度模长和历史时间
        # jerk = 加速度变化量 / 时间变化量
        self.last_a_g = None
        self.last_time = None

        # GNSS 速度突降计算所需的历史速度
        self.last_gps_speed = None
        self.last_gps_time = None

        # 冷却控制
        # 当前时间小于 cooldown_until 时，不再触发新的碰撞报警
        self.cooldown_until = 0

        # 碰撞后观察状态机
        # collision_observing = True 表示已经发生碰撞，正在观察后续是否静止
        self.collision_observing = False
        self.collision_start_time = None
        self.max_collision_level = 0

        # 连续计数器
        # 作用：要求某个风险条件连续满足 N 次，降低单点噪声误报
        self.cnt_l1 = 0
        self.cnt_l2 = 0
        self.cnt_l3 = 0

        # 连续触发阈值
        # 按 100Hz 采样估算：
        # N_L1 = 3 约等于 30ms
        # N_L2 = 4 约等于 40ms
        # N_L3 = 2 约等于 20ms
        self.N_L1 = 3
        self.N_L2 = 4
        self.N_L3 = 2

        # gap_allow 表示允许短暂中断多少帧
        # 例如 condition 连续满足时 cnt 增加
        # 如果中间 1 帧不满足，不马上清零，避免阈值边界抖动
        self.gap_allow = 1
        self.gap_l1 = 0
        self.gap_l2 = 0
        self.gap_l3 = 0

        # GNSS 速度状态阈值集中管理，便于后续按赛道/工地/骑行场景调参
        self.speed_thresholds = {
            "static": 0.5,       # 小于 0.5m/s 认为基本静止
            "moving": 1.5,       # 大于 1.5m/s 认为处于运动
            "high_speed": 5.0,   # 大于 5m/s 认为高速移动，约 18km/h
            "drop": 3.0,         # 速度突降阈值，例如 5m/s 降到 1m/s
        }

        # 碰撞分级阈值集中管理，避免魔法数字散落在判定逻辑中
        self.impact_thresholds = {
            "severe_acc": 6.0,
            "severe_acc_with_gyro": 4.5,
            "severe_gyro": 200.0,
            "high_speed_acc": 4.0,
            "moving_acc": 3.2,
            "moving_gyro": 250.0,
            "l3_acc_gyro_acc": 4.5,
            "l3_acc_gyro_gyro": 350.0,
            "l3_acc_jerk_acc": 4.0,
            "l3_acc_jerk_jerk": 35.0,
            "l2_acc": 4.0,
            "l2_gyro": 300.0,
            "l2_acc_jerk_acc": 3.5,
            "l2_acc_jerk_jerk": 25.0,
            "l1_static_acc": 3.0,
            "l1_static_jerk": 18.0,
            "l1_moving_acc": 2.5,
            "l1_moving_jerk": 15.0,
        }

    def update(self, ax, ay, az, gx, gy, gz, gps_speed=None, timestamp=None):
        """
        输入一帧 IMU + 可选 GNSS 速度数据，返回检测结果。

        参数：
            ax, ay, az:
                三轴加速度，单位 g

            gx, gy, gz:
                三轴角速度，单位 deg/s

            gps_speed:
                GNSS 地面速度，单位 m/s
                可以传 None，表示当前没有有效 GNSS 速度

            timestamp:
                可选外部时间戳。主程序传入后，碰撞模块与总控输出使用同一时间基准。

        返回：
            dict，包含碰撞等级、摔倒方向、SOS 状态和关键特征值。
        """

        # 逻辑骨架：
        # 1. 清洗输入，过滤单帧异常值
        # 2. 对 IMU 和 GNSS 速度做一阶低通滤波
        # 3. 计算加速度模长、角速度模长和 jerk
        # 4. 结合 GNSS 速度得到运动状态和速度突降
        # 5. 写入滑动窗口，并统计窗口峰值/均值
        # 6. 按直通规则、GNSS 辅助规则和连续计数规则输出碰撞等级
        # 7. 碰撞后进入观察状态，超时后判断是否静止摔倒并触发 SOS
        # 8. 返回事件上报所需的等级、原因、特征和时间戳
        now = time.time() if timestamp is None else float(timestamp)
        ax = _finite_or_default(ax)
        ay = _finite_or_default(ay)
        az = _finite_or_default(az, 1.0)
        gx = _finite_or_default(gx)
        gy = _finite_or_default(gy)
        gz = _finite_or_default(gz)
        gps_speed = None if gps_speed is None else max(0.0, _finite_or_default(gps_speed))

        # =========================
        # 1. IMU 数据低通滤波
        # =========================
        # 这里使用新的变量 ax_f，避免覆盖原始输入数据
        ax_f = self.ax_filter.update(ax)
        ay_f = self.ay_filter.update(ay)
        az_f = self.az_filter.update(az)

        gx_f = self.gx_filter.update(gx)
        gy_f = self.gy_filter.update(gy)
        gz_f = self.gz_filter.update(gz)

        # 如果滤波器仍然没有有效值，说明传感器数据不可用
        # 直接返回无效数据结果，避免 math.sqrt(None ** 2) 报错
        if None in (ax_f, ay_f, az_f, gx_f, gy_f, gz_f):
            return self._invalid_result(now)

        # =========================
        # 2. 计算 IMU 特征
        # =========================

        # 加速度模长：
        # 正常静止时应该接近 1g
        # 碰撞时通常会出现远大于 1g 的峰值
        a_g = math.sqrt(ax_f ** 2 + ay_f ** 2 + az_f ** 2)

        # 角速度模长：
        # 用于判断是否发生快速旋转、翻滚或撞击后的姿态变化
        gyro = math.sqrt(gx_f ** 2 + gy_f ** 2 + gz_f ** 2)

        # =========================
        # 3. 计算 jerk
        # =========================
        # jerk 是加速度变化率：
        # jerk = |当前加速度模长 - 上一次加速度模长| / dt
        # 碰撞瞬间 jerk 通常会很大
        if self.last_a_g is None or self.last_time is None:
            jerk = 0.0
        else:
            dt = now - self.last_time
            jerk = abs(a_g - self.last_a_g) / dt if dt > 0 else 0.0

        self.last_a_g = a_g
        self.last_time = now

        # =========================
        # 4. GNSS 速度辅助分析
        # =========================
        # GNSS 速度不直接判定碰撞，只用于辅助：
        # 1. 判断当前是静止、低速、运动、高速
        # 2. 判断是否存在速度突降
        gps_speed_f, speed_drop, moving_state = self._process_gps_speed(
            gps_speed=gps_speed,
            now=now
        )

        # =========================
        # 5. 写入滑动窗口
        # =========================
        # 窗口内保存当前帧特征。
        # 后续通过窗口统计峰值、均值，而不是只看单个点。
        self.window.append({
            "a_g": a_g,
            "gyro": gyro,
            "jerk": jerk,

            # 保存滤波后的三轴加速度，用于碰撞后判断摔倒方向
            "ax": ax_f,
            "ay": ay_f,
            "az": az_f
        })

        # =========================
        # 6. 统计窗口特征
        # =========================
        stats = self._get_window_stats()

        # =========================
        # 7. 碰撞分级
        # =========================
        level, name, reason = self._classify(
            stats=stats,
            gps_speed=gps_speed_f,
            speed_drop=speed_drop,
            moving_state=moving_state
        )

        # =========================
        # 8. 冷却机制
        # =========================
        # 如果当前还在冷却时间内，则不触发新的碰撞报警
        if now < self.cooldown_until:
            level = 0
            name = "normal"
            reason = "cooldown"

        # 如果触发碰撞，则更新冷却时间，并进入碰撞后观察状态
        if level > 0:
            self.cooldown_until = now + self.cooldown_time
            self._start_observation(now, level)

        # =========================
        # 9. 碰撞后观察：判断摔倒和 SOS
        # =========================
        # _check_sos() 会在 observe_time 到达后判断：
        # 是否静止、是否非直立、是否需要 SOS
        sos, fall_direction = self._check_sos(now)

        # 如果自动 SOS 成立，直接升级到严重等级
        if sos:
            level = 3
            name = "severe"
            reason = f"fall_detected_{fall_direction}"

        fall_detected = 1 if sos else 0

        # =========================
        # 10. 返回检测结果
        # =========================
        return {
            "collision": level > 0,
            "collision_alert": level,
            "collision_level": name,
            "reason": reason,

            # 当前帧特征
            "acc_g": round(a_g, 2),
            "gyro_dps": round(gyro, 1),
            "jerk": round(jerk, 2),

            # 窗口峰值特征
            "acc_peak": round(stats["acc_peak"], 2),
            "gyro_peak": round(stats["gyro_peak"], 1),
            "jerk_peak": round(stats["jerk_peak"], 2),

            # GNSS 速度辅助信息
            "gps_speed": round(gps_speed_f, 2) if gps_speed_f is not None else None,
            "speed_drop": round(speed_drop, 2),
            "moving_state": moving_state,

            # 摔倒判断结果
            "fall_detected": fall_detected,
            "fall_direction": fall_direction,

            # SOS 状态
            "need_sos": sos,
            "timestamp": int(now)
        }

    def _invalid_result(self, now):
        """
        IMU 数据无效时的统一返回结果。

        例如：
            传感器未初始化
            I2C/SPI/UART 通信失败
            某一轴数据为空
        """

        return {
            "collision": False,
            "collision_alert": 0,
            "collision_level": "normal",
            "reason": "imu_data_invalid",

            "acc_g": 0.0,
            "gyro_dps": 0.0,
            "jerk": 0.0,

            "acc_peak": 0.0,
            "gyro_peak": 0.0,
            "jerk_peak": 0.0,

            "gps_speed": None,
            "speed_drop": 0.0,
            "moving_state": "unknown",

            "fall_detected": 0,
            "fall_direction": "unknown",

            "need_sos": False,
            "timestamp": int(now)
        }

    def _process_gps_speed(self, gps_speed, now):
        """
        处理 GNSS 速度。

        参数：
            gps_speed:
                GNSS 地面速度，单位 m/s

            now:
                当前时间戳

        返回：
            gps_speed_f:
                滤波后的 GNSS 速度

            speed_drop:
                速度突降值。
                只计算“上一时刻速度 - 当前速度”的正值。

            moving_state:
                运动状态：
                unknown    无 GNSS 数据
                static     基本静止
                slow       低速
                moving     正常运动
                high_speed 高速运动

        注意：
            GNSS 通常刷新率低于 IMU，不能用来检测毫秒级碰撞瞬间。
            它只适合做辅助分级和场景判断。
        """

        gps_speed_f = None
        speed_drop = 0.0
        moving_state = "unknown"

        if gps_speed is None:
            return gps_speed_f, speed_drop, moving_state

        gps_speed_f = self.gps_speed_filter.update(gps_speed)

        if gps_speed_f is None:
            return None, 0.0, "unknown"

        # 速度突降：只关心速度下降，不关心速度上升
        if self.last_gps_speed is not None:
            speed_drop = max(0.0, self.last_gps_speed - gps_speed_f)

        self.last_gps_speed = gps_speed_f
        self.last_gps_time = now

        # 根据滤波后的 GNSS 速度判断运动状态
        if gps_speed_f < self.speed_thresholds["static"]:
            moving_state = "static"
        elif gps_speed_f >= self.speed_thresholds["high_speed"]:
            moving_state = "high_speed"
        elif gps_speed_f >= self.speed_thresholds["moving"]:
            moving_state = "moving"
        else:
            moving_state = "slow"

        return gps_speed_f, speed_drop, moving_state

    def _get_window_stats(self):
        """
        从滑动窗口中提取统计特征。

        返回内容：
            acc_peak:
                窗口内最大加速度模长

            gyro_peak:
                窗口内最大角速度模长

            jerk_peak:
                窗口内最大 jerk

            acc_avg:
                窗口内平均加速度模长

            gyro_avg:
                窗口内平均角速度模长

            ax_avg, ay_avg, az_avg:
                窗口内三轴加速度均值。
                用于判断摔倒方向。
        """

        if not self.window:
            return {
                "acc_peak": 0.0,
                "gyro_peak": 0.0,
                "jerk_peak": 0.0,
                "acc_avg": 0.0,
                "gyro_avg": 0.0,
                "ax_avg": 0.0,
                "ay_avg": 0.0,
                "az_avg": 1.0
            }

        acc = [item["a_g"] for item in self.window]
        gyro = [item["gyro"] for item in self.window]
        jerk = [item["jerk"] for item in self.window]

        ax_list = [item["ax"] for item in self.window]
        ay_list = [item["ay"] for item in self.window]
        az_list = [item["az"] for item in self.window]

        return {
            "acc_peak": max(acc),
            "gyro_peak": max(gyro),
            "jerk_peak": max(jerk),
            "acc_avg": sum(acc) / len(acc),
            "gyro_avg": sum(gyro) / len(gyro),

            "ax_avg": sum(ax_list) / len(ax_list),
            "ay_avg": sum(ay_list) / len(ay_list),
            "az_avg": sum(az_list) / len(az_list)
        }

    def _estimate_fall_direction(self, stats):
        """
        根据三轴平均加速度估计摔倒方向。

        原理：
            当头盔静止时，IMU 测到的主要加速度来自重力。
            重力主要落在哪个轴，就说明头盔朝哪个方向倾倒。

        使用前提：
            只有在碰撞后观察阶段，并且头盔基本静止时才可信。
            如果头盔还在运动，方向判断容易不准。

        默认安装方向：
            +X = 头盔前方
            -X = 头盔后方
            +Y = 头盔左侧
            -Y = 头盔右侧
            +Z = 正常佩戴时向上/接近 +1g

        返回：
            upright       正常直立
            forward       前倾/向前摔
            backward      后仰/向后摔
            left          左侧摔倒
            right         右侧摔倒
            upside_down   倒扣/翻转
            unknown       无法判断
        """

        ax = stats["ax_avg"]
        ay = stats["ay_avg"]
        az = stats["az_avg"]

        abs_x = abs(ax)
        abs_y = abs(ay)
        abs_z = abs(az)

        acc_avg = stats["acc_avg"]

        # 如果模长不接近 1g，说明可能还在运动，不适合判断姿态方向
        if not (0.75 <= acc_avg <= 1.30):
            return "unknown"

        # Z 轴占主导：直立或倒扣
        if abs_z >= abs_x and abs_z >= abs_y:
            if az > 0.6:
                return "upright"
            if az < -0.6:
                return "upside_down"

        # X 轴占主导：前倾或后仰
        if abs_x >= abs_y and abs_x >= abs_z:
            if ax > 0.6:
                return "forward"
            if ax < -0.6:
                return "backward"

        # Y 轴占主导：左右侧倒
        if abs_y >= abs_x and abs_y >= abs_z:
            if ay > 0.6:
                return "left"
            if ay < -0.6:
                return "right"

        return "unknown"

    def _classify(self, stats, gps_speed=None, speed_drop=0.0, moving_state="unknown"):
        """
        碰撞分级函数。

        分级输出：
            0 normal  正常
            1 light   轻微震动/小冲击
            2 warning 明显碰撞
            3 severe  严重碰撞

        设计原则：
            1. 严重冲击保留单次直通，避免漏检真实事故。
            2. 中低等级使用连续 N 次计数，降低误报。
            3. GNSS 速度只做辅助，不单独触发碰撞。
        """

        acc = stats["acc_peak"]
        gyro = stats["gyro_peak"]
        jerk = stats["jerk_peak"]

        # =========================
        # 1. 严重碰撞直通
        # =========================
        # 极高加速度通常意味着强冲击，不能因为连续计数不够而漏报
        t = self.impact_thresholds

        if acc >= t["severe_acc"]:
            return 3, "severe", "bypass_acc_over_6g"

        # 加速度和角速度同时很高，说明发生撞击并伴随剧烈旋转
        if acc >= t["severe_acc_with_gyro"] and gyro >= t["severe_gyro"]:
            return 3, "severe", "bypass_acc_gyro_high"

        # =========================
        # 2. GNSS 速度辅助升级
        # =========================
        # GNSS 不单独触发碰撞，必须结合 IMU 冲击特征
        if gps_speed is not None:
            # 速度突降 + 明显冲击：可能是运动中撞击后突然停止
            if speed_drop >= self.speed_thresholds["drop"] and acc >= t["l2_acc_jerk_acc"]:
                return 3, "severe", "speed_drop_and_impact"

            # 高速状态下出现强冲击，风险更高
            if moving_state == "high_speed" and acc >= t["high_speed_acc"]:
                return 3, "severe", "high_speed_impact"

            # 运动状态下中等冲击 + 较大角速度，判为明显碰撞
            if moving_state in ["moving", "high_speed"] and acc >= t["moving_acc"] and gyro >= t["moving_gyro"]:
                return 2, "warning", "moving_impact"

        # =========================
        # 3. 连续判据
        # =========================
        # L3 连续判据：较高加速度 + 角速度，或加速度 + jerk
        cond_l3 = (
            (acc >= t["l3_acc_gyro_acc"] and gyro >= t["l3_acc_gyro_gyro"])
            or (acc >= t["l3_acc_jerk_acc"] and jerk >= t["l3_acc_jerk_jerk"])
        )
        self._update_counter("l3", cond_l3)

        # L2 连续判据：明显冲击、明显旋转或冲击变化率较高
        cond_l2 = (
            (acc >= t["l2_acc"])
            or (gyro >= t["l2_gyro"])
            or (acc >= t["l2_acc_jerk_acc"] and jerk >= t["l2_acc_jerk_jerk"])
        )
        self._update_counter("l2", cond_l2)

        # L1 连续判据：
        # 静止时适当提高门限，降低敲击、放下头盔造成的误报
        if moving_state == "static":
            cond_l1 = acc >= t["l1_static_acc"] or jerk >= t["l1_static_jerk"]
        else:
            cond_l1 = acc >= t["l1_moving_acc"] or jerk >= t["l1_moving_jerk"]

        self._update_counter("l1", cond_l1)

        # =========================
        # 4. 高等级优先输出
        # =========================
        if self.cnt_l3 >= self.N_L3:
            return 3, "severe", "l3_dwell"

        if self.cnt_l2 >= self.N_L2:
            return 2, "warning", "l2_dwell"

        if self.cnt_l1 >= self.N_L1:
            return 1, "light", "l1_dwell"

        return 0, "normal", "normal"

    def _update_counter(self, level_name, condition):
        """
        连续计数器更新函数。

        参数：
            level_name:
                "l1"、"l2" 或 "l3"

            condition:
                当前等级的触发条件是否满足

        逻辑：
            condition 为 True：
                对应计数器 +1，gap 清零

            condition 为 False：
                如果 gap 还没超过 gap_allow，则先增加 gap
                如果 gap 已超过 gap_allow，则计数器清零

        作用：
            抑制单点毛刺，同时允许阈值边界有 1 帧抖动。
        """

        cnt_name = f"cnt_{level_name}"
        gap_name = f"gap_{level_name}"

        if condition:
            setattr(self, cnt_name, getattr(self, cnt_name) + 1)
            setattr(self, gap_name, 0)
        else:
            current_gap = getattr(self, gap_name)

            if current_gap < self.gap_allow:
                setattr(self, gap_name, current_gap + 1)
            else:
                setattr(self, cnt_name, 0)
                setattr(self, gap_name, 0)

    def _start_observation(self, now, level):
        """
        碰撞后进入观察状态。

        参数：
            now:
                当前时间

            level:
                当前碰撞等级

        说明：
            只要检测到碰撞，就进入观察状态。
            后续 observe_time 秒后，检查是否静止、是否倒地。
        """

        if not self.collision_observing:
            self.collision_observing = True
            self.collision_start_time = now
            self.max_collision_level = level
        else:
            # 如果观察期间又出现更高等级碰撞，记录最高等级
            self.max_collision_level = max(self.max_collision_level, level)

    def _check_sos(self, now):
        """
        自动 SOS 判断 + 摔倒方向判断。

        判断流程：
            1. 如果没有处于碰撞后观察状态，返回 False
            2. 如果观察时间还没到，返回 False
            3. 到达观察时间后，判断是否静止
            4. 根据静止后的三轴加速度判断摔倒方向
            5. 若明显碰撞/严重碰撞 + 静止 + 非直立，则触发 SOS

        返回：
            should_sos:
                是否需要自动 SOS

            fall_direction:
                摔倒方向
        """

        if not self.collision_observing:
            return False, "unknown"

        # 观察时间未到，不做最终判断
        if now - self.collision_start_time < self.observe_time:
            return False, "unknown"

        stats = self._get_window_stats()

        # 静止判断：
        # 平均加速度模长接近 1g，角速度较小
        static = (
            0.85 <= stats["acc_avg"] <= 1.20 and
            stats["gyro_avg"] < 20
        )

        # 只有静止后，摔倒方向判断才比较可信
        fall_direction = self._estimate_fall_direction(stats)

        # 明显碰撞/严重碰撞 + 静止 + 不是直立/未知，则认为可能摔倒
        fall_detected = (
            self.max_collision_level >= 2 and
            static and
            fall_direction not in ["upright", "unknown"]
        )

        should_sos = fall_detected

        # 本轮观察结束，重置观察状态
        self.collision_observing = False
        self.collision_start_time = None
        self.max_collision_level = 0

        return should_sos, fall_direction


def build_event(device_id, result, gps=None):
    """
    构造 MQTT 事件报文。

    Topic:
        helmet/{device_id}/event

    参数：
        device_id:
            设备唯一 ID

        result:
            HelmetCollisionDetector.update() 返回的检测结果

        gps:
            GPS/GNSS 信息，可选

    返回：
        payload 字典，可直接 json.dumps 后发布到 MQTT
    """

    payload = {
        "version": "1.2",
        "device_id": device_id,
        "msg_type": "event",
        "timestamp": result["timestamp"],

        # 事件类型
        "event": "collision",

        # 协议规定的预警字段
        "collision_alert": result["collision_alert"],
        "sos_alert": 2 if result["need_sos"] else 0,

        # 摔倒识别字段
        "fall_detected": result["fall_detected"],
        "fall_direction": result["fall_direction"],

        # 本次触发原因
        "reason": result["reason"],

        # 详细特征，方便服务端记录、调试和后续调参
        "collision_feature": {
            "acc_g": result["acc_g"],
            "gyro_dps": result["gyro_dps"],
            "jerk": result["jerk"],
            "acc_peak": result["acc_peak"],
            "gyro_peak": result["gyro_peak"],
            "jerk_peak": result["jerk_peak"],
            "gps_speed": result["gps_speed"],
            "speed_drop": result["speed_drop"],
            "moving_state": result["moving_state"],
            "fall_direction": result["fall_direction"]
        }
    }

    # 如果有 GPS 信息，则附加到事件报文
    if gps:
        payload["gps"] = gps

    return payload


class CollisionDetector:
    """Compatibility adapter for the old smart_helmet algorithm API."""

    def __init__(self, sample_rate=100):
        self.detector = HelmetCollisionDetector(sample_rate=sample_rate)

    def update(self, imu_data, dt_s=0.1):
        if not imu_data:
            return {"triggered": False}

        result = self.detector.update(
            imu_data.get("ax"),
            imu_data.get("ay"),
            imu_data.get("az"),
            imu_data.get("gx", 0.0),
            imu_data.get("gy", 0.0),
            imu_data.get("gz", 0.0),
        )
        if result.get("collision_alert", 0) <= 0 and not result.get("need_sos"):
            return {"triggered": False}

        return {
            "triggered": True,
            "code": result.get("collision_alert", 0),
            "level": result.get("collision_level", "normal"),
            "category": "collision",
            "message": result.get("reason", "collision_detected"),
            "need_sos": bool(result.get("need_sos")),
            "raw": result,
        }


if __name__ == "__main__":
    # 创建检测器实例
    detector = HelmetCollisionDetector(
        sample_rate=100,
        window_time=0.5,
        cooldown_time=2.0,
        observe_time=10.0
    )

    device_id = "HLM-00123"

    try:
        while True:
            # =========================
            # 示例数据：正常静止
            # =========================
            # 正常静止时，三轴加速度模长应接近 1g
            imu_ax, imu_ay, imu_az = 0.01, 0.02, 1.00

            # 角速度接近 0
            imu_gx, imu_gy, imu_gz = 0.5, 0.3, 0.2

            # GNSS 速度，单位 m/s
            gnss_speed = 2.0

            # =========================
            # 示例数据：模拟一次碰撞
            # =========================
            # 每 8 秒模拟一次短时间冲击
            if int(time.time() * 1000) % 8000 < 50:
                imu_ax, imu_ay, imu_az = 3.5, 1.2, 2.2
                imu_gx, imu_gy, imu_gz = 250, 180, 120
                gnss_speed = 0.2

            # 调用检测器
            detect_result = detector.update(
                imu_ax, imu_ay, imu_az,
                imu_gx, imu_gy, imu_gz,
                gps_speed=gnss_speed
            )

            # 如果检测到碰撞，则构造 MQTT 事件报文
            if detect_result["collision"]:
                event_payload = build_event(
                    device_id=device_id,
                    result=detect_result,
                    gps={
                        "lat": 32.0603,
                        "lon": 118.7969,
                        "fix": 2,
                        "speed": detect_result["gps_speed"]
                    }
                )

                topic = f"helmet/{device_id}/event"
                print(topic, event_payload)

            # 100Hz 循环，对应 0.01 秒
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("程序已手动停止")
