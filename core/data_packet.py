"""统一 JSON 数据包封装。"""

try:
    import ujson as json
except ImportError:
    import json
import time
import config


def now_s():
    """返回秒级时间戳。"""
    return int(time.time())


class DataPacketBuilder:
    """构造通信协议 v1.2 的 data、heartbeat 和 event 数据包。"""

    def __init__(self, helmet_id):
        """保存头盔设备 ID。"""
        self.helmet_id = helmet_id

    def build_data(self, sensor_data, system_data=None, alerts=None, algorithm=None):
        """根据 SensorManager 输出构造 msg_type=data 常规数据包。"""
        system_data = system_data or {}
        alerts = alerts or {}
        payload = {
            "version": "1.2",
            "device_id": self.helmet_id,
            "msg_type": "data",
            "timestamp": now_s(),
            "imu": self._imu(sensor_data.get("imu")),
            "gps": self._gps(sensor_data.get("gps")),
            "env": {
                "temp": self._round2(self._get_env(sensor_data, "temperature")),
                "hum": self._round2(self._get_env(sensor_data, "humidity")),
                "light": self._get(sensor_data, "light", "light"),
                "pressure_hpa": self._round2(self._get_barometer(sensor_data, "pressure_hpa")),
                "pressure_pa": self._round2(self._get_barometer(sensor_data, "pressure_pa")),
                "altitude_m": self._round2(self._get_barometer(sensor_data, "altitude_m")),
            },
            "vital": self._vital(sensor_data),
            "device": {
                "battery": system_data.get("battery"),
                "signal": system_data.get("signal"),
                "work_time": system_data.get("work_time", 0),
                "helmet_on": system_data.get("helmet_on", 1),
            },
            "alerts": {
                "collision_alert": alerts.get("collision_alert", 0),
                "heat_alert": alerts.get("heat_alert", 0),
                "fatigue_alert": alerts.get("fatigue_alert", 0),
                "sos_alert": alerts.get("sos_alert", 0),
                "radar_alert": alerts.get("radar_alert", 0),
                "body_temp_alert": alerts.get("body_temp_alert", 0),
                "hr_alert": alerts.get("hr_alert", 0),
                "spo2_alert": alerts.get("spo2_alert", 0),
            },
        }
        if getattr(config, "TELEMETRY_INCLUDE_ALGORITHM", False):
            algorithm_payload = self._algorithm(algorithm)
            if algorithm_payload is not None:
                payload["algorithm"] = algorithm_payload
        return payload

    def build_telemetry(self, sensor_data, system_data=None, alerts=None, algorithm=None):
        """兼容旧调用名，实际返回协议 v1.1 data 包。"""
        return self.build_data(sensor_data, system_data, alerts, algorithm)

    def build_heartbeat(self, system_data=None):
        """构造 msg_type=heartbeat 心跳包。"""
        system_data = system_data or {}
        return {
            "version": "1.2",
            "device_id": self.helmet_id,
            "msg_type": "heartbeat",
            "timestamp": now_s(),
            "battery": system_data.get("battery"),
            "signal": system_data.get("signal"),
            "helmet_on": system_data.get("helmet_on", 1),
        }

    def build_event(self, event, gps_data=None):
        """构造 msg_type=event 事件或预警包。"""
        gps_data = gps_data or {}
        payload = {
            "version": "1.2",
            "device_id": self.helmet_id,
            "msg_type": "event",
            "timestamp": now_s(),
            "event": event.get("event") or event.get("message") or event.get("category", "alert"),
        }
        category = event.get("category")
        if category == "collision":
            payload["collision_alert"] = self._alert_level(event)
        elif category == "heat":
            payload["heat_alert"] = self._alert_level(event)
        elif category in ("distance", "radar"):
            payload["radar_alert"] = self._alert_level(event)
        elif category == "sos":
            payload["sos_alert"] = 1 if event.get("need_sos", True) else self._alert_level(event)
        elif category == "fatigue":
            payload["fatigue_alert"] = self._alert_level(event)
        elif category == "body_temp":
            payload["body_temp_alert"] = self._alert_level(event)
        elif category == "heart_rate":
            payload["hr_alert"] = self._alert_level(event)
        elif category == "spo2":
            payload["spo2_alert"] = self._alert_level(event)
        payload["gps"] = {
            "lat": self._gps_lat(gps_data),
            "lon": self._gps_lon(gps_data),
            "fix": self._gps_fix(gps_data),
        }
        return payload

    def build_warning(self, warning, gps_data=None, raw=None):
        """兼容旧调用名，实际返回协议 v1.1 event 包。"""
        return self.build_event(warning, gps_data)

    def build_status(self, online):
        """构造在线状态包。"""
        return {"device_id": self.helmet_id, "online": online, "timestamp": now_s()}

    def alerts_from_warnings(self, warnings):
        """把算法输出转换为 v1.1 alerts 状态码。"""
        alerts = {
            "collision_alert": 0,
            "heat_alert": 0,
            "fatigue_alert": 0,
            "sos_alert": 0,
            "radar_alert": 0,
            "body_temp_alert": 0,
            "hr_alert": 0,
            "spo2_alert": 0,
        }
        for warning in warnings or []:
            if not warning or not warning.get("triggered"):
                continue
            level = self._alert_level(warning)
            category = warning.get("category")
            if category == "collision":
                alerts["collision_alert"] = max(alerts["collision_alert"], level)
            elif category == "heat":
                alerts["heat_alert"] = max(alerts["heat_alert"], level)
            elif category == "fatigue":
                alerts["fatigue_alert"] = max(alerts["fatigue_alert"], level)
            elif category == "sos":
                alerts["sos_alert"] = max(alerts["sos_alert"], 1)
            elif category in ("distance", "radar"):
                alerts["radar_alert"] = max(alerts["radar_alert"], level)
            elif category == "body_temp":
                alerts["body_temp_alert"] = max(alerts["body_temp_alert"], level)
            elif category == "heart_rate":
                alerts["hr_alert"] = max(alerts["hr_alert"], level)
            elif category == "spo2":
                alerts["spo2_alert"] = max(alerts["spo2_alert"], level)
        return alerts

    def _imu(self, data):
        """整理 IMU 字段，单位 g 和 deg/s。"""
        data = data or {}
        return {
            "ax": self._round2(data.get("ax")),
            "ay": self._round2(data.get("ay")),
            "az": self._round2(data.get("az")),
            "gx": self._round2(data.get("gx")),
            "gy": self._round2(data.get("gy")),
            "gz": self._round2(data.get("gz")),
        }

    def _gps(self, data):
        """整理 GPS/GNSS 字段。"""
        data = data or {}
        return {
            "lat": self._round2(self._gps_lat(data)),
            "lon": self._round2(self._gps_lon(data)),
            "alt": self._round2(data.get("alt") or data.get("altitude")),
            "speed": self._round2(self._gps_speed_ms(data)),
            "course": self._round2(data.get("course")),
            "sat": data.get("sat", 0) or 0,
            "hdop": self._round2(data.get("hdop")),
            "fix": self._gps_fix(data),
        }

    def _algorithm(self, data):
        if not isinstance(data, dict):
            return None
        collision = data.get("collision") or {}
        runtime = data.get("runtime_feature") or {}
        safety = data.get("safety") or {}
        response = data.get("response") or {}
        pre_warning = data.get("pre_warning") or {}
        fatigue = data.get("fatigue") or {}
        heat = data.get("heat") or {}
        vital = data.get("vital") or {}
        return {
            "version": data.get("version"),
            "timestamp": data.get("timestamp"),
            "safety": {
                "score": safety.get("safety_score"),
                "status": safety.get("risk_status"),
                "main": safety.get("main_risk_type"),
            },
            "response": {
                "level": response.get("response_level"),
                "sos": response.get("sos_alert"),
            },
            "runtime": {
                "pitch": runtime.get("pitch_deg"),
                "low_head_s": runtime.get("low_head_seconds"),
                "nod_1min": runtime.get("nod_count_1min"),
                "motion": runtime.get("motion_intensity"),
            },
            "pre": {
                "score": pre_warning.get("risk_pre_score"),
                "level": pre_warning.get("risk_pre_level"),
                "alert": pre_warning.get("risk_pre_alert"),
            },
            "fatigue": {
                "score": fatigue.get("fatigue_score"),
                "level": fatigue.get("fatigue_level"),
                "alert": fatigue.get("fatigue_alert"),
            },
            "heat": {
                "score": heat.get("heat_score"),
                "level": heat.get("heat_level"),
                "alert": heat.get("heat_alert"),
            },
            "vital": {
                "score": vital.get("vital_score"),
                "level": vital.get("vital_level"),
                "alert": vital.get("vital_alert"),
                "body_temp": vital.get("body_temp_alert"),
                "hr": vital.get("hr_alert"),
                "spo2": vital.get("spo2_alert"),
            },
            "collision": {
                "alert": collision.get("collision_alert"),
                "level": collision.get("collision_level"),
                "type": collision.get("accident_type"),
                "fall": collision.get("fall_detected"),
                "direction": collision.get("fall_direction"),
                "sos": collision.get("need_sos"),
                "reason": collision.get("reason"),
            },
        }

    def _vital(self, sensor_data):
        """整理 v1.2 人体健康字段。"""
        body = sensor_data.get("jx90614") or {}
        heart = sensor_data.get("max30100") or {}
        return {
            "body_temp": self._round2(body.get("body_temp")),
            "body_temp_source": body.get("body_temp_source"),
            "hr": heart.get("hr"),
            "spo2": heart.get("spo2"),
            "hr_valid": heart.get("hr_valid", 0),
            "spo2_valid": heart.get("spo2_valid", 0),
            "contact": heart.get("contact"),
        }

    def _gps_lat(self, data):
        return data.get("lat") if data.get("lat") is not None else data.get("latitude")

    def _gps_lon(self, data):
        return data.get("lon") if data.get("lon") is not None else data.get("longitude")

    def _gps_speed_ms(self, data):
        if data.get("speed") is not None:
            return data.get("speed")
        if data.get("speed_kmh") is not None:
            return data.get("speed_kmh") / 3.6
        return None

    def _gps_fix(self, data):
        if data.get("fix") is not None:
            return data.get("fix")
        return 1 if data.get("valid") else 0

    def _get_env(self, root, name):
        """兼容 temperature/humidity 和 temp/hum 两种驱动字段。"""
        env = root.get("sht40") or root.get("aht20") or {}
        if name == "temperature":
            return env.get("temperature") if env.get("temperature") is not None else env.get("temp")
        if name == "humidity":
            return env.get("humidity") if env.get("humidity") is not None else env.get("hum")
        return None

    def _get_barometer(self, root, name):
        """读取 BMP280/BME280 气压驱动字段。"""
        data = root.get("barometer") or {}
        return data.get(name) if isinstance(data, dict) else None

    def _alert_level(self, warning):
        """把算法等级转换为协议 0-3 状态码。"""
        level = warning.get("level")
        if level in ("severe", "sos", "danger", "high"):
            return 3
        if level in ("medium", "warning"):
            return 2
        if level in ("light", "suspected", "attention", "low", "mild"):
            return 1
        return 0

    def _round2(self, value):
        """浮点数保留两位；None 原样保留。"""
        if value is None:
            return None
        if isinstance(value, float):
            return round(value, 2)
        return value

    def dumps(self, payload):
        """把数据包序列化为 JSON 字符串。"""
        try:
            return json.dumps(payload, separators=(",", ":"))
        except TypeError:
            return json.dumps(payload)

    def _get(self, root, section, key):
        data = root.get(section)
        return data.get(key) if isinstance(data, dict) else None
