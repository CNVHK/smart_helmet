"""MQTT 上报客户端。"""

try:
    import ujson as json
except ImportError:
    import json

import config
import time

try:
    from umqtt.robust import MQTTClient
except ImportError:
    try:
        from umqtt.simple import MQTTClient
    except ImportError:
        MQTTClient = None


class HelmetMQTTClient:
    """封装 MQTT 连接、重连、遥测和预警发布。"""

    def __init__(self):
        """从 config 读取 MQTT 参数。"""
        self.connected = False
        self.client = None
        self.last_connect_attempt = 0
        if MQTTClient is not None:
            self.client = MQTTClient(
                client_id=config.MQTT_CLIENT_ID,
                server=config.MQTT_BROKER,
                port=config.MQTT_PORT,
                user=config.MQTT_USERNAME,
                password=config.MQTT_PASSWORD,
                keepalive=60,
            )

    def connect(self):
        """连接 MQTT Broker。"""
        if self.client is None:
            print("[MQTT] umqtt not found")
            return False
        now = self._ticks_ms()
        reconnect_ms = getattr(config, "MQTT_RECONNECT_INTERVAL_MS", 30000)
        if self.last_connect_attempt and self._ticks_diff(now, self.last_connect_attempt) < reconnect_ms:
            return False
        self.last_connect_attempt = now
        try:
            self.client.connect()
            self.connected = True
            print("[MQTT] connected")
            return True
        except Exception as exc:
            self.connected = False
            print("[MQTT] connect failed:", exc)
            self._safe_disconnect()
            return False

    def reconnect(self):
        """自动重连接口。"""
        self._safe_disconnect()
        return self.connect()

    def publish_telemetry(self, payload):
        """发布遥测数据。"""
        return self.publish_data(payload)

    def publish_warning(self, payload):
        """发布预警数据。"""
        return self.publish_event(payload)

    def publish_data(self, payload):
        """发布 msg_type=data 常规数据。"""
        return self._publish(config.MQTT_TOPIC_DATA, payload)

    def publish_event(self, payload):
        """发布 msg_type=event 事件/预警数据。"""
        return self._publish(config.MQTT_TOPIC_EVENT, payload)

    def publish_heartbeat(self, payload):
        """发布 msg_type=heartbeat 心跳包。"""
        return self._publish(config.MQTT_TOPIC_HEARTBEAT, payload)

    def publish_status(self, payload):
        """发布在线状态。"""
        return self._publish(config.MQTT_TOPIC_STATUS, payload)

    def _publish(self, topic, payload):
        """内部发布函数，发送前把 dict 转 JSON。"""
        if not self.connected and not self.connect():
            return False
        try:
            data = self._dumps(payload) if isinstance(payload, dict) else str(payload)
            topic_bytes = topic if isinstance(topic, bytes) else topic.encode()
            data_bytes = data if isinstance(data, bytes) else data.encode()
            self.client.publish(topic_bytes, data_bytes)
            return True
        except Exception as exc:
            print("[MQTT] publish failed:", exc)
            self.connected = False
            self._safe_disconnect()
            return False

    def _safe_disconnect(self):
        """尽量释放底层 socket 资源。"""
        try:
            if self.client:
                self.client.disconnect()
        except Exception:
            pass

    def _dumps(self, payload):
        try:
            return json.dumps(payload, separators=(",", ":"))
        except TypeError:
            return json.dumps(payload)

    def _ticks_ms(self):
        return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)

    def _ticks_diff(self, now, old):
        return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old
