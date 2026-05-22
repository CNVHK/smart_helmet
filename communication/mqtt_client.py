"""MQTT 上报客户端。"""

try:
    import ujson as json
except ImportError:
    import json

import config
import time
try:
    import gc
except ImportError:
    gc = None
try:
    import socket
except ImportError:
    try:
        import usocket as socket
    except ImportError:
        socket = None

try:
    from umqtt.simple import MQTTClient
except ImportError:
    try:
        from umqtt.robust import MQTTClient
    except ImportError:
        MQTTClient = None


class HelmetMQTTClient:
    """封装 MQTT 连接、重连、遥测和预警发布。"""

    def __init__(self):
        """从 config 读取 MQTT 参数。"""
        self.connected = False
        self.client = None
        self.last_connect_attempt = 0
        self.last_publish_failure = 0
        self.last_hard_failure = 0
        self._new_client()

    def connect(self):
        """连接 MQTT Broker。"""
        if MQTTClient is None:
            print("[MQTT] umqtt not found")
            return False
        if self.client is None:
            self._new_client()
        now = self._ticks_ms()
        reconnect_ms = getattr(config, "MQTT_RECONNECT_INTERVAL_MS", 30000)
        if self.last_connect_attempt and self._ticks_diff(now, self.last_connect_attempt) < reconnect_ms:
            return False
        self.last_connect_attempt = now
        try:
            self._connect_client()
            self.connected = True
            print("[MQTT] connected")
            return True
        except Exception as exc:
            self.connected = False
            print("[MQTT] connect failed:", exc)
            self._mark_failure(hard=True)
            return False

    def reconnect(self):
        """自动重连接口。"""
        self._drop_client()
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
        now = self._ticks_ms()
        retry_ms = int(getattr(config, "MQTT_PUBLISH_RETRY_INTERVAL_MS", 10000))
        hard_retry_ms = int(getattr(config, "MQTT_HARD_FAILURE_COOLDOWN_MS", retry_ms))
        if self.last_hard_failure and self._ticks_diff(now, self.last_hard_failure) < hard_retry_ms:
            return False
        if self.last_publish_failure and self._ticks_diff(now, self.last_publish_failure) < retry_ms:
            return False
        if not self.connected and not self.connect():
            return False
        try:
            if gc:
                gc.collect()
            data = self._dumps(payload) if isinstance(payload, dict) else str(payload)
            topic_bytes = topic if isinstance(topic, bytes) else topic.encode()
            data_bytes = data if isinstance(data, bytes) else data.encode()
            start = self._ticks_ms()
            self.client.publish(topic_bytes, data_bytes)
            elapsed = self._ticks_diff(self._ticks_ms(), start)
            timeout_ms = int(getattr(config, "MQTT_PUBLISH_TIMEOUT_MS", 12000))
            if elapsed > timeout_ms:
                print("[MQTT] publish slow:", elapsed, "ms")
                self._mark_failure(hard=True)
                return False
            self.last_publish_failure = 0
            return True
        except Exception as exc:
            print("[MQTT] publish failed:", exc)
            self._mark_failure(hard=True)
            data = None
            data_bytes = None
            if gc:
                gc.collect()
            return False

    def _safe_disconnect(self):
        """正常链路上的优雅断开。故障链路不要调用这个函数。"""
        try:
            if self.client:
                self.client.disconnect()
        except Exception:
            pass
        self._drop_client()

    def _drop_client(self):
        """直接丢弃 MQTTClient，避免坏 socket 继续触发 QISEND。"""
        self.client = None
        self.connected = False
        if gc:
            gc.collect()

    def _mark_failure(self, hard=False):
        """记录失败并进入冷却；hard=True 时直接丢弃底层 client。"""
        now = self._ticks_ms()
        self.connected = False
        self.last_publish_failure = now
        if hard:
            self.last_hard_failure = now
            self._drop_client()

    def _new_client(self):
        """创建新的 MQTTClient，避免失败后的底层 socket 被长期复用。"""
        if MQTTClient is None:
            self.client = None
            return None
        self.client = MQTTClient(
            client_id=config.MQTT_CLIENT_ID,
            server=config.MQTT_BROKER,
            port=config.MQTT_PORT,
            user=config.MQTT_USERNAME,
            password=config.MQTT_PASSWORD,
            keepalive=getattr(config, "MQTT_KEEPALIVE", 30),
        )
        return self.client

    def _connect_client(self):
        broker = config.MQTT_BROKER
        if not self._is_ipv4(broker) or socket is None or not hasattr(socket, "getaddrinfo"):
            result = self.client.connect()
            self._set_socket_timeout()
            return result

        original_getaddrinfo = socket.getaddrinfo

        def direct_ip_getaddrinfo(host, port, *args, **kwargs):
            if host == broker:
                family = getattr(socket, "AF_INET", 2)
                socktype = getattr(socket, "SOCK_STREAM", 1)
                return [(family, socktype, 0, "", (broker, port))]
            return original_getaddrinfo(host, port, *args, **kwargs)

        socket.getaddrinfo = direct_ip_getaddrinfo
        try:
            result = self.client.connect()
            self._set_socket_timeout()
            return result
        finally:
            socket.getaddrinfo = original_getaddrinfo

    def _set_socket_timeout(self):
        """限制 publish 阻塞时间，避免网络异常时主循环长时间卡死。"""
        timeout = getattr(config, "MQTT_SOCKET_TIMEOUT_S", 8)
        try:
            sock = getattr(self.client, "sock", None)
            if sock is not None and hasattr(sock, "settimeout"):
                sock.settimeout(timeout)
        except Exception:
            pass

    def _is_ipv4(self, value):
        if not isinstance(value, str):
            return False
        parts = value.split(".")
        if len(parts) != 4:
            return False
        for part in parts:
            if not part or not part.isdigit():
                return False
            number = int(part)
            if number < 0 or number > 255:
                return False
        return True

    def _dumps(self, payload):
        try:
            return json.dumps(payload, separators=(",", ":"))
        except TypeError:
            return json.dumps(payload)

    def _ticks_ms(self):
        return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)

    def _ticks_diff(self, now, old):
        return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old
