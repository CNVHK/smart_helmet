"""通信协议辅助函数。"""

try:
    import ujson as json
except ImportError:
    import json


def encode_json(payload):
    """把 dict 安全编码为 JSON 字符串，失败返回 None。"""
    try:
        try:
            return json.dumps(payload, separators=(",", ":"))
        except TypeError:
            return json.dumps(payload)
    except Exception as exc:
        print("[Protocol] json encode failed:", exc)
        return None


def ensure_bytes(payload):
    """把字符串或字典转为 bytes，便于 UART/MQTT 发送。"""
    if isinstance(payload, dict):
        payload = encode_json(payload)
    if payload is None:
        return None
    if isinstance(payload, bytes):
        return payload
    return str(payload).encode()

