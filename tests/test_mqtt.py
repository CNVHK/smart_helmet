"""MQTT 单项测试：连接并发送测试遥测。"""

import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

from communication.mqtt_client import HelmetMQTTClient
from core.data_packet import DataPacketBuilder
import config

def main():
    """连接 MQTT 并发布一条测试数据。"""
    print("mqtt ip:", config.MQTT_BROKER)
    print("mqtt port:", config.MQTT_PORT)
    print("mqtt user:", config.MQTT_USERNAME)

    builder = DataPacketBuilder(config.DEVICE_ID)
    payload = builder.build_telemetry({
        "imu": {"ax": 0, "ay": 0, "az": 1.0, "gx": None, "gy": None, "gz": None},
        "sht40": {"temperature": 25.0, "humidity": 50.0},
        "barometer": None,
        "light": {"light": 100},
        "gps": {"valid": False, "latitude": None, "longitude": None, "speed_kmh": None},
        "ultrasonic": {"front_cm": None},
    })
    mqtt = HelmetMQTTClient()
    connected = mqtt.connect()
    print("connect:", connected)
    if not connected:
        return
    print("publish:", mqtt.publish_telemetry(payload))


main()
