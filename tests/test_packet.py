"""数据包测试：构造 telemetry 和 warning JSON。"""

import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

import config
from core.data_packet import DataPacketBuilder


def main():
    """构造并打印示例数据包。"""
    builder = DataPacketBuilder(config.DEVICE_ID)
    sensor_data = {
        "imu": {"ax": 0.01, "ay": -0.02, "az": 1.0, "gx": None, "gy": None, "gz": None},
        "sht40": {"temperature": 28.6, "humidity": 61.2},
        "barometer": {"pressure": 1012.5},
        "light": {"light": 320},
        "jx90614": {"body_temp": 24.33, "body_temp_source": "infrared", "body_temp_raw": 398619},
        "gps": {"fix": 2, "lat": 32.0603, "lon": 118.7969, "alt": 12.5, "speed": 1.2, "course": 90.0, "sat": 8, "hdop": 1.2},
        "ultrasonic": {"front_cm": 85.2},
    }
    alerts = {"collision_alert": 2, "heat_alert": 0, "fatigue_alert": 0, "sos_alert": 0, "radar_alert": 1, "body_temp_alert": 0}
    telemetry = builder.build_data(sensor_data, {"battery": 85, "signal": 4, "work_time": 3600, "helmet_on": 1}, alerts)
    event = builder.build_event({
        "level": "medium",
        "category": "collision",
        "message": "medium_collision_detected",
        "need_sos": False,
    }, sensor_data["gps"])
    heartbeat = builder.build_heartbeat({"battery": 85, "signal": 4, "helmet_on": 1})
    print(builder.dumps(telemetry))
    print(builder.dumps(event))
    print(builder.dumps(heartbeat))


main()
