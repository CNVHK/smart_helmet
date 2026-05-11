"""智能头盔主程序入口。"""

import time
import config

try:
    from machine import I2C, UART, Pin, ADC
except ImportError:
    I2C = UART = Pin = ADC = None

from drivers.imu import IMU
from drivers.sht40 import SHT40
from drivers.jx90614 import JX90614
from drivers.max30100 import MAX30100
from drivers.light_sensor import LightSensor
from drivers.barometer import Barometer
from drivers.gps_uart import GPSUART
from drivers.gnss_quectel import QuectelGNSS
from drivers.ultrasonic import Ultrasonic
from drivers.buzzer import Buzzer
from drivers.button import Button
from core.sensor_manager import SensorManager
from core.data_packet import DataPacketBuilder
from core.system_status import SystemStatus
from algorithms.collision_detector import CollisionDetector
from algorithms.heat_detector import HeatDetector
from algorithms.fatigue_detector import FatigueDetector
from algorithms.distance_warning import DistanceWarning
from communication.mqtt_client import HelmetMQTTClient
from communication.uart_4g import G4Module


def sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def ticks_ms():
    """毫秒计时兼容函数。"""
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def ticks_diff(now, old):
    """计时差兼容函数。"""
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


def make_i2c():
    """兼容不同 MicroPython 固件的 I2C 构造方式。"""
    attempts = (
        lambda: I2C(config.I2C_ID, freq=config.I2C_FREQ),
        lambda: I2C(config.I2C_ID),
        lambda: I2C(config.I2C_ID, scl=Pin(config.I2C_SCL_PIN), sda=Pin(config.I2C_SDA_PIN), freq=config.I2C_FREQ),
        lambda: I2C(config.I2C_ID, scl=config.I2C_SCL_PIN, sda=config.I2C_SDA_PIN, freq=config.I2C_FREQ),
    )
    for attempt in attempts:
        try:
            return attempt()
        except Exception as exc:
            print("[MAIN] I2C init attempt failed:", exc)
    return None


def scan_i2c(i2c):
    """扫描 I2C 设备，失败时返回空列表。"""
    if i2c is None or not hasattr(i2c, "scan"):
        return []
    try:
        devices = i2c.scan()
        print("[MAIN] I2C devices:", [hex(addr) for addr in devices])
        return devices
    except Exception as exc:
        print("[MAIN] I2C scan failed:", exc)
        return []


def make_uart(uart_id, baudrate, tx_pin, rx_pin):
    """兼容不同 MicroPython 固件的 UART 构造方式。"""
    attempts = (
        lambda: UART(uart_id, baudrate),
        lambda: UART(uart_id, baudrate=baudrate),
        lambda: UART(uart_id),
        lambda: UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin)),
        lambda: UART(uart_id, baudrate=baudrate, tx=tx_pin, rx=rx_pin),
    )
    for attempt in attempts:
        try:
            return attempt()
        except Exception as exc:
            print("[MAIN] UART init attempt failed:", exc)
    return None


def make_pin(pin_id, mode=None, pull=None, pin_name=None):
    """创建 GPIO 引脚，失败时返回 None 进入降级模式。"""
    ids = []
    if pin_name is not None:
        ids.append(pin_name)
    ids.append(pin_id)
    for item in ids:
        try:
            if pull is not None:
                return Pin(item, mode, pull)
            if mode is not None:
                return Pin(item, mode)
            return Pin(item)
        except Exception as exc:
            print("[MAIN] Pin init failed:", item, exc)
    return None


def build_hardware():
    """初始化 I2C、UART、GPIO，并在失败时返回 None 以降级运行。"""
    if I2C is None:
        print("[MAIN] machine module not found, running in dry mode")
        return {}, None, None, None
    i2c = make_i2c()
    i2c_devices = scan_i2c(i2c)
    gps_uart = make_uart(config.GPS_UART_ID, config.GPS_BAUDRATE, config.GPS_TX_PIN, config.GPS_RX_PIN)
    g4_uart = make_uart(config.G4_UART_ID, config.G4_BAUDRATE, config.G4_TX_PIN, config.G4_RX_PIN)
    buzzer_pin = make_pin(config.BUZZER_PIN, Pin.OUT, pin_name=config.BUZZER_PIN_NAME)
    sos_pin = make_pin(config.SOS_BUTTON_PIN, Pin.IN, getattr(Pin, "PULL_UP", None), config.SOS_BUTTON_PIN_NAME)
    cancel_pin = make_pin(config.CANCEL_BUTTON_PIN, Pin.IN, getattr(Pin, "PULL_UP", None), config.CANCEL_BUTTON_PIN_NAME)
    trig = make_pin(config.ULTRASONIC_TRIG_PIN, Pin.OUT, pin_name=config.ULTRASONIC_TRIG_PIN_NAME)
    echo = make_pin(config.ULTRASONIC_ECHO_PIN, Pin.IN, pin_name=config.ULTRASONIC_ECHO_PIN_NAME)
    try:
        light_pin = make_pin(config.LIGHT_ADC_PIN, pin_name=config.LIGHT_ADC_PIN_NAME)
        light_adc = ADC(light_pin) if light_pin is not None else None
    except Exception:
        light_adc = None
    gnss = QuectelGNSS()
    temp_hum = SHT40(i2c, addr=config.SHT40_ADDR) if i2c is not None and config.SHT40_ENABLE else None
    sensors = {
        "imu": IMU(i2c) if i2c is not None else None,
        "sht40": temp_hum,
        "jx90614": JX90614(i2c, config.JX90614_ADDR) if i2c is not None and config.JX90614_ENABLE else None,
        "max30100": MAX30100(i2c) if i2c is not None else None,
        "barometer": Barometer(i2c),
        "light": LightSensor(light_adc),
        "gps": gnss if gnss.gnss is not None else GPSUART(gps_uart),
        "ultrasonic": Ultrasonic(trig, echo),
    }
    sensors = {name: sensor for name, sensor in sensors.items() if sensor is not None}
    buttons = {"sos": Button(sos_pin), "cancel": Button(cancel_pin)}
    return sensors, Buzzer(buzzer_pin), buttons, G4Module(g4_uart)


def select_warning(warnings):
    """从多个算法结果中选择最严重的一条。"""
    rank = {"suspected": 1, "light": 2, "medium": 3, "severe": 4}
    triggered = [item for item in warnings if item and item.get("triggered")]
    if not triggered:
        return None
    return sorted(triggered, key=lambda item: rank.get(item.get("level"), 0), reverse=True)[0]


def upload_payload(payload, mqtt, g4, cache, warning=False):
    """优先 MQTT 上传，失败时尝试 4G 透传，仍失败则缓存。"""
    ok = False
    if config.MQTT_ENABLE and mqtt is not None:
        ok = mqtt.publish_warning(payload) if warning else mqtt.publish_telemetry(payload)
    if not ok and g4 is not None:
        ok = g4.send_json(payload)
    if not ok:
        cache.append((payload, warning))
        while len(cache) > config.COMM_CACHE_MAX:
            cache.pop(0)
    return ok


def flush_cache(mqtt, g4, cache):
    """通信恢复后补发缓存数据。"""
    kept = []
    for payload, warning in cache:
        ok = False
        if config.MQTT_ENABLE and mqtt is not None:
            ok = mqtt.publish_warning(payload) if warning else mqtt.publish_telemetry(payload)
        if not ok and g4 is not None:
            ok = g4.send_json(payload)
        if not ok:
            kept.append((payload, warning))
    cache[:] = kept[-config.COMM_CACHE_MAX:]


def main():
    """主循环：采集、检测、封包、上传、本地报警。"""
    sensors, buzzer, buttons, g4 = build_hardware()
    manager = SensorManager(sensors)
    manager.init_all()
    if buzzer:
        buzzer.init()
    if g4:
        g4.init()

    mqtt = HelmetMQTTClient() if config.MQTT_ENABLE else None
    if mqtt:
        mqtt.connect()

    status = SystemStatus()
    status.set_network("mqtt" if mqtt and mqtt.connected else "4g")
    packet_builder = DataPacketBuilder(config.DEVICE_ID)
    collision = CollisionDetector()
    heat = HeatDetector()
    fatigue = FatigueDetector()
    distance = DistanceWarning()
    cache = []
    last_upload = ticks_ms()
    last_heartbeat = ticks_ms()

    print("[MAIN] smart helmet started")
    while True:
        try:
            sensor_data = manager.read_all()
            warnings = [
                collision.update(sensor_data.get("imu"), config.SENSOR_UPDATE_MS / 1000.0),
                heat.update(sensor_data.get("sht40")),
                fatigue.update(sensor_data),
                distance.update(sensor_data.get("ultrasonic")),
            ]
            if buttons.get("sos") and buttons["sos"].is_pressed():
                warnings.append({
                    "triggered": True,
                    "code": 401,
                    "level": "sos",
                    "category": "sos",
                    "message": "manual_sos",
                    "need_sos": True,
                    "raw": {},
                })
            if buttons.get("cancel") and buttons["cancel"].is_pressed() and buzzer:
                buzzer.off()

            alerts = packet_builder.alerts_from_warnings(warnings)
            telemetry = packet_builder.build_data(sensor_data, status.to_dict(), alerts)
            warning = select_warning(warnings)

            if warning:
                warning_packet = packet_builder.build_event(warning, sensor_data.get("gps"))
                upload_payload(warning_packet, mqtt, g4, cache, warning=True)
                if buzzer:
                    buzzer.alert(warning.get("level"))

            if ticks_diff(ticks_ms(), last_upload) >= config.UPLOAD_INTERVAL_MS:
                upload_payload(telemetry, mqtt, g4, cache, warning=False)
                flush_cache(mqtt, g4, cache)
                last_upload = ticks_ms()

            if ticks_diff(ticks_ms(), last_heartbeat) >= config.HEARTBEAT_INTERVAL_MS:
                heartbeat = packet_builder.build_heartbeat(status.to_dict())
                if mqtt is not None:
                    mqtt.publish_heartbeat(heartbeat)
                last_heartbeat = ticks_ms()

        except Exception as exc:
            print("[MAIN] loop error:", exc)
        sleep_ms(config.SENSOR_UPDATE_MS)


if __name__ == "__main__":
    main()
