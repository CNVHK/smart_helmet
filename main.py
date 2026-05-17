"""SmartHelmet main entry point."""

import sys
import time
import config
try:
    import gc
except ImportError:
    gc = None

try:
    import os
    _ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
except Exception:
    _ROOT_DIR = ".."
if _ROOT_DIR not in sys.path:
    sys.path.append(_ROOT_DIR)

from algorithms.algorithm_architecture import SmartHelmetAlgorithm

try:
    from machine import I2C, UART, Pin, ADC
except ImportError:
    I2C = UART = Pin = ADC = None

from drivers.imu import IMU
from drivers.sht40 import SHT40
from drivers.jx90614 import JX90614
from drivers.max30100 import MAX30100
from drivers.gy302 import GY302
from drivers.light_sensor import ADCLightSensor
from drivers.barometer import Barometer
from drivers.gps_uart import GPSUART
from drivers.gnss_quectel import QuectelGNSS
from drivers.ultrasonic import Ultrasonic
from drivers.buzzer import Buzzer
from drivers.button import Button
from core.sensor_manager import SensorManager
from core.data_packet import DataPacketBuilder
from core.system_status import SystemStatus
from algorithms.distance_warning import DistanceWarning
from communication.mqtt_client import HelmetMQTTClient
from communication.uart_4g import G4Module


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def ticks_ms():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def ticks_diff(now, old):
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


def now_s():
    return int(time.time())


def _field(data, key, default=None):
    if not isinstance(data, dict):
        return default
    value = data.get(key)
    return default if value is None else value


def _gps_speed_mps(gps_data):
    if not getattr(config, "GNSS_ENABLE", True):
        return None
    if not isinstance(gps_data, dict):
        return None
    speed = gps_data.get("speed")
    if speed is not None:
        return speed
    speed_kmh = gps_data.get("speed_kmh")
    if speed_kmh is not None:
        return speed_kmh / 3.6
    return None


def _light_lux(light_data):
    if not isinstance(light_data, dict):
        return None
    value = light_data.get("light")
    if value is None:
        value = light_data.get("lux")
    return value


def _strong_sun(light_data):
    value = _light_lux(light_data)
    try:
        return value is not None and float(value) >= 30000.0
    except (TypeError, ValueError):
        return False


def _valid_vital(vital_data, value_key, valid_key):
    if not isinstance(vital_data, dict):
        return None
    value = vital_data.get(value_key)
    if value is None:
        return None
    contact = vital_data.get("contact")
    valid = vital_data.get(valid_key)
    if contact is False or contact == 0 or valid == 0:
        return None
    return value


def _fmt(value, digits=2):
    if value is None:
        return "None"
    try:
        return str(round(float(value), digits))
    except (TypeError, ValueError):
        return str(value)


def _algorithm_debug_enabled():
    return bool(getattr(config, "ALGORITHM_DEBUG_ENABLE", False))


def _algorithm_debug_interval_ms():
    return int(getattr(config, "ALGORITHM_DEBUG_INTERVAL_MS", 1000))


def _print_algorithm_debug(sensor_data, result, warnings, force=False):
    if not _algorithm_debug_enabled():
        return
    if not force and not bool(getattr(config, "ALGORITHM_DEBUG_PRINT_NORMAL", True)):
        return

    imu = sensor_data.get("imu") or {}
    gps = sensor_data.get("gps") or {}
    runtime = result.get("runtime_feature") or {}
    collision = result.get("collision") or {}
    pre_warning = result.get("pre_warning") or {}
    fatigue = result.get("fatigue") or {}
    heat = result.get("heat") or {}
    safety = result.get("safety") or {}
    response = result.get("response") or {}

    print(
        "[ALG] ts={ts} imu=({ax},{ay},{az};{gx},{gy},{gz}) gps={gps} "
        "pitch={pitch} low_head={low_head}s nod={nod} motion={motion} "
        "collision={ca}/{cl} reason={cr} acc_peak={ap} gyro_peak={gp} jerk_peak={jp} "
        "pre={pa}/{pl} fatigue={fa}/{fl} heat={ha}/{hl} safety={score}/{status} response={resp}".format(
            ts=result.get("timestamp"),
            ax=_fmt(imu.get("ax")),
            ay=_fmt(imu.get("ay")),
            az=_fmt(imu.get("az")),
            gx=_fmt(imu.get("gx")),
            gy=_fmt(imu.get("gy")),
            gz=_fmt(imu.get("gz")),
            gps=_fmt(_gps_speed_mps(gps)),
            pitch=_fmt(runtime.get("pitch_deg"), 1),
            low_head=_fmt(runtime.get("low_head_seconds"), 1),
            nod=runtime.get("nod_count_1min"),
            motion=_fmt(runtime.get("motion_intensity")),
            ca=collision.get("collision_alert"),
            cl=collision.get("collision_level"),
            cr=collision.get("reason"),
            ap=_fmt(collision.get("acc_peak")),
            gp=_fmt(collision.get("gyro_peak"), 1),
            jp=_fmt(collision.get("jerk_peak")),
            pa=pre_warning.get("risk_pre_alert"),
            pl=pre_warning.get("risk_pre_level"),
            fa=fatigue.get("fatigue_alert"),
            fl=fatigue.get("fatigue_level"),
            ha=heat.get("heat_alert"),
            hl=heat.get("heat_level"),
            score=safety.get("safety_score"),
            status=safety.get("risk_status"),
            resp=response.get("response_level"),
        )
    )

    if warnings:
        print("[ALG-WARN]", warnings)


def _warning_rank(level):
    return {
        "normal": 0,
        "safe": 0,
        "attention": 1,
        "low": 1,
        "mild": 1,
        "light": 1,
        "suspected": 1,
        "warning": 2,
        "medium": 2,
        "high": 3,
        "danger": 3,
        "severe": 4,
        "sos": 5,
    }.get(level, 0)


def _buzzer_level(level):
    if level == "sos":
        return "sos"
    if level in ("severe", "danger", "high"):
        return "severe"
    if level in ("warning", "medium"):
        return "medium"
    if level in ("attention", "low", "mild"):
        return "light"
    if level in ("light", "suspected"):
        return level
    return "suspected"


def _warning_packet(device_id, warning, gps_data=None, packet_builder=None):
    if warning and warning.get("category") in (
        "collision",
        "heat",
        "fatigue",
        "sos",
        "distance",
        "body_temp",
        "heart_rate",
        "spo2",
    ):
        packet = dict(warning)
        packet["level"] = _buzzer_level(packet.get("level"))
        if packet_builder is not None:
            return packet_builder.build_event(packet, gps_data)
        return packet
    payload = {
        "version": "1.4",
        "device_id": device_id,
        "msg_type": "event",
        "timestamp": now_s(),
        "event": warning.get("message") if warning else "alert",
    }
    if gps_data:
        payload["gps"] = gps_data
    if warning:
        payload["algorithm_warning"] = warning
    return payload


def _algorithm_warnings(result):
    warnings = []
    if not result:
        return warnings

    collision = result.get("collision") or {}
    if collision.get("collision_alert", 0) > 0 or collision.get("need_sos"):
        warnings.append({
            "triggered": True,
            "code": collision.get("collision_alert", 0),
            "level": collision.get("collision_level") or "severe",
            "category": "collision",
            "message": collision.get("reason") or "collision_detected",
            "need_sos": bool(collision.get("need_sos")),
            "raw": collision,
        })

    pre_warning = result.get("pre_warning") or {}
    if pre_warning.get("risk_pre_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": pre_warning.get("risk_pre_alert", 0),
            "level": pre_warning.get("risk_pre_level") or "low",
            "category": "riding_risk",
            "message": "riding_risk_warning",
            "need_sos": False,
            "raw": pre_warning,
        })

    fatigue = result.get("fatigue") or {}
    if fatigue.get("fatigue_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": fatigue.get("fatigue_alert", 0),
            "level": fatigue.get("fatigue_level") or "mild",
            "category": "fatigue",
            "message": fatigue.get("fatigue_action") or "fatigue_warning",
            "need_sos": False,
            "raw": fatigue,
        })

    heat = result.get("heat") or {}
    if heat.get("heat_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": heat.get("heat_alert", 0),
            "level": heat.get("heat_level") or "attention",
            "category": "heat",
            "message": heat.get("heat_action") or "heat_warning",
            "need_sos": False,
            "raw": heat,
        })

    vital = result.get("vital") or {}
    if vital.get("body_temp_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": vital.get("body_temp_alert", 0),
            "level": vital.get("vital_level") or "attention",
            "category": "body_temp",
            "message": "body_temp_warning",
            "need_sos": False,
            "raw": vital,
        })
    if vital.get("hr_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": vital.get("hr_alert", 0),
            "level": vital.get("vital_level") or "attention",
            "category": "heart_rate",
            "message": "heart_rate_warning",
            "need_sos": False,
            "raw": vital,
        })
    if vital.get("spo2_alert", 0) > 0:
        warnings.append({
            "triggered": True,
            "code": vital.get("spo2_alert", 0),
            "level": vital.get("vital_level") or "attention",
            "category": "spo2",
            "message": "spo2_warning",
            "need_sos": False,
            "raw": vital,
        })

    response = result.get("response") or {}
    if response.get("response_level") == "emergency" or response.get("sos_alert", 0):
        warnings.append({
            "triggered": True,
            "code": response.get("sos_alert", 2),
            "level": "sos",
            "category": "sos",
            "message": "algorithm_emergency",
            "need_sos": True,
            "raw": response,
        })

    return warnings


def make_i2c():
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
    if I2C is None:
        print("[MAIN] machine module not found, running in dry mode")
        return {}, None, None, None
    raw_4g_enabled = bool(getattr(config, "G4_RAW_FALLBACK_ENABLE", False))
    i2c = make_i2c()
    scan_i2c(i2c)
    gps_uart = None
    if getattr(config, "GNSS_ENABLE", True):
        gps_uart = make_uart(config.GPS_UART_ID, config.GPS_BAUDRATE, config.GPS_TX_PIN, config.GPS_RX_PIN)
    g4_uart = None
    if getattr(config, "COMM_UPLOAD_ENABLE", True) and raw_4g_enabled:
        g4_uart = make_uart(config.G4_UART_ID, config.G4_BAUDRATE, config.G4_TX_PIN, config.G4_RX_PIN)
    buzzer_pin = make_pin(config.BUZZER_PIN, Pin.OUT, pin_name=config.BUZZER_PIN_NAME)
    sos_pin = make_pin(config.SOS_BUTTON_PIN, Pin.IN, getattr(Pin, "PULL_UP", None), config.SOS_BUTTON_PIN_NAME)
    cancel_pin = make_pin(config.CANCEL_BUTTON_PIN, Pin.IN, getattr(Pin, "PULL_UP", None), config.CANCEL_BUTTON_PIN_NAME)
    trig = make_pin(config.ULTRASONIC_TRIG_PIN, Pin.OUT, pin_name=config.ULTRASONIC_TRIG_PIN_NAME)
    echo = make_pin(config.ULTRASONIC_ECHO_PIN, Pin.IN, pin_name=config.ULTRASONIC_ECHO_PIN_NAME)

    light_adc = None
    if not getattr(config, "GY302_ENABLE", True):
        try:
            light_pin = make_pin(config.LIGHT_ADC_PIN, pin_name=config.LIGHT_ADC_PIN_NAME)
            light_adc = ADC(light_pin) if light_pin is not None else None
        except Exception:
            light_adc = None

    gnss = QuectelGNSS() if getattr(config, "GNSS_ENABLE", True) else None
    temp_hum = SHT40(i2c, addr=config.SHT40_ADDR) if i2c is not None and config.SHT40_ENABLE else None
    light_sensor = GY302(i2c, addr=config.GY302_ADDR) if i2c is not None and config.GY302_ENABLE else ADCLightSensor(light_adc)
    sensors = {
        "imu": IMU(i2c) if i2c is not None else None,
        "sht40": temp_hum,
        "jx90614": JX90614(i2c, config.JX90614_ADDR) if i2c is not None and config.JX90614_ENABLE else None,
        "max30100": MAX30100(i2c) if i2c is not None else None,
        "barometer": Barometer(i2c, addr=config.BAROMETER_ADDR) if i2c is not None and config.BAROMETER_ENABLE else None,
        "light": light_sensor,
        "ultrasonic": Ultrasonic(trig, echo),
    }
    if getattr(config, "GNSS_ENABLE", True):
        sensors["gps"] = gnss if gnss is not None and gnss.gnss is not None else GPSUART(gps_uart)
    sensors = {name: sensor for name, sensor in sensors.items() if sensor is not None}
    buttons = {"sos": Button(sos_pin), "cancel": Button(cancel_pin)}
    g4 = G4Module(g4_uart) if getattr(config, "COMM_UPLOAD_ENABLE", True) and raw_4g_enabled else None
    return sensors, Buzzer(buzzer_pin), buttons, g4


def select_warning(warnings):
    selected = None
    selected_rank = -1
    for item in warnings or []:
        if not item or not item.get("triggered"):
            continue
        rank = _warning_rank(item.get("level"))
        if rank > selected_rank:
            selected = item
            selected_rank = rank
    return selected


def warning_upload_due(warning, last_warning_upload):
    if not warning:
        return False
    interval_ms = int(getattr(config, "WARNING_UPLOAD_INTERVAL_MS", 5000))
    key = "{}:{}:{}".format(
        warning.get("category"),
        warning.get("message"),
        warning.get("level"),
    )
    now = ticks_ms()
    last = last_warning_upload.get(key)
    if last is None or ticks_diff(now, last) >= interval_ms:
        last_warning_upload[key] = now
        return True
    return False


def upload_payload(payload, mqtt, g4, cache, warning=False):
    if not getattr(config, "COMM_UPLOAD_ENABLE", True):
        return True
    ok = False
    if config.MQTT_ENABLE and mqtt is not None:
        ok = mqtt.publish_warning(payload) if warning else mqtt.publish_telemetry(payload)
    raw_fallback = bool(getattr(config, "G4_RAW_FALLBACK_ENABLE", False))
    if not ok and raw_fallback and g4 is not None:
        ok = g4.send_json(payload)
    if not ok:
        if not warning:
            cache[:] = [item for item in cache if item[1]]
        cache.append((payload, warning))
        while len(cache) > config.COMM_CACHE_MAX:
            cache.pop(0)
    return ok


def flush_cache(mqtt, g4, cache, max_items=None):
    if not cache:
        return
    max_items = int(max_items if max_items is not None else getattr(config, "COMM_CACHE_FLUSH_BATCH", 1))
    kept = []
    sent_count = 0
    for payload, warning in cache:
        if sent_count >= max_items:
            kept.append((payload, warning))
            continue
        ok = False
        if config.MQTT_ENABLE and mqtt is not None:
            ok = mqtt.publish_warning(payload) if warning else mqtt.publish_telemetry(payload)
        raw_fallback = bool(getattr(config, "G4_RAW_FALLBACK_ENABLE", False))
        if not ok and raw_fallback and g4 is not None:
            ok = g4.send_json(payload)
        if not ok:
            kept.append((payload, warning))
        else:
            sent_count += 1
    cache[:] = kept[-config.COMM_CACHE_MAX:]


def main():
    sensors, buzzer, buttons, g4 = build_hardware()
    manager = SensorManager(sensors)
    manager.init_all()
    if buzzer:
        buzzer.init()
    if g4:
        g4.init()

    comm_enabled = bool(getattr(config, "COMM_UPLOAD_ENABLE", True))
    mqtt = HelmetMQTTClient() if comm_enabled and config.MQTT_ENABLE else None

    status = SystemStatus()
    status.set_network("mqtt" if mqtt and mqtt.connected else "4g")
    packet_builder = DataPacketBuilder(config.DEVICE_ID)
    algorithm = SmartHelmetAlgorithm(sample_rate=max(1, int(1000 / max(1, config.SENSOR_UPDATE_MS))))
    distance = DistanceWarning()
    cache = []
    last_upload = ticks_ms()
    last_heartbeat = ticks_ms()
    last_algorithm_debug = ticks_ms()
    last_cache_flush = ticks_ms()
    last_warning_upload = {}
    boot_ms = ticks_ms()

    print("[MAIN] smart helmet started")
    while True:
        try:
            sensor_data = manager.read_all()
            imu_data = sensor_data.get("imu") or {}
            gps_data = sensor_data.get("gps") or {}
            sht_data = sensor_data.get("sht40") or {}
            jx_data = sensor_data.get("jx90614") or {}
            max_data = sensor_data.get("max30100") or {}
            light_data = sensor_data.get("light") or {}
            usage_minutes = max(0, ticks_diff(ticks_ms(), boot_ms) // 60000)

            algorithm_result = algorithm.update(
                ax=_field(imu_data, "ax"),
                ay=_field(imu_data, "ay"),
                az=_field(imu_data, "az", 1.0),
                gx=_field(imu_data, "gx"),
                gy=_field(imu_data, "gy"),
                gz=_field(imu_data, "gz"),
                gps_speed=_gps_speed_mps(gps_data),
                timestamp=now_s(),
                body_temp=_field(jx_data, "body_temp"),
                heart_rate=_valid_vital(max_data, "hr", "hr_valid"),
                spo2=_valid_vital(max_data, "spo2", "spo2_valid"),
                env_temp=_field(sht_data, "temperature"),
                humidity=_field(sht_data, "humidity"),
                usage_minutes=usage_minutes,
                exposure_minutes=usage_minutes,
                work_minutes=usage_minutes,
                battery_level=status.battery,
                signal_level=status.signal,
                worn=bool(status.helmet_on),
                location_valid=True if not getattr(config, "GNSS_ENABLE", True) else bool(gps_data.get("valid")),
                strong_sun=_strong_sun(light_data),
            )

            status.work_time = usage_minutes
            status.set_status(algorithm_result.get("safety", {}).get("risk_status", "normal"))

            warnings = _algorithm_warnings(algorithm_result)
            warnings.append(distance.update(sensor_data.get("ultrasonic")))

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
            if _algorithm_debug_enabled():
                active_warnings = [item for item in warnings if item and item.get("triggered")]
                now_ms = ticks_ms()
                debug_due = ticks_diff(now_ms, last_algorithm_debug) >= _algorithm_debug_interval_ms()
                if debug_due or active_warnings:
                    _print_algorithm_debug(sensor_data, algorithm_result, active_warnings, force=bool(active_warnings))
                    last_algorithm_debug = now_ms

            if getattr(config, "ALGORITHM_DEBUG_ONLY", False):
                if buzzer:
                    buzzer.tick()
                sleep_ms(config.SENSOR_UPDATE_MS)
                continue

            warning = select_warning(warnings)

            if warning:
                if buzzer:
                    buzzer.alert(_buzzer_level(warning.get("level")))
                if warning_upload_due(warning, last_warning_upload):
                    warning_packet = _warning_packet(config.DEVICE_ID, warning, gps_data, packet_builder)
                    upload_payload(warning_packet, mqtt, g4, cache, warning=True)

            if ticks_diff(ticks_ms(), last_upload) >= config.UPLOAD_INTERVAL_MS:
                if gc:
                    gc.collect()
                telemetry = packet_builder.build_data(sensor_data, status.to_dict(), alerts, algorithm_result)
                upload_payload(telemetry, mqtt, g4, cache, warning=False)
                last_upload = ticks_ms()

            if cache and ticks_diff(ticks_ms(), last_cache_flush) >= getattr(config, "COMM_CACHE_FLUSH_INTERVAL_MS", 10000):
                flush_cache(mqtt, g4, cache)
                last_cache_flush = ticks_ms()

            if ticks_diff(ticks_ms(), last_heartbeat) >= config.HEARTBEAT_INTERVAL_MS:
                heartbeat = packet_builder.build_heartbeat(status.to_dict())
                if mqtt is not None:
                    mqtt.publish_heartbeat(heartbeat)
                last_heartbeat = ticks_ms()

        except Exception as exc:
            print("[MAIN] loop error:", exc)
            if gc:
                gc.collect()
        if buzzer:
            buzzer.tick()
        sleep_ms(config.SENSOR_UPDATE_MS)


if __name__ == "__main__":
    main()
