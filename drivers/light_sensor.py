"""Compatibility wrappers for light sensors."""

from drivers.gy302 import GY302


class ADCLightSensor:
    """Legacy ADC light input used when the GY302 module is disabled."""

    def __init__(self, adc=None):
        self.adc = adc
        self._available = adc is not None

    def init(self):
        return self._available

    def check(self):
        return self._available

    def read(self):
        if not self._available:
            return {"ok": False, "sensor": "light", "data": None, "error": "light_not_available"}
        try:
            raw = self.adc.read()
            return {"ok": True, "sensor": "light", "data": {"light": raw, "raw": raw}, "error": None}
        except Exception as exc:
            print("[Light] read failed:", exc)
            return {"ok": False, "sensor": "light", "data": None, "error": "light_read_failed"}


LightSensor = GY302
