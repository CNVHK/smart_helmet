"""光照传感器驱动，默认使用 ADC 原始值。"""


class LightSensor:
    """ADC 光照采集，若后续更换 I2C 光照芯片可保持 read 接口不变。"""

    def __init__(self, adc=None):
        """传入 machine.ADC 对象；为空时进入降级模式。"""
        self.adc = adc
        self._available = adc is not None

    def init(self):
        """初始化 ADC 光照传感器。"""
        return self._available

    def check(self):
        """返回在线状态。"""
        return self._available

    def read(self):
        """读取 raw 光照值。"""
        if not self._available:
            return {"ok": False, "sensor": "light", "data": None, "error": "light_not_available"}
        try:
            raw = self.adc.read()
            return {"ok": True, "sensor": "light", "data": {"light": raw}, "error": None}
        except Exception as exc:
            print("[Light] read failed:", exc)
            return {"ok": False, "sensor": "light", "data": None, "error": "light_read_failed"}

