"""简单数据滤波器。"""


class MovingAverage:
    """固定窗口滑动平均滤波器。"""

    def __init__(self, size=5):
        """创建指定窗口长度的滤波器。"""
        self.size = size
        self.values = []

    def update(self, value):
        """加入新值并返回平均值；None 会被忽略。"""
        if value is None:
            return None
        self.values.append(value)
        if len(self.values) > self.size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)


class SensorFilterBank:
    """按字段名管理多个滑动平均滤波器。"""

    def __init__(self, size=5):
        """创建滤波器组。"""
        self.size = size
        self.filters = {}

    def update_dict(self, prefix, data):
        """对字典中的数值字段做滑动平均。"""
        if not data:
            return data
        result = {}
        for key, value in data.items():
            if isinstance(value, (int, float)):
                name = prefix + "." + key
                if name not in self.filters:
                    self.filters[name] = MovingAverage(self.size)
                result[key] = round(self.filters[name].update(value), 3)
            else:
                result[key] = value
        return result

