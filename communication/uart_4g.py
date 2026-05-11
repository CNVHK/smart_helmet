"""4G 模块 UART 通信。"""

import time
from communication.protocol import ensure_bytes


def _sleep_ms(ms):
    """毫秒延时兼容函数。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def _ticks_ms():
    """毫秒计时兼容函数。"""
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _ticks_diff(now, old):
    """计时差兼容函数。"""
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


class G4Module:
    """4G 模块 AT 指令和透明传输接口。"""

    def __init__(self, uart=None, retry=3):
        """保存 UART 对象和重试次数。"""
        self.uart = uart
        self.retry = retry
        self._available = uart is not None

    def init(self):
        """发送 AT 探测模块。"""
        return self.send_at("AT").find("OK") >= 0 if self._available else False

    def send_at(self, cmd, timeout=1000):
        """发送 AT 指令并读取响应。"""
        if not self._available:
            return ""
        try:
            line = cmd if cmd.endswith("\r\n") else cmd + "\r\n"
            self.uart.write(line)
            return self._read_response(timeout)
        except Exception as exc:
            print("[4G] AT failed:", exc)
            return ""

    def send_json(self, payload):
        """通过透明传输发送 JSON 字符串，带重试机制。"""
        data = ensure_bytes(payload)
        if data is None or not self._available:
            return False
        for _ in range(self.retry):
            try:
                self.uart.write(data + b"\r\n")
                return True
            except Exception as exc:
                print("[4G] send json failed:", exc)
                _sleep_ms(200)
        return False

    def check_network(self):
        """网络检查占位，当前用 AT+CREG? 简单探测。"""
        resp = self.send_at("AT+CREG?", 1000)
        return "+CREG:" in resp and (",1" in resp or ",5" in resp)

    def _read_response(self, timeout):
        start = _ticks_ms()
        chunks = []
        while _ticks_diff(_ticks_ms(), start) < timeout:
            try:
                data = self.uart.read()
                if data:
                    chunks.append(data)
            except Exception:
                break
            _sleep_ms(20)
        try:
            return b"".join(chunks).decode("ascii", "ignore")
        except Exception:
            return ""

