"""Non-blocking active buzzer driver."""

import time


def _ticks_ms():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _ticks_diff(now, old):
    return time.ticks_diff(now, old) if hasattr(time, "ticks_diff") else now - old


class Buzzer:
    """Active buzzer controlled by warning level."""

    def __init__(self, pin=None):
        self.pin = pin
        self._available = pin is not None
        self._mode = None
        self._beeps_left = 0
        self._on_ms = 0
        self._off_ms = 0
        self._next_ms = 0
        self._phase_on = False
        self._hold_until_ms = 0

    def init(self):
        self.off()
        return self._available

    def check(self):
        return self._available

    def on(self):
        if self._available:
            self.pin.value(1)

    def off(self):
        if self._available:
            self.pin.value(0)
        self._mode = None
        self._beeps_left = 0
        self._phase_on = False
        self._hold_until_ms = 0

    def beep(self, count=1, on_ms=120, off_ms=120):
        self._start_pattern("beep", count, on_ms, off_ms)

    def alert(self, level):
        if level == "suspected":
            self._start_pattern(level, 1, 120, 120)
        elif level == "light":
            self._start_pattern(level, 2, 120, 120)
        elif level == "medium":
            self._start_pattern(level, 5, 80, 80)
        elif level == "severe":
            self._mode = "severe"
            self._hold_until_ms = _ticks_ms() + 10000
            self.on()
        elif level == "sos":
            self._mode = "sos"
            self.on()

    def tick(self):
        """Advance the buzzer state without blocking the main loop."""
        now = _ticks_ms()

        if self._mode == "sos":
            self.on()
            return

        if self._mode == "severe":
            if _ticks_diff(now, self._hold_until_ms) >= 0:
                self.off()
            return

        if self._mode is None or _ticks_diff(now, self._next_ms) < 0:
            return

        if self._phase_on:
            if self._available:
                self.pin.value(0)
            self._beeps_left -= 1
            self._phase_on = False
            if self._beeps_left <= 0:
                self._mode = None
                return
            self._next_ms = now + self._off_ms
        else:
            self.on()
            self._phase_on = True
            self._next_ms = now + self._on_ms

    def _start_pattern(self, mode, count, on_ms, off_ms):
        if self._mode == "sos":
            return
        if self._mode == mode and self._beeps_left > 0:
            return
        self._mode = mode
        self._beeps_left = max(1, int(count))
        self._on_ms = max(1, int(on_ms))
        self._off_ms = max(1, int(off_ms))
        self._phase_on = True
        self._next_ms = _ticks_ms() + self._on_ms
        self.on()
