"""MS60 BSD radar parser smoke tests."""

import sys

sys.path.append(".")
sys.path.append("..")
sys.path.append("/flash/smart_helmet")

from drivers.ms60_bsd_radar import MS60BSDRadar


class FakeUART:
    def __init__(self, chunks=None):
        self.chunks = list(chunks or [])
        self.written = []

    def any(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, count=None):
        if not self.chunks:
            return b""
        return self.chunks.pop(0)

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)


def test_bsd_report_parser():
    # obj_num=2, reserved=0, objects: (8m, -15deg, -3m/s, id1), (3m, 20deg, 1m/s, id2)
    payload = [2, 0, 0, 0, 8, 0xF1, 0xFD, 1, 3, 20, 1, 2]
    frame = MS60BSDRadar.build_report(MS60BSDRadar.TYPE_BSD, payload)
    radar = MS60BSDRadar()
    parsed = radar.feed(frame[:4])
    assert parsed == []
    parsed = radar.feed(frame[4:])
    assert parsed[-1]["type"] == "report"
    data = radar.last_data
    assert data["detected"] is True
    assert data["obj_num"] == 2
    assert data["objects"][0]["angle_deg"] == -15
    assert data["objects"][0]["velocity_mps"] == -3
    assert data["nearest_m"] == 3
    assert data["front_cm"] == 300


def test_command_checksum():
    assert MS60BSDRadar.build_command(0x05) == bytes([0x58, 0x05, 0x00, 0x5D, 0x00])
    assert MS60BSDRadar.build_command(0x19, [0x00, 0xC2, 0x01, 0x00]) == bytes(
        [0x58, 0x19, 0x04, 0x00, 0xC2, 0x01, 0x00, 0x38, 0x01]
    )


def test_read_from_uart_and_request():
    frame = MS60BSDRadar.build_report(MS60BSDRadar.TYPE_BSD, [1, 0, 0, 0, 5, 10, 0, 7])
    uart = FakeUART([frame])
    radar = MS60BSDRadar(uart, request_on_read=True)
    result = radar.read()
    assert result["ok"] is True
    assert result["data"]["nearest_m"] == 5
    assert uart.written[0] == MS60BSDRadar.build_command(MS60BSDRadar.CMD_GET_DETECTION)


if __name__ == "__main__":
    test_bsd_report_parser()
    test_command_checksum()
    test_read_from_uart_and_request()
    print("ms60_bsd_radar tests ok")
