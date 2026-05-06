from __future__ import annotations

import importlib

import pytest
import serial

from control.damiao_can.protocol import CanFrame
from control.damiao_usbcan import (
    DEFAULT_CAN_BITRATE,
    DEFAULT_SERIAL_BAUD,
    RECV_CMD_CAN_SUCCESS,
    RECV_FRAME_LEN,
    SEND_CMD_FORWARD_NO_STATUS,
    SEND_FRAME_LEN,
    DamiaoUsbCanBus,
    decode_damiao_usbcan_packet,
    encode_damiao_usbcan_frame,
)


class FakeSerial:
    instances: list["FakeSerial"] = []
    chunks: list[bytes] = []

    def __init__(self, *, port, baudrate, timeout, write_timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.writes: list[bytes] = []
        self.flushed = False
        self.is_open = True
        self.read_calls = 0
        FakeSerial.instances.append(self)

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        self.flushed = True

    def read(self, size: int) -> bytes:
        self.read_calls += 1
        if not FakeSerial.chunks:
            return b""
        chunk = FakeSerial.chunks.pop(0)
        if len(chunk) > size:
            FakeSerial.chunks.insert(0, chunk[size:])
            return chunk[:size]
        return chunk

    def close(self) -> None:
        self.is_open = False


@pytest.fixture(autouse=True)
def reset_fake_serial(monkeypatch):
    FakeSerial.instances = []
    FakeSerial.chunks = []
    monkeypatch.setattr("control.damiao_usbcan.serial.Serial", FakeSerial)


def _packet(can_id: int = 0x123, data: bytes = b"\x01\x02\x03\x04\x05\x06\x07\x08", *, flags: int = 8) -> bytes:
    packet = bytearray(RECV_FRAME_LEN)
    packet[0] = 0xAA
    packet[1] = RECV_CMD_CAN_SUCCESS
    packet[2] = flags
    packet[3:7] = can_id.to_bytes(4, "little")
    packet[7:15] = data
    packet[15] = 0x55
    return bytes(packet)


def test_encode_damiao_usbcan_frame_offsets():
    frame = CanFrame(0x123, b"\x01\x02\x03\x04\x05\x06\x07\x08")

    encoded = encode_damiao_usbcan_frame(frame)

    assert len(encoded) == SEND_FRAME_LEN
    assert encoded[0:2] == b"\x55\xaa"
    assert encoded[2] == SEND_FRAME_LEN
    assert encoded[3] == SEND_CMD_FORWARD_NO_STATUS
    assert encoded[4:8] == (1).to_bytes(4, "little")
    assert encoded[8:12] == (10).to_bytes(4, "little")
    assert encoded[12] == 0
    assert encoded[13:17] == (0x123).to_bytes(4, "little")
    assert encoded[17] == 0
    assert encoded[18] == 8
    assert encoded[19] == 0
    assert encoded[20] == 0
    assert encoded[21:29] == frame.data
    assert encoded[29] == 0


def test_encode_damiao_usbcan_frame_golden_hex():
    frame = CanFrame(0x604, b"\x0a\x0b\x0c\x0d\x0e\x0f\x10\x11")

    encoded = encode_damiao_usbcan_frame(frame)

    assert encoded.hex() == "55aa1e03010000000a0000000004060000000800000a0b0c0d0e0f101100"


def test_decode_damiao_usbcan_packet():
    frame = decode_damiao_usbcan_packet(_packet(0x456, b"\x08\x07\x06\x05\x04\x03\x02\x01"))

    assert frame.arbitration_id == 0x456
    assert frame.data == b"\x08\x07\x06\x05\x04\x03\x02\x01"


@pytest.mark.parametrize(
    "packet",
    [
        b"",
        bytes([0xAB, RECV_CMD_CAN_SUCCESS, 8]) + b"\x00" * 13,
        bytes([0xAA, 0x10, 8]) + b"\x00" * 12 + b"\x55",
        _packet(flags=7),
        _packet(flags=0x48),
        _packet(flags=0x88),
        _packet(can_id=0x800),
        _packet()[:-1] + b"\x54",
    ],
)
def test_decode_rejects_unsupported_packets(packet):
    with pytest.raises(ValueError):
        decode_damiao_usbcan_packet(packet)


def test_bus_send_writes_encoded_bytes_and_flushes():
    bus = DamiaoUsbCanBus("/dev/fake")
    frame = CanFrame(0x123, b"\x01\x02\x03\x04\x05\x06\x07\x08")

    bus.send(frame)

    serial_port = FakeSerial.instances[0]
    assert bus.config.can_bitrate == DEFAULT_CAN_BITRATE
    assert serial_port.port == "/dev/fake"
    assert serial_port.baudrate == DEFAULT_SERIAL_BAUD
    assert serial_port.writes == [encode_damiao_usbcan_frame(frame)]
    assert serial_port.flushed is True


def test_recv_handles_noise_half_frame_and_consecutive_frames():
    first = _packet(0x111, b"abcdefgh")
    second = _packet(0x222, b"ABCDEFGH")
    FakeSerial.chunks = [b"noise" + first[:5], first[5:] + second]
    bus = DamiaoUsbCanBus("/dev/fake")

    received_first = bus.recv(timeout=0.1)
    received_second = bus.recv(timeout=0.1)

    assert received_first is not None
    assert received_first.frame == CanFrame(0x111, b"abcdefgh")
    assert received_first.timestamp is not None
    assert received_second is not None
    assert received_second.frame == CanFrame(0x222, b"ABCDEFGH")


def test_recv_bad_tail_resyncs_to_next_frame():
    bad = _packet(0x111, b"badframe")[:-1] + b"\x54"
    good = _packet(0x222, b"gooddata")
    FakeSerial.chunks = [bad + good]
    bus = DamiaoUsbCanBus("/dev/fake")

    received = bus.recv(timeout=0.1)

    assert received is not None
    assert received.frame == CanFrame(0x222, b"gooddata")


def test_recv_invalid_cmd_resyncs_to_next_frame():
    bad = bytearray(_packet(0x111, b"badframe"))
    bad[1] = 0x10
    good = _packet(0x222, b"gooddata")
    FakeSerial.chunks = [bytes(bad) + good]
    bus = DamiaoUsbCanBus("/dev/fake")

    received = bus.recv(timeout=0.1)

    assert received is not None
    assert received.frame == CanFrame(0x222, b"gooddata")


def test_recv_invalid_dlc_resyncs_to_next_frame():
    bad = _packet(0x111, b"badframe", flags=7)
    good = _packet(0x222, b"gooddata")
    FakeSerial.chunks = [bad + good]
    bus = DamiaoUsbCanBus("/dev/fake")

    received = bus.recv(timeout=0.1)

    assert received is not None
    assert received.frame == CanFrame(0x222, b"gooddata")


def test_recv_discards_unsupported_frames_until_timeout():
    FakeSerial.chunks = [_packet(flags=0x48)]
    bus = DamiaoUsbCanBus("/dev/fake")

    assert bus.recv(timeout=0.0) is None


def test_recv_timeout_returns_none():
    bus = DamiaoUsbCanBus("/dev/fake")

    assert bus.recv(timeout=0.0) is None


def test_unsupported_can_bitrate_rejected_before_open():
    with pytest.raises(ValueError, match="does not modify device CAN bitrate"):
        DamiaoUsbCanBus("/dev/fake", can_bitrate=500000)

    assert FakeSerial.instances == []


def test_close_closes_serial_port():
    bus = DamiaoUsbCanBus("/dev/fake")
    serial_port = FakeSerial.instances[0]

    bus.close()

    assert serial_port.is_open is False


def test_open_failure_is_wrapped(monkeypatch):
    def fail_open(**kwargs):
        assert kwargs["port"] == "/dev/missing"
        assert kwargs["baudrate"] == DEFAULT_SERIAL_BAUD
        raise serial.SerialException("missing")

    monkeypatch.setattr("control.damiao_usbcan.serial.Serial", fail_open)

    with pytest.raises(RuntimeError, match="failed to open Damiao USB-CAN port /dev/missing"):
        DamiaoUsbCanBus("/dev/missing")


def test_list_serial_ports_uses_cu_and_tty_globs(monkeypatch):
    module = importlib.import_module("control.damiao_usbcan")

    def fake_glob(pattern):
        return {
            "/dev/cu.*": ["/dev/cu.one", "/dev/cu.shared"],
            "/dev/tty.*": ["/dev/tty.two", "/dev/cu.shared"],
        }[pattern]

    monkeypatch.setattr(module.glob, "glob", fake_glob)

    assert module.list_serial_ports() == ["/dev/cu.one", "/dev/cu.shared", "/dev/tty.two"]
