from control.bus import ReceivedFrame
from control.damiao_can.protocol import CanFrame
from control.fake_bus import FakeCanBus


def test_fake_bus_records_sent_frames():
    bus = FakeCanBus()
    frame = CanFrame(0x123, b"\x00" * 8)

    bus.send(frame)

    assert bus.sent_frames == [frame]


def test_fake_bus_returns_injected_frames_then_timeout_none():
    frame = CanFrame(0x123, b"\x01" * 8)
    bus = FakeCanBus([frame])

    received = bus.recv(timeout=0.001)

    assert received == ReceivedFrame(frame=frame)
    assert bus.recv(timeout=0.001) is None
