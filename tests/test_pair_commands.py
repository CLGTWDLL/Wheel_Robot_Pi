import importlib
import struct
from contextlib import contextmanager
from typing import Any

from control.bus import ReceivedFrame

commands = importlib.import_module("control.damiao_can.commands")
fake_bus = importlib.import_module("control.fake_bus")

pair_zero_frames = commands.pair_zero_frames
run_pair_conservative_spin = commands.run_pair_conservative_spin
FakeCanBus = fake_bus.FakeCanBus


@contextmanager
def assert_raises(expected: type[BaseException], match: str | None = None):
    try:
        yield
    except expected as exc:
        if match is not None:
            assert match in str(exc)
    else:
        raise AssertionError(f"expected {expected.__name__}")


def assert_approx(value: float, expected: float, tolerance: float = 1e-6) -> None:
    assert abs(value - expected) <= tolerance


def test_pair_zero_frames_only_targets_0x02_and_0x204_without_0xfe():
    frames = pair_zero_frames()

    assert [frame.arbitration_id for frame in frames] == [0x02, 0x204]
    assert all(frame.arbitration_id != 0x604 for frame in frames)
    assert all(frame.data != b"\xff\xff\xff\xff\xff\xff\xff\xfe" for frame in frames)
    assert struct.unpack("<f", frames[1].data[:4])[0] == 0.0


def test_pair_conservative_spin_sequence_and_cycle_targets():
    bus = FakeCanBus()

    run_pair_conservative_spin(bus, duration=0.1, period=0.05, velocity=0.2, wheel_velocity=0.1)

    ids = [frame.arbitration_id for frame in bus.sent_frames]
    assert 0x03 not in ids
    assert 0x05 not in ids
    assert 0x604 not in ids
    assert 0x605 not in ids
    assert ids[:4] == [0x02, 0x204, 0x02, 0x04]
    assert bus.sent_frames[2].data == b"\xff\xff\xff\xff\xff\xff\xff\xfc"
    assert bus.sent_frames[3].data == b"\xff\xff\xff\xff\xff\xff\xff\xfc"
    assert ids[-4:] == [0x02, 0x204, 0x02, 0x04]
    assert struct.unpack("<f", bus.sent_frames[-3].data[:4])[0] == 0.0
    assert bus.sent_frames[-2].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"
    assert bus.sent_frames[-1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"

    cycle_batches = [bus.sent_frames[index : index + 2] for index in range(4, len(bus.sent_frames) - 4, 2)]
    assert len(cycle_batches) >= 2
    for batch in cycle_batches:
        assert [frame.arbitration_id for frame in batch] == [0x02, 0x204]
        assert_approx(struct.unpack("<f", batch[1].data[:4])[0], 0.1)


def test_pair_conservative_spin_applies_mit_torque_feedforward():
    bus = FakeCanBus()

    run_pair_conservative_spin(bus, duration=0.1, period=0.05, velocity=0.2, wheel_velocity=0.1, torque=0.2)

    cycle_batches = [bus.sent_frames[index : index + 2] for index in range(4, len(bus.sent_frames) - 4, 2)]
    assert len(cycle_batches) >= 2
    for batch in cycle_batches:
        mit_frame = batch[0]
        t_uint = ((mit_frame.data[6] & 0x0F) << 8) | mit_frame.data[7]
        decoded_torque = t_uint * 20.0 / 4095.0 - 10.0
        assert_approx(decoded_torque, 0.2, tolerance=0.01)


def test_pair_conservative_spin_sends_zero_and_disables_when_bus_send_fails():
    class FailingAfterFirstCycleSend(FakeCanBus):
        def send(self, frame: Any) -> None:
            super().send(frame)
            if len(self.sent_frames) == 5:
                raise RuntimeError("boom")

    bus = FailingAfterFirstCycleSend()

    with assert_raises(RuntimeError, match="boom"):
        run_pair_conservative_spin(bus, duration=0.1, period=0.05)

    assert [frame.arbitration_id for frame in bus.sent_frames[-4:]] == [0x02, 0x204, 0x02, 0x04]
    assert struct.unpack("<f", bus.sent_frames[-3].data[:4])[0] == 0.0
    assert bus.sent_frames[-2].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"
    assert bus.sent_frames[-1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"


def test_pair_conservative_spin_telemetry_continuous_rx_still_safely_finishes(monkeypatch):
    protocol_module = importlib.import_module("control.damiao_can.protocol")

    monotonic_values = iter([0.0, 0.0, 0.0002, 0.0004, 0.0006, 0.0008, 0.001, 0.0012])

    def fake_monotonic() -> float:
        try:
            return next(monotonic_values)
        except StopIteration:
            return 0.0012

    sleeps: list[float] = []

    def fake_sleep(duration: float) -> None:
        sleeps.append(duration)

    class ContinuousRxBus(FakeCanBus):
        def __init__(self) -> None:
            super().__init__()
            self.recv_calls: list[float | None] = []

        def recv(self, timeout: float | None = None) -> ReceivedFrame:
            self.recv_calls.append(timeout)
            return ReceivedFrame(protocol_module.CanFrame(0x14, b"\x04\x80\x00\x80\x07\xFF\x22\x23"))

    bus = ContinuousRxBus()
    telemetry_frames: list[ReceivedFrame] = []
    monkeypatch.setattr(commands.time, "monotonic", fake_monotonic)
    monkeypatch.setattr(commands.time, "sleep", fake_sleep)

    run_pair_conservative_spin(
        bus,
        duration=0.001,
        period=0.001,
        velocity=0.2,
        wheel_velocity=0.1,
        telemetry=telemetry_frames.append,
    )

    assert len(telemetry_frames) == 1
    assert bus.recv_calls == [0.0008, 0]
    assert sleeps == []
    assert [frame.arbitration_id for frame in bus.sent_frames[-4:]] == [0x02, 0x204, 0x02, 0x04]
    assert struct.unpack("<f", bus.sent_frames[-3].data[:4])[0] == 0.0
    assert bus.sent_frames[-2].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"
    assert bus.sent_frames[-1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"
