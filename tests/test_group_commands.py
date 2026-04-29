import struct

import pytest

from control.damiao_can.commands import run_group_conservative_test, run_group_zero, run_torque_test
from control.fake_bus import FakeCanBus


def test_run_group_zero_sends_only_zero_output_frames_without_0xfe():
    bus = FakeCanBus()

    run_group_zero(bus, duration=0.001, period=0.001)

    assert bus.sent_frames
    assert all(frame.data != b"\xff\xff\xff\xff\xff\xff\xff\xfe" for frame in bus.sent_frames)
    assert {frame.arbitration_id for frame in bus.sent_frames} == {0x02, 0x03, 0x604, 0x605}


def test_group_conservative_test_has_pre_zero_nonzero_and_finally_zero():
    bus = FakeCanBus()

    run_group_conservative_test(bus, duration=0.001, period=0.001)

    assert [frame.arbitration_id for frame in bus.sent_frames[:4]] == [0x02, 0x03, 0x604, 0x605]
    assert any(frame.arbitration_id in {0x604, 0x605} and frame.data[:4] != b"\x00\x00\x00\x00" for frame in bus.sent_frames)
    assert [frame.arbitration_id for frame in bus.sent_frames[-4:]] == [0x02, 0x03, 0x604, 0x605]


def test_group_conservative_test_sends_four_frames_each_cycle_in_group_order():
    bus = FakeCanBus()

    run_group_conservative_test(bus, duration=0.1, period=0.05)

    cycle_batches = [bus.sent_frames[index : index + 4] for index in range(4, len(bus.sent_frames) - 4, 4)]
    assert len(cycle_batches) >= 2
    for batch in cycle_batches:
        assert [frame.arbitration_id for frame in batch] == [0x02, 0x03, 0x604, 0x605]
        assert struct.unpack("<f", batch[2].data[:4])[0] != 0.0
        assert struct.unpack("<f", batch[3].data[:4])[0] != 0.0


def test_torque_test_sequence_includes_pre_zero_enable_nonzero_final_zero_disable():
    bus = FakeCanBus()

    run_torque_test(bus, "dmh6215_left", duration=0.001, period=0.001)

    assert bus.sent_frames[0].arbitration_id == 0x604
    assert struct.unpack("<f", bus.sent_frames[0].data[:4])[0] == 0.0
    assert bus.sent_frames[1].arbitration_id == 0x04
    assert bus.sent_frames[1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfc"
    assert any(
        frame.arbitration_id == 0x604 and struct.unpack("<f", frame.data[:4])[0] != 0.0
        for frame in bus.sent_frames[2:-2]
    )
    assert bus.sent_frames[-2].arbitration_id == 0x604
    assert struct.unpack("<f", bus.sent_frames[-2].data[:4])[0] == 0.0
    assert bus.sent_frames[-1].arbitration_id == 0x04
    assert bus.sent_frames[-1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"


def test_torque_test_sends_zero_and_disable_in_finally_when_bus_send_fails():
    class FailingAfterNonzeroSend(FakeCanBus):
        def send(self, frame):
            super().send(frame)
            if len(self.sent_frames) == 3:
                raise RuntimeError("boom")

    bus = FailingAfterNonzeroSend()

    with pytest.raises(RuntimeError, match="boom"):
        run_torque_test(bus, "dmh6215_left", duration=0.01, period=0.001)

    assert bus.sent_frames[-2].arbitration_id == 0x604
    assert struct.unpack("<f", bus.sent_frames[-2].data[:4])[0] == 0.0
    assert bus.sent_frames[-1].arbitration_id == 0x04
    assert bus.sent_frames[-1].data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"


def test_torque_test_rejects_mit_motor():
    with pytest.raises(ValueError, match="not torque"):
        run_torque_test(FakeCanBus(), "dm4310_left", duration=0.01)
