import struct

import pytest

from control.damiao_can.commands import run_group_conservative_test, run_group_zero, run_torque_test
from control.fake_bus import FakeCanBus


def test_run_group_zero_sends_only_zero_output_frames_without_0xfe():
    bus = FakeCanBus()

    run_group_zero(bus, duration=0.001, period=0.001)

    assert bus.sent_frames
    assert all(frame.data != b"\xff\xff\xff\xff\xff\xff\xff\xfe" for frame in bus.sent_frames)
    assert {frame.arbitration_id for frame in bus.sent_frames} == {0x02, 0x03, 0x204, 0x205}


def test_group_conservative_test_has_pre_zero_nonzero_and_finally_zero():
    bus = FakeCanBus()

    run_group_conservative_test(bus, duration=0.001, period=0.001)

    assert [frame.arbitration_id for frame in bus.sent_frames[:4]] == [0x02, 0x03, 0x204, 0x205]
    assert all(frame.arbitration_id not in {0x604, 0x605} for frame in bus.sent_frames)
    assert all(frame.data[:4] == b"\x00\x00\x00\x00" for frame in bus.sent_frames if frame.arbitration_id in {0x204, 0x205})
    assert [frame.arbitration_id for frame in bus.sent_frames[-4:]] == [0x02, 0x03, 0x204, 0x205]


def test_group_conservative_test_sends_four_frames_each_cycle_in_group_order():
    bus = FakeCanBus()

    run_group_conservative_test(bus, duration=0.1, period=0.05)

    cycle_batches = [bus.sent_frames[index : index + 4] for index in range(4, len(bus.sent_frames) - 4, 4)]
    assert len(cycle_batches) >= 2
    for batch in cycle_batches:
        assert [frame.arbitration_id for frame in batch] == [0x02, 0x03, 0x204, 0x205]
        assert struct.unpack("<f", batch[2].data[:4])[0] == 0.0
        assert struct.unpack("<f", batch[3].data[:4])[0] == 0.0


def test_torque_test_rejects_non_torque_motors():
    with pytest.raises(ValueError, match="not torque"):
        run_torque_test(FakeCanBus(), "dm4310_left", duration=0.01)
    with pytest.raises(ValueError, match="not torque"):
        run_torque_test(FakeCanBus(), "dmh6215_left", duration=0.01)
