import pytest

from control.damiao_can.protocol import CanFrame
from control.slcan import decode_slcan_line, encode_slcan_frame


def test_encode_standard_slcan_frame():
    frame = CanFrame(0x123, b"\x01\x02\x03\x04\x05\x06\x07\x08")

    assert encode_slcan_frame(frame) == "t12380102030405060708\r"


def test_decode_standard_slcan_frame_pads_to_can_frame_length():
    frame = decode_slcan_line("t12320102\r")

    assert frame.arbitration_id == 0x123
    assert frame.data == b"\x01\x02" + b"\x00" * 6


@pytest.mark.parametrize("line", ["", "T1230", "t12", "t123901", "t123201", "tXYZ0"])
def test_decode_rejects_invalid_lines(line):
    with pytest.raises(ValueError):
        decode_slcan_line(line)
