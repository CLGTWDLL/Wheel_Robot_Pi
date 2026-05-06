import importlib
import math
import re
import struct
from collections.abc import Callable
from typing import Any

commands_module = importlib.import_module("control.damiao_can.commands")
motors_module = importlib.import_module("control.damiao_can.motors")
protocol_module = importlib.import_module("control.damiao_can.protocol")

CanFrame = protocol_module.CanFrame
clear_error_frame = protocol_module.clear_error_frame
decode_feedback = protocol_module.decode_feedback
decode_feedback_frame = protocol_module.decode_feedback_frame
disable_frame = protocol_module.disable_frame
enable_frame = protocol_module.enable_frame
float_to_uint = protocol_module.float_to_uint
standard_disable_frame = protocol_module.standard_disable_frame
get_motor = motors_module.get_motor
group_zero_frames = commands_module.group_zero_frames
mit_frame = protocol_module.mit_frame
parameter_read_frame = protocol_module.parameter_read_frame
parameter_save_frame = protocol_module.parameter_save_frame
parameter_write_frame = protocol_module.parameter_write_frame
torque_frame = protocol_module.torque_frame
uint_to_float = protocol_module.uint_to_float
velocity_frame = protocol_module.velocity_frame
zero_position_frame = protocol_module.zero_position_frame


def assert_raises(error_type: type[BaseException], match: str, function: Callable[..., Any], *args: Any, **kwargs: Any) -> None:
    try:
        function(*args, **kwargs)
    except error_type as error:
        assert re.search(match, str(error)) is not None
    else:
        raise AssertionError(f"expected {error_type.__name__}")


def test_can_frame_rejects_invalid_arbitration_id():
    assert_raises(ValueError, "11-bit", CanFrame, 0x800, b"\x00" * 8)


def test_can_frame_rejects_non_eight_byte_data():
    for data in [b"\x00" * 7, b"\x00" * 9]:
        assert_raises(ValueError, "8 bytes", CanFrame, 0x123, data)


def test_enable_disable_frames_use_standard_command_payloads():
    motor = get_motor("dm4310_left")

    assert enable_frame(motor).arbitration_id == motor.can_id
    assert enable_frame(motor).data == b"\xff\xff\xff\xff\xff\xff\xff\xfc"
    assert disable_frame(motor).data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"


def test_standard_disable_frame_uses_requested_can_id():
    frame = standard_disable_frame(0x10)

    assert frame.arbitration_id == 0x10
    assert frame.data == b"\xff\xff\xff\xff\xff\xff\xff\xfd"


def test_clear_error_frame_uses_standard_command_payload():
    motor = get_motor("dm4310_left")

    assert clear_error_frame(motor).arbitration_id == motor.can_id
    assert clear_error_frame(motor).data == b"\xff\xff\xff\xff\xff\xff\xff\xfb"


def test_zero_position_frame_uses_standard_command_payload():
    motor = get_motor("dm4310_left")

    assert zero_position_frame(motor).arbitration_id == motor.can_id
    assert zero_position_frame(motor).data == b"\xff\xff\xff\xff\xff\xff\xff\xfe"


def test_mit_frame_uses_motor_can_id_and_eight_bytes():
    motor = get_motor("dm4310_left")
    frame = mit_frame(motor, position=0, velocity=0, kp=0, kd=0, torque=0)

    assert frame.arbitration_id == motor.can_id
    assert len(frame.data) == 8


def test_mit_frame_rejects_non_finite_values():
    for field in ["position", "velocity", "kp", "kd", "torque"]:
        for value in [math.nan, math.inf, -math.inf]:
            kwargs = {field: value}
            assert_raises(ValueError, "finite", mit_frame, get_motor("dm4310_left"), **kwargs)


def test_velocity_frame_uses_0x200_plus_motor_id_and_little_endian_float():
    motor = get_motor("dmh6215_left")
    frame = velocity_frame(motor, velocity=1.25)

    assert frame.arbitration_id == 0x200 + motor.can_id
    assert frame.data == struct.pack("<f", 1.25) + b"\x00\x00\x00\x00"


def test_velocity_frame_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        assert_raises(ValueError, "finite", velocity_frame, get_motor("dmh6215_left"), velocity=value)


def test_velocity_frame_rejects_motor_velocity_limit_excess():
    motor = get_motor("dmh6215_left")

    assert_raises(ValueError, "velocity command", velocity_frame, motor, velocity=motor.v_max + 0.1)


def test_torque_frame_uses_0x600_plus_motor_id_and_little_endian_float():
    motor = get_motor("dmh6215_left")
    frame = torque_frame(motor, 0.02)

    assert frame.arbitration_id == 0x600 + motor.can_id
    assert frame.data == struct.pack("<f", 0.02) + b"\x00\x00\x00\x00"


def test_torque_frame_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        assert_raises(ValueError, "finite", torque_frame, get_motor("dmh6215_left"), value)


def test_torque_frame_rejects_motor_torque_limit_excess():
    motor = get_motor("dmh6215_left")
    assert_raises(ValueError, "torque command", torque_frame, motor, motor.t_max + 0.1)


def test_decode_feedback_decodes_packed_payload():
    motor = get_motor("dm4310_left")
    data = bytes([0xA2, 0x12, 0x34, 0x56, 0x78, 0x9A, 60, 61])

    feedback = decode_feedback(data, motor)

    assert feedback.motor_id == 0x02
    assert feedback.state == 0x0A
    assert feedback.position_raw == 0x1234
    assert feedback.velocity_raw == 0x567
    assert feedback.torque_raw == 0x89A
    assert feedback.position == uint_to_float(0x1234, -motor.p_max, motor.p_max, 16)
    assert feedback.velocity == uint_to_float(0x567, -motor.v_max, motor.v_max, 12)
    assert feedback.torque == uint_to_float(0x89A, -motor.t_max, motor.t_max, 12)
    assert feedback.mos_temperature == 60
    assert feedback.rotor_temperature == 61


def test_decode_feedback_rejects_non_eight_byte_payload():
    assert_raises(ValueError, "8 bytes", decode_feedback, b"\x00" * 7, get_motor("dm4310_left"))


def test_decode_feedback_frame_routes_by_master_id():
    motor = get_motor("dm4310_left")
    frame = CanFrame(motor.master_id, bytes([motor.can_id, 0, 0, 0, 0, 0, 0, 0]))

    routed = decode_feedback_frame(frame)

    assert routed.motor == motor
    assert routed.feedback.motor_id == motor.can_id


def test_decode_feedback_frame_rejects_unknown_master_id():
    frame = CanFrame(0x111, b"\x00" * 8)

    assert_raises(KeyError, "unknown master_id", decode_feedback_frame, frame)


def test_decode_feedback_frame_rejects_payload_motor_id_mismatch():
    motor = get_motor("dm4310_left")
    frame = CanFrame(motor.master_id, bytes([0x03, 0, 0, 0, 0, 0, 0, 0]))

    assert_raises(ValueError, "does not match", decode_feedback_frame, frame)


def test_parameter_read_frame_uses_global_parameter_id():
    motor = get_motor("dm4310_left")
    frame = parameter_read_frame(motor, register_id=0x10)

    assert frame.arbitration_id == 0x7FF
    assert frame.data == bytes([motor.can_id, 0x00, 0x33, 0x10, 0x00, 0x00, 0x00, 0x00])


def test_parameter_write_frame_uses_global_parameter_id_and_value():
    motor = get_motor("dm4310_left")
    frame = parameter_write_frame(motor, register_id=0x10, value=b"\x01\x02\x03\x04")

    assert frame.arbitration_id == 0x7FF
    assert frame.data == bytes([motor.can_id, 0x00, 0x55, 0x10, 0x01, 0x02, 0x03, 0x04])


def test_parameter_write_frame_rejects_non_four_byte_value():
    for value in [b"\x00" * 3, b"\x00" * 5]:
        assert_raises(ValueError, "4 bytes", parameter_write_frame, get_motor("dm4310_left"), register_id=0x10, value=value)


def test_parameter_save_frame_uses_global_parameter_id():
    motor = get_motor("dm4310_left")
    frame = parameter_save_frame(motor)

    assert frame.arbitration_id == 0x7FF
    assert frame.data == bytes([motor.can_id, 0x00, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00])


def test_float_to_uint_clamps_to_range():
    assert float_to_uint(-100.0, -1.0, 1.0, 8) == 0
    assert float_to_uint(100.0, -1.0, 1.0, 8) == 255


def test_float_to_uint_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        assert_raises(ValueError, "finite", float_to_uint, value, -1.0, 1.0, 8)


def test_uint_to_float_maps_endpoints():
    assert uint_to_float(0, -1.0, 1.0, 8) == -1.0
    assert uint_to_float(255, -1.0, 1.0, 8) == 1.0


def test_group_zero_order_and_no_zero_position_command():
    frames = group_zero_frames()

    assert [frame.arbitration_id for frame in frames] == [0x02, 0x03, 0x204, 0x205]
    assert all(frame.data[-1] != 0xFE for frame in frames)
    assert all(frame.data != b"\xff\xff\xff\xff\xff\xff\xff\xfe" for frame in frames)
