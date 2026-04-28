from __future__ import annotations

import struct

import pytest

from damiao_can.motors import MOTORS, get_motor, get_motor_by_master_id
from damiao_can.protocol import (
    CanFrame,
    clear_error_frame,
    decode_feedback,
    decode_feedback_frame,
    disable_frame,
    enable_frame,
    float_to_uint,
    mit_frame,
    parameter_read_frame,
    parameter_save_frame,
    parameter_write_frame,
    uint_to_float,
    velocity_frame,
    zero_position_frame,
)


def test_motor_registry_has_four_configs_and_lookup_by_name_and_master_id() -> None:
    assert [motor.name for motor in MOTORS] == [
        "dm4310_left",
        "dm4310_right",
        "dmh6215_left",
        "dmh6215_right",
    ]

    expected = {
        "dm4310_left": (0x02, 0x12, "mit", 30.0),
        "dm4310_right": (0x03, 0x13, "mit", 30.0),
        "dmh6215_left": (0x04, 0x14, "velocity", 45.0),
        "dmh6215_right": (0x05, 0x15, "velocity", 45.0),
    }
    for name, (can_id, master_id, mode, v_max) in expected.items():
        motor = get_motor(name)
        assert motor.can_id == can_id
        assert motor.master_id == master_id
        assert motor.mode == mode
        assert motor.v_max == v_max
        assert get_motor_by_master_id(master_id) is motor

    with pytest.raises(KeyError, match="unknown motor"):
        get_motor("missing")
    with pytest.raises(KeyError, match="unknown master_id"):
        get_motor_by_master_id(0x99)


def test_standard_command_frames() -> None:
    motor = get_motor("dm4310_left")

    assert enable_frame(motor).arbitration_id == 0x02
    assert enable_frame(motor).data == bytes.fromhex("FF FF FF FF FF FF FF FC")
    assert disable_frame(motor).data == bytes.fromhex("FF FF FF FF FF FF FF FD")
    assert clear_error_frame(motor).data == bytes.fromhex("FF FF FF FF FF FF FF FB")
    assert zero_position_frame(motor).data == bytes.fromhex("FF FF FF FF FF FF FF FE")


def test_mit_zero_command_for_dm4310_left_packs_expected_frame() -> None:
    motor = get_motor("dm4310_left")

    frame = mit_frame(motor, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)

    assert frame.arbitration_id == 0x02
    assert frame.data == bytes.fromhex("7F FF 7F F0 00 00 07 FF")
    assert frame.data[0:2] == float_to_uint(0.0, -motor.p_max, motor.p_max, 16).to_bytes(2, "big")


def test_velocity_mode_for_dmh6215_left_uses_float_payload_and_speed_id() -> None:
    motor = get_motor("dmh6215_left")

    frame = velocity_frame(motor, velocity=1.25)

    assert frame.arbitration_id == 0x204
    assert frame.data[:4] == struct.pack("<f", 1.25)
    assert frame.data[4:] == b"\x00\x00\x00\x00"


def test_feedback_decode_extracts_id_state_bit_fields_and_temperatures() -> None:
    motor = get_motor("dm4310_left")
    payload = bytes([0x32, 0x12, 0x34, 0xAB, 0xCD, 0xEF, 66, 77])

    feedback = decode_feedback(payload, motor)

    assert feedback.motor_id == 0x02
    assert feedback.state == 0x03
    assert feedback.position_raw == 0x1234
    assert feedback.velocity_raw == 0xABC
    assert feedback.torque_raw == 0xDEF
    assert feedback.position == uint_to_float(0x1234, -motor.p_max, motor.p_max, 16)
    assert feedback.velocity == uint_to_float(0xABC, -motor.v_max, motor.v_max, 12)
    assert feedback.torque == uint_to_float(0xDEF, -motor.t_max, motor.t_max, 12)
    assert feedback.mos_temperature == 66
    assert feedback.rotor_temperature == 77


def test_decode_feedback_frame_routes_by_master_id_and_rejects_unknown_id() -> None:
    payload = bytes([0x34, 0x12, 0x34, 0xAB, 0xCD, 0xEF, 66, 77])

    routed = decode_feedback_frame(CanFrame(arbitration_id=0x14, data=payload))

    assert routed.motor is get_motor("dmh6215_left")
    assert routed.feedback.motor_id == 0x04
    assert routed.feedback.position_raw == 0x1234
    assert routed.feedback.velocity == uint_to_float(0xABC, -45.0, 45.0, 12)

    with pytest.raises(KeyError, match="unknown master_id: 0x99"):
        decode_feedback_frame(CanFrame(arbitration_id=0x99, data=payload))


def test_parameter_read_write_save_frames_use_fixed_id_and_command_bytes() -> None:
    motor = get_motor("dmh6215_left")

    read = parameter_read_frame(motor, register_id=0x0A)
    write = parameter_write_frame(motor, register_id=0x0A, value=struct.pack("<f", 2.5))
    save = parameter_save_frame(motor)

    assert read.arbitration_id == 0x7FF
    assert read.data == bytes([0x04, 0x00, 0x33, 0x0A, 0, 0, 0, 0])
    assert write.arbitration_id == 0x7FF
    assert write.data[:4] == bytes([0x04, 0x00, 0x55, 0x0A])
    assert write.data[4:] == struct.pack("<f", 2.5)
    assert save.arbitration_id == 0x7FF
    assert save.data == bytes([0x04, 0x00, 0xAA, 0x01, 0, 0, 0, 0])
