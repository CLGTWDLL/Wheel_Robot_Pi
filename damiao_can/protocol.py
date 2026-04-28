from __future__ import annotations

import struct
from dataclasses import dataclass

from damiao_can.motors import MotorConfig, get_motor_by_master_id


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes

    def __post_init__(self) -> None:
        if len(self.data) != 8:
            raise ValueError("CAN frame data must be 8 bytes")


@dataclass(frozen=True)
class Feedback:
    motor_id: int
    state: int
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    rotor_temperature: int
    position_raw: int
    velocity_raw: int
    torque_raw: int


@dataclass(frozen=True)
class RoutedFeedback:
    motor: MotorConfig
    feedback: Feedback


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = min(max(value, minimum), maximum)
    return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def enable_frame(motor: MotorConfig) -> CanFrame:
    return _standard_command_frame(motor, 0xFC)


def disable_frame(motor: MotorConfig) -> CanFrame:
    return _standard_command_frame(motor, 0xFD)


def zero_position_frame(motor: MotorConfig) -> CanFrame:
    return _standard_command_frame(motor, 0xFE)


def clear_error_frame(motor: MotorConfig) -> CanFrame:
    return _standard_command_frame(motor, 0xFB)


def mit_frame(
    motor: MotorConfig,
    *,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    torque: float = 0.0,
) -> CanFrame:
    p_uint = float_to_uint(position, -motor.p_max, motor.p_max, 16)
    v_uint = float_to_uint(velocity, -motor.v_max, motor.v_max, 12)
    kp_uint = float_to_uint(kp, motor.kp_min, motor.kp_max, 12)
    kd_uint = float_to_uint(kd, motor.kd_min, motor.kd_max, 12)
    t_uint = float_to_uint(torque, -motor.t_max, motor.t_max, 12)

    data = bytes(
        [
            (p_uint >> 8) & 0xFF,
            p_uint & 0xFF,
            (v_uint >> 4) & 0xFF,
            ((v_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F),
            t_uint & 0xFF,
        ]
    )
    return CanFrame(motor.can_id, data)


def velocity_frame(motor: MotorConfig, *, velocity: float = 0.0) -> CanFrame:
    return CanFrame(0x200 + motor.can_id, struct.pack("<f", velocity) + b"\x00\x00\x00\x00")


def decode_feedback_frame(frame: CanFrame) -> RoutedFeedback:
    motor = get_motor_by_master_id(frame.arbitration_id)
    return RoutedFeedback(motor=motor, feedback=decode_feedback(frame.data, motor))


def decode_feedback(data: bytes, motor: MotorConfig) -> Feedback:
    if len(data) != 8:
        raise ValueError("feedback data must be 8 bytes")

    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]

    return Feedback(
        motor_id=data[0] & 0x0F,
        state=data[0] >> 4,
        position=uint_to_float(position_raw, -motor.p_max, motor.p_max, 16),
        velocity=uint_to_float(velocity_raw, -motor.v_max, motor.v_max, 12),
        torque=uint_to_float(torque_raw, -motor.t_max, motor.t_max, 12),
        mos_temperature=data[6],
        rotor_temperature=data[7],
        position_raw=position_raw,
        velocity_raw=velocity_raw,
        torque_raw=torque_raw,
    )


def parameter_read_frame(motor: MotorConfig, *, register_id: int) -> CanFrame:
    return _parameter_frame(motor, 0x33, register_id, b"\x00\x00\x00\x00")


def parameter_write_frame(motor: MotorConfig, *, register_id: int, value: bytes) -> CanFrame:
    if len(value) != 4:
        raise ValueError("parameter value must be 4 bytes")
    return _parameter_frame(motor, 0x55, register_id, value)


def parameter_save_frame(motor: MotorConfig) -> CanFrame:
    return _parameter_frame(motor, 0xAA, 0x01, b"\x00\x00\x00\x00")


def _standard_command_frame(motor: MotorConfig, command: int) -> CanFrame:
    return CanFrame(motor.can_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF" + bytes([command]))


def _parameter_frame(motor: MotorConfig, command: int, register_id: int, value: bytes) -> CanFrame:
    data = bytes([motor.can_id & 0xFF, (motor.can_id >> 8) & 0xFF, command, register_id]) + value
    return CanFrame(0x7FF, data)
