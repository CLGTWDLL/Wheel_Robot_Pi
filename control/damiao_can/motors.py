from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DeviceConfig:
    name: str
    can_id: int
    master_id: int
    mode: str | None


@dataclass(frozen=True)
class MotorConfig(DeviceConfig):
    mode: str
    p_max: float = 12.5
    v_max: float = 45.0
    t_max: float = 10.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0


MOTORS: tuple[MotorConfig, ...] = (
    MotorConfig(name="dm4310_left", can_id=0x02, master_id=0x12, mode="mit", v_max=30.0),
    MotorConfig(name="dm4310_right", can_id=0x03, master_id=0x13, mode="mit", v_max=30.0),
    MotorConfig(name="dmh6215_left", can_id=0x04, master_id=0x14, mode="torque", v_max=45.0),
    MotorConfig(name="dmh6215_right", can_id=0x05, master_id=0x15, mode="torque", v_max=45.0),
)

DEVICES: tuple[DeviceConfig, ...] = (
    DeviceConfig(name="dm_imu", can_id=0x01, master_id=0x11, mode=None),
    *MOTORS,
)

_DEVICES_BY_NAME = {device.name: device for device in DEVICES}
_MOTORS_BY_NAME = {motor.name: motor for motor in MOTORS}
_MOTORS_BY_MASTER_ID = {motor.master_id: motor for motor in MOTORS}


def get_device(name: str) -> DeviceConfig:
    try:
        return _DEVICES_BY_NAME[name]
    except KeyError as error:
        raise KeyError(f"unknown device: {name}") from error


def get_motor(name: str) -> MotorConfig:
    try:
        return _MOTORS_BY_NAME[name]
    except KeyError as error:
        raise KeyError(f"unknown motor: {name}") from error


def get_motor_by_master_id(master_id: int) -> MotorConfig:
    try:
        return _MOTORS_BY_MASTER_ID[master_id]
    except KeyError as error:
        raise KeyError(f"unknown master_id: 0x{master_id:X}") from error
