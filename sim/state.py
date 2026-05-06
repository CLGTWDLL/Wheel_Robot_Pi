from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from sim.model_semantics import MODEL_SEMANTICS


@dataclass(frozen=True)
class ModelAddresses:
    root_qpos: int
    root_qvel: int
    joint_qpos: dict[str, int]
    joint_qvel: dict[str, int]
    actuators: dict[str, int]


@dataclass(frozen=True)
class SimState:
    base_position: np.ndarray
    base_quaternion: np.ndarray
    base_linear_velocity: np.ndarray
    base_angular_velocity: np.ndarray
    pitch: float
    pitch_rate: float
    wheel_positions: dict[str, float]
    wheel_velocities: dict[str, float]
    leg_joint_positions: dict[str, float]
    leg_joint_velocities: dict[str, float]
    contact_count: int


def model_addresses(model: mujoco.MjModel) -> ModelAddresses:
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    if root_id == -1:
        raise ValueError("missing root freejoint")

    joint_qpos: dict[str, int] = {}
    joint_qvel: dict[str, int] = {}
    for joint_name in MODEL_SEMANTICS.joint_roles:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id == -1:
            raise ValueError(f"missing joint: {joint_name}")
        joint_qpos[joint_name] = int(model.jnt_qposadr[joint_id])
        joint_qvel[joint_name] = int(model.jnt_dofadr[joint_id])

    actuators: dict[str, int] = {}
    for joint_name in MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        actuator_id = _actuator_for_joint(model, joint_id)
        if actuator_id == -1:
            raise ValueError(f"missing actuator for joint: {joint_name}")
        actuators[joint_name] = actuator_id

    return ModelAddresses(
        root_qpos=int(model.jnt_qposadr[root_id]),
        root_qvel=int(model.jnt_dofadr[root_id]),
        joint_qpos=joint_qpos,
        joint_qvel=joint_qvel,
        actuators=actuators,
    )


def extract_sim_state(model: mujoco.MjModel, data: mujoco.MjData) -> SimState:
    addresses = model_addresses(model)
    root_qpos = addresses.root_qpos
    root_qvel = addresses.root_qvel

    base_quaternion = np.array(data.qpos[root_qpos + 3 : root_qpos + 7], dtype=float)
    base_angular_velocity = np.array(data.qvel[root_qvel + 3 : root_qvel + 6], dtype=float)
    wheel_names = {"left": MODEL_SEMANTICS.wheel_joints[0], "right": MODEL_SEMANTICS.wheel_joints[1]}

    return SimState(
        base_position=np.array(data.qpos[root_qpos : root_qpos + 3], dtype=float),
        base_quaternion=base_quaternion,
        base_linear_velocity=np.array(data.qvel[root_qvel : root_qvel + 3], dtype=float),
        base_angular_velocity=base_angular_velocity,
        # 轮轴沿世界 X 轴，底盘翻转180°后倾倒方向绕 X 轴 (wx)。
        pitch=_pitch_from_quaternion(base_quaternion),
        pitch_rate=float(base_angular_velocity[0]),
        wheel_positions={side: float(data.qpos[addresses.joint_qpos[name]]) for side, name in wheel_names.items()},
        wheel_velocities={side: float(data.qvel[addresses.joint_qvel[name]]) for side, name in wheel_names.items()},
        leg_joint_positions={
            name: float(data.qpos[addresses.joint_qpos[name]]) for name in MODEL_SEMANTICS.leg_motor_joints
        },
        leg_joint_velocities={
            name: float(data.qvel[addresses.joint_qvel[name]]) for name in MODEL_SEMANTICS.leg_motor_joints
        },
        contact_count=int(data.ncon),
    )


def _actuator_for_joint(model: mujoco.MjModel, joint_id: int) -> int:
    for actuator_id in range(model.nu):
        if int(model.actuator_trnid[actuator_id, 0]) == joint_id:
            return actuator_id
    return -1


def _pitch_from_quaternion(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    # 提取绕 X 轴的旋转 (Roll)，quat=[0,1,0,0]时为平衡点
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = float(np.arctan2(sinr, cosr))
    # 平衡点 quat=[0,1,0,0] 对应 roll=pi, 偏差 = roll - pi
    error = roll - np.pi if roll >= 0 else roll + np.pi
    return float(error)
