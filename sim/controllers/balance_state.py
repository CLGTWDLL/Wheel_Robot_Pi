from __future__ import annotations

import numpy as np

from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS, WHEEL_RADIUS
from sim.state import SimState


def balance_tangent_state(model: object, data: object, state: SimState) -> np.ndarray:
    """提取 LQR 平衡控制用的 4 维切线状态。

    状态向量: [pitch(rad), pitch_rate(rad/s), avg_wheel_pos(m), avg_wheel_vel(m/s)]

    pitch 和 pitch_rate 与 sim.state.extract_sim_state 保持一致:
    - pitch 从 MuJoCo 四元数 [w, x, y, z] 提取本体 X 轴的倾倒角 (与轮轴平行)
    - pitch_rate = wx = base_angular_velocity[0]

    轮子位置/速度使用 WHEEL_FORWARD_SIGNS 统一方向后取平均。
    """
    del model, data

    wheel_names = MODEL_SEMANTICS.wheel_joints
    forward_wheel_positions = [
        WHEEL_FORWARD_SIGNS[name] * state.wheel_positions[_side_key(name)]
        for name in wheel_names
    ]
    forward_wheel_velocities = [
        WHEEL_FORWARD_SIGNS[name] * state.wheel_velocities[_side_key(name)]
        for name in wheel_names
    ]

    tangent = np.array(
        [
            state.pitch,
            state.pitch_rate,
            float(np.mean(forward_wheel_positions)) * WHEEL_RADIUS,
            float(np.mean(forward_wheel_velocities)) * WHEEL_RADIUS,
        ],
        dtype=float,
    )
    if tangent.shape != (4,) or not np.all(np.isfinite(tangent)):
        raise ValueError("balance tangent state must be shape (4,) and finite")
    return tangent


def _side_key(joint_name: str) -> str:
    """将轮子关节名映射到 SimState 中的 left/right 键。"""
    if joint_name == MODEL_SEMANTICS.wheel_joints[0]:
        return "left"
    return "right"
