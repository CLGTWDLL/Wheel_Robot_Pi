from __future__ import annotations

import numpy as np

from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS
from sim.state import SimState


def balance_tangent_state(model: object, data: object, state: SimState) -> np.ndarray:
    """提取 LQR 平衡控制用的 4 维切线状态。

    状态向量: [pitch, pitch_rate, avg_wheel_pos, avg_wheel_vel]

    pitch 和 pitch_rate 现已修正为绕局部 X 轴的旋转（即真实的倾倒角 Roll）：
    - pitch = roll - pi (跌倒向前时为正)
    - pitch_rate = wx (跌倒向前时为正)

    轮子位置/速度使用 WHEEL_FORWARD_SIGNS 统一方向后取平均。
    """
    del model, data

    wheel_names = MODEL_SEMANTICS.wheel_joints
    # 使用方向符号统一轮速方向: 右轮正转=后退, 取负后=前进
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
            float(np.mean(forward_wheel_positions)),
            float(np.mean(forward_wheel_velocities)),
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
