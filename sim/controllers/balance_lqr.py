from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from sim.controllers.lqr import solve_discrete_lqr
from sim.linearize import linearize_transition
from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS
from sim.state import model_addresses


def compute_balance_lqr_gain(
    model: Any,
    data: Any,
    q_diag: np.ndarray,
    r_diag: np.ndarray,
) -> np.ndarray:
    q_diag = _validate_positive_diag(q_diag, (4,), "q_diag")
    r_diag = _validate_positive_diag(r_diag, (2,), "r_diag")

    a, b = _reduced_balance_system(model, data)
    q = np.diag(q_diag)
    r = np.diag(r_diag)
    try:
        gain = solve_discrete_lqr(a, b, q, r)
    except np.linalg.LinAlgError as exc:
        raise ValueError("failed to solve balance LQR") from exc
    if gain.shape != (2, 4) or not np.all(np.isfinite(gain)):
        raise ValueError("balance LQR gain must be finite shape (2, 4)")
    return gain


def _reduced_balance_system(model: Any, data: Any) -> tuple[np.ndarray, np.ndarray]:
    state = np.concatenate([data.qpos.copy(), data.qvel.copy()])
    control = np.zeros(model.nu)
    
    # NEW: Apply gravity compensation to the leg motors so the robot doesn't collapse
    mujoco.mj_forward(model, data)
    addresses = model_addresses(model)
    for jname in MODEL_SEMANTICS.leg_motor_joints:
        control[addresses.actuators[jname]] = data.qfrc_bias[addresses.joint_qvel[jname]]
        
    result = linearize_transition(model, state, control)
    if not result.valid:
        reason = f": {result.reason}" if result.reason else ""
        raise ValueError(f"failed to linearize balance system{reason}")
    if result.a.shape != (2 * model.nv, 2 * model.nv) or result.b.shape != (2 * model.nv, model.nu):
        raise ValueError("linearized matrices have unexpected shape")
    if not np.all(np.isfinite(result.a)) or not np.all(np.isfinite(result.b)):
        raise ValueError("linearized matrices must be finite")

    p = _balance_state_selection(model)
    s = _wheel_control_selection(model)
    a = p @ result.a @ np.linalg.pinv(p)
    b = p @ result.b @ s
    if a.shape != (4, 4) or b.shape != (4, 2) or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("reduced balance system must be finite with shapes (4, 4) and (4, 2)")
    return a, b


def _balance_state_selection(model: Any) -> np.ndarray:
    addresses = model_addresses(model)
    p = np.zeros((4, 2 * model.nv))
    # 绕 Y 轴的旋转 (Pitch) 及其角速度 wy — quat=[0,1,0,0] 下的倾倒轴
    pitch_index = addresses.root_qvel + 4
    pitch_rate_index = model.nv + addresses.root_qvel + 4
    wheel_dof_indices = [addresses.joint_qvel[name] for name in MODEL_SEMANTICS.wheel_joints]

    p[0, pitch_index] = 1.0
    p[1, pitch_rate_index] = 1.0
    # 轮子行: 关节角度/速度均值，与 balance_tangent_state 运行时一致
    for dof_index in wheel_dof_indices:
        p[2, dof_index] = WHEEL_FORWARD_SIGNS[MODEL_SEMANTICS.wheel_joints[wheel_dof_indices.index(dof_index)]] / len(wheel_dof_indices)
        p[3, model.nv + dof_index] = WHEEL_FORWARD_SIGNS[MODEL_SEMANTICS.wheel_joints[wheel_dof_indices.index(dof_index)]] / len(wheel_dof_indices)
    return p


def _wheel_control_selection(model: Any) -> np.ndarray:
    """构建控制选择矩阵 S, 将 2 维虚拟控制映射到执行器空间。

    使用 WHEEL_FORWARD_SIGNS 确保虚拟控制正值对应前进方向力矩。
    """
    addresses = model_addresses(model)
    s = np.zeros((model.nu, 2))
    for column, joint_name in enumerate(MODEL_SEMANTICS.wheel_joints):
        s[addresses.actuators[joint_name], column] = WHEEL_FORWARD_SIGNS[joint_name]
    return s


def _validate_positive_diag(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)) or not np.all(array > 0.0):
        raise ValueError(f"{name} must be finite positive shape {shape}")
    return array
