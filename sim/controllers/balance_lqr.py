from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from sim.controllers.lqr import solve_discrete_lqr
from sim.linearize import linearize_transition
from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS, WHEEL_RADIUS
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
    
    # NEW: Force B matrix to be perfectly symmetric across wheels.
    # Small asymmetries due to contact solver / floating point can cause
    # LQR to exploit differential torque to fix pitch (since yaw is unpenalized),
    # leading to massive opposite wheel torques.
    b_avg = (b[:, 0] + b[:, 1]) / 2.0
    b[:, 0] = b_avg
    b[:, 1] = b_avg

    # FULLY ANALYTICAL HACK: Inject analytical inverted pendulum physics into A and B matrices.
    L = 0.137  # Height of CoM above wheel center
    g = 9.81
    
    # CRITICAL: The control loop runs at 500Hz (dt=0.002), but model.opt.timestep is 2000Hz (0.0005).
    # We must solve the discrete ARE for the CONTROL timestep, not the simulation timestep!
    control_dt = 0.002
    sim_dt = model.opt.timestep
    dt = control_dt
    
    # Analytical wheel acceleration (no-slip assumption)
    # Mass of base = 1.7kg, Mass of wheel = 0.25kg, R = 0.036m
    # 1.0 Nm on ONE wheel gives F = 1.0 / 0.036 = 27.7N
    # M_eff = M_base + 2 * M_wheel = ~2.2kg
    # a_x = F / M_eff = 27.7 / 2.2 = 12.6 m/s^2
    # b_vel = a_x * dt = 12.6 * 0.002 = 0.0252 m/s per Nm per control step
    b_vel = 0.0252

    # Reconstruct A matrix purely from analytical pendulum dynamics for control_dt
    a_new = np.zeros((4, 4))
    a_new[0, 0] = 1.0                  # pitch -> pitch
    a_new[0, 1] = dt                   # pitch_rate -> pitch
    a_new[1, 0] = (g / L) * dt         # pitch -> pitch_rate (gravity pulls further)
    a_new[1, 1] = 1.0                  # pitch_rate -> pitch_rate
    a_new[2, 2] = 1.0                  # wheel_pos -> wheel_pos
    a_new[2, 3] = dt                   # wheel_vel -> wheel_pos
    a_new[3, 3] = 1.0                  # wheel_vel -> wheel_vel (no friction assumed for LQR)
    
    a[:, :] = a_new

    # The reaction torque and kinematic coupling pitches the robot backward (b[1, 0] < 0)
    # The ratio of pitch accel to linear accel is exactly -1/L.
    b_new = np.zeros((4, 2))
    b_new[1, 0] = -(b_vel / L)
    b_new[1, 1] = -(b_vel / L)
    b_new[3, 0] = b_vel
    b_new[3, 1] = b_vel
    
    b[:, :] = b_new

    if a.shape != (4, 4) or b.shape != (4, 2) or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("reduced balance system must be finite with shapes (4, 4) and (4, 2)")
    return a, b


def _balance_state_selection(model: Any) -> np.ndarray:
    addresses = model_addresses(model)
    p = np.zeros((4, 2 * model.nv))
    # 与 extract_sim_state()/balance_tangent_state() 保持一致: pitch 是局部 Y 轴倾角。
    pitch_index = addresses.root_qvel + 4
    pitch_rate_index = model.nv + addresses.root_qvel + 4
    wheel_dof_indices = [addresses.joint_qvel[name] for name in MODEL_SEMANTICS.wheel_joints]

    p[0, pitch_index] = 1.0
    p[1, pitch_rate_index] = 1.0
    # 轮子行: 关节角度/速度均值，与 balance_tangent_state 运行时一致
    for dof_index in wheel_dof_indices:
        sign = WHEEL_FORWARD_SIGNS[MODEL_SEMANTICS.wheel_joints[wheel_dof_indices.index(dof_index)]]
        p[2, dof_index] = (sign / len(wheel_dof_indices)) * WHEEL_RADIUS
        p[3, model.nv + dof_index] = (sign / len(wheel_dof_indices)) * WHEEL_RADIUS
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
    if array.shape != shape or not np.all(np.isfinite(array)) or not np.all(array >= 0.0):
        raise ValueError(f"{name} must be finite non-negative shape {shape}")
    return array
