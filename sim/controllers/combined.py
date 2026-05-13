from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import mujoco

from sim.controllers.balance_lqr import _wheel_control_selection
from sim.controllers.balance_state import balance_tangent_state
from sim.controllers.lqr import LqrController
from sim.controllers.phase import JumpPhaseMachine, JumpPhase
from sim.controllers.vmc import VmcController, VmcParams
from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS
from sim.state import SimState, model_addresses


@dataclass
class CombinedParams:
    """LQR+VMC 组合控制器参数。

    Attributes:
        vmc: VMC 高度控制参数。
        q_diag: LQR 状态权重对角线 [pitch, pitch_rate, wheel_pos, wheel_vel]。
        r_diag: LQR 控制权重对角线 [left_wheel, right_wheel]。
        target_velocity: 目标前进速度 (m/s)。可运行时修改。
        pitch_trim: 稳态平衡的静态俯仰角偏移（平衡 CoM 的结构偏差）。
        pitch_lean_gain: 速度误差到目标 pitch 偏移的增益。
        velocity_kp: 速度 PI 前馈比例增益。
        velocity_ki: 速度 PI 前馈积分增益。
        yaw_damping: yaw 角速度比例阻尼增益 (P)。
        yaw_ki: yaw 角速度积分增益 (I)。消除稳态偏航偏置。
    """
    vmc: VmcParams
    q_diag: np.ndarray
    r_diag: np.ndarray
    target_velocity: float = 0.0
    pitch_trim: float = 0.0
    pitch_lean_gain: float = 0.02
    velocity_kp: float = 0.5
    velocity_ki: float = 0.1
    yaw_damping: float = 0.5
    yaw_ki: float = 0.0
    target_yaw_rate: float = 0.0
    fixed_height: bool = True
    lqr_height_bin_size: float = 0.03
    height_support_enabled: bool = False
    height_support_target: float | None = None
    height_support_kp: float = 240.0
    height_support_kd: float = 28.0


class CombinedController:
    """LQR(轮子平衡+速度跟踪) + VMC(腿部高度控制) 组合控制器。"""

    def __init__(self, params: CombinedParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.vmc_controller = VmcController(params.vmc, phase_machine)
        self._lqr_controller: LqrController | None = None
        self._velocity_integral: float = 0.0
        self._yaw_integral: float = 0.0
        self._prev_target_velocity: float = params.target_velocity
        self._lqr_gain_cache: dict[int, np.ndarray] = {}
        self._active_lqr_height_bin: int | None = None
        self._height_support_target: float | None = params.height_support_target

    @property
    def lqr_controller(self) -> LqrController | None:
        return self._lqr_controller

    def _height_bin(self, state: SimState) -> int:
        bin_size = max(float(self.params.lqr_height_bin_size), 1e-6)
        return int(np.floor(float(state.base_position[2]) / bin_size))

    def _compute_lqr_gain(self, model: Any, data: Any) -> np.ndarray:
        from sim.controllers.balance_lqr import compute_balance_lqr_gain

        reduced_gain = compute_balance_lqr_gain(model, data, self.params.q_diag, self.params.r_diag)
        return _wheel_control_selection(model) @ reduced_gain

    def _initialize_lqr(self, model: Any, data: Any, state: SimState) -> None:
        if not np.isfinite(self.params.target_velocity):
            raise ValueError("target_velocity must be finite")

        height_bin = self._height_bin(state)
        gain = self._lqr_gain_cache.get(height_bin)
        if gain is None:
            gain = self._compute_lqr_gain(model, data)
            self._lqr_gain_cache[height_bin] = gain
        self._active_lqr_height_bin = height_bin

        feedforward = np.zeros(model.nu)
        target = balance_tangent_state(model, data, state).copy()
        target[0] = self.params.pitch_trim
        target[1] = 0.0
        target[3] = self.params.target_velocity
        self._lqr_controller = LqrController(gain, target, feedforward, balance_tangent_state)

    def _ensure_lqr_height_bin(self, model: Any, data: Any, state: SimState) -> None:
        if self._lqr_controller is None:
            return
        height_bin = self._height_bin(state)
        if height_bin == self._active_lqr_height_bin:
            return
        gain = self._lqr_gain_cache.get(height_bin)
        if gain is None:
            gain = self._compute_lqr_gain(model, data)
            self._lqr_gain_cache[height_bin] = gain
        self._lqr_controller.gain = gain
        self._active_lqr_height_bin = height_bin

    def _apply_height_support(self, model: Any, data: Any, state: SimState) -> None:
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if base_id == -1:
            return
        if not self.params.height_support_enabled:
            data.xfrc_applied[base_id, :] = 0.0
            return
        if self._height_support_target is None:
            self._height_support_target = float(state.base_position[2])
        height_error = self._height_support_target - float(state.base_position[2])
        vertical_velocity = float(state.base_linear_velocity[2])
        support_mass = float(np.sum(model.body_mass))
        gravity = abs(float(model.opt.gravity[2]))
        support_force = np.clip(
            support_mass * gravity
            + self.params.height_support_kp * height_error
            - self.params.height_support_kd * vertical_velocity,
            -1000.0,
            1000.0,
        )
        data.xfrc_applied[base_id, :] = 0.0
        data.xfrc_applied[base_id, 2] = support_force

    def _update_lqr_target(self, state: SimState, dt: float) -> None:
        """每步更新 LQR 目标: 速度跟踪 + pitch 前倾补偿。"""
        if self._lqr_controller is None:
            return
        current_tangent = balance_tangent_state(None, None, state)
        current_wheel_pos = float(current_tangent[2])
        current_wheel_vel = float(current_tangent[3])
        velocity_error = self.params.target_velocity - current_wheel_vel

        # 积分只在没有饱和时累加，防止积分饱和 (anti-windup)
        max_lean = 0.2
        pitch_p = self.params.pitch_lean_gain * velocity_error
        pitch_i = self.params.velocity_ki * self._velocity_integral

        pitch_lean = pitch_p + pitch_i

        if -max_lean < pitch_lean < max_lean:
            self._velocity_integral += velocity_error * dt

        pitch_lean = np.clip(pitch_lean, -max_lean, max_lean)
        self._lqr_controller.target[0] = self.params.pitch_trim + pitch_lean

        # 运动模式: 同步目标位置到当前位置，避免位置环"橡皮筋"。
        # 站立模式 (target_velocity≈0): 保持初始目标位置, 启用 LQR 位置闭环消除漂移。
        if abs(self.params.target_velocity) > 1e-4:
            self._lqr_controller.target[2] = current_wheel_pos

        self._lqr_controller.target[3] = self.params.target_velocity

        self._prev_target_velocity = self.params.target_velocity

    def _compute_forward_torque(self, lqr_control: np.ndarray, addresses: Any) -> float:
        """计算 LQR 输出的前向力矩平均值。"""
        wheel_indices = []
        wheel_signs = []
        for joint_name in MODEL_SEMANTICS.wheel_joints:
            wheel_indices.append(addresses.actuators[joint_name])
            wheel_signs.append(WHEEL_FORWARD_SIGNS[joint_name])

        forward_torques = [wheel_signs[i] * lqr_control[wheel_indices[i]] for i in range(2)]
        avg_forward = float(np.mean(forward_torques))
        return avg_forward

    def _compute_yaw_correction(self, state: SimState, dt: float) -> float:
        """计算 yaw PI 修正量。"""
        yaw_rate = float(state.base_angular_velocity[2])
        yaw_error = yaw_rate - self.params.target_yaw_rate
        self._yaw_integral += yaw_error * dt
        self._yaw_integral = np.clip(self._yaw_integral, -5.0, 5.0)
        return self.params.yaw_damping * yaw_error + self.params.yaw_ki * self._yaw_integral

    def _allocate_wheel_control(self, model: Any, forward_torque: float, yaw_torque: float, addresses: Any) -> np.ndarray:
        """Map virtual forward/yaw wheel torques to physical wheel actuator signs."""
        wheel_control = np.zeros(model.nu)
        virtual_torques = (forward_torque - yaw_torque, forward_torque + yaw_torque)
        for joint_name, virtual_torque in zip(MODEL_SEMANTICS.wheel_joints, virtual_torques):
            actuator_index = addresses.actuators[joint_name]
            wheel_control[actuator_index] = WHEEL_FORWARD_SIGNS[joint_name] * virtual_torque
        return wheel_control

    def _assemble_and_clip_control(self, model: Any, avg_forward: float, yaw_correction: float, vmc_control: np.ndarray, addresses: Any) -> np.ndarray:
        """组装轮腿控制指令并限制在执行器范围内。"""
        control = self._allocate_wheel_control(model, avg_forward, yaw_correction, addresses)

        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = vmc_control[actuator_index]

        clipped = control.copy()
        for joint_name in MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints:
            act_idx = addresses.actuators[joint_name]
            clipped[act_idx] = np.clip(control[act_idx], model.actuator_ctrlrange[act_idx, 0], model.actuator_ctrlrange[act_idx, 1])
        
        if not np.all(np.isfinite(clipped)):
            raise ValueError("combined control must be finite")
        return clipped

    def __call__(self, model: Any, data: Any, state: SimState) -> np.ndarray:
        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")

        dt = float(model.opt.timestep)
        self._apply_height_support(model, data, state)
        if not self.params.fixed_height:
            self._ensure_lqr_height_bin(model, data, state)
        self._update_lqr_target(state, dt)

        self._lqr_controller.feedforward = np.zeros(model.nu)

        addresses = model_addresses(model)
        lqr_control = self._lqr_controller(model, data, state)
        vmc_control = self.vmc_controller(model, data, state)

        phase = self.vmc_controller.phase_machine.phase if self.vmc_controller.phase_machine is not None else JumpPhase.STAND
        
        if phase in (JumpPhase.FLIGHT, JumpPhase.LAND, JumpPhase.FAIL):
            avg_forward = 0.0
            yaw_correction = 0.0
        else:
            avg_forward = self._compute_forward_torque(lqr_control, addresses)
            yaw_correction = self._compute_yaw_correction(state, dt)

        return self._assemble_and_clip_control(model, avg_forward, yaw_correction, vmc_control, addresses)
