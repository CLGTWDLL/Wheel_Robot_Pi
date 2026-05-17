from __future__ import annotations

from dataclasses import dataclass
from contextlib import contextmanager
from typing import Any

import numpy as np
import mujoco

from sim.controllers.balance_lqr import LEG_ROLL_DIFF_SIGNS, equilibrium_pitch_from_geometry
from sim.controllers.balance_state import balance_tangent_state_5d
from sim.controllers.lqr import LqrController
from sim.controllers.phase import JumpPhaseMachine, JumpPhase
from sim.controllers.vmc import LEG_CLOSED_LOOP, VmcController, VmcParams
from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS
from sim.state import SimState, body_id, model_addresses


@dataclass
class CombinedParams:
    """LQR+VMC 组合控制器参数。

    Attributes:
        vmc: VMC 高度控制参数。
        q_diag: LQR 状态权重对角线 [pitch, pitch_rate, roll, roll_rate, wheel_vel]。
            wheel_pos 不在 LQR state 中,避免位置反馈与平衡所需的轮子运动正反馈。
        r_diag: LQR 虚拟控制权重对角线 [forward_wheel, roll_diff_leg]。
        target_velocity: 目标前进速度 (m/s)。可运行时修改。
        pitch_lean_gain: 速度误差到目标 pitch 偏移的增益。
        velocity_ki: 速度积分补偿增益。
        position_kp: 位置外环 P 增益 (1/s)。pos_err [m] → vel_ref 修正 [m/s]。
            target_velocity=0 时启用,把 STAND 阶段的位置漂移转成速度参考送入内环。
        position_kd: 位置外环 D 增益 (无量纲)。把 base 水平速度反馈进 vel_ref,
            提供位置-pitch 二阶环的阻尼,避免与内环构成欠阻尼震荡。
        position_velocity_limit: 位置外环输出 vel_ref 修正幅度限制 (m/s),
            防止大位置误差产生过激速度指令。
        yaw_damping: yaw 角速度比例阻尼增益 (P)。
        yaw_ki: yaw 角速度积分增益 (I)。消除稳态偏航偏置。
    """
    vmc: VmcParams
    q_diag: np.ndarray
    r_diag: np.ndarray
    target_velocity: float = 0.0
    pitch_lean_gain: float = 0.02
    velocity_ki: float = 0.1
    position_kp: float = 1.5
    position_kd: float = 1.5
    position_velocity_limit: float = 0.3
    yaw_damping: float = 0.5
    yaw_ki: float = 0.0
    target_yaw_rate: float = 0.0
    fixed_height: bool = True
    lqr_height_bin_size: float = 0.02
    flight_wheel_q: float = 1.0
    ff_gain: float = 1.0


class CombinedController:
    """LQR(轮子平衡+速度跟踪) + VMC(腿部高度控制) 组合控制器。"""

    def __init__(self, params: CombinedParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.vmc_controller = VmcController(params.vmc, phase_machine)
        self._lqr_controller: LqrController | None = None
        self._velocity_integral: float = 0.0
        self._yaw_integral: float = 0.0
        self._prev_target_velocity: float = params.target_velocity
        self._lqr_gain_cache: dict[tuple[int, int], np.ndarray] = {}
        self._active_lqr_gain_key: tuple[int, int] | None = None
        self._height_wheel_velocity_ff: float = 0.0
        self._last_phase = JumpPhase.STAND
        self._position_anchor: np.ndarray | None = None
        self._equilibrium_pitch: float = 0.0

    @property
    def lqr_controller(self) -> LqrController | None:
        return self._lqr_controller

    def _height_bin(self, state: SimState) -> int:
        bin_size = max(float(self.params.lqr_height_bin_size), 1e-6)
        return int(np.floor(float(state.base_position[2]) / bin_size))

    def _lqr_gain_key(self, model: Any, data: Any, state: SimState) -> tuple[int, int]:
        del model, data
        roll_bin = int(np.floor(float(state.roll) / 0.02))
        return (self._height_bin(state), roll_bin)

    def _compute_lqr_gain(self, model: Any, data: Any) -> np.ndarray:
        from sim.controllers.balance_lqr import compute_balance_lqr_gain_5d

        reduced_gain = compute_balance_lqr_gain_5d(model, data, self.params.q_diag, self.params.r_diag)
        return reduced_gain

    def _initialize_lqr(self, model: Any, data: Any, state: SimState) -> None:
        if not np.isfinite(self.params.target_velocity):
            raise ValueError("target_velocity must be finite")

        gain_key = self._lqr_gain_key(model, data, state)
        gain = self._lqr_gain_cache.get(gain_key)
        if gain is None:
            gain = self._compute_lqr_gain(model, data)
            self._lqr_gain_cache[gain_key] = gain
        self._active_lqr_gain_key = gain_key

        self._equilibrium_pitch = equilibrium_pitch_from_geometry(model, data)
        feedforward = np.zeros(2)
        target = balance_tangent_state_5d(model, data, state).copy()
        target[0] = self._equilibrium_pitch
        target[1] = 0.0
        target[2] = 0.0
        target[3] = 0.0
        target[4] = self.params.target_velocity
        self._lqr_controller = LqrController(gain, target, feedforward, balance_tangent_state_5d)

    def _ensure_lqr_height_bin(self, model: Any, data: Any, state: SimState) -> None:
        if self._lqr_controller is None:
            return
        gain_key = self._lqr_gain_key(model, data, state)
        if gain_key == self._active_lqr_gain_key:
            return
        gain = self._lqr_gain_cache.get(gain_key)
        if gain is None:
            gain = self._compute_lqr_gain(model, data)
            self._lqr_gain_cache[gain_key] = gain
        self._lqr_controller.gain = gain
        self._active_lqr_gain_key = gain_key

    def _average_leg_height_and_wheel_mid_z(self, model: Any, data: Any) -> tuple[float, float]:
        wheel_ids = [
            body_id(model, geometry.wheel_body)
            for geometry in LEG_CLOSED_LOOP.values()
        ]
        base_id = body_id(model, "base_link")
        wheel_mid_z = float(np.mean([data.xipos[wheel_id, 2] for wheel_id in wheel_ids]))
        return float(data.xipos[base_id, 2] - wheel_mid_z), wheel_mid_z

    def _height_wheel_velocity_feedforward(self, current_leg_height: float) -> float:
        if self.params.fixed_height:
            return 0.0
        left_rate = self.vmc_controller.last_target_motor_rate.get("left", 0.0)
        right_rate = self.vmc_controller.last_target_motor_rate.get("right", 0.0)
        theta_rate_mean = 0.5 * (left_rate + right_rate)
        dh_dt_cmd = self.params.vmc.lut.height_dtheta(current_leg_height) * theta_rate_mean
        return self.params.ff_gain * self.params.vmc.lut.dy_wheel_dh(current_leg_height) * dh_dt_cmd

    def _position_outer_loop(self, model: Any, data: Any, state: SimState) -> float:
        """位置 PD 外环: 把本体前进方向上的位置误差和水平速度组合成 vel_ref 修正。

        target_velocity != 0 或 anchor 未设置时返回 0。位置环只在静止保持位时启用。
        加 D 项 (kd * base_forward_vel) 是为了避免单 P 反馈与内环 pitch 跟踪
        构成的二阶系统欠阻尼震荡 (实测 ζ≈0.3,机器人像钟摆持续晃)。
        """
        if self._position_anchor is None:
            return 0.0
        if abs(self.params.target_velocity) > 1e-6:
            return 0.0
        base_id = body_id(model, "base_link")
        rotation = np.asarray(data.xmat[base_id]).reshape(3, 3)
        forward_horiz = rotation[:2, 1]
        forward_norm = float(np.linalg.norm(forward_horiz))
        if forward_norm < 1e-6:
            return 0.0
        forward_horiz = forward_horiz / forward_norm
        delta = self._position_anchor - state.base_position[:2]
        forward_error = float(np.dot(delta, forward_horiz))
        forward_velocity = float(np.dot(state.base_linear_velocity[:2], forward_horiz))
        vel_correction = self.params.position_kp * forward_error - self.params.position_kd * forward_velocity
        limit = max(float(self.params.position_velocity_limit), 0.0)
        return float(np.clip(vel_correction, -limit, limit))

    def _update_lqr_target(self, model: Any, data: Any, state: SimState, dt: float, phase: JumpPhase) -> None:
        """每步更新 LQR 5D 目标: 几何平衡 pitch + 位置/速度跟踪。

        5D state: [pitch, pitch_rate, roll, roll_rate, wheel_vel]。无 wheel_pos,
        因此本函数只写 target[0] (pitch) 和 target[4] (wheel_vel)。
        target[0] = equilibrium_pitch + pitch_lean (速度 PI 输出);
        target[4] = target_velocity + position_outer_loop (位置 P 输出)。
        """
        if self._lqr_controller is None:
            return
        current_tangent = balance_tangent_state_5d(None, None, state)
        current_wheel_vel = float(current_tangent[4])
        release_wheel_target = phase == JumpPhase.FLIGHT

        # equilibrium_pitch 只在 STAND 阶段重算。落地/起跳期间腿被压扁或伸展,
        # CoM 相对 wheel_mid 的本体偏移瞬时变化大,target_pitch 跟着抖动会把
        # LQR 的瞬态控制目标搞乱、放大震荡。其他阶段沿用最近一次 STAND 测得的值。
        if phase == JumpPhase.STAND:
            self._equilibrium_pitch = equilibrium_pitch_from_geometry(model, data)

        position_vel_correction = 0.0
        if phase == JumpPhase.STAND and not release_wheel_target:
            position_vel_correction = self._position_outer_loop(model, data, state)

        if release_wheel_target:
            velocity_target = 0.0
        else:
            velocity_target = self.params.target_velocity + self._height_wheel_velocity_ff + position_vel_correction
        velocity_error = velocity_target - current_wheel_vel

        # 积分只在没有饱和时累加,防止积分饱和 (anti-windup)
        max_lean = 0.2
        pitch_p = self.params.pitch_lean_gain * velocity_error
        pitch_i = self.params.velocity_ki * self._velocity_integral

        pitch_lean = pitch_p + pitch_i

        if -max_lean < pitch_lean < max_lean:
            self._velocity_integral += velocity_error * dt

        pitch_lean = np.clip(pitch_lean, -max_lean, max_lean)
        self._lqr_controller.target[0] = self._equilibrium_pitch + pitch_lean
        self._lqr_controller.target[1] = 0.0
        self._lqr_controller.target[2] = 0.0
        self._lqr_controller.target[3] = 0.0
        self._lqr_controller.target[4] = velocity_target

        self._prev_target_velocity = self.params.target_velocity

    @contextmanager
    def _flight_lqr_weight_schedule(self, model: Any, data: Any, phase: JumpPhase):
        if self._lqr_controller is None or phase != JumpPhase.FLIGHT:
            yield
            return
        original_gain = self._lqr_controller.gain
        original_q_diag = self.params.q_diag
        scheduled_q = original_q_diag.copy()
        # 5D state: Q[4] 是 wheel_vel。飞行期间无地面摩擦，速度反馈意义不大，
        # 弱化让 LQR 优先 pitch/roll 控制。
        scheduled_q[4] = self.params.flight_wheel_q
        try:
            if not np.allclose(scheduled_q, original_q_diag):
                self.params.q_diag = scheduled_q
                self._lqr_controller.gain = self._compute_lqr_gain(model, data)
            yield
        finally:
            self.params.q_diag = original_q_diag
            self._lqr_controller.gain = original_gain

    def _compute_lqr_virtual_outputs(self, lqr_control: np.ndarray) -> tuple[float, float]:
        """返回 LQR 虚拟输出: (forward_wheel, roll_diff_leg)。"""
        if lqr_control.shape != (2,):
            raise ValueError("LQR virtual control must have shape (2,)")
        return float(lqr_control[0]), float(lqr_control[1])

    def _compute_yaw_correction(self, state: SimState, dt: float) -> float:
        """计算 yaw PI 修正量。"""
        yaw_rate = float(state.base_angular_velocity[2])
        yaw_error = yaw_rate - self.params.target_yaw_rate
        self._yaw_integral += yaw_error * dt
        self._yaw_integral = np.clip(self._yaw_integral, -5.0, 5.0)
        return self.params.yaw_damping * yaw_error + self.params.yaw_ki * self._yaw_integral

    def _reset_balance_state(self) -> None:
        self._velocity_integral = 0.0
        self._yaw_integral = 0.0
        self._height_wheel_velocity_ff = 0.0
        self._lqr_controller = None
        self._active_lqr_gain_key = None
        self._position_anchor = None

    def _allocate_wheel_control(self, model: Any, forward_torque: float, yaw_torque: float, addresses: Any) -> np.ndarray:
        """Map virtual forward/yaw wheel torques to physical wheel actuator signs."""
        wheel_control = np.zeros(model.nu)
        virtual_torques = (forward_torque - yaw_torque, forward_torque + yaw_torque)
        for joint_name, virtual_torque in zip(MODEL_SEMANTICS.wheel_joints, virtual_torques):
            actuator_index = addresses.actuators[joint_name]
            wheel_control[actuator_index] = WHEEL_FORWARD_SIGNS[joint_name] * virtual_torque
        return wheel_control

    def _allocate_lqr_control(self, model: Any, forward_torque: float, roll_torque: float, addresses: Any) -> np.ndarray:
        control = np.zeros(model.nu)
        for joint_name in MODEL_SEMANTICS.wheel_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = WHEEL_FORWARD_SIGNS[joint_name] * forward_torque
        for joint_name, sign in LEG_ROLL_DIFF_SIGNS.items():
            control[addresses.actuators[joint_name]] = sign * roll_torque
        return control

    def _assemble_and_clip_control(
        self,
        model: Any,
        avg_forward: float,
        yaw_correction: float,
        roll_torque: float,
        vmc_control: np.ndarray,
        addresses: Any,
    ) -> np.ndarray:
        """组装轮腿控制指令并限制在执行器范围内。"""
        control = self._allocate_lqr_control(model, avg_forward, roll_torque, addresses)
        control += self._allocate_wheel_control(model, 0.0, yaw_correction, addresses)

        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] += vmc_control[actuator_index]

        clipped = control.copy()
        for joint_name in MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints:
            act_idx = addresses.actuators[joint_name]
            clipped[act_idx] = np.clip(control[act_idx], model.actuator_ctrlrange[act_idx, 0], model.actuator_ctrlrange[act_idx, 1])
            phase_machine = self.vmc_controller.phase_machine
            phase = phase_machine.phase if phase_machine is not None else JumpPhase.STAND
            if joint_name in MODEL_SEMANTICS.leg_motor_joints and phase in (
                JumpPhase.STAND,
                JumpPhase.CROUCH,
                JumpPhase.LAND,
            ):
                clipped[act_idx] = np.clip(clipped[act_idx], -3.5, 3.5)
        
        if not np.all(np.isfinite(clipped)):
            raise ValueError("combined control must be finite")
        return clipped

    def __call__(self, model: Any, data: Any, state: SimState) -> np.ndarray:
        dt = float(model.opt.timestep)
        addresses = model_addresses(model)
        vmc_control = self.vmc_controller(model, data, state)

        phase = self.vmc_controller.phase_machine.phase if self.vmc_controller.phase_machine is not None else JumpPhase.STAND

        if phase == JumpPhase.FALLEN:
            self._reset_balance_state()
            self._last_phase = phase
            return np.zeros(model.nu)

        # 起跳入相清零积分。Stand 模式下两个积分在数秒内会缓慢漂移,跨跳积累
        # 会让 pitch_lean 目标偏移,前后两次跳跃的起跳条件不一致。CROUCH 起点强制
        # 同步两个积分,保证每次跳跃从一致的内部状态开始。
        if phase == JumpPhase.CROUCH and self._last_phase != JumpPhase.CROUCH:
            self._velocity_integral = 0.0
            self._yaw_integral = 0.0
            self._position_anchor = None

        # 进入 STAND 时锁住当前位置作为 anchor,清掉飞行/落地期间残留的速度积分,
        # 防止跨阶段误差进入新的平衡保持周期。
        if phase == JumpPhase.STAND and self._last_phase != JumpPhase.STAND:
            self._velocity_integral = 0.0
            self._position_anchor = np.array(state.base_position[:2], dtype=float)

        # 行驶 (target_velocity != 0) 不锁位置;停下时重新锁当前位置。
        if abs(self.params.target_velocity) > 1e-6:
            self._position_anchor = None
        elif phase == JumpPhase.STAND and self._position_anchor is None:
            self._position_anchor = np.array(state.base_position[:2], dtype=float)

        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")

        if not self.params.fixed_height:
            self._ensure_lqr_height_bin(model, data, state)

        current_leg_height, _wheel_mid_z = self._average_leg_height_and_wheel_mid_z(model, data)
        self._height_wheel_velocity_ff = self._height_wheel_velocity_feedforward(current_leg_height)
        if phase in (JumpPhase.FLIGHT, JumpPhase.LAND):
            self._height_wheel_velocity_ff = 0.0

        self._update_lqr_target(model, data, state, dt, phase)

        self._lqr_controller.feedforward = np.zeros(2)
        with self._flight_lqr_weight_schedule(model, data, phase):
            lqr_control = self._lqr_controller(model, data, state)

        avg_forward, roll_torque = self._compute_lqr_virtual_outputs(lqr_control)
        yaw_correction = self._compute_yaw_correction(state, dt)

        control = self._assemble_and_clip_control(model, avg_forward, yaw_correction, roll_torque, vmc_control, addresses)
        self._last_phase = phase
        return control
