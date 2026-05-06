from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import mujoco

from sim.controllers.balance_lqr import compute_balance_lqr_gain, _wheel_control_selection
from sim.controllers.balance_state import balance_tangent_state
from sim.controllers.lqr import LqrController
from sim.controllers.phase import JumpPhaseMachine
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


class CombinedController:
    """LQR(轮子平衡+速度跟踪) + VMC(腿部高度控制) 组合控制器。"""

    def __init__(self, params: CombinedParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.vmc_controller = VmcController(params.vmc, phase_machine)
        self._lqr_controller: LqrController | None = None
        self._velocity_integral: float = 0.0
        self._yaw_integral: float = 0.0
        self._prev_target_velocity: float = params.target_velocity

    @property
    def lqr_controller(self) -> LqrController | None:
        return self._lqr_controller

    def _initialize_lqr(self, model: Any, data: Any, state: SimState) -> None:
        if not np.isfinite(self.params.target_velocity):
            raise ValueError("target_velocity must be finite")

        from sim.controllers.balance_lqr import _reduced_balance_system
        from sim.controllers.lqr import solve_discrete_lqr as _solve_lqr

        # 预先计算好的 z=0.24 时的关节角度，避免腿部伸直导致穿模或接触惩罚力导致刚度矩阵畸变
        orig_qpos = data.qpos.copy()
        data.qpos[2] = 0.24
        squatted_qpos = {
            "robot_base_link_r1": 0.190978,
            "robot_link1_left_r8": 0.278064,
            "robot_link2_left_r3": 0.010834,
            "robot_base_link_r2": 0.317393,
            "robot_base_link_r9": 0.190871,
            "robot_link1_right_r6": 0.277937,
            "robot_link2_right_r7": 0.010836,
            "robot_base_link_r4": 0.317367,
        }
        for name, value in squatted_qpos.items():
            try:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid >= 0:
                    data.qpos[model.jnt_qposadr[jid]] = value
            except Exception:
                pass

        mujoco.mj_forward(model, data)
        a, b = _reduced_balance_system(model, data)
        
        # Restore original state
        data.qpos[:] = orig_qpos
        mujoco.mj_forward(model, data)
        # 单输入 LQR: 合并两轮为一个虚拟输入
        b_sum = b.sum(axis=1, keepdims=True)
        q = np.diag(self.params.q_diag)
        r = np.array([[self.params.r_diag[0]]])
        k_single = _solve_lqr(a, b_sum, q, r)  # (1, 4)

        # 通过 S 矩阵将单输入增益分配到物理执行器空间
        s_matrix = _wheel_control_selection(model)
        # S @ ones(2,1) 给出每个轮子的方向符号
        ones = np.ones((2, 1))
        gain = (s_matrix @ ones) @ k_single  # (nu, 1) @ (1, 4) -> (nu, 4)

        feedforward = np.zeros(model.nu)
        target = balance_tangent_state(model, data, state).copy()
        target[0] = self.params.pitch_trim
        target[1] = 0.0
        target[3] = self.params.target_velocity
        self._lqr_controller = LqrController(gain, target, feedforward, balance_tangent_state)

    def _update_lqr_target(self, state: SimState, dt: float) -> None:
        """每步更新 LQR 目标: 速度跟踪 + pitch 前倾补偿。"""
        if self._lqr_controller is None:
            return
        # 使用方向统一后的平均轮速
        current_wheel_vel = self._forward_wheel_velocity(state)
        velocity_error = self.params.target_velocity - current_wheel_vel
        
        # 积分只在没有饱和时累加，防止积分饱和 (anti-windup)
        # pitch_lean 的输出范围受限，如果触及边界则停止积分
        max_lean = 0.5
        pitch_p = self.params.pitch_lean_gain * velocity_error
        pitch_i = self.params.velocity_ki * self._velocity_integral
        
        pitch_lean = pitch_p + pitch_i
        
        if -max_lean < pitch_lean < max_lean:
            self._velocity_integral += velocity_error * dt
            
        pitch_lean = np.clip(pitch_lean, -max_lean, max_lean)
        self._lqr_controller.target[0] = self.params.pitch_trim - pitch_lean
        
        # 当从静止(stand)切换到运动(drive)时，重置目标位置到当前位置，避免位置环由于累积误差产生巨大的反向力矩
        if abs(self._prev_target_velocity) < 1e-3 and abs(self.params.target_velocity) >= 1e-3:
            self._lqr_controller.target[2] = self._forward_wheel_position(state)
            
        # 积分目标轮子位置，避免位置环变成阻碍前进的"橡胶筋"
        self._lqr_controller.target[2] += self.params.target_velocity * dt
        self._lqr_controller.target[3] = self.params.target_velocity
        
        self._prev_target_velocity = self.params.target_velocity

    def __call__(self, model: Any, data: Any, state: SimState) -> np.ndarray:
        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")

        dt = float(model.opt.timestep)
        self._update_lqr_target(state, dt)

        # 速度 PI 前馈: 取消直接前馈力矩，让 LQR 通过 Pitch 自动调整
        self._lqr_controller.feedforward = np.zeros(model.nu)

        addresses = model_addresses(model)
        lqr_control = self._lqr_controller(model, data, state)
        vmc_control = self.vmc_controller(model, data, state)

        # 将 LQR 两轮输出取方向归一化后平均，消除增益不对称产生的偏航分量
        wheel_indices = []
        wheel_signs = []
        for joint_name in MODEL_SEMANTICS.wheel_joints:
            wheel_indices.append(addresses.actuators[joint_name])
            wheel_signs.append(WHEEL_FORWARD_SIGNS[joint_name])
        # 归一化到前进方向后取平均
        forward_torques = [wheel_signs[i] * lqr_control[wheel_indices[i]] for i in range(2)]
        avg_forward = float(np.mean(forward_torques))

        # yaw PI 控制: P=角速度阻尼, I=消除稳态偏航偏置
        yaw_rate = float(state.base_angular_velocity[2])
        self._yaw_integral += yaw_rate * model.opt.timestep
        self._yaw_integral = np.clip(self._yaw_integral, -5.0, 5.0)
        yaw_correction = -(self.params.yaw_damping * yaw_rate
                           + self.params.yaw_ki * self._yaw_integral)

        control = np.zeros(model.nu)
        for i, joint_name in enumerate(MODEL_SEMANTICS.wheel_joints):
            actuator_index = addresses.actuators[joint_name]
            sign = WHEEL_FORWARD_SIGNS[joint_name]
            # 差速偏航修正：左侧减力、右侧加力实现反向偏航修正
            diff = yaw_correction * (-1.0 if i == 0 else 1.0)
            control[actuator_index] = sign * (avg_forward + diff)
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = vmc_control[actuator_index]

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if not np.all(np.isfinite(clipped)):
            raise ValueError("combined control must be finite")
        return clipped

    @staticmethod
    def _forward_wheel_position(state: SimState) -> float:
        """计算真实的底盘前进位置，避免腿部折叠带来的关节角伪影。"""
        # 机器人的前进方向是全局 Y 轴
        return float(state.base_position[1])

    @staticmethod
    def _forward_wheel_velocity(state: SimState) -> float:
        """计算真实的底盘前进速度，避免腿部折叠带来的关节角伪影。"""
        # 机器人的前进方向是全局 Y 轴
        return float(state.base_linear_velocity[1])
