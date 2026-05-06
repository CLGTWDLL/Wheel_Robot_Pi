from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from sim.controllers.phase import JumpPhase, JumpPhaseMachine
from sim.model_semantics import MODEL_SEMANTICS
from sim.state import SimState, model_addresses


LEG_EFFORT_SIGNS = {
    "robot_base_link_r1": 1.0,
    "robot_base_link_r9": -1.0,
}


@dataclass(frozen=True)
class VmcParams:
    nominal_height: float
    kp_height: float
    kd_height: float
    crouch_offset: float = -0.03
    extend_offset: float = 0.05
    landing_damping: float = 1.0


class VmcController:
    def __init__(self, params: VmcParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.phase_machine = phase_machine

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        params_values = np.array([
            self.params.nominal_height,
            self.params.kp_height,
            self.params.kd_height,
            self.params.crouch_offset,
            self.params.extend_offset,
            self.params.landing_damping,
        ])
        if not np.all(np.isfinite(params_values)):
            raise ValueError("VMC params must be finite")
        if not np.all(np.isfinite(state.base_position)) or not np.all(np.isfinite(state.base_linear_velocity)):
            raise ValueError("VMC state must be finite")

        phase = self.phase_machine.phase if self.phase_machine is not None else JumpPhase.STAND
        if self.phase_machine is not None:
            self.phase_machine.update(
                dt=float(model.opt.timestep),
                base_height=float(state.base_position[2]),
                vertical_velocity=float(state.base_linear_velocity[2]),
                contact_count=state.contact_count,
            )
            phase = self.phase_machine.phase

        target_height = self.params.nominal_height
        damping = self.params.kd_height
        if phase == JumpPhase.CROUCH:
            target_height += self.params.crouch_offset
        elif phase == JumpPhase.EXTEND:
            target_height += self.params.extend_offset
        elif phase == JumpPhase.LAND:
            damping += self.params.landing_damping
        elif phase == JumpPhase.FAIL:
            return np.zeros(model.nu)

        height_error = target_height - float(state.base_position[2])
        vertical_velocity = float(state.base_linear_velocity[2])
        
        # 预估重力补偿 (如 1.5 Nm) 可以避免静止时产生巨大的 P/D 误差
        gravity_ff = -1.5  # 假设需要大约 1.5 Nm 的力矩来抵抗重力 (负号因为 LEG_EFFORT_SIGNS 是 -1.0)
        
        pd_effort = self.params.kp_height * height_error - damping * vertical_velocity + gravity_ff
        
        # 严格限制 PD 输出，防止瞬间巨大的力矩造成底盘俯仰失控
        pd_effort = np.clip(pd_effort, gravity_ff - 1.0, gravity_ff + 1.0)
        
        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            act_idx = addresses.actuators[joint_name]
            # 纯 PD 控制: qfrc_bias 在 2 电机五连杆中不适用于关节级补偿
            control[act_idx] = LEG_EFFORT_SIGNS[joint_name] * pd_effort
        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if not np.all(np.isfinite(clipped)):
            raise ValueError("VMC control must be finite")
        return clipped
