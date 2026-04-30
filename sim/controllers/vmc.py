from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from sim.controllers.phase import JumpPhase, JumpPhaseMachine
from sim.model_semantics import MODEL_SEMANTICS
from sim.state import SimState, model_addresses


LEG_EFFORT_SIGNS = {
    "robot_base_link_r1": -1.0,
    "robot_base_link_r2": -1.0,
    "robot_base_link_r4": 1.0,
    "robot_base_link_r9": 1.0,
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
        del data
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
        effort = self.params.kp_height * height_error - damping * vertical_velocity
        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            control[addresses.actuators[joint_name]] = LEG_EFFORT_SIGNS[joint_name] * effort
        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if not np.all(np.isfinite(clipped)):
            raise ValueError("VMC control must be finite")
        return clipped
