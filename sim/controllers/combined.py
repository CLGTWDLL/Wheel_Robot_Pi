from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from sim.controllers.balance_lqr import compute_balance_lqr_gain
from sim.controllers.balance_state import balance_tangent_state
from sim.controllers.lqr import LqrController
from sim.controllers.phase import JumpPhaseMachine
from sim.controllers.vmc import VmcController, VmcParams
from sim.model_semantics import MODEL_SEMANTICS
from sim.state import SimState, model_addresses


@dataclass(frozen=True)
class CombinedParams:
    vmc: VmcParams
    q_diag: np.ndarray
    r_diag: np.ndarray
    target_velocity: float = 0.0


class CombinedController:
    def __init__(self, params: CombinedParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.vmc_controller = VmcController(params.vmc, phase_machine)
        self._lqr_controller: LqrController | None = None

    @property
    def lqr_controller(self) -> LqrController | None:
        return self._lqr_controller

    def _initialize_lqr(self, model: Any, data: Any, state: SimState) -> None:
        if not np.isfinite(self.params.target_velocity):
            raise ValueError("target_velocity must be finite")

        wheel_gain = compute_balance_lqr_gain(model, data, self.params.q_diag, self.params.r_diag)
        gain = np.zeros((model.nu, 4))
        addresses = model_addresses(model)
        for row, joint_name in enumerate(MODEL_SEMANTICS.wheel_joints):
            gain[addresses.actuators[joint_name], :] = wheel_gain[row, :]

        feedforward = np.zeros(model.nu)
        target = balance_tangent_state(model, data, state).copy()
        target[0] = 0.0
        target[1] = 0.0
        target[3] = self.params.target_velocity
        self._lqr_controller = LqrController(gain, target, feedforward, balance_tangent_state)

    def __call__(self, model: Any, data: Any, state: SimState) -> np.ndarray:
        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")

        lqr_control = self._lqr_controller(model, data, state)
        vmc_control = self.vmc_controller(model, data, state)
        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        for joint_name in MODEL_SEMANTICS.wheel_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = lqr_control[actuator_index]
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = vmc_control[actuator_index]

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if not np.all(np.isfinite(clipped)):
            raise ValueError("combined control must be finite")
        return clipped
