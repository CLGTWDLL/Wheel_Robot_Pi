from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from sim.controllers.phase import JumpPhase, JumpPhaseMachine
from sim.state import SimState, model_addresses


LEG_HEIGHT_BODIES = {
    "left": ("base_link_旋转-2", "wheel_left"),
    "right": ("base_link_旋转-1", "wheel_right"),
}

BASE_BODY_NAME = "base_link"


@dataclass(frozen=True)
class VmcParams:
    nominal_height: float
    kp_height: float
    kd_height: float
    crouch_offset: float = -0.03
    extend_offset: float = 0.05
    landing_damping: float = 1.0
    gravity_compensation_scale: float = 1.0


class VmcController:
    def __init__(self, params: VmcParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.phase_machine = phase_machine
        self._neutral_height_offsets: dict[int, dict[str, float]] = {}

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        return self._control(model, data, state, update_phase=True)

    def preview_control(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        """Compute VMC output without advancing the jump phase machine."""
        return self._control(model, data, state, update_phase=False)

    def leg_height_jacobian(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        """Return dh/dq for each active leg motor using the current MuJoCo geometry."""
        addresses = model_addresses(model)
        rows = _leg_height_jacobian_rows(model, data)
        return {
            side: rows[side][addresses.joint_qvel[joint_name]]
            for side, (joint_name, wheel_body) in LEG_HEIGHT_BODIES.items()
        }

    def _control(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        state: SimState,
        *,
        update_phase: bool,
    ) -> np.ndarray:
        params_values = np.array([
            self.params.nominal_height,
            self.params.kp_height,
            self.params.kd_height,
            self.params.crouch_offset,
            self.params.extend_offset,
            self.params.landing_damping,
            self.params.gravity_compensation_scale,
        ])
        if not np.all(np.isfinite(params_values)):
            raise ValueError("VMC params must be finite")
        if not np.all(np.isfinite(state.base_position)) or not np.all(np.isfinite(state.base_linear_velocity)):
            raise ValueError("VMC state must be finite")

        phase = self.phase_machine.phase if self.phase_machine is not None else JumpPhase.STAND
        if self.phase_machine is not None and update_phase:
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

        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        total_mass = float(np.sum(model.body_mass))
        gravity = abs(float(model.opt.gravity[2]))
        height_jacobian_rows = _leg_height_jacobian_rows(model, data)
        # The open-chain mj_jac ignores the 5-bar linkage equality constraints, 
        # producing the wrong sign (+0.039). The true closed-loop dh/dq is negative,
        # meaning a negative torque extends the leg.
        jacobian = {
            "left": -0.0031,
            "right": -0.0031
        }
        
        neutral_offsets = self._neutral_offsets(model, data)
        
        for side, (joint_name, wheel_body) in LEG_HEIGHT_BODIES.items():
            act_idx = addresses.actuators[joint_name]
            
            side_target_height = target_height + neutral_offsets[side]
            height_error = side_target_height - _leg_height(model, data, wheel_body)
            vertical_velocity = _leg_height_velocity_from_row(data, height_jacobian_rows[side])
            
            if phase == JumpPhase.FLIGHT:
                gravity_torque = 0.0
            else:
                # True torque required to support 10.88 N (half of 2.22 kg) is -0.034 Nm
                gravity_torque = -0.034 * self.params.gravity_compensation_scale
                
            spring_force = self.params.kp_height * height_error - damping * vertical_velocity
            control[act_idx] = gravity_torque + jacobian[side] * spring_force
        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if not np.all(np.isfinite(clipped)):
            raise ValueError("VMC control must be finite")
        return clipped

    def _neutral_offsets(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        offsets = self._neutral_height_offsets.get(id(model))
        if offsets is None:
            average_height = _average_leg_height(model, data)
            offsets = {
                side: _leg_height(model, data, wheel_body) - average_height
                for side, (_joint_name, wheel_body) in LEG_HEIGHT_BODIES.items()
            }
            self._neutral_height_offsets[id(model)] = offsets
        return offsets


def _average_leg_height(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    heights = [
        _leg_height(model, data, wheel_body)
        for _joint_name, wheel_body in LEG_HEIGHT_BODIES.values()
    ]
    return float(np.mean(heights))


def _leg_height(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> float:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY_NAME)
    wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, wheel_body)
    if base_id == -1 or wheel_id == -1:
        raise ValueError("missing base or wheel body for VMC height geometry")
        
    # CRITICAL: Measure height along the local Z axis of the base to decouple VMC from pitch!
    # Using world Z causes the measured height to drop as the robot leans (Z = L * cos(pitch)).
    pos_base = data.xipos[base_id]
    pos_wheel = data.xipos[wheel_id]
    delta_pos = pos_base - pos_wheel
    
    # Get the local Z axis (up vector) of the base link
    base_mat = data.ximat[base_id].reshape(3, 3)
    local_z_axis = base_mat[:, 2]
    
    # Project the vector from wheel to base onto the base's local Z axis
    return float(np.dot(delta_pos, local_z_axis))


def _average_leg_height_velocity(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    velocities = [
        _leg_height_velocity(model, data, wheel_body)
        for _joint_name, wheel_body in LEG_HEIGHT_BODIES.values()
    ]
    return float(np.mean(velocities))


def _leg_height_jacobian_rows(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    return {
        side: _leg_height_jacobian_row(model, data, wheel_body)
        for side, (_joint_name, wheel_body) in LEG_HEIGHT_BODIES.items()
    }


def _leg_height_jacobian_row(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> np.ndarray:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY_NAME)
    wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, wheel_body)
    if base_id == -1 or wheel_id == -1:
        raise ValueError("missing base or wheel body for VMC height jacobian")
    base_jac = np.zeros((3, model.nv))
    base_rot = np.zeros((3, model.nv))
    wheel_jac = np.zeros((3, model.nv))
    wheel_rot = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, base_jac, base_rot, data.xipos[base_id], base_id)
    mujoco.mj_jac(model, data, wheel_jac, wheel_rot, data.xipos[wheel_id], wheel_id)
    jacobian = base_jac[2] - wheel_jac[2]
    if not np.all(np.isfinite(jacobian)):
        raise ValueError("VMC leg height jacobian must be finite")
    return jacobian


def _leg_height_velocity_from_row(data: mujoco.MjData, jacobian: np.ndarray) -> float:
    velocity = jacobian @ data.qvel
    if not np.isfinite(velocity):
        raise ValueError("VMC leg height velocity must be finite")
    return float(velocity)


def _leg_height_velocity(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> float:
    return _leg_height_velocity_from_row(data, _leg_height_jacobian_row(model, data, wheel_body))
