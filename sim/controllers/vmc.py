from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from sim.controllers.leg_height_lut import LegHeightLUT
from sim.controllers.phase import JumpPhase, JumpPhaseMachine
from sim.state import SimState, body_id, equality_id, model_addresses

BASE_BODY_NAME = "base_link"
LEG_JACOBIAN_SINGULAR_VALUE_EPS = 1e-8


@dataclass(frozen=True)
class LegClosedLoopGeometry:
    side: str
    motor_joint: str
    passive_joints: tuple[str, str]
    equality_name: str
    wheel_body: str


LEG_CLOSED_LOOP: dict[str, LegClosedLoopGeometry] = {
    "left": LegClosedLoopGeometry(
        side="left",
        motor_joint="base_link_旋转-2",
        passive_joints=("base_link_旋转-4", "link1_left_旋转-6"),
        equality_name="link23_left_connect",
        wheel_body="wheel_left",
    ),
    "right": LegClosedLoopGeometry(
        side="right",
        motor_joint="base_link_旋转-1",
        passive_joints=("base_link_旋转-3", "link1_right_旋转-5"),
        equality_name="link23_right_connect",
        wheel_body="wheel_right",
    ),
}


@dataclass
class VmcParams:
    # nominal_height is runtime-writable so the cmd_height slider can drive VMC.
    nominal_height: float
    # Joint-space PID gains on the active leg motor angles. Combined with the
    # LUT-driven target motor angle, these replace the old task-space height
    # spring which was unstable near the 5-bar singularity.
    kp_motor: float
    kd_motor: float
    crouch_offset: float = -0.03
    crouch_target_height: float = 0.09
    crouch_duration: float = 0.20
    extend_offset: float = 0.05
    landing_damping: float = 1.0
    land_kp_scale: float = 0.5
    land_kd_scale: float = 2.5
    gravity_compensation_scale: float = 1.0
    max_height_rate: float = 0.1
    jump_extend_feedforward: float = 0.0
    jump_extend_feedforward_duration: float = 0.0
    roll_level_kp_height: float = 0.0
    roll_level_kd_height: float = 0.0
    roll_level_offset_limit: float = 0.0
    lut: LegHeightLUT = field(default_factory=LegHeightLUT.from_json)


class VmcController:
    JACOBIAN_REFRESH_PERIOD = 10  # recompute leg motor jacobian every N control steps

    def __init__(self, params: VmcParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.phase_machine = phase_machine
        self._neutral_height_offsets: dict[int, dict[str, float]] = {}
        self._motor_jacobian_cache: dict[str, float] | None = None
        self._height_jacobian_rows_cache: dict[str, np.ndarray] | None = None
        self._motor_jacobian_step_count: int = 0
        self._height_filtered: float | None = None
        self._crouch_start_height: float | None = None
        self._target_motor_angle_prev: dict[str, float] = {}
        self.last_target_motor_rate: dict[str, float] = {side: 0.0 for side in LEG_CLOSED_LOOP}
        self.last_target_heights: dict[str, float] = {side: 0.0 for side in LEG_CLOSED_LOOP}

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        return self._control(model, data, state, update_phase=True)

    def preview_control(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        """Compute VMC output without advancing the jump phase machine."""
        return self._control(model, data, state, update_phase=False)

    def leg_height_jacobian(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        """Return dh/dq for each active leg motor using the current MuJoCo geometry."""
        return {
            side: _closed_loop_leg_motor_jacobian(model, data, geometry)
            for side, geometry in LEG_CLOSED_LOOP.items()
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
            self.params.kp_motor,
            self.params.kd_motor,
            self.params.crouch_offset,
            self.params.crouch_target_height,
            self.params.crouch_duration,
            self.params.extend_offset,
            self.params.landing_damping,
            self.params.land_kp_scale,
            self.params.land_kd_scale,
            self.params.gravity_compensation_scale,
            self.params.max_height_rate,
            self.params.jump_extend_feedforward,
            self.params.jump_extend_feedforward_duration,
            self.params.roll_level_kp_height,
            self.params.roll_level_kd_height,
            self.params.roll_level_offset_limit,
        ])
        if not np.all(np.isfinite(params_values)):
            raise ValueError("VMC params must be finite")
        if not np.all(np.isfinite(state.base_position)) or not np.all(np.isfinite(state.base_linear_velocity)):
            raise ValueError("VMC state must be finite")

        phase = self.phase_machine.phase if self.phase_machine is not None else JumpPhase.STAND
        current_leg_height = _average_leg_height(model, data)
        if self.phase_machine is not None and update_phase:
            self.phase_machine.update(
                dt=float(model.opt.timestep),
                leg_height=current_leg_height,
                vz=float(state.base_linear_velocity[2]),
                contact_count=state.contact_count,
                pitch=float(state.pitch),
            )
            phase = self.phase_machine.phase

        target_height = self._filtered_nominal_height(float(model.opt.timestep))
        kp_motor = self.params.kp_motor
        kd_motor = self.params.kd_motor
        if phase == JumpPhase.CROUCH:
            if self._crouch_start_height is None:
                self._crouch_start_height = current_leg_height
            duration = max(float(self.params.crouch_duration), float(model.opt.timestep))
            progress = float(np.clip(self.phase_machine.time_in_phase / duration if self.phase_machine else 1.0, 0.0, 1.0))
            blend = progress**3 * (10.0 - 15.0 * progress + 6.0 * progress * progress)
            target_height = (1.0 - blend) * self._crouch_start_height + blend * self.params.crouch_target_height
        elif phase == JumpPhase.EXTEND:
            self._crouch_start_height = None
            target_height += self.params.extend_offset
        elif phase == JumpPhase.FLIGHT:
            # 飞行中预收腿到 crouch 高度：缩短 pendulum length，落地冲击转化的
            # pitch 力矩与 L 成正比；同时给 LAND 阶段留出腿伸展行程吸收冲击。
            self._crouch_start_height = None
            target_height = self.params.crouch_target_height
        elif phase == JumpPhase.LAND:
            self._crouch_start_height = None
            kp_motor *= self.params.land_kp_scale
            kd_motor = kd_motor * self.params.land_kd_scale + self.params.landing_damping
        elif phase == JumpPhase.FALLEN:
            self._crouch_start_height = None
            self.last_target_motor_rate = {side: 0.0 for side in LEG_CLOSED_LOOP}
            self.last_target_heights = {side: 0.0 for side in LEG_CLOSED_LOOP}
            return np.zeros(model.nu)
        else:
            self._crouch_start_height = None

        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        total_mass = float(np.sum(model.body_mass))
        gravity = abs(float(model.opt.gravity[2]))
        if self._motor_jacobian_step_count % self.JACOBIAN_REFRESH_PERIOD == 0:
            self._motor_jacobian_cache = {
                side: _closed_loop_leg_motor_jacobian(model, data, geometry)
                for side, geometry in LEG_CLOSED_LOOP.items()
            }
            self._height_jacobian_rows_cache = _leg_height_jacobian_rows(model, data)
        jacobian = self._motor_jacobian_cache
        assert jacobian is not None
        self._motor_jacobian_step_count += 1

        neutral_offsets = self._neutral_offsets(model, data)
        roll_level_offsets = self._roll_level_height_offsets(phase, state)

        for side, geometry in LEG_CLOSED_LOOP.items():
            act_idx = addresses.actuators[geometry.motor_joint]
            motor_qpos_idx = addresses.joint_qpos[geometry.motor_joint]
            motor_qvel_idx = addresses.joint_qvel[geometry.motor_joint]

            side_target_height = target_height + neutral_offsets[side] + roll_level_offsets[side]
            self.last_target_heights[side] = side_target_height
            target_motor_angle = self.params.lut.motor_angle_from_height(side_target_height)
            previous_angle = self._target_motor_angle_prev.get(side)
            if previous_angle is None:
                self.last_target_motor_rate[side] = 0.0
            else:
                self.last_target_motor_rate[side] = (target_motor_angle - previous_angle) / float(model.opt.timestep)
            self._target_motor_angle_prev[side] = target_motor_angle

            theta_current = float(data.qpos[motor_qpos_idx])
            theta_rate = float(data.qvel[motor_qvel_idx])
            pid_torque = kp_motor * (target_motor_angle - theta_current) - kd_motor * theta_rate

            if phase == JumpPhase.FLIGHT:
                gravity_torque = 0.0
            else:
                gravity_torque = (
                    jacobian[side]
                    * total_mass
                    * gravity
                    / len(LEG_CLOSED_LOOP)
                    * self.params.gravity_compensation_scale
                )

            if phase == JumpPhase.EXTEND:
                control[act_idx] = gravity_torque + self.params.jump_extend_feedforward
            else:
                control[act_idx] = gravity_torque + pid_torque

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if phase in (JumpPhase.STAND, JumpPhase.CROUCH, JumpPhase.LAND):
            for geometry in LEG_CLOSED_LOOP.values():
                act_idx = addresses.actuators[geometry.motor_joint]
                clipped[act_idx] = float(np.clip(clipped[act_idx], -3.5, 3.5))
        if not np.all(np.isfinite(clipped)):
            raise ValueError("VMC control must be finite")
        return clipped

    def _filtered_nominal_height(self, dt: float) -> float:
        target = float(self.params.nominal_height)
        if self._height_filtered is None:
            self._height_filtered = target
            return target
        max_step = max(float(self.params.max_height_rate), 0.0) * dt
        delta = float(np.clip(target - self._height_filtered, -max_step, max_step))
        self._height_filtered += delta
        return self._height_filtered

    def _roll_level_height_offsets(self, phase: JumpPhase, state: SimState) -> dict[str, float]:
        if phase != JumpPhase.STAND:
            return {side: 0.0 for side in LEG_CLOSED_LOOP}
        limit = max(float(self.params.roll_level_offset_limit), 0.0)
        offset = (
            float(self.params.roll_level_kp_height) * float(state.roll)
            + float(self.params.roll_level_kd_height) * float(state.roll_rate)
        )
        offset = float(np.clip(offset, -limit, limit))
        return {"left": offset, "right": -offset}

    def _neutral_offsets(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        offsets = self._neutral_height_offsets.get(id(model))
        if offsets is None:
            average_height = _average_leg_height(model, data)
            offsets = {
                side: _leg_height(model, data, geometry.wheel_body) - average_height
                for side, geometry in LEG_CLOSED_LOOP.items()
            }
            self._neutral_height_offsets[id(model)] = offsets
        return offsets


def _average_leg_height(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    heights = [
        _leg_height(model, data, geometry.wheel_body)
        for geometry in LEG_CLOSED_LOOP.values()
    ]
    return float(np.mean(heights))


def _leg_height(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> float:
    base_id = body_id(model, BASE_BODY_NAME)
    wheel_id = body_id(model, wheel_body)
    return float(data.xipos[base_id, 2] - data.xipos[wheel_id, 2])


def _closed_loop_leg_motor_jacobian(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geometry: LegClosedLoopGeometry,
) -> float:
    addresses = model_addresses(model)
    active_dof = addresses.joint_qvel[geometry.motor_joint]
    passive_dofs = [addresses.joint_qvel[name] for name in geometry.passive_joints]

    constraint_jacobian = _connect_constraint_jacobian(model, data, geometry.equality_name)
    active_constraint = constraint_jacobian[:, active_dof : active_dof + 1]
    passive_constraint = constraint_jacobian[:, passive_dofs]

    u, singular_values, vt = np.linalg.svd(passive_constraint, full_matrices=False)
    if singular_values.size == 0 or singular_values[-1] < LEG_JACOBIAN_SINGULAR_VALUE_EPS:
        sigma_min = float(singular_values[-1]) if singular_values.size else 0.0
        raise ValueError(f"closed-loop jacobian singular at {geometry.motor_joint} (sigma_min={sigma_min:.2e})")

    passive_constraint_pinv = vt.T @ np.diag(1.0 / singular_values) @ u.T
    passive_per_active = -passive_constraint_pinv @ active_constraint
    height_jacobian = _leg_height_jacobian_row(model, data, geometry.wheel_body)
    jacobian = float(height_jacobian[active_dof] + height_jacobian[passive_dofs] @ passive_per_active[:, 0])
    if not np.isfinite(jacobian):
        raise ValueError("VMC closed-loop leg motor jacobian must be finite")
    return jacobian


def _connect_constraint_jacobian(model: mujoco.MjModel, data: mujoco.MjData, equality_name: str) -> np.ndarray:
    constraint_id = equality_id(model, equality_name)

    body_a_id = int(model.eq_obj1id[constraint_id])
    body_b_id = int(model.eq_obj2id[constraint_id])
    if body_a_id < 0 or body_b_id < 0:
        raise ValueError(f"equality constraint must connect two bodies: {equality_name}")

    anchor_a = np.asarray(model.eq_data[constraint_id, 0:3], dtype=float)
    anchor_b = np.asarray(model.eq_data[constraint_id, 3:6], dtype=float)
    point_a = _body_local_point_world(data, body_a_id, anchor_a)
    point_b = _body_local_point_world(data, body_b_id, anchor_b)

    jacobian_a = np.zeros((3, model.nv))
    jacobian_b = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacobian_a, None, point_a, body_a_id)
    mujoco.mj_jac(model, data, jacobian_b, None, point_b, body_b_id)
    constraint_jacobian = jacobian_a - jacobian_b
    if not np.all(np.isfinite(constraint_jacobian)):
        raise ValueError("connect constraint jacobian must be finite")
    return constraint_jacobian


def _body_local_point_world(data: mujoco.MjData, body_id_value: int, local_point: np.ndarray) -> np.ndarray:
    body_rotation = data.xmat[body_id_value].reshape(3, 3)
    return data.xpos[body_id_value] + body_rotation @ local_point


def _leg_height_jacobian_rows(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    return {
        side: _leg_height_jacobian_row(model, data, geometry.wheel_body)
        for side, geometry in LEG_CLOSED_LOOP.items()
    }


def _leg_height_jacobian_row(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> np.ndarray:
    base_id = body_id(model, BASE_BODY_NAME)
    wheel_id = body_id(model, wheel_body)
    base_jac = np.zeros((3, model.nv))
    wheel_jac = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, base_jac, None, data.xipos[base_id], base_id)
    mujoco.mj_jac(model, data, wheel_jac, None, data.xipos[wheel_id], wheel_id)
    jacobian = base_jac[2] - wheel_jac[2]
    if not np.all(np.isfinite(jacobian)):
        raise ValueError("VMC leg height jacobian must be finite")
    return jacobian


def _leg_height_velocity_from_row(data: mujoco.MjData, jacobian: np.ndarray) -> float:
    velocity = jacobian @ data.qvel
    if not np.isfinite(velocity):
        raise ValueError("VMC leg height velocity must be finite")
    return float(velocity)
