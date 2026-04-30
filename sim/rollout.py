from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from sim.metrics import RolloutMetrics
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import SimState, extract_sim_state


Controller = Callable[[mujoco.MjModel, mujoco.MjData, SimState], np.ndarray]


@dataclass(frozen=True)
class RolloutConfig:
    duration: float
    scenario: str = "stand"
    cache_dir: Path | None = None
    fall_height: float = 0.05


@dataclass(frozen=True)
class RolloutResult:
    model: mujoco.MjModel
    data: mujoco.MjData
    metrics: RolloutMetrics
    steps: int
    initial_base_position: np.ndarray
    last_control: np.ndarray
    failure_reason: str | None = None


def zero_controller(model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
    del data, state
    return np.zeros(model.nu)


def run_rollout(xml_path: Path, config: RolloutConfig, controller: Controller = zero_controller) -> RolloutResult:
    model_path = prepare_controlled_mujoco_xml(xml_path, cache_dir=config.cache_dir)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    initial_state = extract_sim_state(model, data)
    initial_base_position = initial_state.base_position.copy()
    min_base_height = float(initial_base_position[2])
    max_base_height = float(initial_base_position[2])
    max_abs_pitch = abs(float(initial_state.pitch))
    contact_count = int(data.ncon)
    control_effort = 0.0
    saturated_controls = 0
    total_controls = 0
    last_control = np.zeros(model.nu)
    finite = True
    failure_reason: str | None = None
    executed_steps = 0

    step_count = max(1, int(np.ceil(config.duration / model.opt.timestep)))
    for _ in range(step_count):
        state = extract_sim_state(model, data)
        control = np.asarray(controller(model, data, state), dtype=float)
        if control.shape != (model.nu,):
            finite = False
            failure_reason = "invalid control shape"
            break
        if not np.all(np.isfinite(control)):
            finite = False
            failure_reason = "nonfinite control"
            last_control = control
            break

        clipped_control = _clip_control(model, control)
        control_effort += float(np.sum(np.square(clipped_control)))
        saturated_controls += int(np.count_nonzero(np.abs(clipped_control - control) > 1e-12))
        total_controls += model.nu
        data.ctrl[:] = clipped_control
        last_control = clipped_control.copy()
        mujoco.mj_step(model, data)
        executed_steps += 1

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            finite = False
            failure_reason = "nonfinite state"
            break

        state = extract_sim_state(model, data)
        min_base_height = min(min_base_height, float(state.base_position[2]))
        max_base_height = max(max_base_height, float(state.base_position[2]))
        max_abs_pitch = max(max_abs_pitch, abs(float(state.pitch)))
        contact_count = max(contact_count, int(state.contact_count))

    final_state = extract_sim_state(model, data)
    saturation_ratio = saturated_controls / total_controls if total_controls else 0.0
    metrics = RolloutMetrics(
        duration=config.duration,
        finite=finite,
        fell=bool(min_base_height < config.fall_height),
        min_base_height=min_base_height,
        max_base_height=max_base_height,
        max_jump_height=max(0.0, max_base_height - float(initial_base_position[2])),
        max_abs_pitch=max_abs_pitch,
        forward_distance=float(final_state.base_position[0] - initial_base_position[0]),
        contact_count=contact_count,
        control_effort=control_effort,
        saturation_ratio=saturation_ratio,
    )
    return RolloutResult(
        model=model,
        data=data,
        metrics=metrics,
        steps=executed_steps,
        initial_base_position=initial_base_position,
        last_control=last_control,
        failure_reason=failure_reason,
    )


def _clip_control(model: mujoco.MjModel, control: np.ndarray) -> np.ndarray:
    if model.nu == 0:
        return control
    return np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
