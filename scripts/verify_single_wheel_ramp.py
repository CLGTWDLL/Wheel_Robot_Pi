from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.controllers.combined import CombinedController
from sim.controllers.default_params import STAND_PARAMS
from sim.controllers.vmc import LEG_CLOSED_LOOP
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import actuator_id, body_id, extract_sim_state


@dataclass(frozen=True)
class RampMetrics:
    steps: int
    finite: bool
    fell: bool
    failure_reason: str | None
    forward_distance: float
    max_abs_pitch: float
    max_abs_roll: float
    max_base_z_delta: float
    max_wheel_height_delta: float
    contact_count: int
    saturation_ratio: float


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a headless single-wheel trapezoid ramp VMC stability check.")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--target-velocity", type=float, default=0.12)
    parser.add_argument("--terrain-side", choices=("left", "right"), default="left")
    args = parser.parse_args()

    tmp_root = Path("tmp")
    tmp_root.mkdir(exist_ok=True)
    temp_dir = Path(tempfile.mkdtemp(prefix="single_wheel_ramp_", dir=tmp_root))
    try:
        metrics = run_single_wheel_ramp(
            duration=args.duration,
            target_velocity=args.target_velocity,
            terrain_side=args.terrain_side,
            cache_dir=temp_dir,
        )
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    print(f"steps={metrics.steps}")
    print(f"finite={metrics.finite}")
    print(f"fell={metrics.fell}")
    print(f"failure_reason={metrics.failure_reason}")
    print(f"forward_distance={metrics.forward_distance:.4f} m")
    print(f"max_abs_pitch={metrics.max_abs_pitch:.4f} rad")
    print(f"max_abs_roll={metrics.max_abs_roll:.4f} rad")
    print(f"max_base_z_delta={metrics.max_base_z_delta:.4f} m")
    print(f"max_wheel_height_delta={metrics.max_wheel_height_delta:.4f} m")
    print(f"contact_count={metrics.contact_count}")
    print(f"saturation_ratio={metrics.saturation_ratio:.4f}")
    return 0 if metrics.finite and not metrics.fell else 1


def run_single_wheel_ramp(
    *,
    duration: float,
    target_velocity: float,
    terrain_side: str,
    cache_dir: Path,
) -> RampMetrics:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=cache_dir,
        terrain="single_wheel_trapezoid",
        terrain_side=terrain_side,
    )
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    params = replace(
        STAND_PARAMS,
        vmc=replace(STAND_PARAMS.vmc),
        q_diag=STAND_PARAMS.q_diag.copy(),
        r_diag=STAND_PARAMS.r_diag.copy(),
        target_velocity=float(target_velocity),
    )
    controller = CombinedController(params)
    controller.params.target_velocity = float(target_velocity)
    linear_cmd_act = actuator_id(model, "cmd_linear_x")
    height_cmd_act = actuator_id(model, "cmd_height")

    initial_state = extract_sim_state(model, data)
    initial_x = float(initial_state.base_position[0])
    initial_z = float(initial_state.base_position[2])
    max_abs_pitch = abs(float(initial_state.pitch))
    max_abs_roll = abs(float(initial_state.roll))
    max_base_z_delta = 0.0
    max_wheel_height_delta = _wheel_height_delta(model, data)
    contact_count = int(data.ncon)
    saturated_controls = 0
    total_controls = 0
    finite = True
    failure_reason: str | None = None
    executed_steps = 0

    step_count = max(1, int(np.ceil(float(duration) / float(model.opt.timestep))))
    for _ in range(step_count):
        state = extract_sim_state(model, data)
        control = np.asarray(controller(model, data, state), dtype=float)
        if control.shape != (model.nu,) or not np.all(np.isfinite(control)):
            finite = False
            failure_reason = "invalid_control"
            break

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        saturated_controls += int(np.count_nonzero(np.abs(clipped - control) > 1e-12))
        total_controls += int(model.nu)
        data.ctrl[:] = clipped
        if linear_cmd_act >= 0:
            data.ctrl[linear_cmd_act] = float(target_velocity)
        if height_cmd_act >= 0:
            data.ctrl[height_cmd_act] = float(controller.params.vmc.nominal_height)
        mujoco.mj_step(model, data)
        executed_steps += 1

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            finite = False
            failure_reason = "nonfinite_state"
            break

        state = extract_sim_state(model, data)
        elapsed = executed_steps * float(model.opt.timestep)
        max_abs_pitch = max(max_abs_pitch, abs(float(state.pitch)))
        max_abs_roll = max(max_abs_roll, abs(float(state.roll)))
        max_base_z_delta = max(max_base_z_delta, abs(float(state.base_position[2]) - initial_z))
        max_wheel_height_delta = max(max_wheel_height_delta, _wheel_height_delta(model, data))
        contact_count = max(contact_count, int(state.contact_count))
        if elapsed > 1.0 and state.base_position[2] < initial_z - 0.15:
            failure_reason = "fell"
            break
        if abs(float(state.pitch)) > 1.5 or abs(float(state.roll)) > 1.5:
            failure_reason = "excessive_attitude"
            break

    final_state = extract_sim_state(model, data)
    return RampMetrics(
        steps=executed_steps,
        finite=finite,
        fell=failure_reason in {"fell", "excessive_attitude"},
        failure_reason=failure_reason,
        forward_distance=float(final_state.base_position[0] - initial_x),
        max_abs_pitch=max_abs_pitch,
        max_abs_roll=max_abs_roll,
        max_base_z_delta=max_base_z_delta,
        max_wheel_height_delta=max_wheel_height_delta,
        contact_count=contact_count,
        saturation_ratio=(saturated_controls / total_controls) if total_controls else 0.0,
    )


def _wheel_height_delta(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    heights = [
        float(data.xipos[body_id(model, geometry.wheel_body), 2])
        for geometry in LEG_CLOSED_LOOP.values()
    ]
    return abs(heights[0] - heights[1])


if __name__ == "__main__":
    raise SystemExit(main())
