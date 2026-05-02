#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections.abc import Sequence
import os
import sys
from pathlib import Path

# 确保项目根目录在 sys.path 中，使 sim 包可被正确导入
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from sim.controllers.combined import CombinedController, CombinedParams
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcController, VmcParams
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml
from sim.rollout import Controller, _clip_control, zero_controller
from sim.state import extract_sim_state


HOME_KEYFRAME = "link23_connected_home"


def ensure_dependencies() -> None:
    missing_packages: list[str] = []
    for module_name, package_name in (
        ("mujoco", "mujoco"),
    ):
        try:
            __import__(module_name)
        except ModuleNotFoundError:
            missing_packages.append(package_name)

    if not missing_packages:
        return

    if os.environ.get("MOJOCO_LQR_UV_BOOTSTRAP") == "1":
        raise ModuleNotFoundError(
            "Missing dependencies: "
            + ", ".join(missing_packages)
            + ". Install them with: uv run"
            + " ".join(f" --with {package}" for package in missing_packages)
            + " python launch_mujoco.py"
        )

    uv_command = ["uv", "run"]
    for package_name in missing_packages:
        uv_command.extend(["--with", package_name])
    uv_command.extend(["python", str(Path(__file__).resolve()), *sys.argv[1:]])

    os.environ["MOJOCO_LQR_UV_BOOTSTRAP"] = "1"
    os.execvp("uv", uv_command)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    sim_root = Path(__file__).resolve().parent
    default_xml = sim_root / "轮腿v1" / "轮腿v1.xml"

    parser = argparse.ArgumentParser(
        description="Launch MuJoCo simulation with the robot XML model."
    )
    parser.add_argument(
        "--xml",
        type=Path,
        default=default_xml,
        help="Path to a MuJoCo XML model file.",
    )
    parser.add_argument(
        "--mode",
        choices=("viewer", "controlled"),
        default="controlled",
        help="Launch the original viewer-only model or the controlled viewer loop.",
    )
    parser.add_argument(
        "--controller",
        choices=("zero", "vmc", "lqr_vmc"),
        default="lqr_vmc",
        help="Controller to use in controlled mode.",
    )
    parser.add_argument(
        "--scenario",
        choices=("stand", "jump"),
        default="stand",
        help="Controlled simulation scenario.",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        help="Optional directory for generated controlled XML and preprocessed meshes.",
    )
    return parser.parse_args(argv)


def apply_home_keyframe(model: object, data: object, mujoco_module: object) -> None:
    key_id = mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
    if key_id != -1:
        mujoco_module.mj_resetDataKeyframe(model, data, key_id)


def build_controlled_model(xml_path: Path, cache_dir: Path | None = None) -> tuple[object, object]:
    import mujoco

    prepared_xml = prepare_controlled_mujoco_xml(xml_path, cache_dir=cache_dir)
    model = mujoco.MjModel.from_xml_path(str(prepared_xml))
    data = mujoco.MjData(model)
    apply_home_keyframe(model, data, mujoco)
    mujoco.mj_forward(model, data)
    return model, data


def create_controlled_controller(controller_name: str, scenario: str) -> Controller:
    if controller_name == "zero":
        return zero_controller

    phase_machine = None
    if scenario == "jump":
        phase_machine = JumpPhaseMachine(JumpPhaseParams())
        phase_machine.start_jump()
    elif scenario != "stand":
        raise ValueError(f"unsupported scenario: {scenario}")

    vmc_params = VmcParams(nominal_height=0.06078, kp_height=1.0, kd_height=0.2)
    if controller_name == "vmc":
        return VmcController(vmc_params, phase_machine=phase_machine)
    if controller_name == "lqr_vmc":
        return CombinedController(
            CombinedParams(
                vmc=vmc_params,
                q_diag=np.array([100.0, 10.0, 1.0, 1.0]),
                r_diag=np.array([1.0, 1.0]),
            ),
            phase_machine=phase_machine,
        )
    raise ValueError(f"unsupported controller: {controller_name}")


def step_controlled_model(model: object, data: object, controller: Controller) -> bool:
    import mujoco

    state = extract_sim_state(model, data)
    control = np.asarray(controller(model, data, state), dtype=float)
    if control.shape != (model.nu,):
        raise ValueError(f"controller returned shape {control.shape}, expected {(model.nu,)}")
    if not np.all(np.isfinite(control)):
        data.ctrl[:] = control
        return False

    data.ctrl[:] = _clip_control(model, control)
    mujoco.mj_step(model, data)
    return bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))


def run_controlled_viewer_loop(
    model: object,
    data: object,
    controller: Controller,
    viewer: object,
) -> None:
    while viewer.is_running():
        with viewer.lock():
            finite = step_controlled_model(model, data, controller)
        if not finite:
            break
        viewer.sync()


def main() -> None:
    ensure_dependencies()
    args = parse_args()

    import mujoco
    import mujoco.viewer

    if args.mode == "viewer":
        prepared_xml = prepare_mujoco_xml(args.xml)
        model = mujoco.MjModel.from_xml_path(str(prepared_xml))
        data = mujoco.MjData(model)
        apply_home_keyframe(model, data, mujoco)

        # Launch a MuJoCo scene and run simulation until the window is closed.
        mujoco.viewer.launch(model, data)
        return

    model, data = build_controlled_model(args.xml, cache_dir=args.cache_dir)
    controller = create_controlled_controller(args.controller, args.scenario)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        run_controlled_viewer_loop(model, data, controller, viewer)


if __name__ == "__main__":
    main()
