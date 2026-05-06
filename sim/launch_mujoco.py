#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections.abc import Sequence
import datetime
import os
import sys
import time
from pathlib import Path

# 确保项目根目录在 sys.path 中，使 sim 包可被正确导入
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from sim.controllers.combined import CombinedController
from sim.controllers.default_params import STAND_PARAMS, STAND_THEN_DRIVE_PARAMS
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcController
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml
from sim.rollout import Controller, _clip_control, zero_controller
from sim.state import extract_sim_state
from sim.logger import setup_system_logger, TelemetryLogger, cleanup_old_logs


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
        choices=("stand", "jump", "drive"),
        default="stand",
        help="Controlled simulation scenario: stand, jump, or drive.",
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
    elif scenario not in ("stand", "drive"):
        raise ValueError(f"unsupported scenario: {scenario}")

    if controller_name == "vmc":
        return VmcController(STAND_PARAMS.vmc, phase_machine=phase_machine)
    if controller_name == "lqr_vmc":
        if scenario == "drive":
            # STAND_THEN_DRIVE_PARAMS 基于旧的 WHEEL_FORWARD_SIGNS 优化, 待重新优化
            return CombinedController(STAND_PARAMS, phase_machine=phase_machine)
        return CombinedController(STAND_PARAMS, phase_machine=phase_machine)
    raise ValueError(f"unsupported controller: {controller_name}")


def step_controlled_model(
    model: object, 
    data: object, 
    controller: Controller, 
    telemetry_logger: TelemetryLogger | None = None,
    target_info: str = "",
) -> bool:
    import mujoco

    state = extract_sim_state(model, data)
    control = np.asarray(controller(model, data, state), dtype=float)
    
    if telemetry_logger is not None:
        telemetry_logger.log_step(data.time, state, target_info, control)
        
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
    telemetry_logger: TelemetryLogger | None = None,
) -> None:
    """以实时速度运行 viewer 仿真循环。

    每帧计算需要追赶的仿真步数，使仿真时间与墙钟时间同步。
    """
    wall_start = time.perf_counter()
    sim_start = data.time
    while viewer.is_running():
        # 计算墙钟时间对应的目标仿真时间
        wall_elapsed = time.perf_counter() - wall_start
        target_sim_time = sim_start + wall_elapsed
        # 步进仿真直到追上目标时间
        with viewer.lock():
            while data.time < target_sim_time:
                target_info = ""
                if hasattr(controller, 'params'):
                    if hasattr(controller.params, 'target_velocity'):
                        target_info = f"v={controller.params.target_velocity:.2f}"
                    if hasattr(controller.params, 'vmc') and hasattr(controller.params.vmc, 'nominal_height'):
                        target_info += f",h={controller.params.vmc.nominal_height:.3f}"
                        
                finite = step_controlled_model(model, data, controller, telemetry_logger, target_info)
                if not finite:
                    return
        viewer.sync()


def _drive_velocity_profile(t: float) -> float:
    """梯形速度曲线: 静止→急加速→匀速→急减速→静止→循环。

    周期 10s: 0-1s 静止, 1-2s 加速到 0.3 m/s, 2-7s 匀速, 7-8s 减速, 8-10s 静止。
    """
    t_mod = t % 10.0
    if t_mod < 1.0:
        return 0.0
    elif t_mod < 2.0:
        return (t_mod - 1.0) * 0.3
    elif t_mod < 7.0:
        return 0.3
    elif t_mod < 8.0:
        return (8.0 - t_mod) * 0.3
    else:
        return 0.0


def main() -> None:
    ensure_dependencies()
    args = parse_args()

    # 初始化日志系统
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    manual_log_base = Path("logs/manual")
    log_dir = manual_log_base / f"run_{timestamp}"
    system_logger = setup_system_logger(log_dir / "system.log")
    
    # 清理旧的日志记录，只保留最近 4 次
    cleanup_old_logs(manual_log_base, max_keep=4)
    
    system_logger.info("="*50)
    system_logger.info(f"Starting launch_mujoco in mode: {args.mode}")
    system_logger.info(f"Scenario: {args.scenario}, Controller: {args.controller}")
    system_logger.info(f"XML Path: {args.xml}")

    import mujoco
    import mujoco.viewer

    if args.mode == "viewer":
        prepared_xml = prepare_mujoco_xml(args.xml)
        model = mujoco.MjModel.from_xml_path(str(prepared_xml))
        data = mujoco.MjData(model)
        apply_home_keyframe(model, data, mujoco)
        system_logger.info("Viewer mode started. No telemetry will be recorded.")
        mujoco.viewer.launch(model, data)
        return

    model, data = build_controlled_model(args.xml, cache_dir=args.cache_dir)
    controller = create_controlled_controller(args.controller, args.scenario)

    system_logger.info("Model built and controller created. Starting simulation loop.")

    with TelemetryLogger(log_dir / "telemetry.csv") as telemetry_logger:
        if args.scenario == "drive" and isinstance(controller, CombinedController):
            with mujoco.viewer.launch_passive(model, data) as viewer:
                wall_start = time.perf_counter()
                sim_start = data.time
                while viewer.is_running():
                    wall_elapsed = time.perf_counter() - wall_start
                    target_sim_time = sim_start + wall_elapsed
                    with viewer.lock():
                        while data.time < target_sim_time:
                            controller.params.target_velocity = _drive_velocity_profile(data.time)
                            
                            target_info = f"v={controller.params.target_velocity:.2f}"
                            if hasattr(controller.params, 'vmc') and hasattr(controller.params.vmc, 'nominal_height'):
                                target_info += f",h={controller.params.vmc.nominal_height:.3f}"
                                
                            finite = step_controlled_model(model, data, controller, telemetry_logger, target_info)
                            if not finite:
                                system_logger.warning("Simulation stopped due to non-finite state or control.")
                                break
                        else:
                            viewer.sync()
                            continue
                        break
        else:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_controlled_viewer_loop(model, data, controller, viewer, telemetry_logger)
                
    system_logger.info("Simulation finished. Logs saved.")


if __name__ == "__main__":
    main()
