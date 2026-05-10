#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections.abc import Sequence
import datetime
import os
import sys
import time
from pathlib import Path
from typing import Any

# 确保项目根目录在 sys.path 中，使 sim 包可被正确导入
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from sim.controllers.combined import CombinedController
from sim.controllers.default_params import STAND_PARAMS
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcController
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml
from sim.rollout import Controller, _clip_control, zero_controller
from sim.state import extract_sim_state
from sim.logger import setup_system_logger, TelemetryLogger, cleanup_old_logs


HOME_KEYFRAME = "link23_connected_stand"
CMD_SLIDER_NAMES = ("cmd_linear_x", "cmd_angular_z")


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
    default_xml = sim_root / "robot" / "robot.urdf"

    parser = argparse.ArgumentParser(
        description="Launch MuJoCo simulation with the robot XML model."
    )
    parser.add_argument(
        "--xml",
        type=Path,
        default=default_xml,
        help="Path to a URDF model file (converted to MJCF automatically).",
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
    parser.add_argument(
        "--show-collision",
        action="store_true",
        help="Hide visual meshes and show collision primitives for verification.",
    )
    return parser.parse_args(argv)


def apply_home_keyframe(model: Any, data: Any, mujoco_module: Any) -> None:
    key_id = mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
    if key_id != -1:
        mujoco_module.mj_resetDataKeyframe(model, data, key_id)


def build_controlled_model(xml_path: Path, cache_dir: Path | None = None) -> tuple[Any, Any]:
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
    model: Any,
    data: Any,
    controller: Controller, 
    telemetry_logger: TelemetryLogger | None = None,
    target_info: str = "",
) -> bool:
    import mujoco

    state = extract_sim_state(model, data)
    control = np.asarray(controller(model, data, state), dtype=float)
    if control.shape != (model.nu,):
        raise ValueError(f"controller returned shape {control.shape}, expected {(model.nu,)}")

    clipped_control = _clip_control(model, control)
    if telemetry_logger is not None:
        telemetry_logger.log_step(data.time, state, target_info, clipped_control)

    if not np.all(np.isfinite(control)) or not np.all(np.isfinite(clipped_control)):
        _write_physical_ctrl(model, data, clipped_control)
        return False

    _write_physical_ctrl(model, data, clipped_control)
    mujoco.mj_step(model, data)
    return bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))


def _write_physical_ctrl(model: Any, data: Any, control: np.ndarray) -> None:
    """写入真实执行器控制量，保留 command slider 的 ctrl 值。"""
    import mujoco
    command_values: dict[int, float] = {}
    for name in CMD_SLIDER_NAMES:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id >= 0:
            command_values[act_id] = float(data.ctrl[act_id])
    data.ctrl[:] = control
    for act_id, value in command_values.items():
        data.ctrl[act_id] = value


def _read_cmd_sliders(model: Any, data: Any) -> tuple[float, float]:
    """读取 cmd_linear_x / cmd_angular_z 滑块的当前 ctrl 值。"""
    import mujoco
    result = [0.0, 0.0]
    for i, name in enumerate(CMD_SLIDER_NAMES):
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id >= 0:
            result[i] = float(data.ctrl[act_id])
    return result[0], result[1]


def _restore_cmd_sliders(model: Any, data: Any, linear_x: float, angular_z: float) -> None:
    """控制器 step 会覆盖 data.ctrl，这里恢复滑块值使其持续生效。"""
    import mujoco
    for name, val in zip(CMD_SLIDER_NAMES, (linear_x, angular_z)):
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id >= 0:
            data.ctrl[act_id] = val


def run_controlled_viewer_loop(
    model: Any,
    data: Any,
    controller: Controller,
    viewer: Any,
    telemetry_logger: TelemetryLogger | None = None,
) -> None:
    """以实时速度运行 viewer 仿真循环。

    每帧计算需要追赶的仿真步数，使仿真时间与墙钟时间同步。
    同时读取 cmd_linear_x / cmd_angular_z 滑块驱动控制器。
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
                # 读取用户滑块指令
                linear_x, angular_z = _read_cmd_sliders(model, data)
                if isinstance(controller, CombinedController):
                    controller.params.target_velocity = linear_x
                    controller.params.target_yaw_rate = angular_z

                target_info = ""
                params = getattr(controller, "params", None)
                target_velocity = getattr(params, "target_velocity", None)
                if target_velocity is not None:
                    target_info = f"v={target_velocity:.2f}"
                vmc = getattr(params, "vmc", None)
                nominal_height = getattr(vmc, "nominal_height", None)
                if nominal_height is not None:
                    target_info += f",h={nominal_height:.3f}"
                        
                finite = step_controlled_model(model, data, controller, telemetry_logger, target_info)
                # 恢复滑块值（step 会覆盖 data.ctrl）
                _restore_cmd_sliders(model, data, linear_x, angular_z)
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

    def apply_viewer_settings(v: Any) -> None:
        if args.show_collision:
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = True
            v.opt.geomgroup[1] = 0 # Hide visual
            v.opt.geomgroup[3] = 1 # Show collision

    if args.mode == "viewer":
        prepared_xml = prepare_controlled_mujoco_xml(args.xml) if args.show_collision else prepare_mujoco_xml(args.xml)
        model = mujoco.MjModel.from_xml_path(str(prepared_xml))
        data = mujoco.MjData(model)
        apply_home_keyframe(model, data, mujoco)
        mujoco.mj_forward(model, data)
        system_logger.info("Viewer mode started. No telemetry will be recorded.")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            apply_viewer_settings(viewer)
            import time
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)
        return

    model, data = build_controlled_model(args.xml, cache_dir=args.cache_dir)
    controller = create_controlled_controller(args.controller, args.scenario)

    system_logger.info("Model built and controller created. Starting simulation loop.")

    with TelemetryLogger(log_dir / "telemetry.csv") as telemetry_logger:
        if args.scenario == "drive" and isinstance(controller, CombinedController):
            with mujoco.viewer.launch_passive(model, data) as viewer:
                apply_viewer_settings(viewer)
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
                apply_viewer_settings(viewer)
                run_controlled_viewer_loop(model, data, controller, viewer, telemetry_logger)
                
    system_logger.info("Simulation finished. Logs saved.")


if __name__ == "__main__":
    main()
