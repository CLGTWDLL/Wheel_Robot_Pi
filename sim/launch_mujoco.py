#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections.abc import Sequence
from copy import deepcopy
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
from sim.controllers.phase import JumpPhase, JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcController
from sim.gamepad import GamepadCommandMapper, GamepadDevice, open_gamepad
from sim.model_xml import CMD_SLIDER_NAMES, prepare_controlled_mujoco_xml
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml
from sim.rollout import Controller, _clip_control, zero_controller
from sim.state import actuator_id, extract_sim_state, model_addresses
from sim.logger import setup_system_logger, TelemetryLogger, cleanup_old_logs

MANUAL_JUMP_PHASE_PARAMS = JumpPhaseParams(
    crouch_duration=0.20,
    crouch_target_height=0.09,
    extend_duration=0.05,
    extend_timeout=1.00,
    flight_timeout=0.60,
    recover_duration=0.20,
)


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
        description="Launch MuJoCo simulation from the URDF-generated MJCF model."
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
        choices=("zero", "vmc", "lqr_vmc", "combined"),
        default="lqr_vmc",
        help="Controller to use in controlled mode.",
    )
    parser.add_argument(
        "--scenario",
        choices=("stand", "jump", "drive", "fall_recover"),
        default="stand",
        help="Controlled simulation scenario: stand, jump, drive, or fall_recover.",
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
    parser.add_argument(
        "--flat-ground",
        action="store_true",
        help="Disable the default single-wheel trapezoid ramp in controlled mode.",
    )
    parser.add_argument(
        "--terrain-side",
        choices=("left", "right"),
        default="left",
        help="Which wheel starts on the default single-wheel trapezoid ramp.",
    )
    parser.add_argument(
        "--enable-gamepad",
        action="store_true",
        help="Enable hidapi-based gamepad input.",
    )
    return parser.parse_args(argv)


def build_controlled_model(
    xml_path: Path,
    cache_dir: Path | None = None,
    *,
    terrain: str | None = "single_wheel_trapezoid",
    terrain_side: str = "left",
) -> tuple[Any, Any]:
    import mujoco

    prepared_xml = prepare_controlled_mujoco_xml(
        xml_path,
        cache_dir=cache_dir,
        terrain=terrain,
        terrain_side=terrain_side,
    )
    model = mujoco.MjModel.from_xml_path(str(prepared_xml))
    data = mujoco.MjData(model)

    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, stand_id)
    _initialize_cmd_sliders(model, data)
    mujoco.mj_forward(model, data)
    return model, data


def _initialize_cmd_sliders(model: Any, data: Any) -> None:
    slider_defaults = {
        "cmd_linear_x": 0.0,
        "cmd_angular_z": 0.0,
        "cmd_height": float(STAND_PARAMS.vmc.nominal_height),
        "cmd_jump": 0.0,
    }
    for name, value in slider_defaults.items():
        act_id = actuator_id(model, name)
        if act_id >= 0:
            low, high = model.actuator_ctrlrange[act_id]
            data.ctrl[act_id] = float(np.clip(value, low, high))


def create_controlled_controller(controller_name: str, scenario: str) -> Controller:
    if controller_name == "zero":
        return zero_controller

    params = deepcopy(STAND_PARAMS)
    params.vmc.crouch_target_height = 0.09
    params.vmc.crouch_duration = MANUAL_JUMP_PHASE_PARAMS.crouch_duration
    params.vmc.extend_offset = 0.03
    params.vmc.max_height_rate = 100.0
    params.vmc.jump_extend_feedforward = 11.5
    params.vmc.jump_extend_feedforward_duration = MANUAL_JUMP_PHASE_PARAMS.extend_duration

    phase_machine = JumpPhaseMachine(MANUAL_JUMP_PHASE_PARAMS)
    if scenario == "jump":
        phase_machine.start_jump()
    elif scenario not in ("stand", "drive", "fall_recover"):
        raise ValueError(f"unsupported scenario: {scenario}")

    if controller_name == "vmc":
        return VmcController(params.vmc, phase_machine=phase_machine)
    if controller_name in ("lqr_vmc", "combined"):
        return CombinedController(params, phase_machine=phase_machine)
    raise ValueError(f"unsupported controller: {controller_name}")


def apply_controlled_scenario_initial_state(model: Any, data: Any, scenario: str) -> None:
    if scenario != "fall_recover":
        return

    import mujoco

    addresses = model_addresses(model)
    root_qpos = addresses.root_qpos
    pitch_angle = 1.2
    data.qpos[root_qpos + 3 : root_qpos + 7] = np.array(
        [np.cos(-pitch_angle / 2.0), np.sin(-pitch_angle / 2.0), 0.0, 0.0],
        dtype=float,
    )
    data.qvel[addresses.root_qvel : addresses.root_qvel + 6] = 0.0
    mujoco.mj_forward(model, data)


_STEP_PROFILE: dict[str, Any] = {
    "bucket_start_sim_t": None,
    "count": 0,
    "sum_ctrl": 0.0,
    "sum_step": 0.0,
    "sum_log": 0.0,
    "max_ctrl": 0.0,
    "max_step": 0.0,
    "max_log": 0.0,
    "max_ncon": 0,
    "max_abs_pitch": 0.0,
    "max_abs_roll": 0.0,
    "min_pos_z": float("inf"),
    "last_pitch": 0.0,
    "last_roll": 0.0,
    "last_pos_z": 0.0,
    "last_phase": None,
}


def _record_step_profile(
    sim_t: float,
    controller: Controller,
    *,
    ctrl_ms: float,
    step_ms: float,
    log_ms: float,
    ncon: int,
    pitch: float,
    roll: float,
    pos_z: float,
) -> None:
    """收集每步耗时与姿态，每 1.0 sim-second 或 phase 切换时打印一次。"""
    p = _STEP_PROFILE
    if p["bucket_start_sim_t"] is None:
        p["bucket_start_sim_t"] = sim_t

    phase_machine = getattr(getattr(controller, "vmc_controller", controller), "phase_machine", None)
    phase = getattr(getattr(phase_machine, "phase", None), "value", None) if phase_machine is not None else None
    phase_changed = (
        phase is not None
        and p["last_phase"] is not None
        and phase != p["last_phase"]
    )

    p["count"] += 1
    p["sum_ctrl"] += ctrl_ms
    p["sum_step"] += step_ms
    p["sum_log"] += log_ms
    if ctrl_ms > p["max_ctrl"]:
        p["max_ctrl"] = ctrl_ms
    if step_ms > p["max_step"]:
        p["max_step"] = step_ms
    if log_ms > p["max_log"]:
        p["max_log"] = log_ms
    if ncon > p["max_ncon"]:
        p["max_ncon"] = ncon
    abs_pitch = abs(pitch)
    abs_roll = abs(roll)
    if abs_pitch > p["max_abs_pitch"]:
        p["max_abs_pitch"] = abs_pitch
    if abs_roll > p["max_abs_roll"]:
        p["max_abs_roll"] = abs_roll
    if pos_z < p["min_pos_z"]:
        p["min_pos_z"] = pos_z
    p["last_pitch"] = pitch
    p["last_roll"] = roll
    p["last_pos_z"] = pos_z

    elapsed = sim_t - p["bucket_start_sim_t"]
    if elapsed >= 1.0 or phase_changed:
        n = max(p["count"], 1)
        reason = f"phase→{phase}" if phase_changed else "1s"
        print(
            f"[step-profile {reason}] sim_t={sim_t:.3f} n={p['count']} "
            f"ctrl avg/max={p['sum_ctrl']/n:.3f}/{p['max_ctrl']:.3f}ms "
            f"step avg/max={p['sum_step']/n:.3f}/{p['max_step']:.3f}ms "
            f"log avg/max={p['sum_log']/n:.3f}/{p['max_log']:.3f}ms "
            f"ncon_max={p['max_ncon']} "
            f"|pitch_max={p['max_abs_pitch']:.3f} roll_max={p['max_abs_roll']:.3f} z_min={p['min_pos_z']:.3f} "
            f"|last p/r/z={p['last_pitch']:+.3f}/{p['last_roll']:+.3f}/{p['last_pos_z']:.3f}",
            flush=True,
        )
        p["bucket_start_sim_t"] = sim_t
        p["count"] = 0
        p["sum_ctrl"] = 0.0
        p["sum_step"] = 0.0
        p["sum_log"] = 0.0
        p["max_ctrl"] = 0.0
        p["max_step"] = 0.0
        p["max_log"] = 0.0
        p["max_ncon"] = 0
        p["max_abs_pitch"] = 0.0
        p["max_abs_roll"] = 0.0
        p["min_pos_z"] = float("inf")
    if phase is not None:
        p["last_phase"] = phase


def step_controlled_model(
    model: Any,
    data: Any,
    controller: Controller,
    telemetry_logger: TelemetryLogger | None = None,
    target_info: str = "",
) -> bool:
    import mujoco

    state = extract_sim_state(model, data)

    t0 = time.perf_counter()
    control = np.asarray(controller(model, data, state), dtype=float)
    t1 = time.perf_counter()
    if control.shape != (model.nu,):
        raise ValueError(f"controller returned shape {control.shape}, expected {(model.nu,)}")

    clipped_control = _clip_control(model, control)
    t_log0 = time.perf_counter()
    if telemetry_logger is not None:
        telemetry_logger.log_step(data.time, state, target_info, clipped_control)
    t_log1 = time.perf_counter()

    if not np.all(np.isfinite(control)) or not np.all(np.isfinite(clipped_control)):
        _write_physical_ctrl(model, data, clipped_control)
        return False

    _write_physical_ctrl(model, data, clipped_control)
    t_step0 = time.perf_counter()
    mujoco.mj_step(model, data)
    t_step1 = time.perf_counter()

    _record_step_profile(
        data.time,
        controller,
        ctrl_ms=(t1 - t0) * 1000.0,
        step_ms=(t_step1 - t_step0) * 1000.0,
        log_ms=(t_log1 - t_log0) * 1000.0,
        ncon=int(data.ncon),
        pitch=float(state.pitch),
        roll=float(state.roll),
        pos_z=float(state.base_position[2]),
    )

    return bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))


def _write_physical_ctrl(model: Any, data: Any, control: np.ndarray) -> None:
    """写入真实执行器控制量，保留 command slider 的 ctrl 值。"""
    command_values: dict[int, float] = {}
    for name in CMD_SLIDER_NAMES:
        act_id = actuator_id(model, name)
        if act_id >= 0:
            command_values[act_id] = float(data.ctrl[act_id])
    data.ctrl[:] = control
    for act_id, value in command_values.items():
        data.ctrl[act_id] = value


def _read_cmd_sliders(model: Any, data: Any) -> dict[str, float]:
    """读取 cmd_* 滑块的当前 ctrl 值。"""
    result: dict[str, float] = {}
    for name in CMD_SLIDER_NAMES:
        act_id = actuator_id(model, name)
        if act_id >= 0:
            result[name] = float(data.ctrl[act_id])
        else:
            result[name] = 0.0
    return result


def _restore_cmd_sliders(model: Any, data: Any, values: dict[str, float]) -> None:
    """控制器 step 会覆盖 data.ctrl，这里恢复滑块值使其持续生效。"""
    for name, val in values.items():
        act_id = actuator_id(model, name)
        if act_id >= 0:
            data.ctrl[act_id] = val


def _build_gamepad_mapper(model: Any) -> GamepadCommandMapper | None:
    """根据 actuator ctrlrange 构造 mapper；任一滑块缺失则返回 None。"""
    ranges: dict[str, tuple[float, float]] = {}
    for name in ("cmd_linear_x", "cmd_angular_z", "cmd_height"):
        act_id = actuator_id(model, name)
        if act_id < 0:
            return None
        low, high = model.actuator_ctrlrange[act_id]
        ranges[name] = (float(low), float(high))
    return GamepadCommandMapper(
        linear_range=ranges["cmd_linear_x"],
        angular_range=ranges["cmd_angular_z"],
        height_range=ranges["cmd_height"],
    )


def _apply_gamepad_to_sliders(
    model: Any,
    data: Any,
    gamepad: GamepadDevice | None,
    mapper: GamepadCommandMapper | None,
) -> None:
    """把手柄当前状态写入 cmd_* 滑块；无手柄或读取失败时不动滑块。"""
    if gamepad is None or mapper is None:
        return
    state = gamepad.poll()
    if state is None:
        return
    linear_x, angular_z, height = mapper.map(state)
    for name, value in (
        ("cmd_linear_x", linear_x),
        ("cmd_angular_z", angular_z),
        ("cmd_height", height),
    ):
        act_id = actuator_id(model, name)
        if act_id >= 0:
            data.ctrl[act_id] = value


def _can_enable_gamepad(system_logger: Any, args: argparse.Namespace) -> bool:
    if not args.enable_gamepad:
        system_logger.info("Gamepad disabled; sim will be driven by MuJoCo Control sliders.")
        return False
    if sys.platform == "darwin" and Path(sys.argv[0]).name == "mjpython":
        system_logger.info("Gamepad enabled under mjpython on macOS (hidapi backend).")
    return True


def _trigger_jump_on_rising_edge(
    controller: Controller,
    jump_command: float,
    previous_jump_command: float,
) -> float:
    """Start one jump when cmd_jump crosses from off to on."""
    phase_machine = getattr(getattr(controller, "vmc_controller", controller), "phase_machine", None)
    is_on = jump_command > 0.5
    was_on = previous_jump_command > 0.5
    if is_on and not was_on and phase_machine is not None and phase_machine.phase == JumpPhase.STAND:
        phase_machine.start_jump()
    return jump_command


def run_controlled_viewer_loop(
    model: Any,
    data: Any,
    controller: Controller,
    viewer: Any,
    telemetry_logger: TelemetryLogger | None = None,
    gamepad: GamepadDevice | None = None,
    gamepad_mapper: GamepadCommandMapper | None = None,
) -> None:
    """以实时速度运行 viewer 仿真循环。

    每帧计算需要追赶的仿真步数，使仿真时间与墙钟时间同步。
    若提供了 gamepad，则在读取 cmd_* 滑块之前先把手柄值写入对应 actuator，
    让滑块面板与手柄保持同步；手柄缺席时完全回退到滑块控制。

    为避免单步耗时上升时陷入「追赶死亡螺旋」（catch-up loop 持有 viewer.lock
    导致渲染线程饿死、跳跃过程冻屏），单帧追赶步数有上限，超出则放弃追赶、
    让仿真落后于墙钟而非阻塞渲染。
    """
    render_period = 1.0 / 60.0  # viewer.sync 目标 60Hz
    max_steps_per_frame = max(1, int(render_period / max(model.opt.timestep, 1e-6)) * 2)
    wall_start = time.perf_counter()
    sim_start = data.time
    previous_jump_command = 0.0
    last_render = wall_start
    while viewer.is_running():
        # 计算墙钟时间对应的目标仿真时间
        wall_elapsed = time.perf_counter() - wall_start
        target_sim_time = sim_start + wall_elapsed
        # 步进仿真直到追上目标时间或达到本帧步数上限
        steps_this_frame = 0
        with viewer.lock():
            while data.time < target_sim_time and steps_this_frame < max_steps_per_frame:
                # 手柄写入滑块（若有），随后按既有路径读取滑块值
                _apply_gamepad_to_sliders(model, data, gamepad, gamepad_mapper)
                # 读取用户滑块指令
                cmd_sliders = _read_cmd_sliders(model, data)
                linear_x = cmd_sliders["cmd_linear_x"]
                angular_z = cmd_sliders["cmd_angular_z"]
                height = cmd_sliders["cmd_height"]
                previous_jump_command = _trigger_jump_on_rising_edge(
                    controller,
                    cmd_sliders["cmd_jump"],
                    previous_jump_command,
                )
                if isinstance(controller, CombinedController):
                    controller.params.target_velocity = linear_x
                    controller.params.target_yaw_rate = angular_z
                    controller.params.vmc.nominal_height = height

                target_info = ""
                params = getattr(controller, "params", None)
                target_velocity = getattr(params, "target_velocity", None)
                if target_velocity is not None:
                    target_info = f"v={target_velocity:.2f}"
                vmc = getattr(params, "vmc", None)
                nominal_height = getattr(vmc, "nominal_height", None)
                if nominal_height is not None:
                    target_info += f",h={nominal_height:.3f}"
                phase_machine = getattr(getattr(controller, "vmc_controller", controller), "phase_machine", None)
                if phase_machine is not None:
                    target_info += f",phase={phase_machine.phase.value}"

                finite = step_controlled_model(model, data, controller, telemetry_logger, target_info)
                # 恢复滑块值（step 会覆盖 data.ctrl）
                _restore_cmd_sliders(model, data, cmd_sliders)
                if not finite:
                    return
                steps_this_frame += 1
        # 若放弃追赶，将 sim_start 前移，避免下一帧继续累积追赶债务
        if data.time < target_sim_time:
            sim_start += target_sim_time - data.time
        now = time.perf_counter()
        if now - last_render >= render_period:
            viewer.sync()
            last_render = now


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
        mujoco.mj_forward(model, data)
        system_logger.info("Viewer mode started. No telemetry will be recorded.")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            apply_viewer_settings(viewer)
            import time
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)
        return

    terrain = None if args.flat_ground else "single_wheel_trapezoid"
    model, data = build_controlled_model(
        args.xml,
        cache_dir=args.cache_dir,
        terrain=terrain,
        terrain_side=args.terrain_side,
    )
    apply_controlled_scenario_initial_state(model, data, args.scenario)
    controller = create_controlled_controller(args.controller, args.scenario)

    if isinstance(controller, CombinedController):
        height_act = actuator_id(model, "cmd_height")
        if height_act >= 0:
            data.ctrl[height_act] = controller.params.vmc.nominal_height

    system_logger.info("Model built and controller created. Starting simulation loop.")

    gamepad = open_gamepad() if _can_enable_gamepad(system_logger, args) else None
    gamepad_mapper = _build_gamepad_mapper(model) if gamepad is not None else None
    if gamepad is not None:
        system_logger.info(f"Gamepad active: {gamepad.name}")

    with TelemetryLogger(log_dir / "telemetry.csv") as telemetry_logger:
        if args.scenario == "drive" and isinstance(controller, CombinedController):
            with mujoco.viewer.launch_passive(model, data) as viewer:
                apply_viewer_settings(viewer)
                wall_start = time.perf_counter()
                sim_start = data.time
                previous_jump_command = 0.0
                while viewer.is_running():
                    wall_elapsed = time.perf_counter() - wall_start
                    target_sim_time = sim_start + wall_elapsed
                    with viewer.lock():
                        while data.time < target_sim_time:
                            cmd_sliders = _read_cmd_sliders(model, data)
                            previous_jump_command = _trigger_jump_on_rising_edge(
                                controller,
                                cmd_sliders["cmd_jump"],
                                previous_jump_command,
                            )
                            controller.params.vmc.nominal_height = cmd_sliders["cmd_height"]
                            controller.params.target_velocity = _drive_velocity_profile(data.time)
                            
                            target_info = f"v={controller.params.target_velocity:.2f}"
                            if hasattr(controller.params, 'vmc') and hasattr(controller.params.vmc, 'nominal_height'):
                                target_info += f",h={controller.params.vmc.nominal_height:.3f}"
                            phase_machine = controller.vmc_controller.phase_machine
                            if phase_machine is not None:
                                target_info += f",phase={phase_machine.phase.value}"
                                
                            finite = step_controlled_model(model, data, controller, telemetry_logger, target_info)
                            _restore_cmd_sliders(model, data, cmd_sliders)
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
                run_controlled_viewer_loop(
                    model,
                    data,
                    controller,
                    viewer,
                    telemetry_logger,
                    gamepad=gamepad,
                    gamepad_mapper=gamepad_mapper,
                )
                
    system_logger.info("Simulation finished. Logs saved.")


if __name__ == "__main__":
    main()
