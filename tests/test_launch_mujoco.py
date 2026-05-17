from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.default_params import STAND_PARAMS
from sim.controllers.balance_state import balance_tangent_state_5d
from sim.controllers.phase import JumpPhase, JumpPhaseMachine
from sim.controllers.vmc import VmcController
from sim.launch_mujoco import (
    _can_enable_gamepad,
    _read_cmd_sliders,
    _restore_cmd_sliders,
    _trigger_jump_on_rising_edge,
    build_controlled_model,
    create_controlled_controller,
    parse_args,
    step_controlled_model,
)
from sim.model_xml import CMD_SLIDER_NAMES, prepare_controlled_mujoco_xml
from sim.model_semantics import MODEL_SEMANTICS
from sim.state import actuator_id, extract_sim_state, model_addresses


class _ControllerWithPhaseMachine:
    def __init__(self, phase_machine: JumpPhaseMachine) -> None:
        self.phase_machine = phase_machine


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)

    def warning(self, message: str) -> None:
        self.messages.append(message)


def test_gamepad_is_disabled_by_default() -> None:
    args = parse_args([])
    logger = _Logger()

    assert not _can_enable_gamepad(logger, args)
    assert any("Gamepad disabled" in message for message in logger.messages)


def test_cmd_jump_is_non_physical_command_actuator(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
    )
    root = ET.parse(model_path).getroot()

    command_body = root.find(".//body[@name='command_slider_body']")
    assert command_body is not None
    assert command_body.find("joint[@name='cmd_jump']") is not None

    actuator = root.find("actuator/motor[@name='cmd_jump']")
    assert actuator is not None
    assert actuator.get("gear") == "0"
    assert actuator.get("ctrlrange") == "0 1"


def test_controlled_model_initializes_cmd_jump_slider(tmp_path: Path) -> None:
    model, data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )

    assert "cmd_jump" in CMD_SLIDER_NAMES
    jump_actuator = actuator_id(model, "cmd_jump")
    assert jump_actuator >= 0
    assert data.ctrl[jump_actuator] == 0.0


def test_leg_actuator_range_allows_dm4310_short_burst_peak(tmp_path: Path) -> None:
    model, _data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )

    leg_ranges = [
        model.actuator_ctrlrange[actuator_id(model, f"act_{joint_name}")]
        for joint_name in ("base_link_旋转-2", "base_link_旋转-1")
    ]

    for low, high in leg_ranges:
        assert abs(low + 12.5) < 1e-9
        assert abs(high - 12.5) < 1e-9


def test_command_slider_read_and_restore_includes_cmd_jump(tmp_path: Path) -> None:
    model, data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )
    jump_actuator = actuator_id(model, "cmd_jump")

    data.ctrl[jump_actuator] = 1.0
    values = _read_cmd_sliders(model, data)
    data.ctrl[:] = 0.0
    _restore_cmd_sliders(model, data, values)

    assert values["cmd_jump"] == 1.0
    assert data.ctrl[jump_actuator] == 1.0


def test_cmd_jump_rising_edge_triggers_once() -> None:
    phase_machine = JumpPhaseMachine()
    controller = _ControllerWithPhaseMachine(phase_machine)

    previous = _trigger_jump_on_rising_edge(controller, 0.0, 0.0)
    assert phase_machine.phase == JumpPhase.STAND

    previous = _trigger_jump_on_rising_edge(controller, 1.0, previous)
    assert phase_machine.phase == JumpPhase.CROUCH
    first_time_in_phase = phase_machine.time_in_phase

    previous = _trigger_jump_on_rising_edge(controller, 1.0, previous)
    assert phase_machine.phase == JumpPhase.CROUCH
    assert phase_machine.time_in_phase == first_time_in_phase

    previous = _trigger_jump_on_rising_edge(controller, 0.0, previous)
    phase_machine.phase = JumpPhase.STAND
    _trigger_jump_on_rising_edge(controller, 1.0, previous)
    assert phase_machine.phase == JumpPhase.CROUCH


def test_jump_phase_does_not_enter_flight_while_still_in_contact() -> None:
    phase_machine = JumpPhaseMachine()
    phase_machine.start_jump()

    phase_machine.update(dt=0.30, leg_height=0.2, vz=0.0, contact_count=2)
    assert phase_machine.phase == JumpPhase.EXTEND

    phase_machine.update(dt=0.30, leg_height=0.2, vz=0.2, contact_count=2)
    assert phase_machine.phase == JumpPhase.EXTEND

    phase_machine.update(dt=0.01, leg_height=0.2, vz=0.2, contact_count=0)
    assert phase_machine.phase == JumpPhase.FLIGHT


def test_jump_phase_accepts_leg_height_and_vz_inputs() -> None:
    phase_machine = JumpPhaseMachine()
    phase_machine.start_jump()

    phase_machine.update(dt=0.30, leg_height=0.09, vz=0.0, contact_count=2)
    assert phase_machine.phase == JumpPhase.EXTEND

    phase_machine.update(dt=0.13, leg_height=0.14, vz=0.2, contact_count=0)
    assert phase_machine.phase == JumpPhase.FLIGHT

    phase_machine.update(dt=0.01, leg_height=0.13, vz=-0.1, contact_count=2)
    assert phase_machine.phase == JumpPhase.LAND


def test_vmc_soft_limits_rated_phases_but_allows_extend_peak(tmp_path: Path) -> None:
    model, data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )
    params = STAND_PARAMS.vmc
    params.jump_extend_feedforward = 12.5
    params.jump_extend_feedforward_duration = 0.05
    params.crouch_target_height = 0.09
    params.crouch_duration = 0.20
    phase_machine = JumpPhaseMachine()
    controller = VmcController(params, phase_machine=phase_machine)
    state = extract_sim_state(model, data)
    addresses = model_addresses(model)
    leg_indices = [addresses.actuators[name] for name in MODEL_SEMANTICS.leg_motor_joints]

    phase_machine.phase = JumpPhase.STAND
    stand_control = controller.preview_control(model, data, state)
    assert np.max(np.abs(stand_control[leg_indices])) <= 3.5 + 1e-9

    phase_machine.phase = JumpPhase.EXTEND
    phase_machine.time_in_phase = 0.01
    extend_control = controller.preview_control(model, data, state)
    assert np.allclose(extend_control[leg_indices], 12.5)


def test_flight_lqr_keeps_pitch_roll_control_and_releases_wheel_targets(tmp_path: Path) -> None:
    model, data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )
    controller = create_controlled_controller("lqr_vmc", "stand")
    assert hasattr(controller, "vmc_controller")
    state = extract_sim_state(model, data)
    controller(model, data, state)
    lqr = controller.lqr_controller
    assert lqr is not None

    phase_machine = controller.vmc_controller.phase_machine
    assert phase_machine is not None
    phase_machine.phase = JumpPhase.FLIGHT
    state = extract_sim_state(model, data)
    control = controller(model, data, state)

    assert np.any(np.abs(control) > 1e-9)
    # 5D state: target[4] 是 wheel_vel target, FLIGHT 释放为 0
    assert lqr.target[4] == 0.0
    assert lqr.gain.shape == (2, 5)
    # 当前 tangent state 不应该影响 LQR target shape
    assert balance_tangent_state_5d(model, data, state).shape == (5,)


def test_cmd_jump_rollout_reaches_airborne_contact_state(tmp_path: Path) -> None:
    model, data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )
    controller = create_controlled_controller("lqr_vmc", "stand")

    data.ctrl[actuator_id(model, "cmd_jump")] = 1.0
    _trigger_jump_on_rising_edge(controller, 1.0, 0.0)
    initial_z = float(data.qpos[2])

    min_contact_count = 100
    max_z = initial_z
    for _ in range(int(1.0 / model.opt.timestep)):
        state = extract_sim_state(model, data)
        min_contact_count = min(min_contact_count, state.contact_count)
        max_z = max(max_z, float(data.qpos[2]))
        assert step_controlled_model(model, data, controller)

    final_phase = controller.vmc_controller.phase_machine.phase

    assert min_contact_count == 0
    assert max_z >= initial_z + 0.10
    assert final_phase != JumpPhase.FALLEN
