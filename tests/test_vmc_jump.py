from pathlib import Path

import numpy as np
import mujoco
import pytest

from sim.controllers.phase import JumpPhase, JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcController, VmcParams
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.rollout import RolloutConfig, run_rollout
from sim.state import extract_sim_state


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _model_and_data(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(controlled_xml))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


def test_jump_phase_machine_advances_by_time_and_height():
    machine = JumpPhaseMachine(JumpPhaseParams(crouch_duration=0.1, extend_duration=0.1, flight_timeout=0.5))

    assert machine.phase == JumpPhase.STAND
    machine.start_jump()
    assert machine.phase == JumpPhase.CROUCH
    machine.update(dt=0.11, base_height=0.3, vertical_velocity=-0.1, contact_count=2)
    assert machine.phase == JumpPhase.EXTEND
    machine.update(dt=0.11, base_height=0.35, vertical_velocity=0.2, contact_count=2)
    assert machine.phase == JumpPhase.FLIGHT
    machine.update(dt=0.01, base_height=0.36, vertical_velocity=-0.1, contact_count=0)
    assert machine.phase == JumpPhase.FLIGHT
    machine.update(dt=0.01, base_height=0.3, vertical_velocity=-0.1, contact_count=1)
    assert machine.phase == JumpPhase.LAND
    machine.update(dt=0.2, base_height=0.3, vertical_velocity=0.0, contact_count=1)
    assert machine.phase == JumpPhase.RECOVER
    machine.update(dt=0.2, base_height=0.3, vertical_velocity=0.0, contact_count=1)
    assert machine.phase == JumpPhase.STAND


def test_vmc_controller_returns_finite_clipped_control(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = VmcController(VmcParams(nominal_height=0.31, kp_height=2.0, kd_height=0.5))

    control = controller(model, data, extract_sim_state(model, data))

    assert control.shape == (model.nu,)
    assert np.all(np.isfinite(control))
    assert np.all(control <= model.actuator_ctrlrange[:, 1])
    assert np.all(control >= model.actuator_ctrlrange[:, 0])


def test_vmc_controller_maps_positive_vertical_effort_to_leg_signs(tmp_path):
    model, data = _model_and_data(tmp_path)
    # 步进到 ramp 结束 (t>1.5s), 使 target_height = nominal_height
    n_ramp_steps = int(2.0 / model.opt.timestep)
    controller = VmcController(VmcParams(nominal_height=0.5, kp_height=10.0, kd_height=0.0))
    for _ in range(n_ramp_steps):
        state = extract_sim_state(model, data)
        ctrl = controller(model, data, state)
        data.ctrl[:] = np.clip(ctrl, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        mujoco.mj_step(model, data)

    state = extract_sim_state(model, data)
    control = controller(model, data, state)

    # 4 个 actuator: 0,1=wheel, 2=r1(LEG_EFFORT_SIGNS=-1), 3=r9(LEG_EFFORT_SIGNS=+1)
    # ramp 结束后 target=0.5 >> z, pd_effort 为正
    # control[2] = -1 * pd < 0, control[3] = +1 * pd > 0
    assert control[2] < 0.0
    assert control[3] > 0.0


def test_vmc_controller_rejects_nonfinite_params(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = VmcController(VmcParams(nominal_height=np.nan, kp_height=1.0, kd_height=0.2))

    try:
        controller(model, data, extract_sim_state(model, data))
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("expected nonfinite VMC params to fail")


def test_vmc_jump_rollout_runs_without_nonfinite_state(tmp_path):
    machine = JumpPhaseMachine(JumpPhaseParams(crouch_duration=0.01, extend_duration=0.01, flight_timeout=0.05))
    machine.start_jump()
    controller = VmcController(VmcParams(nominal_height=0.06078, kp_height=1.0, kd_height=0.2), phase_machine=machine)

    result = run_rollout(XML_PATH, RolloutConfig(duration=0.05, cache_dir=tmp_path, scenario="jump"), controller=controller)

    assert result.metrics.finite
    assert result.failure_reason is None
    assert result.last_control.shape == (result.model.nu,)


def test_vmc_jump_rollout_reaches_small_jump_height(tmp_path):
    machine = JumpPhaseMachine(JumpPhaseParams(
        crouch_duration=0.075, 
        extend_duration=0.144, 
        flight_timeout=0.294
    ))
    machine.start_jump()
    controller = VmcController(
        VmcParams(
            nominal_height=0.0978, 
            kp_height=38.16, 
            kd_height=0.505, 
            crouch_offset=-0.0084, 
            extend_offset=0.1044, 
            landing_damping=0.875
        ),
        phase_machine=machine,
    )

    result = run_rollout(XML_PATH, RolloutConfig(duration=0.5, cache_dir=tmp_path, scenario="jump"), controller=controller)

    assert result.metrics.finite
    assert result.metrics.max_jump_height >= 0.05
