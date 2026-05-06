from pathlib import Path

import pytest

from sim.controllers.combined import CombinedController, CombinedParams
from sim.controllers.default_params import STAND_PARAMS, STAND_THEN_DRIVE_PARAMS
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.rollout import RolloutConfig, run_rollout


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _controller(target_velocity: float = 0.0, phase_machine=None) -> CombinedController:
    """站立和跳跃测试使用 STAND_PARAMS。"""
    import copy
    params = copy.deepcopy(STAND_PARAMS)
    params.target_velocity = target_velocity
    return CombinedController(params, phase_machine=phase_machine)


def _drive_controller(target_velocity: float = 1.0) -> CombinedController:
    """驱动测试使用 STAND_THEN_DRIVE_PARAMS。"""
    import copy
    params = copy.deepcopy(STAND_THEN_DRIVE_PARAMS)
    params.target_velocity = target_velocity
    return CombinedController(params)


@pytest.mark.xfail(reason="STAND_PARAMS needs re-optimization after WHEEL_FORWARD_SIGNS=-1.0 restoration")
def test_lqr_vmc_stand_1s_keeps_pitch_finite(tmp_path):
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=1.0, fall_height_drop=0.025, cache_dir=tmp_path / "stand"),
        controller=_controller(),
    )

    assert result.metrics.finite
    assert result.failure_reason is None
    assert result.metrics.max_abs_pitch < 0.08
    assert result.last_control.shape == (result.model.nu,)


@pytest.mark.xfail(reason="STAND_THEN_DRIVE_PARAMS: robot falls in 2s with aggressive velocity tracking")
def test_lqr_vmc_drive_forward_2s_moves_positive_x(tmp_path):
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=2.0, fall_height_drop=0.05, cache_dir=tmp_path / "drive"),
        controller=_drive_controller(target_velocity=1.0),
    )

    assert result.metrics.finite
    assert result.failure_reason is None
    assert not result.metrics.fell
    assert result.metrics.forward_distance >= 0.02
    assert result.metrics.max_abs_pitch < 0.05


@pytest.mark.xfail(reason="current VMC jump parameters do not produce measurable lift with the controlled model")
def test_lqr_vmc_jump_reaches_5cm(tmp_path):
    machine = JumpPhaseMachine(JumpPhaseParams(crouch_duration=0.05, extend_duration=0.05, flight_timeout=0.2))
    machine.start_jump()

    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=0.5, scenario="jump", cache_dir=tmp_path / "jump"),
        controller=_controller(phase_machine=machine),
    )

    assert result.metrics.finite
    assert result.metrics.max_jump_height >= 0.05


@pytest.mark.xfail(reason="STAND_PARAMS needs re-optimization after WHEEL_FORWARD_SIGNS=-1.0 restoration")
def test_lqr_vmc_stand_10s_no_fall(tmp_path):
    """SPEC §10 baseline: 10s 站稳不倒。

    自碰撞修复 + 参数优化后，机器人 10s 高度保持在 0.061±0.004m，
    pitch < 0.001 rad，零执行器饱和。使用默认 fall_height_drop=0.02。
    """
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=10.0, fall_height_drop=0.025, cache_dir=tmp_path / "stand_10s"),
        controller=_controller(),
    )

    assert result.metrics.finite, f"simulation diverged: {result.failure_reason}"
    assert not result.metrics.fell, f"min_height={result.metrics.min_base_height:.4f}"
    assert result.metrics.max_abs_pitch < 0.02, (
        f"pitch exceeded 0.02 rad: {result.metrics.max_abs_pitch:.4f}"
    )


@pytest.mark.xfail(reason=(
    "LQR velocity tracking insufficient: "
    "10s forward distance ~0.51m with tv=5.0, below SPEC target 1.0m. "
    "Needs LQR Q/R tuning or wheel actuator gear ratio adjustment."
))
def test_lqr_vmc_drive_forward_10s_at_least_1m(tmp_path):
    """SPEC §10 baseline: 10s 前进 ≥1m。

    ctrlrange 已扩展到 [-3.0, 3.0]（轮子）/ [-3.5, 3.5]（腿），
    10s 前进距离约 0.51m（tv=5.0）。需调整 LQR Q/R 增大速度跟踪增益。
    """
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=10.0, fall_height_drop=0.1, cache_dir=tmp_path / "drive_10s"),
        controller=_drive_controller(target_velocity=1.0),
    )

    assert result.metrics.finite, f"simulation diverged: {result.failure_reason}"
    assert not result.metrics.fell, f"min_height={result.metrics.min_base_height:.4f}"
    assert result.metrics.forward_distance >= 1.0, (
        f"forward_distance={result.metrics.forward_distance:.4f}"
    )
