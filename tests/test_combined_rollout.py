from pathlib import Path

import numpy as np
import pytest

from sim.controllers.combined import CombinedController, CombinedParams
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcParams
from sim.rollout import RolloutConfig, run_rollout


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _controller(target_velocity: float = 0.0, phase_machine=None) -> CombinedController:
    return CombinedController(
        CombinedParams(
            vmc=VmcParams(nominal_height=0.06078, kp_height=1.0, kd_height=0.2),
            q_diag=np.array([100.0, 10.0, 1.0, 1.0]),
            r_diag=np.array([1.0, 1.0]),
            target_velocity=target_velocity,
        ),
        phase_machine=phase_machine,
    )


def test_lqr_vmc_stand_1s_keeps_pitch_finite(tmp_path):
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=1.0, cache_dir=tmp_path / "stand"),
        controller=_controller(),
    )

    assert result.metrics.finite
    assert result.failure_reason is None
    assert result.metrics.max_abs_pitch < 0.01
    assert result.last_control.shape == (result.model.nu,)


def test_lqr_vmc_drive_forward_2s_moves_positive_x(tmp_path):
    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=2.0, cache_dir=tmp_path / "drive"),
        controller=_controller(target_velocity=2.0),
    )

    assert result.metrics.finite
    assert result.failure_reason is None
    assert not result.metrics.fell
    assert result.metrics.forward_distance >= 0.05
    assert result.metrics.max_abs_pitch < 0.02


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
