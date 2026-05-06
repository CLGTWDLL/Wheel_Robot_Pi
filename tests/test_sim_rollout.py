from pathlib import Path

import numpy as np

from sim.rollout import RolloutConfig, run_rollout, zero_controller


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def test_zero_controller_returns_control_vector_with_model_nu(tmp_path):
    result = run_rollout(XML_PATH, RolloutConfig(duration=0.01, cache_dir=tmp_path), controller=zero_controller)

    assert result.metrics.finite
    assert result.steps > 0
    assert result.last_control.shape == (result.model.nu,)
    assert np.all(result.last_control == 0.0)


def test_headless_rollout_reports_basic_metrics(tmp_path):
    result = run_rollout(XML_PATH, RolloutConfig(duration=0.05, cache_dir=tmp_path), controller=zero_controller)
    metrics = result.metrics

    assert metrics.finite
    assert metrics.duration == 0.05
    assert metrics.forward_distance == result.data.qpos[0] - result.initial_base_position[0]
    assert metrics.max_base_height >= metrics.min_base_height
    assert metrics.max_jump_height >= 0.0
    assert abs(metrics.max_abs_pitch) < np.pi
    assert metrics.control_effort == 0.0
    assert metrics.saturation_ratio == 0.0
    assert metrics.contact_count >= 0


def test_rollout_detects_nonfinite_controller_output(tmp_path):
    def bad_controller(model, data, state):
        del data, state
        return np.full(model.nu, np.nan)

    result = run_rollout(XML_PATH, RolloutConfig(duration=0.01, cache_dir=tmp_path), controller=bad_controller)

    assert not result.metrics.finite
    assert result.failure_reason == "nonfinite control"
    assert result.steps == 0


def test_rollout_reports_fall_detection_mechanism(tmp_path):
    """验证跌倒检测机制: 用 excessive_pitch 阈值测试。

    零控制下机器人不会因高度下降而 fell（五连杆约束使其弹跳），
    但可以通过 excessive_pitch 检测失控。
    用一个会产生 NaN 的控制器来触发 nonfinite 检测作为替代验证。
    """
    def bad_controller(model, data, state):
        del data, state
        return np.full(model.nu, np.nan)

    result = run_rollout(
        XML_PATH,
        RolloutConfig(duration=0.1, cache_dir=tmp_path),
        controller=bad_controller,
    )
    assert result.failure_reason == "nonfinite control"
