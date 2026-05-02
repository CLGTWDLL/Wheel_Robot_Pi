from pathlib import Path

from sim.controllers.combined import CombinedParams
from sim.optimize import OptimizationConfig, main, optimize_parameters, score_metrics
from sim.metrics import RolloutMetrics


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def test_score_metrics_penalizes_failed_or_fallen_rollouts():
    good = RolloutMetrics(
        duration=1.0,
        finite=True,
        fell=False,
        min_base_height=0.05,
        max_base_height=0.2,
        max_jump_height=0.1,
        max_abs_pitch=0.1,
        forward_distance=0.2,
        contact_count=2,
        control_effort=1.0,
        saturation_ratio=0.0,
    )
    bad = RolloutMetrics(
        duration=1.0,
        finite=False,
        fell=True,
        min_base_height=-1.0,
        max_base_height=0.0,
        max_jump_height=0.0,
        max_abs_pitch=3.0,
        forward_distance=-1.0,
        contact_count=0,
        control_effort=100.0,
        saturation_ratio=1.0,
    )

    assert score_metrics(good) > score_metrics(bad)


def test_optimize_parameters_runs_bounded_trials(tmp_path):
    result = optimize_parameters(
        XML_PATH,
        OptimizationConfig(trials=3, duration=0.05, scenario="jump", seed=7, cache_dir=tmp_path),
    )

    assert len(result.trials) == 3
    assert result.best_trial in result.trials
    assert isinstance(result.best_trial.params, CombinedParams)
    assert result.best_trial.params.vmc.nominal_height >= 0.05
    assert result.best_trial.params.vmc.nominal_height <= 0.09
    assert result.best_trial.params.q_diag.shape == (4,)
    assert result.best_trial.params.r_diag.shape == (2,)


def test_optimize_parameters_is_repeatable_with_seed(tmp_path):
    config = OptimizationConfig(trials=2, duration=0.02, scenario="jump", seed=11, cache_dir=tmp_path)

    first = optimize_parameters(XML_PATH, config)
    second = optimize_parameters(XML_PATH, config)

    for first_trial, second_trial in zip(first.trials, second.trials, strict=True):
        assert first_trial.params.vmc == second_trial.params.vmc
        assert first_trial.params.target_velocity == second_trial.params.target_velocity
        assert (first_trial.params.q_diag == second_trial.params.q_diag).all()
        assert (first_trial.params.r_diag == second_trial.params.r_diag).all()
    assert [trial.score for trial in first.trials] == [trial.score for trial in second.trials]


def test_optimize_cli_runs_bounded_trials(tmp_path, capsys):
    exit_code = main([str(XML_PATH), "--trials", "1", "--duration", "0.01", "--cache-dir", str(tmp_path)])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "best_score=" in out
    assert "trials=1" in out
