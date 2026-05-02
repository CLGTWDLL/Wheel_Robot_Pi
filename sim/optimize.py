from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import random

import numpy as np

from sim.controllers.combined import CombinedController, CombinedParams
from sim.controllers.phase import JumpPhaseMachine, JumpPhaseParams
from sim.controllers.vmc import VmcParams
from sim.metrics import RolloutMetrics
from sim.rollout import RolloutConfig, run_rollout


@dataclass(frozen=True)
class OptimizationConfig:
    trials: int
    duration: float
    scenario: str = "jump"
    seed: int = 0
    cache_dir: Path | None = None


@dataclass(frozen=True)
class OptimizationTrial:
    params: CombinedParams
    score: float
    metrics: RolloutMetrics
    failure_reason: str | None


@dataclass(frozen=True)
class OptimizationResult:
    trials: tuple[OptimizationTrial, ...]
    best_trial: OptimizationTrial


def score_metrics(metrics: RolloutMetrics) -> float:
    score = 0.0
    score += 10.0 * metrics.forward_distance
    score += 20.0 * metrics.max_jump_height
    score -= 0.001 * metrics.control_effort
    score -= 2.0 * metrics.saturation_ratio
    score -= metrics.max_abs_pitch
    if not metrics.finite:
        score -= 100.0
    if metrics.fell:
        score -= 10.0
    return score


def optimize_parameters(xml_path: Path, config: OptimizationConfig) -> OptimizationResult:
    if config.trials <= 0:
        raise ValueError("trials must be positive")
    rng = random.Random(config.seed)
    trials: list[OptimizationTrial] = []
    for _ in range(config.trials):
        params = _sample_combined_params(rng)
        machine = JumpPhaseMachine(_sample_phase_params(rng))
        machine.start_jump()
        controller = CombinedController(params, phase_machine=machine)
        try:
            rollout = run_rollout(
                xml_path,
                RolloutConfig(duration=config.duration, scenario=config.scenario, cache_dir=config.cache_dir),
                controller=controller,
            )
            score = score_metrics(rollout.metrics)
            trial = OptimizationTrial(params=params, score=score, metrics=rollout.metrics, failure_reason=rollout.failure_reason)
        except Exception as exc:
            metrics = RolloutMetrics(
                duration=config.duration,
                finite=False,
                fell=True,
                min_base_height=0.0,
                max_base_height=0.0,
                max_jump_height=0.0,
                max_abs_pitch=0.0,
                forward_distance=0.0,
                contact_count=0,
                control_effort=0.0,
                saturation_ratio=1.0,
            )
            trial = OptimizationTrial(params=params, score=score_metrics(metrics), metrics=metrics, failure_reason=str(exc))
        trials.append(trial)

    best_trial = max(trials, key=lambda trial: trial.score)
    return OptimizationResult(trials=tuple(trials), best_trial=best_trial)


def _sample_combined_params(rng: random.Random) -> CombinedParams:
    return CombinedParams(
        vmc=VmcParams(
            nominal_height=rng.uniform(0.05, 0.09),
            kp_height=rng.uniform(1.0, 8.0),
            kd_height=rng.uniform(0.1, 1.0),
            crouch_offset=rng.uniform(-0.03, -0.005),
            extend_offset=rng.uniform(0.03, 0.1),
            landing_damping=rng.uniform(0.5, 2.0),
        ),
        q_diag=np.array(
            [
                rng.uniform(10.0, 500.0),
                rng.uniform(1.0, 100.0),
                rng.uniform(0.1, 10.0),
                rng.uniform(0.1, 10.0),
            ]
        ),
        r_diag=np.array([rng.uniform(0.1, 10.0), rng.uniform(0.1, 10.0)]),
    )


def _sample_phase_params(rng: random.Random) -> JumpPhaseParams:
    return JumpPhaseParams(
        crouch_duration=rng.uniform(0.01, 0.08),
        extend_duration=rng.uniform(0.01, 0.08),
        flight_timeout=rng.uniform(0.05, 0.25),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run bounded random search over MuJoCo VMC parameters.")
    parser.add_argument("xml", type=Path, help="Path to the source MuJoCo XML model.")
    parser.add_argument("--trials", type=int, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--scenario", default="jump")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cache-dir", type=Path)
    args = parser.parse_args(argv)

    result = optimize_parameters(
        args.xml,
        OptimizationConfig(
            trials=args.trials,
            duration=args.duration,
            scenario=args.scenario,
            seed=args.seed,
            cache_dir=args.cache_dir,
        ),
    )
    print(f"trials={len(result.trials)} best_score={result.best_trial.score:.6f} params={result.best_trial.params}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
