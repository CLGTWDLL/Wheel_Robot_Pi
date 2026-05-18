from __future__ import annotations

from pathlib import Path

from scripts.verify_single_wheel_ramp import run_single_wheel_ramp


def test_roll_leveling_reduces_single_wheel_ramp_roll(tmp_path: Path) -> None:
    enabled_cache = tmp_path / "enabled"
    disabled_cache = tmp_path / "disabled"
    enabled_cache.mkdir()
    disabled_cache.mkdir()

    enabled = run_single_wheel_ramp(
        duration=4.0,
        target_velocity=0.12,
        terrain_side="left",
        cache_dir=enabled_cache,
        enable_roll_leveling=True,
    )
    disabled = run_single_wheel_ramp(
        duration=4.0,
        target_velocity=0.12,
        terrain_side="left",
        cache_dir=disabled_cache,
        enable_roll_leveling=False,
    )

    assert enabled.finite
    assert not enabled.fell
    assert disabled.finite
    assert not disabled.fell
    assert enabled.max_abs_roll < disabled.max_abs_roll * 0.95
