from __future__ import annotations

import json

import numpy as np

from sim.controllers.leg_height_lut import DEFAULT_LUT, LegHeightLUT


def test_default_lut_contains_geometry_feedforward_fields() -> None:
    assert DEFAULT_LUT.dy_grid.shape == DEFAULT_LUT.theta_grid.shape
    assert DEFAULT_LUT.pitch_eq_grid.shape == DEFAULT_LUT.theta_grid.shape
    assert np.all(np.isfinite(DEFAULT_LUT.dy_grid))
    assert np.all(np.isfinite(DEFAULT_LUT.pitch_eq_grid))


def test_default_lut_geometry_values_match_current_probe() -> None:
    assert np.isclose(DEFAULT_LUT.pitch_eq_from_height(0.142), -0.0205, atol=0.005)
    assert np.isclose(DEFAULT_LUT.dy_wheel_dh(0.142), 0.164, atol=0.04)


def test_lut_interpolates_height_derivatives_from_json(tmp_path) -> None:
    path = tmp_path / "lut.json"
    path.write_text(
        json.dumps(
            {
                "theta_grid": [0.0, 1.0, 2.0],
                "h_grid": [0.10, 0.20, 0.30],
                "dy_grid": [0.0, 0.5, 1.0],
                "pitch_eq_grid": [-0.1, -0.2, -0.3],
                "h_min": 0.10,
                "h_max": 0.30,
                "theta_min": 0.0,
                "theta_max": 2.0,
            }
        )
    )

    lut = LegHeightLUT.from_json(path)

    assert np.isclose(lut.motor_angle_from_height(0.15), 0.5)
    assert np.isclose(lut.height_dtheta(0.15), 0.1)
    assert np.isclose(lut.motor_angle_dh(0.15), 10.0)
    assert np.isclose(lut.dy_wheel_dh(0.15), 5.0)
    assert np.isclose(lut.pitch_eq_from_height(0.15), -0.15)
