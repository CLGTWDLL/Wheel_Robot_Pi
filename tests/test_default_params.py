from __future__ import annotations

from dataclasses import replace

import numpy as np

from sim.controllers.default_params import STAND_PARAMS, params_from_dict, params_to_dict


def test_combined_params_roundtrip_preserves_control_fields() -> None:
    params = replace(
        STAND_PARAMS,
        q_diag=np.array([1.0, 2.0, 3.0, 4.0, 5.0]),
        r_diag=np.array([7.0, 8.0]),
        target_velocity=0.31,
        pitch_trim=0.12,
        pitch_trim_from_lut=False,
        pitch_lean_gain=0.23,
        velocity_kp=0.34,
        velocity_ki=0.45,
        yaw_damping=0.56,
        yaw_ki=0.67,
        target_yaw_rate=0.78,
        fixed_height=True,
        lqr_height_bin_size=0.09,
        flight_wheel_q=1.2,
        ff_gain=4.5,
    )
    params.vmc.crouch_target_height = 0.091
    params.vmc.crouch_duration = 0.21
    params.vmc.land_kp_scale = 0.6
    params.vmc.land_kd_scale = 2.7

    restored = params_from_dict(params_to_dict(params))

    np.testing.assert_allclose(restored.q_diag, params.q_diag)
    np.testing.assert_allclose(restored.r_diag, params.r_diag)
    assert restored.target_velocity == params.target_velocity
    assert restored.pitch_trim == params.pitch_trim
    assert restored.pitch_trim_from_lut is params.pitch_trim_from_lut
    assert restored.pitch_lean_gain == params.pitch_lean_gain
    assert restored.velocity_kp == params.velocity_kp
    assert restored.velocity_ki == params.velocity_ki
    assert restored.yaw_damping == params.yaw_damping
    assert restored.yaw_ki == params.yaw_ki
    assert restored.target_yaw_rate == params.target_yaw_rate
    assert restored.fixed_height is params.fixed_height
    assert restored.lqr_height_bin_size == params.lqr_height_bin_size
    assert restored.flight_wheel_q == params.flight_wheel_q
    assert restored.ff_gain == params.ff_gain
    assert restored.vmc.crouch_target_height == params.vmc.crouch_target_height
    assert restored.vmc.crouch_duration == params.vmc.crouch_duration
    assert restored.vmc.land_kp_scale == params.vmc.land_kp_scale
    assert restored.vmc.land_kd_scale == params.vmc.land_kd_scale
