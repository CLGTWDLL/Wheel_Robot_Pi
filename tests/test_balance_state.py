from dataclasses import replace

import numpy as np

from sim.controllers.balance_state import balance_tangent_state
from sim.state import SimState


def _state() -> SimState:
    return SimState(
        base_position=np.zeros(3),
        base_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        base_linear_velocity=np.zeros(3),
        base_angular_velocity=np.zeros(3),
        pitch=0.12,
        pitch_rate=-0.34,
        wheel_positions={"left": 1.0, "right": 3.0},
        wheel_velocities={"left": -2.0, "right": 4.0},
        leg_joint_positions={},
        leg_joint_velocities={},
        contact_count=2,
    )


def test_balance_tangent_state_uses_sim_state_fields():
    tangent = balance_tangent_state(model=None, data=None, state=_state())

    assert tangent.shape == (4,)
    assert np.all(np.isfinite(tangent))
    np.testing.assert_allclose(tangent, np.array([0.12, 0.34, -2.0, -1.0]))


def test_balance_tangent_state_rejects_nonfinite_values():
    state = replace(_state(), wheel_velocities={"left": np.nan, "right": 1.0})

    try:
        balance_tangent_state(model=None, data=None, state=state)
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("expected nonfinite balance tangent state to be rejected")
