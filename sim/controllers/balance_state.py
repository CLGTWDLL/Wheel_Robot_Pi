from __future__ import annotations

import numpy as np

from sim.state import SimState


def balance_tangent_state(model: object, data: object, state: SimState) -> np.ndarray:
    del model, data

    tangent = np.array(
        [
            state.pitch,
            state.pitch_rate,
            np.mean(list(state.wheel_positions.values()), dtype=float),
            np.mean(list(state.wheel_velocities.values()), dtype=float),
        ],
        dtype=float,
    )
    if tangent.shape != (4,) or not np.all(np.isfinite(tangent)):
        raise ValueError("balance tangent state must be shape (4,) and finite")
    return tangent
