"""Reference trajectory shared by estimator and controller simulations."""

from __future__ import annotations

import math

import numpy as np

from .controller import TrajectoryReference


def reference_state(timestamp_s: float) -> TrajectoryReference:
    position = np.array(
        [
            0.50 * timestamp_s,
            0.45 * math.sin(0.55 * timestamp_s),
            1.00 + 0.12 * math.sin(0.35 * timestamp_s),
        ]
    )
    velocity = np.array(
        [
            0.50,
            0.45 * 0.55 * math.cos(0.55 * timestamp_s),
            0.12 * 0.35 * math.cos(0.35 * timestamp_s),
        ]
    )
    acceleration = np.array(
        [
            0.0,
            -0.45 * 0.55**2 * math.sin(0.55 * timestamp_s),
            -0.12 * 0.35**2 * math.sin(0.35 * timestamp_s),
        ]
    )
    return TrajectoryReference(position, velocity, acceleration)
