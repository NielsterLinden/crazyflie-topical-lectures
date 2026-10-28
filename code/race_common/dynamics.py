"""Shared discrete dynamics and physically meaningful command limits."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


GRAVITY_M_S2 = 9.80665
AXES = 3


def position_velocity_model(dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Return constant-acceleration matrices for state [position, velocity].

    Acceleration is assumed constant only over this one time step. Keeping this
    construction in one place prevents the starter filter and advanced
    controller from silently adopting different state ordering or kinematics.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    identity_xyz = np.eye(AXES)
    position_rows = slice(0, AXES)
    velocity_rows = slice(AXES, 2 * AXES)

    transition = np.eye(2 * AXES)
    transition[position_rows, velocity_rows] = identity_xyz * dt

    acceleration_input = np.zeros((2 * AXES, AXES))
    acceleration_input[position_rows, :] = identity_xyz * (0.5 * dt**2)
    acceleration_input[velocity_rows, :] = identity_xyz * dt
    return transition, acceleration_input


@dataclass(frozen=True)
class AccelerationLimit:
    acceleration_world_m_s2: np.ndarray
    required_tilt_rad: float
    saturated: bool


def limit_acceleration(
    requested_world_m_s2: np.ndarray,
    maximum_magnitude_m_s2: float,
    maximum_tilt_rad: float,
) -> AccelerationLimit:
    """Apply total-acceleration and thrust-vector tilt limits.

    The command excludes gravity. The firmware must therefore generate total
    specific force requested + [0, 0, g]. Its horizontal-to-vertical ratio
    determines the required tilt.
    """
    requested = np.asarray(requested_world_m_s2, dtype=float)
    if requested.shape != (AXES,) or not np.all(np.isfinite(requested)):
        raise ValueError("requested acceleration must contain three finite values")
    if maximum_magnitude_m_s2 <= 0.0:
        raise ValueError("maximum acceleration must be positive")
    if not 0.0 < maximum_tilt_rad < 0.5 * np.pi:
        raise ValueError("maximum tilt must be between 0 and 90 degrees")

    limited = requested.copy()
    saturated = False

    magnitude = float(np.linalg.norm(limited))
    if magnitude > maximum_magnitude_m_s2:
        limited *= maximum_magnitude_m_s2 / magnitude
        saturated = True

    # Near free fall, the ideal model has almost no vertical thrust component.
    # A small floor keeps the geometric limit finite and conservative.
    vertical_support = max(0.1, GRAVITY_M_S2 + limited[2])
    maximum_lateral = vertical_support * np.tan(maximum_tilt_rad)
    lateral = float(np.linalg.norm(limited[:2]))
    if lateral > maximum_lateral:
        limited[:2] *= maximum_lateral / lateral
        saturated = True

    required_tilt = float(np.arctan2(np.linalg.norm(limited[:2]), vertical_support))
    return AccelerationLimit(limited, required_tilt, saturated)
