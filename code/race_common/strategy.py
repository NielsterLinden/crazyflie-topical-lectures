"""Starter guidance and uncertainty-aware speed policies."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .hoop import Hoop


@dataclass
class RaceLimits:
    minimum_speed_m_s: float = 0.20
    nominal_speed_m_s: float = 0.80
    maximum_speed_m_s: float = 1.50
    drone_radius_m: float = 0.10
    tracking_error_m: float = 0.05
    confidence_multiplier: float = 2.5


def approach_points(hoop: Hoop, distance_m: float = 0.50) -> tuple[np.ndarray, np.ndarray]:
    """Return points before and after the hoop along its normal."""
    if distance_m <= 0.0:
        raise ValueError("distance_m must be positive")
    return hoop.centre - distance_m * hoop.normal, hoop.centre + distance_m * hoop.normal


def choose_crossing_point(
    hoop: Hoop,
    next_target: np.ndarray | None = None,
) -> np.ndarray:
    """Baseline strategy: cross centrally.

    TODO 4: choose an off-centre point that shortens the route to next_target
    while maintaining a justified uncertainty and tracking margin.
    """
    del next_target
    return hoop.centre.copy()


def recommended_speed(
    hoop: Hoop,
    estimated_position: np.ndarray,
    position_covariance: np.ndarray,
    limits: RaceLimits,
) -> tuple[float, float]:
    """Return baseline speed and conservative predicted clearance.

    TODO 5: make speed depend on clearance, covariance, braking distance, and
    observed tracking performance. The fixed baseline is intentionally naive.
    """
    clearance = hoop.conservative_clearance(
        estimated_position,
        position_covariance,
        limits.drone_radius_m,
        limits.tracking_error_m,
        limits.confidence_multiplier,
    )
    speed = float(np.clip(limits.nominal_speed_m_s, limits.minimum_speed_m_s, limits.maximum_speed_m_s))
    return speed, clearance
