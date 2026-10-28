"""Geometry shared by hoop construction and race-line optimization."""

from __future__ import annotations

import numpy as np


def unit_vector(vector: np.ndarray, name: str = "direction") -> np.ndarray:
    """Return a normalized three-vector."""
    result = np.asarray(vector, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    norm = float(np.linalg.norm(result))
    if norm <= 1e-12:
        raise ValueError(f"{name} must be non-zero")
    return result / norm


def plane_basis(normal: np.ndarray) -> np.ndarray:
    """Return a 3x2 orthonormal basis perpendicular to the normal."""
    normal = unit_vector(normal, "plane normal")

    # Choose a reference that is not almost parallel to the normal; otherwise
    # their cross product loses numerical precision.
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(normal @ reference)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])

    first = unit_vector(np.cross(normal, reference))
    second = unit_vector(np.cross(normal, first))
    return np.column_stack((first, second))
