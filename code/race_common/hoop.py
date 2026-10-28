"""Hoop geometry and covariance projection helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from race_common.geometry import plane_basis, unit_vector


@dataclass(frozen=True)
class Hoop:
    centre: np.ndarray
    normal: np.ndarray
    radius_m: float
    thickness_m: float = 0.02

    def __post_init__(self) -> None:
        centre = np.asarray(self.centre, dtype=float)
        normal = unit_vector(self.normal, "normal")
        if centre.shape != (3,):
            raise ValueError("centre must contain x, y, z")
        if self.radius_m <= 0.0:
            raise ValueError("radius_m must be positive")
        object.__setattr__(self, "centre", centre)
        object.__setattr__(self, "normal", normal)

    def plane_basis(self) -> np.ndarray:
        """Return a 3x2 orthonormal basis spanning the hoop plane."""
        return plane_basis(self.normal)

    def signed_plane_distance(self, position: np.ndarray) -> float:
        return float(self.normal @ (np.asarray(position) - self.centre))

    def radial_offset(self, position: np.ndarray) -> float:
        relative = np.asarray(position, dtype=float) - self.centre
        in_plane = relative - self.normal * float(self.normal @ relative)
        return float(np.linalg.norm(in_plane))

    def plane_covariance(self, position_covariance: np.ndarray) -> np.ndarray:
        basis = self.plane_basis()
        covariance = np.asarray(position_covariance, dtype=float)
        if covariance.shape != (3, 3):
            raise ValueError("position_covariance must be 3x3")
        return basis.T @ covariance @ basis

    def conservative_clearance(
        self,
        position: np.ndarray,
        position_covariance: np.ndarray,
        drone_radius_m: float,
        tracking_error_m: float,
        confidence_multiplier: float,
    ) -> float:
        plane_covariance = self.plane_covariance(position_covariance)
        sigma_worst = float(np.sqrt(np.max(np.linalg.eigvalsh(plane_covariance))))
        uncertainty_margin = confidence_multiplier * sigma_worst
        return (
            self.radius_m
            - self.radial_offset(position)
            - drone_radius_m
            - tracking_error_m
            - uncertainty_margin
        )
