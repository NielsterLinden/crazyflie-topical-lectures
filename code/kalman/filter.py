"""Linear position-velocity Kalman filter used by the starter exercises."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from race_common.dynamics import position_velocity_model


@dataclass
class FilterConfig:
    acceleration_noise_std: float = 1.5
    position_noise_std: float = 0.04
    flow_velocity_noise_std: float = 0.08
    range_noise_std: float = 0.03
    initial_position_std: float = 0.25
    initial_velocity_std: float = 0.50
    chi2_gate: float | None = None


class PositionVelocityKalmanFilter:
    """State [px, py, pz, vx, vy, vz], with acceleration as process input.

    Following Fruhwirth (1987): C is state covariance, V measurement
    covariance, and R predicted-residual covariance. Diagnostics retain the
    residual and its covariance before incorporating the current measurement.
    """

    def __init__(self, config: FilterConfig | None = None) -> None:
        self.config = config or FilterConfig()
        # state-vector: (position,velocity)
        self.x = np.zeros(6)
        # State covariance C; the p attribute remains available to existing callers.
        self.C = np.diag(
            [self.config.initial_position_std**2] * 3
            + [self.config.initial_velocity_std**2] * 3
        )
        self.last_predicted_residual = np.array([])
        # Store r_k^{k-1} and R_k^{k-1}, including when the update is rejected.
        self.last_predicted_residual_covariance = np.empty((0, 0))
        self.last_chi2 = float("nan")
        self.accepted_updates = 0
        self.rejected_updates = 0
        self.last_V = None

    @property
    def p(self) -> np.ndarray:
        """Compatibility name for the state covariance C."""
        return self.C

    @p.setter
    def p(self, covariance: np.ndarray) -> None:
        self.C = covariance

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    @property
    def position_covariance(self) -> np.ndarray:
        return self.C[:3, :3].copy()

    def predict(self, dt: float, acceleration_world: np.ndarray) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        acceleration = np.asarray(acceleration_world, dtype=float)
        if acceleration.shape != (3,):
            raise ValueError("acceleration_world must contain ax, ay, az")

        # F transports the state; B maps a known acceleration into position
        # and velocity increments. Both are exact for interval-held acceleration.
        transition, acceleration_input = position_velocity_model(dt)

        # Model one independent, unknown acceleration perturbation held over
        # this sample interval.
        acceleration_covariance = np.eye(3) * self.config.acceleration_noise_std**2
        process_noise = (
            acceleration_input @ acceleration_covariance @ acceleration_input.T
        )

        self.x = transition @ self.x + acceleration_input @ acceleration
        self.C = transition @ self.C @ transition.T + process_noise

    def update_position(self, measurement: np.ndarray) -> bool:
        # H selects position; V is the covariance of its measurement errors.
        H = np.hstack((np.eye(3), np.zeros((3, 3))))
        V = np.eye(3) * self.config.position_noise_std**2
        return self._update(measurement, H, V)

    def update_flow_velocity(self, measurement_xy: np.ndarray) -> bool:
        H = np.zeros((2, 6))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        V = np.eye(2) * self.config.flow_velocity_noise_std**2
        return self._update(measurement_xy, H, V)

    def update_range(self, height_m: float) -> bool:
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        V = np.array([[self.config.range_noise_std**2]])
        return self._update(np.array([height_m]), H, V)

    def update_zero_velocity(self, noise_std: float = 0.02) -> bool:
        H = np.hstack((np.zeros((3, 3)), np.eye(3)))
        V = np.eye(3) * noise_std**2
        return self._update(np.zeros(3), H, V)

    def _update(self, measurement: np.ndarray, H: np.ndarray, V: np.ndarray) -> bool:
        # On entry x and C are x_k^{k-1} and C_k^{k-1}; m is m_k.
        self.last_V = V.copy()
        m = np.asarray(measurement, dtype=float)
        residual = m - H @ self.x
        R = H @ self.C @ H.T + V
        # The same inverse enters the chi-square increment and the gain.
        R_inverse = np.linalg.inv(R)
        chi2 = float(residual @ R_inverse @ residual)

        self.last_predicted_residual = residual
        self.last_predicted_residual_covariance = R
        self.last_chi2 = chi2

        # Callers may select chi2_gate per update, as in Day 2's threshold exercise.
        if self.config.chi2_gate is not None and chi2 > self.config.chi2_gate:
            self.rejected_updates += 1
            return False

        kalman_gain = self.C @ H.T @ R_inverse
        self.x = self.x + kalman_gain @ residual

        # Joseph form preserves symmetry and positive semidefiniteness better.
        identity = np.eye(6)
        correction = identity - kalman_gain @ H
        self.C = correction @ self.C @ correction.T + kalman_gain @ V @ kalman_gain.T
        self.C = 0.5 * (self.C + self.C.T)
        self.accepted_updates += 1
        return True
