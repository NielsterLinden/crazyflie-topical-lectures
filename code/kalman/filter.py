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
    nis_gate: float | None = None


class PositionVelocityKalmanFilter:
    """State [px, py, pz, vx, vy, vz], with acceleration as process input."""

    def __init__(self, config: FilterConfig | None = None) -> None:
        self.config = config or FilterConfig()
        self.x = np.zeros(6)
        self.p = np.diag(
            [self.config.initial_position_std**2] * 3
            + [self.config.initial_velocity_std**2] * 3
        )
        self.last_innovation = np.array([])
        self.last_innovation_covariance = np.empty((0, 0))
        self.last_nis = float("nan")
        self.accepted_updates = 0
        self.rejected_updates = 0

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    @property
    def position_covariance(self) -> np.ndarray:
        return self.p[:3, :3].copy()

    def predict(self, dt: float, acceleration_world: np.ndarray) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        acceleration = np.asarray(acceleration_world, dtype=float)
        if acceleration.shape != (3,):
            raise ValueError("acceleration_world must contain ax, ay, az")

        transition, acceleration_input = position_velocity_model(dt)

        # Model one independent, unknown acceleration perturbation held over
        # this sample interval.
        acceleration_covariance = np.eye(3) * self.config.acceleration_noise_std**2
        process_noise = (
            acceleration_input @ acceleration_covariance @ acceleration_input.T
        )

        self.x = transition @ self.x + acceleration_input @ acceleration
        self.p = transition @ self.p @ transition.T + process_noise

        # TODO 1: model accelerometer bias as part of the state.
        # TODO 2: increase process noise during aggressive manoeuvres.

    def update_position(self, measurement: np.ndarray) -> bool:
        h = np.hstack((np.eye(3), np.zeros((3, 3))))
        r = np.eye(3) * self.config.position_noise_std**2
        return self._update(measurement, h, r)

    def update_flow_velocity(self, measurement_xy: np.ndarray) -> bool:
        h = np.zeros((2, 6))
        h[0, 3] = 1.0
        h[1, 4] = 1.0
        r = np.eye(2) * self.config.flow_velocity_noise_std**2
        return self._update(measurement_xy, h, r)

    def update_range(self, height_m: float) -> bool:
        h = np.zeros((1, 6))
        h[0, 2] = 1.0
        r = np.array([[self.config.range_noise_std**2]])
        return self._update(np.array([height_m]), h, r)

    def update_zero_velocity(self, noise_std: float = 0.02) -> bool:
        h = np.hstack((np.zeros((3, 3)), np.eye(3)))
        r = np.eye(3) * noise_std**2
        return self._update(np.zeros(3), h, r)

    def _update(self, measurement: np.ndarray, h: np.ndarray, r: np.ndarray) -> bool:
        measurement = np.asarray(measurement, dtype=float)
        innovation = measurement - h @ self.x
        innovation_covariance = h @ self.p @ h.T + r
        solved_innovation = np.linalg.solve(innovation_covariance, innovation)
        nis = float(innovation.T @ solved_innovation)

        self.last_innovation = innovation
        self.last_innovation_covariance = innovation_covariance
        self.last_nis = nis

        # Callers may select nis_gate per update, as in Day 2's threshold exercise.
        if self.config.nis_gate is not None and nis > self.config.nis_gate:
            self.rejected_updates += 1
            return False

        kalman_gain = np.linalg.solve(innovation_covariance, h @ self.p).T
        self.x = self.x + kalman_gain @ innovation

        # Joseph form preserves symmetry and positive semidefiniteness better.
        identity = np.eye(6)
        residual = identity - kalman_gain @ h
        self.p = residual @ self.p @ residual.T + kalman_gain @ r @ kalman_gain.T
        self.p = 0.5 * (self.p + self.p.T)
        self.accepted_updates += 1
        return True
