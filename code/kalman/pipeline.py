"""Sensor callback interface shared by simulation and live-flight adapters."""

from __future__ import annotations

import numpy as np

from .filter import PositionVelocityKalmanFilter


class SensorFusionPipeline:
    def __init__(self, estimator: PositionVelocityKalmanFilter) -> None:
        self.estimator = estimator
        self.last_imu_time_s: float | None = None

    def on_imu(self, timestamp_s: float, acceleration_world: np.ndarray) -> None:
        """Predict using gravity-compensated acceleration in the world frame."""
        if self.last_imu_time_s is not None:
            self.estimator.predict(timestamp_s - self.last_imu_time_s, acceleration_world)
        self.last_imu_time_s = timestamp_s

    def on_position(self, position_m: np.ndarray) -> bool:
        return self.estimator.update_position(position_m)

    def on_flow_velocity(self, velocity_xy_m_s: np.ndarray) -> bool:
        return self.estimator.update_flow_velocity(velocity_xy_m_s)

    def on_range(self, height_m: float) -> bool:
        return self.estimator.update_range(height_m)

    def on_stationary(self) -> bool:
        return self.estimator.update_zero_velocity()

    # TODO 6 (optional extension): add a cflib adapter that maps logged variables into
    # these callbacks. Keep frame conversion, units, and timestamps explicit.
    # The 30-minute E6 core uses supplied live.py, with replay.py as fallback.
