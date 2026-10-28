"""Outer-loop trajectory controller for the hoop-race exercises."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from race_common.dynamics import limit_acceleration


def _xyz(values: tuple[float, float, float]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,):
        raise ValueError("controller gains must contain x, y, z values")
    return array


@dataclass(frozen=True)
class TrajectoryReference:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    def __post_init__(self) -> None:
        for name in ("position", "velocity", "acceleration"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,):
                raise ValueError(f"{name} must contain x, y, z")
            object.__setattr__(self, name, value)


@dataclass
class ControllerConfig:
    position_gain: tuple[float, float, float] = (2.0, 2.0, 3.0)
    velocity_gain: tuple[float, float, float] = (1.6, 1.6, 2.0)
    feedforward_gain: float = 1.0
    maximum_acceleration_m_s2: float = 4.0
    maximum_bank_deg: float = 30.0


@dataclass(frozen=True)
class ControlCommand:
    acceleration_world: np.ndarray
    required_bank_rad: float
    saturated: bool
    squared_effort: float


class OuterLoopController:
    """PD plus acceleration feed-forward with acceleration and bank limits."""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()
        self.kp = _xyz(self.config.position_gain)
        self.kv = _xyz(self.config.velocity_gain)

    def command(
        self,
        reference: TrajectoryReference,
        estimated_position: np.ndarray,
        estimated_velocity: np.ndarray,
    ) -> ControlCommand:
        position = np.asarray(estimated_position, dtype=float)
        velocity = np.asarray(estimated_velocity, dtype=float)
        if position.shape != (3,) or velocity.shape != (3,):
            raise ValueError("position and velocity estimates must contain x, y, z")

        position_error = reference.position - position
        velocity_error = reference.velocity - velocity
        requested = (
            self.config.feedforward_gain * reference.acceleration
            + self.kp * position_error
            + self.kv * velocity_error
        )

        command_limit = limit_acceleration(
            requested,
            self.config.maximum_acceleration_m_s2,
            np.deg2rad(self.config.maximum_bank_deg),
        )
        limited = command_limit.acceleration_world_m_s2

        # TODO 7: add integral action and anti-windup, then demonstrate that it
        # improves rejection of a constant disturbance without harming recovery.
        return ControlCommand(
            acceleration_world=limited,
            required_bank_rad=command_limit.required_tilt_rad,
            saturated=command_limit.saturated,
            squared_effort=float(limited @ limited),
        )
