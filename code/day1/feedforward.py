#!/usr/bin/env python3
"""Compare P, PD, and PD plus feed-forward trajectory control."""

from __future__ import annotations

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from flight_control.controller import ControllerConfig, OuterLoopController
from flight_control.trajectory import reference_state


@dataclass(frozen=True)
class ControlMetrics:
    rms_tracking_error_m: float
    nominal_rms_tracking_error_m: float
    maximum_tracking_error_m: float
    integrated_control_effort: float
    saturation_fraction: float


def run_controller(
    name: str,
    config: ControllerConfig,
    duration_s: float = 18.0,
    dt: float = 0.01,
) -> dict[str, np.ndarray | ControlMetrics | str]:
    controller = OuterLoopController(config)
    initial = reference_state(0.0)
    position = initial.position.copy()
    velocity = initial.velocity.copy()
    actual_acceleration = np.zeros(3)
    actuator_time_constant_s = 0.12

    time_values = []
    reference_values = []
    position_values = []
    acceleration_values = []
    bank_values = []
    saturation_values = []
    effort_values = []

    for sample in range(int(duration_s / dt) + 1):
        timestamp_s = sample * dt
        reference = reference_state(timestamp_s)
        command = controller.command(reference, position, velocity)

        # Repeatable crosswind-like disturbance for controller comparison.
        disturbance = np.array([0.0, 0.45 if 7.0 <= timestamp_s <= 10.0 else 0.0, 0.0])
        demanded_acceleration = command.acceleration_world + disturbance
        actual_acceleration += (
            dt / actuator_time_constant_s * (demanded_acceleration - actual_acceleration)
        )
        velocity += actual_acceleration * dt
        position += velocity * dt

        time_values.append(timestamp_s)
        reference_values.append(reference.position)
        position_values.append(position.copy())
        acceleration_values.append(command.acceleration_world)
        bank_values.append(command.required_bank_rad)
        saturation_values.append(command.saturated)
        effort_values.append(command.squared_effort)

    times = np.asarray(time_values)
    references = np.asarray(reference_values)
    positions = np.asarray(position_values)
    errors = np.linalg.norm(positions - references, axis=1)
    nominal_errors = errors[times < 7.0]
    metrics = ControlMetrics(
        rms_tracking_error_m=float(np.sqrt(np.mean(errors**2))),
        nominal_rms_tracking_error_m=float(np.sqrt(np.mean(nominal_errors**2))),
        maximum_tracking_error_m=float(np.max(errors)),
        integrated_control_effort=float(np.trapezoid(effort_values, times)),
        saturation_fraction=float(np.mean(saturation_values)),
    )
    return {
        "name": name,
        "time": times,
        "reference": references,
        "position": positions,
        "acceleration": np.asarray(acceleration_values),
        "bank": np.asarray(bank_values),
        "saturated": np.asarray(saturation_values),
        "metrics": metrics,
    }


def run_comparison() -> list[dict[str, np.ndarray | ControlMetrics | str]]:
    return [
        run_controller(
            "P",
            ControllerConfig(velocity_gain=(0.0, 0.0, 0.0), feedforward_gain=0.0),
        ),
        run_controller("PD", ControllerConfig(feedforward_gain=0.0)),
        run_controller("PD + feed-forward", ControllerConfig(feedforward_gain=1.0)),
    ]


def plot(results: list[dict[str, np.ndarray | ControlMetrics | str]]) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    reference = results[0]["reference"]
    axes[0, 0].plot(reference[:, 0], reference[:, 1], "k--", label="reference")

    for result in results:
        name = result["name"]
        times = result["time"]
        positions = result["position"]
        errors = np.linalg.norm(positions - result["reference"], axis=1)
        axes[0, 0].plot(positions[:, 0], positions[:, 1], label=name)
        axes[0, 1].plot(times, errors, label=name)
        axes[1, 0].plot(times, np.rad2deg(result["bank"]), label=name)
        axes[1, 1].plot(times, np.linalg.norm(result["acceleration"], axis=1), label=name)

    axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Closed-loop trajectory")
    axes[0, 1].axvspan(7.0, 10.0, color="0.9", label="disturbance")
    axes[0, 1].set(xlabel="time [s]", ylabel="error [m]", title="Tracking error")
    axes[1, 0].set(xlabel="time [s]", ylabel="bank [deg]", title="Required bank")
    axes[1, 1].set(xlabel="time [s]", ylabel="acceleration [m/s2]", title="Control effort")
    for axis in axes.flat:
        axis.legend(fontsize="small")
    figure.tight_layout()

    for result in results:
        print(f"{result['name']}: {result['metrics']}")
    plt.show()


if __name__ == "__main__":
    plot(run_comparison())
