#!/usr/bin/env python3
"""Reusable synthetic estimation baseline, independent of gates and plotting."""

from __future__ import annotations

import numpy as np

from .filter import FilterConfig, PositionVelocityKalmanFilter
from .pipeline import SensorFusionPipeline
from flight_control.trajectory import reference_state


SCENARIOS = ("nominal", "dropout", "outlier", "combined")


def run(seed: int = 7, scenario: str = "combined") -> dict[str, np.ndarray]:
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario: {scenario}")
    random = np.random.default_rng(seed)
    estimator = PositionVelocityKalmanFilter(FilterConfig())
    pipeline = SensorFusionPipeline(estimator)

    records: dict[str, list] = {
        "time": [],
        "truth": [],
        "estimate": [],
        "position_std": [],
        "position_covariance": [],
        "position_measurement": [],
        "chi2": [],
    }

    dt = 0.02
    for sample in range(int(18.0 / dt) + 1):
        timestamp_s = sample * dt
        sample_chi2 = []
        reference = reference_state(timestamp_s)
        true_position = reference.position
        true_velocity = reference.velocity
        true_acceleration = reference.acceleration
        measured_acceleration = true_acceleration + random.normal(0.0, 0.08, 3)
        pipeline.on_imu(timestamp_s, measured_acceleration)

        position_measurement = np.full(3, np.nan)
        # Draw scheduled observations even during dropout, keeping all noise
        # draws identical across scenarios with the same seed.
        if sample % 5 == 0:
            observation = true_position + random.normal(0.0, 0.04, 3)
            if scenario in ("outlier", "combined") and abs(timestamp_s - 12.0) < dt / 2.0:
                observation += np.array([0.0, 0.65, 0.0])
            dropout = scenario in ("dropout", "combined") and 7.0 <= timestamp_s <= 10.0
            if not dropout:
                position_measurement = observation
                pipeline.on_position(position_measurement)
                sample_chi2.append(estimator.last_chi2)

        # Simulated 25 Hz horizontal optical-flow velocity and 10 Hz range.
        if sample % 2 == 0:
            pipeline.on_flow_velocity(true_velocity[:2] + random.normal(0.0, 0.08, 2))
            sample_chi2.append(estimator.last_chi2)
        if sample % 5 == 0:
            pipeline.on_range(float(true_position[2] + random.normal(0.0, 0.03)))
            sample_chi2.append(estimator.last_chi2)

        records["time"].append(timestamp_s)
        records["truth"].append(true_position)
        records["estimate"].append(estimator.position)
        records["position_std"].append(np.sqrt(np.diag(estimator.position_covariance)))
        records["position_covariance"].append(estimator.position_covariance)
        records["position_measurement"].append(position_measurement)
        records["chi2"].append(max(sample_chi2) if sample_chi2 else np.nan)

    return {key: np.asarray(value) for key, value in records.items()}
