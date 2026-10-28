"""Reproducible, offline sensor scenarios for the E3-E5 experiments."""

from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np

from .exercise_policies import StationaryWindow
from kalman.recording import load_recording
from flight_control.trajectory import reference_state


GRAVITY = 9.80665
SCENARIOS = ("nominal", "dropout", "outlier", "range-step", "manoeuvre", "stationary")
SENSOR_NAMES = ("position", "flow", "range")


@dataclass
class ExperimentData:
    time: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    measured_acceleration: np.ndarray
    measured_position: np.ndarray
    measured_flow: np.ndarray
    measured_range: np.ndarray
    gyro: np.ndarray
    specific_force: np.ndarray
    motors: np.ndarray
    scenario: str
    has_truth: bool = True
    metadata: dict | None = None


def load_prepared_data(recording: Path, metadata_path: Path, scenario: str) -> ExperimentData:
    """Use the existing live/prepared format without inventing absent sensors."""
    if scenario not in ("nominal", "dropout", "outlier"):
        raise ValueError("prepared recordings support nominal, dropout or position outlier only; use synthetic scenarios for other sensors")
    samples, metadata = load_recording(recording, metadata_path)
    time = np.array([row[0] - samples[0][0] for row in samples])
    if np.any(np.diff(time) > 0.30 + 1e-9):
        raise ValueError("recording gaps exceed 0.30 s; use the checked recording or synthetic fallback")
    first_position = next((index for index, row in enumerate(samples) if row[2] is not None), None)
    if first_position is None:
        raise ValueError("prepared recording needs a position observation for initialization")
    # Absolute position is unanchored before this observation. Start analysis
    # here rather than propagating an arbitrary origin or back-filling position.
    time = time[first_position:]
    analysis_origin = samples[0][0]
    samples = samples[first_position:]
    acceleration = np.array([row[1] for row in samples])
    position = np.array([row[2] if row[2] is not None else np.full(3, np.nan) for row in samples])
    metadata = dict(metadata, input_recording=str(recording), input_metadata=str(metadata_path),
                    analysis_time_origin_s=float(analysis_origin),
                    initialization_time_s=float(time[0]),
                    skipped_initial_samples=first_position,
                    input_sha256=hashlib.sha256(recording.read_bytes()).hexdigest(),
                    input_metadata_sha256=hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
                    interventions=[])
    if scenario == "outlier":
        candidates = np.flatnonzero((time >= 12.0) & np.all(np.isfinite(position), axis=1))
        if not len(candidates):
            raise ValueError("outlier study needs a position observation at or after 12 s; use the checked 30 s recording or synthetic fallback")
        index = candidates[0]
        position[index, 1] += 0.65
        metadata["interventions"].append(dict(kind="software_position_outlier", time_s=float(time[index]), offset_y_m=0.65))
    absent = np.full((len(time), 3), np.nan)
    return ExperimentData(time, absent.copy(), absent.copy(), absent.copy(), acceleration,
                          position, np.full((len(time), 2), np.nan), np.full(len(time), np.nan),
                          absent.copy(), absent.copy(), np.full((len(time), 4), np.nan),
                          scenario, has_truth=False, metadata=metadata)


def stationary_motion(time_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rest, smooth departure, constant velocity, smooth arrival, rest.

    Integrating a cubic velocity ramp gives continuous velocity and acceleration.
    The constant-speed part is intentionally indistinguishable from rest by an
    ideal nonrotating IMU; optical flow supplies the extra motion cue.
    """
    speed = 0.3
    if time_s < 4.0:
        distance, velocity, acceleration = 0.0, 0.0, 0.0
    elif time_s < 6.0:
        u = (time_s - 4.0) / 2.0
        distance = 2.0 * speed * (u**3 - 0.5 * u**4)
        velocity = speed * (3.0 * u**2 - 2.0 * u**3)
        acceleration = speed * (6.0 * u - 6.0 * u**2) / 2.0
    elif time_s < 12.0:
        distance, velocity, acceleration = speed * (time_s - 5.0), speed, 0.0
    elif time_s < 14.0:
        u = (time_s - 12.0) / 2.0
        distance = 7.0 * speed + 2.0 * speed * (u - u**3 + 0.5 * u**4)
        velocity = speed * (1.0 - 3.0 * u**2 + 2.0 * u**3)
        acceleration = -speed * (6.0 * u - 6.0 * u**2) / 2.0
    else:
        distance, velocity, acceleration = 8.0 * speed, 0.0, 0.0
    return (np.array([distance, 0.0, 1.0]), np.array([velocity, 0.0, 0.0]),
            np.array([acceleration, 0.0, 0.0]))


def generate_data(seed: int, scenario: str, duration_s: float = 18.0) -> ExperimentData:
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario: {scenario}")
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration must be finite and positive")
    time = np.arange(int(round(duration_s / 0.02)) + 1) * 0.02
    position, velocity, acceleration = [], [], []
    for timestamp in time:
        if scenario == "stationary":
            p, v, a = stationary_motion(timestamp)
        else:
            reference = reference_state(timestamp)
            p, v, a = reference.position.copy(), reference.velocity.copy(), reference.acceleration.copy()
            if scenario == "manoeuvre":
                # One finite acceleration pulse, with continuous position/velocity.
                elapsed = np.clip(timestamp - 8.0, 0.0, 1.0)
                p[0] += 1.5 * elapsed**2 + 3.0 * max(timestamp - 9.0, 0.0)
                v[0] += 3.0 * elapsed
                a[0] += 3.0 if 8.0 <= timestamp < 9.0 else 0.0
        position.append(p)
        velocity.append(v)
        acceleration.append(a)
    position, velocity, acceleration = map(np.asarray, (position, velocity, acceleration))

    # Draw every channel at every time before scheduling or disabling sensors.
    # Changing R, gating or dropout availability cannot change the noise draws.
    streams = [np.random.default_rng(child) for child in np.random.SeedSequence(seed).spawn(5)]
    acceleration_noise = streams[0].normal(0.0, 0.08, acceleration.shape)
    measured_acceleration = acceleration + acceleration_noise
    if scenario == "manoeuvre":
        # A declared 200 ms input delay exposes mismatch during the manoeuvre.
        delayed = np.maximum(np.arange(len(time)) - 10, 0)
        measured_acceleration = acceleration[delayed] + acceleration_noise
    if scenario == "stationary":
        measured_acceleration += np.array([0.03, 0.0, 0.0])
    measured_position = position + streams[1].normal(0.0, 0.04, position.shape)
    measured_flow = velocity[:, :2] + streams[2].normal(0.0, 0.08, (len(time), 2))
    measured_range = position[:, 2] + streams[3].normal(0.0, 0.03, len(time))
    gyro = streams[4].normal(0.0, 0.002, acceleration.shape)
    specific_force = measured_acceleration + np.array([0.0, 0.0, GRAVITY])
    if scenario == "outlier":
        measured_position[np.isclose(time, 12.0)] += np.array([0.0, 0.65, 0.0])
    if scenario == "range-step":
        measured_range[(time >= 8.0) & (time < 11.0)] -= 0.35
    return ExperimentData(time, position, velocity, acceleration, measured_acceleration,
                          measured_position, measured_flow, measured_range, gyro,
                          specific_force, np.zeros((len(time), 4)), scenario)


def sensor_available(sensor: str, sample: int, timestamp: float, disabled: tuple[str, ...],
                     dropout_s: float) -> bool:
    if sensor in disabled:
        return False
    if sensor == "position" and dropout_s > 0.0 and 7.0 <= timestamp <= 7.0 + dropout_s:
        return False
    period = {"position": 5, "flow": 2, "range": 5}[sensor]
    return sample % period == 0


def stationary_features(data: ExperimentData, sample: int,
                        flow_samples: list[tuple[float, np.ndarray]]) -> StationaryWindow:
    timestamp = data.time[sample]
    start = max(0, sample - 50)
    selection = slice(start, sample + 1)
    norms = np.linalg.norm(data.specific_force[selection], axis=1)
    recent_flow = [value for time, value in flow_samples if timestamp - 1.0 <= time <= timestamp]
    fresh = bool(flow_samples) and timestamp - flow_samples[-1][0] <= 0.1
    speed = float(np.linalg.norm(np.mean(recent_flow, axis=0))) if fresh and recent_flow else None
    return StationaryWindow(
        duration_s=float((sample - start) * 0.02),
        gyro_max_rad_s=float(np.max(np.linalg.norm(data.gyro[selection], axis=1))),
        acceleration_norm_mean_m_s2=float(np.mean(norms)),
        acceleration_norm_std_m_s2=float(np.std(norms)),
        motors_idle=bool(np.all(data.motors[selection] == 0.0)),
        flow_speed_m_s=speed,
    )
