"""Prepared recording validation and incremental filtering, without UI or radio."""

import csv
import json
import math
from pathlib import Path

import numpy as np

from .filter import PositionVelocityKalmanFilter


ACCELERATION_COLUMNS = [f"a{axis}_world_m_s2" for axis in "xyz"]
POSITION_COLUMNS = [f"p{axis}_m" for axis in "xyz"]


def load_recording(path, metadata_path):
    # Windows editors may prepend a BOM; utf-8-sig accepts either UTF-8 form.
    metadata = json.loads(Path(metadata_path).read_text(encoding="utf-8-sig"))
    if not isinstance(metadata, dict):
        raise ValueError("recording metadata must be a JSON object")
    contract = metadata.get("prepared_replay", {})
    if not isinstance(contract, dict):
        raise ValueError("prepared_replay must be a JSON object")
    if (contract.get("acceleration_frame") != "world"
            or contract.get("gravity_removed") is not True
            or contract.get("acceleration_interval") != "preceding"
            or contract.get("position_source") not in ("onboard", "external")):
        raise ValueError("metadata must declare the prepared replay frame, gravity, interval and position source")
    samples = []
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"time_s", *ACCELERATION_COLUMNS, *POSITION_COLUMNS}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"recording needs columns: {sorted(required)}")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("recording rows must match the CSV header")
            time_s = float(row["time_s"])
            acceleration = np.array([float(row[name]) for name in ACCELERATION_COLUMNS])
            position_values = [row[name] for name in POSITION_COLUMNS]
            if any(position_values) and not all(position_values):
                raise ValueError("position components must all be present or all blank")
            position = np.array([float(value) for value in position_values]) if all(position_values) else None
            values = [time_s, *acceleration, *(position if position is not None else [])]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("prepared recordings must contain finite values")
            if samples and time_s <= samples[-1][0]:
                raise ValueError("prepared times must be strictly increasing; normalize clocks/duplicates first")
            samples.append((time_s, acceleration, position))
    if len(samples) < 2:
        raise ValueError("replay needs at least two samples")
    return samples, metadata


class ReplayFilter:
    """Incremental form shared by recorded replay and motor-free live logging."""

    def __init__(self, config):
        self.estimator = PositionVelocityKalmanFilter(config)
        self.previous_time = None

    def step(self, time_s, acceleration, position):
        estimator = self.estimator
        # Each prepared row supplies the process input for the interval ending here.
        if self.previous_time is not None:
            estimator.predict(time_s - self.previous_time, acceleration)
        self.previous_time = time_s
        accepted = estimator.update_position(position) if position is not None else None
        row = dict(time_s=time_s, nis=estimator.last_nis if position is not None else "",
                   accepted=int(accepted) if accepted is not None else "")
        for axis_index, axis in enumerate("xyz"):
            row[f"p{axis}_m"] = estimator.position[axis_index]
            row[f"v{axis}_m_s"] = estimator.velocity[axis_index]
            row[f"sigma_{axis}_m"] = math.sqrt(estimator.p[axis_index, axis_index])
            row[f"measurement_{axis}_m"] = position[axis_index] if position is not None else ""
            row[f"innovation_{axis}_m"] = estimator.last_innovation[axis_index] if position is not None else ""
        return row


def replay(samples, config):
    live_filter = ReplayFilter(config)
    return [live_filter.step(*sample) for sample in samples]
