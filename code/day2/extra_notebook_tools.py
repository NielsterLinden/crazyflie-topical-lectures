"""Optional six-state recording and experiment helpers."""

from dataclasses import asdict
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from .experiment_data import generate_data, load_prepared_data
from .experiments import (plot_comparison, plot_diagnostics, plot_stationarity,
                          run_experiment, save_metrics, save_run, summarize,
                          validate_position_dropout)
from .notebook_tools import _run_capture


def load_capture_overview(directory):
    """Align every recorded stream to the actual prepared-data origin."""
    from .live import CHANNELS, PRIORITY, SampleAssembler
    from kalman.recording import load_recording
    from scipy.spatial.transform import Rotation

    directory = Path(directory)
    samples, metadata = load_recording(directory / "prepared.csv", directory / "recording.json")
    if metadata.get("status") != "complete":
        raise ValueError("Select a complete capture for inspection.")
    for name, digest in metadata.get("source_sha256", {}).items():
        if name in ("raw.csv", "prepared.csv") and hashlib.sha256((directory / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Capture file changed: {name}")
    clock = SampleAssembler(metadata.get("accelerometer_bias_subtracted_g", (0, 0, 0)))
    packets = []
    with (directory / "raw.csv").open(encoding="utf-8-sig", newline="") as stream:
        for index, row in enumerate(csv.DictReader(stream)):
            stamp = clock.unwrap(int(row["firmware_timestamp_ms"]))
            sensor = row["stream"]
            values = json.loads(row["values_json"])
            if sensor not in CHANNELS:
                raise ValueError(f"Unknown recorded stream: {sensor}")
            packets.append((stamp, PRIORITY[sensor], index, sensor, values))
    packets.sort(key=lambda packet: packet[:3])
    origin = metadata.get("time_origin_firmware_ms")
    if origin is None:
        # Older captures lack the origin field. Recover it with the same causal
        # pairing and verify the first prepared sample before using that clock.
        pairing = SampleAssembler(metadata.get("accelerometer_bias_subtracted_g", (0, 0, 0)))
        for stamp, _, _, sensor, values in packets:
            sample = pairing.accept(sensor, stamp, values)
            if sample is not None:
                if (not np.allclose(sample[1], samples[0][1])
                        or samples[0][2] is None or not np.allclose(sample[2], samples[0][2])):
                    raise ValueError("Cannot align this older capture to its prepared data; use a fresh capture.")
                origin = stamp - samples[0][0] * 1000
                break
        if origin is None:
            raise ValueError("No usable sample establishes the capture time origin.")
    if not math.isfinite(origin):
        raise ValueError("Capture time origin must be finite.")
    result = {"prepared_time_s": np.array([sample[0] for sample in samples]),
              "world_acceleration": np.array([sample[1] for sample in samples]),
              "position": np.array([sample[2] for sample in samples]),
              "time_origin_firmware_ms": origin}
    for sensor in ("imu_position", "attitude", "velocity"):
        selected = [packet for packet in packets if packet[3] == sensor]
        if sensor != "velocity" and not selected:
            raise ValueError(f"Capture lacks {sensor} telemetry.")
        times = np.array([(packet[0] - origin) / 1000 for packet in selected])
        values = np.array([[packet[4][name] for name in CHANNELS[sensor]] for packet in selected], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Non-finite {sensor} telemetry.")
        result[sensor] = (times, values)
    quaternions = result["attitude"][1]
    if np.any(np.abs(np.linalg.norm(quaternions, axis=1) - 1) > .05):
        raise ValueError("Invalid recorded attitude quaternion.")
    angles = Rotation.from_quat(quaternions).as_euler("xyz")
    result["attitude"] = (result["attitude"][0], np.rad2deg(np.unwrap(angles, axis=0)))
    return result



def plot_capture_overview(overview, destination):
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(5, 1, figsize=(13, 14), sharex=True)
    body_time, body = overview["imu_position"]
    attitude_time, angles = overview["attitude"]
    velocity_time, velocity = overview["velocity"]
    for index, name in enumerate("xyz"):
        axes[0].plot(body_time, body[:, index], label=name)
        axes[1].plot(attitude_time, angles[:, index], label=("roll", "pitch", "yaw")[index])
        axes[2].plot(overview["prepared_time_s"], overview["world_acceleration"][:, index], label=name)
        axes[3].plot(overview["prepared_time_s"], overview["position"][:, index], label=name)
        if len(velocity_time):
            axes[4].plot(velocity_time, velocity[:, index], label=name)
    for axis, label in zip(axes, ("Body specific force [g]", "Onboard attitude [deg]",
                                  "World acceleration [m/s2]", "Onboard position [m]", "Onboard velocity [m/s]")):
        axis.set_ylabel(label)
        axis.grid(True, alpha=.3)
        if axis.lines:
            axis.legend(ncol=3)
    if not len(velocity_time):
        axes[4].text(.5, .5, "Onboard velocity was not logged.", transform=axes[4].transAxes, ha="center")
    axes[-1].set_xlabel("Time from first usable prepared sample [s]")
    figure.tight_layout()
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=140)
    plt.close(figure)
    return destination



def capture_live(output, uri, *, duration_s=60, ready_for_motor_stop=False,
                 position_std=0.04, comparison_position_std=0.12):
    """Run the motor-free logger with a wall-clock bound, including connection.

    A child process isolates radio callbacks from the notebook kernel. The
    existing logger owns pairing, motor checks, journaling and filter outputs.
    Failed captures stay on disk for inspection and are never selected as input.
    """
    if not ready_for_motor_stop:
        raise ValueError("Confirm the drone is on a stable surface, clear of propellers, "
                         "and other radio clients are closed before engaging the firmware stop.")
    if not isinstance(uri, str) or not uri.strip():
        raise ValueError("Run the notebook's Connect cell and pass its returned URI.")
    if not math.isfinite(duration_s) or not 1 <= duration_s <= 120:
        raise ValueError("Notebook capture duration must be between 1 and 120 seconds.")
    if any(not math.isfinite(value) or value <= 0
           for value in (position_std, comparison_position_std)):
        raise ValueError("Position standard deviations must be finite and positive.")
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Choose a new capture directory: {output}")
    command = [sys.executable, "-u", "-m", "day2.live", "--uri", uri.strip(),
               "--output", str(output), "--duration", str(duration_s),
               "--position-std", str(position_std),
               "--comparison-position-std", str(comparison_position_std),
               "--prepare-for-handling", "--no-show"]
    timeout_s = duration_s + 30
    print(f"Motor-free capture: {duration_s:g} s after first sample; "
          f"overall limit {timeout_s:g} s. Leave the drone on the surface until STOP VERIFIED.", flush=True)
    environment = dict(os.environ, MPLBACKEND="Agg")
    try:
        result = _run_capture(command, cwd=Path(__file__).resolve().parents[1],
                              env=environment, timeout_s=timeout_s)
    except subprocess.TimeoutExpired as error:
        raise TimeoutError(f"Capture exceeded {timeout_s:g} s and was stopped. "
                           f"Any partial data remain in {output}; select a checked recording "
                           "or the explicit simulation fallback.") from error
    except KeyboardInterrupt:
        print(f"Capture interrupted; child stopped. Any partial data remain in {output}.")
        raise
    if result.returncode != 0:
        raise RuntimeError(f"Capture failed; any partial data remain in {output}.\n"
                           f"{result.stdout}{result.stderr}\nSelect a checked recording or the explicit simulation fallback.")
    from kalman.recording import load_recording

    recording, metadata = output / "prepared.csv", output / "recording.json"
    samples, details = load_recording(recording, metadata)
    if details.get("status") != "complete" or len(samples) < 2:
        raise RuntimeError("Capture did not complete; inspect saved data before choosing a fallback.")
    return recording, metadata



def select_data(recording=None, metadata=None, *, seed=0, scenario="nominal"):
    """Use a chosen recording, or explicitly labelled simulation when neither is set."""
    if (recording is None) != (metadata is None):
        raise ValueError("provide both recording and metadata, or neither for simulation")
    if recording is None:
        return generate_data(seed, scenario)
    # Bad or missing selected files remain errors; never silently replace them.
    return load_prepared_data(Path(recording), Path(metadata), scenario)



def compare(data, cases, output, *, seed=0, alpha=0.01,
            predicted_residual_policy=None, stationary_policy=None, notes=""):
    """Run matched cases, preserving the CLI's CSVs, plots and uncertainty summaries."""
    if data is None:
        raise ValueError("Rerun the data-selection cell successfully before continuing.")
    names = [case.name for case in cases]
    if not names or len(set(names)) != len(names):
        raise ValueError("provide cases with unique names")
    if any(Path(name).name != name or name in (".", "..") or "\\" in name for name in names):
        raise ValueError("case names must be simple filenames")
    for case in cases:
        validate_position_dropout(data, case)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    runs, summaries = [], []
    for case in cases:
        result = run_experiment(data, case, alpha,
                                predicted_residual_policy=predicted_residual_policy,
                                stationary_policy=stationary_policy)
        directory = output / case.name
        save_run(directory, result, data)
        plot_diagnostics(result, directory / "diagnostics.png", alpha, data.has_truth)
        if result["stationarity"]:
            plot_stationarity(result, directory / "stationarity.png")
        runs.append((case, seed, result, data))
        summaries.append(summarize(result, data, case, seed))
    plot_comparison(runs, output / "comparison.png")
    save_metrics(output, summaries)
    (output / "summary.json").write_text(json.dumps(summaries, indent=2, allow_nan=False) + "\n")
    settings = dict(seed=seed, alpha=alpha, has_truth=data.has_truth,
                    source=data.metadata if not data.has_truth else {"synthetic": True, "scenario": data.scenario},
                    cases=[asdict(case) for case in cases], notes=notes)
    for name, policy in (("predicted_residual_policy", predicted_residual_policy), ("stationary_policy", stationary_policy)):
        if policy is not None:
            try:
                settings[name] = inspect.getsource(policy)
            except (OSError, TypeError):
                settings[name] = "Defined in notebook; save the edited notebook alongside these results."
    (output / "settings.json").write_text(json.dumps(settings, indent=2, allow_nan=False) + "\n")
    return summaries

