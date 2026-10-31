"""Notebook capture and experiment helpers."""

from contextlib import contextmanager
from dataclasses import asdict
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Sequence

import numpy as np

from .experiment_data import generate_data, load_prepared_data
from .experiments import (plot_comparison, plot_diagnostics, plot_stationarity,
                          run_experiment, save_metrics, save_run, summarize,
                          validate_position_dropout)
from race_common.motor_stop import engage_motor_stop


@contextmanager
def _capture_receipt(destination, details):
    """Only mark a capture complete after logging and cleanup have succeeded."""
    receipt = Path(str(destination) + ".capture.json")
    if destination.exists() or receipt.exists():
        raise FileExistsError(f"Choose a new capture filename: {destination}")
    details = dict(details, status="recording", rows=0)
    with receipt.open("x", encoding="utf-8") as stream:
        json.dump(details, stream, indent=2)
    try:
        yield details
    except BaseException as error:
        details.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                       reason=str(error))
        raise
    finally:
        if destination.is_file():
            details["sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
        receipt.write_text(json.dumps(details, indent=2) + "\n", encoding="utf-8")


def completed_capture(path):
    """Auto-select only a complete, unchanged recording made by capture()."""
    path = Path(path)
    receipt = Path(str(path) + ".capture.json")
    if not path.is_file() or not receipt.is_file():
        return None
    details = json.loads(receipt.read_text(encoding="utf-8"))
    if details.get("status") != "complete" or details.get("rows", 0) < 2:
        return None
    if details.get("sha256") != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError(f"Capture changed after completion: {path}")
    return path


def read_imu(path):
    """Read a checked cfclient or Python IMU CSV, retaining body units."""
    columns = tuple(f"{sensor}.{axis}" for sensor in ("acc", "gyro") for axis in "xyz")
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.DictReader(stream)
        values = np.array([[float(row[name]) for name in columns] for row in rows])
    if values.ndim != 2 or values.shape[0] < 2 or not np.all(np.isfinite(values)):
        raise ValueError("Need at least two finite six-channel IMU samples.")
    return values


def simulated_imu_pair(directory):
    """Create two independent stationary examples; never represent them as drone data."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = []
    columns = [f"{sensor}.{axis}" for sensor in ("acc", "gyro") for axis in "xyz"]
    for name, seed in (("calibration", 42), ("validation", 43)):
        path = directory / f"{name}.csv"
        if path.exists():
            if completed_capture(path) is None:
                raise ValueError(f"Incomplete simulated example: {path}; choose a fresh output directory.")
            details = json.loads(Path(str(path) + ".capture.json").read_text())
            if details.get("source") != "simulation" or details.get("seed") != seed:
                raise ValueError(f"Expected a labelled simulated example: {path}")
        else:
            rng = np.random.default_rng(seed)
            # Fixed sensor offsets; independent noise in the held-out recording.
            values = rng.normal([.01, -.02, 1., .3, -.2, .5],
                                [.002, .002, .002, .08, .08, .08], size=(3000, 6))
            with _capture_receipt(path, {"source": "simulation", "seed": seed}) as receipt:
                with path.open("x", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                    writer.writerows(values)
                receipt.update(status="complete", rows=len(values))
        paths.append(path)
    return tuple(paths)


def plot_imu_calibration(values, names, standardized_calibration, standardized_validation):
    """Plot already calculated quantities; keep the calibration math in the notebook."""
    import matplotlib.pyplot as plt

    for view in ("samples", "standardized comparison"):
        figure, axes = plt.subplots(3, 2, figsize=(12, 8))
        for index, (axis, name) in enumerate(zip(axes.flat, names)):
            unit = "g" if name.startswith("acc.") else "deg/s"
            if view == "samples":
                axis.plot(values[:, index], linewidth=0.7)
                axis.set(xlabel="sample index", ylabel=unit, title=name)
            else:
                calibration = standardized_calibration[:, index]
                if np.all(np.isnan(calibration)):
                    axis.text(.5, .5, "Zero calibration RMS; normalization undefined",
                              transform=axis.transAxes, ha="center", va="center", wrap=True)
                else:
                    validation = (None if standardized_validation is None
                                  else standardized_validation[:, index])
                    combined = calibration if validation is None else np.concatenate((calibration, validation))
                    bins = np.histogram_bin_edges(combined, bins=35)
                    axis.hist(calibration, bins=bins, density=True, histtype="step", linewidth=1.5,
                              label="Calibration: self-standardized")
                    if validation is not None:
                        axis.hist(validation, bins=bins, density=True, histtype="step", linewidth=1.5,
                                  label="Held-out: frozen calibration")
                    reference = np.linspace(min(-4, combined.min()), max(4, combined.max()), 300)
                    axis.plot(reference, np.exp(-.5 * reference**2) / np.sqrt(2 * np.pi),
                              "k--", label="N(0, 1) reference")
                    axis.legend(fontsize="small")
                axis.set(xlabel="(sample - calibration mean) / calibration RMS", ylabel="density", title=name)
            axis.grid(True, alpha=.3)
        figure.suptitle(f"Stationary IMU calibration: {view}")
        figure.tight_layout()
        plt.show()
        plt.close(figure)


def display_table(rows):
    """Render a small comparison without adding a dataframe dependency."""
    from html import escape
    from IPython.display import HTML, display

    if not rows:
        print("No measurements available for this comparison.")
        return
    columns = list(rows[0])
    header = "".join(f"<th>{escape(str(name))}</th>" for name in columns)
    body = "".join("<tr>" + "".join(f"<td>{escape(str(row.get(name)))}</td>"
                                    for name in columns) + "</tr>" for row in rows)
    display(HTML(f"<table><thead><tr>{header}</tr></thead><tbody>{body}</tbody></table>"))


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


@contextmanager
def motor_stopped_connection(uri: str, cache_directory: Path):
    """Connect to a Crazyflie and verify its firmware motor stop before yielding."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    with SyncCrazyflie(
        uri, cf=Crazyflie(rw_cache=str(cache_directory))
    ) as connection:
        connection.wait_for_params()
        cf = connection.cf
        stop_check = engage_motor_stop(cf)
        yield cf, stop_check


def add_log_config(cf, name: str, channels: Sequence[str], period_ms: int,
                   data_callback, error_callback):
    """Validate TOC channels and register a typed log block with callbacks."""
    from cflib.crazyflie.log import LogConfig

    elements = log_channel_elements(cf, channels)
    if not isinstance(period_ms, int) or isinstance(period_ms, bool) or period_ms <= 0:
        raise ValueError("Logging period must be a positive integer number of milliseconds.")
    config = LogConfig(name=name, period_in_ms=period_ms)
    for channel, element in elements.items():
        config.add_variable(channel, element.ctype)
    config.data_received_cb.add_callback(data_callback)
    config.error_cb.add_callback(error_callback)
    cf.log.add_config(config)
    return config


def log_channel_elements(cf, channels: Sequence[str]):
    """Return TOC elements for unique channel names or report all missing names."""
    if not channels or isinstance(channels, str) or len(set(channels)) != len(channels):
        raise ValueError("Provide a non-empty list of unique log channel names.")
    elements = {
        channel: cf.log.toc.get_element_by_complete_name(channel)
        for channel in channels
    }
    missing = [channel for channel, element in elements.items() if element is None]
    if missing:
        raise ValueError("Firmware lacks required log channels: " + ", ".join(missing))
    return elements


def capture(uri, output, channels, *, duration_s=60, period_ms=20):
    """Save selected raw Crazyflie log channels as a timestamped CSV."""
    if not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Provide the assigned Crazyradio URI.")
    if not channels or isinstance(channels, str) or len(set(channels)) != len(channels):
        raise ValueError("Provide a non-empty list of unique log channel names.")
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("Capture duration must be finite and positive.")
    if not isinstance(period_ms, int) or isinstance(period_ms, bool) or period_ms <= 0:
        raise ValueError("Logging period must be a positive integer number of milliseconds.")

    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    changed = threading.Event()
    lock = threading.Lock()
    state = {"first_sample": None, "last_sample": None, "rows": 0, "error": None}

    details = dict(uri=uri, channels=list(channels), duration_s=duration_s, period_ms=period_ms)
    with _capture_receipt(destination, details) as receipt, \
            destination.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("firmware_timestamp_ms", *channels))
        stream.flush()

        def receive(timestamp, data, _config):
            with lock:
                if state["error"] is not None:
                    return
                try:
                    values = [float(data[channel]) for channel in channels]
                    if not all(math.isfinite(value) for value in values):
                        raise ValueError("log packet contains a non-finite value")
                    now = time.monotonic()
                    if state["first_sample"] is None:
                        state["first_sample"] = now
                    writer.writerow((int(timestamp), *values))
                    stream.flush()
                    state["last_sample"] = now
                    state["rows"] += 1
                    receipt["rows"] = state["rows"]
                except (KeyError, TypeError, ValueError, OverflowError) as error:
                    state["error"] = f"Invalid telemetry: {error}"
                changed.set()

        def log_error(_config, message):
            with lock:
                state["error"] = f"Logging failed: {message}"
                changed.set()

        cache_directory = destination.parent / ".cflib-cache"
        with motor_stopped_connection(uri, cache_directory) as (cf, _stop_check):
            receipt["motor_stop_check"] = _stop_check
            print("STOP VERIFIED: firmware lock set and all four motor outputs zero.")
            cf.connection_lost.add_callback(
                lambda _uri, message: log_error(None, f"connection lost: {message}")
            )
            config = add_log_config(
                cf, "Day2_capture", channels, period_ms, receive, log_error
            )
            started = False
            try:
                config.start()
                started = True
                wait_started = time.monotonic()
                while True:
                    now = time.monotonic()
                    with lock:
                        error = state["error"]
                        first_sample = state["first_sample"]
                        last_sample = state["last_sample"]
                        row_count = state["rows"]
                    if error is not None:
                        raise RuntimeError(error)
                    if first_sample is None:
                        if now - wait_started > 10:
                            raise TimeoutError("No telemetry received within 10 seconds.")
                    else:
                        if last_sample is not None and now - last_sample > 1:
                            raise TimeoutError(
                                "Telemetry stopped; partial CSV was retained."
                            )
                        if now - first_sample >= duration_s:
                            if row_count < 2:
                                raise RuntimeError(
                                    "Fewer than two samples were recorded."
                                )
                            break
                    changed.wait(0.05)
                    changed.clear()
            finally:
                try:
                    if started:
                        config.stop()
                finally:
                    config.delete()
                    stream.flush()

        receipt["status"] = "complete"

    return destination, state["rows"]


def _run_capture(command, *, cwd, env, timeout_s):
    """Relay progress to the terminal or notebook; bound the child lifetime."""
    lines = []
    messages = queue.Queue()
    with subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, bufsize=1) as process:
        def read_output():
            for line in process.stdout:
                messages.put(line)

        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        deadline = time.monotonic() + timeout_s
        try:
            while process.poll() is None or reader.is_alive() or not messages.empty():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(command, timeout_s)
                try:
                    line = messages.get(timeout=min(0.1, remaining))
                except queue.Empty:
                    continue
                lines.append(line)
                print(line, end="", flush=True)
            return subprocess.CompletedProcess(command, process.returncode, "".join(lines), "")
        except BaseException:
            process.kill()
            process.wait()
            raise
        finally:
            reader.join(timeout=1)


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
        raise ValueError("Set the team's assigned URI explicitly or through CF_URI.")
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


def capture_vertical(output, uri, *, ready_for_motor_stop=False, duration_s=20):
    """Acquire sensor-level height data in a bounded child; leave stop engaged."""
    if not ready_for_motor_stop or not isinstance(uri, str) or not uri.strip():
        raise ValueError("Set the assigned URI and confirm a stable surface, hands clear and other radio clients closed.")
    if not math.isfinite(duration_s) or not 18 <= duration_s <= 30:
        raise ValueError("Vertical capture duration must be between 18 and 30 seconds.")
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Choose a new capture directory: {output}")
    # This notebook already owns the timeout; avoid a nested supervisor/worker.
    command = [sys.executable, '-u', '-m', 'day2.vertical_live', '--uri', uri.strip(),
               '--output', str(output), '--duration', str(duration_s),
               '--prepare-for-handling', '--capture-worker']
    print("Keep the drone on the surface until STOP VERIFIED, then follow the five-second rest check and motion prompts.", flush=True)
    try:
        result = _run_capture(command, cwd=Path(__file__).resolve().parents[1],
                              env=dict(os.environ, MPLBACKEND='Agg'), timeout_s=duration_s + 30)
    except subprocess.TimeoutExpired as error:
        raise TimeoutError("Capture exceeded its bound; select a checked sensor recording or simulation.") from error
    if result.returncode:
        raise RuntimeError(f"Capture failed; inspect {output}, then explicitly select a fallback.\n{result.stdout}")
    from .vertical import select
    select(output)  # Validate before returning a usable selection to the notebook.
    return output


def next_attempt(parent):
    """Choose a new attempt path; the experiment writer creates that directory.

    Notebook cells call this on every execution, so changing one assumption does
    not require a new capture or a kernel restart. Writers still refuse to
    overwrite an existing directory, including partially failed experiments.
    """
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    number = 1
    while (parent / f"attempt-{number:03d}").exists():
        number += 1
    result = parent / f"attempt-{number:03d}"
    print("Saving this attempt to:", result)
    return result


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
