"""Save the existing experiment diagnostics from short, editable notebook cells."""

from dataclasses import asdict
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

from .experiment_data import generate_data, load_prepared_data
from .experiments import (plot_comparison, plot_diagnostics, plot_stationarity,
                          run_experiment, save_metrics, save_run, summarize,
                          validate_position_dropout)


def _run_capture(command, *, cwd, env, timeout_s):
    """Relay progress into Jupyter while bounding the child process lifetime."""
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


def capture(output, uri, *, duration_s=60, ready_for_motor_stop=False,
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
            innovation_policy=None, stationary_policy=None, notes=""):
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
                                innovation_policy=innovation_policy,
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
    for name, policy in (("innovation_policy", innovation_policy), ("stationary_policy", stationary_policy)):
        if policy is not None:
            try:
                settings[name] = inspect.getsource(policy)
            except (OSError, TypeError):
                settings[name] = "Defined in notebook; save the edited notebook alongside these results."
    (output / "settings.json").write_text(json.dumps(settings, indent=2, allow_nan=False) + "\n")
    return summaries
