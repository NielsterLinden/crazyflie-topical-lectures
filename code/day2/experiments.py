"""Offline E3-E5 experiments: matched inputs, diagnostics, figures and metrics."""

import argparse
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import chi2

from . import exercise_policies
from .experiment_data import (GRAVITY, SCENARIOS, SENSOR_NAMES, ExperimentData,
                              generate_data, load_prepared_data, sensor_available,
                              stationary_features)
from kalman.filter import FilterConfig, PositionVelocityKalmanFilter
from kalman.pipeline import SensorFusionPipeline
from kalman.recording import ACCELERATION_COLUMNS, POSITION_COLUMNS


@dataclass
class ExperimentCase:
    name: str
    config: FilterConfig
    scenario: str
    disabled: tuple[str, ...] = ()
    dropout_s: float = 0.0
    participant_gate: bool = False
    participant_stationary: bool = False


def validate_position_dropout(data: ExperimentData, case: ExperimentCase) -> None:
    """Ensure the intended intervention removes updates and shows recovery."""
    if case.dropout_s == 0:
        return
    if not np.isfinite(case.dropout_s) or case.dropout_s < 0:
        raise ValueError("dropout duration must be finite and nonnegative")
    start, end = 7.0, 7.0 + case.dropout_s
    recovery_end = end + 2.0
    fallback = ("Use a checked recording covering the interval (normally a 30 s capture), "
                "or explicitly select the synthetic fallback for E5.")
    if data.time[-1] < recovery_end - 1e-9:
        raise ValueError(f"Position-dropout comparison needs data through {recovery_end:g} s "
                         f"for loss and recovery. {fallback}")
    observed = np.all(np.isfinite(data.measured_position), axis=1)
    if data.has_truth:
        observed &= np.array([sensor_available("position", index, timestamp, case.disabled, 0)
                              for index, timestamp in enumerate(data.time)])
    else:
        observed[0] = False  # The first position initializes the recorded-data estimate.
        if "position" in case.disabled:
            observed[:] = False
    windows = (("before", data.time < start),
               ("during", (data.time >= start) & (data.time <= end)),
               ("after", (data.time > end) & (data.time <= recovery_end)))
    for name, window in windows:
        if np.count_nonzero(observed & window) < 2:
            raise ValueError(f"Position-dropout comparison needs at least two source position "
                             f"updates {name} the {start:g}-{end:g} s gap. {fallback}")


def run_experiment(data: ExperimentData, case: ExperimentCase, alpha: float = 0.01,
                   *, innovation_policy=None, stationary_policy=None) -> dict:
    validate_position_dropout(data, case)
    # Notebook cells can supply the same decisions as exercise_policies.py,
    # without changing module globals or the command-line experiment runner.
    if innovation_policy is None:
        innovation_policy = exercise_policies.innovation_threshold
    if stationary_policy is None:
        stationary_policy = exercise_policies.stationary_update_allowed
    estimator = PositionVelocityKalmanFilter(replace(case.config))
    # Declared starting-height prior, identical for every case. Its uncertainty
    # is still initial_position_std; subsequent states never use simulation truth.
    if data.has_truth:
        estimator.x[2] = 1.0
    else:
        # Use the first observed position once, as initialization, with its
        # assumed measurement variance. The loader starts analysis at the first
        # available observation, retaining the original recording clock.
        estimator.x[:3] = data.measured_position[0]
        estimator.p[:3, :3] = np.eye(3) * case.config.position_noise_std**2
    pipeline = SensorFusionPipeline(estimator)
    states, events, stationary_records, flow_samples = [], [], [], []

    def update(sensor, measurement, timestamp, callback):
        dimension = np.size(measurement)
        threshold = case.config.nis_gate
        if case.participant_gate:
            threshold = innovation_policy(dimension, alpha)
        if threshold is not None and (not np.isfinite(threshold) or threshold <= 0.0):
            raise ValueError("the innovation policy must return a positive finite threshold or None")
        estimator.config.nis_gate = threshold
        accepted = callback(measurement)
        # Copy now: the next asynchronous sensor update overwrites last_*.
        innovation = estimator.last_innovation.copy()
        covariance = estimator.last_innovation_covariance.copy()
        pull = np.linalg.solve(np.linalg.cholesky(covariance), innovation)
        events.append(dict(time_s=float(timestamp), sensor=sensor, dimension=dimension,
                           accepted=bool(accepted), threshold=threshold,
                           measurement=np.atleast_1d(measurement).copy(),
                           predicted_measurement=np.atleast_1d(measurement) - innovation,
                           innovation=innovation, covariance=covariance, pull=pull,
                           nis=float(estimator.last_nis)))

    for sample, timestamp in enumerate(data.time):
        pipeline.on_imu(float(timestamp), data.measured_acceleration[sample])
        observations = (
            ("position", data.measured_position[sample], pipeline.on_position),
            ("flow", data.measured_flow[sample], pipeline.on_flow_velocity),
            ("range", data.measured_range[sample], pipeline.on_range),
        )
        for sensor, measurement, callback in observations:
            if not data.has_truth:
                available = (sensor == "position" and sample > 0
                             and sensor not in case.disabled
                             and not (case.dropout_s > 0 and 7.0 <= timestamp <= 7.0 + case.dropout_s)
                             and np.all(np.isfinite(measurement)))
            else:
                available = sensor_available(sensor, sample, timestamp, case.disabled, case.dropout_s)
            if available:
                update(sensor, measurement, timestamp, callback)
                if sensor == "flow":
                    flow_samples.append((float(timestamp), measurement.copy()))
        if data.scenario == "stationary":
            window = stationary_features(data, sample, flow_samples)
            requested = case.participant_stationary and stationary_policy(window)
            stationary_records.append(dict(time_s=float(timestamp), **asdict(window),
                                           requested=bool(requested),
                                           truly_stationary=bool(np.linalg.norm(data.velocity[sample]) < 1e-9)))
            # Ten Hz pseudo-measurements avoid treating every IMU sample as a
            # new independent stationary constraint. Overlap still correlates them.
            if requested and sample % 5 == 0:
                update("zero_velocity", np.zeros(3), timestamp,
                       lambda measurement: pipeline.on_stationary())
        states.append(dict(time_s=float(timestamp), estimate=estimator.x.copy(),
                           variance=np.diag(estimator.p).copy(),
                           truth=np.r_[data.position[sample], data.velocity[sample]]))
    return dict(states=states, events=events, stationarity=stationary_records)


def interval_labels(data: ExperimentData, case: ExperimentCase) -> np.ndarray:
    times = data.time
    labels = np.full(len(times), "nominal", dtype="U20")
    labels[times < 2.0] = "startup"
    if case.dropout_s:
        labels[(times >= 7.0) & (times <= 7.0 + case.dropout_s)] = "dropout"
    if data.scenario == "outlier":
        outlier_time = 12.0
        if data.metadata:
            outlier_time = data.metadata["interventions"][0]["time_s"]
        labels[(times >= outlier_time) & (times < outlier_time + 0.5)] = "outlier_recovery"
    elif data.scenario == "range-step":
        labels[(times >= 8.0) & (times < 11.0)] = "range_step"
    elif data.scenario == "manoeuvre":
        labels[(times >= 8.0) & (times < 10.0)] = "manoeuvre"
    elif data.scenario == "stationary":
        labels[times < 4.0] = "initial_rest"
        labels[(times >= 4.0) & (times < 14.0)] = "moving"
        labels[times >= 14.0] = "final_rest"
    return labels


def lag_correlation(values: np.ndarray, lag: int, times: np.ndarray,
                    period_s: float) -> float | None:
    if len(values) <= lag + 1:
        return None
    # Exclude dropout gaps and joins between disjoint nominal intervals.
    regular = np.isclose(times[lag:] - times[:-lag], lag * period_s)
    before, after = values[:-lag][regular], values[lag:][regular]
    if len(before) < 2:
        return None
    if np.std(before) == 0.0 or np.std(after) == 0.0:
        return None
    return float(np.corrcoef(before, after)[0, 1])


def summarize(result: dict, data: ExperimentData, case: ExperimentCase, seed: int) -> dict:
    labels = interval_labels(data, case)
    estimate = np.array([row["estimate"] for row in result["states"]])
    truth = np.array([row["truth"] for row in result["states"]])
    sigma = np.sqrt(np.array([row["variance"] for row in result["states"]]))
    errors = estimate - truth
    intervals = []
    for label in dict.fromkeys(labels):
        selected = labels == label
        position_error = np.linalg.norm(errors[selected, :3], axis=1)
        intervals.append(dict(interval=label, samples=int(np.sum(selected)),
                              position_rmse_m=float(np.sqrt(np.mean(position_error**2))) if data.has_truth else None,
                              maximum_position_error_m=float(np.max(position_error)) if data.has_truth else None,
                              coverage95=np.mean(np.abs(errors[selected]) <= 1.96 * sigma[selected], axis=0).tolist()
                              if data.has_truth else [None] * 6))
    diagnostics = []
    # Log-time intervals can vary in recordings. Skip temporal correlation
    # claims there; the synthetic streams have known regular sensor periods.
    for sensor in (*SENSOR_NAMES, "zero_velocity"):
        for label in dict.fromkeys(labels):
            events = [event for event in result["events"]
                      if event["sensor"] == sensor
                      and labels[np.searchsorted(data.time, event["time_s"])] == label]
            if not events:
                continue
            pulls = np.array([event["pull"] for event in events])
            times = np.array([event["time_s"] for event in events])
            period = 0.04 if sensor == "flow" else 0.1
            diagnostics.append(dict(sensor=sensor, interval=label, dimension=events[0]["dimension"],
                                    updates=len(events), rejected=sum(not event["accepted"] for event in events),
                                    nis_mean=float(np.mean([event["nis"] for event in events])),
                                    pull_mean=np.mean(pulls, axis=0).tolist(),
                                    pull_std=np.std(pulls, axis=0, ddof=1).tolist() if len(pulls) > 1 else None,
                                    pull_lag1=[lag_correlation(pulls[:, axis], 1, times, period)
                                               for axis in range(pulls.shape[1])] if data.has_truth else None))
    false_stationary = sum(event["sensor"] == "zero_velocity" and event["accepted"]
                           and np.linalg.norm(data.velocity[int(round(event["time_s"] / 0.02))]) > 1e-9
                           for event in result["events"])
    return dict(case=case.name, seed=seed, has_truth=data.has_truth,
                intervals=intervals, diagnostics=diagnostics,
                false_stationary_updates=int(false_stationary) if data.has_truth else None)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_run(directory: Path, result: dict, data: ExperimentData) -> None:
    directory.mkdir(parents=True)
    states = []
    for index, state in enumerate(result["states"]):
        row = dict(time_s=state["time_s"])
        for component, name in enumerate(("px", "py", "pz", "vx", "vy", "vz")):
            row[name] = float(state["estimate"][component])
            row[f"truth_{name}"] = float(state["truth"][component]) if data.has_truth else ""
            row[f"sigma_{name}"] = float(np.sqrt(state["variance"][component]))
        for axis, name in enumerate("xyz"):
            row[f"acceleration_{name}_m_s2"] = float(data.measured_acceleration[index, axis])
        states.append(row)
    write_csv(directory / "states.csv", states)
    events = []
    for event in result["events"]:
        row = {key: event[key] for key in ("time_s", "sensor", "dimension", "accepted", "nis", "threshold")}
        for axis in range(3):
            for key in ("measurement", "predicted_measurement", "innovation", "pull"):
                row[f"{key}_{axis}"] = float(event[key][axis]) if axis < event["dimension"] else ""
            for column in range(3):
                row[f"S_{axis}{column}"] = (float(event["covariance"][axis, column])
                                            if max(axis, column) < event["dimension"] else "")
        events.append(row)
    write_csv(directory / "updates.csv", events)
    write_csv(directory / "stationarity.csv", result["stationarity"])


def plot_diagnostics(result: dict, destination: Path, alpha: float, regular_timing: bool = True) -> None:
    import matplotlib.pyplot as plt

    sensors = [sensor for sensor in (*SENSOR_NAMES, "zero_velocity")
               if any(event["sensor"] == sensor for event in result["events"])]
    if not sensors:
        return
    figure, axes = plt.subplots(3, len(sensors), figsize=(4 * len(sensors), 9), squeeze=False)
    for column, sensor in enumerate(sensors):
        events = [event for event in result["events"] if event["sensor"] == sensor]
        time = np.array([event["time_s"] for event in events])
        nis = np.array([event["nis"] for event in events])
        accepted = np.array([event["accepted"] for event in events])
        axes[0, column].plot(time, nis, ".", markersize=2, label="all pre-gate NIS")
        axes[0, column].plot(time[~accepted], nis[~accepted], "rx", label="rejected")
        axes[0, column].axhline(chi2.ppf(1 - alpha, events[0]["dimension"]), linestyle="--",
                               color="black", label="chi-square reference")
        axes[0, column].set(title=f"{sensor}, m={events[0]['dimension']}", xlabel="time [s]", ylabel="NIS")
        axes[0, column].legend(fontsize="small")
        pulls = np.array([event["pull"] for event in events])
        for axis in range(pulls.shape[1]):
            axes[1, column].hist(pulls[:, axis], bins=35, histtype="step", density=True, label=f"component {axis}")
            if regular_timing:
                period = 0.04 if sensor == "flow" else 0.1
                correlations = [lag_correlation(pulls[:, axis], lag, time, period) for lag in range(1, 11)]
                axes[2, column].plot(range(1, 11), [value if value is not None else np.nan for value in correlations])
            else:
                axes[2, column].plot(time, pulls[:, axis], ".", markersize=2)
        axes[1, column].set(xlabel="whitened innovation", ylabel="density (all updates)")
        axes[1, column].legend(fontsize="small")
        axes[2, column].set(xlabel="lag [sensor updates]" if regular_timing else "recording time [s]",
                            ylabel="pull correlation" if regular_timing else "whitened innovation")
    figure.suptitle("All intervals combined; use summary.json for interval-specific diagnostics")
    figure.tight_layout()
    figure.savefig(destination, dpi=120)
    plt.close(figure)


def plot_comparison(runs: list[tuple], destination: Path) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 2, figsize=(12, 10))
    colors = {}
    palette = plt.get_cmap("tab10")
    for case, seed, result, data in runs:
        estimate = np.array([row["estimate"] for row in result["states"]])
        truth = np.array([row["truth"] for row in result["states"]])
        sigma = np.sqrt(np.array([row["variance"] for row in result["states"]]))
        first_in_case = case.name not in colors
        if first_in_case:
            colors[case.name] = palette(len(colors) % 10)
        color = colors[case.name]
        label = case.name if first_in_case else "_nolegend_"
        position_values = estimate[:, :3] - truth[:, :3] if data.has_truth else estimate[:, :3]
        velocity_values = estimate[:, 3:] - truth[:, 3:] if data.has_truth else estimate[:, 3:]
        for axis in range(3):
            axes[axis, 0].plot(data.time, position_values[:, axis], color=color, alpha=0.5, label=label)
            centre_p = 0 if data.has_truth else estimate[:, axis]
            centre_v = 0 if data.has_truth else estimate[:, axis + 3]
            axes[axis, 0].plot(data.time, centre_p + 1.96 * sigma[:, axis], "--", color=color, alpha=0.15)
            axes[axis, 0].plot(data.time, centre_p - 1.96 * sigma[:, axis], "--", color=color, alpha=0.15)
            axes[axis, 1].plot(data.time, velocity_values[:, axis], color=color, alpha=0.5)
            axes[axis, 1].plot(data.time, centre_v + 1.96 * sigma[:, axis + 3], "--", color=color, alpha=0.15)
            axes[axis, 1].plot(data.time, centre_v - 1.96 * sigma[:, axis + 3], "--", color=color, alpha=0.15)
            if not data.has_truth and first_in_case:
                axes[axis, 0].plot(data.time, data.measured_position[:, axis], ".", color="0.5", markersize=2)
            axes[axis, 0].set_ylabel(f"{'xyz'[axis]} position {'error ' if data.has_truth else ''}[m]")
            axes[axis, 1].set_ylabel(f"{'xyz'[axis]} velocity {'error ' if data.has_truth else ''}[m/s]")
    axes[0, 0].legend(fontsize="x-small", ncols=2)
    for panel in axes[-1]:
        panel.set_xlabel("time [s]")
    has_truth = runs[0][3].has_truth
    figure.suptitle("All runs: estimate minus truth, dashed model 95% bands" if has_truth else
                   "Recorded observations and estimates; model bands, no independent truth")
    figure.tight_layout()
    figure.savefig(destination, dpi=120)
    plt.close(figure)


def plot_stationarity(result: dict, destination: Path) -> None:
    import matplotlib.pyplot as plt

    rows = result["stationarity"]
    if not rows:
        return
    time = [row["time_s"] for row in rows]
    figure, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(time, [row["acceleration_norm_mean_m_s2"] - GRAVITY for row in rows], label="mean norm minus g")
    axes[0].plot(time, [row["acceleration_norm_std_m_s2"] for row in rows], label="norm std")
    axes[0].set_ylabel("acceleration [m/s2]")
    axes[0].legend()
    axes[1].plot(time, [row["gyro_max_rad_s"] for row in rows])
    axes[1].set_ylabel("max gyro [rad/s]")
    axes[2].plot(time, [row["flow_speed_m_s"] if row["flow_speed_m_s"] is not None else np.nan for row in rows])
    axes[2].set_ylabel("flow speed [m/s]")
    axes[3].step(time, [row["truly_stationary"] for row in rows], label="simulation truth")
    axes[3].step(time, [row["requested"] for row in rows], label="policy requests ZUPT")
    axes[3].set(xlabel="time [s]", ylabel="stationary decision")
    axes[3].legend()
    figure.tight_layout()
    figure.savefig(destination, dpi=120)
    plt.close(figure)


def make_cases(args) -> list[ExperimentCase]:
    config = FilterConfig(acceleration_noise_std=args.acceleration_std,
                          position_noise_std=0.04 * np.sqrt(args.r_scale),
                          flow_velocity_noise_std=0.08 * np.sqrt(args.r_scale),
                          range_noise_std=0.03 * np.sqrt(args.r_scale))
    scenario = "stationary" if args.study == "stationarity" else args.scenario
    dropout = args.dropout_seconds if scenario == "dropout" else 0.0
    base = ExperimentCase("baseline", config, scenario, tuple(args.disable), dropout)
    if args.study == "process":
        baseline_config = replace(config, acceleration_noise_std=FilterConfig().acceleration_noise_std)
        return [replace(base, config=baseline_config), replace(base, name="Q-candidate")]
    if args.study == "covariance":
        cases = [base]
        for scale in (0.1, 10.0):
            changed = replace(config, position_noise_std=config.position_noise_std * np.sqrt(scale),
                              flow_velocity_noise_std=config.flow_velocity_noise_std * np.sqrt(scale),
                              range_noise_std=config.range_noise_std * np.sqrt(scale))
            cases.append(replace(base, name=f"R-times-{scale:g}", config=changed))
        return cases
    if args.study == "gating":
        return [base, replace(base, name="participant-gate", participant_gate=True)]
    if args.study == "stationarity":
        return [base, replace(base, name="participant-zupt", participant_stationary=True)]
    if args.study == "dropout":
        base = replace(base, scenario="nominal", disabled=(), dropout_s=0.0)
        if args.recording is not None:
            return [base, replace(base, name="dropout-3s", dropout_s=3.0),
                    replace(base, name="dropout-6s", dropout_s=6.0),
                    replace(base, name="without-position", disabled=("position",))]
        return [base, replace(base, name="dropout-3s", dropout_s=3.0),
                replace(base, name="dropout-6s", dropout_s=6.0),
                *(replace(base, name=f"without-{sensor}", disabled=(sensor,)) for sensor in SENSOR_NAMES),
                replace(base, name="without-absolute", disabled=("position", "range"))]
    return [base]


def save_metrics(output: Path, summaries: list[dict]) -> None:
    rows = []
    for summary in summaries:
        for interval in summary["intervals"]:
            row = dict(case=summary["case"], seed=summary["seed"], has_truth=summary["has_truth"], interval=interval["interval"],
                       samples=interval["samples"], position_rmse_m=interval["position_rmse_m"],
                       maximum_position_error_m=interval["maximum_position_error_m"])
            for component, name in enumerate(("px", "py", "pz", "vx", "vy", "vz")):
                row[f"coverage95_{name}"] = interval["coverage95"][component]
            rows.append(row)
    write_csv(output / "metrics.csv", rows)
    aggregate = []
    keys = dict.fromkeys((row["case"], row["interval"]) for row in rows)
    for case, interval in keys:
        selected = [row for row in rows if row["case"] == case and row["interval"] == interval]
        entry = dict(case=case, interval=interval, runs=len(selected))
        for metric in ("position_rmse_m", "maximum_position_error_m", "coverage95_px", "coverage95_py", "coverage95_pz"):
            values = [row[metric] for row in selected if row[metric] is not None]
            entry[metric] = dict(mean=float(np.mean(values)) if values else None,
                                 std=float(np.std(values, ddof=1)) if len(values) > 1 else None)
        aggregate.append(entry)
    (output / "aggregate.json").write_text(json.dumps(aggregate, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def export_prepared(data: ExperimentData, case: ExperimentCase, output: Path, seed: int) -> None:
    """Create a declared synthetic fallback in the existing E6 replay format."""
    with (output / "prepared.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *ACCELERATION_COLUMNS, *POSITION_COLUMNS])
        for sample, timestamp in enumerate(data.time):
            position = data.measured_position[sample] if sensor_available("position", sample, timestamp, case.disabled, case.dropout_s) else ["", "", ""]
            writer.writerow([timestamp, *data.measured_acceleration[sample], *position])
    metadata = dict(dataset_id=f"synthetic-{case.scenario}-seed-{seed}", synthetic=True,
                    provenance="Controlled synthetic fallback. Not a real drone recording, calibration or hardware validation.",
                    prepared_replay=dict(acceleration_frame="world", gravity_removed=True,
                                         acceleration_interval="preceding", position_source="external"),
                    position_source_note="Synthetic noisy position, distinct from simulated truth; external is the replay format label.",
                    timestamp_kind="synthetic seconds", scenario=case.scenario,
                    calibration="not applicable", seed=seed,
                    source_sha256={"prepared.csv": hashlib.sha256((output / "prepared.csv").read_bytes()).hexdigest()})
    (output / "recording.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def check_policies(exercise: str = "all") -> list[str]:
    """Executable targets for the two small participant edits; no experiment I/O."""
    failures = []
    if exercise in ("E4", "all"):
        for dimension in (1, 2, 3):
            actual = exercise_policies.innovation_threshold(dimension, 0.01)
            if actual is None or not np.isclose(actual, chi2.ppf(0.99, dimension)):
                failures.append(f"E4: check the 1% false-rejection threshold for dimension {dimension}")
    if exercise == "E4":
        return failures
    rest = exercise_policies.StationaryWindow(1.0, 0.003, GRAVITY, 0.02, True, 0.005)
    examples = [("rest", rest, True),
                ("constant-velocity motion", replace(rest, flow_speed_m_s=0.3), False),
                ("missing motion cue", replace(rest, flow_speed_m_s=None), False),
                ("running motors", replace(rest, motors_idle=False), False),
                ("rotation", replace(rest, gyro_max_rad_s=1.0), False),
                ("acceleration", replace(rest, acceleration_norm_mean_m_s2=12.0), False),
                ("short window", replace(rest, duration_s=0.0), False)]
    for name, window, expected in examples:
        if bool(exercise_policies.stationary_update_allowed(window)) != expected:
            failures.append(f"E5: stationary decision for {name} should be {expected}")
    return failures


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=("single", "covariance", "process", "gating", "dropout", "stationarity"), default="single")
    parser.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--recording", type=Path, help="prepared.csv from a live capture or checked recording")
    parser.add_argument("--metadata", type=Path, help="companion recording.json")
    parser.add_argument("--export-prepared", action="store_true", help="also save a declared synthetic E6 fallback")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=1)
    parser.add_argument("--acceleration-std", type=float, default=1.5)
    parser.add_argument("--r-scale", type=float, default=1.0, help="multiply assumed measurement variances, not injected noise")
    parser.add_argument("--dropout-seconds", type=float, default=3.0)
    parser.add_argument("--disable", choices=SENSOR_NAMES, nargs="*", default=[])
    parser.add_argument("--alpha", type=float, default=0.01)
    parser.add_argument("--check-policies", choices=("E4", "E5", "all"), nargs="?", const="all")
    args = parser.parse_args(argv)
    if args.check_policies:
        failures = check_policies(args.check_policies)
        print("\n".join(failures) if failures else "E4/E5 policy checks passed (synthetic checks, not flight qualification).")
        return int(bool(failures))
    if args.output is None:
        parser.error("--output is required for experiments")
    if (args.recording is None) != (args.metadata is None):
        parser.error("--recording and --metadata must be supplied together")
    if args.recording is not None and args.seed_count != 1:
        parser.error("one recording is one experiment, not independent seed trials; use synthetic data for ensembles")
    if args.recording is not None and (args.study == "stationarity" or args.scenario in ("stationary", "range-step", "manoeuvre")):
        parser.error("the live helper supplies only position/acceleration; use the synthetic fallback for gyro/flow/range scenarios")
    if args.recording is not None and args.export_prepared:
        parser.error("--export-prepared creates synthetic fallback data; omit --recording")
    if args.seed < 0 or args.seed_count < 1:
        parser.error("seed must be nonnegative and seed-count positive")
    if not np.isfinite(args.acceleration_std) or args.acceleration_std <= 0.0:
        parser.error("acceleration-std must be finite and positive")
    if not np.isfinite(args.r_scale) or args.r_scale <= 0.0:
        parser.error("r-scale must be finite and positive")
    if not np.isfinite(args.dropout_seconds) or not 0.0 <= args.dropout_seconds <= 11.0:
        parser.error("dropout-seconds must be between 0 and 11")
    if not 0.0 < args.alpha < 1.0:
        parser.error("alpha must be between 0 and 1")
    cases = make_cases(args)
    if any(case.participant_gate for case in cases) and exercise_policies.innovation_threshold(3, args.alpha) is None:
        print("E4 starter returns None: participant gating is currently disabled. Edit exercise_policies.py.")
    if args.study == "stationarity":
        print("E5 calls your stationary policy; the supplied starter declines all updates.")
    try:
        datasets_from_recording = {}
        if args.recording is not None:
            for scenario in dict.fromkeys(case.scenario for case in cases):
                datasets_from_recording[scenario] = load_prepared_data(args.recording, args.metadata, scenario)
            for case in cases:
                validate_position_dropout(datasets_from_recording[case.scenario], case)
        args.output.mkdir(parents=True, exist_ok=False)
        summaries, runs = [], []
        export_data = None
        for seed in range(args.seed, args.seed + args.seed_count):
            datasets = (datasets_from_recording if args.recording is not None else
                        {scenario: generate_data(seed, scenario) for scenario in dict.fromkeys(case.scenario for case in cases)})
            for case in cases:
                data = datasets[case.scenario]
                if export_data is None:
                    export_data = data
                result = run_experiment(data, case, args.alpha)
                directory = args.output / case.name / (f"seed-{seed:03d}" if data.has_truth else "recording")
                save_run(directory, result, data)
                if seed == args.seed:
                    plot_diagnostics(result, directory / "diagnostics.png", args.alpha, regular_timing=data.has_truth)
                    plot_stationarity(result, directory / "stationarity.png")
                summary = summarize(result, data, case, seed if data.has_truth else None)
                summaries.append(summary)
                nominal = next((row for row in summary["intervals"] if row["interval"] == "nominal"), None)
                if nominal is not None and data.has_truth:
                    print(f"{case.name}, seed {seed}: nominal position RMSE {nominal['position_rmse_m']:.4f} m")
                if data.scenario == "stationary":
                    print(f"{case.name}, seed {seed}: accepted ZUPTs during motion = {summary['false_stationary_updates']}")
                runs.append((case, seed, result, data))
        (args.output / "summary.json").write_text(json.dumps(summaries, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        save_metrics(args.output, summaries)
        policy_source = Path(exercise_policies.__file__).read_text(encoding="utf-8")
        (args.output / "policies.py.txt").write_text(policy_source, encoding="utf-8")
        code_root = Path(__file__).resolve().parents[1]
        source_paths = [Path(__file__), Path(__file__).with_name("experiment_data.py"),
                        code_root / "kalman/filter.py", code_root / "kalman/pipeline.py",
                        code_root / "kalman/recording.py"]
        settings = dict(study=args.study, seeds=list(range(args.seed, args.seed + args.seed_count)) if args.recording is None else [],
                        alpha=args.alpha, cases=[asdict(case) for case in cases],
                        sensor_rates_hz=dict(imu=50, position=10, flow=25, range=10),
                        actual_noise_std=dict(acceleration=0.08, position=0.04, flow=0.08, range=0.03),
                        initial_mean=[0, 0, 1, 0, 0, 0],
                        stationary_pseudo_measurement_std_m_s=0.02,
                        policy_sha256=hashlib.sha256(policy_source.encode("utf-8")).hexdigest(),
                        source_sha256={str(path.relative_to(code_root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                       for path in source_paths},
                        notes="Offline synthetic data; level, nonrotating IMU. Stationary case adds 0.03 m/s^2 X bias; manoeuvre case delays IMU input by 0.2 s.")
        if args.recording is not None:
            settings.update(source="prepared recording; no independent truth", recording_metadata=data.metadata,
                            sensor_rates_hz=None, actual_noise_std=None,
                            initial_mean="first observed position, zero velocity; first position used once for initialization",
                            notes="Cascaded when onboard position/attitude share IMU inputs. Position-only updates; no accuracy or coverage claims.")
        if args.export_prepared:
            export_prepared(export_data, cases[0], args.output, args.seed)
        (args.output / "settings.json").write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
        plot_comparison(runs, args.output / "comparison.png")
        # Pair plots remain readable when an ablated sensor makes one state's
        # uncertainty much larger than all the other cases in the overview.
        for case in cases[1:]:
            selected = [run for run in runs if run[0].name in (cases[0].name, case.name)]
            plot_comparison(selected, args.output / case.name / "comparison.png")
    except (OSError, ValueError, np.linalg.LinAlgError) as exc:
        parser.error(str(exc))
    source = "matched noise draws" if args.recording is None else "identical recorded samples; RMSE/coverage unavailable"
    print(f"Saved {len(runs)} runs to {args.output}; comparisons use {source}.")
    print("Inspect comparison.png, per-sensor diagnostics.png, updates.csv and summary.json. Offline only.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
