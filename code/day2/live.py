"""Motor-free E3-E6 capture: compare live estimates and save reusable inputs."""

import argparse
import csv
import heapq
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import time

import numpy as np
from scipy.spatial.transform import Rotation

from kalman.filter import FilterConfig
from kalman.recording import ACCELERATION_COLUMNS, POSITION_COLUMNS, ReplayFilter
from .replay import save_results
from race_common.motor_stop import engage_motor_stop


GRAVITY = 9.80665
PERIOD_MS = 50
REORDER_S = 0.15
MAX_AGE_S = 0.15
MAX_GAP_S = 0.30
CHANNELS = {
    "motors": [f"motor.m{index}" for index in range(1, 5)],
    "attitude": [f"stateEstimate.q{axis}" for axis in "xyzw"],
    "imu_position": [f"acc.{axis}" for axis in "xyz"] + [f"stateEstimate.{axis}" for axis in "xyz"],
}
PRIORITY = {"motors": 0, "attitude": 1, "imu_position": 2}


def world_acceleration(acceleration_g, quaternion_xyzw):
    """Body specific force in g -> world translational acceleration in m/s2."""
    force = np.asarray(acceleration_g, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if force.shape != (3,) or quaternion.shape != (4,) or not np.all(np.isfinite([*force, *quaternion])):
        raise ValueError("invalid acceleration or quaternion")
    if not 0.95 <= np.linalg.norm(quaternion) <= 1.05:
        raise ValueError("onboard quaternion is not approximately unit length")
    return Rotation.from_quat(quaternion).apply(force * GRAVITY) + [0, 0, -GRAVITY]


class SampleAssembler:
    """Causal, bounded-age pairing after the acquisition reorder buffer.

    Log timestamps are 24-bit firmware milliseconds, not exact sensor times.
    Attitude and motor snapshots are never taken from the future or reused as
    position updates. Position and acceleration share one 24-byte log packet.
    """

    def __init__(self, acceleration_bias_g=(0, 0, 0)):
        self.bias = np.asarray(acceleration_bias_g, dtype=float)
        if self.bias.shape != (3,) or not np.all(np.isfinite(self.bias)):
            raise ValueError("accelerometer bias must contain three finite g values")
        self.high_water_ms = None
        self.last_event_ms = {}
        self.attitude = None
        self.motors = None
        self.previous_sample_ms = None
        self.origin_ms = None
        self.skipped = 0

    def unwrap(self, timestamp):
        modulus = 1 << 24
        raw = int(timestamp) % modulus
        if self.high_water_ms is None:
            self.high_water_ms = raw
        delta = (raw - self.high_water_ms % modulus + modulus // 2) % modulus - modulus // 2
        unwrapped = self.high_water_ms + delta
        self.high_water_ms = max(self.high_water_ms, unwrapped)
        return unwrapped

    def accept(self, stream, timestamp_ms, data):
        values = np.asarray([data[name] for name in CHANNELS[stream]], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"non-finite {stream} telemetry")
        if timestamp_ms <= self.last_event_ms.get(stream, -float("inf")):
            self.skipped += 1
            return None
        self.last_event_ms[stream] = timestamp_ms
        if stream == "motors":
            if np.any(values != 0):
                raise ValueError("motor outputs are nonzero despite firmware stop; leave the drone on a safe surface and stop the experiment")
            self.motors = timestamp_ms
            return None
        if stream == "attitude":
            if not 0.95 <= np.linalg.norm(values) <= 1.05:
                raise ValueError("invalid onboard quaternion")
            self.attitude = (timestamp_ms, values)
            return None
        dependencies = [self.motors, self.attitude[0] if self.attitude is not None else None]
        if any(stamp is None for stamp in dependencies):
            self.skipped += 1
            return None
        if any(not 0 <= (timestamp_ms - stamp) / 1000 <= MAX_AGE_S for stamp in dependencies):
            if self.previous_sample_ms is not None:
                raise ValueError("attitude or motor telemetry is stale; recording stopped")
            self.skipped += 1
            return None
        if self.previous_sample_ms is not None:
            gap = (timestamp_ms - self.previous_sample_ms) / 1000
            if gap <= 0:
                self.skipped += 1
                return None
            if gap > MAX_GAP_S:
                raise ValueError("telemetry gap too long for held-input propagation; use the saved recording or fallback")
        acceleration = world_acceleration(values[:3] - self.bias, self.attitude[1])
        if self.origin_ms is None:
            self.origin_ms = timestamp_ms
        self.previous_sample_ms = timestamp_ms
        return (timestamp_ms - self.origin_ms) / 1000, acceleration, values[3:]


class LiveSession:
    """Journal input before filtering so interrupted runs retain their data."""

    def __init__(self, output, configs, bias_g, uri):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.configs = configs
        self.filters = [ReplayFilter(config) for config in configs]
        self.results = [[], []]
        self.assembler = SampleAssembler(bias_g)
        self.pending = []
        self.sequence = 0
        self.committed_ms = -float("inf")
        self.latest_sample = None
        self.raw_stream = (self.output / "raw.csv").open("x", newline="", encoding="utf-8")
        self.raw_writer = csv.writer(self.raw_stream)
        self.raw_writer.writerow(["host_monotonic_s", "firmware_timestamp_ms", "stream", "values_json"])
        self.prepared_stream = (self.output / "prepared.csv").open("x", newline="", encoding="utf-8")
        self.prepared_writer = csv.writer(self.prepared_stream)
        self.prepared_writer.writerow(["time_s", *ACCELERATION_COLUMNS, *POSITION_COLUMNS])
        self.metadata = {
            "source": "motor-free live Crazyflie logs", "uri": uri,
            "prepared_replay": {"acceleration_frame": "world", "gravity_removed": True,
                                "acceleration_interval": "preceding", "position_source": "onboard"},
            "channels": CHANNELS, "log_period_ms": PERIOD_MS,
            "accelerometer_bias_subtracted_g": list(bias_g),
            "gravity_m_s2": GRAVITY,
            "conversion": "R_body_to_world(q_xyzw) @ ((acc_g - bias_g) * gravity) + [0,0,-gravity]",
            "timing": "Unwrapped firmware log milliseconds; latest causal attitude/motors within 150 ms; 150 ms arrival reorder buffer. Current acceleration approximates the preceding interval. No sensor-delay compensation.",
            "limitations": "Cascaded estimation: onboard position/attitude share IMU information. Not raw Lighthouse fusion, independent truth or flight qualification. Existing onboard calibration retained; optional bias is an additional justified correction only.",
            "status": "recording", "interval_notes": "Add observed rest/motion intervals here after capture.",
        }
        self.write_metadata()

    def write_metadata(self):
        self.metadata["skipped_events"] = self.assembler.skipped
        self.metadata["prepared_samples"] = len(self.results[0])
        (self.output / "recording.json").write_text(json.dumps(self.metadata, indent=2) + "\n", encoding="utf-8")

    def enqueue(self, stream, timestamp, data, arrival):
        self.raw_writer.writerow([arrival, timestamp, stream, json.dumps(data, sort_keys=True)])
        self.raw_stream.flush()
        # Do not defer a motor-active warning behind the reorder window.
        if stream == "motors" and any(float(data[name]) != 0 for name in CHANNELS[stream]):
            raise ValueError("motor outputs are nonzero; this helper is for motors-disabled hand motion only")
        stamp = self.assembler.unwrap(timestamp)
        if stamp < self.committed_ms:
            self.assembler.skipped += 1
            return
        heapq.heappush(self.pending, (stamp, PRIORITY[stream], self.sequence, arrival, stream, data))
        self.sequence += 1

    def drain(self, now, force=False):
        while self.pending and (force or now - self.pending[0][3] >= REORDER_S):
            stamp, _, _, _, stream, data = heapq.heappop(self.pending)
            self.committed_ms = stamp
            sample = self.assembler.accept(stream, stamp, data)
            if sample is None:
                continue
            self.latest_sample = sample
            t, acceleration, position = sample
            self.prepared_writer.writerow([t, *acceleration, *position])
            self.prepared_stream.flush()
            for processor, results in zip(self.filters, self.results):
                results.append(processor.step(*sample))

    def finish(self, status, reason=""):
        self.metadata.update(status=status, stop_reason=reason)
        self.raw_stream.close()
        self.prepared_stream.close()
        self.metadata["source_sha256"] = {
            name: hashlib.sha256((self.output / name).read_bytes()).hexdigest()
            for name in ("raw.csv", "prepared.csv")
        }
        self.write_metadata()
        if len(self.results[0]) >= 2:
            for name, results, config in zip(("baseline", "comparison"), self.results, self.configs):
                save_results(results, self.metadata, config, self.output / name)


def draw_comparison(figure, results, window_s=None):
    """Update on the main thread; both configurations use identical input rows."""
    figure.clear()
    axes = figure.subplots(3, 3, sharex=True)
    latest = results[0][-1]["time_s"]
    for rows, label, color in zip(results, ("baseline", "comparison"), ("C0", "C1")):
        rows = [row for row in rows if window_s is None or row["time_s"] >= latest - window_s]
        times = np.array([row["time_s"] for row in rows])
        for index, axis in enumerate("xyz"):
            position = np.array([row[f"p{axis}_m"] for row in rows])
            sigma = np.array([row[f"sigma_{axis}_m"] for row in rows])
            axes[index, 0].plot(times, position, color=color, label=label)
            axes[index, 0].fill_between(times, position - 1.96 * sigma, position + 1.96 * sigma, color=color, alpha=.15)
            axes[index, 1].plot(times, [row[f"v{axis}_m_s"] for row in rows], color=color)
            axes[index, 2].plot(times, [row[f"innovation_{axis}_m"] for row in rows], color=color)
            if label == "baseline":
                axes[index, 0].plot(times, [row[f"measurement_{axis}_m"] for row in rows], ".", color="0.4", label="onboard position")
    for index, axis in enumerate("xyz"):
        for column, quantity in enumerate(("position [m]", "velocity [m/s]", "innovation [m]")):
            axes[index, column].set_ylabel(f"{axis} {quantity}")
            axes[index, column].grid(True)
    for axis in axes[-1]:
        axis.set_xlabel("Firmware log time since first sample [s]")
    axes[0, 0].legend(fontsize="small")
    figure.suptitle("Live cascaded estimates - shaded bands are model uncertainty, not validated accuracy")
    figure.tight_layout()


def collect(session, uri, duration_s, no_show):
    # Radio dependencies are deliberately absent from offline imports and --help.
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    import matplotlib.pyplot as plt

    events = queue.Queue(maxsize=5000)
    errors = queue.Queue()

    def callback(stream):
        def receive(timestamp, data, _config):
            try:
                events.put_nowait((stream, timestamp, dict(data), time.monotonic()))
            except queue.Full:
                errors.put("telemetry queue overflow; stop and use the recording fallback")
        return receive

    cflib.crtp.init_drivers()
    print(f"Connecting to {uri}; engaging firmware stop before hand-held recording", flush=True)
    with SyncCrazyflie(uri, cf=Crazyflie()) as connection:
        cf = connection.cf
        connection.wait_for_params()
        session.metadata["motor_stop_check"] = engage_motor_stop(cf)
        session.write_metadata()
        print("STOP VERIFIED: firmware locked, four motor outputs zero. "
              "Hand-held recording may begin; keep clear of propellers.", flush=True)
        missing = [name for names in CHANNELS.values() for name in names
                   if cf.log.toc.get_element_by_complete_name(name) is None]
        if missing:
            raise ValueError("firmware lacks required log channels: " + ", ".join(missing))
        cf.connection_lost.add_callback(lambda _uri, message: errors.put("connection lost: " + message))
        configs = []
        figure = None
        started = time.monotonic()
        sample_started = None
        last_packet = started
        last_plot = started
        last_sample_count = 0
        last_sample_received = started
        last_status = started - 5
        try:
            for stream, names in CHANNELS.items():
                config = LogConfig(name="E6_" + stream, period_in_ms=PERIOD_MS)
                for name in names:
                    config.add_variable(name, "float")
                cf.log.add_config(config)
                configs.append(config)
                config.data_received_cb.add_callback(callback(stream))
                config.error_cb.add_callback(lambda _config, message: errors.put("logging error: " + message))
                config.start()
            while True:
                now = time.monotonic()
                if not errors.empty():
                    raise ValueError(errors.get_nowait())
                try:
                    event = events.get(timeout=.02)
                    session.enqueue(*event)
                    last_packet = now
                except queue.Empty:
                    pass
                session.drain(now)
                if len(session.results[0]) != last_sample_count:
                    last_sample_count = len(session.results[0])
                    last_sample_received = now
                if session.results[0] and sample_started is None:
                    sample_started = now
                    print("Receiving data. Move slowly by hand; include rest, translation and gentle rotation.")
                if sample_started is None and now - started > 10:
                    raise ValueError("no usable telemetry within 10 s; check channels/positioning or use prepared data")
                if now - last_packet > 1:
                    raise ValueError("telemetry stopped; partial data saved")
                if sample_started is not None and now - last_sample_received > 1:
                    raise ValueError("usable IMU/position samples stopped; partial data saved")
                if session.latest_sample is not None and now - last_status >= 5:
                    stamp, acceleration, _ = session.latest_sample
                    print(f"t={stamp:.1f} s; world acceleration={np.round(acceleration, 3)} m/s2; motor outputs zero")
                    last_status = now
                if sample_started is not None and now - sample_started >= duration_s:
                    break
                if not no_show and session.results[0] and now - last_plot >= .5:
                    if figure is not None and not plt.fignum_exists(figure.number):
                        break
                    if figure is None:
                        figure = plt.figure(figsize=(13, 9))
                        plt.show(block=False)
                    draw_comparison(figure, session.results, window_s=10)
                    plt.pause(.001)
                    last_plot = now
            session.drain(time.monotonic(), force=True)
        finally:
            for config in configs:
                try:
                    config.stop()
                    config.delete()
                except Exception:
                    pass  # Radio loss must not prevent saving data already received.
            if figure is not None:
                plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=os.getenv("CF_URI"))
    parser.add_argument("--output", type=Path, required=True, help="New directory, never overwritten")
    parser.add_argument("--duration", type=float, default=60, help="Capture seconds after the first usable sample")
    parser.add_argument("--position-std", type=float, default=.04)
    parser.add_argument("--comparison-position-std", type=float, default=.12)
    parser.add_argument("--acc-bias-g", type=float, nargs=3, default=(0, 0, 0), metavar=("BX", "BY", "BZ"), help="Additional validated body-axis bias to subtract, in g; do not subtract onboard correction twice")
    parser.add_argument("--prepare-for-handling", "--motors-disabled", dest="prepare_for_handling",
                        action="store_true", help="Confirm drone is on a stable surface, clear of propellers; helper engages and verifies firmware stop")
    parser.add_argument("--no-show", action="store_true", help="Save outputs without a live GUI")
    args = parser.parse_args(argv)
    if not args.uri or not args.prepare_for_handling:
        parser.error("set CF_URI or --uri; place the drone on a stable surface, keep clear of propellers, and confirm --prepare-for-handling")
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error("--duration must be between 1 and 600 seconds")
    if any(not math.isfinite(value) or value <= 0 for value in (args.position_std, args.comparison_position_std)):
        parser.error("position standard deviations must be finite and positive")
    if not all(math.isfinite(value) for value in args.acc_bias_g):
        parser.error("accelerometer bias must be finite")
    configs = [FilterConfig(position_noise_std=value) for value in (args.position_std, args.comparison_position_std)]
    session = None
    status, reason = "complete", ""
    try:
        session = LiveSession(args.output, configs, args.acc_bias_g, args.uri)
        collect(session, args.uri, args.duration, args.no_show)
    except KeyboardInterrupt:
        status, reason = "interrupted", "operator stopped capture"
    except Exception as error:
        status, reason = "failed", str(error)
    finally:
        if session is not None:
            session.finish(status, reason)
    if session is None or len(session.results[0]) < 2 or status == "failed":
        parser.exit(1, f"Live capture stopped: {reason or 'fewer than two samples'}. Use the prepared recording fallback.\n")
    import matplotlib.pyplot as plt
    figure = plt.figure(figsize=(13, 9))
    draw_comparison(figure, session.results)
    figure.savefig(args.output / "live-comparison.png", dpi=150)
    plt.close(figure)
    print(f"Saved replayable input, two filter outputs and comparison to {args.output}")
    print("Firmware stop remains engaged. Place the drone down and power-cycle before flight/preflight checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
