"""Day 1 radio discovery and identity checks for motor-free practicals."""

from contextlib import contextmanager
import argparse
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

import day1.crazyflie as crazyflie

from race_common.motor_stop import engage_motor_stop


def assigned_uri(label=None, uri=None):
    """Find the sticker's actual channel, or check an explicitly assigned URI.

    Discovery uses Day 1's implementation directly.
    Nothing is scanned until this function is called from an opted-in live cell.
    """
    if label is not None:
        return crazyflie.find_drone(label)
    return crazyflie.resolve_uri(uri)


@contextmanager
def motor_stopped_connection(uri, cache_directory):
    """Open a raw-sensor connection without changing estimator/controller settings."""
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    # Match Day 1's driver lifecycle; capture callers provide an overall deadline.
    crazyflie.init()
    cf = Crazyflie(rw_cache=str(cache_directory))
    connection = SyncCrazyflie(uri, cf=cf)
    try:
        connection.open_link()
        connection.wait_for_params()
        check = engage_motor_stop(connection.cf)
        yield connection, check
    finally:
        # SyncCrazyflie's context cleanup is skipped if its __enter__ raises.
        # Close the underlying link even if TOC/parameter download was interrupted.
        failing = sys.exc_info()[0] is not None
        try:
            cf.close_link()
        except Exception:
            if not failing:
                raise


def run_worker(command, *, cwd, env, timeout_s):
    """Relay progress and reap the child on completion, timeout or interruption."""
    lines = []
    messages = queue.Queue()
    with subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, bufsize=1) as process:
        def read_output():
            for line in process.stdout:
                messages.put(line)

        reader = threading.Thread(target=read_output, daemon=True)
        try:
            reader.start()
            deadline = time.monotonic() + timeout_s
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
            if reader.ident is not None:
                reader.join(timeout=1)


def _identify(selection, cache_directory, result_path):
    """Write a receipt only after verified identification and link cleanup."""
    selected = assigned_uri(**selection)
    with motor_stopped_connection(selected, cache_directory) as (connection, check):
        print("STOP VERIFIED: firmware locked, four motor outputs zero.", flush=True)
        crazyflie.status(connection)
        crazyflie.identify(connection)
    with Path(result_path).open("x", encoding="utf-8") as stream:
        json.dump({"uri": selected, "motor_stop_check": check}, stream)


def check_drone(label=None, uri=None, *, cache_directory="./cache", timeout_s=30):
    """Discover, lock, report status and blink; return the actual connected URI.

    Discovery, handshake, parameters, stop verification and blinking share one
    deadline. The confirmation prompt runs after the child has closed the link.
    The stop remains engaged; flight requires a power cycle and fresh preflight.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Identity-check timeout must be finite and positive.")
    with tempfile.TemporaryDirectory(prefix="crazyflie-identity-") as directory:
        result_path = Path(directory) / "identity.json"
        command = [sys.executable, "-u", "-m", "notebook_radio",
                   "--selection", json.dumps({"label": label, "uri": uri}),
                   "--cache", str(Path(cache_directory).resolve()), "--result", str(result_path)]
        try:
            result = run_worker(command, cwd=Path(__file__).resolve().parent,
                                env=dict(os.environ), timeout_s=timeout_s)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError(f"Identity check exceeded {timeout_s:g} s; worker stopped. "
                               "Leave the drone on the surface; use saved data or simulation.") from error
        if result.returncode:
            raise crazyflie.DroneError("Identity check failed; leave the drone on the surface "
                                      "and use saved data or simulation.\n" + result.stdout)
        if not result_path.is_file():
            raise crazyflie.DroneError("Identity check produced no verified result; retry or work offline.")
        receipt = json.loads(result_path.read_text(encoding="utf-8"))
        selected = crazyflie.resolve_uri(receipt["uri"])
        if receipt["motor_stop_check"].get("value") != 1:
            raise crazyflie.DroneError("Firmware stop was not verified; leave the drone on the surface.")
    if input("Did your drone blink? [y/N]: ").strip().lower() not in ("y", "yes"):
        raise crazyflie.DroneError("Drone identity unconfirmed; check the sticker before reconnecting.")
    print("Firmware stop remains engaged.")
    return selected


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bounded notebook identity-check worker")
    parser.add_argument("--selection", required=True, type=json.loads)
    parser.add_argument("--cache", required=True, type=Path)
    parser.add_argument("--result", required=True, type=Path)
    args = parser.parse_args()
    _identify(args.selection, args.cache, args.result)
