"""Notebook baseline capture and tuning-file construction; no flight commands."""

import argparse
import json
import os
from pathlib import Path
import sys

from notebook_radio import motor_stopped_connection, run_worker
from race_common.tuning import PARAMETERS, parameter_value, validate_tuning


def capture_baseline(uri, directory):
    """Capture on the grounded, stopped drone; leave firmware stop engaged."""
    if not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Identify the grounded drone first")
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if (directory / "baseline.json").exists():
        raise FileExistsError("Choose a new baseline capture folder")
    result = run_worker([sys.executable, "-u", "-m", "day3.tuning", "--uri", uri,
                         "--output", str(directory / "baseline.json")],
                        cwd=Path(__file__).resolve().parents[1], env=dict(os.environ), timeout_s=30)
    if result.returncode:
        raise RuntimeError("Baseline capture failed; use the checked fallback.\n" + result.stdout)
    validate_tuning(make_tuning(directory / "baseline.json"), uri)
    return directory / "baseline.json"


def make_tuning(baseline_path, *, mode="baseline", factor=1., factor_range=None, range_checked=False,
                parameter_group=None):
    baseline = json.loads(Path(baseline_path).read_text(encoding="utf-8-sig"))
    if baseline.get("motor_stop_check", {}).get("value") != 1:
        raise ValueError("Use a baseline captured with verified firmware stop")
    tuning = dict(version=1, uri=baseline["uri"], baseline=baseline["parameters"],
                  mode=mode, factor=factor, factor_range=factor_range, range_checked=range_checked)
    if parameter_group is not None:
        tuning["parameter_group"] = parameter_group
    return validate_tuning(tuning)


def _capture(uri, path):
    with motor_stopped_connection(uri, path.parent / ".cflib-cache") as (connection, stop_check):
        cf = connection.cf
        for name, expected in (("stabilizer.controller", 1), ("stabilizer.estimator", 2)):
            if parameter_value(cf, name) != expected:
                raise ValueError("Prepare and check PID controller 1 / Kalman estimator 2 before capturing a baseline")
        parameters = {name: parameter_value(cf, name) for name in PARAMETERS
                      if cf.param.toc.get_element_by_complete_name(name) is not None}
        # Validate required gains and values before marking the capture complete.
        validate_tuning(dict(version=1, uri=uri, baseline=parameters, mode="baseline",
                             factor=1., factor_range=None, range_checked=False))
        record = dict(uri=uri, parameters=parameters, motor_stop_check=stop_check)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2)
    print("Baseline captured. Firmware stop remains engaged; power-cycle before flight preflight.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    _capture(args.uri, args.output)
