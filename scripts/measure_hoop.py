#!/usr/bin/env python3
"""Survey a gate outline in the Lighthouse frame with a hand-moved Crazyflie."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cflib.crazyflie.log import LogConfig


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "code"))

from race_common.hoop_fit import (  # noqa: E402
    CircleFit3D,
    RectangleFit3D,
    fit_circle_3d,
    fit_rectangle_3d,
)


class PositionSampler:
    """Collect timestamped position estimates from the cflib logging thread."""

    def __init__(self) -> None:
        self._samples: list[tuple[float, np.ndarray]] = []
        self._lock = threading.Lock()
        self.first_sample = threading.Event()

    def callback(self, _timestamp: int, data: dict, _log_config: LogConfig) -> None:
        position = np.array([
            data["stateEstimate.x"],
            data["stateEstimate.y"],
            data["stateEstimate.z"],
        ])
        with self._lock:
            self._samples.append((time.monotonic(), position))
        self.first_sample.set()

    def samples_since(self, start_time: float) -> np.ndarray:
        with self._lock:
            return np.asarray([
                position for timestamp, position in self._samples
                if timestamp >= start_time
            ])


def _rounded(values: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values]


def _print_fit(fit: CircleFit3D) -> None:
    print("\nCircle fit:")
    print(f"  centre       : {_rounded(fit.centre)} m")
    print(f"  normal       : {_rounded(fit.normal)}")
    print(f"  radius       : {fit.radius_m:.6f} m")
    print(f"  plane RMS/max: {fit.plane_rms_m:.6f} / {fit.plane_max_m:.6f} m")
    print(f"  radial RMS/max: {fit.radial_rms_m:.6f} / {fit.radial_max_m:.6f} m")


def _print_rectangle_fit(fit: RectangleFit3D) -> None:
    print("\nRectangle fit:")
    print(f"  centre       : {_rounded(fit.centre)} m")
    print(f"  normal       : {_rounded(fit.normal)}")
    print(f"  width axis   : {_rounded(fit.width_axis)}")
    print(f"  height axis  : {_rounded(fit.height_axis)}")
    print(f"  width x height: {fit.width_m:.6f} x {fit.height_m:.6f} m")
    print(f"  plane RMS/max: {fit.plane_rms_m:.6f} / {fit.plane_max_m:.6f} m")
    print(f"  outline RMS/max: {fit.outline_rms_m:.6f} / {fit.outline_max_m:.6f} m")


def _write_result(
    path: Path,
    hoop_name: str,
    fit: CircleFit3D,
    points: list[np.ndarray],
    force: bool,
) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"{path} already exists; use --force to replace it")
    result = {
        "name": hoop_name,
        "centre": _rounded(fit.centre),
        "normal": _rounded(fit.normal),
        "radius_m": round(fit.radius_m, 6),
        "survey": {
            "point_count": len(points),
            "plane_rms_m": round(fit.plane_rms_m, 6),
            "plane_max_m": round(fit.plane_max_m, 6),
            "radial_rms_m": round(fit.radial_rms_m, 6),
            "radial_max_m": round(fit.radial_max_m, 6),
            "points_m": [_rounded(point) for point in points],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


def _write_rectangle_result(
    path: Path,
    gate_name: str,
    fit: RectangleFit3D,
    points: list[np.ndarray],
    force: bool,
) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"{path} already exists; use --force to replace it")
    result = {
        "name": gate_name,
        "shape": "rectangle",
        "centre": _rounded(fit.centre),
        "normal": _rounded(fit.normal),
        "width_axis": _rounded(fit.width_axis),
        "height_axis": _rounded(fit.height_axis),
        "width_m": round(fit.width_m, 6),
        "height_m": round(fit.height_m, 6),
        # Existing circular race logic can consume this conservative fallback.
        "radius_m": round(min(fit.width_m, fit.height_m) / 2.0, 6),
        "survey": {
            "point_count": len(points),
            "plane_rms_m": round(fit.plane_rms_m, 6),
            "plane_max_m": round(fit.plane_max_m, 6),
            "outline_rms_m": round(fit.outline_rms_m, 6),
            "outline_max_m": round(fit.outline_max_m, 6),
            "points_m": [_rounded(point) for point in points],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


def _interactive_survey(
    sampler: PositionSampler,
    shape: str,
    minimum_spacing_m: float,
) -> list[np.ndarray]:
    outline = "hoop rim" if shape == "circle" else "rectangular gate outline"
    print(f"Place the centre-referenced pointer on the {outline}.")
    print("Press Enter to start recording, move once around the full outline, then")
    print("press Enter again to stop. Keep the motors disabled throughout.")
    while True:
        command = input("Command [Enter=record, q=quit]: ").strip().lower()
        if command == "q":
            return []
        if command:
            print("Unknown command. Press Enter to record or q to quit.")
            continue

        start_time = time.monotonic()
        input("Recording... press Enter after completing the outline: ")
        raw_points = sampler.samples_since(start_time)
        points = _spatially_subsample(raw_points, minimum_spacing_m)
        print(
            f"Captured {len(raw_points)} position samples; retained {len(points)} "
            "spatially separated points."
        )
        minimum_points = 6 if shape == "circle" else 12
        if len(points) < minimum_points:
            print(
                f"Need at least {minimum_points} retained points. Move more slowly "
                "around the complete outline or reduce --sample-spacing."
            )
            continue

        accept = input("Fit this trace? [Y/n]: ").strip().lower()
        if accept in ("", "y", "yes"):
            return [point for point in points]


def _spatially_subsample(points: np.ndarray, minimum_spacing_m: float) -> np.ndarray:
    """Keep samples separated in space, reducing dwell-time weighting."""
    if len(points) == 0:
        return np.empty((0, 3))
    retained = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - retained[-1]) >= minimum_spacing_m:
            retained.append(point)
    return np.asarray(retained)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--uri",
        default=os.getenv("CF_URI"),
        help="Crazyflie radio URI; defaults to CF_URI.",
    )
    parser.add_argument("--name", default="hoop-1", help="Hoop name in race configuration.")
    parser.add_argument(
        "--shape",
        choices=("circle", "rectangle"),
        default="circle",
        help="Geometry to survey.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("hoop-measurement.json"),
        help="Output JSON path.",
    )
    parser.add_argument("--force", action="store_true", help="Replace an existing output file.")
    parser.add_argument("--log-period-ms", type=int, default=50)
    parser.add_argument(
        "--sample-spacing",
        type=float,
        default=0.02,
        help="Minimum distance in metres between retained outline samples.",
    )
    parser.add_argument(
        "--normal-towards",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Orient the fitted normal towards this Lighthouse-frame direction.",
    )
    args = parser.parse_args()
    if not args.uri:
        parser.error("set CF_URI or pass --uri for the assigned drone")
    if args.log_period_ms <= 0 or args.sample_spacing <= 0.0:
        parser.error("log period and sample spacing must be positive")
    return args


def main() -> int:
    args = parse_args()
    # Keep --help and offline geometry tests usable without the radio libraries.
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    sampler = PositionSampler()
    log_config = LogConfig(name="Hoop survey", period_in_ms=args.log_period_ms)
    for variable in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        log_config.add_variable(variable, "float")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(sampler.callback)
        log_config.start()
        try:
            if not sampler.first_sample.wait(timeout=5.0):
                raise RuntimeError("no position data received within five seconds")
            points = _interactive_survey(sampler, args.shape, args.sample_spacing)
        finally:
            log_config.stop()
            log_config.delete()

    if not points:
        print("No measurement written.")
        return 1

    if args.shape == "circle":
        fit = fit_circle_3d(np.asarray(points), args.normal_towards)
        _print_fit(fit)
    else:
        fit = fit_rectangle_3d(np.asarray(points), args.normal_towards)
        _print_rectangle_fit(fit)
    if args.normal_towards is None:
        flip = input(
            "Flip the normal to select the opposite crossing direction? [y/N] "
        )
        if flip.lower() == "y":
            if isinstance(fit, CircleFit3D):
                fit = CircleFit3D(
                    centre=fit.centre,
                    normal=-fit.normal,
                    radius_m=fit.radius_m,
                    plane_rms_m=fit.plane_rms_m,
                    radial_rms_m=fit.radial_rms_m,
                    plane_max_m=fit.plane_max_m,
                    radial_max_m=fit.radial_max_m,
                )
            else:
                fit = RectangleFit3D(
                    centre=fit.centre,
                    normal=-fit.normal,
                    width_axis=fit.width_axis,
                    height_axis=-fit.height_axis,
                    width_m=fit.width_m,
                    height_m=fit.height_m,
                    plane_rms_m=fit.plane_rms_m,
                    outline_rms_m=fit.outline_rms_m,
                    plane_max_m=fit.plane_max_m,
                    outline_max_m=fit.outline_max_m,
                )
            print(f"  selected normal: {_rounded(fit.normal)}")

    if isinstance(fit, CircleFit3D):
        _write_result(args.output, args.name, fit, points, args.force)
    else:
        _write_rectangle_result(args.output, args.name, fit, points, args.force)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
