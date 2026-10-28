#!/usr/bin/env python3
"""Compare recorded race setpoints and onboard position estimates offline."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from race_common.geometry import plane_basis, unit_vector


@dataclass(frozen=True)
class RaceLog:
    position_times: np.ndarray
    positions: np.ndarray
    command_times: np.ndarray
    commands: np.ndarray
    phases: np.ndarray


@dataclass(frozen=True)
class Comparison:
    times: np.ndarray
    positions: np.ndarray
    targets: np.ndarray
    errors: np.ndarray
    command_times: np.ndarray
    commands: np.ndarray
    time_origin: float


def load_telemetry(path: Path) -> RaceLog:
    """Read sparse event rows; never forward-fill sensor diagnostics."""
    positions, commands = [], []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not {"event", "host_monotonic_s"}.issubset(reader.fieldnames or []):
            raise ValueError("Expected a race CSV with event and host_monotonic_s columns")
        for line, row in enumerate(reader, start=2):
            event = row["event"]
            if event not in ("position", "command"):
                continue
            fields = ("x_m", "y_m", "z_m") if event == "position" else (
                "command_x_m", "command_y_m", "command_z_m",
            )
            try:
                values = [float(row[key]) for key in ("host_monotonic_s", *fields)]
                if not np.all(np.isfinite(values)):
                    raise ValueError("non-finite timestamp or position")
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"Invalid {event} row at CSV line {line}: {error}") from error
            if event == "position":
                positions.append(values)
            else:
                commands.append((*values, row.get("phase", "") or ""))
    if not positions or not commands:
        raise ValueError("Need both event=position and event=command rows; no setpoints to compare")
    # Callback threads can append rows in a different order to acquisition.
    positions.sort(key=lambda row: row[0])
    commands.sort(key=lambda row: row[0])
    position_array = np.asarray(positions, dtype=float)
    return RaceLog(
        position_array[:, 0], position_array[:, 1:],
        np.asarray([row[0] for row in commands]),
        np.asarray([row[1:4] for row in commands]),
        np.asarray([row[4] for row in commands]),
    )


def select_phase(log: RaceLog, phase: str) -> str:
    if phase != "auto":
        return phase
    for candidate in ("race", "trajectory"):
        if candidate in log.phases:
            return candidate
    raise ValueError("No race or trajectory commands; select an explicit --phase")


def compare(log: RaceLog, phase: str, max_gap_s: float) -> Comparison:
    phase = select_phase(log, phase)
    if not np.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("max-gap must be finite and positive")
    selected_commands = np.ones(len(log.commands), dtype=bool)
    if phase != "all":
        selected_commands = log.phases == phase
    if not np.any(selected_commands):
        available = ", ".join(sorted(set(log.phases)))
        raise ValueError(f"No commands for phase {phase!r}; available phases: {available}")

    # Setpoints are held until the next send, not linearly interpolated into
    # future commands. Host receive time includes radio/transport latency.
    indices = np.searchsorted(log.command_times, log.position_times, side="right") - 1
    indices = np.maximum(indices, 0)
    age = log.position_times - log.command_times[indices]
    matched = (age >= 0) & (age <= max_gap_s) & selected_commands[indices]
    first = log.command_times[selected_commands][0]
    last = log.command_times[selected_commands][-1]
    in_window = (log.position_times >= first) & (log.position_times <= last + max_gap_s)
    # Stop at a subsequent phase such as landing instead of plotting it as race.
    if phase != "all":
        in_window &= selected_commands[indices]
    if not np.any(in_window):
        raise ValueError("No position samples overlap the selected commands")
    targets = np.full(log.positions.shape, np.nan)
    targets[matched] = log.commands[indices[matched]]
    return Comparison(
        log.position_times[in_window], log.positions[in_window], targets[in_window],
        (log.positions - targets)[in_window], log.command_times[selected_commands],
        log.commands[selected_commands], first,
    )


def gate_outline(gate: dict) -> np.ndarray:
    centre = np.asarray(gate["centre"], dtype=float)
    if centre.shape != (3,) or not np.all(np.isfinite(centre)):
        raise ValueError("Gate centre must contain three finite numbers")
    normal = unit_vector(gate["normal"], "gate normal")
    if gate.get("shape", "circle") == "rectangle":
        width = float(gate["width_m"])
        height = float(gate["height_m"])
        if not np.isfinite(width + height) or min(width, height) <= 0:
            raise ValueError("Rectangle dimensions must be finite and positive")
        u = unit_vector(gate["width_axis"], "width axis")
        v = unit_vector(gate["height_axis"], "height axis")
        if max(abs(u @ v), abs(normal @ u), abs(normal @ v)) > 1e-3:
            raise ValueError("Rectangle axes must be orthogonal and lie in its plane")
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]])
        return centre + corners[:, :1] * width / 2 * u + corners[:, 1:] * height / 2 * v
    if gate.get("shape", "circle") != "circle":
        raise ValueError(f"Unknown gate shape: {gate['shape']}")
    radius = float(gate["radius_m"])
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Hoop radius must be finite and positive")
    angles = np.linspace(0, 2 * np.pi, 100)
    basis = plane_basis(normal)
    return centre + radius * (
        np.cos(angles)[:, None] * basis[:, 0] + np.sin(angles)[:, None] * basis[:, 1]
    )


def break_gaps(times: np.ndarray, points: np.ndarray, max_gap_s: float) -> np.ndarray:
    """Insert NaNs rather than drawing an invented path through a log dropout."""
    breaks = np.flatnonzero(np.diff(times) > max_gap_s) + 1
    return np.insert(np.asarray(points, dtype=float), breaks, np.nan, axis=0)


def plot_comparison(comparison: Comparison, course: dict, surveys: list[dict],
                    planned: np.ndarray | None, max_gap_s: float):
    import matplotlib.pyplot as plt

    view = plt.figure(figsize=(14, 10), layout="constrained")
    axes = [view.add_subplot(2, 2, 1, projection="3d")]
    axes.extend(view.add_subplot(2, 2, index) for index in (2, 3, 4))
    projections = [(0, 1), (0, 2), (1, 2)]
    measured = break_gaps(comparison.times, comparison.positions, max_gap_s)
    commands = break_gaps(comparison.command_times, comparison.commands, max_gap_s)
    traces = [(commands, "Goal (sent setpoints)", "C0", "--"),
              (measured, "Flown (onboard estimate)", "C1", "-")]
    if planned is not None:
        traces.insert(0, (planned, "Full plan (regenerated)", "0.65", ":"))
    extents = []

    def draw(points, label, color, linestyle):
        extents.append(points)
        axes[0].plot(*points.T, label=label, color=color, linestyle=linestyle)
        for axis, (a, b) in zip(axes[1:], projections):
            axis.plot(points[:, a], points[:, b], label=label, color=color, linestyle=linestyle)

    for points, label, color, linestyle in traces:
        draw(points, label, color, linestyle)

    # Repeated laps describe repeated crossings of the same physical gate.
    # Deduplicate by rim geometry, independent of normal direction.
    seen = set()
    for gate in course["hoops"]:
        outline = gate_outline(gate)
        normal = unit_vector(gate["normal"], "normal")
        if gate.get("shape", "circle") == "rectangle":
            corners = np.round(outline[:-1], 5)
            key = ("rectangle", *sorted(map(tuple, corners)))
        else:
            # n and -n describe the same plane, including a horizontal hoop.
            key = ("circle", *np.round(gate["centre"], 5),
                   *np.round(np.outer(normal, normal).ravel(), 5),
                   round(float(gate["radius_m"]), 5))
        if key in seen:
            continue
        seen.add(key)
        name = f"Gate {len(seen)}: {gate['name']}"
        draw(outline, name, "C2", "-")
        centre = np.asarray(gate["centre"])
        axes[0].quiver(*centre, *normal, length=0.20, color="C2")
        axes[0].text(*centre, f" G{len(seen)}", fontsize=8)
    for survey in surveys:
        draw(gate_outline(survey), f"Survey: {survey.get('name', 'gate')}", "C3", "-")

    for key, marker in (("start", "o"), ("finish", "x")):
        point = np.asarray(course[key], dtype=float)
        extents.append(point[None, :])
        axes[0].scatter(*point, color="black", marker=marker, label=key)
        for axis, (a, b) in zip(axes[1:], projections):
            axis.scatter(point[a], point[b], color="black", marker=marker)

    # Equal metres on all axes; include gates as well as flight samples.
    finite_points = np.concatenate(extents)
    finite_points = finite_points[np.all(np.isfinite(finite_points), axis=1)]
    centre = (finite_points.min(axis=0) + finite_points.max(axis=0)) / 2
    half_span = max(float(np.ptp(finite_points, axis=0).max()) * 0.55, 0.25)
    for setter, value in zip((axes[0].set_xlim, axes[0].set_ylim, axes[0].set_zlim), centre):
        setter(value - half_span, value + half_span)
    axes[0].set_box_aspect((1, 1, 1))
    axes[0].set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title="3D course")
    for axis, (a, b), title in zip(axes[1:], projections, ("Top view", "Side view x/z", "Side view y/z")):
        axis.set(xlabel=f"{'xyz'[a]} [m]", ylabel=f"{'xyz'[b]} [m]", title=title)
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=0.3)
    axes[0].legend(fontsize=7, loc="upper left")
    view.suptitle(course["course_name"])

    errors = plt.figure(figsize=(12, 8), layout="constrained")
    error_axes = errors.subplots(4, 1, sharex=True)
    # Keep both invalid matches and receive gaps visible in time-domain plots.
    values = np.column_stack((comparison.times - comparison.time_origin, comparison.errors))
    values = break_gaps(comparison.times, values, max_gap_s)
    for index, axis in enumerate(error_axes[:3]):
        axis.plot(values[:, 0], values[:, index + 1], color=f"C{index}")
        axis.axhline(0, color="0.6", linewidth=0.8)
        axis.set_ylabel(f"Delta {'xyz'[index]} [m]")
    error_axes[3].plot(values[:, 0], np.linalg.norm(values[:, 1:], axis=1), color="black")
    limit = course.get("limits", {}).get("maximum_tracking_error_m")
    if limit is not None:
        error_axes[3].axhline(limit, color="C3", linestyle="--", label="Configured tracking limit")
        error_axes[3].legend()
    error_axes[3].set(ylabel="Error norm [m]", xlabel="Host time since first selected command [s]")
    for axis in error_axes:
        axis.grid(alpha=0.3)
    errors.suptitle("Onboard position estimate minus last sent setpoint (host-time alignment)")
    return view, errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", type=Path, help="Race telemetry CSV")
    parser.add_argument("--course", type=Path, required=True, help="Course JSON used for the flight")
    parser.add_argument("--phase", default="auto", help="Command phase, all, or auto (default: race, then legacy trajectory)")
    parser.add_argument("--max-gap", type=float, default=0.25, help="Maximum command age/line gap in seconds")
    parser.add_argument("--gate-survey", type=Path, action="append", default=[], help="Optional gate survey JSON; repeat for multiple gates")
    parser.add_argument("--output", type=Path, help="Save PREFIX-trajectory.png and PREFIX-errors.png")
    parser.add_argument("--no-show", action="store_true", help="Save without opening plot windows (requires --output)")
    args = parser.parse_args()
    if args.no_show and args.output is None:
        parser.error("--no-show requires --output")
    try:
        if args.no_show:
            import matplotlib
            matplotlib.use("Agg")
        # Accept JSON saved by Windows editors with or without a UTF-8 BOM.
        course = json.loads(args.course.read_text(encoding="utf-8-sig"))
        surveys = [json.loads(path.read_text(encoding="utf-8-sig")) for path in args.gate_survey]
        log = load_telemetry(args.telemetry)
        phase = select_phase(log, args.phase)
        comparison = compare(log, phase, args.max_gap)
        valid = np.all(np.isfinite(comparison.errors), axis=1)
        if not np.any(valid):
            raise ValueError("No positions have a sufficiently recent command; check timestamps or max-gap")
        planned = None
        figures = plot_comparison(comparison, course, surveys, planned, args.max_gap)
        distances = np.linalg.norm(comparison.errors[valid], axis=1)
        print(f"Compared {valid.sum()}/{len(valid)} position samples in phase {phase}")
        print(f"3D RMS error: {np.sqrt(np.mean(distances**2)):.3f} m; maximum: {distances.max():.3f} m")
        print("Host receive/send alignment includes transport delay; onboard position is not ground truth.")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            for figure, suffix in zip(figures, ("trajectory", "errors")):
                path = args.output.parent / f"{args.output.name}-{suffix}.png"
                figure.savefig(path, dpi=150)
                print(f"Saved {path}")
        import matplotlib.pyplot as plt
        if not args.no_show:
            plt.show()
        for figure in figures:
            plt.close(figure)
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(f"Race plot failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
