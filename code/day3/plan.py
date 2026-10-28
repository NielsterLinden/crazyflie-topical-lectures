#!/usr/bin/env python3
"""Construct and preview an offline course; never connects to a drone."""

import argparse
import csv
import json
from pathlib import Path
from trajectory_tools.course import construct_course, read_json
from trajectory_tools.planner import build_trajectory
from trajectory_tools.preview import plot_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course", type=Path, required=True, help="Course or template JSON with endpoints and agreed limits")
    parser.add_argument("--gate-survey", action="append", type=Path, default=[], help="measure_hoop.py JSON; repeat for multiple gates")
    parser.add_argument("--crossing", action="append", default=[], help="Survey name followed by :+ or :-; repeat in crossing order")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--frame-margin", type=float, help="Physical frame/support padding beyond each opening, metres per side; overrides every gate")
    parser.add_argument("--name", help="Name for the constructed course")
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing directories are never overwritten")
    parser.add_argument("--show", action="store_true", help="Open the interactive Matplotlib preview as well as saving PNG")
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError("output already exists; choose a new directory")
        course = construct_course(read_json(args.course), [read_json(path) for path in args.gate_survey],
                                  args.crossing, args.laps, frame_margin_m=args.frame_margin)
        if args.name:
            course["course_name"] = args.name
        trajectory = build_trajectory(course)
        times, positions, velocities, accelerations = trajectory.sample(course["limits"]["setpoint_rate_hz"])
        figure = plot_trajectory(course, trajectory)
        args.output.mkdir(parents=True)
        (args.output / "course.json").write_text(json.dumps(course, indent=2) + "\n", encoding="utf-8")
        with (args.output / "trajectory.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "ax_m_s2", "ay_m_s2", "az_m_s2"])
            for t, p, v, a in zip(times, positions, velocities, accelerations):
                writer.writerow([t, *p, *v, *a])
        summary = {
            "planned_duration_s": trajectory.duration_s,
            "speed_bound_m_s": trajectory.speed_bound_m_s,
            "acceleration_bound_m_s2": trajectory.acceleration_bound_m_s2,
            "crossings": [{"name": gate["name"], "time_s": time} for gate, time in zip(course["hoops"], trajectory.crossing_times_s)],
            "source_course": str(args.course), "source_surveys": [str(path) for path in args.gate_survey],
            "note": "Geometric reference only; no simulated flight, measured finish time or flight approval.",
        }
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        figure.savefig(args.output / "preview.png", dpi=160)
        print(f"Planned duration {trajectory.duration_s:.2f} s (excludes takeoff/landing); {len(course['hoops'])} crossings")
        print(f"Wrote disabled course, trajectory CSV, summary and preview to {args.output}")
        print("Preview only: the qualified runner may construct a different path. Check its own dry run before flight.")
        import matplotlib.pyplot as plt
        if args.show:
            plt.show()
        plt.close(figure)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Planning failed: {error}\n")


if __name__ == "__main__":
    main()
