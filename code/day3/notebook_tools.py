"""Keep a notebook's course and computed trajectory together for export."""

from copy import deepcopy
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from trajectory_tools.planner import Trajectory, build_trajectory
from race_common.hoop import Hoop


def clearance_budget(course, position_covariance, *, survey_allowance_m,
                     tracking_allowance_m, sigma_multiplier=2.5):
    """Project Day 2's position covariance into each gate's opening plane.

    Add survey and control-tracking allowances as conservative bounds. Their
    sum is a planning allowance, not a calibrated collision probability.
    """
    if not isinstance(course, dict):
        raise ValueError("Select valid course inputs before budgeting clearance.")
    covariance = np.asarray(position_covariance, dtype=float)
    if (covariance.shape != (3, 3) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, rtol=0, atol=1e-12)):
        raise ValueError("Position covariance must be a finite symmetric 3x3 matrix in m^2.")
    if np.linalg.eigvalsh(covariance).min() < -1e-12:
        raise ValueError("Position covariance must be positive semidefinite.")
    allowances = np.asarray([survey_allowance_m, tracking_allowance_m, sigma_multiplier])
    if not np.isfinite(allowances).all() or (allowances < 0).any() or sigma_multiplier == 0:
        raise ValueError("Allowances must be nonnegative and the sigma multiplier positive.")
    gates = []
    for entry in course['hoops']:
        gate = Hoop(entry['centre'], entry['normal'], entry['radius_m'])
        plane_covariance = gate.plane_covariance(covariance)
        sigma = float(np.sqrt(max(0.0, np.linalg.eigvalsh(plane_covariance).max())))
        margin = float(survey_allowance_m + tracking_allowance_m + sigma_multiplier * sigma)
        gates.append({'name': entry['name'], 'plane_std_m': sigma, 'required_margin_m': margin})
    if not gates:
        raise ValueError("A clearance budget needs at least one gate.")
    return {'position_covariance_m2': covariance.tolist(),
            'survey_allowance_m': float(survey_allowance_m),
            'tracking_allowance_m': float(tracking_allowance_m),
            'sigma_multiplier': float(sigma_multiplier),
            'required_margin_m': max(gate['required_margin_m'] for gate in gates),
            'gates': gates}


def _trajectory_signature(trajectory):
    digest = hashlib.sha256()
    digest.update(trajectory.spline.x.tobytes())
    digest.update(trajectory.spline.c.tobytes())
    digest.update(repr((trajectory.crossing_times_s, trajectory.speed_bound_m_s,
                        trajectory.acceleration_bound_m_s2)).encode("ascii"))
    return digest.hexdigest()


@dataclass(frozen=True)
class CoursePlan:
    course: dict
    trajectory: Trajectory
    course_snapshot: str
    trajectory_signature: str
    provenance_snapshot: str

    @property
    def provenance(self):
        """Return a fresh copy so later notebook edits cannot change provenance."""
        return json.loads(self.provenance_snapshot)


def plan_course(course, *, provenance=None):
    """Return a matched pair only after the planner accepts a private input copy."""
    if not isinstance(course, dict):
        raise ValueError("No valid course inputs. Correct them and rerun course construction before planning.")
    course = deepcopy(course)
    trajectory = build_trajectory(course)
    return CoursePlan(course, trajectory, json.dumps(course, sort_keys=True),
                      _trajectory_signature(trajectory),
                      json.dumps(provenance if provenance is not None else {}, sort_keys=True))


def checked_plan(plan):
    """Reject missing or edited plans before any export files are written."""
    if not isinstance(plan, CoursePlan):
        raise ValueError("No valid selected plan. Correct the inputs and rerun planning before export.")
    if (json.dumps(plan.course, sort_keys=True) != plan.course_snapshot
            or _trajectory_signature(plan.trajectory) != plan.trajectory_signature):
        raise ValueError("Plan changed since validation. Rerun planning before export.")
    if plan.course.get("enabled_for_flight") is not False:
        raise ValueError("Notebook exports must keep enabled_for_flight=false.")
    return deepcopy(plan.course), plan.trajectory


def save_plan(plan, directory):
    """Write the checked course, reference samples and provenance together.

    The notebook owns the run directory. Re-exporting there replaces the three
    plan files; use a new run label to retain a separate comparison.
    """
    course, trajectory = checked_plan(plan)
    times, positions, velocities, accelerations = trajectory.sample(
        course["limits"]["setpoint_rate_hz"])
    summary = {
        **plan.provenance,
        "planned_duration_s": trajectory.duration_s,
        "speed_bound_m_s": trajectory.speed_bound_m_s,
        "acceleration_bound_m_s2": trajectory.acceleration_bound_m_s2,
        "crossings": [{"name": gate["name"], "time_s": time}
                      for gate, time in zip(course["hoops"], trajectory.crossing_times_s)],
        "note": "Planned reference only; no flight commands or physical qualification.",
    }
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "course.json").write_text(json.dumps(course, indent=2) + "\n", encoding="utf-8")
    np.savetxt(directory / "trajectory.csv",
               np.column_stack((times, positions, velocities, accelerations)),
               delimiter=",", comments="",
               header="time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_m_s2,az_m_s2")
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def save_example_replay(plan, directory, lag_s=0.12):
    """Save a labelled delayed-reference example for the offline R7 analysis.

    The position stream is exactly p_reference(t - lag), clipped at the start.
    This exercises timing and plotting; it is not a simulated drone/controller.
    Like save_plan, this replaces generated files in the notebook-owned directory.
    """
    course, trajectory = checked_plan(plan)
    if not np.isfinite(lag_s) or lag_s < 0:
        raise ValueError("Example lag must be finite and nonnegative.")
    times, positions, _, _ = trajectory.sample(course['limits']['setpoint_rate_hz'])
    delayed_positions = trajectory.spline(np.maximum(times - lag_s, 0.0))
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    course_path = directory / 'course.json'
    course_path.write_text(json.dumps(course, indent=2) + '\n', encoding='utf-8')
    telemetry_path = directory / 'telemetry.synthetic.csv'
    fields = ['event', 'host_monotonic_s', 'phase', 'x_m', 'y_m', 'z_m',
              'command_x_m', 'command_y_m', 'command_z_m']
    with telemetry_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for time, target, position in zip(times, positions, delayed_positions):
            writer.writerow({'event': 'command', 'host_monotonic_s': time, 'phase': 'race',
                             'command_x_m': target[0], 'command_y_m': target[1],
                             'command_z_m': target[2]})
            writer.writerow({'event': 'position', 'host_monotonic_s': time,
                             'x_m': position[0], 'y_m': position[1], 'z_m': position[2]})
    source = {'source': 'synthetic delayed-reference example', 'lag_s': float(lag_s),
              'plan_provenance': plan.provenance,
              'note': 'Prescribed positions; no drone dynamics, estimator or flight-time prediction.'}
    (directory / 'source.json').write_text(json.dumps(source, indent=2) + '\n', encoding='utf-8')
    return telemetry_path, course_path
