"""Keep a notebook's course and computed trajectory together for export."""

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json

from trajectory_tools.planner import Trajectory, build_trajectory


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
