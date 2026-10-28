"""Course/trajectory pairing stays valid across notebook edits and failures."""

from pathlib import Path
import unittest

from trajectory_tools.course import construct_course, read_json
from .notebook_tools import checked_plan, plan_course


class PlanTests(unittest.TestCase):
    def course(self):
        root = Path(__file__).resolve().parents[2]
        return construct_course(read_json(root / "examples/course.shuttle.synthetic.json"), laps=2)

    def test_plan_owns_a_copy_and_returns_a_matched_disabled_course(self):
        source = self.course()
        plan = plan_course(source)
        source["limits"]["target_speed_m_s"] = .45
        course, trajectory = checked_plan(plan)
        self.assertIs(trajectory, plan.trajectory)
        self.assertEqual(course["limits"]["target_speed_m_s"], .3)
        self.assertIs(course["enabled_for_flight"], False)
        self.assertEqual(len(course["hoops"]), len(trajectory.crossing_times_s))

    def test_missing_or_modified_plan_cannot_be_exported(self):
        with self.assertRaisesRegex(ValueError, "No valid selected plan"):
            checked_plan(None)
        with self.assertRaisesRegex(ValueError, "No valid course inputs"):
            plan_course(None)
        plan = plan_course(self.course())
        plan.course["limits"]["target_speed_m_s"] = .4
        with self.assertRaisesRegex(ValueError, "changed since validation"):
            checked_plan(plan)
        plan = plan_course(self.course())
        plan.trajectory.spline.c[0, 0, 0] += .1
        with self.assertRaisesRegex(ValueError, "changed since validation"):
            checked_plan(plan)

    def test_checked_course_can_be_changed_without_mutating_baseline(self):
        source = self.course()
        source["limits"]["pre_post_distance_m"] = .35
        baseline = plan_course(source)
        candidate, _ = checked_plan(baseline)
        candidate["limits"]["target_speed_m_s"] = .4
        candidate_plan = plan_course(candidate)
        restored, _ = checked_plan(candidate_plan)
        restored["limits"]["target_speed_m_s"] = source["limits"]["target_speed_m_s"]
        self.assertEqual(restored, source)
        self.assertEqual(checked_plan(baseline)[0], source)

    def test_invalid_approach_does_not_produce_a_plan(self):
        course = self.course()
        course["limits"]["pre_post_distance_m"] = 1.6
        with self.assertRaisesRegex(ValueError, "control hull"):
            plan_course(course)

    def test_provenance_is_snapshotted_and_inherited_without_shared_mutable_state(self):
        provenance = {"source_surveys": ["gate.json"], "crossing_order": ["gate:+"], "laps": 2}
        baseline = plan_course(self.course(), provenance=provenance)
        provenance["source_surveys"].append("unused.json")
        provenance["laps"] = 99
        candidate, _ = checked_plan(baseline)
        candidate["limits"]["target_speed_m_s"] = .4
        candidate_plan = plan_course(candidate, provenance=baseline.provenance)
        detached = candidate_plan.provenance
        detached["source_surveys"].clear()
        detached["laps"] = 10
        expected = {"source_surveys": ["gate.json"], "crossing_order": ["gate:+"], "laps": 2}
        self.assertEqual(baseline.provenance, expected)
        self.assertEqual(candidate_plan.provenance, expected)


if __name__ == "__main__":
    unittest.main()
