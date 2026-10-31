"""Course/trajectory pairing stays valid across notebook edits and failures."""

from pathlib import Path
from copy import deepcopy
import csv
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from trajectory_tools.course import construct_course, read_json
from .notebook_tools import (
    checked_plan, plan_course, save_plan, clearance_budget, save_example_replay,
)


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

    def test_saved_samples_and_provenance_belong_to_the_selected_plan(self):
        provenance = {"source_surveys": ["gate.json"], "laps": 2}
        plan = plan_course(self.course(), provenance=provenance)
        provenance["laps"] = 99
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "run"
            summary = save_plan(plan, directory)
            course = json.loads((directory / "course.json").read_text())
            self.assertEqual(course, plan.course)
            self.assertIs(course["enabled_for_flight"], False)
            self.assertEqual(summary["laps"], 2)
            self.assertEqual(summary, json.loads((directory / "summary.json").read_text()))
            samples = np.genfromtxt(directory / "trajectory.csv", delimiter=",", names=True)
            for name, expected in zip(("x_m", "y_m", "z_m"),
                                      plan.trajectory.spline(samples["time_s"]).T):
                np.testing.assert_allclose(samples[name], expected)
            self.assertEqual(len(summary["crossings"]), len(course["hoops"]))

    def test_invalid_plan_does_not_write_or_replace_export_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "run"
            with self.assertRaises(ValueError):
                save_plan(None, directory)
            self.assertFalse(directory.exists())
            plan = plan_course(self.course())
            save_plan(plan, directory)
            before = {path.name: path.read_bytes() for path in directory.iterdir()}
            plan.course["enabled_for_flight"] = True
            with self.assertRaises(ValueError):
                save_plan(plan, directory)
            self.assertEqual(before, {path.name: path.read_bytes() for path in directory.iterdir()})

    def test_clearance_budget_projects_state_covariance_and_preserves_course(self):
        course = self.course()
        original = deepcopy(course)
        covariance = np.diag([0.01, 0.04, 0.09])
        budget = clearance_budget(course, covariance, survey_allowance_m=.01,
                                  tracking_allowance_m=.03, sigma_multiplier=2.5)
        # The X-normal gate sees only Y/Z uncertainty; the largest sigma is .3 m.
        self.assertAlmostEqual(budget['gates'][0]['plane_std_m'], .3)
        self.assertAlmostEqual(budget['required_margin_m'], .01 + .03 + 2.5 * .3)
        self.assertEqual(course, original)
        course['hoops'][0]['normal'] = [0, 0, 1]
        horizontal = clearance_budget(course, covariance, survey_allowance_m=.01,
                                      tracking_allowance_m=.03, sigma_multiplier=2.5)
        self.assertAlmostEqual(horizontal['gates'][0]['plane_std_m'], .2)

    def test_clearance_budget_rejects_invalid_covariance_and_allowances(self):
        for covariance in (np.eye(2), np.diag([1, -1, 1]), np.eye(3) * np.nan,
                           [[1, .1, 0], [0, 1, 0], [0, 0, 1]]):
            with self.subTest(covariance=covariance), self.assertRaises(ValueError):
                clearance_budget(self.course(), covariance, survey_allowance_m=.01,
                                 tracking_allowance_m=.03)
        for keyword, value in (('survey_allowance_m', -.01), ('tracking_allowance_m', np.nan),
                               ('sigma_multiplier', 0)):
            arguments = dict(survey_allowance_m=.01, tracking_allowance_m=.03)
            arguments[keyword] = value
            with self.subTest(keyword=keyword), self.assertRaises(ValueError):
                clearance_budget(self.course(), np.eye(3), **arguments)

    def test_example_replay_uses_the_validated_plan_and_declares_its_lag(self):
        plan = plan_course(self.course(), provenance={'source_surveys': ['gate.json']})
        with tempfile.TemporaryDirectory() as directory:
            telemetry, course_path = save_example_replay(plan, directory, lag_s=.12)
            self.assertEqual(json.loads(course_path.read_text()), plan.course)
            with telemetry.open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            commands = [row for row in rows if row['event'] == 'command']
            positions = [row for row in rows if row['event'] == 'position']
            times = np.array([float(row['host_monotonic_s']) for row in positions])
            measured = np.array([[float(row[f'{axis}_m']) for axis in 'xyz'] for row in positions])
            np.testing.assert_allclose(measured, plan.trajectory.spline(np.maximum(times - .12, 0)))
            np.testing.assert_allclose(
                [[float(row[f'command_{axis}_m']) for axis in 'xyz'] for row in commands],
                plan.trajectory.spline(times))
            self.assertTrue(all(row['phase'] == 'race' for row in commands))
            source = json.loads((Path(directory) / 'source.json').read_text())
            self.assertEqual(source['lag_s'], .12)
            self.assertIn('synthetic', source['source'])
            self.assertEqual(source['plan_provenance'], plan.provenance)
            self.assertIs(json.loads(course_path.read_text())['enabled_for_flight'], False)

    def test_invalid_example_replay_cannot_replace_existing_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'example'
            plan = plan_course(self.course())
            for lag in (-.1, np.nan):
                with self.subTest(lag=lag), self.assertRaises(ValueError):
                    save_example_replay(plan, path, lag_s=lag)
                self.assertFalse(path.exists())
            save_example_replay(plan, path)
            before = {p.name: p.read_bytes() for p in path.iterdir()}
            plan.course['enabled_for_flight'] = True
            with self.assertRaises(ValueError):
                save_example_replay(plan, path)
            self.assertEqual(before, {p.name: p.read_bytes() for p in path.iterdir()})

    def test_cli_preserves_a_directory_created_after_its_initial_check(self):
        from .plan import main

        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "run"

            def another_run_claims_directory(*args):
                destination.mkdir()
                (destination / "keep.txt").write_text("earlier results")

            argv = ["day3", "--course", str(root / "examples/course.shuttle.synthetic.json"),
                    "--output", str(destination)]
            with patch("sys.argv", argv), patch("sys.stderr"), patch(
                    "day3.plan.plot_trajectory", side_effect=another_run_claims_directory):
                with self.assertRaises(SystemExit) as error:
                    main()
            self.assertEqual(error.exception.code, 1)
            self.assertEqual([path.name for path in destination.iterdir()], ["keep.txt"])
            self.assertEqual((destination / "keep.txt").read_text(), "earlier results")


if __name__ == "__main__":
    unittest.main()
