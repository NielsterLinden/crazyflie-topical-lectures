"""Tests for supplied experiment plumbing; participant policies stay unfinished."""

import contextlib
import csv
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.stats import chi2

from . import exercise_policies
from .experiment_data import (GRAVITY, SCENARIOS, generate_data, sensor_available,
                              load_prepared_data, stationary_features, stationary_motion)
from .experiments import (ExperimentCase, check_policies, lag_correlation, main,
                          export_prepared, make_cases, run_experiment, summarize)
from kalman.filter import FilterConfig, PositionVelocityKalmanFilter


class ExperimentTests(unittest.TestCase):
    def test_repeatability_and_matched_sensor_noise(self):
        nominal = generate_data(7, "nominal")
        repeated = generate_data(7, "nominal")
        for scenario in SCENARIOS:
            data = generate_data(7, scenario)
            np.testing.assert_allclose(data.measured_flow - data.velocity[:, :2],
                                       nominal.measured_flow - nominal.velocity[:, :2], atol=1e-14)
        np.testing.assert_array_equal(nominal.measured_acceleration, repeated.measured_acceleration)
        self.assertFalse(np.array_equal(nominal.measured_position, generate_data(8, "nominal").measured_position))

    def test_separate_fault_scenarios(self):
        nominal = generate_data(3, "nominal")
        outlier = generate_data(3, "outlier")
        changed = np.where(np.any(outlier.measured_position != nominal.measured_position, axis=1))[0]
        np.testing.assert_array_equal(nominal.time[changed], [12.0])
        np.testing.assert_array_equal(outlier.measured_range, nominal.measured_range)
        step = generate_data(3, "range-step")
        selected = (nominal.time >= 8.0) & (nominal.time < 11.0)
        np.testing.assert_allclose((step.measured_range - nominal.measured_range)[selected], -0.35)
        np.testing.assert_array_equal(step.measured_position, nominal.measured_position)

    def test_sensor_scheduling_dropout_and_ablation(self):
        self.assertFalse(sensor_available("position", 400, 8.0, (), 3.0))
        self.assertTrue(sensor_available("flow", 400, 8.0, (), 3.0))
        self.assertFalse(sensor_available("flow", 400, 8.0, ("flow",), 0.0))
        self.assertTrue(sensor_available("position", 550, 11.0, (), 3.0))

    def test_covariance_recipe_scales_only_assumed_variances(self):
        from argparse import Namespace

        args = Namespace(acceleration_std=0.08, r_scale=1.0, study="covariance",
                         scenario="nominal", dropout_seconds=3.0, disable=[])
        cases = make_cases(args)
        self.assertEqual(len(cases), 3)
        for case, multiplier in zip(cases, (1.0, 0.1, 10.0)):
            self.assertAlmostEqual(case.config.position_noise_std**2, 0.04**2 * multiplier)
            self.assertAlmostEqual(case.config.acceleration_noise_std, 0.08)
        args.study = "process"
        baseline, candidate = make_cases(args)
        self.assertEqual(baseline.config.acceleration_noise_std, 1.5)
        self.assertEqual(candidate.config.acceleration_noise_std, 0.08)
        self.assertEqual(baseline.config.position_noise_std, candidate.config.position_noise_std)

    def test_dropouts_do_not_change_actual_observations(self):
        nominal, dropout = generate_data(4, "nominal"), generate_data(4, "dropout")
        for channel in ("measured_acceleration", "measured_position", "measured_flow", "measured_range"):
            np.testing.assert_array_equal(getattr(nominal, channel), getattr(dropout, channel))

    def test_complete_pre_gate_diagnostics_and_whitening(self):
        data = generate_data(2, "nominal", duration_s=0.2)
        case = ExperimentCase("baseline", FilterConfig(), "nominal")
        result = run_experiment(data, case)
        self.assertEqual(len(result["events"]), 12)  # 3 position + 6 flow + 3 range
        self.assertEqual([event["sensor"] for event in result["events"][:3]], ["position", "flow", "range"])
        for event in result["events"]:
            self.assertEqual(event["innovation"].shape, (event["dimension"],))
            np.testing.assert_allclose(event["pull"] @ event["pull"], event["nis"], rtol=1e-12)
            self.assertTrue(event["accepted"])

    def test_participant_gate_is_called_for_each_dimension(self):
        data = generate_data(2, "nominal", duration_s=0.2)
        case = ExperimentCase("gated", FilterConfig(), "nominal", participant_gate=True)
        with patch.object(exercise_policies, "innovation_threshold", side_effect=lambda m, alpha: chi2.ppf(1 - alpha, m)) as policy:
            result = run_experiment(data, case)
        self.assertEqual({call.args[0] for call in policy.call_args_list}, {1, 2, 3})
        for event in result["events"]:
            self.assertAlmostEqual(event["threshold"], chi2.ppf(0.99, event["dimension"]))

    def test_rejected_measurement_remains_in_diagnostic_history(self):
        data = generate_data(2, "nominal", duration_s=0.2)
        data.measured_position[0] = 20.0
        case = ExperimentCase("gated", FilterConfig(nis_gate=0.1), "nominal")
        result = run_experiment(data, case)
        event = result["events"][0]
        self.assertFalse(event["accepted"])
        self.assertEqual(event["sensor"], "position")
        self.assertGreater(event["nis"], event["threshold"])
        np.testing.assert_allclose(event["measurement"] - event["predicted_measurement"], event["innovation"])
        np.testing.assert_allclose(event["pull"] @ event["pull"], event["nis"])

    def test_rejected_updates_leave_state_and_covariance_unchanged(self):
        estimator = PositionVelocityKalmanFilter(FilterConfig(nis_gate=0.01))
        for update, measurement in ((estimator.update_position, np.ones(3) * 10),
                                    (estimator.update_flow_velocity, np.ones(2) * 10),
                                    (estimator.update_range, 10.0)):
            before_x, before_p = estimator.x.copy(), estimator.p.copy()
            self.assertFalse(update(measurement))
            np.testing.assert_array_equal(estimator.x, before_x)
            np.testing.assert_array_equal(estimator.p, before_p)

    def test_full_and_dropout_runs_match_before_dropout(self):
        data = generate_data(4, "nominal")
        full = ExperimentCase("full", FilterConfig(), "nominal")
        dropout = replace(full, name="dropout", dropout_s=3.0)
        first, second = run_experiment(data, full), run_experiment(data, dropout)
        for left, right in zip(first["states"], second["states"]):
            if left["time_s"] < 7.0:
                np.testing.assert_array_equal(left["estimate"], right["estimate"])
        self.assertFalse(any(event["sensor"] == "position" and 7 <= event["time_s"] <= 10
                             for event in second["events"]))

    def test_summary_coverage_and_per_sensor_counts(self):
        data = generate_data(5, "nominal", duration_s=0.2)
        case = ExperimentCase("baseline", FilterConfig(), "nominal")
        result = run_experiment(data, case)
        summary = summarize(result, data, case, 5)
        self.assertEqual(sum(row["updates"] for row in summary["diagnostics"]), len(result["events"]))
        coverage = summary["intervals"][0]["coverage95"]
        self.assertEqual(len(coverage), 6)
        self.assertTrue(all(0 <= value <= 1 for value in coverage))

    def test_all_scenarios_have_finite_outputs(self):
        for scenario in SCENARIOS:
            with self.subTest(scenario=scenario):
                data = generate_data(0, scenario)
                case = ExperimentCase(scenario, FilterConfig(), scenario,
                                      dropout_s=3.0 if scenario == "dropout" else 0.0)
                result = run_experiment(data, case)
                json.dumps(summarize(result, data, case, 0), allow_nan=False)
                self.assertTrue(all(np.all(np.isfinite(row["estimate"])) for row in result["states"]))

    def test_correlation_excludes_dropouts_and_short_series(self):
        self.assertIsNone(lag_correlation(np.array([1.0, 2.0]), 1, np.array([0.0, 0.1]), 0.1))
        self.assertIsNone(lag_correlation(np.array([1., 2., 3., 4.]), 1, np.array([0., 1., 2., 3.]), 0.1))
        self.assertAlmostEqual(lag_correlation(np.arange(10.), 1, np.arange(10.) * 0.1, 0.1), 1.0)

    def test_stationary_motion_is_continuous_at_transitions(self):
        for time in (4., 6., 12., 14.):
            for before, after in zip(stationary_motion(time - 1e-7), stationary_motion(time + 1e-7)):
                np.testing.assert_allclose(before, after, atol=1e-6)
        np.testing.assert_array_equal(stationary_motion(8.)[2], np.zeros(3))
        self.assertGreater(np.linalg.norm(stationary_motion(8.)[1]), 0.0)

    def test_stationarity_features_are_causal_and_need_fresh_motion_cue(self):
        data = generate_data(0, "stationary")
        sample = 100
        window = stationary_features(data, sample, [(2.0, np.array([0.01, 0.0]))])
        self.assertAlmostEqual(window.flow_speed_m_s, 0.01)
        self.assertIsNone(stationary_features(data, sample, [(1.0, np.zeros(2))]).flow_speed_m_s)
        changed = generate_data(0, "stationary")
        changed.specific_force[sample + 1:] = 1000.0
        self.assertEqual(window, stationary_features(changed, sample, [(2.0, np.array([0.01, 0.0]))]))

    def test_stationary_truth_only_scores_the_decision(self):
        data = generate_data(1, "stationary")
        case = ExperimentCase("zupt", FilterConfig(), "stationary", participant_stationary=True)
        with patch.object(exercise_policies, "stationary_update_allowed", return_value=True):
            result = run_experiment(data, case)
        self.assertGreater(summarize(result, data, case, 1)["false_stationary_updates"], 0)
        baseline = run_experiment(data, replace(case, participant_stationary=False))
        self.assertFalse(any(event["sensor"] == "zero_velocity" for event in baseline["events"]))
        data.velocity[:] = 99.0  # Truth labels can change scores, never estimates.
        with patch.object(exercise_policies, "stationary_update_allowed", return_value=True):
            relabelled = run_experiment(data, case)
        np.testing.assert_array_equal([row["estimate"] for row in result["states"]],
                                      [row["estimate"] for row in relabelled["states"]])

    def test_policy_checks_are_separate_and_fail_for_unfinished_starters(self):
        # Deliberately unfinished participant functions are not failing unit tests.
        with patch.object(exercise_policies, "innovation_threshold", return_value=None):
            self.assertTrue(check_policies("E4"))
        with patch.object(exercise_policies, "innovation_threshold", side_effect=lambda m, alpha: chi2.ppf(1 - alpha, m)):
            self.assertEqual(check_policies("E4"), [])
        with patch.object(exercise_policies, "stationary_update_allowed", return_value=False):
            self.assertTrue(check_policies("E5"))

    def test_cli_outputs_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "results"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--study", "single", "--seed-count", "2", "--output", str(output)]), 0)
            settings = json.loads((output / "settings.json").read_text())
            self.assertEqual(settings["sensor_rates_hz"]["flow"], 25)
            self.assertEqual(len(json.loads((output / "summary.json").read_text())), 2)
            aggregate = json.loads((output / "aggregate.json").read_text())
            self.assertTrue(all(row["runs"] == 2 for row in aggregate))
            self.assertIsNotNone(aggregate[0]["position_rmse_m"]["std"])
            self.assertTrue((output / "baseline/seed-001/states.csv").is_file())
            self.assertTrue((output / "comparison.png").stat().st_size > 1000)
            self.assertTrue((output / "baseline/seed-000/diagnostics.png").is_file())
            with (output / "baseline/seed-000/updates.csv").open() as stream:
                row = next(csv.DictReader(stream))
                self.assertIn("S_22", row)
                self.assertIn("measurement_0", row)
            before = (output / "settings.json").read_bytes()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(["--output", str(output)])
            self.assertEqual(before, (output / "settings.json").read_bytes())

    def test_recording_uses_only_available_position_without_claiming_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = ExperimentCase("baseline", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, root, 0)
            data = load_prepared_data(root / "prepared.csv", root / "recording.json", "nominal")
            self.assertFalse(data.has_truth)
            result = run_experiment(data, case)
            np.testing.assert_array_equal(result["states"][0]["estimate"][:3], data.measured_position[0])
            self.assertEqual({event["sensor"] for event in result["events"]}, {"position"})
            self.assertTrue(all(event["time_s"] > 0 for event in result["events"]))
            summary = summarize(result, data, case, None)
            self.assertTrue(all(row["position_rmse_m"] is None and row["coverage95"] == [None] * 6 for row in summary["intervals"]))
            self.assertTrue(all(row["pull_lag1"] is None for row in summary["diagnostics"]))
            json.dumps(summary, allow_nan=False)

    def test_multi_seed_export_matches_first_seed_and_metadata(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "ensemble"
            with contextlib.redirect_stdout(io.StringIO()), \
                    patch("day2.experiments.plot_diagnostics"), \
                    patch("day2.experiments.plot_stationarity"), \
                    patch("day2.experiments.plot_comparison"):
                self.assertEqual(main(["--seed", "20", "--seed-count", "2",
                                       "--export-prepared", "--output", str(output)]), 0)
            metadata = json.loads((output / "recording.json").read_text())
            self.assertEqual(metadata["seed"], 20)
            data = load_prepared_data(output / "prepared.csv", output / "recording.json", "nominal")
            expected = generate_data(20, "nominal")
            np.testing.assert_array_equal(data.measured_acceleration, expected.measured_acceleration)
            observed = np.all(np.isfinite(data.measured_position), axis=1)
            np.testing.assert_array_equal(data.measured_position[observed], expected.measured_position[observed])

    def test_recording_waits_for_position_without_retiming_or_reusing_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = ExperimentCase("baseline", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, root, 0)
            recording = root / "prepared.csv"
            with recording.open(newline="") as stream:
                rows = list(csv.reader(stream))
            rows[1][4:] = ["", "", ""]
            with recording.open("w", newline="") as stream:
                csv.writer(stream).writerows(rows)
            original = recording.read_bytes()
            data = load_prepared_data(recording, root / "recording.json", "nominal")
            self.assertAlmostEqual(data.time[0], 0.1)
            self.assertEqual(data.metadata["skipped_initial_samples"], 5)
            self.assertAlmostEqual(data.metadata["initialization_time_s"], 0.1)
            result = run_experiment(data, case)
            np.testing.assert_array_equal(result["states"][0]["estimate"][:3], data.measured_position[0])
            self.assertTrue(all(event["time_s"] > data.time[0] for event in result["events"]))
            self.assertTrue(all(np.all(np.isfinite(row["estimate"])) for row in result["states"]))
            self.assertEqual(recording.read_bytes(), original)
            for row in rows[1:]:
                row[4:] = ["", "", ""]
            with recording.open("w", newline="") as stream:
                csv.writer(stream).writerows(rows)
            with self.assertRaisesRegex(ValueError, "needs a position observation"):
                load_prepared_data(recording, root / "recording.json", "nominal")

    def test_recording_fault_injection_preserves_original_and_does_not_resample(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = ExperimentCase("baseline", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, root, 0)
            original = (root / "prepared.csv").read_bytes()
            clean = load_prepared_data(root / "prepared.csv", root / "recording.json", "nominal")
            fault = load_prepared_data(root / "prepared.csv", root / "recording.json", "outlier")
            np.testing.assert_array_equal(clean.time, fault.time)
            np.testing.assert_array_equal(clean.measured_acceleration, fault.measured_acceleration)
            changed = np.flatnonzero(np.nan_to_num(fault.measured_position[:, 1] - clean.measured_position[:, 1]))
            self.assertEqual(len(changed), 1)
            self.assertAlmostEqual(fault.measured_position[changed[0], 1] - clean.measured_position[changed[0], 1], 0.65)
            self.assertEqual(original, (root / "prepared.csv").read_bytes())
            with self.assertRaisesRegex(ValueError, "support"):
                load_prepared_data(root / "prepared.csv", root / "recording.json", "range-step")

    def test_recording_cli_saves_missing_metrics_as_blank_and_rejects_seed_ensembles(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            export_prepared(generate_data(0, "nominal"), ExperimentCase("baseline", FilterConfig(), "nominal"), root, 0)
            args = ["--recording", str(root / "prepared.csv"), "--metadata", str(root / "recording.json"),
                    "--study", "process", "--acceleration-std", "0.08", "--output", str(root / "analysis")]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            settings = json.loads((root / "analysis/settings.json").read_text())
            self.assertIsNone(settings["actual_noise_std"])
            self.assertEqual(settings["seeds"], [])
            with (root / "analysis/metrics.csv").open() as stream:
                self.assertTrue(all(row["position_rmse_m"] == "" and row["coverage95_px"] == "" for row in csv.DictReader(stream)))
            self.assertTrue((root / "analysis/Q-candidate/recording/diagnostics.png").is_file())
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args + ["--seed-count", "2"])


if __name__ == "__main__":
    unittest.main()
