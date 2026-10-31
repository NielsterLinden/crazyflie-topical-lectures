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
                          export_prepared, make_cases, plot_diagnostics, run_experiment, summarize,
                          interval_labels, validate_position_dropout)
from kalman.filter import FilterConfig, PositionVelocityKalmanFilter
from kalman.recording import load_recording, replay


class ExperimentTests(unittest.TestCase):
    def test_outlier_after_late_initialization_is_gated_not_used_as_a_prior(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            recording, metadata = root / "prepared.csv", root / "recording.json"
            example = Path(__file__).resolve().parents[2] / "examples/replay.synthetic.json"
            metadata.write_text(example.read_text())
            for further_positions in (True, False):
                with recording.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["time_s", "ax_world_m_s2", "ay_world_m_s2",
                                     "az_world_m_s2", "px_m", "py_m", "pz_m"])
                    for index in range(125):
                        observed = index == 120 or (further_positions and index > 120)
                        position = [0, 0, 1] if observed else ["", "", ""]
                        writer.writerow([index / 10, 0, 0, 0, *position])
                original = recording.read_bytes()
                if not further_positions:
                    with self.assertRaisesRegex(ValueError, "after initialization"):
                        load_prepared_data(recording, metadata, "outlier")
                    continue
                data = load_prepared_data(recording, metadata, "outlier")
                np.testing.assert_array_equal(data.measured_position[0], [0, 0, 1])
                self.assertEqual(data.metadata["interventions"][0]["time_s"], 12.1)
                case = ExperimentCase("gated", FilterConfig(chi2_gate=chi2.ppf(.99, 3)), "outlier")
                result = run_experiment(data, case)
                fault = result["events"][0]
                self.assertEqual(fault["time_s"], 12.1)
                self.assertFalse(fault["accepted"])
                self.assertAlmostEqual(fault["predicted_residual"][1], .65)
                self.assertTrue(all(event["accepted"] for event in result["events"][1:]))
                self.assertEqual(recording.read_bytes(), original)

    def test_nonzero_recording_clock_is_shared_by_experiments_and_replay(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = ExperimentCase("baseline", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, root, 0)
            recording, metadata = root / "prepared.csv", root / "recording.json"
            with recording.open(newline="") as stream:
                rows = list(csv.reader(stream))
            for row in rows[1:]:
                row[0] = str(float(row[0]) + 100.)
            # Retain leading rows with no position, and their clock origin.
            rows[1][4:] = ["", "", ""]
            with recording.open("w", newline="") as stream:
                csv.writer(stream).writerows(rows)
            original = recording.read_bytes()
            data = load_prepared_data(recording, metadata, "nominal")
            samples, _ = load_recording(recording, metadata)
            replay_rows = replay(samples, FilterConfig())
            self.assertAlmostEqual(data.time[0], 100.1)
            self.assertAlmostEqual(data.metadata["initialization_time_s"], 100.1)
            self.assertEqual(data.metadata["analysis_time_origin_s"], 100.)
            np.testing.assert_allclose(data.time, [row["time_s"] for row in replay_rows][5:])
            np.testing.assert_allclose(data.elapsed_time, data.time - 100.)
            gap = replace(case, dropout_s=3.)
            result = run_experiment(data, gap)
            self.assertTrue(any(event["time_s"] < 107. for event in result["events"]))
            self.assertFalse(any(107. <= event["time_s"] <= 110. for event in result["events"]))
            self.assertTrue(any(110. < event["time_s"] <= 112. for event in result["events"]))
            labels = interval_labels(data, gap)
            self.assertTrue(np.all(labels[data.time < 102.] == "startup"))
            self.assertTrue(np.all(labels[(data.time >= 107.) & (data.time <= 110.)] == "dropout"))
            fault = load_prepared_data(recording, metadata, "outlier")
            index = np.flatnonzero(np.any(np.nan_to_num(fault.measured_position - data.measured_position), axis=1))
            self.assertEqual(len(index), 1)
            self.assertAlmostEqual(fault.time[index[0]], 112.)
            self.assertEqual(fault.metadata["interventions"][0]["time_s"], 112.)
            self.assertEqual(interval_labels(fault, case)[index[0]], "outlier_recovery")
            self.assertEqual(recording.read_bytes(), original)

    def test_dropout_rejects_short_recording_and_missing_observation_phases(self):
        case = ExperimentCase("dropout", FilterConfig(), "dropout", dropout_s=3)
        short = replace(generate_data(0, "nominal", duration_s=5), has_truth=False)
        with self.assertRaisesRegex(ValueError, "through 12 s"):
            run_experiment(short, case)
        for name in ("before", "during", "after"):
            data = replace(generate_data(0, "nominal"), has_truth=False)
            windows = {"before": data.time < 7,
                       "during": (data.time >= 7) & (data.time <= 10),
                       "after": (data.time > 10) & (data.time <= 12)}
            data.measured_position[windows[name]] = np.nan
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, name):
                validate_position_dropout(data, case)

    def test_recorded_dropout_removes_updates_and_retains_recovery(self):
        data = replace(generate_data(0, "nominal"), has_truth=False)
        full = ExperimentCase("complete", FilterConfig(), "nominal")
        gap = replace(full, name="gap", dropout_s=3)
        original_positions = data.measured_position.copy()
        complete = run_experiment(data, full)
        interrupted = run_experiment(data, gap)
        removed = [event for event in complete["events"] if 7 <= event["time_s"] <= 10]
        self.assertGreater(len(removed), 0)
        self.assertEqual(len(complete["events"]) - len(interrupted["events"]), len(removed))
        self.assertTrue(any(10 < event["time_s"] <= 12 for event in interrupted["events"]))
        np.testing.assert_array_equal(data.measured_position, original_positions)

    def test_cli_rejects_ineffective_dropout_before_writing_results(self):
        short = replace(generate_data(0, "nominal", duration_s=5), has_truth=False)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            with patch("day2.experiments.load_prepared_data", return_value=short), \
                 contextlib.redirect_stderr(io.StringIO()) as errors:
                with self.assertRaises(SystemExit):
                    main(["--study", "dropout", "--recording", "input.csv",
                          "--metadata", "input.json", "--output", str(output)])
            self.assertIn("loss and recovery", errors.getvalue())
            self.assertFalse(output.exists())

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
            self.assertEqual(event["predicted_residual"].shape, (event["dimension"],))
            self.assertEqual(event["predicted_residual_covariance"].shape,
                             (event["dimension"], event["dimension"]))
            np.testing.assert_allclose(event["pull"] @ event["pull"], event["chi2"], rtol=1e-12)
            self.assertTrue(event["accepted"])

    def test_diagnostic_pull_histograms_show_moments_and_normal_reference(self):
        import matplotlib.pyplot as plt

        pulls = np.array([[0.0, 1.0, -2.0], [1.0, 2.0, 0.0], [2.0, 3.0, 2.0]])
        events = [
            dict(sensor="position", time_s=float(index), chi2=float(pull @ pull),
                 accepted=True, threshold=7.815, dimension=3, pull=pull)
            for index, pull in enumerate(pulls)
        ]
        result = {"events": events}

        with tempfile.TemporaryDirectory() as temporary, \
                patch("matplotlib.pyplot.close"):
            destination = Path(temporary) / "diagnostics.png"
            plot_diagnostics(result, destination, alpha=0.05, regular_timing=False)
            figure = plt.gcf()
            histogram_axis = figure.axes[1]
            labels = histogram_axis.get_legend_handles_labels()[1]
            self.assertTrue(destination.is_file())
            self.assertIn("component 0: mean=1.00, var=1.00", labels)
            self.assertIn("component 1: mean=2.00, var=1.00", labels)
            self.assertIn("component 2: mean=0.00, var=4.00", labels)
            self.assertIn("N(0, 1)", labels)
            reference = next(line for line in histogram_axis.lines
                             if line.get_label() == "N(0, 1)")
            np.testing.assert_allclose(
                reference.get_ydata(),
                np.exp(-0.5 * reference.get_xdata()**2) / np.sqrt(2 * np.pi),
            )
            plt.close("all")

    def test_participant_gate_is_called_for_each_dimension(self):
        data = generate_data(2, "nominal", duration_s=0.2)
        case = ExperimentCase("gated", FilterConfig(), "nominal", participant_gate=True)
        with patch.object(exercise_policies, "predicted_residual_threshold", side_effect=lambda m, alpha: chi2.ppf(1 - alpha, m)) as policy:
            result = run_experiment(data, case)
        self.assertEqual({call.args[0] for call in policy.call_args_list}, {1, 2, 3})
        for event in result["events"]:
            self.assertAlmostEqual(event["threshold"], chi2.ppf(0.99, event["dimension"]))

    def test_rejected_measurement_remains_in_diagnostic_history(self):
        data = generate_data(2, "nominal", duration_s=0.2)
        data.measured_position[0] = 20.0
        case = ExperimentCase("gated", FilterConfig(chi2_gate=0.1), "nominal")
        result = run_experiment(data, case)
        event = result["events"][0]
        self.assertFalse(event["accepted"])
        self.assertEqual(event["sensor"], "position")
        self.assertGreater(event["chi2"], event["threshold"])
        np.testing.assert_allclose(event["measurement"] - event["predicted_measurement"], event["predicted_residual"])
        np.testing.assert_allclose(event["pull"] @ event["pull"], event["chi2"])

    def test_rejected_updates_leave_state_and_covariance_unchanged(self):
        estimator = PositionVelocityKalmanFilter(FilterConfig(chi2_gate=0.01))
        for update, measurement in ((estimator.update_position, np.ones(3) * 10),
                                    (estimator.update_flow_velocity, np.ones(2) * 10),
                                    (estimator.update_range, 10.0)):
            before_x, before_p = estimator.x.copy(), estimator.C.copy()
            self.assertFalse(update(measurement))
            np.testing.assert_array_equal(estimator.x, before_x)
            np.testing.assert_array_equal(estimator.C, before_p)

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
        with patch.object(exercise_policies, "predicted_residual_threshold", return_value=None):
            self.assertTrue(check_policies("E4"))
        with patch.object(exercise_policies, "predicted_residual_threshold", side_effect=lambda m, alpha: chi2.ppf(1 - alpha, m)):
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
                self.assertIn("R_predicted_22", row)
                covariance_columns = [name for name in row if name.startswith("R_predicted_")]
                self.assertEqual(len(covariance_columns), 9)
                guide = Path(__file__).with_name("six_state_reference.md").read_text(encoding="utf-8")
                for name in (covariance_columns[0], covariance_columns[-1]):
                    self.assertIn(f"`{name}`", guide)
                self.assertIn("measurement_0", row)
                residual = np.array([float(row[f"predicted_residual_{axis}"]) for axis in range(3)])
                R = np.array([[float(row[f"R_predicted_{axis}{column}"])
                               for column in range(3)] for axis in range(3)])
                self.assertAlmostEqual(float(row["chi2"]), float(residual @ np.linalg.solve(R, residual)))
                old_term = "inno" + "vation"
                self.assertFalse(any(old_term in name for name in row))
            before = (output / "settings.json").read_bytes()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(["--output", str(output)])
            self.assertEqual(before, (output / "settings.json").read_bytes())

    def test_documented_dropout_constraints_match_both_data_routes(self):
        data = generate_data(0, "nominal", duration_s=12.2)
        np.testing.assert_allclose(np.diff(data.time), .02)
        for sensor, expected_rate in (("position", 10), ("flow", 25), ("range", 10)):
            times = [time for sample, time in enumerate(data.time)
                     if sensor_available(sensor, sample, float(time), (), 0)]
            np.testing.assert_allclose(np.diff(times), 1 / expected_rate)
        case = ExperimentCase("gap", FilterConfig(), "dropout", dropout_s=3.)
        simulated = run_experiment(data, case)
        sensors_in_gap = {event["sensor"] for event in simulated["events"]
                          if 7.1 < event["time_s"] < 9.9}
        self.assertEqual(sensors_in_gap, {"flow", "range"})
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            export_prepared(data, ExperimentCase("input", FilterConfig(), "nominal"), output, 0)
            recorded_data = load_prepared_data(output / "prepared.csv", output / "recording.json", "nominal")
            recorded = run_experiment(recorded_data, case)
        self.assertEqual({event["sensor"] for event in recorded["events"]}, {"position"})
        self.assertFalse(any(7.1 < event["time_s"] < 9.9 for event in recorded["events"]))

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
