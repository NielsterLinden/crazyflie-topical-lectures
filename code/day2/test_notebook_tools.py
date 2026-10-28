"""Notebook comparisons reuse the same filter and preserve participant decisions."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import subprocess
import sys
import io
from contextlib import redirect_stdout

import numpy as np

from .experiment_data import generate_data
from .experiments import ExperimentCase, export_prepared, run_experiment
from .notebook_tools import _run_capture, capture, compare, next_attempt, select_data
from kalman.filter import FilterConfig


class NotebookToolsTests(unittest.TestCase):
    def test_unselected_data_writes_no_comparison(self):
        cases = [ExperimentCase("baseline", FilterConfig(), "nominal")]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            with self.assertRaisesRegex(ValueError, "data-selection cell"):
                compare(None, cases, output)
            self.assertFalse(output.exists())

    def test_invalid_dropout_comparison_writes_no_baseline_or_plot(self):
        data = generate_data(0, "nominal", duration_s=5)
        cases = [ExperimentCase("complete", FilterConfig(), "nominal"),
                 ExperimentCase("gap", FilterConfig(), "dropout", dropout_s=3)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            with self.assertRaisesRegex(ValueError, "synthetic fallback"):
                compare(data, cases, output)
            self.assertFalse(output.exists())

    def test_repeat_attempts_preserve_results_and_allow_retry_after_policy_error(self):
        data = generate_data(0, "nominal", duration_s=1)
        case = ExperimentCase("baseline", FilterConfig(), "nominal", participant_gate=True)
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory) / "E4"
            first = next_attempt(parent)
            compare(data, [case], first, innovation_policy=lambda dimension, alpha: None)
            original = (first / "baseline/updates.csv").read_bytes()
            second = next_attempt(parent)
            compare(data, [case], second, innovation_policy=lambda dimension, alpha: 1.0)
            self.assertNotEqual(first, second)
            self.assertEqual((first / "baseline/updates.csv").read_bytes(), original)
            failed = next_attempt(parent)
            with self.assertRaises(ValueError):
                compare(data, [case], failed, innovation_policy=lambda dimension, alpha: -1)
            retry = next_attempt(parent)
            self.assertNotEqual(failed, retry)
            compare(data, [case], retry, innovation_policy=lambda dimension, alpha: None)
            self.assertEqual(first.name, "attempt-001")
            self.assertEqual(retry.name, "attempt-004")

    def test_capture_checks_before_starting_radio_process(self):
        with tempfile.TemporaryDirectory() as directory, patch("day2.notebook_tools._run_capture") as run:
            output = Path(directory) / "capture"
            for kwargs in ({}, {"ready_for_motor_stop": True, "duration_s": 121},
                           {"ready_for_motor_stop": True, "duration_s": float("nan")},
                           {"ready_for_motor_stop": True, "position_std": -1}):
                with self.assertRaises(ValueError):
                    capture(output, "radio://0/80/2M/E7E7E7E7E7", **kwargs)
            with self.assertRaises(ValueError):
                capture(output, "", ready_for_motor_stop=True)
            with self.assertRaises(FileExistsError):
                capture(directory, "test://no-radio", ready_for_motor_stop=True)
            run.assert_not_called()
            self.assertFalse(output.exists())

    def test_capture_is_bounded_and_failed_inputs_are_not_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failed"
            for error in (subprocess.TimeoutExpired(["logger"], 31), KeyboardInterrupt()):
                with patch("day2.notebook_tools._run_capture", side_effect=error) as run:
                    with self.assertRaises(TimeoutError if isinstance(error, subprocess.TimeoutExpired)
                                           else KeyboardInterrupt):
                        capture(output, "test://no-radio", duration_s=1, ready_for_motor_stop=True)
                    self.assertEqual(run.call_args.kwargs["timeout_s"], 31)
            with patch("day2.notebook_tools._run_capture", return_value=
                       subprocess.CompletedProcess([], 1, "", "motor outputs are nonzero")):
                with self.assertRaisesRegex(RuntimeError, "motor outputs are nonzero"):
                    capture(output, "test://no-radio", ready_for_motor_stop=True)

    def test_capture_returns_only_complete_validated_recordings_without_propeller_confirmation(self):
        from .live import CHANNELS, LiveSession

        status = "complete"

        def logger(command, **kwargs):
            self.assertEqual(command[1:4], ["-u", "-m", "day2.live"])
            self.assertIn("--prepare-for-handling", command)
            self.assertIn("--no-show", command)
            self.assertEqual(kwargs["env"]["MPLBACKEND"], "Agg")
            self.assertEqual(kwargs["timeout_s"], 90)
            session = LiveSession(command[command.index("--output") + 1],
                                  [FilterConfig(), FilterConfig()], [0, 0, 0], "test://no-radio")
            for timestamp in (100, 150, 200):
                for stream, values in (("motors", [0, 0, 0, 0]),
                                       ("attitude", [0, 0, 0, 1]),
                                       ("imu_position", [0, 0, 1, 0, 0, 1])):
                    session.enqueue(stream, timestamp, dict(zip(CHANNELS[stream], values)), 0)
            session.drain(1, force=True)
            session.finish(status)
            return subprocess.CompletedProcess(command, 0, "Saved test recording\n", "")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            with patch("day2.notebook_tools._run_capture", side_effect=logger):
                recording, metadata = capture(output, "test://no-radio", ready_for_motor_stop=True)
            self.assertEqual(recording, output.resolve() / "prepared.csv")
            self.assertEqual(json.loads(metadata.read_text())["prepared_samples"], 3)
            data = select_data(recording, metadata)
            self.assertFalse(data.has_truth)
            # Success exit alone is insufficient if the logger says interrupted.
            status = "interrupted"
            with patch("day2.notebook_tools._run_capture", side_effect=logger):
                with self.assertRaisesRegex(RuntimeError, "did not complete"):
                    capture(Path(directory) / "interrupted", "test://no-radio", ready_for_motor_stop=True)

    def test_progress_is_relayed_and_timeout_kills_child(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()) as output:
            command = [sys.executable, "-u", "-c", "print('STOP VERIFIED', flush=True)"]
            result = _run_capture(command, cwd=directory, env=None, timeout_s=5)
            self.assertEqual(result.returncode, 0)
            self.assertIn("STOP VERIFIED", output.getvalue())
            child = subprocess.Popen
            processes = []

            def start(*args, **kwargs):
                process = child(*args, **kwargs)
                processes.append(process)
                return process

            with patch("day2.notebook_tools.subprocess.Popen", side_effect=start):
                with self.assertRaises(subprocess.TimeoutExpired):
                    _run_capture([sys.executable, "-c", "import time; time.sleep(10)"],
                                 cwd=directory, env=None, timeout_s=0.1)
            self.assertIsNotNone(processes[0].poll())

    def test_selected_recordings_do_not_silently_become_simulation(self):
        with self.assertRaises(ValueError):
            select_data("missing.csv", None)
        with self.assertRaises(OSError):
            select_data("missing.csv", "missing.json")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            case = ExperimentCase("source", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, path, 0)
            data = select_data(path / "prepared.csv", path / "recording.json")
            self.assertFalse(data.has_truth)
            self.assertTrue(data.metadata["synthetic"])
            self.assertTrue(np.all(np.isnan(data.measured_flow)))

    def test_notebook_gate_changes_results_without_modifying_global_policies(self):
        from . import exercise_policies

        original = exercise_policies.innovation_threshold
        data = generate_data(0, "outlier")
        case = ExperimentCase("gate", FilterConfig(), "outlier", participant_gate=True)
        baseline = run_experiment(data, case)
        dimensions = set()

        def threshold(dimension, alpha):
            dimensions.add(dimension)
            return 1.0

        changed = run_experiment(data, case, innovation_policy=threshold)
        self.assertEqual(dimensions, {1, 2, 3})
        self.assertTrue(all(event["accepted"] for event in baseline["events"]))
        self.assertTrue(any(not event["accepted"] for event in changed["events"]))
        self.assertIs(exercise_policies.innovation_threshold, original)

    def test_notebook_stationary_decision_is_used(self):
        data = generate_data(0, "stationary", duration_s=2.0)
        case = ExperimentCase("zupt", FilterConfig(), "stationary", participant_stationary=True)
        no_updates = run_experiment(data, case)
        # Deliberately permissive for this wiring test, not an exercise solution.
        changed = run_experiment(data, case, stationary_policy=lambda window: True)
        self.assertFalse(any(event["sensor"] == "zero_velocity" for event in no_updates["events"]))
        self.assertTrue(any(event["sensor"] == "zero_velocity" for event in changed["events"]))

    def test_saved_results_are_labelled_and_existing_outputs_are_preserved(self):
        data = select_data(seed=0)
        case = ExperimentCase("baseline", FilterConfig(), "nominal")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            summary = compare(data, [case], output, notes="Prediction: compare the bands.")
            self.assertTrue(summary[0]["has_truth"])
            settings = json.loads((output / "settings.json").read_text())
            self.assertTrue(settings["source"]["synthetic"])
            self.assertTrue((output / "baseline/states.csv").is_file())
            self.assertTrue((output / "baseline/updates.csv").is_file())
            self.assertTrue((output / "comparison.png").is_file())
            with self.assertRaises(FileExistsError):
                compare(data, [case], output)


if __name__ == "__main__":
    unittest.main()
