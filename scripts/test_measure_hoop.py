"""Offline checks for the supplied survey tool; no radio connection is made."""

import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock
from types import ModuleType, SimpleNamespace

import numpy as np

import measure_hoop


class SurveyToolTests(unittest.TestCase):
    def sample(self, sampler, timestamp, x=0):
        with patch.object(measure_hoop.time, "monotonic", return_value=timestamp):
            sampler.callback(0, {"stateEstimate.x": x, "stateEstimate.y": 0,
                                 "stateEstimate.z": 1}, None)

    def test_full_trace_boundaries_and_resumed_gap_are_checked(self):
        sampler = measure_hoop.PositionSampler()
        self.sample(sampler, 0)
        with patch.object(measure_hoop.time, "monotonic", return_value=.01):
            start = sampler.begin_trace()
        for index in range(1, 9):
            self.sample(sampler, index * .05, x=index)
        # Enough geometric points exist, but a stalled tail invalidates the trace.
        reason = sampler.finish_trace(start, 1.1)
        self.assertIn("incomplete", reason)
        with tempfile.TemporaryDirectory() as directory:
            path = sampler.save_failed_trace(Path(directory), start, 1.1, "circle", reason)
            record = json.loads(path.read_text())
            self.assertEqual(record["status"], "failed")
            self.assertEqual(len(record["samples"]), 8)
            self.assertNotIn("centre", record)
            self.assertNotIn("radius_m", record)
        sampler = measure_hoop.PositionSampler()
        self.sample(sampler, 0)
        with patch.object(measure_hoop.time, "monotonic", return_value=.01):
            start = sampler.begin_trace()
        self.sample(sampler, .1)
        self.sample(sampler, .8)
        self.assertIn("gap", sampler.finish_trace(start, .9))

    def test_watchdog_and_callback_errors_reject_traces(self):
        for reason in ("connection lost", "logging error"):
            sampler = measure_hoop.PositionSampler()
            self.sample(sampler, 0)
            with patch.object(measure_hoop.time, "monotonic", return_value=.01):
                start = sampler.begin_trace()
            sampler.fail(reason)
            self.assertEqual(sampler.finish_trace(start, .1), reason)
        sampler = measure_hoop.PositionSampler()
        self.sample(sampler, 0)
        finished = MagicMock()
        finished.wait.return_value = False
        with patch.object(measure_hoop.time, "monotonic", return_value=1):
            sampler.monitor(finished)
        with self.assertRaisesRegex(RuntimeError, "updates stopped"):
            sampler.begin_trace()

    def test_healthy_trace_is_accepted_and_later_samples_are_excluded(self):
        sampler = measure_hoop.PositionSampler()
        self.sample(sampler, 0)
        with patch.object(measure_hoop.time, "monotonic", return_value=.01):
            start = sampler.begin_trace()
        for index in range(1, 9):
            self.sample(sampler, index * .05, x=index)
        self.assertIsNone(sampler.finish_trace(start, .45))
        self.sample(sampler, .6, x=100)
        self.assertEqual(len(sampler.samples_since(start, .45)), 8)
        invalid = measure_hoop.PositionSampler()
        self.sample(invalid, 0, x=float("nan"))
        with self.assertRaisesRegex(RuntimeError, "invalid position"):
            invalid.begin_trace()

    def test_bad_trace_is_saved_and_never_offered_for_fitting(self):
        for interrupted in (False, True):
            sampler = measure_hoop.PositionSampler()
            self.sample(sampler, measure_hoop.time.monotonic())
            prompts = []

            def respond(prompt):
                prompts.append(prompt)
                if prompt.startswith("Command"):
                    return ""
                if prompt.startswith("Recording"):
                    self.sample(sampler, measure_hoop.time.monotonic(), x=1)
                    if interrupted:
                        raise KeyboardInterrupt
                    sampler.fail("connection lost during trace")
                    return ""
                self.fail("A failed trace must not be offered for fitting")

            with tempfile.TemporaryDirectory() as directory, patch("builtins.input", side_effect=respond):
                with self.assertRaises(KeyboardInterrupt if interrupted else RuntimeError):
                    measure_hoop._interactive_survey(sampler, "circle", .02, Path(directory))
                records = list(Path(directory).glob("failed-*.json"))
                self.assertEqual(len(records), 1)
                self.assertEqual(json.loads(records[0].read_text())["status"], "failed")
            self.assertFalse(any(prompt.startswith("Fit") for prompt in prompts))

    def test_survey_requires_verified_stop_before_position_logging(self):
        modules = {name: ModuleType(name) for name in (
            "cflib", "cflib.crtp", "cflib.crazyflie", "cflib.crazyflie.log",
            "cflib.crazyflie.syncCrazyflie")}
        modules["cflib"].crtp = modules["cflib.crtp"]
        modules["cflib.crtp"].init_drivers = MagicMock()
        modules["cflib.crazyflie"].Crazyflie = MagicMock()
        config = MagicMock()
        modules["cflib.crazyflie.log"].LogConfig = MagicMock(return_value=config)
        connection = MagicMock()
        modules["cflib.crazyflie.syncCrazyflie"].SyncCrazyflie = MagicMock(return_value=connection)
        args = SimpleNamespace(uri="test://no-radio", log_period_ms=50,
                               shape="circle", sample_spacing=.02)
        with patch.dict(sys.modules, modules), patch.object(measure_hoop, "parse_args", return_value=args), \
             patch("builtins.input", return_value="STOP AND SURVEY"), patch("builtins.print"), \
             patch('notebook_radio.crazyflie.init'), \
             patch('notebook_radio.engage_motor_stop', side_effect=TimeoutError) as stop:
            with self.assertRaises(TimeoutError):
                measure_hoop.main()
            connection.wait_for_params.assert_called_once()
            stop.assert_called_once_with(connection.cf)
            config.start.assert_not_called()

    def test_assigned_uri_is_required_and_help_is_radio_free(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch.object(sys, 'argv', ['measure_hoop.py']), patch('sys.stderr'):
                with self.assertRaises(SystemExit) as error:
                    measure_hoop.parse_args()
                self.assertEqual(error.exception.code, 2)
            with patch.object(sys, 'argv', ['measure_hoop.py', '--help']), patch('sys.stdout'):
                with self.assertRaises(SystemExit) as result:
                    measure_hoop.parse_args()
                self.assertEqual(result.exception.code, 0)
        uri = 'radio://0/80/2M/E7E7E7E701'
        with patch.dict(os.environ, {'CF_URI': uri}):
            with patch.object(sys, 'argv', ['measure_hoop.py']):
                self.assertEqual(measure_hoop.parse_args().uri, uri)

    def test_circle_json_retains_measurements_and_does_not_overwrite(self):
        points = np.array([[0, 1, 1], [0, 0, 2], [0, -1, 1], [0, 0, 0]])
        fit = measure_hoop.fit_circle_3d(points, np.array([1, 0, 0]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'survey.json'
            measure_hoop._write_result(path, 'gate-1', fit, list(points), False)
            result = json.loads(path.read_text())
            self.assertEqual(result['name'], 'gate-1')
            self.assertEqual(result['survey']['point_count'], 4)
            self.assertEqual(result['survey']['points_m'], points.tolist())
            np.testing.assert_allclose(result['normal'], [1, 0, 0])
            with self.assertRaises(FileExistsError):
                measure_hoop._write_result(path, 'gate-1', fit, list(points), False)

    def test_rectangle_json_preserves_shape_and_inscribed_aperture(self):
        points = np.array([[-2, -1, 0], [2, -1, 0], [2, 1, 0], [-2, 1, 0]])
        fit = measure_hoop.fit_rectangle_3d(points, np.array([0, 0, 1]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'rectangle.json'
            measure_hoop._write_rectangle_result(path, 'gate-2', fit, list(points), False)
            result = json.loads(path.read_text())
            self.assertEqual(result['shape'], 'rectangle')
            self.assertAlmostEqual(result['radius_m'], 1)
            self.assertAlmostEqual(result['radius_m'], min(result['width_m'], result['height_m']) / 2)
            self.assertIn('width_axis', result)
            self.assertIn('height_axis', result)

    def test_script_has_no_arming_or_setpoint_path(self):
        tree = ast.parse(Path(measure_hoop.__file__).read_text())
        names = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertFalse(names.intersection({
            'commander', 'high_level_commander', 'send_arming_request',
            'send_setpoint', 'set_value'}))


if __name__ == '__main__':
    unittest.main()
