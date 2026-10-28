"""Checks for the reused controller classes and motor-free Day 1 experiments."""

import ast
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

import numpy as np

from . import hover_pid, practical, velocity_pid


class ControllerTests(unittest.TestCase):
    def test_signs_zero_error_reset_and_limits(self):
        for direction in (-1, 1):
            position = hover_pid.AxisPID(1.2, 0.6, ki=0.4)
            command, terms = position.update(0, direction, 0, 0.02)
            self.assertLess(command * direction, 0)
            self.assertLessEqual(abs(command), position.v_max)
            position.reset()
            self.assertEqual(position.update(0, 0, 0, 0.02)[0], 0)
            velocity = velocity_pid.AxisVelocityPID(12, kvi=1)
            tilt, _ = velocity.update(0, direction, 0.02)
            self.assertLess(tilt * direction, 0)
            self.assertLessEqual(abs(tilt), velocity.tilt_max)
            velocity.reset()
            self.assertEqual(velocity.update(0, 0, 0.02)[0], 0)

    def test_integral_does_not_jump_outside_its_bound_after_clipping_and_rejection(self):
        for sign in (-1, 1):
            for controller in (hover_pid.AxisPID(10, 0, ki=1, i_max=0.15),
                               velocity_pid.AxisVelocityPID(100, kvi=1, i_max=0.15)):
                with self.subTest(sign=sign, controller=type(controller).__name__):
                    for sample in range(100):
                        if isinstance(controller, hover_pid.AxisPID):
                            command, terms = controller.update(sign * 10, 0, 0, 1)
                        else:
                            command, terms = controller.update(sign * 10, 0, 1)
                        self.assertLessEqual(abs(terms['i']), 0.15)
                        self.assertEqual(controller._i, 0)

    def test_invalid_update_intervals_are_rejected(self):
        for dt in (0, -1, float('nan')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                hover_pid.AxisPID(1, 0).update(0, 0, 0, dt)
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                velocity_pid.AxisVelocityPID(12).update(0, 0, dt)

    def test_cascade_reuses_the_position_and_velocity_classes(self):
        controller = velocity_pid.CascadePID(dict(kp=1.2, kd=0.6), dict(kv=12))
        self.assertIsInstance(controller.position.x, hover_pid.AxisPID)
        self.assertIsInstance(controller.velocity.x, velocity_pid.AxisVelocityPID)
        tilt, terms = controller.update((0, 0), (0.1, -0.1), (0, 0), 0.02)
        self.assertLess(tilt[0], 0)
        self.assertGreater(tilt[1], 0)
        self.assertLess(terms['v_set'][0], 0)

    def test_seeded_noise_changes_feedback_not_the_truth_record_definition(self):
        def run(seed):
            return hover_pid.simulate_axis(hover_pid.AxisPID(1.2, 0.6), 0.5, 0.02, 100,
                                           position_noise_std=0.01, velocity_noise_std=0.03, seed=seed)
        first, same, other = run(20), run(20), run(21)
        np.testing.assert_array_equal(first['x'], same['x'])
        self.assertFalse(np.array_equal(first['measured_x'], first['x']))
        self.assertFalse(np.array_equal(first['measured_x'], other['measured_x']))
        self.assertTrue(np.isfinite(first['x']).all())


class PracticalTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def test_runner_reuses_original_plants_and_saves_evidence_without_overwriting(self):
        output = self.root / 'baseline'
        with patch.object(hover_pid, 'simulate_axis', wraps=hover_pid.simulate_axis) as original:
            practical.main(['--output', str(output), '--no-show'])
        original.assert_called_once()
        for name in ('position', 'velocity', 'cascade'):
            self.assertGreater((output / f'{name}.csv').stat().st_size, 1000)
        self.assertGreater((output / 'comparison.png').stat().st_size, 1000)
        settings = json.loads((output / 'settings.json').read_text())
        self.assertEqual(settings['data_kind'], 'simulation')
        self.assertEqual(settings['metrics']['velocity']['error_units'], 'm/s')
        self.assertEqual(settings['metrics']['cascade']['error_units'], 'm')
        with patch('sys.stderr'), self.assertRaises(SystemExit):
            practical.main(['--output', str(output), '--no-show'])

    def test_replay_accepts_bom_and_rejects_nonfinite_or_repeated_times(self):
        path = self.root / 'state.csv'
        source = 't_s,x,y,vx,vy\n0,0,0,0,0\n0.1,0.1,-0.1,0.2,-0.2\n'
        path.write_text(source, encoding='utf-8-sig')
        data = practical.load_recording(path)
        self.assertEqual(data['x'][1], 0.1)
        output = self.root / 'replay'
        practical.main(['--mode', 'replay', '--recording', str(path), '--output', str(output), '--no-show'])
        with (output / 'x.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertLess(float(rows[1]['cmd']), 0)
        self.assertLess(float(rows[1]['velocity_tilt_cmd']), 0)
        self.assertLess(float(rows[1]['cascade_tilt_cmd']), 0)
        self.assertIn('recording_sha256', json.loads((output / 'settings.json').read_text()))
        for invalid in (source.replace('0.1,0.1', '0,0.1'), source.replace('0.2,-0.2', 'nan,-0.2'),
                        source.replace('0.2,-0.2', '0.2')):
            path.write_text(invalid)
            with self.assertRaises(ValueError):
                practical.load_recording(path)

    def test_replay_saves_and_plots_independent_velocity_and_cascade_commands(self):
        import matplotlib.pyplot as plt

        path = self.root / 'displacement.csv'
        path.write_text('t_s,x,y,vx,vy\n0,0,0,0,0\n0.1,0.2,-0.2,0,0\n'
                        '0.2,0.2,-0.2,0.1,-0.1\n', encoding='utf-8')
        output = self.root / 'separate-commands'
        # Inspect plotted line data as well as CSV values, not just PNG existence.
        with patch.object(plt, 'close'):
            practical.main(['--mode', 'replay', '--recording', str(path),
                            '--output', str(output), '--no-show'])
            figure = plt.gcf()
        self.addCleanup(plt.close, figure)
        tilt_axes = [axis for axis in figure.axes
                     if axis.get_ylabel() == 'Tilt toward positive axis [deg]']
        self.assertEqual(len(tilt_axes), 2)
        for name, sign, axis in zip(('x', 'y'), (1, -1), tilt_axes):
            with (output / f'{name}.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            np.testing.assert_allclose([float(row['velocity_target']) for row in rows], 0)
            self.assertEqual(float(rows[1]['velocity_tilt_cmd']), 0)
            self.assertAlmostEqual(float(rows[1]['cascade_tilt_cmd']), -sign * 2.88)
            self.assertLess(sign * float(rows[2]['velocity_tilt_cmd']), 0)
            lines = {line.get_label(): line for line in axis.lines}
            for label, column in (('standalone velocity-loop tilt', 'velocity_tilt_cmd'),
                                  ('cascade tilt', 'cascade_tilt_cmd')):
                np.testing.assert_allclose(lines[label].get_ydata(),
                                           [float(row[column]) for row in rows])

    def test_replay_integral_histories_remain_independent(self):
        data = dict(t_s=np.array([0, 0.1, 0.2]), x=np.array([0, 0.1, 0.2]),
                    y=np.array([0, -0.1, -0.2]), vx=np.zeros(3), vy=np.zeros(3))
        args = SimpleNamespace(kp=1.2, kd=0.6, ki=0, v_max=0.3,
                               kv=12, kvi=1, tilt_max=6)
        for axis, result in practical.replay(data, args).items():
            standalone = velocity_pid.replay_velocity_axis(
                velocity_pid.AxisVelocityPID(12, kvi=1, tilt_max=6),
                np.zeros(3), data['v' + axis], data['t_s'])
            cascade = velocity_pid.replay_velocity_axis(
                velocity_pid.AxisVelocityPID(12, kvi=1, tilt_max=6),
                result['cmd'], data['v' + axis], data['t_s'])
            np.testing.assert_allclose(result['velocity_tilt_cmd'], standalone['tilt_cmd'])
            np.testing.assert_allclose(result['cascade_tilt_cmd'], cascade['tilt_cmd'])

    def test_handcheck_retains_partial_capture_on_failure(self):
        output = self.root / 'failed'

        def fail(uri, duration, samples):
            samples.append([0, 1, 2, 0, 0])
            raise ValueError('stale motor-status telemetry')

        with patch.object(practical, 'record_handcheck', side_effect=fail), patch('sys.stderr'):
            with self.assertRaises(SystemExit):
                practical.main(['--mode', 'handcheck', '--output', str(output), '--no-show'])
        self.assertIn('0,1,2,0,0', (output / 'state.csv').read_text())
        self.assertFalse((output / 'analysis').exists())

    def test_handcheck_source_has_no_flight_or_estimator_parameter_commands(self):
        tree = ast.parse(Path(practical.__file__).read_text())
        names = [node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)]
        for forbidden in ('commander', 'high_level_commander', 'supervisor', 'set_value',
                          'send_setpoint', 'send_arming_request'):
            self.assertNotIn(forbidden, names)

    def test_handcheck_requires_all_zero_motor_channels_and_unwraps_firmware_time(self):
        for motor_values in ({f'motor.m{i}': 0 for i in range(1, 5)},
                             {'motor.m1': 1}, {}):
            with self.subTest(motors=motor_values):
                class Callback:
                    def add_callback(self, callback):
                        self.callback = callback

                class LogBlock:
                    def __init__(self, name, period_in_ms):
                        self.name = name
                        self.data_received_cb = Callback()
                        self.error_cb = Callback()

                    def add_variable(self, name, kind):
                        pass

                    def start(self):
                        if self.name == 'day1_motors':
                            self.data_received_cb.callback(0, motor_values, self)
                        else:
                            state = dict(zip(('stateEstimate.x', 'stateEstimate.y',
                                              'stateEstimate.vx', 'stateEstimate.vy'), (1, 2, 0, 0)))
                            for timestamp in ((1 << 24) - 25, 25):
                                self.data_received_cb.callback(timestamp, state, self)

                    def stop(self):
                        pass

                root = ModuleType('cflib')
                crtp = ModuleType('cflib.crtp')
                crtp.init_drivers = Mock()
                crazyflie = ModuleType('cflib.crazyflie')
                crazyflie.Crazyflie = Mock()
                log = ModuleType('cflib.crazyflie.log')
                log.LogConfig = LogBlock
                sync = ModuleType('cflib.crazyflie.syncCrazyflie')
                connection = MagicMock()
                # No commander or supervisor exists on this fake: accidental
                # flight calls fail the test, without requiring USB or cflib.
                connection.__enter__.return_value = SimpleNamespace(
                    cf=SimpleNamespace(log=SimpleNamespace(add_config=Mock())))
                sync.SyncCrazyflie = Mock(return_value=connection)
                root.crtp = crtp
                modules = {module.__name__: module for module in (root, crtp, crazyflie, log, sync)}
                with patch.dict(sys.modules, modules), patch('builtins.input', return_value='RECORD ONLY'):
                    if len(motor_values) == 4:
                        result = practical.record_handcheck('radio://0/80/2M/E7E7E7E701', 0)
                        np.testing.assert_allclose(result['t_s'], [0, 0.05])
                    else:
                        with self.assertRaisesRegex(ValueError, 'motor command'):
                            practical.record_handcheck('radio://0/80/2M/E7E7E7E701', 0)

    def test_simulation_rejects_invalid_settings(self):
        for settings in (['--dt', 'nan'], ['--tau', '0'], ['--duration', '2'],
                         ['--position-noise-std', '-1'], ['--dt', '0.5']):
            with self.subTest(settings=settings), patch('sys.stderr'), self.assertRaises(SystemExit):
                practical.main([*settings, '--output', str(self.root / 'invalid'), '--no-show'])


if __name__ == '__main__':
    unittest.main()
