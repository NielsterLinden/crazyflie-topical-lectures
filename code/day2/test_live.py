"""Radio-free tests for the live E6 adapter and saved replay equivalence."""

import ast
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kalman.filter import FilterConfig
from .live import CHANNELS, GRAVITY, LiveSession, SampleAssembler, draw_comparison, main, world_acceleration
from kalman.recording import ReplayFilter, load_recording, replay


def packet(stream, values):
    return dict(zip(CHANNELS[stream], values))


class LiveTests(unittest.TestCase):
    def setUp(self):
        self.assembler = SampleAssembler()
        self.motors = packet("motors", [0] * 4)
        self.attitude = packet("attitude", [0, 0, 0, 1])
        self.observation = packet("imu_position", [0, 0, 1, .1, .2, .3])

    def prepare(self, timestamp=100):
        self.assembler.accept("motors", timestamp, self.motors)
        self.assembler.accept("attitude", timestamp, self.attitude)

    def test_rotation_gravity_and_bias(self):
        for angles in ([0, 0, 0], [20, -30, 65], [180, 0, 0]):
            rotation = Rotation.from_euler("xyz", angles, degrees=True)
            force = rotation.inv().apply([0, 0, 1])
            np.testing.assert_allclose(world_acceleration(force, rotation.as_quat()), 0, atol=1e-12)
        np.testing.assert_allclose(world_acceleration([1, 0, 1], [0, 0, 0, 1]), [GRAVITY, 0, 0])
        with self.assertRaises(ValueError):
            world_acceleration([0, 0, 1], [0, 0, 0, 0])
        self.assembler = SampleAssembler([.02, 0, 0])
        self.prepare()
        sample = self.assembler.accept("imu_position", 100, packet("imu_position", [.02, 0, 1, 0, 0, 0]))
        np.testing.assert_allclose(sample[1], 0)

    def test_causal_pairing_no_duplicate_position_updates(self):
        self.assertIsNone(self.assembler.accept("imu_position", 50, self.observation))
        self.prepare(100)
        first = self.assembler.accept("imu_position", 100, self.observation)
        self.assertEqual(first[0], 0)
        np.testing.assert_allclose(first[1], 0)
        np.testing.assert_allclose(first[2], [.1, .2, .3])
        self.assertIsNone(self.assembler.accept("imu_position", 100, self.observation))
        self.assertIsNone(self.assembler.accept("imu_position", 90, self.observation))
        self.assertAlmostEqual(self.assembler.accept("imu_position", 150, self.observation)[0], .05)

    def test_future_stale_and_active_motor_data_are_rejected(self):
        self.prepare(200)
        self.assertIsNone(self.assembler.accept("imu_position", 100, self.observation))
        self.assembler.accept("imu_position", 200, self.observation)
        with self.assertRaisesRegex(ValueError, "stale"):
            self.assembler.accept("imu_position", 500, self.observation)
        with self.assertRaisesRegex(ValueError, "nonzero"):
            self.assembler.accept("motors", 600, packet("motors", [0, 1, 0, 0]))

    def test_long_gap_is_not_integrated_with_one_acceleration_snapshot(self):
        self.prepare(100)
        self.assembler.accept("imu_position", 100, self.observation)
        self.prepare(1000)
        with self.assertRaisesRegex(ValueError, "gap too long"):
            self.assembler.accept("imu_position", 1000, self.observation)

    def test_timestamp_wrap_preserves_late_packet_time(self):
        modulus = 1 << 24
        self.assertEqual(self.assembler.unwrap(modulus - 20), modulus - 20)
        self.assertEqual(self.assembler.unwrap(30), modulus + 30)
        self.assertEqual(self.assembler.unwrap(modulus - 10), modulus - 10)
        self.assertEqual(self.assembler.unwrap(80), modulus + 80)

    def test_incremental_filter_matches_offline_replay(self):
        samples = [(0, np.zeros(3), np.array([.1, .2, .3])),
                   (.05, np.array([.1, 0, 0]), np.array([.2, .2, .3]))]
        for sigma in (.04, .12):
            config = FilterConfig(position_noise_std=sigma)
            processor = ReplayFilter(config)
            self.assertEqual([processor.step(*sample) for sample in samples], replay(samples, config))

    def test_reorder_journal_and_saved_replay_match_both_live_filters(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            configs = [FilterConfig(position_noise_std=sigma) for sigma in (.04, .12)]
            session = LiveSession(output, configs, [0, 0, 0], "test://no-radio")
            for stamp in (100, 150, 200):
                # The IMU callback arrives first; same-timestamp causal pairing
                # must not depend on radio callback order.
                session.enqueue("imu_position", stamp, self.observation, 0)
                session.enqueue("attitude", stamp, self.attitude, .01)
                session.enqueue("motors", stamp, self.motors, .02)
            session.drain(.1)
            self.assertEqual(len(session.results[0]), 0)
            session.drain(.3)
            self.assertEqual(len(session.results[0]), 3)
            session.enqueue("imu_position", 50, self.observation, .4)
            self.assertEqual(session.assembler.skipped, 1)
            session.finish("interrupted", "test stop")
            samples, metadata = load_recording(output / "prepared.csv", output / "recording.json")
            self.assertEqual(metadata["status"], "interrupted")
            self.assertEqual(len(metadata["source_sha256"]["raw.csv"]), 64)
            for actual, config in zip(session.results, configs):
                self.assertEqual(actual, replay(samples, config))
            import matplotlib.pyplot as plt
            figure = plt.figure(figsize=(13, 9))
            draw_comparison(figure, session.results, window_s=10)
            figure.savefig(output / "live-comparison.png")
            self.assertEqual(len(figure.axes), 9)
            self.assertGreater((output / "live-comparison.png").stat().st_size, 10000)
            plt.close(figure)
            self.assertTrue((output / "baseline.png").is_file())
            self.assertTrue((output / "comparison.png").is_file())
            with self.assertRaises(FileExistsError):
                LiveSession(output, configs, [0, 0, 0], "test://no-radio")

    def test_live_module_contains_no_control_or_parameter_calls(self):
        source = Path(__file__).with_name("live.py").read_text()
        calls = [node.func.attr for node in ast.walk(ast.parse(source))
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)]
        for name in calls:
            self.assertFalse(name.startswith("send_"), name)
            self.assertNotIn(name, ("set_value", "takeoff", "land", "go_to", "request_arming"))

    def test_confirmation_and_uri_required_before_connecting(self):
        with self.assertRaises(SystemExit):
            main(["--uri", "test://no-radio", "--output", "unused"])


if __name__ == "__main__":
    unittest.main()
