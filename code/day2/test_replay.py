"""Checks for the restricted prepared-data E6 replay, never a live adapter."""

import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from kalman.filter import FilterConfig
from kalman.recording import ReplayFilter, load_recording, replay
from .replay import save_results


EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.csv = self.root / "input.csv"
        self.metadata = self.root / "recording.json"
        self.metadata.write_text((EXAMPLES / "replay.synthetic.json").read_text())

    def write_rows(self, rows):
        with self.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "ax_world_m_s2", "ay_world_m_s2", "az_world_m_s2", "px_m", "py_m", "pz_m"])
            writer.writerows(rows)

    def test_preceding_interval_input_is_integrated_once(self):
        samples = [(0, np.zeros(3), None), (0.2, np.array([2, 0, 0]), None),
                   (0.4, np.array([0, 2, 0]), None)]
        results = replay(samples, FilterConfig())
        self.assertAlmostEqual(results[1]["px_m"], 0.04)
        self.assertAlmostEqual(results[1]["vx_m_s"], 0.4)
        self.assertAlmostEqual(results[2]["px_m"], 0.12)
        self.assertAlmostEqual(results[2]["py_m"], 0.04)
        self.assertEqual(results[2]["chi2"], "")
        self.assertEqual(results[2]["predicted_residual_x_m"], "")

    def test_gap_limit_applies_to_file_loading_and_direct_replay(self):
        for gap in (0.3001, 60.0):
            with self.subTest(gap=gap):
                self.write_rows([[100, 0, 0, 0, "", "", ""],
                                 [100 + gap, 1, 0, 0, "", "", ""]])
                with self.assertRaisesRegex(ValueError, "gaps exceed 0.30"):
                    load_recording(self.csv, self.metadata)
                with self.assertRaisesRegex(ValueError, "gaps exceed 0.30"):
                    replay([(100, np.zeros(3), None),
                            (100 + gap, np.ones(3), None)], FilterConfig())

    def test_invalid_interval_does_not_change_filter_or_clock(self):
        estimator = ReplayFilter(FilterConfig())
        estimator.step(100., np.zeros(3), np.array([1., 2., 3.]))
        x, C = estimator.estimator.x.copy(), estimator.estimator.C.copy()
        for timestamp in (100., 99., 160., float("nan"), float("inf")):
            with self.subTest(timestamp=timestamp), self.assertRaises(ValueError):
                estimator.step(timestamp, np.ones(3), None)
            np.testing.assert_array_equal(estimator.estimator.x, x)
            np.testing.assert_array_equal(estimator.estimator.C, C)
            self.assertEqual(estimator.previous_time, 100.)
        self.assertEqual(estimator.step(100.3, np.zeros(3), None)["time_s"], 100.3)

    def test_gap_boundary_and_recorded_clock_are_preserved(self):
        self.write_rows([[100., 0, 0, 0, "", "", ""],
                         [100.3, 0, 0, 0, "", "", ""]])
        samples, _ = load_recording(self.csv, self.metadata)
        self.assertEqual([row[0] for row in samples], [100., 100.3])
        rows = replay(samples, FilterConfig())
        self.assertEqual([row["time_s"] for row in rows], [100., 100.3])

    def test_epoch_clock_accepts_boundary_but_rejects_real_oversized_gaps(self):
        origin = 1700000000.1
        for gap in (0.3, 0.300001, 0.3001, 60.):
            with self.subTest(gap=gap):
                # These decimal endpoints reproduce subtraction above 0.30 s.
                end = 1700000000.4 if gap == 0.3 else origin + gap
                self.write_rows([[origin, 0, 0, 0, "", "", ""],
                                 [end, 0, 0, 0, "", "", ""]])
                if gap == 0.3:
                    self.assertGreater(end - origin, 0.3)
                    samples, _ = load_recording(self.csv, self.metadata)
                    rows = replay(samples, FilterConfig())
                    self.assertEqual(rows[-1]["time_s"], end)
                else:
                    with self.assertRaisesRegex(ValueError, "gaps exceed"):
                        load_recording(self.csv, self.metadata)
                    with self.assertRaisesRegex(ValueError, "gaps exceed"):
                        replay([(origin, np.zeros(3), None),
                                (end, np.zeros(3), None)], FilterConfig())

    def test_e6_figure_has_position_velocity_and_residual_for_each_axis(self):
        import matplotlib.pyplot as plt
        samples = [(100., np.zeros(3), None), (100.2, np.array([1., 2., 3.]), None)]
        results = replay(samples, FilterConfig())
        # Inspect the real figure before save_results closes it.
        close_figure = plt.close
        with patch.object(plt, "close"):
            try:
                save_results(results, {}, FilterConfig(), self.root / "panels")
                figure = plt.gcf()
                self.assertEqual(len(figure.axes), 9)
                for index, axis in enumerate("xyz"):
                    velocity_plot = figure.axes[index * 3 + 1]
                    self.assertEqual(velocity_plot.get_ylabel(), f"{axis} velocity [m/s]")
                    np.testing.assert_allclose(velocity_plot.lines[0].get_xdata(), [100., 100.2])
                    np.testing.assert_allclose(velocity_plot.lines[0].get_ydata(),
                                               [row[f"v{axis}_m_s"] for row in results])
                    self.assertIn("position", figure.axes[index * 3].get_ylabel())
                    self.assertIn("predicted residual", figure.axes[index * 3 + 2].get_ylabel())
            finally:
                close_figure(plt.gcf())

    def test_utf8_inputs_with_and_without_bom(self):
        csv_text = (EXAMPLES / "replay.synthetic.csv").read_text(encoding="utf-8")
        metadata_text = self.metadata.read_text(encoding="utf-8")
        expected_samples, expected_metadata = load_recording(EXAMPLES / "replay.synthetic.csv", self.metadata)
        for csv_encoding in ("utf-8", "utf-8-sig"):
            for metadata_encoding in ("utf-8", "utf-8-sig"):
                with self.subTest(csv=csv_encoding, metadata=metadata_encoding):
                    self.csv.write_text(csv_text, encoding=csv_encoding)
                    self.metadata.write_text(metadata_text, encoding=metadata_encoding)
                    samples, metadata = load_recording(self.csv, self.metadata)
                    self.assertEqual(metadata, expected_metadata)
                    self.assertEqual(len(samples), len(expected_samples))
                    for actual, expected in zip(samples, expected_samples):
                        self.assertEqual(actual[0], expected[0])
                        np.testing.assert_array_equal(actual[1], expected[1])
                        if expected[2] is None:
                            self.assertIsNone(actual[2])
                        else:
                            np.testing.assert_array_equal(actual[2], expected[2])

    def test_fixture_and_output_artifacts(self):
        samples, metadata = load_recording(EXAMPLES / "replay.synthetic.csv", self.metadata)
        config = FilterConfig()
        rows = replay(samples, config)
        self.assertEqual(len(rows), 5)
        self.assertEqual(rows[0]["accepted"], 1)
        self.assertEqual(rows[1]["accepted"], "")
        output = self.root / "results" / "baseline"
        save_results(rows, metadata, config, output)
        with Path(str(output) + ".csv").open(newline="") as stream:
            saved = list(csv.DictReader(stream))
        for axis in "xyz":
            name = f"predicted_residual_{axis}_m"
            self.assertIn(name, saved[0])
            self.assertEqual(saved[1][name], "")
            for actual, expected in zip(saved, rows):
                self.assertEqual(actual[name], str(expected[name]))
        old_term = "inno" + "vation"
        self.assertFalse(any(old_term in name for name in saved[0]))
        self.assertGreater(Path(str(output) + ".png").stat().st_size, 1000)
        settings = json.loads(Path(str(output) + ".json").read_text())
        self.assertEqual(settings["filter"]["position_noise_std"], 0.04)
        with self.assertRaisesRegex(ValueError, "already exists"):
            save_results(rows, metadata, config, output)

    def test_invalid_or_unprepared_inputs_are_rejected(self):
        cases = [
            [[0, 0, 0, 0, "", "", ""], [0, 0, 0, 0, "", "", ""]],
            [[1, 0, 0, 0, "", "", ""], [0, 0, 0, 0, "", "", ""]],
            [[0, 0, 0, 0, "", "", ""], [1, "nan", 0, 0, "", "", ""]],
            [[0, 0, 0, 0, "", "", ""], [1, 0, 0, 0, 1, "", 2]],
            [[0, 0, 0, 0, "", "", ""]],
        ]
        for rows in cases:
            with self.subTest(rows=rows):
                self.write_rows(rows)
                with self.assertRaises(ValueError):
                    load_recording(self.csv, self.metadata)
        self.write_rows([[0, 0, 0, 0, "", "", ""], [1, 0, 0, 0, "", "", ""]])
        metadata = json.loads(self.metadata.read_text())
        metadata["prepared_replay"]["gravity_removed"] = False
        self.metadata.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "metadata"):
            load_recording(self.csv, self.metadata)


if __name__ == "__main__":
    unittest.main()
