"""Checks for the restricted prepared-data E6 replay, never a live adapter."""

import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from kalman.filter import FilterConfig
from kalman.recording import load_recording, replay
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
        samples = [(0, np.zeros(3), None), (1, np.array([2, 0, 0]), None),
                   (2, np.array([0, 2, 0]), None)]
        results = replay(samples, FilterConfig())
        self.assertEqual(results[1]["px_m"], 1)
        self.assertEqual(results[1]["vx_m_s"], 2)
        self.assertEqual(results[2]["px_m"], 3)
        self.assertEqual(results[2]["py_m"], 1)
        self.assertEqual(results[2]["nis"], "")
        self.assertEqual(results[2]["innovation_x_m"], "")

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
