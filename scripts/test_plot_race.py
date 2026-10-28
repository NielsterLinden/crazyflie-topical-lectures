"""Offline telemetry alignment and course rendering regression tests."""

import csv
import itertools
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_race import RaceLog, break_gaps, compare, gate_outline, load_telemetry, main, plot_comparison, select_phase


class ComparisonTests(unittest.TestCase):
    def log(self):
        return RaceLog(
            np.array([9.9, 10.1, 10.3, 10.8, 11.1]),
            np.array([[0, 0, 0], [1.1, 0, 0], [2.2, 0, 0], [3, 0, 0], [0, 0, 0]]),
            np.array([10, 10.2, 11]),
            np.array([[1, 0, 0], [2, 0, 0], [0, 0, 0]]),
            np.array(["trajectory", "trajectory", "landing"]),
        )

    def test_holds_last_command_does_not_interpolate_or_include_landing(self):
        result = compare(self.log(), "trajectory", 0.7)
        np.testing.assert_allclose(result.times, [10.1, 10.3, 10.8])
        np.testing.assert_allclose(result.targets[:, 0], [1, 2, 2])
        np.testing.assert_allclose(result.errors[:, 0], [0.1, 0.2, 1])

    def test_stale_command_errors_are_missing(self):
        log = self.log()
        # A later command extends the window beyond a deliberate stream gap.
        log.command_times[2] = 12
        log.phases[2] = "trajectory"
        result = compare(log, "trajectory", 0.25)
        self.assertTrue(np.isnan(result.errors[-1]).all())
        np.testing.assert_allclose(result.errors[:2, 0], [0.1, 0.2])

    def test_all_phases_include_landing(self):
        result = compare(self.log(), "all", 0.7)
        self.assertEqual(result.times[-1], 11.1)
        np.testing.assert_allclose(result.errors[-1], 0)

    def test_auto_phase_supports_current_and_legacy_logs(self):
        log = self.log()
        self.assertEqual(select_phase(log, "auto"), "trajectory")
        np.testing.assert_allclose(compare(log, "auto", 0.7).times, [10.1, 10.3, 10.8])
        log.phases[0] = "race"
        self.assertEqual(select_phase(log, "auto"), "race")
        log.phases[:] = "landing"
        with self.assertRaisesRegex(ValueError, "explicit --phase"):
            compare(log, "auto", 0.7)

    def test_invalid_phase_or_gap_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "No commands for phase"):
            compare(self.log(), "missing", 0.25)
        for gap in (0, -1, float("nan")):
            with self.subTest(gap=gap), self.assertRaisesRegex(ValueError, "max-gap"):
                compare(self.log(), "all", gap)

    def test_duplicate_command_time_uses_last_sent_value(self):
        log = self.log()
        log.command_times[1] = 10
        result = compare(log, "all", 0.7)
        self.assertEqual(result.targets[0, 0], 2)

    def test_sparse_csv_sorting_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "race.csv"
            fields = ["event", "host_monotonic_s", "x_m", "y_m", "z_m",
                      "command_x_m", "command_y_m", "command_z_m", "phase"]
            rows = [
                {"event": "tof", "host_monotonic_s": ""},
                dict(zip(fields, ["position", 10.2, 1.1, 2, 3, "", "", "", ""])),
                dict(zip(fields, ["command", 10, "", "", "", 1, 2, 3, "trajectory"])),
                dict(zip(fields, ["position", 10.1, 1, 2, 3, "", "", "", ""])),
            ]
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fields)
                writer.writeheader()
                writer.writerows(rows)
            log = load_telemetry(path)
            np.testing.assert_allclose(log.position_times, [10.1, 10.2])
            self.assertEqual(log.phases[0], "trajectory")
            path.write_text("event,host_monotonic_s,x_m,y_m,z_m\nposition,10,nan,0,0\n")
            with self.assertRaisesRegex(ValueError, "CSV line 2"):
                load_telemetry(path)

    def test_missing_commands_are_not_replaced_with_guessed_plan_times(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.csv"
            path.write_text("event,host_monotonic_s,x_m,y_m,z_m\nposition,10,0,0,0\n")
            with self.assertRaisesRegex(ValueError, "both event=position"):
                load_telemetry(path)


class PlotTests(unittest.TestCase):
    def test_cli_accepts_utf8_bom_in_csv_course_and_survey(self):
        self.addCleanup(plt.close, "all")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            telemetry = root / "flight.csv"
            course = root / "course.json"
            survey = root / "gate.json"
            course_data = {"course_name": "test", "hoops": [self.circle()],
                           "start": [0, 0, 0], "finish": [1, 0, 0]}
            fields = ["event", "host_monotonic_s", "x_m", "y_m", "z_m",
                      "command_x_m", "command_y_m", "command_z_m", "phase"]
            # Test each input independently as well as files all saved with a BOM.
            for encodings in itertools.product(("utf-8", "utf-8-sig"), repeat=3):
                with self.subTest(encodings=encodings):
                    with telemetry.open("w", newline="", encoding=encodings[0]) as stream:
                        writer = csv.writer(stream)
                        writer.writerow(fields)
                        writer.writerow(["command", 10, "", "", "", 1, 2, 3, "race"])
                        writer.writerow(["position", 10.1, 1.1, 2, 3, "", "", "", ""])
                    course.write_text(json.dumps(course_data), encoding=encodings[1])
                    survey.write_text(json.dumps(self.circle()), encoding=encodings[2])
                    output = root / "comparison"
                    arguments = ["plot_race.py", str(telemetry), "--course", str(course),
                                 "--gate-survey", str(survey), "--output", str(output), "--no-show"]
                    with patch("sys.argv", arguments), patch("builtins.print"):
                        self.assertEqual(main(), 0)
                    for suffix in ("trajectory", "errors"):
                        self.assertGreater((root / f"comparison-{suffix}.png").stat().st_size, 1000)

    def circle(self):
        return {"name": "horizontal", "centre": [1, 2, 3],
                "normal": [0, 0, 1], "radius_m": 0.5}

    def test_horizontal_circle_is_in_correct_plane(self):
        points = gate_outline(self.circle())
        np.testing.assert_allclose(points[:, 2], 3)
        np.testing.assert_allclose(np.linalg.norm(points[:, :2] - [1, 2], axis=1), 0.5)

    def test_rectangle_outline_uses_measured_axes(self):
        gate = {**self.circle(), "shape": "rectangle", "width_m": 2, "height_m": 1,
                "width_axis": [1, 0, 0], "height_axis": [0, 1, 0]}
        points = gate_outline(gate)
        np.testing.assert_allclose(np.ptp(points, axis=0), [2, 1, 0])
        np.testing.assert_allclose(points[0], points[-1])
        gate["height_axis"] = [1, 0, 0]
        with self.assertRaisesRegex(ValueError, "orthogonal"):
            gate_outline(gate)

    def test_plot_has_projections_errors_and_deduplicated_hoops(self):
        log = ComparisonTests().log()
        comparison = compare(log, "trajectory", 0.7)
        gate = self.circle()
        reverse = {**gate, "name": "return", "normal": [0, 0, -1]}
        course = {"course_name": "test", "hoops": [gate, reverse],
                  "start": [0, 0, 0], "finish": [2, 0, 0]}
        figures = plot_comparison(comparison, course, [], None, 0.7)
        self.addCleanup(plt.close, "all")
        self.assertEqual(len(figures[0].axes), 4)
        self.assertEqual(len(figures[1].axes), 4)
        labels = figures[0].axes[0].get_legend_handles_labels()[1]
        self.assertEqual(sum(label.startswith("Gate") for label in labels), 1)
        with tempfile.TemporaryDirectory() as directory:
            for index, figure in enumerate(figures):
                path = Path(directory) / f"figure-{index}.png"
                figure.savefig(path)
                self.assertGreater(path.stat().st_size, 10000)

    def test_dropouts_break_drawn_lines(self):
        points = np.zeros((3, 3))
        result = break_gaps(np.array([0, 0.1, 1]), points, 0.25)
        self.assertEqual(result.shape, (4, 3))
        self.assertTrue(np.isnan(result[2]).all())


if __name__ == "__main__":
    unittest.main()
