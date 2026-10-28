"""The gate front end adds policy results without changing the shared estimator."""

import unittest
import contextlib
import io
from pathlib import Path
import tempfile

import numpy as np

from kalman.simulation import run as run_filter
from .uncertainty import main, make_hoops, run


class UncertaintyTests(unittest.TestCase):
    def test_scenarios_keep_noise_matched_and_isolate_faults(self):
        nominal = run_filter(7, "nominal")
        dropout = run_filter(7, "dropout")
        outlier = run_filter(7, "outlier")
        time = nominal["time"]
        gap = (time >= 7.0) & (time <= 10.0)
        np.testing.assert_array_equal(dropout["position_measurement"][~gap], nominal["position_measurement"][~gap])
        self.assertTrue(np.all(np.isnan(dropout["position_measurement"][gap])))
        fault = np.isclose(time, 12.0)
        np.testing.assert_array_equal(outlier["position_measurement"][~fault], nominal["position_measurement"][~fault])
        np.testing.assert_allclose(outlier["position_measurement"][fault] - nominal["position_measurement"][fault], [[0, 0.65, 0]])
        for scenario in ("nominal", "dropout", "outlier"):
            filtered = run_filter(7, scenario)
            racing = run(7, scenario)
            for name in filtered:
                np.testing.assert_array_equal(filtered[name], racing[name])
        with self.assertRaisesRegex(ValueError, "unknown scenario"):
            run_filter(7, "typo")

    def test_cli_saves_each_scenario_and_validates_arguments(self):
        with tempfile.TemporaryDirectory() as temporary:
            for scenario in ("nominal", "dropout", "outlier"):
                output = Path(temporary) / f"{scenario}.png"
                main(["--scenario", scenario, "--seed", "7", "--output", str(output), "--no-show"])
                self.assertGreater(output.stat().st_size, 1000)
        with contextlib.redirect_stdout(io.StringIO()) as output, self.assertRaises(SystemExit) as exit_status:
            main(["--help"])
        self.assertEqual(exit_status.exception.code, 0)
        self.assertIn("--scenario", output.getvalue())
        for arguments in (["--scenario", "typo"], ["--seed", "-1"], ["--unknown"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exit_status:
                main(arguments)
            self.assertEqual(exit_status.exception.code, 2)

    def test_gate_policy_preserves_estimation_inputs_and_results(self):
        estimation = run_filter(7)
        racing = run(7)
        for name, values in estimation.items():
            np.testing.assert_array_equal(values, racing[name], err_msg=name)
        self.assertNotIn("speed", estimation)
        self.assertNotIn("clearance", estimation)
        self.assertEqual(racing["speed"].shape, estimation["time"].shape)
        self.assertTrue(np.all(np.isfinite(racing["clearance"])))

    def test_full_covariance_is_available_to_the_gate_projection(self):
        records = run_filter(3)
        self.assertEqual(records["position_covariance"].shape[1:], (3, 3))
        np.testing.assert_allclose(np.diagonal(records["position_covariance"], axis1=1, axis2=2),
                                   records["position_std"]**2, atol=1e-15)
        for hoop in make_hoops():
            projected = hoop.plane_covariance(records["position_covariance"][-1])
            self.assertEqual(projected.shape, (2, 2))
            self.assertTrue(np.all(np.linalg.eigvalsh(projected) >= 0))


if __name__ == "__main__":
    unittest.main()
