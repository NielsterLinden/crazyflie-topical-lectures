"""Independent mathematical and output checks for the tracking example."""

import csv
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from day2 import tracking


class TrackingTests(unittest.TestCase):
    def position_demo(self):
        notebook = json.loads((Path(__file__).parent / '01_tracking.ipynb').read_text())
        cells = {cell['id']: ''.join(cell['source']) for cell in notebook['cells']}
        namespace = dict(np=np)
        with redirect_stdout(io.StringIO()):
            exec(cells['day2-position-velocity-demo'], namespace)
        return namespace, cells

    def test_position_measurement_changes_velocity_through_cross_covariance(self):
        demo, _ = self.position_demo()
        first, second = demo['demo_updates'][:2]
        self.assertEqual(first['K'][1], 0)
        self.assertAlmostEqual(second['C'][1, 0], 0.2)
        expected_gain = second['C'][1, 0] / (second['C'][0, 0] + demo['demo_V'])
        self.assertAlmostEqual(second['K'][1], expected_gain)
        self.assertAlmostEqual(demo['demo_states'][1, 1] - second['predicted'][1],
                               expected_gain * second['residual'])
        self.assertNotEqual(demo['demo_delta_v'], 0)
        # Removing only the cross-covariance retains the position correction.
        self.assertEqual(demo['demo_uncoupled_state'][1], second['predicted'][1])
        self.assertAlmostEqual(demo['demo_uncoupled_state'][0], demo['demo_states'][1, 0])

    def test_position_only_filter_matches_regularized_line_fit(self):
        demo, _ = self.position_demo()
        time = demo['demo_time']
        design = np.column_stack((np.ones_like(time), time))
        # Independent batch fit to intercept/slope with the same unit prior covariance.
        normal = np.eye(2) + design.T @ design / demo['demo_V']
        initial_C = np.linalg.solve(normal, np.eye(2))
        initial_state = np.linalg.solve(normal, design.T @ demo['demo_measurements'] / demo['demo_V'])
        F = np.array([[1., time[-1]], [0., 1.]])
        np.testing.assert_allclose(demo['demo_states'][-1], F @ initial_state, atol=1e-12)
        np.testing.assert_allclose(demo['demo_covariances'][-1], F @ initial_C @ F.T, atol=1e-12)
        self.assertAlmostEqual(demo['demo_states'][-1, 1], demo['demo_true_velocity'], delta=.02)
        self.assertTrue(np.all(np.linalg.eigvalsh(demo['demo_covariances']) > 0))

    def test_position_velocity_plot_is_nonblank_and_has_two_panels(self):
        import matplotlib.pyplot as plt

        demo, cells = self.position_demo()
        with patch.object(plt, 'show'):
            exec(cells['day2-position-velocity-plot'], demo)
        figure = demo['demo_figure']
        try:
            self.assertEqual(len(figure.axes), 2)
            self.assertEqual(figure.axes[0].get_ylabel(), 'Position [m]')
            self.assertEqual(figure.axes[1].get_ylabel(), 'Velocity [m/s]')
            for axis in figure.axes:
                self.assertTrue(axis.collections)  # Uncertainty band, including velocity.
            figure.canvas.draw()
            self.assertGreater(np.asarray(figure.canvas.buffer_rgba()).std(), 5)
        finally:
            plt.close(figure)

    def test_planes_start_at_one_and_first_gap_has_no_scattering(self):
        detector = tracking.Detector()
        np.testing.assert_array_equal(detector.z, np.arange(1, 11))
        np.testing.assert_array_equal(detector.prior_covariance, np.eye(2))
        track = tracking.simulate(detector, np.random.default_rng(7))
        F, G = tracking.transport(1.)
        np.testing.assert_allclose(track['truth'][0], F @ track['initial_truth'])
        fit = tracking.fit_track(track, detector)
        np.testing.assert_allclose(fit['predicted_C'][0], F @ detector.prior_covariance @ F.T)
        for before, after in zip(track['truth'][:-1], track['truth'][1:]):
            increment = after - F @ before
            np.testing.assert_allclose(increment, G * increment[1], atol=1e-15)

    def test_kick_then_propagation_covariance_and_units(self):
        F, G = tracking.transport(2.)
        np.testing.assert_array_equal(F, [[1, 2], [0, 1]])
        np.testing.assert_array_equal(G, [2, 1])
        np.testing.assert_allclose(np.outer(G, G) * .02**2,
                                   [[.0016, .0008], [.0008, .0004]])

    def test_sequential_solution_matches_joint_gaussian_fit(self):
        detector = tracking.Detector()
        for seed in range(5):
            track = tracking.simulate(detector, np.random.default_rng(seed))
            fit = tracking.fit_track(track, detector)
            state, covariance = tracking.batch_solution(track['measurements'], detector)
            np.testing.assert_allclose(fit['filtered'][-1], state, atol=1e-12)
            np.testing.assert_allclose(fit['filtered_C'][-1], covariance, atol=1e-12)
            self.assertTrue(np.all(np.linalg.eigvalsh(fit['filtered_C']) > 0))

    def test_thousand_track_pulls_coverage_and_joint_chi_square(self):
        result = tracking.ensemble()
        summary = result['summary']
        self.assertEqual(result['rows'].shape, (1000, 16))
        np.testing.assert_allclose(summary['mean_pull'], [0, 0], atol=.10)
        np.testing.assert_allclose(summary['std_pull'], [1, 1], atol=.10)
        np.testing.assert_allclose(summary['marginal_95_percent_coverage'], [.95, .95], atol=.03)
        self.assertAlmostEqual(summary['mean_state_error_chi2'], 2, delta=.20)
        self.assertEqual(summary['full_active_area_tracks'], int(result['rows'][:, 14].sum()))
        self.assertLess(summary['full_active_area_tracks'], 1000)
        np.testing.assert_allclose(result['rows'][:, 12],
                                   np.sum(result['rows'][:, 10:12]**2, axis=1))

    def test_outgoing_state_adds_unobserved_final_kick(self):
        incoming = tracking.ensemble(20)
        outgoing = tracking.ensemble(20, outgoing=True)
        np.testing.assert_array_equal(incoming['rows'][:, 1:3], outgoing['rows'][:, 1:3])
        expected = incoming['covariance'].copy()
        expected[:, 1, 1] += .02**2
        np.testing.assert_allclose(outgoing['covariance'], expected)
        np.testing.assert_array_equal(incoming['rows'][:, 3], outgoing['rows'][:, 3])
        self.assertTrue(np.any(incoming['rows'][:, 4] != outgoing['rows'][:, 4]))

    def test_finite_aperture_skips_hits_without_selecting_tracks(self):
        detector = tracking.Detector(active_height_m=.01)
        result = tracking.ensemble(40, detector=detector, finite_aperture=True)
        self.assertEqual(len(result['rows']), 40)
        track, fit = result['example']
        np.testing.assert_array_equal(fit['observed'], track['active'])
        for index in np.flatnonzero(~fit['observed']):
            np.testing.assert_array_equal(fit['filtered'][index], fit['predicted'][index])
            np.testing.assert_array_equal(fit['filtered_C'][index], fit['predicted_C'][index])
        self.assertTrue(np.any(result['rows'][:, 13] < 10))

    def test_saved_outputs_and_cli(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'trial'
            self.assertEqual(tracking.main(['--tracks', '12', '--output', str(output)]), 0)
            with (output / 'tracks.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 12)
            self.assertIn('C_xt_m', rows[0])
            self.assertEqual(json.loads((output / 'summary.json').read_text())['tracks'], 12)
            for filename in ('track.png', 'track.svg', 'pulls.png'):
                self.assertGreater((output / filename).stat().st_size, 1000)
            with self.assertRaises(FileExistsError):
                tracking.save(tracking.ensemble(2), output)

    def test_invalid_parameters_fail(self):
        for kwargs in ({'planes': 1}, {'measurement_std_m': 0},
                       {'scattering_slope_std': float('nan')}):
            with self.assertRaises(ValueError):
                tracking.Detector(**kwargs)
        with self.assertRaises(ValueError):
            tracking.transport(-1)
        with self.assertRaises(ValueError):
            tracking.ensemble(1)

    def test_notebook_reference_is_optional_and_runs_when_enabled(self):
        from day2.notebook_tools import next_attempt

        notebook = json.loads((Path(__file__).parent / '01_tracking.ipynb').read_text())
        source = ''.join(next(cell for cell in notebook['cells']
                              if cell['id'] == 'day2-tracking-reference')['source'])
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            output = Path(temporary) / 'notebook'
            displayed = []
            namespace = dict(output=output, next_attempt=next_attempt, json=json,
                             display=displayed.append, Image=lambda **kwargs: kwargs['filename'])
            exec(source, namespace)
            self.assertFalse(output.exists())
            exec(source.replace('run_tracking_reference = False',
                                'run_tracking_reference = True'), namespace)
            self.assertEqual(len(displayed), 2)
            self.assertTrue(all(Path(path).is_file() for path in displayed))
            self.assertEqual(namespace['tracking_result']['summary']['tracks'], 1000)


if __name__ == '__main__':
    unittest.main()
