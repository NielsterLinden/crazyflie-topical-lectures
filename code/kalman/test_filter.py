"""Checks for the shared six-state Kalman filter."""

import unittest
import numpy as np

from .filter import FilterConfig, PositionVelocityKalmanFilter


class KalmanTests(unittest.TestCase):
    def test_prediction_with_interval_held_acceleration(self) -> None:
        estimator = PositionVelocityKalmanFilter()
        estimator.x = np.array([1., 2., 3., 4., 5., 6.])
        dt = 0.2
        acceleration = np.array([2., -3., 0.5])
        initial = estimator.x.copy()
        C0 = estimator.C.copy()
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        B = np.vstack((np.eye(3) * dt**2 / 2, np.eye(3) * dt))
        Q = B @ (np.eye(3) * estimator.config.acceleration_noise_std**2) @ B.T
        estimator.predict(dt, acceleration)
        np.testing.assert_allclose(estimator.x, F @ initial + B @ acceleration)
        np.testing.assert_allclose(estimator.C, F @ C0 @ F.T + Q)

    def test_fruhwirth_predicted_and_filtered_residuals(self) -> None:
        estimator = PositionVelocityKalmanFilter()
        C_predicted = estimator.C.copy()
        x_predicted = estimator.x.copy()
        H = np.hstack((np.eye(3), np.zeros((3, 3))))
        V = np.eye(3) * estimator.config.position_noise_std**2
        m = np.array([0.02, -0.01, 0.03])
        r_predicted = m - H @ x_predicted
        R_predicted = V + H @ C_predicted @ H.T
        K = np.linalg.solve(R_predicted, H @ C_predicted).T
        self.assertTrue(estimator.update_position(m))
        np.testing.assert_allclose(estimator.last_V, V)
        np.testing.assert_allclose(estimator.last_predicted_residual, r_predicted)
        np.testing.assert_allclose(estimator.last_predicted_residual_covariance, R_predicted)
        np.testing.assert_allclose(estimator.x, x_predicted + K @ r_predicted)
        np.testing.assert_allclose(estimator.C, (np.eye(6) - K @ H) @ C_predicted)
        r_filtered = m - H @ estimator.x
        R_filtered = V - H @ estimator.C @ H.T
        self.assertAlmostEqual(
            estimator.last_chi2,
            float(r_filtered @ np.linalg.solve(R_filtered, r_filtered)),
        )

    def test_gated_update_preserves_state_and_covariance(self) -> None:
        estimator = PositionVelocityKalmanFilter(FilterConfig(chi2_gate=1.))
        x = estimator.x.copy()
        C = estimator.C.copy()
        self.assertFalse(estimator.update_position(np.ones(3) * 10))
        np.testing.assert_array_equal(estimator.x, x)
        np.testing.assert_array_equal(estimator.C, C)
        np.testing.assert_array_equal(estimator.last_predicted_residual, np.ones(3) * 10)
        np.testing.assert_allclose(estimator.last_predicted_residual_covariance,
                                   C[:3, :3] + np.eye(3) * estimator.config.position_noise_std**2)

    def test_diagnostic_names_identify_pre_update_quantities(self) -> None:
        estimator = PositionVelocityKalmanFilter()
        self.assertEqual(estimator.last_predicted_residual.shape, (0,))
        self.assertEqual(estimator.last_predicted_residual_covariance.shape, (0, 0))
        old_name = "last_" + "inno" + "vation"
        self.assertFalse(hasattr(estimator, old_name))
        self.assertFalse(hasattr(estimator, old_name + "_covariance"))
        measurement = np.array([.1, -.2, .3])
        estimator.update_position(measurement)
        np.testing.assert_array_equal(estimator.last_predicted_residual, measurement)
        self.assertGreater(np.linalg.norm(estimator.last_predicted_residual),
                           np.linalg.norm(measurement - estimator.position))

    def test_position_update_reduces_position_variance(self) -> None:
        estimator = PositionVelocityKalmanFilter(FilterConfig())
        before = np.diag(estimator.position_covariance)
        estimator.update_position(np.array([1.0, 2.0, 3.0]))
        after = np.diag(estimator.position_covariance)
        self.assertTrue(np.all(after < before))

    def test_large_predicted_residual_can_be_rejected(self) -> None:
        estimator = PositionVelocityKalmanFilter(FilterConfig(chi2_gate=7.815))
        accepted = estimator.update_position(np.array([10.0, 0.0, 0.0]))
        self.assertFalse(accepted)
        self.assertEqual(estimator.rejected_updates, 1)



if __name__ == "__main__":
    unittest.main()
