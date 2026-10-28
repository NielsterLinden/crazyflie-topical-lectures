"""Checks for the shared six-state Kalman filter."""

import unittest
import numpy as np

from .filter import FilterConfig, PositionVelocityKalmanFilter


class KalmanTests(unittest.TestCase):
    def test_position_update_reduces_position_variance(self) -> None:
        estimator = PositionVelocityKalmanFilter(FilterConfig())
        before = np.diag(estimator.position_covariance)
        estimator.update_position(np.array([1.0, 2.0, 3.0]))
        after = np.diag(estimator.position_covariance)
        self.assertTrue(np.all(after < before))

    def test_large_innovation_can_be_rejected(self) -> None:
        estimator = PositionVelocityKalmanFilter(FilterConfig(nis_gate=7.815))
        accepted = estimator.update_position(np.array([10.0, 0.0, 0.0]))
        self.assertFalse(accepted)
        self.assertEqual(estimator.rejected_updates, 1)



if __name__ == "__main__":
    unittest.main()
