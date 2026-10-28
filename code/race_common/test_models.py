"""Checks for gate geometry and race integration profiles."""

import unittest
import numpy as np

from .hoop import Hoop
from .profiles import ESTIMATION_ONLY, FALLBACK, FULL, get_profile


class HoopTests(unittest.TestCase):
    def test_plane_basis_is_orthonormal(self) -> None:
        hoop = Hoop(np.zeros(3), np.array([1.0, 1.0, 0.0]), 0.5)
        basis = hoop.plane_basis()
        np.testing.assert_allclose(basis.T @ basis, np.eye(2), atol=1e-12)
        np.testing.assert_allclose(basis.T @ hoop.normal, np.zeros(2), atol=1e-12)

    def test_uncertainty_reduces_clearance(self) -> None:
        hoop = Hoop(np.zeros(3), np.array([1.0, 0.0, 0.0]), 0.5)
        low = hoop.conservative_clearance(np.zeros(3), np.eye(3) * 0.01**2, 0.1, 0.02, 2.0)
        high = hoop.conservative_clearance(np.zeros(3), np.eye(3) * 0.10**2, 0.1, 0.02, 2.0)
        self.assertGreater(low, high)


class ProfileTests(unittest.TestCase):
    def test_fallback_does_not_depend_on_participant_flight_code(self) -> None:
        self.assertEqual(FALLBACK.state_source, "onboard")
        self.assertEqual(FALLBACK.controller, "onboard_position")
        self.assertEqual(FALLBACK.speed_policy, "fixed")
        self.assertEqual(FALLBACK.crossing_policy, "centre")

    def test_profiles_are_selectable_by_name(self) -> None:
        self.assertIs(get_profile("fallback"), FALLBACK)
        self.assertIs(get_profile("estimation_only"), ESTIMATION_ONLY)
        self.assertIs(get_profile("full"), FULL)



if __name__ == "__main__":
    unittest.main()
