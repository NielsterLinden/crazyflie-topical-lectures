import unittest

import numpy as np

from .geometry import plane_basis, unit_vector
from .hoop_fit import fit_circle_3d, fit_rectangle_3d


class CircleFit3DTest(unittest.TestCase):
    def test_recovers_tilted_circle(self) -> None:
        centre = np.array([1.2, -0.4, 0.9])
        normal = unit_vector(np.array([1.0, 0.3, -0.2]))
        basis = plane_basis(normal)
        angles = np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)
        points = centre + 0.5 * (
            np.cos(angles)[:, None] * basis[:, 0]
            + np.sin(angles)[:, None] * basis[:, 1]
        )

        fit = fit_circle_3d(points, normal_reference=normal)

        np.testing.assert_allclose(fit.centre, centre, atol=1e-12)
        np.testing.assert_allclose(fit.normal, normal, atol=1e-12)
        self.assertAlmostEqual(fit.radius_m, 0.5)
        self.assertLess(fit.plane_rms_m, 1e-12)
        self.assertLess(fit.radial_rms_m, 1e-12)

    def test_normal_follows_reference_direction(self) -> None:
        points = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        fit = fit_circle_3d(points, normal_reference=np.array([-1.0, 0.0, 0.0]))

        self.assertGreater(float(fit.normal @ np.array([-1.0, 0.0, 0.0])), 0.0)

    def test_rejects_collinear_points(self) -> None:
        with self.assertRaisesRegex(ValueError, "collinear"):
            fit_circle_3d(np.array([
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ]))


class RectangleFit3DTest(unittest.TestCase):
    def test_recovers_tilted_rectangle_from_outline_samples(self) -> None:
        centre = np.array([1.5, 0.2, 1.1])
        normal = unit_vector(np.array([-0.3, 1.0, 0.2]))
        basis = plane_basis(normal)
        width_axis = basis[:, 0]
        height_axis = basis[:, 1]
        along_width = np.linspace(-0.6, 0.6, 10, endpoint=False)
        along_height = np.linspace(-0.4, 0.4, 8, endpoint=False)
        points = np.vstack([
            centre + along_width[:, None] * width_axis - 0.4 * height_axis,
            centre + 0.6 * width_axis + along_height[:, None] * height_axis,
            centre - along_width[:, None] * width_axis + 0.4 * height_axis,
            centre - 0.6 * width_axis - along_height[:, None] * height_axis,
        ])

        fit = fit_rectangle_3d(points, normal_reference=normal)

        np.testing.assert_allclose(fit.centre, centre, atol=1e-10)
        np.testing.assert_allclose(fit.normal, normal, atol=1e-10)
        np.testing.assert_allclose(
            np.cross(fit.width_axis, fit.height_axis), fit.normal, atol=1e-10
        )
        self.assertAlmostEqual(fit.width_m, 1.2)
        self.assertAlmostEqual(fit.height_m, 0.8)
        self.assertLess(fit.outline_rms_m, 1e-10)

    def test_requires_at_least_four_points(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            fit_rectangle_3d(np.zeros((3, 3)))


if __name__ == "__main__":
    unittest.main()
