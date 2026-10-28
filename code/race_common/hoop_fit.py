"""Fit circular or rectangular gates to three-dimensional outline samples."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class CircleFit3D:
    centre: np.ndarray
    normal: np.ndarray
    radius_m: float
    plane_rms_m: float
    radial_rms_m: float
    plane_max_m: float
    radial_max_m: float


@dataclass(frozen=True)
class RectangleFit3D:
    centre: np.ndarray
    normal: np.ndarray
    width_axis: np.ndarray
    height_axis: np.ndarray
    width_m: float
    height_m: float
    plane_rms_m: float
    outline_rms_m: float
    plane_max_m: float
    outline_max_m: float


def _fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    point_mean = np.mean(points, axis=0)
    centred = points - point_mean
    _left, singular_values, axes = np.linalg.svd(centred, full_matrices=False)
    if singular_values[1] <= max(1e-10, singular_values[0] * 1e-8):
        raise ValueError("points are collinear or too closely clustered")
    return point_mean, centred, axes


def _orient_normal(
    normal: np.ndarray,
    normal_reference: np.ndarray | None,
) -> np.ndarray:
    if normal_reference is not None:
        reference = np.asarray(normal_reference, dtype=float)
        if reference.shape != (3,) or not np.all(np.isfinite(reference)):
            raise ValueError("normal_reference must contain three finite values")
        if np.linalg.norm(reference) <= 1e-12:
            raise ValueError("normal_reference must be non-zero")
        return -normal if float(normal @ reference) < 0.0 else normal

    # Make unconstrained fits reproducible; physical crossing direction must
    # still be selected by the operator.
    dominant = int(np.argmax(np.abs(normal)))
    return -normal if normal[dominant] < 0.0 else normal


def fit_circle_3d(
    points: np.ndarray,
    normal_reference: np.ndarray | None = None,
) -> CircleFit3D:
    """Return the least-squares plane and circle through 3D rim samples.

    The normal sign is geometrically ambiguous. If ``normal_reference`` is
    supplied, the returned normal points into the same half-space.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        raise ValueError("points must have shape (N, 3) with N >= 3")
    if not np.all(np.isfinite(points)):
        raise ValueError("points must be finite")

    point_mean, centred, axes = _fit_plane(points)

    plane_axes = axes[:2]
    normal = axes[2]
    coordinates = centred @ plane_axes.T

    # In-plane algebraic least squares: (u-c_u)^2 + (v-c_v)^2 = r^2.
    design = np.column_stack((2.0 * coordinates, np.ones(points.shape[0])))
    target = np.sum(coordinates**2, axis=1)
    solution, _residuals, rank, _singular = np.linalg.lstsq(
        design, target, rcond=None
    )
    if rank < 3:
        raise ValueError("rim points do not define a unique circle")

    centre_2d = solution[:2]
    radius_squared = float(solution[2] + centre_2d @ centre_2d)
    if radius_squared <= 0.0:
        raise ValueError("fitted circle has a non-positive radius")

    centre = point_mean + centre_2d @ plane_axes
    radius_m = float(np.sqrt(radius_squared))
    plane_errors = (points - centre) @ normal
    radial_errors = np.linalg.norm(coordinates - centre_2d, axis=1) - radius_m

    normal = _orient_normal(normal, normal_reference)

    return CircleFit3D(
        centre=centre,
        normal=normal,
        radius_m=radius_m,
        plane_rms_m=float(np.sqrt(np.mean(plane_errors**2))),
        radial_rms_m=float(np.sqrt(np.mean(radial_errors**2))),
        plane_max_m=float(np.max(np.abs(plane_errors))),
        radial_max_m=float(np.max(np.abs(radial_errors))),
    )


def fit_rectangle_3d(
    points: np.ndarray,
    normal_reference: np.ndarray | None = None,
) -> RectangleFit3D:
    """Fit an orthogonal rectangle to samples distributed around its outline."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 4:
        raise ValueError("points must have shape (N, 3) with N >= 4")
    if not np.all(np.isfinite(points)):
        raise ValueError("points must be finite")

    point_mean, centred, axes = _fit_plane(points)
    plane_axes = axes[:2]
    coordinates = centred @ plane_axes.T

    def outline_distances(parameters: np.ndarray) -> np.ndarray:
        """Shortest in-plane distance from each sample to the rectangle rim."""
        centre_2d = parameters[:2]
        theta, half_width, half_height = parameters[2:]
        width_direction = np.array([np.cos(theta), np.sin(theta)])
        height_direction = np.array([-np.sin(theta), np.cos(theta)])
        relative = coordinates - centre_2d
        local_x = np.abs(relative @ width_direction)
        local_y = np.abs(relative @ height_direction)

        outside_x = np.maximum(local_x - half_width, 0.0)
        outside_y = np.maximum(local_y - half_height, 0.0)
        outside_distance = np.hypot(outside_x, outside_y)
        inside_distance = np.minimum(half_width - local_x, half_height - local_y)
        inside = (local_x <= half_width) & (local_y <= half_height)
        return np.where(inside, inside_distance, outside_distance)

    lower = np.array([-np.inf, -np.inf, -np.inf, 1e-8, 1e-8])
    upper = np.full(5, np.inf)

    # Rectangle orientation is periodic and the boundary-distance objective is
    # not globally smooth. Several inexpensive starting angles avoid selecting
    # a local minimum based on the arbitrary PCA basis.
    candidates = []
    for initial_theta in np.linspace(0.0, np.pi / 2.0, 12, endpoint=False):
        width_direction = np.array([np.cos(initial_theta), np.sin(initial_theta)])
        height_direction = np.array([-np.sin(initial_theta), np.cos(initial_theta)])
        along_width = coordinates @ width_direction
        along_height = coordinates @ height_direction
        width_limits = np.array([np.min(along_width), np.max(along_width)])
        height_limits = np.array([np.min(along_height), np.max(along_height)])
        centre_2d = (
            np.mean(width_limits) * width_direction
            + np.mean(height_limits) * height_direction
        )
        initial = np.array([
            centre_2d[0],
            centre_2d[1],
            initial_theta,
            np.ptp(width_limits) / 2.0,
            np.ptp(height_limits) / 2.0,
        ])
        result = least_squares(outline_distances, initial, bounds=(lower, upper))
        if result.success:
            candidates.append(result)

    if not candidates:
        raise ValueError("rectangle fit did not converge")
    result = min(candidates, key=lambda candidate: candidate.cost)

    centre_2d = result.x[:2]
    theta, half_width, half_height = result.x[2:]
    width_direction_2d = np.array([np.cos(theta), np.sin(theta)])
    height_direction_2d = np.array([-np.sin(theta), np.cos(theta)])
    width_axis = width_direction_2d @ plane_axes
    height_axis = height_direction_2d @ plane_axes
    geometric_normal = np.cross(width_axis, height_axis)
    geometric_normal /= np.linalg.norm(geometric_normal)
    normal = _orient_normal(geometric_normal, normal_reference)
    if float(normal @ geometric_normal) < 0.0:
        height_axis = -height_axis

    centre = point_mean + centre_2d @ plane_axes
    plane_errors = (points - centre) @ normal
    boundary_errors = outline_distances(result.x)
    outline_errors = np.hypot(plane_errors, boundary_errors)

    return RectangleFit3D(
        centre=centre,
        normal=normal,
        width_axis=width_axis,
        height_axis=height_axis,
        width_m=float(2.0 * half_width),
        height_m=float(2.0 * half_height),
        plane_rms_m=float(np.sqrt(np.mean(plane_errors**2))),
        outline_rms_m=float(np.sqrt(np.mean(outline_errors**2))),
        plane_max_m=float(np.max(np.abs(plane_errors))),
        outline_max_m=float(np.max(outline_errors)),
    )
