"""Motion-free geometric paths for a two-corner towel half-fold.

The generated arcs are design artifacts, not robot trajectories. They contain
no timing, joint state, controller call, collision result, or reachability
claim. Coordinates use the same workcell frame as :mod:`towel_geometry`.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from tools.lib.towel_geometry import FoldSpec, Point, TowelGeometryError


@dataclass(frozen=True, slots=True)
class DualFoldWaypoint:
    progress: float
    moving_points_xyz: tuple[tuple[float, float, float], tuple[float, float, float]]


def _point_on_arc(
    point: Point,
    *,
    axis: str,
    fold_coordinate: float,
    direction_sign: float,
    radius: float,
    angle: float,
) -> tuple[float, float, float]:
    folded_coordinate = fold_coordinate + direction_sign * radius * math.cos(angle)
    height = radius * math.sin(angle)
    if axis == "x":
        return folded_coordinate, point[1], height
    return point[0], folded_coordinate, height


def build_geometric_fold_arc(
    fold: FoldSpec,
    *,
    sample_count: int = 9,
) -> tuple[DualFoldWaypoint, ...]:
    """Sample a synchronized semicircle for the two moving towel corners.

    The arc preserves the distance between the moving corners and reaches the
    reflected geometric targets. It deliberately performs no robot-specific
    feasibility check.
    """
    if not isinstance(sample_count, int) or isinstance(sample_count, bool):
        raise TowelGeometryError("sample_count must be an integer")
    if sample_count < 3:
        raise TowelGeometryError("sample_count must be at least three")
    if fold.axis not in {"x", "y"}:
        raise TowelGeometryError("fold axis must be x or y")
    if fold.direction not in {"positive_to_negative", "negative_to_positive"}:
        raise TowelGeometryError("invalid fold direction")

    direction_sign = 1.0 if fold.direction == "positive_to_negative" else -1.0
    axis_index = 0 if fold.axis == "x" else 1
    radii = tuple(
        abs(point[axis_index] - fold.fold_coordinate)
        for point in fold.moving_start
    )
    if any(radius <= 1.0e-12 for radius in radii):
        raise TowelGeometryError("moving corners must not lie on the fold line")
    if not math.isclose(radii[0], radii[1], rel_tol=1.0e-9, abs_tol=1.0e-12):
        raise TowelGeometryError("moving corners must have equal fold radii")

    waypoints = []
    for index in range(sample_count):
        progress = index / (sample_count - 1)
        angle = math.pi * progress
        points = tuple(
            _point_on_arc(
                point,
                axis=fold.axis,
                fold_coordinate=fold.fold_coordinate,
                direction_sign=direction_sign,
                radius=radius,
                angle=angle,
            )
            for point, radius in zip(fold.moving_start, radii, strict=True)
        )
        waypoints.append(
            DualFoldWaypoint(
                progress=progress,
                moving_points_xyz=points,  # type: ignore[arg-type]
            )
        )

    for point, expected in zip(
        waypoints[-1].moving_points_xyz, fold.moving_target, strict=True
    ):
        if not all(
            math.isclose(actual, target, abs_tol=1.0e-9)
            for actual, target in zip(point[:2], expected, strict=True)
        ):
            raise TowelGeometryError("fold arc did not reach its target")
    return tuple(waypoints)
