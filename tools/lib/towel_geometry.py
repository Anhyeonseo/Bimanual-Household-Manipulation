"""Pure geometry for square-towel observations and fold plans.

The module has no ROS, camera, simulator, or hardware dependency. Coordinates
are expressed in a right-handed workcell XY plane: +x is right and +y is top.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence

Point = tuple[float, float]
Polygon = tuple[Point, ...]
CORNER_LABELS = ("top_left", "top_right", "bottom_right", "bottom_left")


class TowelGeometryError(ValueError):
    """An observation cannot support a deterministic geometric decision."""


@dataclass(frozen=True, slots=True)
class QuadrilateralMetrics:
    ordered_corners: Polygon
    area: float
    centroid: Point
    edge_lengths: tuple[float, float, float, float]
    diagonal_lengths: tuple[float, float]
    edge_relative_spread: float
    diagonal_relative_difference: float
    axis_alignment_error_deg: float


@dataclass(frozen=True, slots=True)
class FoldSpec:
    axis: str
    direction: str
    fold_coordinate: float
    fold_line: tuple[Point, Point]
    moving_labels: tuple[str, str]
    stationary_labels: tuple[str, str]
    moving_start: tuple[Point, Point]
    moving_target: tuple[Point, Point]
    expected_footprint: Polygon
    expected_area_ratio: float


def _points(values: Iterable[Sequence[float]], *, minimum: int = 3) -> Polygon:
    points = tuple((float(value[0]), float(value[1])) for value in values)
    if len(points) < minimum:
        raise TowelGeometryError(f"at least {minimum} points are required")
    if not all(math.isfinite(value) for point in points for value in point):
        raise TowelGeometryError("all point coordinates must be finite")
    if len(set(points)) != len(points):
        raise TowelGeometryError("points must be unique")
    return points


def signed_polygon_area(points: Iterable[Sequence[float]]) -> float:
    polygon = _points(points)
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(
            polygon, polygon[1:] + polygon[:1], strict=True
        )
    )


def polygon_area(points: Iterable[Sequence[float]]) -> float:
    return abs(signed_polygon_area(points))


def polygon_centroid(points: Iterable[Sequence[float]]) -> Point:
    polygon = _points(points)
    twice_area = 2.0 * signed_polygon_area(polygon)
    if abs(twice_area) <= 1.0e-12:
        raise TowelGeometryError("polygon area must be nonzero")
    scale = 1.0 / (3.0 * twice_area)
    cross_terms = tuple(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(
            polygon, polygon[1:] + polygon[:1], strict=True
        )
    )
    center_x = scale * sum(
        (p0[0] + p1[0]) * cross
        for p0, p1, cross in zip(
            polygon, polygon[1:] + polygon[:1], cross_terms, strict=True
        )
    )
    center_y = scale * sum(
        (p0[1] + p1[1]) * cross
        for p0, p1, cross in zip(
            polygon, polygon[1:] + polygon[:1], cross_terms, strict=True
        )
    )
    return center_x, center_y


def order_square_corners(corners: Iterable[Sequence[float]]) -> Polygon:
    """Return TL, TR, BR, BL in workcell coordinates.

    The ordering is recomputed from every observation; corner identity is not
    persisted through a lift, rotation, or fold.
    """
    points = _points(corners, minimum=4)
    if len(points) != 4:
        raise TowelGeometryError("exactly four corner candidates are required")
    center = (
        sum(point[0] for point in points) / 4.0,
        sum(point[1] for point in points) / 4.0,
    )
    clockwise = sorted(
        points,
        key=lambda point: math.atan2(point[1] - center[1], point[0] - center[0]),
        reverse=True,
    )
    # y-x selects the workcell top-left corner. Coordinate ties are broken
    # deterministically so a symmetric 45-degree observation remains stable.
    start = max(
        range(4),
        key=lambda index: (
            clockwise[index][1] - clockwise[index][0],
            -clockwise[index][0],
            clockwise[index][1],
        ),
    )
    ordered = tuple(clockwise[(start + offset) % 4] for offset in range(4))
    if polygon_area(ordered) <= 1.0e-12:
        raise TowelGeometryError("corner candidates form a degenerate polygon")
    return ordered


def _axis_error_deg(vector: Point) -> float:
    angle = math.atan2(vector[1], vector[0])
    nearest = round(angle / (math.pi / 2.0)) * (math.pi / 2.0)
    error = abs(math.remainder(angle - nearest, math.pi))
    return math.degrees(min(error, math.pi - error))


def quadrilateral_metrics(
    corners: Iterable[Sequence[float]],
) -> QuadrilateralMetrics:
    ordered = order_square_corners(corners)
    edges = tuple(
        math.dist(start, end)
        for start, end in zip(ordered, ordered[1:] + ordered[:1], strict=True)
    )
    diagonals = (math.dist(ordered[0], ordered[2]), math.dist(ordered[1], ordered[3]))
    mean_edge = sum(edges) / 4.0
    mean_diagonal = sum(diagonals) / 2.0
    if mean_edge <= 1.0e-12 or mean_diagonal <= 1.0e-12:
        raise TowelGeometryError("corner geometry is degenerate")
    edge_spread = (max(edges) - min(edges)) / mean_edge
    diagonal_difference = abs(diagonals[0] - diagonals[1]) / mean_diagonal
    edge_vectors = tuple(
        (end[0] - start[0], end[1] - start[1])
        for start, end in zip(ordered, ordered[1:] + ordered[:1], strict=True)
    )
    return QuadrilateralMetrics(
        ordered_corners=ordered,
        area=polygon_area(ordered),
        centroid=polygon_centroid(ordered),
        edge_lengths=edges,  # type: ignore[arg-type]
        diagonal_lengths=diagonals,
        edge_relative_spread=edge_spread,
        diagonal_relative_difference=diagonal_difference,
        axis_alignment_error_deg=max(_axis_error_deg(vector) for vector in edge_vectors),
    )


def choose_first_fold_axis(costs: Mapping[str, float]) -> str:
    """Choose x or y deterministically from finite nonnegative plan costs."""
    if set(costs) != {"x", "y"}:
        raise TowelGeometryError("fold-axis costs must contain exactly x and y")
    normalized = {axis: float(cost) for axis, cost in costs.items()}
    if not all(math.isfinite(cost) and cost >= 0.0 for cost in normalized.values()):
        raise TowelGeometryError("fold-axis costs must be finite and nonnegative")
    return min(("x", "y"), key=lambda axis: (normalized[axis], axis))


def _rectangle(left: float, right: float, bottom: float, top: float) -> Polygon:
    if not left < right or not bottom < top:
        raise TowelGeometryError("rectangle bounds must have positive area")
    return ((left, top), (right, top), (right, bottom), (left, bottom))


def build_half_fold(
    corners: Iterable[Sequence[float]],
    axis: str,
    direction: str,
) -> FoldSpec:
    """Build one geometric half-fold for an aligned towel footprint."""
    metrics = quadrilateral_metrics(corners)
    ordered = metrics.ordered_corners
    labels = dict(zip(CORNER_LABELS, ordered, strict=True))
    xs = tuple(point[0] for point in ordered)
    ys = tuple(point[1] for point in ordered)
    left, right, bottom, top = min(xs), max(xs), min(ys), max(ys)
    center_x, center_y = metrics.centroid

    if axis == "x":
        valid = ("positive_to_negative", "negative_to_positive")
        if direction not in valid:
            raise TowelGeometryError(f"x fold direction must be one of {valid}")
        positive = direction == "positive_to_negative"
        moving = tuple(
            label for label, point in labels.items()
            if (point[0] > center_x) == positive
        )
        stationary = tuple(label for label in CORNER_LABELS if label not in moving)
        target = tuple((2.0 * center_x - labels[label][0], labels[label][1]) for label in moving)
        footprint = (
            _rectangle(left, center_x, bottom, top)
            if positive else _rectangle(center_x, right, bottom, top)
        )
        line = ((center_x, bottom), (center_x, top))
        coordinate = center_x
    elif axis == "y":
        valid = ("positive_to_negative", "negative_to_positive")
        if direction not in valid:
            raise TowelGeometryError(f"y fold direction must be one of {valid}")
        positive = direction == "positive_to_negative"
        moving = tuple(
            label for label, point in labels.items()
            if (point[1] > center_y) == positive
        )
        stationary = tuple(label for label in CORNER_LABELS if label not in moving)
        target = tuple((labels[label][0], 2.0 * center_y - labels[label][1]) for label in moving)
        footprint = (
            _rectangle(left, right, bottom, center_y)
            if positive else _rectangle(left, right, center_y, top)
        )
        line = ((left, center_y), (right, center_y))
        coordinate = center_y
    else:
        raise TowelGeometryError("fold axis must be x or y")

    if len(moving) != 2 or len(stationary) != 2:
        raise TowelGeometryError(
            "corners must straddle the selected centerline as two pairs"
        )
    return FoldSpec(
        axis=axis,
        direction=direction,
        fold_coordinate=coordinate,
        fold_line=line,
        moving_labels=moving,  # type: ignore[arg-type]
        stationary_labels=stationary,  # type: ignore[arg-type]
        moving_start=tuple(labels[label] for label in moving),  # type: ignore[arg-type]
        moving_target=target,  # type: ignore[arg-type]
        expected_footprint=footprint,
        expected_area_ratio=0.5,
    )


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _line_intersection(a: Point, b: Point, c: Point, d: Point) -> Point:
    ab = (b[0] - a[0], b[1] - a[1])
    cd = (d[0] - c[0], d[1] - c[1])
    denominator = ab[0] * cd[1] - ab[1] * cd[0]
    if abs(denominator) <= 1.0e-12:
        return b
    ac = (c[0] - a[0], c[1] - a[1])
    scale = (ac[0] * cd[1] - ac[1] * cd[0]) / denominator
    return a[0] + scale * ab[0], a[1] + scale * ab[1]


def convex_polygon_intersection(
    subject: Iterable[Sequence[float]],
    clip: Iterable[Sequence[float]],
) -> Polygon:
    """Intersect two convex polygons with Sutherland-Hodgman clipping."""
    output = list(_points(subject))
    clip_polygon = _points(clip)
    orientation = 1.0 if signed_polygon_area(clip_polygon) > 0.0 else -1.0
    for clip_start, clip_end in zip(
        clip_polygon, clip_polygon[1:] + clip_polygon[:1], strict=True
    ):
        input_points = output
        output = []
        if not input_points:
            break
        previous = input_points[-1]
        previous_inside = orientation * _cross(clip_start, clip_end, previous) >= -1e-12
        for current in input_points:
            current_inside = orientation * _cross(clip_start, clip_end, current) >= -1e-12
            if current_inside:
                if not previous_inside:
                    output.append(
                        _line_intersection(previous, current, clip_start, clip_end)
                    )
                output.append(current)
            elif previous_inside:
                output.append(
                    _line_intersection(previous, current, clip_start, clip_end)
                )
            previous = current
            previous_inside = current_inside
    return tuple(output)


def polygon_iou(
    first: Iterable[Sequence[float]],
    second: Iterable[Sequence[float]],
) -> float:
    first_polygon = _points(first)
    second_polygon = _points(second)
    intersection = convex_polygon_intersection(first_polygon, second_polygon)
    intersection_area = polygon_area(intersection) if len(intersection) >= 3 else 0.0
    union = polygon_area(first_polygon) + polygon_area(second_polygon) - intersection_area
    if union <= 1.0e-12:
        raise TowelGeometryError("polygon union must have positive area")
    return intersection_area / union
