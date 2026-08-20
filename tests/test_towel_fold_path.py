from __future__ import annotations

import math

import pytest

from tools.lib.towel_fold_path import build_geometric_fold_arc
from tools.lib.towel_geometry import TowelGeometryError, build_half_fold


SQUARE = ((-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0))


@pytest.mark.parametrize("axis", ("x", "y"))
@pytest.mark.parametrize(
    "direction", ("positive_to_negative", "negative_to_positive")
)
def test_arc_starts_and_ends_at_fold_spec(axis, direction):
    fold = build_half_fold(SQUARE, axis, direction)
    arc = build_geometric_fold_arc(fold, sample_count=5)
    for actual, expected in zip(
        arc[0].moving_points_xyz, fold.moving_start, strict=True
    ):
        assert actual[:2] == pytest.approx(expected)
    for actual, expected in zip(
        arc[-1].moving_points_xyz, fold.moving_target, strict=True
    ):
        assert actual[:2] == pytest.approx(expected)
    assert all(point[2] == pytest.approx(1.0) for point in arc[2].moving_points_xyz)


def test_arc_preserves_distance_between_moving_corners():
    fold = build_half_fold(SQUARE, "x", "positive_to_negative")
    arc = build_geometric_fold_arc(fold, sample_count=17)
    distances = [math.dist(*waypoint.moving_points_xyz) for waypoint in arc]
    assert distances == pytest.approx([2.0] * len(arc))


@pytest.mark.parametrize("sample_count", (True, 2, 2.5))
def test_invalid_sample_count_fails_closed(sample_count):
    fold = build_half_fold(SQUARE, "x", "positive_to_negative")
    with pytest.raises(TowelGeometryError, match="sample_count"):
        build_geometric_fold_arc(fold, sample_count=sample_count)
