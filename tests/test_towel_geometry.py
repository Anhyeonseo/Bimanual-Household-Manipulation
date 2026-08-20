from __future__ import annotations

import math

import pytest

from tools.lib.towel_geometry import (
    TowelGeometryError,
    build_half_fold,
    choose_first_fold_axis,
    order_square_corners,
    polygon_area,
    polygon_centroid,
    polygon_iou,
    quadrilateral_metrics,
)


SQUARE = ((-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0))


def rotate(points, degrees):
    angle = math.radians(degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return tuple(
        (cosine * x - sine * y, sine * x + cosine * y)
        for x, y in points
    )


def test_polygon_area_and_centroid_are_order_invariant_for_cyclic_order():
    assert polygon_area(SQUARE) == pytest.approx(4.0)
    assert polygon_centroid(SQUARE) == pytest.approx((0.0, 0.0))
    reversed_square = tuple(reversed(SQUARE))
    assert polygon_area(reversed_square) == pytest.approx(4.0)
    assert polygon_centroid(reversed_square) == pytest.approx((0.0, 0.0))


def test_corner_order_is_recomputed_as_tl_tr_br_bl():
    shuffled = (SQUARE[2], SQUARE[0], SQUARE[3], SQUARE[1])
    assert order_square_corners(shuffled) == SQUARE


def test_rotated_square_metrics_preserve_shape_and_report_alignment_error():
    metrics = quadrilateral_metrics(rotate(SQUARE, 17.0))
    assert metrics.area == pytest.approx(4.0)
    assert metrics.edge_relative_spread == pytest.approx(0.0, abs=1e-12)
    assert metrics.diagonal_relative_difference == pytest.approx(0.0, abs=1e-12)
    assert metrics.axis_alignment_error_deg == pytest.approx(17.0)


def test_two_orthogonal_half_folds_reduce_footprint_to_one_quarter():
    first = build_half_fold(SQUARE, "x", "positive_to_negative")
    second = build_half_fold(
        first.expected_footprint, "y", "positive_to_negative"
    )
    assert polygon_area(first.expected_footprint) == pytest.approx(2.0)
    assert polygon_area(second.expected_footprint) == pytest.approx(1.0)
    assert first.moving_labels == ("top_right", "bottom_right")
    assert first.moving_target[0] == pytest.approx((-1.0, 1.0))
    assert first.moving_target[1] == pytest.approx((-1.0, -1.0))


def test_fold_axis_selection_is_cost_based_and_deterministic():
    assert choose_first_fold_axis({"x": 2.0, "y": 1.0}) == "y"
    assert choose_first_fold_axis({"x": 1.0, "y": 1.0}) == "x"
    with pytest.raises(TowelGeometryError):
        choose_first_fold_axis({"x": float("nan"), "y": 1.0})


def test_polygon_iou_handles_partial_and_disjoint_overlap():
    shifted = tuple((x + 1.0, y) for x, y in SQUARE)
    assert polygon_iou(SQUARE, shifted) == pytest.approx(1.0 / 3.0)
    far = tuple((x + 10.0, y) for x, y in SQUARE)
    assert polygon_iou(SQUARE, far) == 0.0


def test_degenerate_corner_inputs_fail_closed():
    with pytest.raises(TowelGeometryError):
        order_square_corners(((0, 0), (1, 0), (1, 0), (0, 1)))
