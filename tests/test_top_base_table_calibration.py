"""Tests for current tabletop-to-base calibration."""

import importlib.util
import sys
from pathlib import Path

import cv2

import numpy as np

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'tools' / 'calibrate_top_base_table.py'
SPEC = importlib.util.spec_from_file_location(
    'calibrate_top_base_table',
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def downward_camera():
    """Return a camera above a horizontal plane looking down."""
    transform = np.eye(4)
    transform[:3, :3] = np.diag([1.0, -1.0, -1.0])
    transform[:3, 3] = [0.0, 0.0, 1.0]
    return transform


def test_base_plane_homography_matches_ray_intersection():
    """The homography must match known ray-plane intersections."""
    projection = np.asarray(
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
    )
    homography = MODULE.base_plane_homography(
        projection,
        downward_camera(),
        0.0,
        np.asarray([0.20, -0.30]),
    )
    actual = MODULE.transform_pixels(
        np.asarray([[320.0, 240.0], [370.0, 290.0]]),
        homography,
    )
    np.testing.assert_allclose(
        actual,
        np.asarray([[-0.20, 0.30], [-0.10, 0.20]]),
        atol=1e-12,
    )


def test_gridboard_contract_uses_ids_10_through_29():
    """The tabletop board must use IDs distinct from the TCP board."""
    _, board, ids = MODULE.planar_gridboard()
    assert ids == tuple(range(10, 30))
    points = np.concatenate(board.objPoints, axis=0)
    assert points[:, 0].min() == pytest.approx(0.0)
    assert points[:, 1].min() == pytest.approx(0.0)
    assert points[:, 0].max() == pytest.approx(0.095)
    assert points[:, 1].max() == pytest.approx(0.120)


def test_plane_evaluation_accepts_flat_expected_height():
    """A flat board at the expected height must pass plane metrics."""
    base_from_grid = np.eye(4)
    base_from_grid[2, 3] = -0.005
    points = np.asarray(
        [[0.0, 0.0, 0.0], [0.095, 0.0, 0.0], [0.0, 0.120, 0.0]]
    )
    result = MODULE.evaluate_plane(base_from_grid, points, -0.005)
    assert result['tilt_deg'] == pytest.approx(0.0)
    assert result['height_error_max_mm'] == pytest.approx(0.0)


@pytest.mark.parametrize(
    ('values', 'reason'),
    [
        ((1.501, 20.0, 0.0, 0.0), 'PnP reprojection RMS'),
        ((0.5, 9.9, 0.0, 0.0), 'image border'),
        ((0.5, 20.0, 3.001, 0.0), 'plane tilt'),
        ((0.5, 20.0, 0.0, 8.001), 'height differs'),
    ],
)
def test_classification_fails_closed(values, reason):
    """Each geometric threshold must reject independently."""
    status, failures = MODULE.classify(*values)
    assert status == 'REJECTED_TABLE_BASE_CALIBRATION'
    assert any(reason in failure for failure in failures)


def test_rendered_gridboard_is_detected_with_complete_id_set(tmp_path):
    """A complete synthetic board must expose all twenty IDs."""
    dictionary, board, _ = MODULE.planar_gridboard()
    assert dictionary is not None
    image = board.draw((760, 960), marginSize=80, borderBits=1)
    bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    camera = np.asarray(
        [[800.0, 0.0, 380.0], [0.0, 800.0, 480.0], [0.0, 0.0, 1.0]]
    )
    result = MODULE.detect_gridboard(bgr, camera, np.zeros(5))
    assert result['detected_ids'] == tuple(range(10, 30))
    assert result['detection_scale'] == pytest.approx(2.0)
    assert result['pnp_rms_px'] < 1.5
