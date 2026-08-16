"""Tests for multi-position Top tabletop calibration assembly."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "assemble_top_base_table_session.py"
SPEC = importlib.util.spec_from_file_location(
    "assemble_top_base_table_session", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.path.insert(0, str(ROOT / "tools"))
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_minimal_rotation_maps_normal_with_proper_rotation():
    source = np.asarray([0.1, -0.2, 0.97], dtype=np.float64)
    source /= np.linalg.norm(source)
    rotation = MODULE.minimal_rotation(source, np.asarray([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(
        rotation @ source,
        np.asarray([0.0, 0.0, 1.0]),
        atol=1e-12,
    )
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_common_plane_fit_flattens_tilt_at_configured_height():
    x_values, y_values = np.meshgrid(
        np.linspace(0.2, 0.7, 7), np.linspace(-0.4, 0.1, 6)
    )
    z_values = 0.02 * x_values - 0.01 * y_values - 0.015
    points = np.column_stack(
        (x_values.reshape(-1), y_values.reshape(-1), z_values.reshape(-1))
    )
    correction, quality = MODULE.fit_common_plane(points, -0.005)
    corrected = (
        correction @ np.column_stack((points, np.ones(len(points)))).T
    ).T[:, :3]
    np.testing.assert_allclose(corrected[:, 2], -0.005, atol=1e-12)
    np.testing.assert_allclose(
        corrected.mean(axis=0)[:2], points.mean(axis=0)[:2], atol=1e-12
    )
    assert quality["plane_fit_rms_mm"] == pytest.approx(0.0, abs=1e-9)


def test_metric_error_allows_gridboard_normal_reflection_without_scaling():
    source = np.asarray(
        [[0.0, 0.0], [0.1, 0.0], [0.1, 0.2], [0.0, 0.2]],
        dtype=np.float64,
    )
    reflection = np.asarray([[0.0, 1.0], [1.0, 0.0]], dtype=np.float64)
    target = source @ reflection.T + np.asarray([0.45, -0.12])
    errors = MODULE.orthogonal_metric_errors_mm(source, target)
    np.testing.assert_allclose(errors, 0.0, atol=1e-10)


def test_largest_axis_aligned_rectangle_recovers_rectangular_coverage():
    points = np.asarray(
        [[0.2, -0.4], [0.7, -0.4], [0.7, 0.1], [0.2, 0.1]],
        dtype=np.float64,
    )
    rectangle = MODULE.largest_axis_aligned_rectangle(points, samples=51)
    np.testing.assert_allclose(rectangle, [0.2, -0.4, 0.7, 0.1], atol=1e-7)
