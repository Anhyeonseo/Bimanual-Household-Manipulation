from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/run/plan_towel_fold_sequence_once.py"
SPEC = importlib.util.spec_from_file_location("plan_towel_fold_sequence_once", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_tcp_path_distance_is_zero_on_adjacent_task_chord():
    assert MODULE.point_segment_distance_m(
        (0.5, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.0)


def test_tcp_path_distance_measures_lateral_moveit_deviation():
    assert MODULE.point_segment_distance_m(
        (0.5, 0.003, 0.004), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)


def test_tcp_path_distance_clamps_before_and_after_segment():
    assert MODULE.point_segment_distance_m(
        (-0.003, 0.004, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)
    assert MODULE.point_segment_distance_m(
        (1.003, 0.004, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)


def test_tcp_path_distance_rejects_nonfinite_or_wrong_shape():
    with pytest.raises(RuntimeError, match="three finite XYZ"):
        MODULE.point_segment_distance_m((0.0, 0.0), (0.0,) * 3, (1.0,) * 3)
    with pytest.raises(RuntimeError, match="must be finite"):
        MODULE.point_segment_distance_m(
            (float("nan"), 0.0, 0.0), (0.0,) * 3, (1.0,) * 3
        )
