from __future__ import annotations

from pathlib import Path

import pytest

from tools.lib.towel_dataset import load_annotation
from tools.lib.towel_perception import (
    prerecorded_annotation_observation,
    project_pixel_to_workcell,
)
from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    TowelObservation,
    TowelState,
    estimate_towel_state,
    load_towel_contract,
)


ROOT = Path(__file__).resolve().parents[1]
ANNOTATION = load_annotation(ROOT / "config/towel_annotation.example.json")
CONTRACT = load_towel_contract(
    ROOT / "config/towel_task_contract.candidate.yaml"
)
HOMOGRAPHY = (
    (0.001, 0.0, -0.32),
    (0.0, -0.001, 0.24),
    (0.0, 0.0, 1.0),
)


def convert(**changes):
    values = {
        "pixel_to_workcell_homography": HOMOGRAPHY,
        "calibration_sha256": "c" * 64,
        "expected_full_mask_area_px": 90000.0,
        "topology_confidence": 0.95,
        "fold_axis_costs": {"x": 1.0, "y": 1.1},
        "fold_direction_costs": {
            "x_positive_to_negative": 1.0,
            "x_negative_to_positive": 1.1,
            "y_positive_to_negative": 1.0,
            "y_negative_to_positive": 1.1,
        },
    }
    values.update(changes)
    return prerecorded_annotation_observation(ANNOTATION, **values)


def test_homography_projects_pixels_into_workcell_coordinates():
    assert project_pixel_to_workcell((320, 240), HOMOGRAPHY) == pytest.approx(
        (0.0, 0.0)
    )


def test_prerecorded_backend_bridges_annotation_to_aligned_state():
    document = convert()
    observation = TowelObservation.from_dict(document)
    estimate = estimate_towel_state(
        observation, PerceptionLimits.from_contract(CONTRACT)
    )
    assert estimate.state == TowelState.ALIGNED
    assert document["backend"] == "prerecorded_reviewed_polygon_v1"
    assert document["visible_area_ratio"] == pytest.approx(1.0)


def test_low_topology_confidence_never_becomes_aligned():
    observation = TowelObservation.from_dict(convert(topology_confidence=0.2))
    estimate = estimate_towel_state(
        observation, PerceptionLimits.from_contract(CONTRACT)
    )
    assert estimate.state == TowelState.AMBIGUOUS


def test_invalid_homography_and_expected_area_fail_closed():
    with pytest.raises(ValueError, match="invertible"):
        convert(pixel_to_workcell_homography=((0, 0, 0),) * 3)
    with pytest.raises(ValueError, match="positive"):
        convert(expected_full_mask_area_px=0.0)
