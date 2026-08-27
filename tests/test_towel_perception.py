from __future__ import annotations

from pathlib import Path

import pytest

from tools.lib.towel_dataset import load_annotation
from tools.lib.towel_perception import (
    blue_towel_image_observation,
    blue_towel_mask_candidate,
    inspect_binary_mask,
    inspect_blue_towel_evidence,
    mask_shape_features,
    metric_fold_outline_match,
    prerecorded_annotation_observation,
    propose_blue_towel_mask,
    project_pixel_to_workcell,
    project_raw_pixel_to_workcell,
    rasterize_annotation_mask,
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


def test_raw_pixel_is_rectified_before_plane_projection():
    import numpy as np

    identity = np.eye(3)
    assert project_raw_pixel_to_workcell(
        (320, 240),
        camera_matrix=identity,
        distortion_coefficients=(0, 0, 0, 0, 0),
        projection_matrix=identity,
        rectified_pixel_to_workcell_homography=HOMOGRAPHY,
    ) == pytest.approx((0.0, 0.0))


def test_prerecorded_backend_bridges_annotation_to_aligned_state():
    document = convert()
    observation = TowelObservation.from_dict(document)
    estimate = estimate_towel_state(
        observation, PerceptionLimits.from_contract(CONTRACT)
    )
    assert estimate.state == TowelState.ALIGNED
    assert document["backend"] == "prerecorded_reviewed_polygon_v1"
    assert document["visible_area_ratio"] == pytest.approx(1.0)
    assert document["clear_view_valid"] is True


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


def test_mask_diagnostics_report_components_area_and_border_contact():
    import numpy as np

    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[0:5, 2:8] = 1
    mask[10:12, 20:23] = 1
    diagnostics = inspect_binary_mask(mask)
    assert diagnostics.component_count == 2
    assert diagnostics.foreground_area_ratio == pytest.approx(36 / 600)
    assert diagnostics.largest_component_area_ratio == pytest.approx(30 / 600)
    assert diagnostics.secondary_component_area_ratio == pytest.approx(6 / 600)
    assert diagnostics.touches_frame_border is True


def test_reviewed_annotation_rasterizes_for_the_same_mask_gate():
    mask = rasterize_annotation_mask(ANNOTATION)
    diagnostics = inspect_binary_mask(mask)
    assert mask.shape == (480, 640)
    assert diagnostics.component_count == 1
    assert diagnostics.foreground_area_ratio > 0.0
    assert diagnostics.touches_frame_border is False


def test_prerecorded_backend_rejects_border_touch_and_occluded_clear_view():
    border = dict(ANNOTATION)
    border["segmentation_polygon_px"] = [
        [0.0, 100.0], [500.0, 100.0], [500.0, 400.0], [0.0, 400.0]
    ]
    document = prerecorded_annotation_observation(
        border,
        pixel_to_workcell_homography=HOMOGRAPHY,
        calibration_sha256="c" * 64,
        expected_full_mask_area_px=90000.0,
        topology_confidence=0.95,
    )
    assert document["clear_view_valid"] is False

    occluded = dict(ANNOTATION)
    occluded["occluded"] = True
    document = prerecorded_annotation_observation(
        occluded,
        pixel_to_workcell_homography=HOMOGRAPHY,
        calibration_sha256="c" * 64,
        expected_full_mask_area_px=90000.0,
        topology_confidence=0.95,
    )
    assert document["clear_view_valid"] is False


def test_blue_presence_gate_rejects_empty_table_before_grabcut():
    import cv2
    import numpy as np

    image = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.line(image, (0, 200), (639, 240), (180, 170, 165), 5)
    evidence = inspect_blue_towel_evidence(image)
    mask = propose_blue_towel_mask(image)
    candidate = blue_towel_mask_candidate(image)
    assert evidence.towel_present is False
    assert np.count_nonzero(mask) == 0
    assert candidate.clear_view_valid is False
    assert candidate.rejection_reason == "towel_not_present"


def test_blue_candidate_keeps_towel_and_rejects_border_contact():
    import cv2
    import numpy as np

    centered = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(centered, (150, 100), (500, 390), (190, 150, 125), -1)
    candidate = blue_towel_mask_candidate(centered)
    assert candidate.blue_evidence.towel_present is True
    assert candidate.mask_diagnostics.component_count == 1
    assert candidate.clear_view_valid is True
    assert candidate.rejection_reason is None

    clipped = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(clipped, (0, 100), (500, 390), (190, 150, 125), -1)
    candidate = blue_towel_mask_candidate(clipped)
    assert candidate.blue_evidence.towel_present is True
    assert candidate.conservative_border_contact is True
    assert candidate.clear_view_valid is False
    assert candidate.rejection_reason == "towel_evidence_touches_frame_border"


def test_mask_shape_backend_promotes_only_a_clear_quadrilateral_outline():
    import cv2
    import numpy as np

    image = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(image, (190, 110), (440, 360), (190, 150, 125), -1)
    candidate = blue_towel_mask_candidate(image)
    shape = mask_shape_features(candidate.mask)
    assert shape.simplified_vertex_count == 4
    assert len(shape.outline_quadrilateral_px) == 4
    assert shape.topology_confidence >= 0.8

    document = blue_towel_image_observation(
        image,
        observation_id="image-flat-1",
        source_sha256="a" * 64,
        calibration_sha256="c" * 64,
        camera_matrix=np.eye(3),
        distortion_coefficients=(0, 0, 0, 0, 0),
        projection_matrix=np.eye(3),
        rectified_pixel_to_workcell_homography=HOMOGRAPHY,
        expected_full_towel_area_m2=0.25 * 0.25,
    )
    observation = TowelObservation.from_dict(document)
    estimate = estimate_towel_state(
        observation, PerceptionLimits.from_contract(CONTRACT)
    )
    assert document["backend"] == "blue_towel_grabcut_outline_v1"
    assert document["fold_count"] == 0
    assert estimate.state == TowelState.ALIGNED


def test_nonquadrilateral_outline_stays_below_topology_gate():
    import cv2
    import numpy as np

    mask = np.zeros((480, 640), dtype=np.uint8)
    points = np.array(
        [[130, 120], [500, 100], [530, 300], [400, 390], [220, 330]],
        dtype=np.int32,
    )
    cv2.fillPoly(mask, [points], 255)
    shape = mask_shape_features(mask)
    assert shape.simplified_vertex_count != 4
    assert shape.outline_quadrilateral_px == ()
    assert shape.topology_confidence < 0.8


def test_metric_fold_outline_match_scores_nominal_first_and_second_fold():
    first = metric_fold_outline_match(
        ((0.0, 0.0), (0.304, 0.0), (0.304, 0.148), (0.0, 0.148)),
        expected_fold_count=1,
        unfolded_towel_size_m=(0.304, 0.296),
    )
    second = metric_fold_outline_match(
        ((0.0, 0.0), (0.152, 0.0), (0.152, 0.148), (0.0, 0.148)),
        expected_fold_count=2,
        unfolded_towel_size_m=(0.304, 0.296),
    )
    assert first.normalized_metric_outline_iou > 0.99
    assert second.normalized_metric_outline_iou > 0.99
    assert first.target_long_side_m == pytest.approx(0.304)
    assert second.target_long_side_m == pytest.approx(0.152)


def test_fold_postcondition_requires_explicit_action_context():
    import cv2
    import numpy as np

    image = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(image, (170, 170), (470, 315), (190, 150, 125), -1)
    with pytest.raises(ValueError, match="verified fold action context"):
        blue_towel_image_observation(
            image,
            observation_id="fold-without-context",
            source_sha256="a" * 64,
            calibration_sha256="c" * 64,
            camera_matrix=np.eye(3),
            distortion_coefficients=(0, 0, 0, 0, 0),
            projection_matrix=np.eye(3),
            rectified_pixel_to_workcell_homography=HOMOGRAPHY,
            expected_full_towel_area_m2=0.304 * 0.296,
            expected_fold_count=1,
            unfolded_towel_size_m=(0.304, 0.296),
        )


def test_verified_fold_context_can_authorize_only_the_expected_fold_count():
    import cv2
    import numpy as np

    image = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(image, (168, 170), (472, 318), (190, 150, 125), -1)
    document = blue_towel_image_observation(
        image,
        observation_id="verified-first-fold",
        source_sha256="a" * 64,
        calibration_sha256="c" * 64,
        camera_matrix=np.eye(3),
        distortion_coefficients=(0, 0, 0, 0, 0),
        projection_matrix=np.eye(3),
        rectified_pixel_to_workcell_homography=HOMOGRAPHY,
        expected_full_towel_area_m2=0.304 * 0.296,
        expected_fold_count=1,
        fold_action_context_verified=True,
        unfolded_towel_size_m=(0.304, 0.296),
    )
    assert document["fold_count"] == 1
    assert document["outline_iou"] > 0.95
    assert document["fold_postcondition"]["expected_fold_count"] == 1
