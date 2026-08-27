from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from tools.run.bootstrap_towel_yolo_assisted_review import (
    collect_unreviewed_paths,
    review_priority,
    union_towel_prediction_masks,
)
from tools.run.bootstrap_towel_segmentation_pilot import CATEGORY_ORDER


def test_union_towel_prediction_masks_ignores_other_classes_and_resizes():
    masks = np.array(
        [
            [[1.0, 0.0], [0.0, 0.0]],
            [[0.0, 1.0], [0.0, 0.0]],
        ],
        dtype=np.float32,
    )
    output, count, confidence = union_towel_prediction_masks(
        masks,
        np.array([0, 1]),
        np.array([0.8, 0.99]),
        width=4,
        height=4,
        mask_threshold=0.5,
    )
    assert output.shape == (4, 4)
    assert count == 1
    assert confidence == 0.8
    assert np.count_nonzero(output[:, :2]) > 0
    assert np.count_nonzero(output[:, 2:]) == 0


def test_review_priority_surfaces_uncertain_fallback_and_border_cases():
    assert review_priority(
        fallback_used=False,
        prediction_count=1,
        max_confidence=0.95,
        touches_border=False,
        area_ratio=0.25,
    ) == "low"
    assert review_priority(
        fallback_used=False,
        prediction_count=1,
        max_confidence=0.85,
        touches_border=False,
        area_ratio=0.25,
    ) == "medium"
    assert review_priority(
        fallback_used=True,
        prediction_count=0,
        max_confidence=0.0,
        touches_border=False,
        area_ratio=0.25,
    ) == "high"
    assert review_priority(
        fallback_used=False,
        prediction_count=1,
        max_confidence=0.99,
        touches_border=True,
        area_ratio=0.25,
    ) == "high"


def test_collect_unreviewed_paths_unions_multiple_review_roots(tmp_path):
    session = tmp_path / "session"
    for category in CATEGORY_ORDER:
        category_root = session / category
        category_root.mkdir(parents=True)
        for index in (1, 2):
            assert cv2.imwrite(
                str(category_root / f"frame_{index}.jpg"),
                np.full((8, 8, 3), 127, dtype=np.uint8),
            )

    review_roots = []
    for review_index, excluded_index in enumerate((1, 2)):
        review_root = tmp_path / f"review-{review_index}"
        annotation_root = review_root / "annotations"
        annotation_root.mkdir(parents=True)
        annotation = {
            "record_kind": "towel_observation_annotation",
            "source": {"image_path": f"01_flat/frame_{excluded_index}.jpg"},
        }
        (annotation_root / f"annotation-{review_index}.json").write_text(
            __import__("json").dumps(annotation), encoding="utf-8"
        )
        review_roots.append(review_root)

    selected, excluded = collect_unreviewed_paths(session, review_roots)
    assert excluded == {"01_flat/frame_1.jpg", "01_flat/frame_2.jpg"}
    assert selected["01_flat"] == []
    assert len(selected["02_light_wrinkle"]) == 2
