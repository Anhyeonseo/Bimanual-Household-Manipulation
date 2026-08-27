from __future__ import annotations

import numpy as np
import pytest

from tools.run.evaluate_towel_yolo_segmentation import (
    binary_mask_iou,
    summarize_records,
)


def test_binary_mask_iou_handles_overlap_and_two_empty_masks():
    expected = np.array([[1, 1], [0, 0]], dtype=np.uint8)
    predicted = np.array([[0, 1], [0, 1]], dtype=np.uint8)
    assert binary_mask_iou(expected, predicted) == pytest.approx(1 / 3)
    assert binary_mask_iou(np.zeros((2, 2)), np.zeros((2, 2))) == 1.0


def test_binary_mask_iou_rejects_shape_mismatch():
    with pytest.raises(ValueError, match="same shape"):
        binary_mask_iou(np.zeros((2, 2)), np.zeros((3, 2)))


def test_summary_reports_presence_empty_rejection_and_nonempty_iou():
    summary = summarize_records(
        [
            {
                "observation_id": "towel-a",
                "empty_expected": False,
                "prediction_count": 1,
                "mask_iou": 0.9,
            },
            {
                "observation_id": "towel-b",
                "empty_expected": False,
                "prediction_count": 0,
                "mask_iou": 0.0,
            },
            {
                "observation_id": "empty-a",
                "empty_expected": True,
                "prediction_count": 0,
                "mask_iou": 1.0,
            },
        ]
    )
    assert summary["towel_detected_count"] == 1
    assert summary["empty_rejected_count"] == 1
    assert summary["nonempty_mask_iou_mean"] == pytest.approx(0.45)
    assert summary["nonempty_mask_iou_min"] == 0.0
    assert summary["false_negative_observation_ids"] == ["towel-b"]
    assert summary["empty_false_positive_observation_ids"] == []
