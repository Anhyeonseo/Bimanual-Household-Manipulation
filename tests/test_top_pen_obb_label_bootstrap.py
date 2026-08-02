import importlib.util
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "bootstrap_top_pen_obb_labels.py"
SPEC = importlib.util.spec_from_file_location("bootstrap_top_pen_obb_labels", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_dim_fallback_finds_rotated_pen_on_dark_frame():
    reference = np.full((480, 640, 3), 130, dtype=np.uint8)
    image = np.full((480, 640, 3), 110, dtype=np.uint8)
    expected = ((360.0, 260.0), (150.0, 24.0), 28.0)
    cv2.fillConvexPoly(
        image,
        np.round(cv2.boxPoints(expected)).astype(np.int32),
        (20, 20, 20),
    )

    candidates = MODULE.candidate_rectangles(image, reference)

    assert candidates
    points = candidates[0]["points"]
    center = np.mean(points, axis=0)
    assert np.linalg.norm(center - np.array([360.0, 260.0])) < 5.0
    assert 130.0 <= candidates[0]["long_side_px"] <= 165.0
    assert 18.0 <= candidates[0]["short_side_px"] <= 32.0


def test_label_text_normalizes_four_corners():
    points = np.array(
        [[64.0, 48.0], [128.0, 48.0], [128.0, 96.0], [64.0, 96.0]],
        dtype=np.float32,
    )

    tokens = MODULE.label_text(points, width=640, height=480).split()

    assert tokens[0] == "0"
    assert [float(value) for value in tokens[1:]] == [
        0.1,
        0.1,
        0.2,
        0.1,
        0.2,
        0.2,
        0.1,
        0.2,
    ]


def test_choose_reference_prefers_train_split():
    positive = {
        "id": "positive",
        "condition": {"background": "board", "lighting": "dim"},
    }
    negatives = [
        {
            "id": "validation_negative",
            "split": "validation",
            "condition": {"background": "board", "lighting": "dim"},
        },
        {
            "id": "train_negative",
            "split": "train",
            "condition": {"background": "board", "lighting": "dim"},
        },
    ]

    assert MODULE.choose_reference(positive, negatives)["id"] == "train_negative"
