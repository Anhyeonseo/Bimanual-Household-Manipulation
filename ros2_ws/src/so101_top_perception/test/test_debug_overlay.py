from __future__ import annotations

import cv2
import numpy as np

from so101_top_perception.debug_overlay import (
    PASS_COLOR,
    REJECT_COLOR,
    render_debug_overlay,
)


def pose() -> dict:
    return {
        "raw_corners_px": [
            [30.0, 50.0],
            [150.0, 50.0],
            [150.0, 80.0],
            [30.0, 80.0],
        ],
        "raw_center_px": [90.0, 65.0],
        "board_position_m": [0.42, -0.06],
        "yaw_rad": 0.1,
        "confidence": 0.93,
    }


def contains_color(image: np.ndarray, color: tuple[int, int, int]) -> bool:
    return bool(np.any(np.all(image == np.asarray(color), axis=2)))


def test_pass_overlay_draws_obb_axis_center_and_status() -> None:
    source = np.full((120, 200, 3), 80, dtype=np.uint8)
    rendered = render_debug_overlay(
        source,
        pose=pose(),
        code="TRACKING_BOARD_ONLY",
        reason="one valid observation",
        frame_age_s=0.04,
        detector_backend="opencv_dnn_ultralytics_obb",
    )
    assert rendered.shape == source.shape
    assert not np.shares_memory(rendered, source)
    assert contains_color(rendered, PASS_COLOR)
    assert int(cv2.absdiff(rendered, source).sum()) > 0


def test_reject_overlay_marks_frame_red_without_fabricating_a_box() -> None:
    source = np.full((120, 200, 3), 80, dtype=np.uint8)
    rendered = render_debug_overlay(
        source,
        pose=None,
        code="OBJECT_COUNT_INVALID",
        reason="detected 0",
        frame_age_s=0.03,
        detector_backend="opencv_dnn_ultralytics_obb",
    )
    assert contains_color(rendered, REJECT_COLOR)
    # The lower image remains untouched when no OBB exists.
    assert np.array_equal(rendered[90:, :, :], source[90:, :, :])


def test_overlay_rejects_invalid_image_shape() -> None:
    invalid = np.zeros((120, 200), dtype=np.uint8)
    try:
        render_debug_overlay(
            invalid,
            pose=None,
            code="INVALID",
            reason="bad image",
            frame_age_s=0.0,
            detector_backend="test",
        )
    except ValueError as error:
        assert "HxWx3" in str(error)
    else:
        raise AssertionError("invalid image shape was accepted")
