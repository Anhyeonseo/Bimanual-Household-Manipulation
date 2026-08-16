"""Render a read-only visual explanation of one Top perception decision."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .runtime_monitor import pose_confidence


PASS_COLOR = (40, 220, 40)
REJECT_COLOR = (40, 40, 240)
AXIS_COLOR = (0, 220, 255)
CENTER_COLOR = (255, 180, 0)
TEXT_COLOR = (245, 245, 245)


def _text(
    image: np.ndarray,
    value: str,
    row: int,
    color: tuple[int, int, int],
) -> None:
    origin = (12, 28 + 24 * row)
    cv2.putText(
        image,
        value[:110],
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        value[:110],
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        color,
        1,
        cv2.LINE_AA,
    )


def _draw_pose(image: np.ndarray, pose: dict) -> None:
    corners = np.rint(
        np.asarray(pose["raw_corners_px"], dtype=np.float64)
    ).astype(np.int32)
    if corners.shape != (4, 2):
        raise ValueError("raw_corners_px must contain four image points")
    cv2.polylines(image, [corners], True, PASS_COLOR, 2, cv2.LINE_AA)

    center = np.rint(
        np.asarray(pose["raw_center_px"], dtype=np.float64)
    ).astype(np.int32)
    if center.shape != (2,):
        raise ValueError("raw_center_px must contain one image point")
    cv2.drawMarker(
        image,
        tuple(int(value) for value in center),
        CENTER_COLOR,
        cv2.MARKER_CROSS,
        18,
        2,
        cv2.LINE_AA,
    )

    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    index = int(np.argmax(lengths))
    start = corners[index]
    end = corners[(index + 1) % 4]
    cv2.arrowedLine(
        image,
        tuple(int(value) for value in start),
        tuple(int(value) for value in end),
        AXIS_COLOR,
        3,
        cv2.LINE_AA,
        tipLength=0.12,
    )


def render_debug_overlay(
    image_bgr: np.ndarray,
    *,
    pose: dict | None,
    code: str,
    reason: str,
    frame_age_s: float,
    detector_backend: str,
) -> np.ndarray:
    """Return a BGR frame annotated with PASS geometry or REJECT reason."""
    image = np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("image_bgr must be a uint8 HxWx3 image")
    annotated = image.copy()
    accepted = pose is not None
    color = PASS_COLOR if accepted else REJECT_COLOR
    cv2.rectangle(annotated, (0, 0), (annotated.shape[1] - 1, 82), (0, 0, 0), -1)
    cv2.rectangle(annotated, (0, 0), (7, 82), color, -1)
    _text(
        annotated,
        f"YOLO-OBB {'PASS' if accepted else 'REJECT'}  code={code}",
        0,
        color,
    )
    age_text = (
        f"{frame_age_s * 1000.0:.1f} ms"
        if math.isfinite(frame_age_s)
        else "unknown"
    )
    _text(
        annotated,
        f"backend={detector_backend}  frame_age={age_text}  motion=OFF",
        1,
        TEXT_COLOR,
    )
    if accepted:
        _draw_pose(annotated, pose)
        board_x, board_y = pose["board_position_m"]
        _text(
            annotated,
            "conf=%.3f  board=(%.4f, %.4f)m  yaw=%.1fdeg"
            % (
                pose_confidence(pose),
                float(board_x),
                float(board_y),
                math.degrees(float(pose["yaw_rad"])),
            ),
            2,
            TEXT_COLOR,
        )
    else:
        _text(annotated, f"reason={reason}", 2, color)
    return annotated
