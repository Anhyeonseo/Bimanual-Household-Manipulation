#!/usr/bin/env python3
"""Create a board-relative Top-camera homography from one chessboard image."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np
import yaml


PATTERN_SIZE = (7, 7)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def matrix_values(matrix: np.ndarray) -> list[float]:
    return [float(value) for value in matrix.reshape(-1)]


def point_values(points: np.ndarray) -> list[list[float]]:
    return [
        [float(point[0]), float(point[1])]
        for point in points.reshape(-1, 2)
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect a 7x7 chessboard, undistort its corners, and save a "
            "rectified-pixel to board-plane homography."
        )
    )
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--camera-info", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--square-size-m", required=True, type=float)
    parser.add_argument("--provisional-pan-x-m", required=True, type=float)
    parser.add_argument("--provisional-pan-y-m", required=True, type=float)
    parser.add_argument(
        "--left-base-to-pan-x-m",
        type=float,
        default=0.0388353,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image_path = args.image.resolve()
    camera_info_path = args.camera_info.resolve()
    output_path = args.output.resolve()

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")
    with camera_info_path.open(encoding="utf-8") as stream:
        camera_info = yaml.safe_load(stream)

    width = int(camera_info["image_width"])
    height = int(camera_info["image_height"])
    if image.shape[1] != width or image.shape[0] != height:
        raise RuntimeError(
            "image and camera-info resolution mismatch: "
            f"image={image.shape[1]}x{image.shape[0]} "
            f"camera_info={width}x{height}"
        )

    camera_matrix = np.asarray(
        camera_info["camera_matrix"]["data"],
        dtype=np.float64,
    ).reshape(3, 3)
    distortion = np.asarray(
        camera_info["distortion_coefficients"]["data"],
        dtype=np.float64,
    )
    projection = np.asarray(
        camera_info["projection_matrix"]["data"],
        dtype=np.float64,
    ).reshape(3, 4)[:, :3]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        PATTERN_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        raise RuntimeError("complete 7x7 chessboard was not detected")
    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        ),
    )
    rectified_pixels = cv2.undistortPoints(
        corners,
        camera_matrix,
        distortion,
        P=projection,
    ).reshape(-1, 2)

    square = float(args.square_size_m)
    board_points = np.asarray(
        [
            [row * square, column * square]
            for row in range(PATTERN_SIZE[1])
            for column in range(PATTERN_SIZE[0])
        ],
        dtype=np.float64,
    )
    pixel_to_board, _ = cv2.findHomography(
        rectified_pixels,
        board_points,
        method=0,
    )
    if pixel_to_board is None:
        raise RuntimeError("homography solve failed")
    board_to_pixel = np.linalg.inv(pixel_to_board)

    predicted_board = cv2.perspectiveTransform(
        rectified_pixels.reshape(-1, 1, 2),
        pixel_to_board,
    ).reshape(-1, 2)
    metric_errors_mm = (
        np.linalg.norm(predicted_board - board_points, axis=1) * 1000.0
    )
    predicted_pixels = cv2.perspectiveTransform(
        board_points.reshape(-1, 1, 2),
        board_to_pixel,
    ).reshape(-1, 2)
    pixel_errors = np.linalg.norm(
        predicted_pixels - rectified_pixels,
        axis=1,
    )
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())

    board_origin_x = (
        float(args.left_base_to_pan_x_m)
        + float(args.provisional_pan_x_m)
    )
    board_origin_y = float(args.provisional_pan_y_m)
    span = (PATTERN_SIZE[0] - 1) * square

    output = {
        "schema_version": 1,
        "status": "BOARD_RELATIVE_VALID_BASE_REGISTRATION_REQUIRED",
        "motion_authorized": False,
        "camera": {
            "name": camera_info["camera_name"],
            "image_width": width,
            "image_height": height,
            "input_domain": "rectified_pixel_using_projection_matrix",
            "camera_info_file": camera_info_path.name,
            "camera_info_sha256": file_sha256(camera_info_path),
        },
        "capture": {
            "image_file": image_path.name,
            "image_sha256": file_sha256(image_path),
            "sharpness_laplacian_variance": sharpness,
            "raw_corner_pixels": point_values(corners),
            "rectified_corner_pixels": point_values(rectified_pixels),
        },
        "board": {
            "inner_corners": [PATTERN_SIZE[0], PATTERN_SIZE[1]],
            "square_size_m": square,
            "origin": "O=corner[0,0]",
            "positive_x": "O_to_B=row_increasing",
            "positive_y": "O_to_A=column_increasing",
            "inner_corner_span_m": [span, span],
        },
        "homography": {
            "rectified_pixel_to_board_m": {
                "rows": 3,
                "cols": 3,
                "data": matrix_values(pixel_to_board),
            },
            "board_m_to_rectified_pixel": {
                "rows": 3,
                "cols": 3,
                "data": matrix_values(board_to_pixel),
            },
        },
        "fit_quality": {
            "corner_count": len(rectified_pixels),
            "mean_error_mm": float(np.mean(metric_errors_mm)),
            "p95_error_mm": float(np.percentile(metric_errors_mm, 95)),
            "max_error_mm": float(np.max(metric_errors_mm)),
            "mean_error_px": float(np.mean(pixel_errors)),
            "p95_error_px": float(np.percentile(pixel_errors, 95)),
            "max_error_px": float(np.max(pixel_errors)),
        },
        "base_registration": {
            "status": "PROVISIONAL_RULER_MEASUREMENT",
            "motion_authorized": False,
            "reference": "left_base_joint_vertical_axis",
            "left_base_link_to_pan_axis_xy_m": [
                float(args.left_base_to_pan_x_m),
                0.0,
            ],
            "pan_axis_to_board_origin_xy_m": [
                float(args.provisional_pan_x_m),
                float(args.provisional_pan_y_m),
            ],
            "provisional_board_origin_in_left_base_link_xy_m": [
                board_origin_x,
                board_origin_y,
            ],
            "required_next_gate": (
                "robot-assisted registration with at least 3 table points"
            ),
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False)

    print(
        "TOP_HOMOGRAPHY_PASS "
        f"corners={len(rectified_pixels)} "
        f"mean_mm={np.mean(metric_errors_mm):.4f} "
        f"p95_mm={np.percentile(metric_errors_mm, 95):.4f} "
        f"max_mm={np.max(metric_errors_mm):.4f} "
        f"motion_authorized=0 output={output_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
