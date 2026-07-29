"""Pure OpenCV detector and calibration validation for Top perception."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import cv2

import numpy as np

import yaml


class DetectionError(RuntimeError):
    """A fail-closed detection or input-validation failure."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class DetectorConfig:
    """Thresholds for the dark planar-object detector."""

    threshold: int = 110
    min_area_px: float = 1000.0
    min_width_px: int = 20
    min_height_px: int = 20
    min_solidity: float = 0.5
    image_edge_margin_px: int = 8
    exclusion_rectangles_px: tuple[tuple[int, int, int, int], ...] = ()

    def validate(self) -> None:
        if not 0 <= self.threshold <= 255:
            raise ValueError("threshold must be within 0..255")
        if self.min_area_px <= 0.0:
            raise ValueError("min_area_px must be positive")
        if self.min_width_px <= 0 or self.min_height_px <= 0:
            raise ValueError("minimum dimensions must be positive")
        if not 0.0 < self.min_solidity <= 1.0:
            raise ValueError("min_solidity must be within (0, 1]")
        if self.image_edge_margin_px < 0:
            raise ValueError("image_edge_margin_px must not be negative")
        for rectangle in self.exclusion_rectangles_px:
            if len(rectangle) != 4:
                raise ValueError("each exclusion rectangle must be x,y,w,h")
            x, y, width, height = rectangle
            if x < 0 or y < 0 or width <= 0 or height <= 0:
                raise ValueError(
                    "exclusion rectangles require non-negative origins "
                    "and positive dimensions"
                )


@dataclass(frozen=True)
class Calibration:
    """Validated camera and board-homography calibration."""

    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray
    projection: np.ndarray
    pixel_to_board: np.ndarray
    board_span: np.ndarray
    camera_info_sha256: str
    homography_status: str
    base_registration_status: str
    motion_authorized: bool


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"invalid YAML document: {path}")
    return document


def matrix(document: dict, key: str, rows: int, cols: int) -> np.ndarray:
    entry = document[key]
    if int(entry["rows"]) != rows or int(entry["cols"]) != cols:
        raise ValueError(f"{key} must be a {rows}x{cols} matrix")
    values = np.asarray(entry["data"], dtype=np.float64)
    if values.size != rows * cols or not np.all(np.isfinite(values)):
        raise ValueError(f"{key} contains invalid values")
    return values.reshape(rows, cols)


def load_calibration(
    camera_info_path: Path,
    homography_path: Path,
) -> Calibration:
    camera_info = load_yaml(camera_info_path)
    homography = load_yaml(homography_path)

    width = int(camera_info["image_width"])
    height = int(camera_info["image_height"])
    if width <= 0 or height <= 0:
        raise ValueError("camera-info resolution must be positive")

    homography_camera = homography.get("camera", {})
    if (
        int(homography_camera.get("image_width", -1)) != width
        or int(homography_camera.get("image_height", -1)) != height
    ):
        raise RuntimeError("homography and camera-info resolution mismatch")
    expected_hash = str(homography_camera.get("camera_info_sha256", ""))
    actual_hash = file_sha256(camera_info_path)
    if expected_hash != actual_hash:
        raise RuntimeError(
            "camera-info hash does not match the homography calibration"
        )
    if (
        homography_camera.get("input_domain")
        != "rectified_pixel_using_projection_matrix"
    ):
        raise RuntimeError("unsupported homography input domain")

    board_document = homography.get("board", {})
    board_span = np.asarray(
        board_document.get(
            "calibrated_span_m",
            board_document.get("inner_corner_span_m", []),
        ),
        dtype=np.float64,
    )
    if (
        board_span.shape != (2,)
        or not np.all(np.isfinite(board_span))
        or np.any(board_span <= 0.0)
    ):
        raise RuntimeError("homography has no valid calibrated board span")

    base_registration = homography.get("base_registration", {})
    motion_authorized = bool(homography.get("motion_authorized", False)) and bool(
        base_registration.get("motion_authorized", False)
    )
    return Calibration(
        image_width=width,
        image_height=height,
        camera_matrix=matrix(camera_info, "camera_matrix", 3, 3),
        distortion=matrix(
            camera_info,
            "distortion_coefficients",
            1,
            5,
        ).reshape(-1),
        projection=matrix(camera_info, "projection_matrix", 3, 4)[:, :3],
        pixel_to_board=matrix(
            homography["homography"],
            "rectified_pixel_to_board_m",
            3,
            3,
        ),
        board_span=board_span,
        camera_info_sha256=actual_hash,
        homography_status=str(homography.get("status", "UNKNOWN")),
        base_registration_status=str(
            base_registration.get("status", "UNKNOWN")
        ),
        motion_authorized=motion_authorized,
    )


def normalize_axis_yaw(angle: float) -> float:
    """Normalize an undirected rectangle axis to [-pi/2, pi/2)."""
    while angle >= math.pi / 2.0:
        angle -= math.pi
    while angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def find_candidates(
    image: np.ndarray,
    config: DetectorConfig,
) -> list[np.ndarray]:
    config.validate()
    if image.ndim != 3 or image.shape[2] != 3:
        raise DetectionError("INVALID_IMAGE", "image must be BGR8")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, config.threshold)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
    )
    image_height, image_width = mask.shape
    for x, y, width, height in config.exclusion_rectangles_px:
        x1 = min(image_width, x + width)
        y1 = min(image_height, y + height)
        if x < image_width and y < image_height:
            mask[y:y1, x:x1] = 0
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        _, _, width, height = cv2.boundingRect(contour)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area / hull_area if hull_area > 0.0 else 0.0
        if (
            area >= config.min_area_px
            and width >= config.min_width_px
            and height >= config.min_height_px
            and solidity >= config.min_solidity
        ):
            candidates.append(contour)
    return candidates


def transform_to_board(
    raw_pixels: np.ndarray,
    calibration: Calibration,
) -> np.ndarray:
    points = np.asarray(raw_pixels, dtype=np.float64).reshape(-1, 1, 2)
    rectified = cv2.undistortPoints(
        points,
        calibration.camera_matrix,
        calibration.distortion,
        P=calibration.projection,
    )
    return cv2.perspectiveTransform(
        rectified,
        calibration.pixel_to_board,
    ).reshape(-1, 2)


def contour_pose(
    contour: np.ndarray,
    calibration: Calibration,
) -> dict:
    rectangle = cv2.minAreaRect(contour)
    raw_center = np.asarray(rectangle[0], dtype=np.float64)
    raw_box = cv2.boxPoints(rectangle).astype(np.float64)
    board_center = transform_to_board(raw_center, calibration)[0]
    board_box = transform_to_board(raw_box, calibration)

    edges = np.roll(board_box, -1, axis=0) - board_box
    edge_lengths = np.linalg.norm(edges, axis=1)
    longest_axis = edges[int(np.argmax(edge_lengths))]
    yaw = normalize_axis_yaw(
        math.atan2(float(longest_axis[1]), float(longest_axis[0]))
    )

    area = float(cv2.contourArea(contour))
    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
    solidity = area / hull_area if hull_area > 0.0 else 0.0
    return {
        "raw_center_px": [float(raw_center[0]), float(raw_center[1])],
        "raw_corners_px": [
            [float(point[0]), float(point[1])]
            for point in raw_box
        ],
        "board_position_m": [
            float(board_center[0]),
            float(board_center[1]),
        ],
        "size_m": [
            float(np.max(edge_lengths)),
            float(np.min(edge_lengths)),
        ],
        "board_corners_m": [
            [float(point[0]), float(point[1])]
            for point in board_box
        ],
        "yaw_rad": float(yaw),
        "yaw_deg": float(math.degrees(yaw)),
        "yaw_semantics": "undirected_long_axis_modulo_pi",
        "area_px": area,
        "solidity": solidity,
    }


def detect_one_object(
    image: np.ndarray,
    calibration: Calibration,
    config: DetectorConfig,
    require_full_footprint: bool = True,
) -> dict:
    if (
        image.shape[1] != calibration.image_width
        or image.shape[0] != calibration.image_height
    ):
        raise DetectionError(
            "RESOLUTION_MISMATCH",
            "image and camera-info resolution mismatch: "
            f"image={image.shape[1]}x{image.shape[0]} "
            f"camera_info={calibration.image_width}x"
            f"{calibration.image_height}",
        )

    candidates = find_candidates(image, config)
    relevant_poses = []
    for contour in candidates:
        candidate_pose = contour_pose(contour, calibration)
        candidate_corners = np.asarray(
            candidate_pose["board_corners_m"],
            dtype=np.float64,
        )
        footprint_intersects = bool(
            np.all(np.max(candidate_corners, axis=0) >= 0.0)
            and np.all(
                np.min(candidate_corners, axis=0)
                <= calibration.board_span
            )
        )
        if footprint_intersects:
            relevant_poses.append(candidate_pose)

    if len(relevant_poses) != 1:
        raise DetectionError(
            "OBJECT_COUNT_INVALID",
            "expected exactly 1 object intersecting the calibrated region, "
            f"detected {len(relevant_poses)} "
            f"(ignored {len(candidates) - len(relevant_poses)} fully outside)",
        )

    pose = relevant_poses[0]
    raw_corners = np.asarray(pose["raw_corners_px"], dtype=np.float64)
    margin = float(config.image_edge_margin_px)
    image_fully_visible = bool(
        np.all(raw_corners[:, 0] >= margin)
        and np.all(raw_corners[:, 1] >= margin)
        and np.all(
            raw_corners[:, 0]
            <= calibration.image_width - 1 - margin
        )
        and np.all(
            raw_corners[:, 1]
            <= calibration.image_height - 1 - margin
        )
    )
    if not image_fully_visible:
        raise DetectionError(
            "IMAGE_FOOTPRINT_CLIPPED",
            "object footprint reaches the camera image safety margin",
        )
    board_center = np.asarray(
        pose["board_position_m"],
        dtype=np.float64,
    )
    center_inside = bool(
        np.all(board_center >= 0.0)
        and np.all(board_center <= calibration.board_span)
    )
    if not center_inside:
        raise DetectionError(
            "CENTER_OUTSIDE_CALIBRATED_REGION",
            "object center is outside calibrated board region "
            f"x=0..{calibration.board_span[0]:.6f}m "
            f"y=0..{calibration.board_span[1]:.6f}m",
        )

    board_corners = np.asarray(pose["board_corners_m"], dtype=np.float64)
    footprint_inside = bool(
        np.all(board_corners >= 0.0)
        and np.all(board_corners <= calibration.board_span)
    )
    if require_full_footprint and not footprint_inside:
        raise DetectionError(
            "OUTSIDE_CALIBRATED_REGION",
            "object footprint is outside calibrated board region "
            f"x=0..{calibration.board_span[0]:.6f}m "
            f"y=0..{calibration.board_span[1]:.6f}m",
        )
    pose["calibration_region"] = {
        "span_m": [
            float(calibration.board_span[0]),
            float(calibration.board_span[1]),
        ],
        "center_inside": True,
        "footprint_inside": footprint_inside,
        "image_fully_visible": image_fully_visible,
        "extrapolated": not footprint_inside,
        "ignored_fully_outside_count": (
            len(candidates) - len(relevant_poses)
        ),
    }
    return pose


def detect_image_file(
    image_path: Path,
    camera_info_path: Path,
    homography_path: Path,
    config: DetectorConfig,
) -> dict:
    """Run the shared detector on one file and return the CLI contract."""
    image_path = image_path.resolve()
    camera_info_path = camera_info_path.resolve()
    homography_path = homography_path.resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")

    calibration = load_calibration(camera_info_path, homography_path)
    pose = detect_one_object(image, calibration, config)
    return {
        "status": "TOP_OBJECT_POSE_PASS",
        "detected_count": 1,
        "frame_id": "top_board",
        "pose": pose,
        "motion_authorized": False,
        "robot_target_available": False,
        "source": {
            "image": str(image_path),
            "image_sha256": file_sha256(image_path),
            "camera_info": str(camera_info_path),
            "camera_info_sha256": calibration.camera_info_sha256,
            "homography": str(homography_path),
            "homography_status": calibration.homography_status,
            "base_registration_status": calibration.base_registration_status,
        },
        "detector": {
            "type": "dark_planar_contour",
            "threshold": config.threshold,
            "min_area_px": config.min_area_px,
            "min_width_px": config.min_width_px,
            "min_height_px": config.min_height_px,
            "min_solidity": config.min_solidity,
        },
    }


def frame_age_seconds(
    now_nanoseconds: int,
    stamp_seconds: int,
    stamp_nanoseconds: int,
    max_frame_age_s: float,
    future_tolerance_s: float,
) -> float:
    if max_frame_age_s <= 0.0 or future_tolerance_s < 0.0:
        raise ValueError("invalid frame-age limits")
    if stamp_seconds == 0 and stamp_nanoseconds == 0:
        raise DetectionError(
            "MISSING_TIMESTAMP",
            "image has no source timestamp",
        )
    stamp = stamp_seconds * 1_000_000_000 + stamp_nanoseconds
    age = (now_nanoseconds - stamp) / 1_000_000_000.0
    if age < -future_tolerance_s:
        raise DetectionError(
            "CLOCK_SKEW",
            f"image timestamp is {-age:.6f}s in the future",
        )
    age = max(0.0, age)
    if age > max_frame_age_s:
        raise DetectionError(
            "STALE_FRAME",
            f"image age {age:.6f}s exceeds {max_frame_age_s:.6f}s",
        )
    return age
