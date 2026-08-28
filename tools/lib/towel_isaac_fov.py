"""Calibrated Top-camera containment contract for the R2 Isaac S0 stage."""

from __future__ import annotations

from hashlib import sha256
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


class TowelIsaacFovError(ValueError):
    """The calibrated camera contract cannot support an S0 FOV claim."""


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def load_yaml_mapping(path: Path, name: str) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise TowelIsaacFovError(f"{name} root must be a mapping")
    return document


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TowelIsaacFovError(f"{name} must be a mapping")
    return value


def _matrix(value: object, shape: tuple[int, int], name: str) -> np.ndarray:
    document = _mapping(value, name)
    data = np.asarray(document.get("data"), dtype=np.float64)
    if data.size != shape[0] * shape[1] or not np.all(np.isfinite(data)):
        raise TowelIsaacFovError(f"{name} must contain a finite {shape} matrix")
    return data.reshape(shape)


def _project(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points))))
    projected = (homography @ homogeneous.T).T
    if np.any(np.abs(projected[:, 2]) < 1.0e-12):
        raise TowelIsaacFovError("FOV projection reached a point at infinity")
    return projected[:, :2] / projected[:, 2:3]


def _camera_pose_from_plane(
    intrinsic: np.ndarray,
    board_to_pixel: np.ndarray,
    board_origin_xy: np.ndarray,
    table_z_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    normalized = np.linalg.inv(intrinsic) @ board_to_pixel
    scale = 2.0 / (
        np.linalg.norm(normalized[:, 0]) + np.linalg.norm(normalized[:, 1])
    )
    first = scale * normalized[:, 0]
    second = scale * normalized[:, 1]
    approximate = np.column_stack((first, second, np.cross(first, second)))
    left, _, right = np.linalg.svd(approximate)
    board_to_camera = left @ right
    if np.linalg.det(board_to_camera) < 0.0:
        left[:, -1] *= -1.0
        board_to_camera = left @ right
    translation = scale * normalized[:, 2]
    camera_in_board = -board_to_camera.T @ translation
    if camera_in_board[2] <= 0.0:
        raise TowelIsaacFovError("calibrated camera is not above the table plane")
    position = np.array(
        [
            board_origin_xy[0] + camera_in_board[0],
            board_origin_xy[1] + camera_in_board[1],
            table_z_m + camera_in_board[2],
        ],
        dtype=np.float64,
    )
    camera_to_workcell = board_to_camera.T
    return position, camera_to_workcell


def validate_calibrated_top_fov(
    source: Mapping[str, Any],
    *,
    camera_info_path: Path,
    homography_path: Path,
    task_contract_path: Path,
) -> dict[str, object]:
    """Validate the canonical proxy and its 30 mm margin in calibrated pixels."""
    camera_sha = file_sha256(camera_info_path)
    homography_sha = file_sha256(homography_path)
    contract_sha = file_sha256(task_contract_path)
    camera = load_yaml_mapping(camera_info_path, "camera info")
    homography = load_yaml_mapping(homography_path, "worktable homography")
    contract = load_yaml_mapping(task_contract_path, "task contract")

    if contract.get("motion_authorized") is not False:
        raise TowelIsaacFovError("task contract violates the motion lock")
    observation = _mapping(
        contract.get("workcell_observation_candidate"),
        "workcell observation",
    )
    top_camera = _mapping(observation.get("top_camera"), "top camera")
    envelope_contract = _mapping(
        observation.get("towel_envelope"),
        "towel envelope",
    )
    if (
        top_camera.get("metric_calibration_validated") is not True
        or top_camera.get("optical_containment_result")
        != "PASS_WITH_PLACEMENT_REGION"
        or envelope_contract.get("actual_towel_validation_performed") is not True
    ):
        raise TowelIsaacFovError("Top-camera containment contract is not validated")
    if top_camera.get("worktable_config_sha256") != homography_sha:
        raise TowelIsaacFovError("task contract does not pin the worktable homography")

    camera_doc = _mapping(homography.get("camera"), "homography camera")
    if camera_doc.get("camera_info_sha256") != camera_sha:
        raise TowelIsaacFovError("homography does not pin the camera info")
    if (
        homography.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or homography.get("motion_authorized") is not False
    ):
        raise TowelIsaacFovError("worktable homography is not validated and locked")
    registration = _mapping(homography.get("base_registration"), "base registration")
    if registration.get("transform_validated") is not True:
        raise TowelIsaacFovError("worktable base registration is not validated")

    width = int(camera.get("image_width", 0))
    height = int(camera.get("image_height", 0))
    if (
        width != int(top_camera.get("width", -1))
        or height != int(top_camera.get("height", -1))
        or width != int(camera_doc.get("image_width", -1))
        or height != int(camera_doc.get("image_height", -1))
    ):
        raise TowelIsaacFovError("camera dimensions do not match the task contract")

    projection = _matrix(camera.get("projection_matrix"), (3, 4), "projection")
    intrinsic = projection[:, :3]
    homography_doc = _mapping(homography.get("homography"), "homography")
    board_to_pixel = _matrix(
        homography_doc.get("board_m_to_rectified_pixel"),
        (3, 3),
        "board-to-pixel homography",
    )
    board = _mapping(homography.get("board"), "board")
    board_origin = np.asarray(
        board.get("origin_in_left_base_link_xy_m"), dtype=np.float64
    )
    board_span = np.asarray(board.get("calibrated_span_m"), dtype=np.float64)
    table_z_m = float(board.get("table_z_in_left_base_link_m", math.nan))
    if (
        board_origin.shape != (2,)
        or board_span.shape != (2,)
        or not np.all(np.isfinite(board_origin))
        or not np.all(np.isfinite(board_span))
        or not math.isfinite(table_z_m)
    ):
        raise TowelIsaacFovError("calibrated board geometry is invalid")

    poses = np.asarray(source.get("rigid_proxy_pose_xyz_yaw_rad"), dtype=np.float64)
    proxy_size = np.asarray(source.get("proxy_size_xyz_m"), dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != 4 or proxy_size.shape != (3,):
        raise TowelIsaacFovError("S0 proxy source has invalid dimensions")
    if not np.allclose(poses, poses[0], atol=1.0e-12, rtol=0.0):
        raise TowelIsaacFovError("S0 FOV gate requires identical local proxy poses")
    if not math.isclose(float(proxy_size[0]), float(proxy_size[1]), abs_tol=1.0e-9):
        raise TowelIsaacFovError("S0 FOV gate requires a square proxy")
    expected_table_z = float(poses[0, 2] - 0.5 * proxy_size[2])
    if not math.isclose(expected_table_z, table_z_m, abs_tol=1.0e-9):
        raise TowelIsaacFovError("S0 proxy and calibrated table heights differ")

    margin_m = float(envelope_contract.get("required_perimeter_margin_mm")) / 1000.0
    half_extent = 0.5 * float(proxy_size[0]) + margin_m
    center = poses[0, :2]
    envelope = np.asarray(
        [
            center + (-half_extent, -half_extent),
            center + (half_extent, -half_extent),
            center + (half_extent, half_extent),
            center + (-half_extent, half_extent),
        ],
        dtype=np.float64,
    )
    envelope_board = envelope - board_origin
    board_margins = np.column_stack(
        (
            envelope_board[:, 0],
            board_span[0] - envelope_board[:, 0],
            envelope_board[:, 1],
            board_span[1] - envelope_board[:, 1],
        )
    )
    minimum_board_margin = float(np.min(board_margins))
    pixels = _project(board_to_pixel, envelope_board)
    pixel_margins = np.column_stack(
        (
            pixels[:, 0],
            (width - 1.0) - pixels[:, 0],
            pixels[:, 1],
            (height - 1.0) - pixels[:, 1],
        )
    )
    minimum_pixel_margin = float(np.min(pixel_margins))
    if minimum_board_margin < 0.0:
        raise TowelIsaacFovError("towel envelope leaves the calibrated board span")
    if minimum_pixel_margin < 0.0:
        raise TowelIsaacFovError("towel envelope leaves the calibrated image")

    image_pixels = np.asarray(
        [
            [0.0, 0.0],
            [width - 1.0, 0.0],
            [width - 1.0, height - 1.0],
            [0.0, height - 1.0],
        ]
    )
    image_footprint = _project(np.linalg.inv(board_to_pixel), image_pixels)
    image_footprint += board_origin
    camera_position, camera_rotation = _camera_pose_from_plane(
        intrinsic,
        board_to_pixel,
        board_origin,
        table_z_m,
    )
    measured_height = float(camera_position[2] - table_z_m)
    approximate_height = (
        float(top_camera.get("camera_to_table_vertical_distance_mm")) / 1000.0
    )
    if abs(measured_height - approximate_height) > 0.020:
        raise TowelIsaacFovError(
            "derived camera height disagrees with the measured height"
        )

    return {
        "image_width": width,
        "image_height": height,
        "required_perimeter_margin_m": margin_m,
        "envelope_workcell_xy_m": envelope.tolist(),
        "envelope_rectified_pixels": pixels.tolist(),
        "minimum_image_margin_px": minimum_pixel_margin,
        "minimum_calibrated_board_margin_m": minimum_board_margin,
        "image_footprint_workcell_xy_m": image_footprint.tolist(),
        "camera_position_workcell_m": camera_position.tolist(),
        "camera_to_workcell_rotation": camera_rotation.tolist(),
        "camera_vertical_distance_m": measured_height,
        "identity": {
            "camera_info_sha256": camera_sha,
            "worktable_homography_sha256": homography_sha,
            "task_contract_sha256": contract_sha,
        },
    }
