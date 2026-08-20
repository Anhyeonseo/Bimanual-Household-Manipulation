#!/usr/bin/env python3
"""Solve a fail-closed Top-board to robot-base registration candidate.

The yellow rigid-body marker is above the calibrated worktable plane.
Therefore its raw pixel must be intersected with a board-parallel plane at the
marker frame's URDF FK height; applying the table homography directly would
introduce parallax error.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml


JOINT_NAMES = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"invalid YAML document: {path}")
    return document


def yaml_matrix(document: dict, key: str, rows: int, cols: int) -> np.ndarray:
    entry = document[key]
    values = np.asarray(entry["data"], dtype=np.float64)
    if (
        int(entry["rows"]) != rows
        or int(entry["cols"]) != cols
        or values.size != rows * cols
        or not np.all(np.isfinite(values))
    ):
        raise ValueError(f"{key} must contain a finite {rows}x{cols} matrix")
    return values.reshape(rows, cols)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm <= 0.0:
        raise ValueError("joint axis must be non-zero")
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    one_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def parse_vector(value: str | None, default: str) -> np.ndarray:
    result = np.fromstring(value if value is not None else default, sep=" ")
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"invalid URDF vector: {value}")
    return result


def urdf_fk(
    urdf_xml: str,
    base_link: str,
    target_link: str,
    joint_positions: dict[str, float],
) -> np.ndarray:
    root = ET.fromstring(urdf_xml)
    joints_by_child: dict[str, ET.Element] = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None:
            joints_by_child[str(child.attrib["link"])] = joint

    chain: list[ET.Element] = []
    link = target_link
    while link != base_link:
        if link not in joints_by_child:
            raise ValueError(f"no URDF chain from {base_link} to {target_link}")
        joint = joints_by_child[link]
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None:
            raise ValueError("joint has no parent")
        link = str(parent.attrib["link"])
    chain.reverse()

    result = np.eye(4, dtype=np.float64)
    for joint in chain:
        origin = joint.find("origin")
        xyz = parse_vector(
            None if origin is None else origin.attrib.get("xyz"),
            "0 0 0",
        )
        rpy = parse_vector(
            None if origin is None else origin.attrib.get("rpy"),
            "0 0 0",
        )
        result = result @ transform(rpy_matrix(rpy), xyz)
        joint_type = str(joint.attrib.get("type", "fixed"))
        if joint_type in ("revolute", "continuous"):
            axis_element = joint.find("axis")
            axis = parse_vector(
                None if axis_element is None else axis_element.attrib.get("xyz"),
                "1 0 0",
            )
            angle = float(joint_positions.get(str(joint.attrib["name"]), 0.0))
            result = result @ transform(
                axis_angle_matrix(axis, angle),
                np.zeros(3),
            )
        elif joint_type != "fixed":
            raise ValueError(f"unsupported joint type in FK chain: {joint_type}")
    return result


def board_object_points(board: dict) -> np.ndarray:
    columns, rows = (int(value) for value in board["inner_corners"])
    square = float(board["square_size_m"])
    # Capture ordering is seven entries along board +Y for each +X row.
    return np.asarray(
        [
            [row * square, column * square, 0.0]
            for row in range(rows)
            for column in range(columns)
        ],
        dtype=np.float64,
    )


def solve_board_camera_pose(
    homography: dict,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    object_points = board_object_points(homography["board"])
    image_points = np.asarray(
        homography["capture"]["raw_corner_pixels"],
        dtype=np.float64,
    )
    if image_points.shape != (object_points.shape[0], 2):
        raise ValueError("board corner count does not match image corner count")
    solved, rotation_vector, translation = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        raise RuntimeError("board camera pose solve failed")
    rotation, _ = cv2.Rodrigues(rotation_vector)
    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation,
        camera_matrix,
        distortion,
    )
    residual = projected.reshape(-1, 2) - image_points
    rms_px = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    camera_center_board = -rotation.T @ translation.reshape(3)
    return rotation, camera_center_board, rms_px


def raw_pixel_to_board_plane(
    raw_pixel: np.ndarray,
    plane_height_m: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    board_to_camera_rotation: np.ndarray,
    camera_center_board: np.ndarray,
) -> np.ndarray:
    normalized = cv2.undistortPoints(
        np.asarray(raw_pixel, dtype=np.float64).reshape(1, 1, 2),
        camera_matrix,
        distortion,
    ).reshape(2)
    ray_camera = np.asarray([normalized[0], normalized[1], 1.0])
    ray_board = board_to_camera_rotation.T @ ray_camera
    if abs(float(ray_board[2])) < 1e-9:
        raise RuntimeError("camera ray is parallel to the requested plane")
    distance = (float(plane_height_m) - camera_center_board[2]) / ray_board[2]
    if distance <= 0.0:
        raise RuntimeError("requested board plane is behind the camera")
    point = camera_center_board + distance * ray_board
    return point


def fit_rigid_2d(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError("source and target must be matching Nx2 arrays")
    if source.shape[0] < 3:
        raise ValueError("at least three correspondences are required")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    u, singular_values, vt = np.linalg.svd(source_centered.T @ target_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    residuals = (rotation @ source.T).T + translation - target
    return rotation, translation, residuals, singular_values


def pairwise_distances(points: np.ndarray) -> list[float]:
    return [
        float(np.linalg.norm(points[index] - points[other]))
        for index in range(len(points))
        for other in range(index + 1, len(points))
    ]


def triangle_area(points: np.ndarray) -> float:
    if len(points) != 3:
        return 0.0
    first, second, third = points
    first_edge = second - first
    second_edge = third - first
    cross = first_edge[0] * second_edge[1] - first_edge[1] * second_edge[0]
    return abs(float(cross)) / 2.0


def classify_fit(
    rms_residual_mm: float,
    max_residual_mm: float,
    geometry_condition_ratio: float,
    span_ratio: float,
) -> str:
    if (
        rms_residual_mm > 3.0
        or max_residual_mm > 5.0
        or geometry_condition_ratio < 0.01
        or not 0.8 <= span_ratio <= 1.2
    ):
        return "REJECTED_REGISTRATION_GEOMETRY_OR_RIGID_FIT"
    return "PROVISIONAL_VISUAL_MARKER_REQUIRES_INDEPENDENT_VALIDATION"


def registration_contract(
    session: dict,
) -> tuple[dict, str, list[dict], str, str]:
    frames = session["frames"]
    if "marker" in frames:
        return (
            frames,
            str(frames["marker"]),
            session.get("visual_marker_points", []),
            "base_marker_xyz_m",
            "height_corrected_visual_marker_se2",
        )
    return (
        frames,
        str(frames["tcp"]),
        session.get("visual_tcp_points", []),
        "base_tcp_xyz_m",
        "height_corrected_visual_tcp_se2",
    )


def solve(
    session: dict,
    camera_info: dict,
    homography: dict,
    urdf_xml: str,
) -> dict:
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    (
        frames,
        target_frame,
        points,
        stored_xyz_key,
        method,
    ) = registration_contract(session)
    if len(points) < 3 or any(p.get("capture_status") != "PASS" for p in points):
        raise RuntimeError("at least three PASS visual marker points are required")

    camera_matrix = yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = yaml_matrix(
        camera_info,
        "distortion_coefficients",
        1,
        5,
    ).reshape(-1)
    rotation_board_camera, camera_center, pose_rms_px = solve_board_camera_pose(
        homography,
        camera_matrix,
        distortion,
    )

    base_xyz = []
    board_xyz = []
    stored_fk_errors = []
    point_results = []
    for point in points:
        measured = np.asarray(point["measured_arm_rad"], dtype=np.float64)
        if measured.shape != (5,) or not np.all(np.isfinite(measured)):
            raise ValueError(f"{point['id']} has invalid measured_arm_rad")
        joint_positions = dict(zip(JOINT_NAMES, measured, strict=True))
        fk_xyz = urdf_fk(
            urdf_xml,
            str(frames["robot"]),
            target_frame,
            joint_positions,
        )[:3, 3]
        stored_xyz = point.get(stored_xyz_key)
        if stored_xyz is None:
            stored_error = 0.0
        else:
            stored_xyz_array = np.asarray(stored_xyz, dtype=np.float64)
            if stored_xyz_array.shape != (3,) or not np.all(
                np.isfinite(stored_xyz_array)
            ):
                raise ValueError(f"{point['id']} has invalid {stored_xyz_key}")
            stored_error = float(np.linalg.norm(fk_xyz - stored_xyz_array))
            if stored_error > 0.003:
                raise RuntimeError(
                    f"{point['id']} URDF FK disagrees with captured TF by "
                    f"{stored_error * 1000.0:.2f} mm"
                )
        board_point = raw_pixel_to_board_plane(
            np.asarray(point["marker_center_px"], dtype=np.float64),
            float(fk_xyz[2]),
            camera_matrix,
            distortion,
            rotation_board_camera,
            camera_center,
        )
        base_xyz.append(fk_xyz)
        board_xyz.append(board_point)
        stored_fk_errors.append(stored_error)
        point_results.append(
            {
                "id": str(point["id"]),
                stored_xyz_key: [float(value) for value in fk_xyz],
                "board_marker_xyz_m": [float(value) for value in board_point],
                "stored_tf_fk_error_mm": stored_error * 1000.0,
            }
        )

    base_xyz_array = np.asarray(base_xyz)
    board_xyz_array = np.asarray(board_xyz)
    base_to_board_rotation, base_to_board_translation, residuals, singular = (
        fit_rigid_2d(base_xyz_array[:, :2], board_xyz_array[:, :2])
    )
    residual_norms = np.linalg.norm(residuals, axis=1)
    inverse_rotation = base_to_board_rotation.T
    inverse_translation = -inverse_rotation @ base_to_board_translation
    base_distances = pairwise_distances(base_xyz_array[:, :2])
    board_distances = pairwise_distances(board_xyz_array[:, :2])
    condition_ratio = (
        float(singular[-1] / singular[0]) if singular[0] > 0.0 else 0.0
    )
    rms_residual_mm = float(
        np.sqrt(np.mean(residual_norms * residual_norms)) * 1000.0
    )
    max_residual_mm = float(np.max(residual_norms) * 1000.0)
    span_ratio = (
        max(board_distances) / max(base_distances)
        if max(base_distances) > 0.0
        else 0.0
    )

    # Even a passing fit remains provisional until a separately captured pose
    # validates it. A poor rigid fit or short/collinear geometry is rejected.
    status = classify_fit(
        rms_residual_mm,
        max_residual_mm,
        condition_ratio,
        span_ratio,
    )
    return {
        "schema_version": 1,
        "status": status,
        "motion_authorized": False,
        "robot_target_available": False,
        "frames": dict(frames),
        "method": method,
        "camera_pose": {
            "camera_center_in_board_m": [
                float(value) for value in camera_center
            ],
            "board_pnp_rms_px": pose_rms_px,
        },
        "base_to_board": {
            "rotation_2x2": [
                [float(value) for value in row]
                for row in base_to_board_rotation
            ],
            "translation_xy_m": [
                float(value) for value in base_to_board_translation
            ],
            "yaw_rad": float(
                math.atan2(
                    base_to_board_rotation[1, 0],
                    base_to_board_rotation[0, 0],
                )
            ),
        },
        "board_to_base": {
            "rotation_2x2": [
                [float(value) for value in row] for row in inverse_rotation
            ],
            "translation_xy_m": [
                float(value) for value in inverse_translation
            ],
            "yaw_rad": float(
                math.atan2(inverse_rotation[1, 0], inverse_rotation[0, 0])
            ),
        },
        "fit_quality": {
            "point_count": len(points),
            "rms_residual_mm": rms_residual_mm,
            "max_residual_mm": max_residual_mm,
            "urdf_vs_captured_tf_max_mm": max(stored_fk_errors) * 1000.0,
            "base_min_pair_distance_mm": min(base_distances) * 1000.0,
            "base_max_pair_distance_mm": max(base_distances) * 1000.0,
            "board_min_pair_distance_mm": min(board_distances) * 1000.0,
            "board_max_pair_distance_mm": max(board_distances) * 1000.0,
            "base_triangle_area_mm2": triangle_area(
                base_xyz_array[:, :2]
            ) * 1_000_000.0,
            "board_triangle_area_mm2": triangle_area(
                board_xyz_array[:, :2]
            ) * 1_000_000.0,
            "geometry_condition_ratio": condition_ratio,
            "board_to_base_max_span_ratio": span_ratio,
            "acceptance_thresholds": {
                "rms_residual_mm_max": 3.0,
                "max_residual_mm_max": 5.0,
                "geometry_condition_ratio_min": 0.01,
                "board_to_base_span_ratio_min": 0.8,
                "board_to_base_span_ratio_max": 1.2,
            },
        },
        "points": point_results,
        "required_next_gate": (
            "repeat registration with at least five geometrically separated "
            "poses using the fixed URDF marker frame, then capture an "
            "independent validation pose; do not authorize motion"
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--session",
        type=Path,
        default=Path("output/top_base_registration_session.yaml"),
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=Path(
            "ros2_ws/src/manipulation_camera_manager/config/"
            "top_camera_info.yaml"
        ),
    )
    parser.add_argument(
        "--homography",
        type=Path,
        default=Path(
            "ros2_ws/src/manipulation_camera_manager/config/"
            "top_worktable_homography.yaml"
        ),
    )
    parser.add_argument(
        "--urdf-xacro",
        type=Path,
        default=Path(
            "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/top_base_registration_candidate.yaml"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro)],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    result = solve(
        load_yaml(args.session),
        load_yaml(args.camera_info),
        load_yaml(args.homography),
        urdf_xml,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, sort_keys=False)
    print(
        f"TOP_BASE_REGISTRATION_{result['status']} "
        f"rms_mm={result['fit_quality']['rms_residual_mm']:.3f} "
        f"max_mm={result['fit_quality']['max_residual_mm']:.3f} "
        f"condition={result['fit_quality']['geometry_condition_ratio']:.6f} "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
