#!/usr/bin/env python3
"""Create a current table-plane calibration without an obsolete chessboard."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import cv2

import numpy as np

import yaml


GRID_MARKERS_X = 4
GRID_MARKERS_Y = 5
GRID_MARKER_LENGTH_M = 0.020
GRID_MARKER_GAP_M = 0.005
GRID_FIRST_ID = 10
DETECTION_SCALES = (2.0, 1.75, 2.25, 1.5, 3.5)


def load_yaml(path: Path) -> dict:
    """Load one mapping-valued YAML document."""
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f'invalid YAML document: {path}')
    return document


def matrix(document: dict, key: str, rows: int, cols: int) -> np.ndarray:
    """Load and validate a finite matrix entry."""
    entry = document[key]
    values = np.asarray(entry['data'], dtype=np.float64)
    if (
        int(entry['rows']) != rows
        or int(entry['cols']) != cols
        or values.size != rows * cols
        or not np.all(np.isfinite(values))
    ):
        raise ValueError(f'{key} must contain a finite {rows}x{cols} matrix')
    return values.reshape(rows, cols)


def file_sha256(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def planar_gridboard() -> tuple[object, object, tuple[int, ...]]:
    """Build the printed IDs 10-29 planar GridBoard contract."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.GridBoard_create(
        GRID_MARKERS_X,
        GRID_MARKERS_Y,
        GRID_MARKER_LENGTH_M,
        GRID_MARKER_GAP_M,
        dictionary,
        GRID_FIRST_ID,
    )
    ids = tuple(int(value) for value in board.ids.reshape(-1))
    return dictionary, board, ids


def base_plane_homography(
    projection: np.ndarray,
    base_from_camera: np.ndarray,
    table_z_m: float,
    board_origin_xy_m: np.ndarray,
) -> np.ndarray:
    """Map rectified pixels to a base-axis-aligned table frame."""
    projection = np.asarray(projection, dtype=np.float64)
    base_from_camera = np.asarray(base_from_camera, dtype=np.float64)
    origin = np.asarray(board_origin_xy_m, dtype=np.float64)
    if projection.shape != (3, 3):
        raise ValueError('projection must be 3x3')
    if base_from_camera.shape != (4, 4):
        raise ValueError('base_from_camera must be 4x4')
    if origin.shape != (2,):
        raise ValueError('board_origin_xy_m must contain x and y')
    if not (
        np.all(np.isfinite(projection))
        and np.all(np.isfinite(base_from_camera))
        and np.all(np.isfinite(origin))
        and math.isfinite(table_z_m)
    ):
        raise ValueError('calibration inputs must be finite')

    camera_rays = base_from_camera[:3, :3] @ np.linalg.inv(projection)
    camera_xyz = base_from_camera[:3, 3]
    denominator = camera_rays[2]
    height = float(table_z_m) - float(camera_xyz[2])
    base_homography = np.vstack(
        (
            camera_xyz[0] * denominator + height * camera_rays[0],
            camera_xyz[1] * denominator + height * camera_rays[1],
            denominator,
        )
    )
    board_from_base = np.asarray(
        [
            [1.0, 0.0, -origin[0]],
            [0.0, 1.0, -origin[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    result = board_from_base @ base_homography
    if abs(float(np.linalg.det(result))) < 1e-12:
        raise ValueError('table-plane homography is singular')
    return result / result[2, 2]


def transform_pixels(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    """Apply a planar homography to pixel points."""
    values = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(values, homography).reshape(-1, 2)


def detect_gridboard(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> dict:
    """Detect the complete GridBoard and estimate its camera pose."""
    if image is None or image.ndim != 3:
        raise ValueError('image must be a BGR image')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary, board, expected_ids = planar_gridboard()
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners = None
    ids = None
    selected_scale = None
    best_detected_ids = ()
    for scale in DETECTION_SCALES:
        detection_gray = cv2.resize(
            gray,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        )
        candidate_corners, candidate_ids, _ = cv2.aruco.detectMarkers(
            detection_gray,
            dictionary,
            parameters=parameters,
        )
        candidate_detected_ids = (
            ()
            if candidate_ids is None
            else tuple(
                sorted(int(value) for value in candidate_ids.reshape(-1))
            )
        )
        if len(candidate_detected_ids) > len(best_detected_ids):
            best_detected_ids = candidate_detected_ids
        if candidate_detected_ids == tuple(sorted(expected_ids)):
            corners = candidate_corners
            ids = candidate_ids
            selected_scale = scale
            break
    if ids is None or corners is None or selected_scale is None:
        raise RuntimeError(
            f'expected marker IDs {expected_ids}, '
            f'best detected {best_detected_ids}'
        )
    corners = [
        np.asarray(value, dtype=np.float32) / selected_scale
        for value in corners
    ]
    detected_ids = tuple(sorted(int(value) for value in ids.reshape(-1)))

    solved, rotation_vector, translation_vector = cv2.aruco.estimatePoseBoard(
        corners,
        ids,
        board,
        camera_matrix,
        distortion,
        None,
        None,
    )
    if int(solved) != len(expected_ids):
        raise RuntimeError(
            f'pose used {int(solved)} of {len(expected_ids)} markers'
        )
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_from_grid = np.eye(4, dtype=np.float64)
    camera_from_grid[:3, :3] = rotation
    camera_from_grid[:3, 3] = translation_vector.reshape(3)

    points_by_id = {
        int(marker_id): np.asarray(points, dtype=np.float64)
        for marker_id, points in zip(
            board.ids.reshape(-1),
            board.objPoints,
            strict=True,
        )
    }
    squared_errors = []
    all_pixels = []
    all_object_points = []
    for detected_corners, marker_id in zip(
        corners,
        ids.reshape(-1),
        strict=True,
    ):
        pixels = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
        object_points = points_by_id[int(marker_id)]
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        residual = projected.reshape(4, 2) - pixels
        squared_errors.extend(
            float(value) for value in np.sum(residual * residual, axis=1)
        )
        all_pixels.append(pixels)
        all_object_points.append(object_points)

    pixels = np.concatenate(all_pixels, axis=0)
    height, width = gray.shape
    return {
        'camera_from_grid': camera_from_grid,
        'pnp_rms_px': math.sqrt(float(np.mean(squared_errors))),
        'image_border_px': float(
            min(
                pixels[:, 0].min(),
                pixels[:, 1].min(),
                (width - 1) - pixels[:, 0].max(),
                (height - 1) - pixels[:, 1].max(),
            )
        ),
        'pixels_raw': pixels,
        'object_points': np.concatenate(all_object_points, axis=0),
        'detected_ids': detected_ids,
        'detection_scale': selected_scale,
    }


def evaluate_plane(
    base_from_grid: np.ndarray,
    object_points: np.ndarray,
    expected_table_z_m: float,
) -> dict:
    """Measure agreement with the expected physical table plane."""
    homogeneous = np.column_stack(
        (object_points, np.ones(len(object_points), dtype=np.float64))
    )
    base_points = (base_from_grid @ homogeneous.T).T[:, :3]
    normal = base_from_grid[:3, 2]
    normal /= np.linalg.norm(normal)
    tilt_deg = math.degrees(
        math.acos(float(np.clip(abs(normal[2]), -1.0, 1.0)))
    )
    errors = np.abs(base_points[:, 2] - float(expected_table_z_m))
    return {
        'base_points': base_points,
        'normal': normal,
        'tilt_deg': tilt_deg,
        'height_mean_m': float(np.mean(base_points[:, 2])),
        'height_error_max_mm': float(np.max(errors) * 1000.0),
    }


def classify(
    pnp_rms_px: float,
    image_border_px: float,
    plane_tilt_deg: float,
    height_error_max_mm: float,
) -> tuple[str, list[str]]:
    """Classify a candidate without authorizing motion."""
    failures = []
    if pnp_rms_px > 1.5:
        failures.append('PnP reprojection RMS exceeds 1.5 px')
    if image_border_px < 10.0:
        failures.append('GridBoard is too close to the image border')
    if plane_tilt_deg > 3.0:
        failures.append('GridBoard plane tilt exceeds 3 deg')
    if height_error_max_mm > 8.0:
        failures.append('GridBoard height differs from the table by over 8 mm')
    status = (
        'TABLE_BASE_CALIBRATION_CANDIDATE_MOTION_STILL_NOT_AUTHORIZED'
        if not failures
        else 'REJECTED_TABLE_BASE_CALIBRATION'
    )
    return status, failures


def matrix_document(value: np.ndarray) -> dict:
    """Serialize a matrix using the repository YAML contract."""
    return {
        'rows': int(value.shape[0]),
        'cols': int(value.shape[1]),
        'data': [
            [float(item) for item in row]
            for row in np.asarray(value, dtype=np.float64)
        ],
    }


def calibrate(
    image_path: Path,
    camera_info_path: Path,
    eye_to_hand_path: Path,
    table_z_m: float,
    board_origin_xy_m: np.ndarray,
    board_span_xy_m: np.ndarray,
) -> dict:
    """Create a fail-closed current table/base candidate."""
    camera_info = load_yaml(camera_info_path)
    eye_to_hand = load_yaml(eye_to_hand_path)
    if not str(eye_to_hand.get('status', '')).startswith(
        'EYE_TO_HAND_VALIDATED_'
    ):
        raise RuntimeError(
            'eye-to-hand candidate is not independently validated'
        )
    if bool(eye_to_hand.get('motion_authorized', False)):
        raise RuntimeError('eye-to-hand input unexpectedly authorizes motion')

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'failed to read image: {image_path}')
    camera_matrix = matrix(camera_info, 'camera_matrix', 3, 3)
    distortion = matrix(
        camera_info,
        'distortion_coefficients',
        1,
        5,
    ).reshape(-1)
    projection = matrix(camera_info, 'projection_matrix', 3, 4)[:, :3]
    base_from_camera = matrix(eye_to_hand, 'base_to_camera', 4, 4)

    detection = detect_gridboard(image, camera_matrix, distortion)
    base_from_grid = base_from_camera @ detection['camera_from_grid']
    plane = evaluate_plane(
        base_from_grid,
        detection['object_points'],
        table_z_m,
    )
    pixel_to_board = base_plane_homography(
        projection,
        base_from_camera,
        table_z_m,
        board_origin_xy_m,
    )
    rectified = cv2.undistortPoints(
        detection['pixels_raw'].reshape(-1, 1, 2),
        camera_matrix,
        distortion,
        P=projection,
    ).reshape(-1, 2)
    predicted_board = transform_pixels(rectified, pixel_to_board)
    expected_board = (
        plane['base_points'][:, :2] - board_origin_xy_m.reshape(1, 2)
    )
    metric_errors_mm = (
        np.linalg.norm(predicted_board - expected_board, axis=1) * 1000.0
    )
    status, failures = classify(
        detection['pnp_rms_px'],
        detection['image_border_px'],
        plane['tilt_deg'],
        plane['height_error_max_mm'],
    )
    base_from_board = np.eye(4, dtype=np.float64)
    base_from_board[:2, 3] = board_origin_xy_m
    base_from_board[2, 3] = table_z_m
    return {
        'schema_version': 1,
        'status': status,
        'motion_authorized': False,
        'robot_target_available': False,
        'transform_validated': False,
        'method': 'current_planar_gridboard_plus_validated_eye_to_hand',
        'frames': {
            'robot': 'left_base_link',
            'camera': 'top_camera_optical_frame',
            'board': 'top_board',
        },
        'planar_gridboard': {
            'dictionary': 'DICT_4X4_50',
            'markers_x': GRID_MARKERS_X,
            'markers_y': GRID_MARKERS_Y,
            'marker_length_m': GRID_MARKER_LENGTH_M,
            'marker_separation_m': GRID_MARKER_GAP_M,
            'first_marker_id': GRID_FIRST_ID,
            'detected_ids': list(detection['detected_ids']),
            'detection_scale': detection['detection_scale'],
        },
        'observed_gridboard': {
            'base_from_grid': matrix_document(base_from_grid),
            'origin_in_left_base_link_m': [
                float(value) for value in base_from_grid[:3, 3]
            ],
        },
        'table_plane': {
            'expected_z_in_left_base_link_m': float(table_z_m),
            'normal_in_left_base_link': [
                float(value) for value in plane['normal']
            ],
            'tilt_deg': plane['tilt_deg'],
            'height_mean_m': plane['height_mean_m'],
            'height_error_max_mm': plane['height_error_max_mm'],
        },
        'board': {
            'origin_in_left_base_link_xy_m': [
                float(value) for value in board_origin_xy_m
            ],
            'span_xy_m': [float(value) for value in board_span_xy_m],
            'axes': 'parallel_to_left_base_link_xy',
        },
        'base_from_board': matrix_document(base_from_board),
        'homography': {
            'rectified_pixel_to_board_m': matrix_document(pixel_to_board),
            'board_m_to_rectified_pixel': matrix_document(
                np.linalg.inv(pixel_to_board)
            ),
        },
        'fit_quality': {
            'pnp_rms_px': detection['pnp_rms_px'],
            'image_border_px': detection['image_border_px'],
            'grid_corner_metric_error_mean_mm': float(
                np.mean(metric_errors_mm)
            ),
            'grid_corner_metric_error_max_mm': float(
                np.max(metric_errors_mm)
            ),
        },
        'sources': {
            'image_sha256': file_sha256(image_path),
            'camera_info_sha256': file_sha256(camera_info_path),
            'eye_to_hand_sha256': file_sha256(eye_to_hand_path),
        },
        'failure_reasons': failures,
        'required_next_gate': (
            'repeat at a second table position, verify plane agreement, then '
            'promote the calibration without authorizing robot motion'
        ),
    }


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument(
        '--camera-info',
        type=Path,
        default=Path(
            'ros2_ws/src/manipulation_camera_manager/config/'
            'top_camera_info.yaml'
        ),
    )
    parser.add_argument(
        '--eye-to-hand',
        type=Path,
        default=Path(
            'artifacts/top_eye_to_hand/2026-07-30/'
            'independent_validation/candidate.yaml'
        ),
    )
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--table-z-m', type=float, default=-0.005)
    parser.add_argument('--board-origin-x-m', type=float, default=0.34)
    parser.add_argument('--board-origin-y-m', type=float, default=-0.28)
    parser.add_argument('--board-span-x-m', type=float, default=0.18)
    parser.add_argument('--board-span-y-m', type=float, default=0.28)
    return parser.parse_args()


def main() -> int:
    """Run calibration and write a YAML candidate."""
    args = parse_args()
    result = calibrate(
        args.image.resolve(),
        args.camera_info.resolve(),
        args.eye_to_hand.resolve(),
        float(args.table_z_m),
        np.asarray(
            [args.board_origin_x_m, args.board_origin_y_m],
            dtype=np.float64,
        ),
        np.asarray(
            [args.board_span_x_m, args.board_span_y_m],
            dtype=np.float64,
        ),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(result, stream, sort_keys=False)
    print(
        'TOP_BASE_TABLE_CALIBRATION_%s pnp_rms_px=%.6f '
        'tilt_deg=%.6f height_error_max_mm=%.6f output=%s'
        % (
            'PASS' if not result['failure_reasons'] else 'REJECTED',
            result['fit_quality']['pnp_rms_px'],
            result['table_plane']['tilt_deg'],
            result['table_plane']['height_error_max_mm'],
            args.output.resolve(),
        )
    )
    return 0 if not result['failure_reasons'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
