#!/usr/bin/env python3
"""Capture one stationary Top eye-to-hand calibration sample.

The default mode is read-only and torque-off.  The optional resident-hold mode
also reads the resident status service so a terminal measured anchor is used
only while the adapter reports an active torque hold.  Neither mode creates a
motion publisher, motion service client, Action client, or serial connection.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger


ARM_JOINT_NAMES_BY_SIDE = {
    side: tuple(
        f"{side}_{joint}_joint"
        for joint in (
            "base",
            "shoulder",
            "elbow",
            "wrist_flex",
            "wrist_roll",
        )
    )
    for side in ("left", "right")
}
# Backward-compatible alias used by existing left-arm capture imports.
ARM_JOINT_NAMES = ARM_JOINT_NAMES_BY_SIDE["left"]
EXPECTED_MARKER_IDS = (0, 1, 2, 3)
GRIDBOARD_MARKERS_X = 2
GRIDBOARD_MARKERS_Y = 2
GRIDBOARD_MARKER_LENGTH_M = 0.020
GRIDBOARD_MARKER_SEPARATION_M = 0.005
GRIDBOARD_DICTIONARY = cv2.aruco.DICT_4X4_50
RESIDENT_STATUS_SERVICE = "/bimanual_stream_adapter/status"
RESIDENT_ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"


def resident_torque_hold_is_valid(document: dict) -> bool:
    """Return whether status proves a stationary armed resident hold."""
    return bool(
        document.get("motion_authorized") is True
        and document.get("state") == "ready"
        and document.get("torque_hold_active") is True
        and document.get("owner")
        and int(document.get("arbiter_epoch", 0)) > 0
        and document.get("fault_diagnostic") is None
    )


def resident_torque_hold_matches(
    document: dict,
    *,
    required_owner: str = "",
    required_epoch: int = 0,
) -> bool:
    if not resident_torque_hold_is_valid(document):
        return False
    if required_owner and document.get("owner") != required_owner:
        return False
    if required_epoch > 0 and int(document.get("arbiter_epoch", 0)) != required_epoch:
        return False
    return True


def stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + (
        float(message.header.stamp.nanosec) * 1e-9
    )


def decode_image(message: Image) -> np.ndarray:
    if message.width <= 0 or message.height <= 0:
        raise ValueError("image dimensions must be positive")
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "mono8": 1,
    }
    if message.encoding not in channels_by_encoding:
        raise ValueError(f"unsupported encoding: {message.encoding}")
    channels = channels_by_encoding[message.encoding]
    expected_step = int(message.width) * channels
    if int(message.step) < expected_step:
        raise ValueError("image step is shorter than one packed row")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.step) * int(message.height)
    if raw.size < expected_size:
        raise ValueError("image data is truncated")
    rows = raw[:expected_size].reshape(int(message.height), int(message.step))
    packed = rows[:, :expected_step]
    if channels == 1:
        return cv2.cvtColor(
            packed.reshape(int(message.height), int(message.width)),
            cv2.COLOR_GRAY2BGR,
        )
    image = packed.reshape(
        int(message.height),
        int(message.width),
        channels,
    )
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def ordered_arm_positions(
    message: JointState,
    arm: str = "left",
) -> np.ndarray:
    if arm not in ARM_JOINT_NAMES_BY_SIDE:
        raise ValueError(f"unsupported arm: {arm}")
    joint_names = ARM_JOINT_NAMES_BY_SIDE[arm]
    positions = dict(zip(message.name, message.position, strict=True))
    missing = [name for name in joint_names if name not in positions]
    if missing:
        raise ValueError(f"joint state is missing {missing}")
    result = np.asarray(
        [positions[name] for name in joint_names],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(result)):
        raise ValueError("joint positions must be finite")
    return result


def detect_expected_gridboard(image: np.ndarray) -> tuple[int, ...]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(
        GRIDBOARD_DICTIONARY
    )
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    _, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )
    if ids is None:
        return ()
    return tuple(sorted(int(value) for value in ids.reshape(-1)))


def load_camera_model(path: Path) -> tuple[np.ndarray, np.ndarray]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))

    def matrix(key: str, rows: int, cols: int) -> np.ndarray:
        value = document.get(key, {})
        if value.get("rows") != rows or value.get("cols") != cols:
            raise ValueError(f"{key} has invalid dimensions")
        result = np.asarray(value.get("data", []), dtype=np.float64)
        if result.size != rows * cols or not np.all(np.isfinite(result)):
            raise ValueError(f"{key} has invalid values")
        return result.reshape(rows, cols)

    return (
        matrix("camera_matrix", 3, 3),
        matrix("distortion_coefficients", 1, 5).reshape(-1),
    )


def detect_expected_gridboard_pose(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[tuple[int, ...], np.ndarray | None, np.ndarray | None, float]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(GRIDBOARD_DICTIONARY)
    board = cv2.aruco.GridBoard_create(
        GRIDBOARD_MARKERS_X,
        GRIDBOARD_MARKERS_Y,
        GRIDBOARD_MARKER_LENGTH_M,
        GRIDBOARD_MARKER_SEPARATION_M,
        dictionary,
        EXPECTED_MARKER_IDS[0],
    )
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )
    if ids is None:
        return (), None, None, math.inf
    detected_ids = tuple(sorted(int(value) for value in ids.reshape(-1)))
    if detected_ids != EXPECTED_MARKER_IDS:
        return detected_ids, None, None, math.inf
    solved, rotation_vector, translation_vector = cv2.aruco.estimatePoseBoard(
        corners,
        ids,
        board,
        camera_matrix,
        distortion,
        None,
        None,
    )
    if int(solved) != len(EXPECTED_MARKER_IDS):
        return detected_ids, None, None, math.inf
    rotation, _ = cv2.Rodrigues(rotation_vector)
    board_points_by_id = {
        int(marker_id): np.asarray(points, dtype=np.float64)
        for marker_id, points in zip(
            board.ids.reshape(-1),
            board.objPoints,
            strict=True,
        )
    }
    squared_errors = []
    for detected_corners, marker_id in zip(
        corners,
        ids.reshape(-1),
        strict=True,
    ):
        pixels = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
        projected, _ = cv2.projectPoints(
            board_points_by_id[int(marker_id)],
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        residual = projected.reshape(4, 2) - pixels
        squared_errors.extend(
            float(value) for value in np.sum(residual * residual, axis=1)
        )
    return (
        detected_ids,
        translation_vector.reshape(3),
        rotation,
        math.sqrt(float(np.mean(squared_errors))),
    )


def maximum_target_pose_span(
    translations: list[np.ndarray],
    rotations: list[np.ndarray],
) -> tuple[float, float]:
    if not translations or len(translations) != len(rotations):
        raise ValueError("target poses must be non-empty and paired")
    maximum_translation_m = 0.0
    maximum_rotation_rad = 0.0
    for first in range(len(translations)):
        for second in range(first + 1, len(translations)):
            maximum_translation_m = max(
                maximum_translation_m,
                float(
                    np.linalg.norm(
                        translations[first] - translations[second]
                    )
                ),
            )
            relative = rotations[first].T @ rotations[second]
            cosine = float(
                np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
            )
            maximum_rotation_rad = max(
                maximum_rotation_rad,
                math.acos(cosine),
            )
    return maximum_translation_m, maximum_rotation_rad


def capture_window_spans(
    joint_positions: list[np.ndarray],
    target_translations: list[np.ndarray],
    target_rotations: list[np.ndarray],
) -> tuple[np.ndarray, float, float, float]:
    """Return joint and visual pose spans for one candidate window."""
    positions = np.asarray(joint_positions, dtype=np.float64)
    if positions.ndim != 2 or len(positions) == 0:
        raise ValueError("joint_positions must be a non-empty 2-D array")
    joint_span = np.ptp(positions, axis=0)
    translation_span_m, rotation_span_rad = maximum_target_pose_span(
        target_translations,
        target_rotations,
    )
    return (
        joint_span,
        float(np.max(joint_span)),
        float(translation_span_m * 1000.0),
        float(math.degrees(rotation_span_rad)),
    )


class EyeToHandSampleCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("top_eye_to_hand_sample_capture")
        self._args = args
        self._latest_joint_state: JointState | None = None
        self._images: list[np.ndarray] = []
        self._image_stamps: list[float] = []
        self._joint_positions: list[np.ndarray] = []
        self._target_translations: list[np.ndarray] = []
        self._target_rotations: list[np.ndarray] = []
        self._pnp_rms_pixels: list[float] = []
        self._rejected_windows = 0
        self._frames_received = 0
        self._waiting_for_joint_state = 0
        self._stamp_skew_rejections = 0
        self._marker_rejections = 0
        self._pnp_rejections = 0
        self._waiting_for_torque_hold = 0
        self._resident_status: dict | None = None
        self._resident_status_future = None
        self._resident_status_requests = 0
        self._resident_status_responses = 0
        self._resident_status_error = ""
        self._resident_status_last_state = "none"
        self._camera_matrix, self._distortion = load_camera_model(
            args.camera_info
        )
        self._last_capture_monotonic = -math.inf
        self._finished = False
        joint_qos: int | QoSProfile = 10
        if args.resident_torque_hold_anchor:
            joint_qos = QoSProfile(depth=1)
            joint_qos.reliability = ReliabilityPolicy.RELIABLE
            joint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._joint_subscription = self.create_subscription(
            JointState,
            args.joint_topic,
            self._on_joint_state,
            joint_qos,
        )
        self._image_subscription = self.create_subscription(
            Image,
            args.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._resident_status_client = None
        self._resident_status_timer = None
        if args.resident_torque_hold_anchor:
            self._resident_status_client = self.create_client(
                Trigger,
                RESIDENT_STATUS_SERVICE,
            )
            if not self._resident_status_client.wait_for_service(
                timeout_sec=min(args.timeout, 2.0)
            ):
                raise RuntimeError(
                    "resident status service unavailable: "
                    f"{RESIDENT_STATUS_SERVICE}"
                )
            self._resident_status_timer = self.create_timer(
                0.1,
                self._request_resident_status,
            )

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_joint_state(self, message: JointState) -> None:
        try:
            ordered_arm_positions(message, self._args.arm)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._latest_joint_state = message

    def _request_resident_status(self) -> None:
        client = self._resident_status_client
        if client is None:
            return
        future = self._resident_status_future
        if future is not None:
            if not future.done():
                return
            try:
                response = future.result()
                document = json.loads(response.message)
                self._resident_status_responses += 1
                self._resident_status_last_state = str(
                    document.get("state", "missing")
                )
                self._resident_status = (
                    document
                    if response.success
                    and resident_torque_hold_matches(
                        document,
                        required_owner=self._args.resident_required_owner,
                        required_epoch=self._args.resident_required_epoch,
                    )
                    else None
                )
                self._resident_status_error = ""
            except Exception as error:
                self._resident_status = None
                self._resident_status_error = (
                    f"{type(error).__name__}: {error}"
                )
            self._resident_status_future = None
        self._resident_status_future = client.call_async(Trigger.Request())
        self._resident_status_requests += 1

    def _on_image(self, message: Image) -> None:
        if self._finished:
            return
        self._frames_received += 1
        if (
            self._args.resident_torque_hold_anchor
            and self._resident_status is None
        ):
            self._waiting_for_torque_hold += 1
            return
        if self._latest_joint_state is None:
            self._waiting_for_joint_state += 1
            return
        now = time.monotonic()
        if now - self._last_capture_monotonic < self._args.interval:
            return
        image_stamp = stamp_seconds(message)
        joint_stamp = stamp_seconds(self._latest_joint_state)
        if image_stamp <= 0.0 or joint_stamp <= 0.0:
            self.get_logger().warning("zero source timestamp; frame rejected")
            return
        if (
            not self._args.resident_torque_hold_anchor
            and abs(image_stamp - joint_stamp) > self._args.max_stamp_skew
        ):
            self._stamp_skew_rejections += 1
            return
        try:
            image = decode_image(message)
            positions = ordered_arm_positions(
                self._latest_joint_state,
                self._args.arm,
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        detected_ids, target_translation, target_rotation, pnp_rms_px = (
            detect_expected_gridboard_pose(
                image,
                self._camera_matrix,
                self._distortion,
            )
        )
        if detected_ids != EXPECTED_MARKER_IDS:
            self._marker_rejections += 1
            self.get_logger().warning(
                "TOP_EYE_TO_HAND_MARKERS_INCOMPLETE "
                f"expected={EXPECTED_MARKER_IDS} detected={detected_ids}"
            )
            return
        if target_translation is None or target_rotation is None:
            self.get_logger().warning("TOP_EYE_TO_HAND_BOARD_POSE_FAILED")
            return
        if pnp_rms_px > self._args.max_pnp_rms_px:
            self._pnp_rejections += 1
            self.get_logger().warning(
                "TOP_EYE_TO_HAND_PNP_REJECTED "
                f"rms_px={pnp_rms_px:.3f} "
                f"limit_px={self._args.max_pnp_rms_px:.3f}"
            )
            return
        self._images.append(image)
        self._image_stamps.append(image_stamp)
        self._joint_positions.append(positions)
        self._target_translations.append(target_translation)
        self._target_rotations.append(target_rotation)
        self._pnp_rms_pixels.append(pnp_rms_px)
        self._last_capture_monotonic = now
        self.get_logger().info(
            "TOP_EYE_TO_HAND_FRAME_ACCEPTED "
            f"count={len(self._images)}/{self._args.frames}"
        )
        if len(self._images) >= self._args.frames:
            if self._write_capture():
                self._finished = True
            else:
                self._drop_oldest_frame()

    def _drop_oldest_frame(self) -> None:
        for values in (
            self._images,
            self._image_stamps,
            self._joint_positions,
            self._target_translations,
            self._target_rotations,
            self._pnp_rms_pixels,
        ):
            values.pop(0)

    def _write_capture(self) -> bool:
        (
            span,
            maximum_span,
            target_translation_span_mm,
            target_rotation_span_deg,
        ) = capture_window_spans(
            self._joint_positions,
            self._target_translations,
            self._target_rotations,
        )
        positions = np.asarray(self._joint_positions, dtype=np.float64)
        if maximum_span > self._args.max_joint_span:
            self._rejected_windows += 1
            self.get_logger().warning(
                "TOP_EYE_TO_HAND_WINDOW_REJECTED "
                "reason=robot moved during capture "
                f"max_joint_span_rad={maximum_span:.6f} "
                f"rejected_windows={self._rejected_windows}"
            )
            return False
        if (
            target_translation_span_mm
            > self._args.max_target_translation_span_mm
            or target_rotation_span_deg
            > self._args.max_target_rotation_span_deg
        ):
            self._rejected_windows += 1
            self.get_logger().warning(
                "TOP_EYE_TO_HAND_WINDOW_REJECTED "
                "reason=calibration target moved during capture "
                f"translation_span_mm={target_translation_span_mm:.3f} "
                f"rotation_span_deg={target_rotation_span_deg:.3f} "
                f"rejected_windows={self._rejected_windows}"
            )
            return False
        resident_status = self._resident_status
        resident_hold = self._args.resident_torque_hold_anchor
        if resident_hold and resident_status is None:
            return False
        output_directory = self._args.output_directory.resolve()
        output_directory.mkdir(parents=True, exist_ok=False)
        image_files = []
        for index, image in enumerate(self._images):
            image_path = output_directory / f"frame_{index:03d}.png"
            if not cv2.imwrite(str(image_path), image):
                raise RuntimeError(f"failed to write {image_path}")
            image_files.append(image_path.name)
        median_positions = np.median(positions, axis=0)
        document = {
            "schema_version": 1,
            "status": (
                "STATIONARY_RESIDENT_TORQUE_HOLD_CAPTURE_PASS"
                if resident_hold
                else "STATIONARY_READ_ONLY_CAPTURE_PASS"
            ),
            "motion_authorized": resident_hold,
            "robot_target_available": False,
            "capture": {
                "id": self._args.capture_id,
                "arm": self._args.arm,
                "measured_arm_rad": [
                    float(value) for value in median_positions
                ],
                "joint_span_rad": [float(value) for value in span],
                "target_translation_span_mm": target_translation_span_mm,
                "target_rotation_span_deg": target_rotation_span_deg,
                "pnp_rms_px_max": max(self._pnp_rms_pixels),
                "pnp_rms_px_limit": self._args.max_pnp_rms_px,
                "rejected_stability_windows": self._rejected_windows,
                "image_files": image_files,
                "image_source_stamp_first": self._image_stamps[0],
                "image_source_stamp_last": self._image_stamps[-1],
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
                "joint_state_source": (
                    "resident_terminal_measured_anchor"
                    if resident_hold
                    else "timestamp_synchronized_joint_state"
                ),
            },
        }
        if resident_hold:
            assert resident_status is not None
            document["resident_torque_hold"] = {
                "status_service": RESIDENT_STATUS_SERVICE,
                "owner": resident_status["owner"],
                "arbiter_epoch": int(resident_status["arbiter_epoch"]),
                "torque_hold_active": True,
                "terminal_anchor_stamp": stamp_seconds(
                    self._latest_joint_state
                ),
                "required_owner": self._args.resident_required_owner,
                "required_epoch": self._args.resident_required_epoch,
            }
        output_yaml = output_directory / "capture.yaml"
        output_yaml.write_text(
            yaml.safe_dump(document, sort_keys=False),
            encoding="utf-8",
        )
        print(
            "TOP_EYE_TO_HAND_SAMPLE_PASS "
            f"id={self._args.capture_id} frames={len(self._images)} "
            f"max_joint_span_rad={maximum_span:.6f} "
            f"target_span_mm={target_translation_span_mm:.3f} "
            f"target_span_deg={target_rotation_span_deg:.3f} "
            f"output={output_yaml}"
        )
        return True

    def timeout_diagnostic(self) -> str:
        return (
            "TOP_EYE_TO_HAND_CAPTURE_TIMEOUT "
            f"frames_received={self._frames_received} "
            f"accepted={len(self._images)} "
            f"waiting_for_joint_state={self._waiting_for_joint_state} "
            f"waiting_for_torque_hold={self._waiting_for_torque_hold} "
            f"status_requests={self._resident_status_requests} "
            f"status_responses={self._resident_status_responses} "
            f"status_last_state={self._resident_status_last_state} "
            f"status_error={self._resident_status_error!r} "
            f"stamp_skew_rejections={self._stamp_skew_rejections} "
            f"marker_rejections={self._marker_rejections} "
            f"pnp_rejections={self._pnp_rejections}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[3]
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--image-topic", default="/camera/top/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument(
        "--resident-torque-hold-anchor",
        action="store_true",
        help=(
            "Use the resident terminal measured anchor only while the status "
            "service proves an armed READY torque hold"
        ),
    )
    parser.add_argument("--resident-required-owner", default="")
    parser.add_argument("--resident-required-epoch", type=int, default=0)
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=root
        / "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml",
    )
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--max-stamp-skew", type=float, default=0.25)
    parser.add_argument("--max-joint-span", type=float, default=0.003)
    parser.add_argument("--max-pnp-rms-px", type=float, default=2.5)
    parser.add_argument(
        "--max-target-translation-span-mm", type=float, default=2.5
    )
    parser.add_argument(
        "--max-target-rotation-span-deg", type=float, default=1.0
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.frames < 5:
        parser.error("--frames must be at least 5")
    if args.interval <= 0.0:
        parser.error("--interval must be positive")
    if args.max_stamp_skew <= 0.0:
        parser.error("--max-stamp-skew must be positive")
    if args.max_joint_span <= 0.0:
        parser.error("--max-joint-span must be positive")
    if args.max_pnp_rms_px <= 0.0:
        parser.error("--max-pnp-rms-px must be positive")
    if args.max_target_translation_span_mm <= 0.0:
        parser.error("--max-target-translation-span-mm must be positive")
    if args.max_target_rotation_span_deg <= 0.0:
        parser.error("--max-target-rotation-span-deg must be positive")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if (
        args.resident_torque_hold_anchor
        and args.joint_topic != RESIDENT_ANCHOR_TOPIC
    ):
        parser.error(
            "--resident-torque-hold-anchor requires --joint-topic "
            f"{RESIDENT_ANCHOR_TOPIC}"
        )
    if args.resident_required_epoch < 0:
        parser.error("--resident-required-epoch must be non-negative")
    if (
        (args.resident_required_owner or args.resident_required_epoch)
        and not args.resident_torque_hold_anchor
    ):
        parser.error(
            "resident owner/epoch requirements need "
            "--resident-torque-hold-anchor"
        )
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = EyeToHandSampleCapture(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.finished:
            if time.monotonic() >= deadline:
                print(node.timeout_diagnostic())
                raise RuntimeError(
                    "capture timed out before all valid frames arrived"
                )
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
