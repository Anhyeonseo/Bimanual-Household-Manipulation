#!/usr/bin/env python3
"""Capture one stationary wrist eye-in-hand calibration sample.

Mirrors capture_top_eye_to_hand_sample.py, but the roles are reversed: the
planar ArUco GridBoard (IDs 10-29) is fixed to the table and the camera moves
with the wrist. Each capture still requires the arm to hold still while the
image/joint_state pair is taken; the diverse poses come from repeating this
tool across a buffered-leg move between captures, not from motion within one
capture.

This node never commands motion.  Right-arm captures additionally read the
resident status service and its terminal measured anchor so a frame is accepted
only while the already-positioned arm is held under torque by the resident
adapter.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
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

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from capture_top_eye_to_hand_sample import (
    RESIDENT_ANCHOR_TOPIC,
    RESIDENT_STATUS_SERVICE,
    resident_torque_hold_matches,
)


ARM_JOINT_NAMES_BY_SIDE = {
    side: tuple(
        f"{side}_{joint}_joint"
        for joint in ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
    )
    for side in ("left", "right")
}
EXPECTED_MARKER_IDS = tuple(range(10, 30))


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
        cv2.aruco.DICT_4X4_50
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


class WristEyeInHandSampleCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wrist_eye_in_hand_sample_capture")
        self._args = args
        self._latest_joint_state: JointState | None = None
        self._images: list[np.ndarray] = []
        self._image_stamps: list[float] = []
        self._joint_positions: list[np.ndarray] = []
        self._last_capture_monotonic = -math.inf
        self._finished = False
        self._frames_received = 0
        self._waiting_for_torque_hold = 0
        self._resident_status: dict | None = None
        self._resident_status_monotonic = -math.inf
        self._resident_status_future = None
        self._resident_status_requests = 0
        self._resident_status_responses = 0
        self._resident_status_error = ""
        self._resident_status_last_state = "none"
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
                self._resident_status_monotonic = time.monotonic()
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
            and (
                self._resident_status is None
                or time.monotonic() - self._resident_status_monotonic > 0.5
            )
        ):
            self._waiting_for_torque_hold += 1
            return
        if self._latest_joint_state is None:
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
        detected_ids = detect_expected_gridboard(image)
        if detected_ids != EXPECTED_MARKER_IDS:
            self.get_logger().warning(
                "WRIST_EYE_IN_HAND_MARKERS_INCOMPLETE "
                f"expected={EXPECTED_MARKER_IDS} detected={detected_ids}"
            )
            return
        self._images.append(image)
        self._image_stamps.append(image_stamp)
        self._joint_positions.append(positions)
        self._last_capture_monotonic = now
        self.get_logger().info(
            "WRIST_EYE_IN_HAND_FRAME_ACCEPTED "
            f"count={len(self._images)}/{self._args.frames}"
        )
        if len(self._images) >= self._args.frames:
            self._write_capture()
            self._finished = True

    def _write_capture(self) -> None:
        positions = np.asarray(self._joint_positions)
        span = np.ptp(positions, axis=0)
        maximum_span = float(np.max(span))
        if maximum_span > self._args.max_joint_span:
            raise RuntimeError(
                "robot moved during capture: "
                f"max_joint_span_rad={maximum_span:.6f}"
            )
        output_directory = self._args.output_directory.resolve()
        output_directory.mkdir(parents=True, exist_ok=False)
        image_files = []
        for index, image in enumerate(self._images):
            image_path = output_directory / f"frame_{index:03d}.png"
            if not cv2.imwrite(str(image_path), image):
                raise RuntimeError(f"failed to write {image_path}")
            image_files.append(image_path.name)
        median_positions = np.median(positions, axis=0)
        if self._args.resident_torque_hold_anchor:
            status = "WRIST_EYE_IN_HAND_RESIDENT_TORQUE_HOLD_CAPTURE_PASS"
        elif self._args.route_target_only:
            status = "WRIST_ROUTE_TARGET_STATIONARY_CAPTURE_PASS"
        else:
            status = "WRIST_EYE_IN_HAND_STATIONARY_CAPTURE_PASS"
        document = {
            "schema_version": 1,
            "record_kind": (
                "right_wrist_visibility_route_target"
                if self._args.route_target_only
                else "wrist_eye_in_hand_capture"
            ),
            "status": status,
            "motion_authorized": self._args.resident_torque_hold_anchor,
            "source_motion_authorized": (
                self._args.resident_torque_hold_anchor
            ),
            "robot_target_available": self._args.route_target_only,
            "purpose": (
                "visibility_route_target_only"
                if self._args.route_target_only
                else "eye_in_hand_calibration"
            ),
            "arm": self._args.arm,
            "capture": {
                "id": self._args.capture_id,
                "arm": self._args.arm,
                "measured_arm_rad": [
                    float(value) for value in median_positions
                ],
                "joint_span_rad": [float(value) for value in span],
                "image_files": image_files,
                "image_source_stamp_first": self._image_stamps[0],
                "image_source_stamp_last": self._image_stamps[-1],
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
                "joint_state_source": (
                    "resident_terminal_measured_anchor"
                    if self._args.resident_torque_hold_anchor
                    else "timestamp_synchronized_joint_state"
                ),
            },
        }
        if self._args.resident_torque_hold_anchor:
            resident_status = self._resident_status
            if resident_status is None:
                raise RuntimeError("resident torque hold disappeared")
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
            "WRIST_EYE_IN_HAND_SAMPLE_PASS "
            f"id={self._args.capture_id} frames={len(self._images)} "
            f"max_joint_span_rad={maximum_span:.6f} "
            f"output={output_yaml}"
        )

    def timeout_diagnostic(self) -> str:
        return (
            "WRIST_EYE_IN_HAND_CAPTURE_TIMEOUT "
            f"frames_received={self._frames_received} "
            f"accepted={len(self._images)} "
            f"waiting_for_torque_hold={self._waiting_for_torque_hold} "
            f"status_requests={self._resident_status_requests} "
            f"status_responses={self._resident_status_responses} "
            f"status_last_state={self._resident_status_last_state} "
            f"status_error={self._resident_status_error!r}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--image-topic", default="/camera/wrist_a/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument(
        "--resident-torque-hold-anchor",
        action="store_true",
        help=(
            "Require an armed resident READY hold and consume its terminal "
            "measured anchor"
        ),
    )
    parser.add_argument(
        "--route-target-only",
        action="store_true",
        help=(
            "Record a motionless unarmed right-arm visibility waypoint; "
            "the images are not eligible for eye-in-hand calibration"
        ),
    )
    parser.add_argument("--resident-required-owner", default="")
    parser.add_argument("--resident-required-epoch", type=int, default=0)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--max-stamp-skew", type=float, default=0.25)
    parser.add_argument("--max-joint-span", type=float, default=0.003)
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
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if (
        args.arm == "right"
        and not args.resident_torque_hold_anchor
        and not args.route_target_only
    ):
        parser.error(
            "right wrist calibration requires "
            "--resident-torque-hold-anchor; use --route-target-only only "
            "for an unarmed visibility waypoint"
        )
    if (
        args.arm == "right"
        and args.resident_torque_hold_anchor
        and not args.resident_required_owner
    ):
        parser.error(
            "right wrist calibration requires an explicit "
            "--resident-required-owner"
        )
    if (
        args.arm == "right"
        and args.resident_torque_hold_anchor
        and args.resident_required_epoch <= 0
    ):
        parser.error(
            "right wrist calibration requires a positive "
            "--resident-required-epoch"
        )
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
    if args.route_target_only and (
        args.arm != "right" or args.resident_torque_hold_anchor
    ):
        parser.error(
            "--route-target-only is only for an unarmed right-arm waypoint"
        )
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
    node = WristEyeInHandSampleCapture(args)
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
