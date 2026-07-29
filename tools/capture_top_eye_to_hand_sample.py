#!/usr/bin/env python3
"""Capture one stationary Top eye-to-hand calibration sample.

This node only subscribes to image and joint-state topics.  It never creates a
motion publisher, Action client, service client, or serial connection.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


ARM_JOINT_NAMES = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
EXPECTED_MARKER_IDS = (0, 1, 2, 3)


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


def ordered_arm_positions(message: JointState) -> np.ndarray:
    positions = dict(zip(message.name, message.position, strict=True))
    missing = [name for name in ARM_JOINT_NAMES if name not in positions]
    if missing:
        raise ValueError(f"joint state is missing {missing}")
    result = np.asarray(
        [positions[name] for name in ARM_JOINT_NAMES],
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


class EyeToHandSampleCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("top_eye_to_hand_sample_capture")
        self._args = args
        self._latest_joint_state: JointState | None = None
        self._images: list[np.ndarray] = []
        self._image_stamps: list[float] = []
        self._joint_positions: list[np.ndarray] = []
        self._last_capture_monotonic = -math.inf
        self._finished = False
        self._joint_subscription = self.create_subscription(
            JointState,
            args.joint_topic,
            self._on_joint_state,
            10,
        )
        self._image_subscription = self.create_subscription(
            Image,
            args.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_joint_state(self, message: JointState) -> None:
        try:
            ordered_arm_positions(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._latest_joint_state = message

    def _on_image(self, message: Image) -> None:
        if self._finished or self._latest_joint_state is None:
            return
        now = time.monotonic()
        if now - self._last_capture_monotonic < self._args.interval:
            return
        image_stamp = stamp_seconds(message)
        joint_stamp = stamp_seconds(self._latest_joint_state)
        if image_stamp <= 0.0 or joint_stamp <= 0.0:
            self.get_logger().warning("zero source timestamp; frame rejected")
            return
        if abs(image_stamp - joint_stamp) > self._args.max_stamp_skew:
            return
        try:
            image = decode_image(message)
            positions = ordered_arm_positions(self._latest_joint_state)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        detected_ids = detect_expected_gridboard(image)
        if detected_ids != EXPECTED_MARKER_IDS:
            self.get_logger().warning(
                "TOP_EYE_TO_HAND_MARKERS_INCOMPLETE "
                f"expected={EXPECTED_MARKER_IDS} detected={detected_ids}"
            )
            return
        self._images.append(image)
        self._image_stamps.append(image_stamp)
        self._joint_positions.append(positions)
        self._last_capture_monotonic = now
        self.get_logger().info(
            "TOP_EYE_TO_HAND_FRAME_ACCEPTED "
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
        document = {
            "schema_version": 1,
            "status": "STATIONARY_READ_ONLY_CAPTURE_PASS",
            "motion_authorized": False,
            "robot_target_available": False,
            "capture": {
                "id": self._args.capture_id,
                "measured_arm_rad": [
                    float(value) for value in median_positions
                ],
                "joint_span_rad": [float(value) for value in span],
                "image_files": image_files,
                "image_source_stamp_first": self._image_stamps[0],
                "image_source_stamp_last": self._image_stamps[-1],
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
            },
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
            f"output={output_yaml}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--image-topic", default="/camera/top/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
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
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = EyeToHandSampleCapture(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.finished:
            if time.monotonic() >= deadline:
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
