#!/usr/bin/env python3
"""Capture one Top-camera frame after a complete 7x7 chessboard is detected."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PATTERN_SIZE = (7, 7)
TOPIC = "/camera/top/image_raw"


def decode_image(message: Image) -> np.ndarray:
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "mono8": 1,
    }
    channels = channels_by_encoding.get(message.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding}")

    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    pixels = rows[:, : message.width * channels]
    if channels == 1:
        return pixels.reshape(message.height, message.width).copy()

    image = pixels.reshape(message.height, message.width, channels).copy()
    if message.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


class HomographyCapture(Node):
    def __init__(self, output: Path, timeout_seconds: float) -> None:
        super().__init__("top_homography_capture")
        self.output = output
        self.deadline = time.monotonic() + timeout_seconds
        self.completed = False
        self.failure: str | None = None
        self.subscription = self.create_subscription(
            Image,
            TOPIC,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.2, self._check_timeout)
        self.get_logger().info(
            f"waiting for complete 7x7 chessboard on {TOPIC}"
        )

    def _on_image(self, message: Image) -> None:
        if self.completed or self.failure is not None:
            return

        try:
            image = decode_image(message)
        except ValueError as error:
            self.failure = str(error)
            return

        gray = (
            image
            if image.ndim == 2
            else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        )
        found, corners = cv2.findChessboardCorners(
            gray,
            PATTERN_SIZE,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK,
        )
        if not found:
            return

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
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output), image):
            self.failure = f"failed to write {self.output}"
            return

        first = corners[0, 0]
        last = corners[-1, 0]
        self.get_logger().info(
            "HOMOGRAPHY_CAPTURE_PASS "
            f"path={self.output} width={message.width} height={message.height} "
            f"encoding={message.encoding} corners={len(corners)} "
            f"sharpness={sharpness:.2f} "
            f"first_px=({first[0]:.2f},{first[1]:.2f}) "
            f"last_px=({last[0]:.2f},{last[1]:.2f})"
        )
        self.completed = True

    def _check_timeout(self) -> None:
        if time.monotonic() >= self.deadline:
            self.failure = (
                f"timeout: no complete 7x7 chessboard detected on {TOPIC}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one Top image with a detected 7x7 chessboard."
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = HomographyCapture(args.output.resolve(), args.timeout)
    try:
        while (
            rclpy.ok()
            and not node.completed
            and node.failure is None
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.failure = "interrupted"
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.completed:
        return 0
    print(f"HOMOGRAPHY_CAPTURE_FAIL {node.failure}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
