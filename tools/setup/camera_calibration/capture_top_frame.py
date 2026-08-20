#!/usr/bin/env python3
"""Capture one settled Top-camera frame with explicit sensor-data QoS."""

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


class TopFrameCapture(Node):
    def __init__(
        self,
        output: Path,
        timeout_seconds: float,
        settle_frames: int,
    ) -> None:
        super().__init__("top_frame_capture")
        self.output = output
        self.deadline = time.monotonic() + timeout_seconds
        self.settle_frames = settle_frames
        self.received = 0
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
            f"waiting for Top image on {TOPIC}; "
            f"settle_frames={settle_frames}"
        )

    def _on_image(self, message: Image) -> None:
        if self.completed or self.failure is not None:
            return
        self.received += 1
        if self.received <= self.settle_frames:
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
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output), image):
            self.failure = f"failed to write {self.output}"
            return

        self.get_logger().info(
            "TOP_FRAME_CAPTURE_PASS "
            f"path={self.output} width={message.width} height={message.height} "
            f"encoding={message.encoding} sharpness={sharpness:.2f}"
        )
        self.completed = True

    def _check_timeout(self) -> None:
        if time.monotonic() >= self.deadline:
            self.failure = f"timeout waiting for {TOPIC}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one settled Top-camera frame."
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--settle-frames", type=int, default=5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.settle_frames < 0:
        raise ValueError("--settle-frames must be non-negative")

    rclpy.init()
    node = TopFrameCapture(
        args.output.resolve(),
        args.timeout,
        args.settle_frames,
    )
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
    print(f"TOP_FRAME_CAPTURE_FAIL {node.failure}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
