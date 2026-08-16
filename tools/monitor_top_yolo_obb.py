#!/usr/bin/env python3
"""Display the Top YOLO-OBB debug topic without any motion capability."""

from __future__ import annotations

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WINDOW_NAME = "SO101 Top YOLO-OBB monitor (q/ESC to close)"


def decode_rgb8(message: Image) -> np.ndarray:
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError(
            f"debug image encoding must be rgb8 or bgr8, got {message.encoding}"
        )
    required_step = int(message.width) * 3
    if int(message.step) < required_step:
        raise ValueError("debug image step is too small")
    required_bytes = int(message.step) * int(message.height)
    if len(message.data) < required_bytes:
        raise ValueError("debug image data is truncated")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        int(message.height),
        int(message.step),
    )
    image = rows[:, :required_step].reshape(
        int(message.height),
        int(message.width),
        3,
    )
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


class TopYoloMonitor(Node):
    def __init__(self, topic: str, stale_timeout_s: float) -> None:
        super().__init__("top_yolo_obb_visual_monitor")
        self._topic = topic
        self._stale_timeout_s = stale_timeout_s
        self._last_frame_at: float | None = None
        self._last_warning_at = 0.0
        self._closed = False
        self._latest_frame: np.ndarray | None = None
        self._frame_count = 0
        self.create_subscription(
            Image,
            topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self._check_stale)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 720)
        self.get_logger().info(
            f"TOP_YOLO_VISUAL_MONITOR_READY topic={topic} motion_commands=0"
        )

    @property
    def closed(self) -> bool:
        return self._closed

    def _callback(self, message: Image) -> None:
        try:
            frame = decode_rgb8(message)
        except Exception as error:
            self.get_logger().error(f"debug frame rejected: {error}")
            return
        self._last_frame_at = time.monotonic()
        self._latest_frame = frame
        self._frame_count += 1

    def render_once(self) -> None:
        now = time.monotonic()
        if self._latest_frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "WAITING FOR YOLO DEBUG FRAMES",
                (42, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                self._topic,
                (42, 235),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
            status = (
                f"publishers={self.count_publishers(self._topic)}  motion=OFF"
            )
            cv2.putText(
                frame,
                status,
                (42, 275),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (80, 80, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            frame = self._latest_frame.copy()
            elapsed = now - (self._last_frame_at or now)
            status = (
                f"monitor_frames={self._frame_count}  "
                f"receive_age={elapsed:.2f}s"
            )
            cv2.putText(
                frame,
                status,
                (12, frame.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                status,
                (12, frame.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )
            if elapsed > self._stale_timeout_s:
                cv2.putText(
                    frame,
                    f"STALE FRAME: {elapsed:.1f}s",
                    (12, frame.shape[0] - 46),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self._closed = True
        elif cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            self._closed = True

    def _check_stale(self) -> None:
        now = time.monotonic()
        reference = self._last_frame_at
        elapsed = math.inf if reference is None else now - reference
        if elapsed <= self._stale_timeout_s:
            return
        if now - self._last_warning_at >= self._stale_timeout_s:
            label = "no frames received" if reference is None else f"stale {elapsed:.1f}s"
            self.get_logger().warning(
                f"TOP_YOLO_VISUAL_MONITOR_NO_FRAME topic={self._topic} {label}"
            )
            self._last_warning_at = now


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/perception/top/yolo_obb_debug",
    )
    parser.add_argument("--stale-timeout-s", type=float, default=2.0)
    args = parser.parse_args()
    if args.stale_timeout_s <= 0.0:
        parser.error("--stale-timeout-s must be positive")

    rclpy.init()
    node = TopYoloMonitor(args.topic, args.stale_timeout_s)
    try:
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=0.03)
            node.render_once()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
