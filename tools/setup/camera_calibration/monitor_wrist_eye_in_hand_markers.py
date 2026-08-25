#!/usr/bin/env python3
"""Live preview of the wrist camera with planar GridBoard marker detection
overlaid, so a pose can be checked for full board visibility before running
capture_wrist_eye_in_hand_sample.py.

Read-only monitoring only. Subscribes to an image topic, shows detected vs.
missing marker IDs on screen. Never publishes, moves anything, or writes any
file. Press 'q' or Ctrl+C to exit.
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


EXPECTED_MARKER_IDS = tuple(range(10, 30))
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
    image = packed.reshape(int(message.height), int(message.width), channels)
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


class MarkerMonitor(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("wrist_eye_in_hand_marker_monitor")
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._parameters = cv2.aruco.DetectorParameters_create()
        self._parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._latest = None
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

    def _on_image(self, message: Image) -> None:
        try:
            self._latest = decode_image(message)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def render(self):
        if self._latest is None:
            return None
        image = self._latest.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._parameters
        )
        detected = () if ids is None else tuple(
            sorted(int(value) for value in ids.reshape(-1))
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        missing = tuple(sorted(set(EXPECTED_MARKER_IDS) - set(detected)))
        complete = detected == EXPECTED_MARKER_IDS
        banner_color = (0, 200, 0) if complete else (0, 0, 220)
        cv2.rectangle(image, (0, 0), (image.shape[1], 24), banner_color, -1)
        status = "ALL 20 VISIBLE - READY TO CAPTURE" if complete else (
            f"missing={list(missing)}"
        )
        cv2.putText(
            image, f"detected={len(detected)}/20  {status}", (6, 17),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/wrist_a/image_raw")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MarkerMonitor(args.image_topic)
    window_name = f"{args.image_topic} eye-in-hand marker monitor"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            frame = node.render()
            if frame is not None:
                cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
