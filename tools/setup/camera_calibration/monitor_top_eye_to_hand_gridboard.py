#!/usr/bin/env python3
"""Live Top-camera preview for the TCP 2x2 ArUco GridBoard calibration target.

Read-only monitor: it subscribes to one image topic only and never writes,
publishes, connects to the robot, or sends a motion command.  Press q or Esc
to close the preview.
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from capture_top_eye_to_hand_sample import (
    EXPECTED_MARKER_IDS,
    decode_image,
    detect_expected_gridboard_pose,
    load_camera_model,
)


WINDOW_NAME = "SO101 Top GridBoard monitor (q/ESC to close)"


class TopGridBoardMonitor(Node):
    def __init__(
        self,
        topic: str,
        stale_timeout_s: float,
        camera_info: Path | None,
        max_pnp_rms_px: float,
    ) -> None:
        super().__init__("top_eye_to_hand_gridboard_monitor")
        self._topic = topic
        self._stale_timeout_s = stale_timeout_s
        self._max_pnp_rms_px = max_pnp_rms_px
        self._camera_model = (
            None if camera_info is None else load_camera_model(camera_info)
        )
        self._readiness = deque(maxlen=30)
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self._parameters = cv2.aruco.DetectorParameters_create()
        self._parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._latest_frame: np.ndarray | None = None
        self._last_frame_at: float | None = None
        self._frame_count = 0
        self._closed = False
        self.create_subscription(
            Image,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 720)
        self.get_logger().info(
            "TOP_GRIDBOARD_MONITOR_READY "
            f"topic={topic} expected_ids={EXPECTED_MARKER_IDS} motion_commands=0"
        )

    @property
    def closed(self) -> bool:
        return self._closed

    def _on_image(self, message: Image) -> None:
        try:
            self._latest_frame = decode_image(message)
        except ValueError as error:
            self.get_logger().warning(f"frame rejected: {error}")
            return
        self._last_frame_at = time.monotonic()
        self._frame_count += 1

    def render_once(self) -> None:
        now = time.monotonic()
        if self._latest_frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "WAITING FOR TOP CAMERA",
                (52, 210),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                self._topic,
                (52, 250),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
        else:
            frame = self._latest_frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self._dictionary,
                parameters=self._parameters,
            )
            detected = () if ids is None else tuple(
                sorted(int(value) for value in ids.reshape(-1))
            )
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            missing = tuple(
                marker_id
                for marker_id in EXPECTED_MARKER_IDS
                if marker_id not in detected
            )
            markers_complete = detected == EXPECTED_MARKER_IDS
            pnp_rms_px = math.inf
            capture_ready = markers_complete
            if markers_complete and self._camera_model is not None:
                camera_matrix, distortion = self._camera_model
                _, translation, rotation, pnp_rms_px = (
                    detect_expected_gridboard_pose(
                        frame,
                        camera_matrix,
                        distortion,
                    )
                )
                capture_ready = (
                    translation is not None
                    and rotation is not None
                    and pnp_rms_px <= self._max_pnp_rms_px
                )
            self._readiness.append(capture_ready)
            color = (0, 180, 0) if capture_ready else (0, 0, 220)
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 72), color, -1)
            if not markers_complete:
                status = f"GRIDBOARD INCOMPLETE: detected={list(detected)} missing={list(missing)}"
            elif self._camera_model is None:
                status = "GRIDBOARD READY: IDs 0,1,2,3 all visible"
            elif capture_ready:
                status = f"CAPTURE READY: PnP RMS={pnp_rms_px:.3f}px"
            else:
                status = (
                    f"PNP REJECTED: RMS={pnp_rms_px:.3f}px "
                    f"limit={self._max_pnp_rms_px:.3f}px"
                )
            cv2.putText(
                frame,
                status,
                (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            age = now - (self._last_frame_at or now)
            ready_ratio = sum(self._readiness) / len(self._readiness)
            diagnostics = (
                f"frames={self._frame_count} receive_age={age:.2f}s "
                f"ready={ready_ratio:.0%}/{len(self._readiness)} "
                f"topic={self._topic}"
            )
            cv2.putText(
                frame,
                diagnostics,
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            if age > self._stale_timeout_s:
                cv2.putText(
                    frame,
                    f"STALE FRAME: {age:.1f}s",
                    (12, frame.shape[0] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/top/image_raw")
    parser.add_argument("--stale-timeout-s", type=float, default=2.0)
    parser.add_argument("--camera-info", type=Path)
    parser.add_argument("--max-pnp-rms-px", type=float, default=2.5)
    args = parser.parse_args()
    if args.stale_timeout_s <= 0.0:
        parser.error("--stale-timeout-s must be positive")
    if args.max_pnp_rms_px <= 0.0:
        parser.error("--max-pnp-rms-px must be positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TopGridBoardMonitor(
        args.image_topic,
        args.stale_timeout_s,
        args.camera_info,
        args.max_pnp_rms_px,
    )
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
