#!/usr/bin/env python3
"""Monitor the planar tabletop GridBoard before a calibration capture.

This process is read-only.  It subscribes to the Top image, overlays detected
ArUco markers, and evaluates the same fail-closed geometry gates used by
``calibrate_top_base_table.py``.  It never publishes or commands robot motion.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from calibrate_top_base_table import (
    classify,
    detect_gridboard,
    evaluate_plane,
    load_yaml,
    matrix,
    planar_gridboard,
)
from capture_top_frame import decode_image


WINDOW_NAME = "SO101 Top table GridBoard monitor (q/ESC to close)"
DEFAULT_CAMERA_INFO = Path(
    "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml"
)
DEFAULT_EYE_TO_HAND = Path(
    "artifacts/top_eye_to_hand/2026-07-30/"
    "independent_validation/candidate.yaml"
)


class TopBaseTableGridBoardMonitor(Node):
    """Show a live, read-only capture-readiness preview."""

    def __init__(
        self,
        topic: str,
        camera_info_path: Path,
        eye_to_hand_path: Path,
        table_z_m: float,
        stale_timeout_s: float,
        analysis_rate_hz: float,
    ) -> None:
        super().__init__("top_base_table_gridboard_monitor")
        camera_info = load_yaml(camera_info_path)
        eye_to_hand = load_yaml(eye_to_hand_path)
        if not str(eye_to_hand.get("status", "")).startswith(
            "EYE_TO_HAND_VALIDATED_"
        ):
            raise RuntimeError("eye-to-hand input is not independently validated")

        self._camera_matrix = matrix(camera_info, "camera_matrix", 3, 3)
        self._distortion = matrix(
            camera_info, "distortion_coefficients", 1, 5
        ).reshape(-1)
        self._base_from_camera = matrix(eye_to_hand, "base_to_camera", 4, 4)
        self._table_z_m = float(table_z_m)
        self._topic = topic
        self._stale_timeout_s = float(stale_timeout_s)
        self._analysis_period_s = 1.0 / float(analysis_rate_hz)
        self._dictionary, _, self._expected_ids = planar_gridboard()
        self._detector_parameters = cv2.aruco.DetectorParameters_create()
        self._detector_parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )
        self._latest_frame: np.ndarray | None = None
        self._last_frame_at: float | None = None
        self._last_analysis_at = 0.0
        self._diagnostic_lines = ["Waiting for GridBoard IDs 10-29"]
        self._ready = False
        self._closed = False
        self._frame_count = 0
        self.create_subscription(
            Image,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 720)
        self.get_logger().info(
            "TOP_BASE_TABLE_GRIDBOARD_MONITOR_READY "
            f"topic={topic} expected_ids={list(self._expected_ids)} "
            "motion_commands=0"
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

    def _analyze(self, frame: np.ndarray, now: float) -> None:
        if now - self._last_analysis_at < self._analysis_period_s:
            return
        self._last_analysis_at = now
        try:
            detection = detect_gridboard(
                frame,
                self._camera_matrix,
                self._distortion,
            )
            base_from_grid = (
                self._base_from_camera @ detection["camera_from_grid"]
            )
            plane = evaluate_plane(
                base_from_grid,
                detection["object_points"],
                self._table_z_m,
            )
            _, failures = classify(
                detection["pnp_rms_px"],
                detection["image_border_px"],
                plane["tilt_deg"],
                plane["height_error_max_mm"],
            )
            self._ready = not failures
            self._diagnostic_lines = [
                (
                    f"IDs 10-29 COMPLETE  scale={detection['detection_scale']:.2f}"
                ),
                (
                    f"PnP={detection['pnp_rms_px']:.3f}px  "
                    f"border={detection['image_border_px']:.1f}px"
                ),
                (
                    f"tilt={plane['tilt_deg']:.2f}deg  "
                    f"height_err={plane['height_error_max_mm']:.2f}mm"
                ),
            ]
            if failures:
                self._diagnostic_lines.append("FAIL: " + "; ".join(failures))
        except RuntimeError as error:
            self._ready = False
            self._diagnostic_lines = [str(error)]

    def _draw_markers(self, frame: np.ndarray) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self._dictionary,
            parameters=self._detector_parameters,
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    def render_once(self) -> None:
        now = time.monotonic()
        if self._latest_frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "WAITING FOR TOP CAMERA",
                (48, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            frame = self._latest_frame.copy()
            self._analyze(frame, now)
            self._draw_markers(frame)

        color = (0, 150, 0) if self._ready else (0, 0, 190)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 116), color, -1)
        heading = "READY TO CAPTURE" if self._ready else "NOT READY"
        cv2.putText(
            frame,
            heading,
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        for index, line in enumerate(self._diagnostic_lines[:4]):
            cv2.putText(
                frame,
                line[:92],
                (10, 44 + index * 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        age = (
            float("inf")
            if self._last_frame_at is None
            else now - self._last_frame_at
        )
        if age > self._stale_timeout_s:
            cv2.putText(
                frame,
                "NO LIVE TOP FRAME" if not np.isfinite(age) else f"STALE {age:.1f}s",
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
    parser.add_argument("--camera-info", type=Path, default=DEFAULT_CAMERA_INFO)
    parser.add_argument("--eye-to-hand", type=Path, default=DEFAULT_EYE_TO_HAND)
    parser.add_argument("--table-z-m", type=float, default=-0.005)
    parser.add_argument("--stale-timeout-s", type=float, default=2.0)
    parser.add_argument("--analysis-rate-hz", type=float, default=2.0)
    args = parser.parse_args()
    if args.stale_timeout_s <= 0.0:
        parser.error("--stale-timeout-s must be positive")
    if args.analysis_rate_hz <= 0.0:
        parser.error("--analysis-rate-hz must be positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TopBaseTableGridBoardMonitor(
        args.image_topic,
        args.camera_info.resolve(),
        args.eye_to_hand.resolve(),
        args.table_z_m,
        args.stale_timeout_s,
        args.analysis_rate_hz,
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
