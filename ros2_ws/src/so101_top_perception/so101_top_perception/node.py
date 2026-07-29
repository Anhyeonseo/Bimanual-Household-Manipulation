"""ROS 2 wrapper for fail-closed Top-camera object-pose detection."""

from __future__ import annotations

import math
import time
from pathlib import Path

import cv2

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from sensor_msgs.msg import Image

from so101_interfaces.msg import TopObjectPose

from .detector import (
    DetectionError,
    DetectorConfig,
    detect_one_object,
    frame_age_seconds,
    load_calibration,
)


class TopObjectPoseNode(Node):
    """Publish valid board observations without authorizing robot motion."""

    def __init__(self) -> None:
        super().__init__("top_object_pose")

        self.declare_parameter("camera_info_path", "")
        self.declare_parameter("homography_path", "")
        image_topic = self.declare_parameter(
            "image_topic",
            "/camera/top/image_raw",
        ).value
        pose_topic = self.declare_parameter(
            "pose_topic",
            "/perception/top/object_pose_board",
        ).value
        diagnostics_topic = self.declare_parameter(
            "diagnostics_topic",
            "/perception/top/diagnostics",
        ).value
        self._output_frame_id = self.declare_parameter(
            "output_frame_id",
            "top_board",
        ).value
        self._max_frame_age_s = float(
            self.declare_parameter("max_frame_age_s", 0.2).value
        )
        self._future_tolerance_s = float(
            self.declare_parameter("future_tolerance_s", 0.05).value
        )
        self._stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", 1.5).value
        )
        diagnostics_period_s = float(
            self.declare_parameter("diagnostics_period_s", 0.5).value
        )
        exclusion_values = tuple(
            int(value)
            for value in self.declare_parameter(
                "exclusion_rectangles_px",
                [0, 0, 1, 1],
            ).value
        )
        if len(exclusion_values) % 4 != 0:
            raise ValueError(
                "exclusion_rectangles_px must contain x,y,w,h groups"
            )
        exclusion_rectangles = tuple(
            exclusion_values[index : index + 4]
            for index in range(0, len(exclusion_values), 4)
        )
        self._detector_config = DetectorConfig(
            threshold=int(self.declare_parameter("threshold", 110).value),
            min_area_px=float(
                self.declare_parameter("min_area_px", 1000.0).value
            ),
            min_width_px=int(
                self.declare_parameter("min_width_px", 20).value
            ),
            min_height_px=int(
                self.declare_parameter("min_height_px", 20).value
            ),
            min_solidity=float(
                self.declare_parameter("min_solidity", 0.5).value
            ),
            image_edge_margin_px=int(
                self.declare_parameter("image_edge_margin_px", 8).value
            ),
            exclusion_rectangles_px=exclusion_rectangles,
        )
        self._allow_partial_footprint = bool(
            self.declare_parameter(
                "allow_partial_footprint_observation",
                False,
            ).value
        )
        self._validate_parameters(diagnostics_period_s)

        camera_info_value = str(
            self.get_parameter("camera_info_path").value
        )
        homography_value = str(
            self.get_parameter("homography_path").value
        )
        if not camera_info_value or not homography_value:
            raise ValueError(
                "camera_info_path and homography_path must be configured"
            )
        camera_info_path = Path(camera_info_value)
        homography_path = Path(homography_value)
        self._calibration = load_calibration(
            camera_info_path,
            homography_path,
        )

        self._pose_publisher = self.create_publisher(
            TopObjectPose,
            pose_topic,
            QoSProfile(depth=1),
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            diagnostics_topic,
            10,
        )
        self._subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._started_at = time.monotonic()
        self._last_image_received_at = None
        self._last_frame_age_s = math.nan
        self._last_diagnostic_code = ""
        self._timer = self.create_timer(
            diagnostics_period_s,
            self._publish_stream_diagnostic,
        )

        if self._calibration.motion_authorized:
            self.get_logger().warning(
                "calibration requests motion authorization, but this "
                "perception node forces motion_authorized=false"
            )
        self.get_logger().info(
            "TOP_PERCEPTION_READY input=%s output=%s frame=%s "
            "motion_authorized=false"
            % (image_topic, pose_topic, self._output_frame_id)
        )

    def _validate_parameters(self, diagnostics_period_s: float) -> None:
        self._detector_config.validate()
        if not self._output_frame_id:
            raise ValueError("output_frame_id must not be empty")
        if self._max_frame_age_s <= 0.0:
            raise ValueError("max_frame_age_s must be positive")
        if self._future_tolerance_s < 0.0:
            raise ValueError("future_tolerance_s must not be negative")
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        if diagnostics_period_s <= 0.0:
            raise ValueError("diagnostics_period_s must be positive")

    @staticmethod
    def _decode_image(message: Image) -> np.ndarray:
        if message.height == 0 or message.width == 0:
            raise DetectionError("DECODE_ERROR", "image dimensions are zero")
        channels = 1 if message.encoding == "mono8" else 3
        if message.encoding not in ("rgb8", "bgr8", "mono8"):
            raise DetectionError(
                "DECODE_ERROR",
                f"unsupported encoding: {message.encoding}",
            )
        required_step = int(message.width) * channels
        if int(message.step) < required_step:
            raise DetectionError("DECODE_ERROR", "image step is too small")
        required_bytes = int(message.step) * int(message.height)
        if len(message.data) < required_bytes:
            raise DetectionError("DECODE_ERROR", "image data is truncated")

        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            int(message.height),
            int(message.step),
        )
        pixels = rows[:, :required_step]
        if message.encoding == "mono8":
            gray = pixels.reshape(int(message.height), int(message.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        color = pixels.reshape(int(message.height), int(message.width), 3)
        if message.encoding == "rgb8":
            return cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        return color.copy()

    def _image_callback(self, message: Image) -> None:
        self._last_image_received_at = time.monotonic()
        try:
            frame_age_s = frame_age_seconds(
                self.get_clock().now().nanoseconds,
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
                self._max_frame_age_s,
                self._future_tolerance_s,
            )
            image = self._decode_image(message)
            pose = detect_one_object(
                image,
                self._calibration,
                self._detector_config,
                require_full_footprint=(
                    not self._allow_partial_footprint
                ),
            )
        except DetectionError as error:
            self._last_frame_age_s = math.nan
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                error.code,
                str(error),
            )
            return
        except Exception as error:
            self._last_frame_age_s = math.nan
            self._publish_diagnostic(
                DiagnosticStatus.ERROR,
                "PROCESSING_ERROR",
                str(error),
            )
            return

        output = TopObjectPose()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._output_frame_id
        output.x_m = pose["board_position_m"][0]
        output.y_m = pose["board_position_m"][1]
        output.yaw_rad = pose["yaw_rad"]
        output.confidence = float(
            min(1.0, max(0.0, pose["solidity"]))
        )
        output.frame_age_s = float(frame_age_s)
        footprint_inside = bool(
            pose["calibration_region"]["footprint_inside"]
        )
        output.footprint_inside = footprint_inside
        output.image_fully_visible = bool(
            pose["calibration_region"]["image_fully_visible"]
        )
        output.motion_authorized = False
        output.robot_target_available = False
        output.detector_status = (
            "TRACKING_BOARD_ONLY"
            if footprint_inside
            else "TRACKING_CENTER_CALIBRATED_FULLY_VISIBLE"
        )
        self._pose_publisher.publish(output)

        self._last_frame_age_s = frame_age_s
        if footprint_inside:
            self._publish_diagnostic(
                DiagnosticStatus.OK,
                "TRACKING_BOARD_ONLY",
                "one valid board-relative observation",
                pose=pose,
            )
        else:
            self._publish_diagnostic(
                DiagnosticStatus.OK,
                "TRACKING_CENTER_CALIBRATED_FULLY_VISIBLE",
                "center is calibrated and the full object is visible",
                pose=pose,
            )

    @staticmethod
    def _value(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostic(
        self,
        level: int,
        code: str,
        reason: str,
        pose: dict | None = None,
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "top_perception/object_pose"
        status.hardware_id = "top_camera"
        status.message = code
        status.values = [
            self._value("reason", reason),
            self._value("frame_id", self._output_frame_id),
            self._value(
                "frame_age_s",
                (
                    f"{self._last_frame_age_s:.6f}"
                    if math.isfinite(self._last_frame_age_s)
                    else "unknown"
                ),
            ),
            self._value(
                "footprint_inside",
                (
                    pose["calibration_region"]["footprint_inside"]
                    if pose is not None
                    else False
                ),
            ),
            self._value(
                "image_fully_visible",
                (
                    pose["calibration_region"]["image_fully_visible"]
                    if pose is not None
                    else False
                ),
            ),
            self._value("motion_authorized", False),
            self._value("robot_target_available", False),
            self._value(
                "homography_status",
                self._calibration.homography_status,
            ),
            self._value(
                "base_registration_status",
                self._calibration.base_registration_status,
            ),
        ]
        if pose is not None:
            status.values.extend(
                [
                    self._value("x_m", f"{pose['board_position_m'][0]:.6f}"),
                    self._value("y_m", f"{pose['board_position_m'][1]:.6f}"),
                    self._value("yaw_rad", f"{pose['yaw_rad']:.6f}"),
                    self._value("confidence", f"{pose['solidity']:.6f}"),
                ]
            )
        array.status = [status]
        self._diagnostics_publisher.publish(array)
        if code != self._last_diagnostic_code:
            message = f"TOP_PERCEPTION_{code} reason={reason}"
            if level == DiagnosticStatus.ERROR:
                self.get_logger().error(message)
            elif level == DiagnosticStatus.WARN:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self._last_diagnostic_code = code

    def _publish_stream_diagnostic(self) -> None:
        last_received = self._last_image_received_at
        reference = self._started_at if last_received is None else last_received
        elapsed = time.monotonic() - reference
        if elapsed > self._stale_timeout_s:
            code = "NO_IMAGE" if last_received is None else "STALE_STREAM"
            self._last_frame_age_s = math.nan
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                code,
                f"no image received for {elapsed:.3f}s",
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TopObjectPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
