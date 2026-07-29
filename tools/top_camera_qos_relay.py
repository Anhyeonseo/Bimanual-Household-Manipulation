#!/usr/bin/env python3
"""Relay the Top camera image with an explicit reliable output QoS.

The camera manager publishes sensor data with BEST_EFFORT reliability.  The
ROS camera calibration tool can select an incompatible QoS when the remote
publisher has not yet been discovered during node construction.  This relay
subscribes with the sensor-data profile and republishes locally as RELIABLE.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image


INPUT_TOPIC = "/camera/top/image_raw"
OUTPUT_TOPIC = "/camera/top/calibration_image"


class TopCameraQosRelay(Node):
    def __init__(self) -> None:
        super().__init__("top_camera_qos_relay")

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            Image,
            OUTPUT_TOPIC,
            reliable_qos,
        )
        self._subscription = self.create_subscription(
            Image,
            INPUT_TOPIC,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._received = 0
        self._startup_timer = self.create_timer(5.0, self._check_startup)
        self.get_logger().info(
            f"waiting for Top images: {INPUT_TOPIC} -> {OUTPUT_TOPIC}"
        )

    def _on_image(self, message: Image) -> None:
        self._publisher.publish(message)
        self._received += 1
        if self._received == 1:
            self.get_logger().info(
                "FIRST_IMAGE_RELAYED "
                f"width={message.width} height={message.height} "
                f"encoding={message.encoding}"
            )

    def _check_startup(self) -> None:
        if self._received == 0:
            self.get_logger().warning(
                f"NO_IMAGE_RECEIVED topic={INPUT_TOPIC}; "
                "check ROS_DOMAIN_ID, RMW_IMPLEMENTATION, and the Pi camera manager"
            )
            return
        self.destroy_timer(self._startup_timer)


def main() -> None:
    rclpy.init()
    node = TopCameraQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
