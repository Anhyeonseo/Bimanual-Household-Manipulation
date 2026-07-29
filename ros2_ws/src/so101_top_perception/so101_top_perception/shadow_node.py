"""ROS 2 node for non-actionable Top-to-base shadow targets."""

from __future__ import annotations

from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from so101_interfaces.msg import ShadowObjectTarget, TopObjectPose

from .shadow_target import (
    BoardObservation,
    evaluate_shadow,
    load_shadow_config,
    ShadowTargetError,
    source_stamp_age_seconds,
)


class TopShadowTargetNode(Node):
    """Transform observations while permanently withholding motion authority."""

    def __init__(self) -> None:
        super().__init__('top_shadow_target')
        self.declare_parameter('config_path', '')
        input_topic = self.declare_parameter(
            'input_topic',
            '/perception/top/object_pose_board',
        ).value
        output_topic = self.declare_parameter(
            'output_topic',
            '/perception/top/object_shadow_left_base',
        ).value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/perception/top/shadow_diagnostics',
        ).value
        self._stale_timeout_s = float(
            self.declare_parameter('stale_timeout_s', 1.0).value
        )
        diagnostics_period_s = float(
            self.declare_parameter('diagnostics_period_s', 0.5).value
        )
        if self._stale_timeout_s <= 0.0 or diagnostics_period_s <= 0.0:
            raise ValueError('diagnostic timing parameters must be positive')

        config_value = str(self.get_parameter('config_path').value)
        if not config_value:
            raise ValueError('config_path must be configured')
        self._config = load_shadow_config(Path(config_value))
        self._publisher = self.create_publisher(
            ShadowObjectTarget,
            output_topic,
            QoSProfile(depth=1),
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            diagnostics_topic,
            10,
        )
        self._subscription = self.create_subscription(
            TopObjectPose,
            input_topic,
            self._pose_callback,
            QoSProfile(depth=1),
        )
        self._started_at = time.monotonic()
        self._last_received_at = None
        self._last_code = ''
        self._timer = self.create_timer(
            diagnostics_period_s,
            self._publish_stream_diagnostic,
        )
        self.get_logger().info(
            'TOP_SHADOW_READY input=%s output=%s frame=%s '
            'transform_validated=%s motion_authorized=false '
            'robot_target_available=false rejected_mixed_reference_mm=%.3f'
            % (
                input_topic,
                output_topic,
                self._config.output_frame,
                str(self._config.transform_validated).lower(),
                self._config.rejected_mixed_reference_disagreement_mm,
            )
        )

    @staticmethod
    def _value(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def _pose_callback(self, message: TopObjectPose) -> None:
        self._last_received_at = time.monotonic()
        try:
            stamp_age_s = source_stamp_age_seconds(
                self.get_clock().now().nanoseconds,
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
                self._config.max_frame_age_s,
                self._config.future_tolerance_s,
            )
            frame_age_s = max(float(message.frame_age_s), stamp_age_s)
            observation = BoardObservation(
                source_frame=str(message.header.frame_id),
                x_m=float(message.x_m),
                y_m=float(message.y_m),
                yaw_rad=float(message.yaw_rad),
                frame_age_s=frame_age_s,
                confidence=float(message.confidence),
                footprint_inside=bool(message.footprint_inside),
                image_fully_visible=bool(message.image_fully_visible),
                motion_authorized=bool(message.motion_authorized),
                robot_target_available=bool(message.robot_target_available),
            )
            result = evaluate_shadow(self._config, observation)
        except ShadowTargetError as error:
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                error.code,
                str(error),
            )
            return
        except Exception as error:
            self._publish_diagnostic(
                DiagnosticStatus.ERROR,
                'SHADOW_PROCESSING_ERROR',
                str(error),
            )
            return

        output = ShadowObjectTarget()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._config.output_frame
        output.x_m = float(result.position_m[0])
        output.y_m = float(result.position_m[1])
        output.z_m = float(result.position_m[2])
        output.yaw_rad = float(result.yaw_rad)
        output.source_frame_age_s = float(observation.frame_age_s)
        output.confidence = float(observation.confidence)
        output.source_footprint_inside = bool(observation.footprint_inside)
        output.source_image_fully_visible = bool(
            observation.image_fully_visible
        )
        output.shadow_pose_available = True
        output.transform_validated = bool(result.transform_validated)
        output.inside_workspace = bool(result.inside_workspace)
        output.fresh = True
        output.motion_authorized = False
        output.robot_target_available = False
        output.status = result.status
        self._publisher.publish(output)

        level = (
            DiagnosticStatus.OK
            if result.inside_workspace
            else DiagnosticStatus.WARN
        )
        self._publish_diagnostic(
            level,
            result.status,
            'computed non-actionable base-frame shadow pose',
            result=result,
            observation=observation,
        )

    def _publish_diagnostic(
        self,
        level: int,
        code: str,
        reason: str,
        result=None,
        observation=None,
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'top_perception/shadow_target'
        status.hardware_id = 'top_camera_to_left_base'
        status.message = code
        status.values = [
            self._value('reason', reason),
            self._value('output_frame', self._config.output_frame),
            self._value('transform_validated', self._config.transform_validated),
            self._value('motion_authorized', False),
            self._value('robot_target_available', False),
            self._value(
                'rejected_mixed_reference_disagreement_mm',
                f'{self._config.rejected_mixed_reference_disagreement_mm:.3f}',
            ),
        ]
        if result is not None and observation is not None:
            status.values.extend(
                [
                    self._value('x_m', f'{result.position_m[0]:.6f}'),
                    self._value('y_m', f'{result.position_m[1]:.6f}'),
                    self._value('z_m', f'{result.position_m[2]:.6f}'),
                    self._value('yaw_rad', f'{result.yaw_rad:.6f}'),
                    self._value('inside_workspace', result.inside_workspace),
                    self._value(
                        'source_frame_age_s',
                        f'{observation.frame_age_s:.6f}',
                    ),
                    self._value('confidence', f'{observation.confidence:.6f}'),
                    self._value(
                        'source_image_fully_visible',
                        observation.image_fully_visible,
                    ),
                ]
            )
        array.status = [status]
        self._diagnostics_publisher.publish(array)
        if code != self._last_code:
            message = f'TOP_SHADOW_{code} reason={reason}'
            if level == DiagnosticStatus.ERROR:
                self.get_logger().error(message)
            elif level == DiagnosticStatus.WARN:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self._last_code = code

    def _publish_stream_diagnostic(self) -> None:
        reference = (
            self._started_at
            if self._last_received_at is None
            else self._last_received_at
        )
        elapsed = time.monotonic() - reference
        if elapsed > self._stale_timeout_s:
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                'NO_BOARD_OBSERVATION',
                f'no valid board observation received for {elapsed:.3f}s',
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TopShadowTargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
