"""ROS 2 facade for the source-agnostic resident bimanual stream adapter."""

from __future__ import annotations

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from so101_interfaces.msg import BimanualJointFeedback
from so101_interfaces.srv import BimanualStreamCommand

from std_srvs.srv import Trigger

from .backend_lease import acquire_backend_lease
from .bimanual_stream_adapter import (
    AdapterState,
    BimanualTransientFeedbackError,
    CANONICAL_JOINT_NAMES,
    F8_FIRMWARE_VERSION,
    ResidentBimanualStreamAdapter,
    TimedJointPoint,
    load_operational_limits,
    normalize_joint_positions,
)
from .device_discovery import resolve_serial_device
from .serial_port import open_exclusive_serial
from .stream_transport_v2 import StreamValidationTransportV2


HOST_BAUD = 921_600


class BimanualStreamNode(Node):
    """Own the serial port and serialize all 12-axis command sources."""

    def __init__(self) -> None:
        super().__init__("bimanual_stream_adapter")
        default_limits = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "bimanual_operational_limits.json"
        )
        self.declare_parameter("serial_device", "auto")
        self.declare_parameter("motion_authorized", False)
        self.declare_parameter("operational_limits_file", default_limits)
        self.declare_parameter("poll_period_s", 0.2)
        self.declare_parameter("feedback_period_s", 0.05)
        self.declare_parameter("heartbeat_period_s", 0.1)
        self.declare_parameter("response_timeout_s", 0.12)

        self._lease = None
        self._serial = None
        self._adapter = None
        poll_period_s = float(self.get_parameter("poll_period_s").value)
        if not 0.05 <= poll_period_s <= 1.0:
            raise ValueError("poll_period_s must be within 0.05..1.0")
        feedback_period_s = float(
            self.get_parameter("feedback_period_s").value
        )
        if not 0.02 <= feedback_period_s <= 1.0:
            raise ValueError("feedback_period_s must be within 0.02..1.0")
        heartbeat_period_s = float(
            self.get_parameter("heartbeat_period_s").value
        )
        if not 0.02 <= heartbeat_period_s <= 0.2:
            raise ValueError("heartbeat_period_s must be within 0.02..0.2")
        try:
            ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
            self._lease = acquire_backend_lease("stm32", ros_domain_id)
            limits = load_operational_limits(
                Path(str(self.get_parameter("operational_limits_file").value))
            )
            device = resolve_serial_device(
                str(self.get_parameter("serial_device").value)
            )
            import serial

            timeout_s = float(self.get_parameter("response_timeout_s").value)
            if not 0.05 <= timeout_s <= 0.2:
                raise ValueError("response_timeout_s must be within 0.05..0.2")
            self._serial = open_exclusive_serial(
                serial,
                device,
                HOST_BAUD,
                timeout_s=timeout_s,
            )
            transport = StreamValidationTransportV2(
                self._serial,
                response_timeout_s=timeout_s,
            )
            self._adapter = ResidentBimanualStreamAdapter(
                transport,
                limits,
                motion_authorized=bool(
                    self.get_parameter("motion_authorized").value
                ),
            )
            prepared = self._adapter.prepare()
        except Exception:
            self._release_resources()
            raise

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._anchor_publisher = self.create_publisher(
            JointState,
            "~/anchor_joint_states",
            qos,
        )
        self._publish_anchor(prepared)

        feedback_qos = QoSProfile(depth=1)
        feedback_qos.reliability = ReliabilityPolicy.RELIABLE
        self._joint_state_publisher = self.create_publisher(
            JointState,
            "~/joint_states",
            feedback_qos,
        )
        # Preserve the namespaced upper-application topic and also expose the
        # standard ROS joint-state feed consumed by MoveIt and robot_state_publisher.
        self._moveit_joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            feedback_qos,
        )
        self._feedback_publisher = self.create_publisher(
            BimanualJointFeedback,
            "~/feedback",
            feedback_qos,
        )

        self._command_service = self.create_service(
            BimanualStreamCommand,
            "~/command",
            self._on_command,
        )
        self._status_service = self.create_service(
            Trigger,
            "~/status",
            self._on_status,
        )
        self._refresh_anchor_service = self.create_service(
            Trigger,
            "~/refresh_anchor",
            self._on_refresh_anchor,
        )
        self._poll_timer = self.create_timer(poll_period_s, self._poll_active)
        self._heartbeat_timer = self.create_timer(
            heartbeat_period_s,
            self._send_heartbeat,
        )
        self._feedback_timer = self.create_timer(
            feedback_period_s,
            self._publish_feedback,
        )
        self._publish_feedback()
        self.get_logger().info(
            "resident bimanual stream ready "
            f"firmware=0x{F8_FIRMWARE_VERSION:08X} motion_authorized="
            f"{str(bool(self.get_parameter('motion_authorized').value)).lower()}"
        )

    def _publish_anchor(self, prepared) -> None:
        anchor = JointState()
        anchor.header.stamp = self.get_clock().now().to_msg()
        anchor.name = list(CANONICAL_JOINT_NAMES)
        anchor.position = [
            value / 1_000_000.0 for value in prepared.positions_urad
        ]
        self._anchor_publisher.publish(anchor)

    def _release_resources(self) -> None:
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        lease = self._lease
        self._lease = None
        if lease is not None:
            lease.release()

    @staticmethod
    def _offset_ms(point) -> int:
        seconds = int(point.time_from_start.sec)
        nanoseconds = int(point.time_from_start.nanosec)
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("time_from_start is invalid")
        total_nanoseconds = seconds * 1_000_000_000 + nanoseconds
        if total_nanoseconds % 1_000_000:
            raise ValueError("time_from_start must use whole milliseconds")
        return total_nanoseconds // 1_000_000

    @classmethod
    def _points_from_request(cls, request) -> tuple[TimedJointPoint, ...]:
        if not request.points:
            raise ValueError("command requires at least one trajectory point")
        result = []
        for point in request.points:
            if point.velocities or point.accelerations or point.effort:
                raise ValueError(
                    "resident position stream does not accept velocity, "
                    "acceleration, or effort fields"
                )
            positions = normalize_joint_positions(
                request.joint_names,
                point.positions,
            )
            result.append(
                TimedJointPoint(
                    offset_ms=cls._offset_ms(point),
                    positions_rad=positions,
                )
            )
        return tuple(result)

    def _on_command(self, request, response):
        adapter = self._adapter
        if adapter is None:
            response.accepted = False
            response.adapter_state = "unavailable"
            response.arbiter_epoch = 0
            response.diagnostic = "adapter is unavailable"
            return response
        try:
            operation = request.operation
            request_type = BimanualStreamCommand.Request
            if (
                operation != request_type.SPLICE
                and request.splice_offset_ms != 0
            ):
                raise ValueError(
                    "splice_offset_ms must be zero unless operation is SPLICE"
                )
            if operation == request_type.STOP:
                adapter.stop(request.owner)
            else:
                points = self._points_from_request(request)
                if operation == request_type.START_FINITE:
                    adapter.start(request.owner, points, finite=True)
                elif operation == request_type.START_OPEN:
                    adapter.start(request.owner, points, finite=False)
                elif operation == request_type.APPEND:
                    adapter.append(request.owner, points)
                elif operation == request_type.SPLICE:
                    adapter.splice(
                        request.owner,
                        points,
                        splice_offset_ms=int(request.splice_offset_ms),
                    )
                else:
                    raise ValueError(f"unsupported operation={operation}")
            response.accepted = True
            response.diagnostic = "accepted"
        except Exception as error:
            response.accepted = False
            response.diagnostic = f"{type(error).__name__}: {error}"
            self.get_logger().error(response.diagnostic)
        response.adapter_state = adapter.state.value
        response.arbiter_epoch = adapter.epoch
        return response

    def _on_status(self, _request, response):
        adapter = self._adapter
        if adapter is None:
            response.success = False
            response.message = '{"state":"unavailable"}'
            return response
        try:
            adapter.keepalive()
        except Exception as error:
            self.get_logger().error(f"resident keepalive stopped: {error}")
        response.success = adapter.state is not AdapterState.FAULTED
        response.message = json.dumps(
            {
                "state": adapter.state.value,
                "owner": adapter.owner,
                "arbiter_epoch": adapter.epoch,
                "motion_authorized": bool(
                    self.get_parameter("motion_authorized").value
                ),
                "firmware_version": f"0x{F8_FIRMWARE_VERSION:08X}",
                "fault_diagnostic": adapter.fault_diagnostic,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        return response

    def _on_refresh_anchor(self, _request, response):
        adapter = self._adapter
        if adapter is None:
            response.success = False
            response.message = '{"state":"unavailable"}'
            return response
        try:
            prepared = adapter.refresh_unarmed_anchor()
            self._publish_anchor(prepared)
            response.success = True
            response.message = json.dumps(
                {
                    "state": adapter.state.value,
                    "firmware_version": f"0x{F8_FIRMWARE_VERSION:08X}",
                    "joint_count": len(prepared.positions_urad),
                    "torque_enabled": False,
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            self.get_logger().info(
                "resident torque-off anchor refreshed immediately before motion"
            )
        except Exception as error:
            response.success = False
            response.message = f"{type(error).__name__}: {error}"
            self.get_logger().error(
                f"resident anchor refresh failed: {error}"
            )
        return response

    def _send_heartbeat(self) -> None:
        adapter = self._adapter
        if adapter is None or not adapter.heartbeat_required:
            return
        try:
            adapter.keepalive()
        except Exception as error:
            self.get_logger().error(f"resident keepalive stopped: {error}")

    def _publish_feedback(self) -> None:
        adapter = self._adapter
        if adapter is None or adapter.state in (
            AdapterState.NEW,
            AdapterState.FAULTED,
            AdapterState.STOPPED,
        ):
            return
        try:
            snapshot = adapter.feedback_snapshot()
        except BimanualTransientFeedbackError as error:
            self.get_logger().warning(str(error))
            return
        except Exception as error:
            self.get_logger().error(f"resident feedback stopped: {error}")
            return

        stamp = self.get_clock().now().to_msg()
        positions = [value / 1_000_000.0 for value in snapshot.positions_urad]

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(CANONICAL_JOINT_NAMES)
        joint_state.position = positions
        self._joint_state_publisher.publish(joint_state)
        self._moveit_joint_state_publisher.publish(joint_state)

        feedback = BimanualJointFeedback()
        feedback.header.stamp = stamp
        feedback.joint_names = list(CANONICAL_JOINT_NAMES)
        feedback.positions = positions
        feedback.sample_age_ms = list(snapshot.sample_age_ms)
        feedback.present_mask = snapshot.present_mask
        feedback.firmware_tick_ms = snapshot.firmware_tick_ms
        feedback.completed_pairs = snapshot.completed_pairs
        self._feedback_publisher.publish(feedback)

    def _poll_active(self) -> None:
        adapter = self._adapter
        if (
            adapter is None
            or adapter.state is not AdapterState.ACTIVE
            or adapter.owner is None
        ):
            return
        try:
            previous_state = adapter.state
            adapter.poll(adapter.owner)
            if (
                previous_state is AdapterState.ACTIVE
                and adapter.state is AdapterState.READY
            ):
                prepared = adapter.prepared_state
                if prepared is None:
                    raise RuntimeError(
                        "terminal resident anchor is unavailable"
                    )
                self._publish_anchor(prepared)
        except Exception as error:
            self.get_logger().error(f"resident stream stopped: {error}")

    def destroy_node(self):
        adapter = self._adapter
        if (
            adapter is not None
            and adapter.owner is not None
            and adapter.state not in (
                AdapterState.FAULTED,
                AdapterState.STOPPED,
            )
        ):
            try:
                adapter.stop(adapter.owner)
            except Exception as error:
                self.get_logger().error(f"shutdown stop failed: {error}")
        self._release_resources()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BimanualStreamNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
