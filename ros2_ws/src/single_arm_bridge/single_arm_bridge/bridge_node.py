"""ROS 2 readback bridge with opt-in, one-point motion commands."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectory

from .action_execution import MotionExecutionCore
from .backend_lease import acquire_backend_lease
from .buffered_action_execution import BufferedActionExecutionCore
from .calibration import load_calibration
from .commanded_setpoint_state import CommandedSetpointState
from .device_discovery import resolve_serial_device
from .follow_joint_trajectory_server import FollowJointTrajectoryActionAdapter
from .hardware_identity import validate_hardware_identity
from .motion_goal_arbiter import MotionGoalArbiter
from .parallel_gripper_command_server import (
    ParallelGripperCommandActionAdapter,
)
from .serial_port import open_exclusive_serial
from .transport import (
    ActuatorTransport,
    PositionReadError,
    StateResponseDeferred,
    StopLatchedError,
    TransportError,
)


DEFAULT_DEVICE = "auto"


class SingleArmBridge(Node):
    def __init__(self) -> None:
        super().__init__("single_arm_bridge")
        default_calibration = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "single_arm_calibration.json"
        )
        self.declare_parameter("serial_device", DEFAULT_DEVICE)
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("feedback_rate_hz", 5.0)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("calibration_file", default_calibration)

        baud_rate = self.get_parameter("baud_rate").value
        feedback_rate = self.get_parameter("feedback_rate_hz").value
        self._allow_motion = bool(self.get_parameter("allow_motion").value)
        calibration_file = self.get_parameter("calibration_file").value

        if not 1.0 <= feedback_rate <= 10.0:
            raise ValueError("feedback_rate_hz must be within 1..10")

        self._faulted = False
        self._shutdown_requested = False
        self._feedback_errors = 0
        self._heartbeat_errors = 0
        self._feedback_resume_at = 0.0
        self._motion_armed = False
        self._serial = None
        self._backend_lease = None
        self._arm_action_adapter = None
        self._gripper_action_adapter = None
        self._motion_arbiter = MotionGoalArbiter()
        self._commanded_setpoints = CommandedSetpointState()
        self._latest_positions: tuple[float, ...] | None = None
        self._latest_feedback_at = 0.0
        self._feedback_max_age_s = max(0.5, 2.5 / feedback_rate)

        try:
            ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
            self._backend_lease = acquire_backend_lease("stm32", ros_domain_id)
            serial_device = resolve_serial_device(
                str(self.get_parameter("serial_device").value)
            )
            self._calibration = load_calibration(calibration_file)

            import serial

            self._serial = open_exclusive_serial(
                serial,
                serial_device,
                baud_rate,
                timeout_s=0.12,
            )
            self._transport = ActuatorTransport(
                self._serial,
                response_timeout_s=0.12,
            )
            hello = self._transport.enter_binary_mode()
            validate_hardware_identity(
                hello,
                self._calibration.calibration_hash,
            )
            self._execution_core = MotionExecutionCore(
                self._transport,
                hello,
                self._calibration,
            )
            self._buffered_execution_core = BufferedActionExecutionCore(
                self._transport,
                hello,
                self._calibration,
            )
        except Exception as error:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
            if self._backend_lease is not None:
                self._backend_lease.release()
            raise RuntimeError(f"STM32 connection failed: {error}") from error

        self._joint_publisher = self.create_publisher(
            JointState,
            "joint_states",
            10,
        )
        self._command_subscription = self.create_subscription(
            JointTrajectory,
            "joint_command",
            self._on_joint_command,
            10,
        )
        self._clear_fault_service = self.create_service(
            Trigger,
            "clear_fault",
            self._on_clear_fault,
        )
        self._servo_diagnostics_service = self.create_service(
            Trigger,
            "get_servo_diagnostics",
            self._on_get_servo_diagnostics,
        )
        self._heartbeat_timer = self.create_timer(0.1, self._send_heartbeat)
        self._feedback_timer = self.create_timer(
            1.0 / feedback_rate,
            self._publish_feedback,
        )
        arm_attempted = False
        try:
            if self._allow_motion:
                self._arm_action_adapter = FollowJointTrajectoryActionAdapter(
                    self,
                    self._execution_core,
                    self._calibration,
                    self._motion_backend_ready,
                    self._fresh_joint_positions,
                    motion_arbiter=self._motion_arbiter,
                    setpoint_state=self._commanded_setpoints,
                    buffered_execution_core=self._buffered_execution_core,
                )
                self._gripper_action_adapter = (
                    ParallelGripperCommandActionAdapter(
                        self,
                        self._execution_core,
                        self._calibration,
                        self._motion_backend_ready,
                        self._fresh_joint_positions,
                        motion_arbiter=self._motion_arbiter,
                        setpoint_state=self._commanded_setpoints,
                    )
                )
                if hello.stop_latched:
                    # A latched controller may still have physical servo torque
                    # enabled. Force the firmware's six-axis write/readback
                    # DISABLE transaction before exposing the blocked backend.
                    self._transport.disable()
                    self.get_logger().warning(
                        "STM32 stop is latched; physical torque disabled; "
                        "inspect the arm and call /clear_fault"
                    )
                else:
                    arm_attempted = True
                    self._transport.arm_and_enable(
                        self._calibration.calibration_hash
                    )
                    self._motion_armed = True
            else:
                # READ_ONLY is a physical contract, not only a ROS command
                # filter. Firmware acknowledges this call only after all six
                # Torque Enable registers read back as zero.
                self._transport.disable()
        except Exception as error:
            if arm_attempted:
                try:
                    self._transport.safe_stop()
                except Exception:
                    pass
            try:
                # Even a failed or latched arming path must end with physical
                # torque disabled. DISABLE itself performs six-axis readback.
                self._transport.disable()
                self._motion_armed = False
            except Exception:
                pass
            if self._arm_action_adapter is not None:
                self._arm_action_adapter.destroy()
                self._arm_action_adapter = None
            if self._gripper_action_adapter is not None:
                self._gripper_action_adapter.destroy()
                self._gripper_action_adapter = None
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
            if self._backend_lease is not None:
                self._backend_lease.release()
            raise RuntimeError(
                f"STM32 motion initialization failed: {error}"
            ) from error

        if self._motion_armed:
            mode = "MOTION_ENABLED"
        elif self._allow_motion:
            mode = "MOTION_BLOCKED_LATCHED"
        else:
            mode = "READ_ONLY"
        self.get_logger().info(
            f"connected firmware=0x{hello.firmware_version:08X} "
            f"calibration=0x{hello.calibration_hash:08X} mode={mode}"
        )

    def _send_heartbeat(self) -> None:
        if self._shutdown_requested or self._faulted:
            return
        try:
            self._transport.heartbeat()
            self._heartbeat_errors = 0
        except StopLatchedError as error:
            self._handle_transport_error("heartbeat", error, immediate=True)
        except Exception as error:
            self._handle_transport_error("heartbeat", error)

    def _publish_feedback(self) -> None:
        if self._shutdown_requested or self._faulted:
            return
        if time.monotonic() < self._feedback_resume_at:
            return
        if self._motion_active():
            # Firmware trajectory execution owns the servo bus. The Action
            # polling thread collects its unsolicited terminal status; resume
            # physical position reads on the first regular cycle after it ends.
            self._feedback_errors = 0
            return
        try:
            state = self._transport.get_state(include_positions=True)
            if state.stop_latched:
                self._handle_transport_error(
                    "safety latch",
                    TransportError("STM32 stop is latched"),
                    immediate=True,
                )
                return
            assert state.raw_positions is not None
            positions = tuple(
                self._calibration.raw_feedback_to_radians(state.raw_positions)
            )
            self._latest_positions = positions
            self._latest_feedback_at = time.monotonic()
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self._calibration.ros_joint_names
            message.position = list(positions)
            self._joint_publisher.publish(message)
            if (
                self._arm_action_adapter is None
                and self._gripper_action_adapter is None
            ):
                self._process_motion_results()
            self._feedback_errors = 0
        except StateResponseDeferred:
            # A terminal motion result is valid serial traffic. The MCU can omit
            # the overlapping position response while final verification ends;
            # the next regular 5 Hz cycle will obtain fresh joint state.
            self._feedback_errors = 0
        except PositionReadError as error:
            self._handle_transport_error(
                "feedback",
                error,
                immediate=error.stop_latched,
            )
        except StopLatchedError as error:
            self._handle_transport_error("feedback", error, immediate=True)
        except Exception as error:
            self._handle_transport_error("feedback", error)

    def _process_motion_results(self) -> None:
        for result in self._transport.drain_motion_results():
            if result.status_code == 6:
                self.get_logger().info(
                    "motion completed "
                    f"max_error_raw={result.detail} sequence={result.request_sequence}"
                )
                continue
            self._handle_transport_error(
                "motion result",
                TransportError(
                    f"status={result.status_code} detail={result.detail} "
                    f"sequence={result.request_sequence}"
                ),
                immediate=True,
            )

    def _on_joint_command(self, message: JointTrajectory) -> None:
        if (
            self._arm_action_adapter is not None
            or self._gripper_action_adapter is not None
        ):
            self.get_logger().error(
                "joint command rejected: standard Actions own motion"
            )
            return
        if not self._allow_motion or self._faulted or not self._motion_armed:
            self.get_logger().error("joint command rejected: motion is not enabled")
            return
        if len(message.points) != 1:
            self.get_logger().error("joint command requires exactly one point")
            return
        expected_names = self._calibration.ros_joint_names
        if set(message.joint_names) != set(expected_names):
            self.get_logger().error("joint command must contain all six known joints")
            return

        point = message.points[0]
        if len(point.positions) != 6:
            self.get_logger().error("joint command must contain six positions")
            return
        duration_ms = (
            point.time_from_start.sec * 1000
            + point.time_from_start.nanosec // 1_000_000
        )
        indexed = dict(zip(message.joint_names, point.positions, strict=True))
        ordered_positions = [indexed[name] for name in expected_names]

        try:
            positions_urad = self._calibration.radians_to_urad(ordered_positions)
            self._transport.send_setpoint(positions_urad, duration_ms)
            self._feedback_resume_at = (
                time.monotonic() + (duration_ms / 1000.0) + 0.3
            )
            self.get_logger().info(f"joint command accepted duration={duration_ms}ms")
        except Exception as error:
            self._handle_transport_error("joint command", error, immediate=True)

    def _on_get_servo_diagnostics(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        reserved = False
        try:
            reserved = self._motion_arbiter.try_reserve("diagnostics")
            if not reserved or self._motion_active():
                raise RuntimeError(
                    "cannot read diagnostics while a motion goal is active"
                )
            snapshot = self._transport.get_diagnostics()
            joints = []
            for name, sample in zip(
                self._calibration.ros_joint_names,
                snapshot.joints,
                strict=True,
            ):
                signed_current = (
                    sample.current_raw - 0x10000
                    if sample.current_raw >= 0x8000
                    else sample.current_raw
                )
                joints.append(
                    {
                        "name": name,
                        "servo_id": sample.servo_id,
                        "sample_time_ms": sample.sample_time_ms,
                        "torque_enabled": sample.torque_enabled,
                        "position_raw": sample.position_raw,
                        "speed_raw": sample.speed_raw,
                        "load_raw": sample.load_raw,
                        "load_magnitude_raw": sample.load_raw & 0x03FF,
                        "current_raw": sample.current_raw,
                        "current_signed_raw": signed_current,
                        "voltage_raw": sample.voltage_raw,
                        "voltage_v": sample.voltage_raw / 10.0,
                        "temperature_c": sample.temperature_c,
                        "p_gain": sample.p_gain,
                        "d_gain": sample.d_gain,
                        "i_gain": sample.i_gain,
                        "torque_limit_raw": sample.torque_limit_raw,
                        "goal_position_raw": sample.goal_position_raw,
                        "model_number": sample.model_number,
                        "servo_firmware_version": (
                            f"{sample.firmware_major_version}."
                            f"{sample.firmware_minor_version}"
                        ),
                        "maximum_torque_limit_raw": (
                            sample.maximum_torque_limit_raw
                        ),
                        "minimum_startup_force_raw": (
                            sample.minimum_startup_force_raw
                        ),
                        "cw_dead_zone_raw": sample.cw_dead_zone_raw,
                        "ccw_dead_zone_raw": sample.ccw_dead_zone_raw,
                        "protection_current_raw": (
                            sample.protection_current_raw
                        ),
                        "operating_mode": sample.operating_mode,
                        "protective_torque_raw": (
                            sample.protective_torque_raw
                        ),
                        "protection_time_raw": sample.protection_time_raw,
                        "overload_torque_raw": sample.overload_torque_raw,
                    }
                )
            response.success = True
            response.message = json.dumps(
                {
                    "protocol_version": snapshot.protocol_version,
                    "joint_count": snapshot.joint_count,
                    "calibration_hash": (
                        f"0x{snapshot.calibration_hash:08X}"
                    ),
                    "joints": joints,
                },
                separators=(",", ":"),
            )
            self.get_logger().info("servo diagnostics snapshot captured")
        except Exception as error:
            response.success = False
            response.message = f"servo diagnostics rejected: {error}"
            self.get_logger().error(response.message)
        finally:
            if reserved:
                self._motion_arbiter.release("diagnostics")
        return response

    def _on_clear_fault(self, request: Trigger.Request, response: Trigger.Response):
        del request
        try:
            if self._motion_active() or self._motion_arbiter.owner is not None:
                raise RuntimeError("cannot clear fault while an Action goal is active")
            self._transport.clear_fault()
            self._commanded_setpoints.reset()
            self._latest_positions = None
            self._latest_feedback_at = 0.0
            hello = self._transport.enter_binary_mode()
            validate_hardware_identity(
                hello,
                self._calibration.calibration_hash,
            )
            if self._allow_motion:
                self._transport.arm_and_enable(self._calibration.calibration_hash)
                self._motion_armed = True
            self._execution_core.replace_transport_after_explicit_recovery(
                self._transport,
                hello,
            )
            self._buffered_execution_core.replace_transport_after_explicit_recovery(
                self._transport,
                hello,
            )
            self._faulted = False
            self._feedback_errors = 0
            self._heartbeat_errors = 0
            response.success = True
            response.message = (
                "fault cleared; commands enabled"
                if self._allow_motion
                else "fault cleared; read-only mode"
            )
            self.get_logger().info(response.message)
        except Exception as error:
            self._motion_armed = False
            self._faulted = True
            try:
                self._transport.safe_stop()
            except Exception:
                pass
            response.success = False
            response.message = f"fault clear rejected: {error}"
            self.get_logger().error(response.message)
        return response

    def _handle_transport_error(
        self,
        stage: str,
        error: Exception,
        immediate: bool = False,
    ) -> None:
        # SIGINT can invalidate the ROS context while a MultiThreadedExecutor
        # callback is returning from serial I/O.  Do not convert that lifecycle
        # race into a transport fault or attempt to publish through rosout.
        # destroy_node() still performs the independent physical DISABLE gate.
        if self._shutdown_requested or not rclpy.ok(context=self.context):
            return
        if immediate:
            error_count = 3
        elif stage == "heartbeat":
            self._heartbeat_errors += 1
            error_count = self._heartbeat_errors
        elif stage == "feedback":
            self._feedback_errors += 1
            error_count = self._feedback_errors
        else:
            error_count = 3
        if error_count < 3:
            self.get_logger().warning(
                f"transient {stage} delay "
                f"({error_count}/3): {error}"
            )
            return
        self.get_logger().error(f"{stage} error: {error}")
        self._faulted = True
        self._motion_armed = False
        self._commanded_setpoints.reset()
        if self._arm_action_adapter is not None:
            self._arm_action_adapter.notify_connection_loss(
                f"{stage}: {error}"
            )
        if self._gripper_action_adapter is not None:
            self._gripper_action_adapter.notify_connection_loss(
                f"{stage}: {error}"
            )
        try:
            self._transport.safe_stop()
        except Exception as stop_error:
            self.get_logger().error(f"SAFE_STOP acknowledgement failed: {stop_error}")

    def destroy_node(self) -> bool:
        self.prepare_shutdown()
        if self._arm_action_adapter is not None:
            self._arm_action_adapter.destroy()
            self._arm_action_adapter = None
        if self._gripper_action_adapter is not None:
            self._gripper_action_adapter.destroy()
            self._gripper_action_adapter = None
        if (
            hasattr(self, "_transport")
            and self._serial is not None
            and self._serial.is_open
        ):
            disable_error: Exception | None = None
            for _ in range(3):
                try:
                    # Ctrl+C may interrupt a feedback read after consuming only
                    # part of a frame. Drop that fragment and retry DISABLE
                    # directly: a latched heartbeat must never prevent physical
                    # torque removal.
                    self._serial.reset_input_buffer()
                    self._transport.disable()
                    self._motion_armed = False
                    disable_error = None
                    break
                except Exception as error:
                    disable_error = error
            if disable_error is not None:
                message = f"DISABLE during shutdown failed: {disable_error}"
                if rclpy.ok():
                    self.get_logger().error(message)
                else:
                    print(message, file=sys.stderr)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        if self._backend_lease is not None:
            self._backend_lease.release()
        return super().destroy_node()

    def _motion_backend_ready(self) -> bool:
        return (
            self._allow_motion
            and self._motion_armed
            and not self._faulted
            and not self._execution_core.blocked
            and not self._buffered_execution_core.blocked
        )

    def _motion_active(self) -> bool:
        return (
            self._execution_core.active
            or self._buffered_execution_core.active
        )

    def _fresh_joint_positions(self) -> tuple[float, ...] | None:
        positions = self._latest_positions
        if positions is None:
            return None
        if time.monotonic() - self._latest_feedback_at > self._feedback_max_age_s:
            return None
        return positions

    def prepare_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        # Quiesce timer callbacks before the executor/context teardown.  The
        # flag is set first so an already-running callback also becomes silent.
        self._shutdown_requested = True
        self._commanded_setpoints.reset()
        for timer_name in ("_heartbeat_timer", "_feedback_timer"):
            timer = getattr(self, timer_name, None)
            if timer is not None:
                timer.cancel()
        if self._arm_action_adapter is not None:
            self._arm_action_adapter.notify_connection_loss("node shutdown")
        if self._gripper_action_adapter is not None:
            self._gripper_action_adapter.notify_connection_loss("node shutdown")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SingleArmBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.prepare_shutdown()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
