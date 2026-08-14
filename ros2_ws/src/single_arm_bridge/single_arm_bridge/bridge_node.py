"""ROS 2 readback bridge with opt-in, one-point motion commands."""

from __future__ import annotations

import hashlib
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
from so101_interfaces.srv import (
    RightArmConfigureOnce,
    RightArmConfiguration,
    RightArmJogOnce,
    RightArmTorqueEnableOnce,
)

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectory

from .action_execution import MotionExecutionCore
from .backend_lease import acquire_backend_lease
from .bimanual_feedback import (
    compose_bimanual_feedback,
    validate_bimanual_calibrations,
)
from .buffered_action_execution import BufferedActionExecutionCore
from .calibration import load_calibration
from .commanded_setpoint_state import CommandedSetpointState
from .device_discovery import resolve_serial_device
from .follow_joint_trajectory_server import FollowJointTrajectoryActionAdapter
from .hardware_identity import (
    RIGHT_ARM_J2_BASE_LIMIT_CAPABILITY,
    RIGHT_ARM_J2_BASE_LIMIT_FIRMWARE_VERSION,
    BIMANUAL_OPERATIONAL_LIMITS_CAPABILITY,
    BIMANUAL_OPERATIONAL_LIMITS_FIRMWARE_VERSIONS,
    BIMANUAL_DISPATCH_REFACTOR_FIRMWARE_VERSION,
    validate_hardware_identity,
)
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
RIGHT_ARM_JOG_CONFIRMATION = "RIGHT_ARM_JOG_ONCE"
RIGHT_ARM_TORQUE_ENABLE_CONFIRMATION = "RIGHT_ARM_TORQUE_ENABLE_ONCE"
RIGHT_ARM_CONFIGURE_CONFIRMATION = "RIGHT_ARM_CONFIGURE_ONCE"
RIGHT_ARM_JOG_MINIMUM_ABSOLUTE_DELTA_RAW = 8
RIGHT_ARM_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW = 20
RIGHT_ARM_J2_BASE_LIMITS_SHA256 = (
    "dfbfaf6c7138fab30afebc1f3e69c7d53edb01060bd349f65c6f048f150dff34"
)
RIGHT_ARM_J2_BASE_LIMITS_STATUS = (
    "J2_B_BASE_LIMIT_CANDIDATE_AWAITING_NO_MOTION_AND_ACTIVE_VALIDATION"
)
BIMANUAL_OPERATIONAL_LIMITS_SHA256 = (
    "436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263"
)
BIMANUAL_OPERATIONAL_LIMITS_STATUS = (
    "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
)


class SingleArmBridge(Node):
    def __init__(self) -> None:
        super().__init__("single_arm_bridge")
        default_calibration = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "single_arm_calibration.json"
        )
        default_right_calibration = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "right_arm_calibration.candidate.json"
        )
        default_right_j2b_limits = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "right_arm_j2b_command_limits.candidate.json"
        )
        self.declare_parameter("serial_device", DEFAULT_DEVICE)
        default_bimanual_operational_limits = str(
            Path(get_package_share_directory("single_arm_bridge"))
            / "config"
            / "bimanual_operational_limits.json"
        )
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("feedback_rate_hz", 5.0)
        self.declare_parameter("bimanual_feedback_rate_hz", 2.0)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("allow_right_arm_jog", False)
        self.declare_parameter("publish_bimanual_read_only", False)
        self.declare_parameter("left_arm_power_off_confirmed", False)
        self.declare_parameter("require_right_arm_j2_base_limits", False)
        self.declare_parameter("calibration_file", default_calibration)
        self.declare_parameter("require_bimanual_operational_limits", False)
        self.declare_parameter("right_calibration_file", default_right_calibration)
        self.declare_parameter(
            "right_arm_j2_base_limits_file", default_right_j2b_limits
        )

        self.declare_parameter(
            "bimanual_operational_limits_file", default_bimanual_operational_limits
        )
        baud_rate = self.get_parameter("baud_rate").value
        feedback_rate = self.get_parameter("feedback_rate_hz").value
        bimanual_feedback_rate = self.get_parameter(
            "bimanual_feedback_rate_hz"
        ).value
        self._allow_motion = bool(self.get_parameter("allow_motion").value)
        self._allow_right_arm_jog = bool(
            self.get_parameter("allow_right_arm_jog").value
        )
        self._publish_bimanual_read_only = bool(
            self.get_parameter("publish_bimanual_read_only").value
        )
        self._left_arm_power_off_confirmed = bool(
            self.get_parameter("left_arm_power_off_confirmed").value
        )
        self._require_right_arm_j2_base_limits = bool(
            self.get_parameter("require_right_arm_j2_base_limits").value
        )
        self._require_bimanual_operational_limits = bool(
            self.get_parameter("require_bimanual_operational_limits").value
        )
        right_j2b_limits_file = self.get_parameter(
            "right_arm_j2_base_limits_file"
        ).value
        bimanual_operational_limits_file = self.get_parameter(
            "bimanual_operational_limits_file"
        ).value
        calibration_file = self.get_parameter("calibration_file").value
        right_calibration_file = self.get_parameter(
            "right_calibration_file"
        ).value

        if self._allow_motion and self._allow_right_arm_jog:
            raise ValueError(
                "allow_motion and allow_right_arm_jog are mutually exclusive"
            )
        if self._publish_bimanual_read_only and (
            self._allow_motion or self._allow_right_arm_jog
        ):
            raise ValueError(
                "publish_bimanual_read_only is mutually exclusive with "
                "both motion modes"
            )
        if (
            self._allow_right_arm_jog
            and not self._left_arm_power_off_confirmed
        ):
            raise ValueError(
                "right-arm jog requires left_arm_power_off_confirmed:=true "
                "after physically removing left-arm 12 V power"
            )
        if (
            self._require_right_arm_j2_base_limits
            and not self._allow_right_arm_jog
        ):
            raise ValueError("J2-B limits require allow_right_arm_jog:=true")
        if self._require_bimanual_operational_limits and self._allow_motion:
            raise ValueError(
                "bimanual operational-limit candidate does not authorize "
                "the legacy left-arm trajectory backend"
            )
        if (
            self._require_right_arm_j2_base_limits
            and self._require_bimanual_operational_limits
        ):
            raise ValueError(
                "legacy J2-B and bimanual operational-limit modes "
                "are mutually exclusive"
            )

        if not 1.0 <= feedback_rate <= 10.0:
            raise ValueError("feedback_rate_hz must be within 1..10")
        if not 0.5 <= bimanual_feedback_rate <= 5.0:
            raise ValueError("bimanual_feedback_rate_hz must be within 0.5..5")

        self._faulted = False
        self._shutdown_requested = False
        self._feedback_errors = 0
        self._bimanual_feedback_errors = 0
        self._heartbeat_errors = 0
        self._feedback_resume_at = 0.0
        self._motion_armed = False
        self._right_arm_output_active = False
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
            self._right_j2b_limits_document = None
            if self._require_right_arm_j2_base_limits:
                limits_bytes = Path(right_j2b_limits_file).read_bytes()
                limits_sha256 = hashlib.sha256(limits_bytes).hexdigest()
                if limits_sha256 != RIGHT_ARM_J2_BASE_LIMITS_SHA256:
                    raise ValueError(
                        "J2-B command-limit manifest SHA mismatch: "
                        f"expected={RIGHT_ARM_J2_BASE_LIMITS_SHA256} "
                        f"actual={limits_sha256}"
                    )
                limits_document = json.loads(limits_bytes.decode("utf-8"))
                if (
                    limits_document.get("record_kind")
                    != "right_arm_j2b_command_limits_candidate"
                    or limits_document.get("status")
                    != RIGHT_ARM_J2_BASE_LIMITS_STATUS
                    or limits_document.get("motion_authorized") is not False
                    or limits_document.get("general_trajectory_authorized")
                    is not False
                ):
                    raise ValueError("J2-B command-limit manifest contract changed")
                self._right_j2b_limits_document = limits_document
            self._bimanual_operational_limits_document = None
            if self._require_bimanual_operational_limits:
                limits_bytes = Path(
                    bimanual_operational_limits_file
                ).read_bytes()
                limits_sha256 = hashlib.sha256(limits_bytes).hexdigest()
                if limits_sha256 != BIMANUAL_OPERATIONAL_LIMITS_SHA256:
                    raise ValueError(
                        "bimanual operational-limit manifest SHA mismatch: "
                        f"expected={BIMANUAL_OPERATIONAL_LIMITS_SHA256} "
                        f"actual={limits_sha256}"
                    )
                limits_document = json.loads(limits_bytes.decode("utf-8"))
                if (
                    limits_document.get("record_kind")
                    != "bimanual_operational_limits"
                    or limits_document.get("status")
                    != BIMANUAL_OPERATIONAL_LIMITS_STATUS
                    or limits_document.get("operator_approved") is not True
                    or limits_document.get("firmware_limit_authorized")
                    is not True
                ):
                    raise ValueError(
                        "bimanual operational-limit manifest contract changed"
                    )
                self._bimanual_operational_limits_document = limits_document

            ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
            self._backend_lease = acquire_backend_lease("stm32", ros_domain_id)
            serial_device = resolve_serial_device(
                str(self.get_parameter("serial_device").value)
            )
            self._calibration = load_calibration(calibration_file)
            self._right_calibration = None
            if self._publish_bimanual_read_only:
                right_document = json.loads(
                    Path(right_calibration_file).read_text(encoding="utf-8")
                )
                if (
                    right_document.get("record_kind")
                    != "right_arm_calibration_candidate"
                    or right_document.get("motion_authorized") is not False
                ):
                    raise ValueError(
                        "R4 requires the fail-closed right-arm calibration candidate"
                    )
                self._right_calibration = load_calibration(
                    right_calibration_file
                )
                validate_bimanual_calibrations(
                    self._calibration, self._right_calibration
                )

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
            if self._require_right_arm_j2_base_limits and (
                hello.firmware_version
                != RIGHT_ARM_J2_BASE_LIMIT_FIRMWARE_VERSION
                or (
                    hello.capabilities
                    & RIGHT_ARM_J2_BASE_LIMIT_CAPABILITY
                )
                == 0
            ):
                raise ValueError(
                    "J2-B mode requires firmware=0x00024200 "
                    "and capability=0x10000000"
                )
            self._hello = hello
            if (
                hello.firmware_version
                in BIMANUAL_OPERATIONAL_LIMITS_FIRMWARE_VERSIONS
                and not self._require_bimanual_operational_limits
            ):
                raise ValueError(
                    "firmware=0x00024400/0x00024500 requires "
                    "require_bimanual_operational_limits:=true"
                )
            if self._require_bimanual_operational_limits and (
                hello.firmware_version
                not in BIMANUAL_OPERATIONAL_LIMITS_FIRMWARE_VERSIONS
                or (
                    hello.capabilities
                    & BIMANUAL_OPERATIONAL_LIMITS_CAPABILITY
                )
                == 0
            ):
                raise ValueError(
                    "bimanual operational-limit mode requires "
                    "firmware=0x00024400/0x00024500 and "
                    "capability=0x20000000"
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
        self._bimanual_joint_publisher = None
        if self._publish_bimanual_read_only:
            self._bimanual_joint_publisher = self.create_publisher(
                JointState,
                "bimanual_joint_states",
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
        self._right_arm_discovery_service = self.create_service(
            Trigger,
            "discover_right_arm_read_only",
            self._on_discover_right_arm_read_only,
        )
        self._right_arm_configuration_service = self.create_service(
            RightArmConfiguration,
            "get_right_arm_configuration",
            self._on_get_right_arm_configuration,
        )
        self._right_arm_configure_service = self.create_service(
            RightArmConfigureOnce,
            "right_arm_configure_once",
            self._on_right_arm_configure_once,
        )
        self._right_arm_jog_service = self.create_service(
            RightArmJogOnce,
            "right_arm_jog_once",
            self._on_right_arm_jog_once,
        )
        self._right_arm_torque_enable_service = self.create_service(
            RightArmTorqueEnableOnce,
            "right_arm_torque_enable_once",
            self._on_right_arm_torque_enable_once,
        )
        self._right_arm_disable_service = self.create_service(
            Trigger,
            "right_arm_disable",
            self._on_right_arm_disable,
        )
        self._right_arm_stop_service = self.create_service(
            Trigger,
            "right_arm_stop",
            self._on_right_arm_stop,
        )
        self._right_arm_j2b_identity_service = self.create_service(
            Trigger,
            "right_arm_j2_base_limits_identity",
            self._on_right_arm_j2_base_limits_identity,
        )
        self._bimanual_operational_limits_identity_service = (
            self.create_service(
                Trigger,
                "bimanual_operational_limits_identity",
                self._on_bimanual_operational_limits_identity,
            )
        )
        self._heartbeat_timer = self.create_timer(0.1, self._send_heartbeat)
        self._feedback_timer = self.create_timer(
            1.0 / feedback_rate,
            self._publish_feedback,
        )
        self._bimanual_feedback_timer = self.create_timer(
            1.0 / bimanual_feedback_rate,
            self._publish_bimanual_feedback,
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
            elif self._allow_right_arm_jog:
                if hello.stop_latched:
                    raise RuntimeError(
                        "STM32 stop is latched; reset the STM32 before "
                        "starting isolated right-arm jog mode"
                    )
                # This mode is permitted only after the operator has removed
                # left-arm 12 V power. Do not issue DISABLE: it would perform
                # a six-axis left-bus write/readback and is unrelated to the
                # independent right-arm UART4 jog path.
                pass
            elif self._publish_bimanual_read_only:
                # The combined topic is allowed only after both buses prove
                # that every servo Torque Enable register reads back as zero.
                self._disable_both_arms_verified()
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
            if not self._allow_right_arm_jog:
                try:
                    # Even a failed or latched arming path must end with
                    # physical torque disabled. DISABLE performs six-axis
                    # left-bus readback and is intentionally not used by the
                    # power-off-confirmed right-arm isolation mode.
                    self._transport.disable()
                    self._motion_armed = False
                except Exception:
                    pass
            if self._publish_bimanual_read_only:
                try:
                    self._transport.disable_right_arm_verified()
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
        elif (
            self._allow_right_arm_jog
            and self._require_bimanual_operational_limits
        ):
            mode = "RIGHT_ARM_JOG_BIMANUAL_OPERATIONAL_LIMITS"
        elif (
            self._allow_right_arm_jog
            and self._require_right_arm_j2_base_limits
        ):
            mode = "RIGHT_ARM_JOG_ISOLATED_J2_BASE"
        elif self._allow_right_arm_jog:
            mode = "RIGHT_ARM_JOG_ISOLATED"
        elif (
            self._publish_bimanual_read_only
            and self._require_bimanual_operational_limits
            and hello.firmware_version
            == BIMANUAL_DISPATCH_REFACTOR_FIRMWARE_VERSION
        ):
            mode = "BIMANUAL_READ_ONLY_DISPATCH_REFACTOR"
        elif (
            self._publish_bimanual_read_only
            and self._require_bimanual_operational_limits
        ):
            mode = "BIMANUAL_READ_ONLY_OPERATIONAL_LIMITS"
        elif self._publish_bimanual_read_only:
            mode = "BIMANUAL_READ_ONLY"
        elif self._require_bimanual_operational_limits:
            mode = "READ_ONLY_BIMANUAL_OPERATIONAL_LIMITS"
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

    def _disable_both_arms_verified(self) -> None:
        """Attempt both independent torque-off gates before failing startup."""

        failures: list[str] = []
        try:
            self._transport.disable()
        except Exception as error:
            failures.append(f"left={error}")
        try:
            right = self._transport.disable_right_arm_verified()
            if right.torque_enabled_mask != 0 or right.failure_count != 0:
                failures.append(
                    "right=invalid successful response "
                    f"mask=0x{right.torque_enabled_mask:02X} "
                    f"failures={right.failure_count}"
                )
        except Exception as error:
            failures.append(f"right={error}")
        self._motion_armed = False
        self._right_arm_output_active = False
        if failures:
            raise TransportError(
                "bimanual verified disable failed: " + "; ".join(failures)
            )

    def _publish_bimanual_feedback(self) -> None:
        if (
            not self._publish_bimanual_read_only
            or self._shutdown_requested
            or self._faulted
        ):
            return
        reserved = False
        try:
            reserved = self._motion_arbiter.try_reserve(
                "bimanual_read_only_feedback"
            )
            if not reserved or self._motion_active():
                return
            left_state = self._transport.get_state(include_positions=True)
            if left_state.stop_latched:
                raise StopLatchedError("STM32 stop is latched")
            assert left_state.raw_positions is not None
            right_state = self._transport.discover_right_arm_read_only()
            assert self._right_calibration is not None
            sample = compose_bimanual_feedback(
                self._calibration,
                self._right_calibration,
                left_state.raw_positions,
                right_state,
            )
            message = JointState()
            # The two UART buses are sampled sequentially. This stamp denotes
            # publication time; R4 does not claim hardware-synchronous data.
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(sample.names)
            message.position = list(sample.positions)
            assert self._bimanual_joint_publisher is not None
            self._bimanual_joint_publisher.publish(message)
            self._bimanual_feedback_errors = 0
        except StopLatchedError as error:
            self._handle_transport_error(
                "bimanual feedback", error, immediate=True
            )
        except Exception as error:
            self._handle_transport_error("bimanual feedback", error)
        finally:
            if reserved:
                self._motion_arbiter.release("bimanual_read_only_feedback")

    def _publish_feedback(self) -> None:
        if self._shutdown_requested or self._faulted:
            return
        if self._allow_right_arm_jog:
            # R1.0.1 is intentionally isolated from the left-arm position bus.
            # The heartbeat remains active, while right-arm state is read only
            # inside the explicitly confirmed one-shot request.
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

    def _on_discover_right_arm_read_only(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        reserved = False
        try:
            reserved = self._motion_arbiter.try_reserve("right_arm_discovery")
            if not reserved or self._motion_active():
                raise RuntimeError(
                    "cannot discover right arm while a motion goal is active"
                )
            snapshot = self._transport.discover_right_arm_read_only()
            response.success = snapshot.status_code == 0
            response.message = json.dumps(
                {
                    "read_only": True,
                    "commanded_operations": ["READ present_position"],
                    "joint_count": snapshot.joint_count,
                    "present_mask": f"0x{snapshot.present_mask:02X}",
                    "positions_raw": snapshot.positions_raw,
                    "read_statuses": snapshot.read_statuses,
                    "transaction_count": snapshot.transaction_count,
                    "failure_count": snapshot.failure_count,
                },
                separators=(",", ":"),
            )
            self.get_logger().info("right-arm read-only discovery captured")
        except Exception as error:
            response.success = False
            response.message = f"right-arm discovery rejected: {error}"
            self.get_logger().error(response.message)
        finally:
            if reserved:
                self._motion_arbiter.release("right_arm_discovery")
        return response

    def _on_right_arm_configure_once(
        self,
        request: RightArmConfigureOnce.Request,
        response: RightArmConfigureOnce.Response,
    ):
        reserved = False
        try:
            if not self._allow_right_arm_jog:
                raise RuntimeError("right-arm isolation mode is not enabled")
            if request.confirmation != RIGHT_ARM_CONFIGURE_CONFIRMATION:
                raise ValueError("right-arm configure confirmation is invalid")
            if not 1 <= request.servo_id <= 6:
                raise ValueError(
                    "right-arm configure servo_id must be within 1..6"
                )
            reserved = self._motion_arbiter.try_reserve(
                "right_arm_configure_once"
            )
            if not reserved or self._motion_active():
                raise RuntimeError(
                    "cannot configure right arm while motion is active"
                )
            self._right_arm_output_active = True
            snapshot = self._transport.configure_right_arm_once(
                int(request.servo_id)
            )
            response.status_code = snapshot.status_code
            response.torque_enabled = snapshot.torque_enabled
            response.p_gain = snapshot.p_gain
            response.d_gain = snapshot.d_gain
            response.i_gain = snapshot.i_gain
            response.operating_mode = snapshot.operating_mode
            response.present_position_raw = snapshot.present_position_raw
            response.goal_position_raw = snapshot.goal_position_raw
            response.goal_speed_raw = snapshot.goal_speed_raw
            response.torque_limit_raw = snapshot.torque_limit_raw
            response.accepted = (
                snapshot.status_code == 0
                and snapshot.torque_enabled == 0
            )
            response.diagnostic = (
                f"status={snapshot.status_code} servo_id={snapshot.servo_id} "
                f"pid={snapshot.p_gain}/{snapshot.d_gain}/{snapshot.i_gain} "
                f"mode={snapshot.operating_mode} "
                f"goal_speed={snapshot.goal_speed_raw} "
                f"torque_limit={snapshot.torque_limit_raw} "
                "torque_remains_disabled=true automatic_motion=false"
            )
            if response.accepted:
                self.get_logger().info(
                    f"right-arm configuration applied torque-off "
                    f"servo_id={request.servo_id}"
                )
            else:
                self.get_logger().error(
                    f"right-arm configure-once rejected servo_id={request.servo_id}"
                )
        except Exception as error:
            response.accepted = False
            response.status_code = 255
            response.diagnostic = f"right-arm configure rejected: {error}"
            self.get_logger().error(response.diagnostic)
        finally:
            if reserved:
                self._motion_arbiter.release("right_arm_configure_once")
        return response

    def _on_get_right_arm_configuration(
        self,
        request: RightArmConfiguration.Request,
        response: RightArmConfiguration.Response,
    ):
        reserved = False
        try:
            if not 1 <= request.servo_id <= 6:
                raise ValueError(
                    "right-arm configuration servo_id must be within 1..6"
                )
            reserved = self._motion_arbiter.try_reserve(
                "right_arm_configuration"
            )
            if not reserved or self._motion_active():
                raise RuntimeError(
                    "cannot read right-arm configuration while motion is active"
                )
            snapshot = self._transport.read_right_arm_configuration(
                int(request.servo_id)
            )
            response.status_code = snapshot.status_code
            response.read_status = snapshot.read_status
            response.successful_block_mask = snapshot.successful_block_mask
            response.sample_time_ms = snapshot.sample_time_ms
            response.torque_enabled = snapshot.torque_enabled
            response.p_gain = snapshot.p_gain
            response.d_gain = snapshot.d_gain
            response.i_gain = snapshot.i_gain
            response.voltage_raw = snapshot.voltage_raw
            response.temperature_c = snapshot.temperature_c
            response.position_raw = snapshot.position_raw
            response.speed_raw = snapshot.speed_raw
            response.load_raw = snapshot.load_raw
            response.current_raw = snapshot.current_raw
            response.runtime_torque_limit_raw = (
                snapshot.runtime_torque_limit_raw
            )
            response.goal_position_raw = snapshot.goal_position_raw
            response.model_number = snapshot.model_number
            response.firmware_major_version = snapshot.firmware_major_version
            response.firmware_minor_version = snapshot.firmware_minor_version
            response.maximum_torque_limit_raw = (
                snapshot.maximum_torque_limit_raw
            )
            response.minimum_startup_force_raw = (
                snapshot.minimum_startup_force_raw
            )
            response.cw_dead_zone_raw = snapshot.cw_dead_zone_raw
            response.ccw_dead_zone_raw = snapshot.ccw_dead_zone_raw
            response.protection_current_raw = snapshot.protection_current_raw
            response.operating_mode = snapshot.operating_mode
            response.protective_torque_raw = snapshot.protective_torque_raw
            response.protection_time_raw = snapshot.protection_time_raw
            response.overload_torque_raw = snapshot.overload_torque_raw
            response.success = (
                snapshot.status_code == 0
                and snapshot.read_status == 0
                and snapshot.successful_block_mask == 0x1F
            )
            response.diagnostic = (
                f"status={snapshot.status_code} "
                f"read_status={snapshot.read_status} "
                f"successful_block_mask=0x{snapshot.successful_block_mask:02X} "
                "read_only=true reads=0:5,16:14,33:4,40:10,56:15 "
                "writes=none"
            )
            if response.success:
                self.get_logger().info(
                    f"right-arm configuration captured servo_id={request.servo_id}"
                )
            else:
                self.get_logger().error(
                    f"right-arm configuration incomplete servo_id={request.servo_id}"
                )
        except Exception as error:
            response.success = False
            response.status_code = 255
            response.diagnostic = (
                f"right-arm configuration rejected: {error}"
            )
            self.get_logger().error(response.diagnostic)
        finally:
            if reserved:
                self._motion_arbiter.release("right_arm_configuration")
        return response

    def _on_right_arm_j2_base_limits_identity(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        if (
            not self._require_right_arm_j2_base_limits
            or self._right_j2b_limits_document is None
        ):
            response.success = False
            response.message = "J2-B bridge mode is not enabled"
            return response
        response.success = True
        response.message = json.dumps(
            {
                "firmware_version": f"0x{self._hello.firmware_version:08X}",
                "capabilities": f"0x{self._hello.capabilities:08X}",
                "manifest_sha256": RIGHT_ARM_J2_BASE_LIMITS_SHA256,
                "status": self._right_j2b_limits_document["status"],
                "joints": self._right_j2b_limits_document["joints"],
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        return response

    def _on_bimanual_operational_limits_identity(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        if (
            not self._require_bimanual_operational_limits
            or self._bimanual_operational_limits_document is None
        ):
            response.success = False
            response.message = (
                "bimanual operational-limit bridge mode is not enabled"
            )
            return response
        response.success = True
        response.message = json.dumps(
            {
                "firmware_version": f"0x{self._hello.firmware_version:08X}",
                "capabilities": f"0x{self._hello.capabilities:08X}",
                "manifest_sha256": BIMANUAL_OPERATIONAL_LIMITS_SHA256,
                "status": (
                    self._bimanual_operational_limits_document["status"]
                ),
                "arms": self._bimanual_operational_limits_document["arms"],
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        return response

    def _on_right_arm_jog_once(
        self,
        request: RightArmJogOnce.Request,
        response: RightArmJogOnce.Response,
    ):
        reserved = False
        try:
            if not self._allow_right_arm_jog:
                raise RuntimeError("right-arm jog mode is not enabled")
            if request.confirmation != RIGHT_ARM_JOG_CONFIRMATION:
                raise ValueError("right-arm jog confirmation is invalid")
            if not 1 <= request.servo_id <= 6:
                raise ValueError("right-arm jog servo_id must be within 1..6")
            if not (
                RIGHT_ARM_JOG_MINIMUM_ABSOLUTE_DELTA_RAW
                <= abs(request.delta_raw)
                <= RIGHT_ARM_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW
            ):
                raise ValueError("right-arm jog delta must be within +/-8..20 raw")
            reserved = self._motion_arbiter.try_reserve("right_arm_jog")
            if not reserved or self._motion_active():
                raise RuntimeError("cannot jog right arm while a motion goal is active")
            snapshot = self._transport.jog_right_arm_once(
                int(request.servo_id), int(request.delta_raw)
            )
            response.status_code = snapshot.status_code
            response.torque_enabled = snapshot.torque_enabled
            response.start_position_raw = snapshot.start_position_raw
            response.target_position_raw = snapshot.target_position_raw
            response.observed_position_raw = snapshot.observed_position_raw
            response.accepted = snapshot.status_code in (0, 8)
            response.diagnostic = (
                f"status={snapshot.status_code} servo_id={snapshot.servo_id} "
                f"delta_raw={snapshot.delta_raw} "
                "writes=goal_position_only "
                "automatic_torque_enable=false"
            )
            if response.accepted:
                self._right_arm_output_active = True
                self.get_logger().warning(
                    "right-arm bounded jog sent; verify physical direction before "
                    "sending another command"
                )
            else:
                self.get_logger().error("right-arm bounded jog rejected")
        except Exception as error:
            response.accepted = False
            response.status_code = 255
            response.diagnostic = f"right-arm jog rejected: {error}"
            self.get_logger().error(response.diagnostic)
        finally:
            if reserved:
                self._motion_arbiter.release("right_arm_jog")
        return response

    def _on_right_arm_torque_enable_once(
        self,
        request: RightArmTorqueEnableOnce.Request,
        response: RightArmTorqueEnableOnce.Response,
    ):
        reserved = False
        try:
            if not self._allow_right_arm_jog:
                raise RuntimeError("right-arm jog mode is not enabled")
            if request.confirmation != RIGHT_ARM_TORQUE_ENABLE_CONFIRMATION:
                raise ValueError("right-arm torque-enable confirmation is invalid")
            if not 1 <= request.servo_id <= 6:
                raise ValueError(
                    "right-arm torque-enable servo_id must be within 1..6"
                )
            reserved = self._motion_arbiter.try_reserve("right_arm_torque_enable")
            if not reserved or self._motion_active():
                raise RuntimeError(
                    "cannot enable right-arm torque while a motion goal is active"
                )
            snapshot = self._transport.enable_right_arm_torque_once(
                int(request.servo_id)
            )
            response.status_code = snapshot.status_code
            response.torque_enabled = snapshot.torque_enabled
            response.present_position_raw = snapshot.present_position_raw
            response.held_goal_position_raw = snapshot.held_goal_position_raw
            response.observed_position_raw = snapshot.observed_position_raw
            response.accepted = snapshot.status_code in (0, 11)
            response.diagnostic = (
                f"status={snapshot.status_code} servo_id={snapshot.servo_id} "
                "writes=goal_position_at_present_then_torque_enable "
                "pid_speed_limits_unchanged=true"
            )
            if response.accepted:
                self._right_arm_output_active = True
                self.get_logger().warning(
                    "right-arm single-servo torque enabled at present position; "
                    "keep the arm supported and issue only the reviewed jog"
                )
            else:
                self.get_logger().error("right-arm torque-enable rejected")
        except Exception as error:
            response.accepted = False
            response.status_code = 255
            response.diagnostic = f"right-arm torque-enable rejected: {error}"
            self.get_logger().error(response.diagnostic)
        finally:
            if reserved:
                self._motion_arbiter.release("right_arm_torque_enable")
        return response

    def _on_right_arm_disable(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        try:
            if not self._allow_right_arm_jog:
                raise RuntimeError("right-arm isolation mode is not enabled")
            snapshot = self._transport.disable_right_arm_verified()
            if (
                snapshot.torque_enabled_mask != 0
                or snapshot.failure_count != 0
            ):
                raise RuntimeError(
                    "invalid verified disable response: "
                    f"mask=0x{snapshot.torque_enabled_mask:02X} "
                    f"failures={snapshot.failure_count}"
                )
            self._right_arm_output_active = False
            response.success = True
            response.message = (
                "right-arm verified disable complete; all right-bus "
                "torque read back as zero"
            )
            self.get_logger().info(response.message)
        except Exception as error:
            # Preserve output_active on failure so shutdown still attempts the
            # independent latched SAFE_STOP path.
            response.success = False
            response.message = f"right-arm verified disable failed: {error}"
            self.get_logger().error(response.message)
        return response

    def _on_right_arm_stop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        try:
            if not self._allow_right_arm_jog:
                raise RuntimeError("right-arm isolation mode is not enabled")
            # SAFE_STOP is deliberately not gated by the motion arbiter. A
            # stop must wait for the current serial transaction and then
            # disable all right-bus torque without waiting for motion cleanup.
            self._transport.safe_stop()
            self._right_arm_output_active = False
            self._faulted = True
            self._motion_armed = False
            self._commanded_setpoints.reset()
            response.success = True
            response.message = (
                "right-arm stop latched; all right-bus torque disabled"
            )
            self.get_logger().warning(response.message)
        except Exception as error:
            # Keep output_active set on failure so heartbeat timeout and node
            # shutdown retain independent torque-off retry paths.
            response.success = False
            response.message = f"right-arm stop failed: {error}"
            self.get_logger().error(response.message)
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
        elif stage == "bimanual feedback":
            self._bimanual_feedback_errors += 1
            error_count = self._bimanual_feedback_errors
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
            if self._allow_right_arm_jog:
                if self._right_arm_output_active:
                    try:
                        # SAFE_STOP touches the right bus only when R1 has
                        # sent a jog. It never invokes left-bus readback.
                        self._transport.safe_stop()
                    except Exception as error:
                        message = f"right-arm SAFE_STOP during shutdown failed: {error}"
                        if rclpy.ok():
                            self.get_logger().error(message)
                        else:
                            print(message, file=sys.stderr)
            elif self._publish_bimanual_read_only:
                disable_error: Exception | None = None
                for _ in range(3):
                    try:
                        self._serial.reset_input_buffer()
                        self._disable_both_arms_verified()
                        disable_error = None
                        break
                    except Exception as error:
                        disable_error = error
                if disable_error is not None:
                    message = (
                        "bimanual verified DISABLE during shutdown failed: "
                        f"{disable_error}"
                    )
                    if rclpy.ok():
                        self.get_logger().error(message)
                    else:
                        print(message, file=sys.stderr)
            else:
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
        for timer_name in (
            "_heartbeat_timer",
            "_feedback_timer",
            "_bimanual_feedback_timer",
        ):
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
