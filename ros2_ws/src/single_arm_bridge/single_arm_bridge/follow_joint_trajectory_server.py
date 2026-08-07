"""ROS 2 FollowJointTrajectory adapter for the STM32 execution core."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from .action_execution import (
    ExecutionError,
    ExecutionOutcome,
    MotionExecutionCore,
    TerminalState,
    format_execution_outcome,
)
from .action_validation import (
    GoalValidationError,
    TrajectoryPointData,
    ValidatedBufferedTrajectory,
    validate_buffered_trajectory,
    validate_single_point_trajectory,
)
from .buffered_action_execution import BufferedActionExecutionCore
from .calibration import ArmCalibration
from .commanded_setpoint_state import CommandedSetpointState
from .motion_goal_arbiter import MotionGoalArbiter

DEFAULT_ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
RECOVERY_DURATION_MS = 2000
RECOVERY_FEEDBACK_OVERRUN_RAW = 20
RECOVERY_TARGET_MARGIN_RAW = 20
RECOVERY_MAX_JOINT_DELTA_RAW = 64


@dataclass(frozen=True, slots=True)
class PreparedArmGoal:
    arm_positions: tuple[float, ...]
    full_positions: tuple[float, ...]
    actual_arm_positions: tuple[float, ...]
    duration_ms: int
    buffered_trajectory: ValidatedBufferedTrajectory | None = None


def _duration_ns(duration: Any) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _position_to_raw(joint: Any, position: float) -> int:
    return round(
        joint.zero_raw
        + joint.direction * position * 4096.0 / (2.0 * math.pi)
    )


def _validate_bounded_inward_recovery(
    calibration: ArmCalibration,
    actual: tuple[float, ...],
    target_arm: tuple[float, ...],
    outside_strict: dict[str, int],
    duration_ms: int,
) -> None:
    if duration_ms != RECOVERY_DURATION_MS:
        raise GoalValidationError(
            "bounded inward recovery duration must be exactly 2000 ms"
        )

    joint_names = tuple(calibration.ros_joint_names)
    arm_names = joint_names[:5]
    if any(
        name not in arm_names and overrun > RECOVERY_FEEDBACK_OVERRUN_RAW
        for name, overrun in outside_strict.items()
    ):
        raise GoalValidationError(
            "arm recovery cannot repair out-of-range gripper feedback"
        )

    actual_arm_raw = tuple(
        _position_to_raw(joint, position)
        for joint, position in zip(
            calibration.joints[:5], actual[:5], strict=True
        )
    )
    target_arm_raw = tuple(
        _position_to_raw(joint, position)
        for joint, position in zip(
            calibration.joints[:5], target_arm, strict=True
        )
    )

    for name, joint, actual_raw, target_raw in zip(
        arm_names,
        calibration.joints[:5],
        actual_arm_raw,
        target_arm_raw,
        strict=True,
    ):
        if abs(target_raw - actual_raw) > RECOVERY_MAX_JOINT_DELTA_RAW:
            raise GoalValidationError(
                f"bounded recovery {name} displacement exceeds "
                f"{RECOVERY_MAX_JOINT_DELTA_RAW} raw"
            )
        inner_minimum = joint.minimum_raw + RECOVERY_TARGET_MARGIN_RAW
        inner_maximum = joint.maximum_raw - RECOVERY_TARGET_MARGIN_RAW
        if not inner_minimum <= target_raw <= inner_maximum:
            raise GoalValidationError(
                f"bounded recovery {name} target raw {target_raw} must be "
                f"at least {RECOVERY_TARGET_MARGIN_RAW} raw inside strict "
                f"range {joint.minimum_raw}..{joint.maximum_raw}"
            )
        if name not in outside_strict:
            continue
        if outside_strict[name] <= RECOVERY_FEEDBACK_OVERRUN_RAW:
            continue
        if actual_raw < joint.minimum_raw and target_raw <= actual_raw:
            raise GoalValidationError(
                f"bounded recovery {name} target does not move inward"
            )
        if actual_raw > joint.maximum_raw and target_raw >= actual_raw:
            raise GoalValidationError(
                f"bounded recovery {name} target does not move inward"
            )


def prepare_follow_joint_trajectory_goal(
    request: Any,
    calibration: ArmCalibration,
    latest_positions: Sequence[float] | None,
    commanded_positions: Sequence[float] | None = None,
) -> PreparedArmGoal:
    """Validate a ROS goal and preserve the commanded gripper target."""
    multi_dof = request.multi_dof_trajectory
    if multi_dof.joint_names or multi_dof.points:
        raise GoalValidationError("multi-DOF trajectory is not supported")
    if (
        request.trajectory.header.stamp.sec
        or request.trajectory.header.stamp.nanosec
    ):
        raise GoalValidationError("scheduled trajectory start is not supported")
    if (
        request.path_tolerance
        or request.component_path_tolerance
        or request.goal_tolerance
        or request.component_goal_tolerance
        or _duration_ns(request.goal_time_tolerance) != 0
    ):
        raise GoalValidationError("custom trajectory tolerances are not supported")

    points = tuple(
        TrajectoryPointData(
            positions=tuple(point.positions),
            time_from_start_ns=_duration_ns(point.time_from_start),
            velocities=tuple(point.velocities),
            accelerations=tuple(point.accelerations),
            effort=tuple(point.effort),
        )
        for point in request.trajectory.points
    )
    all_joint_names = tuple(calibration.ros_joint_names)
    arm_joint_names = all_joint_names[:5]
    arm_limits = {
        name: calibration.ros_radian_limits[name]
        for name in arm_joint_names
    }
    if latest_positions is None:
        raise GoalValidationError("fresh joint feedback is required")
    actual = tuple(float(value) for value in latest_positions)
    if len(actual) != len(all_joint_names):
        raise GoalValidationError("joint feedback count is invalid")
    if not all(math.isfinite(value) for value in actual):
        raise GoalValidationError("joint feedback contains a non-finite value")
    try:
        outside_strict = calibration.validate_feedback_recovery_envelope(actual)
    except ValueError as error:
        raise GoalValidationError(
            f"joint feedback is outside recovery range: {error}"
        ) from error
    needs_bounded_recovery = any(
        overrun > RECOVERY_FEEDBACK_OVERRUN_RAW
        for overrun in outside_strict.values()
    )

    preserved = actual
    if commanded_positions is not None:
        preserved = tuple(float(value) for value in commanded_positions)
        if len(preserved) != len(all_joint_names):
            raise GoalValidationError("commanded setpoint count is invalid")
        if not all(math.isfinite(value) for value in preserved):
            raise GoalValidationError(
                "commanded setpoint contains a non-finite value"
            )

    buffered_trajectory = None
    if len(points) >= 2:
        if needs_bounded_recovery:
            raise GoalValidationError(
                "buffered trajectory cannot start from a bounded recovery pose"
            )
        velocity_limits = {name: 0.5 for name in arm_joint_names}
        acceleration_limits = {name: 1.0 for name in arm_joint_names}
        start_tolerances = {
            name: (0.055 if name.endswith("shoulder_joint") else 0.050)
            for name in arm_joint_names
        }
        buffered_trajectory = validate_buffered_trajectory(
            request.trajectory.joint_names,
            points,
            arm_joint_names,
            arm_limits,
            actual[:5],
            velocity_limits,
            acceleration_limits,
            start_tolerance_rad=start_tolerances,
        )
        ordered_positions = buffered_trajectory.ordered_points[-1]
        duration_ms = buffered_trajectory.duration_ms
    else:
        validated = validate_single_point_trajectory(
            request.trajectory.joint_names,
            points,
            arm_joint_names,
            arm_limits,
        )
        if needs_bounded_recovery:
            _validate_bounded_inward_recovery(
                calibration,
                actual,
                validated.ordered_positions,
                outside_strict,
                validated.duration_ms,
            )
        ordered_positions = validated.ordered_positions
        duration_ms = validated.duration_ms

    full_positions = ordered_positions + (preserved[5],)
    try:
        calibration.radians_to_urad(list(full_positions))
    except ValueError as error:
        raise GoalValidationError(
            f"combined arm and gripper target is unsafe: {error}"
        ) from error
    return PreparedArmGoal(
        arm_positions=ordered_positions,
        full_positions=full_positions,
        actual_arm_positions=actual[:5],
        duration_ms=duration_ms,
        buffered_trajectory=buffered_trajectory,
    )


class FollowJointTrajectoryActionAdapter:
    """Expose one safe, non-preempting legacy/buffered arm ActionServer."""

    def __init__(
        self,
        node: Any,
        execution_core: MotionExecutionCore,
        calibration: ArmCalibration,
        motion_ready: Callable[[], bool],
        latest_positions: Callable[[], Sequence[float] | None],
        motion_arbiter: MotionGoalArbiter | None = None,
        setpoint_state: CommandedSetpointState | None = None,
        buffered_execution_core: BufferedActionExecutionCore | None = None,
        action_name: str = DEFAULT_ACTION_NAME,
        poll_interval_s: float = 0.02,
        completion_timeout_s: float = 3.5,
    ) -> None:
        if poll_interval_s <= 0.0 or completion_timeout_s <= 0.0:
            raise ValueError("Action timing values must be positive")
        self._node = node
        self._execution_core = execution_core
        self._buffered_execution_core = buffered_execution_core
        self._calibration = calibration
        self._motion_ready = motion_ready
        self._latest_positions = latest_positions
        self._motion_arbiter = motion_arbiter or MotionGoalArbiter()
        self._setpoint_state = setpoint_state or CommandedSetpointState()
        self._poll_interval_s = poll_interval_s
        self._completion_timeout_s = completion_timeout_s
        self._state_lock = threading.RLock()
        self._goal_reserved = False
        self._prepared_goal: PreparedArmGoal | None = None
        self._external_outcome: ExecutionOutcome | None = None
        self._server = ActionServer(
            node,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _goal_callback(self, request: Any) -> GoalResponse:
        with self._state_lock:
            if self._goal_reserved:
                self._node.get_logger().warning(
                    "arm goal rejected: another goal is reserved"
                )
                return GoalResponse.REJECT
            buffered_blocked = (
                self._buffered_execution_core is not None
                and self._buffered_execution_core.blocked
            )
            if (
                not self._motion_ready()
                or self._execution_core.blocked
                or buffered_blocked
            ):
                self._node.get_logger().warning(
                    "arm goal rejected: motion backend is not ready"
                )
                return GoalResponse.REJECT
            if not self._motion_arbiter.try_reserve("arm"):
                self._node.get_logger().warning(
                    "arm goal rejected: another motion owner is active"
                )
                return GoalResponse.REJECT
            try:
                latest = self._latest_positions()
                commanded = self._setpoint_state.snapshot()
                prepared = prepare_follow_joint_trajectory_goal(
                    request,
                    self._calibration,
                    latest,
                    commanded,
                )
            except (GoalValidationError, ValueError) as error:
                self._motion_arbiter.release("arm")
                self._node.get_logger().warning(f"arm goal rejected: {error}")
                return GoalResponse.REJECT
            self._prepared_goal = prepared
            self._goal_reserved = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, unused_goal_handle: Any) -> CancelResponse:
        del unused_goal_handle
        with self._state_lock:
            return (
                CancelResponse.ACCEPT
                if self._goal_reserved
                else CancelResponse.REJECT
            )

    def _execute_callback(self, goal_handle: Any):
        with self._state_lock:
            prepared = self._prepared_goal
        if prepared is None:
            with self._state_lock:
                self._goal_reserved = False
                self._motion_arbiter.release("arm")
            return self._abort_without_execution(
                goal_handle,
                "accepted goal preparation is missing",
            )

        try:
            active_core = self._execution_core
            if prepared.buffered_trajectory is not None:
                if self._buffered_execution_core is None:
                    raise ExecutionError(
                        "buffered Action execution core is unavailable"
                    )
                active_core = self._buffered_execution_core
                active_core.start_goal(
                    prepared.buffered_trajectory,
                    preserved_gripper_rad=prepared.full_positions[5],
                )
            else:
                active_core.start_goal(
                    prepared.full_positions,
                    prepared.duration_ms,
                )
            self._publish_initial_feedback(goal_handle, prepared)
            deadline = (
                time.monotonic()
                + prepared.duration_ms / 1000.0
                + self._completion_timeout_s
            )
            while True:
                outcome = self._take_external_outcome()
                if outcome is None and goal_handle.is_cancel_requested:
                    try:
                        outcome = active_core.cancel_active_goal()
                    except ExecutionError as error:
                        outcome = ExecutionOutcome(
                            TerminalState.ABORTED,
                            0,
                            None,
                            None,
                            f"cancel failed: {error}",
                        )
                if outcome is None:
                    outcome = active_core.poll()
                if outcome is not None:
                    return self._finish_goal(goal_handle, outcome)
                if time.monotonic() >= deadline:
                    outcome = active_core.handle_connection_loss(
                        "motion result timeout"
                    )
                    if outcome is None:
                        return self._abort_without_execution(
                            goal_handle,
                            "motion result timeout without an active goal",
                        )
                    return self._finish_goal(goal_handle, outcome)
                time.sleep(self._poll_interval_s)
        except ExecutionError as error:
            return self._abort_without_execution(goal_handle, str(error))
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._prepared_goal = None
                self._external_outcome = None
                self._motion_arbiter.release("arm")

    def _publish_initial_feedback(
        self,
        goal_handle: Any,
        prepared: PreparedArmGoal,
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self._node.get_clock().now().to_msg()
        feedback.joint_names = self._calibration.ros_joint_names[:5]
        feedback.desired.positions = list(prepared.arm_positions)
        feedback.actual.positions = list(prepared.actual_arm_positions)
        feedback.error.positions = [
            desired - actual
            for desired, actual in zip(
                prepared.arm_positions,
                prepared.actual_arm_positions,
                strict=True,
            )
        ]
        goal_handle.publish_feedback(feedback)

    def _take_external_outcome(self) -> ExecutionOutcome | None:
        with self._state_lock:
            outcome = self._external_outcome
            self._external_outcome = None
            return outcome

    @staticmethod
    def _result(error_code: int, reason: str):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = reason
        return result

    def _finish_goal(self, goal_handle: Any, outcome: ExecutionOutcome):
        terminal_log = format_execution_outcome(outcome)
        if outcome.state is TerminalState.SUCCEEDED:
            with self._state_lock:
                prepared = self._prepared_goal
            if prepared is None:
                self._setpoint_state.reset()
                return self._abort_without_execution(
                    goal_handle,
                    "completed goal preparation is missing",
                )
            self._setpoint_state.commit(prepared.full_positions)
            self._node.get_logger().info(terminal_log)
            goal_handle.succeed()
            return self._result(
                FollowJointTrajectory.Result.SUCCESSFUL,
                outcome.reason,
            )
        if outcome.state is TerminalState.CANCELED:
            self._setpoint_state.reset()
            self._node.get_logger().warning(terminal_log)
            goal_handle.canceled()
            return self._result(
                FollowJointTrajectory.Result.SUCCESSFUL,
                outcome.reason,
            )
        self._setpoint_state.reset()
        self._node.get_logger().error(terminal_log)
        goal_handle.abort()
        return self._result(
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            outcome.reason,
        )

    def _abort_without_execution(self, goal_handle: Any, reason: str):
        self._setpoint_state.reset()
        self._node.get_logger().error(
            format_execution_outcome(
                ExecutionOutcome(
                    TerminalState.ABORTED,
                    0,
                    None,
                    None,
                    reason,
                )
            )
        )
        goal_handle.abort()
        return self._result(FollowJointTrajectory.Result.INVALID_GOAL, reason)

    def notify_connection_loss(self, reason: str) -> None:
        self._setpoint_state.reset()
        if self._motion_arbiter.owner != "arm":
            return
        core = self._execution_core
        if (
            self._buffered_execution_core is not None
            and self._buffered_execution_core.active
        ):
            core = self._buffered_execution_core
        outcome = core.handle_connection_loss(reason)
        if outcome is None:
            return
        with self._state_lock:
            self._external_outcome = outcome

    def destroy(self) -> None:
        self._setpoint_state.reset()
        self._motion_arbiter.release("arm")
        self._server.destroy()
