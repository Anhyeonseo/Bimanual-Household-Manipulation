"""ROS 2 ParallelGripperCommand adapter for the STM32 execution core."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from control_msgs.action import ParallelGripperCommand

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from .action_execution import (
    ExecutionError,
    ExecutionOutcome,
    MotionExecutionCore,
    TerminalState,
)
from .action_validation import (
    GoalValidationError,
    GripperCommandData,
    validate_gripper_command,
)
from .calibration import ArmCalibration
from .commanded_setpoint_state import CommandedSetpointState
from .motion_goal_arbiter import MotionGoalArbiter


DEFAULT_ACTION_NAME = "/left_gripper_controller/gripper_cmd"
DEFAULT_DURATION_MS = 1000


@dataclass(frozen=True, slots=True)
class PreparedGripperGoal:
    target_position: float
    full_positions: tuple[float, ...]
    actual_position: float
    duration_ms: int


def prepare_parallel_gripper_goal(
    request: Any,
    calibration: ArmCalibration,
    latest_positions: Sequence[float] | None,
    duration_ms: int = DEFAULT_DURATION_MS,
    commanded_positions: Sequence[float] | None = None,
) -> PreparedGripperGoal:
    """Validate a gripper goal while preserving commanded arm targets."""
    if not 300 <= duration_ms <= 2000:
        raise GoalValidationError("gripper duration must be within 300..2000 ms")
    if request.command.header.stamp.sec or request.command.header.stamp.nanosec:
        raise GoalValidationError("scheduled gripper command is not supported")

    all_joint_names = tuple(calibration.ros_joint_names)
    gripper_name = all_joint_names[5]
    target = validate_gripper_command(
        GripperCommandData(
            positions=tuple(request.command.position),
            joint_names=tuple(request.command.name),
            velocities=tuple(request.command.velocity),
            efforts=tuple(request.command.effort),
        ),
        gripper_name,
        calibration.ros_radian_limits[gripper_name],
    )

    if latest_positions is None:
        raise GoalValidationError("fresh joint feedback is required")
    actual = tuple(float(value) for value in latest_positions)
    if len(actual) != len(all_joint_names):
        raise GoalValidationError("joint feedback count is invalid")
    if not all(math.isfinite(value) for value in actual):
        raise GoalValidationError("joint feedback contains a non-finite value")
    try:
        calibration.radians_to_urad(list(actual))
    except ValueError as error:
        raise GoalValidationError(
            f"joint feedback is outside safe range: {error}"
        ) from error

    preserved = actual
    if commanded_positions is not None:
        preserved = tuple(float(value) for value in commanded_positions)
        if len(preserved) != len(all_joint_names):
            raise GoalValidationError("commanded setpoint count is invalid")
        if not all(math.isfinite(value) for value in preserved):
            raise GoalValidationError(
                "commanded setpoint contains a non-finite value"
            )

    full_positions = preserved[:5] + (target,)
    try:
        calibration.radians_to_urad(list(full_positions))
    except ValueError as error:
        raise GoalValidationError(
            f"combined arm and gripper target is unsafe: {error}"
        ) from error
    return PreparedGripperGoal(
        target_position=target,
        full_positions=full_positions,
        actual_position=actual[5],
        duration_ms=duration_ms,
    )


class ParallelGripperCommandActionAdapter:
    """Expose one hardware-safe, non-preempting gripper ActionServer."""

    def __init__(
        self,
        node: Any,
        execution_core: MotionExecutionCore,
        calibration: ArmCalibration,
        motion_ready: Callable[[], bool],
        latest_positions: Callable[[], Sequence[float] | None],
        motion_arbiter: MotionGoalArbiter | None = None,
        setpoint_state: CommandedSetpointState | None = None,
        action_name: str = DEFAULT_ACTION_NAME,
        duration_ms: int = DEFAULT_DURATION_MS,
        poll_interval_s: float = 0.02,
        completion_timeout_s: float = 3.5,
    ) -> None:
        if not 300 <= duration_ms <= 2000:
            raise ValueError("gripper duration must be within 300..2000 ms")
        if poll_interval_s <= 0.0 or completion_timeout_s <= 0.0:
            raise ValueError("Action timing values must be positive")
        self._node = node
        self._execution_core = execution_core
        self._calibration = calibration
        self._motion_ready = motion_ready
        self._latest_positions = latest_positions
        self._motion_arbiter = motion_arbiter or MotionGoalArbiter()
        self._setpoint_state = setpoint_state or CommandedSetpointState()
        self._duration_ms = duration_ms
        self._poll_interval_s = poll_interval_s
        self._completion_timeout_s = completion_timeout_s
        self._state_lock = threading.RLock()
        self._goal_reserved = False
        self._prepared_goal: PreparedGripperGoal | None = None
        self._external_outcome: ExecutionOutcome | None = None
        self._server = ActionServer(
            node,
            ParallelGripperCommand,
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
                    "gripper goal rejected: another goal is reserved"
                )
                return GoalResponse.REJECT
            if not self._motion_ready() or self._execution_core.blocked:
                self._node.get_logger().warning(
                    "gripper goal rejected: motion backend is not ready"
                )
                return GoalResponse.REJECT
            if not self._motion_arbiter.try_reserve("gripper"):
                self._node.get_logger().warning(
                    "gripper goal rejected: another motion owner is active"
                )
                return GoalResponse.REJECT
            try:
                latest = self._latest_positions()
                commanded = self._setpoint_state.snapshot()
                prepared = prepare_parallel_gripper_goal(
                    request,
                    self._calibration,
                    latest,
                    self._duration_ms,
                    commanded,
                )
            except (GoalValidationError, ValueError) as error:
                self._motion_arbiter.release("gripper")
                self._node.get_logger().warning(f"gripper goal rejected: {error}")
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
                self._motion_arbiter.release("gripper")
            return self._abort_without_execution(
                goal_handle,
                "accepted gripper goal preparation is missing",
            )

        try:
            self._execution_core.start_goal(
                prepared.full_positions,
                prepared.duration_ms,
            )
            self._publish_feedback(goal_handle, prepared.actual_position)
            deadline = (
                time.monotonic()
                + prepared.duration_ms / 1000.0
                + self._completion_timeout_s
            )
            while True:
                outcome = self._take_external_outcome()
                if outcome is None and goal_handle.is_cancel_requested:
                    try:
                        outcome = self._execution_core.cancel_active_goal()
                    except ExecutionError as error:
                        outcome = ExecutionOutcome(
                            TerminalState.ABORTED,
                            0,
                            None,
                            None,
                            f"cancel failed: {error}",
                        )
                if outcome is None:
                    outcome = self._execution_core.poll()
                if outcome is not None:
                    return self._finish_goal(goal_handle, prepared, outcome)
                if time.monotonic() >= deadline:
                    outcome = self._execution_core.handle_connection_loss(
                        "gripper motion result timeout"
                    )
                    if outcome is None:
                        return self._abort_without_execution(
                            goal_handle,
                            "gripper timeout without an active goal",
                            prepared.actual_position,
                        )
                    return self._finish_goal(goal_handle, prepared, outcome)
                time.sleep(self._poll_interval_s)
        except ExecutionError as error:
            return self._abort_without_execution(
                goal_handle,
                str(error),
                prepared.actual_position,
            )
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._prepared_goal = None
                self._external_outcome = None
                self._motion_arbiter.release("gripper")

    def _publish_feedback(self, goal_handle: Any, position: float) -> None:
        feedback = ParallelGripperCommand.Feedback()
        self._fill_state(feedback.state, position)
        goal_handle.publish_feedback(feedback)

    def _fill_state(self, state: Any, fallback_position: float) -> None:
        position = fallback_position
        latest = self._latest_positions()
        if latest is not None and len(latest) == 6:
            candidate = float(latest[5])
            if math.isfinite(candidate):
                position = candidate
        state.header.stamp = self._node.get_clock().now().to_msg()
        state.name = [self._calibration.ros_joint_names[5]]
        state.position = [position]

    def _take_external_outcome(self) -> ExecutionOutcome | None:
        with self._state_lock:
            outcome = self._external_outcome
            self._external_outcome = None
            return outcome

    def _result(
        self,
        position: float,
        reached_goal: bool,
        stalled: bool = False,
    ):
        result = ParallelGripperCommand.Result()
        self._fill_state(result.state, position)
        result.stalled = stalled
        result.reached_goal = reached_goal
        return result

    def _finish_goal(
        self,
        goal_handle: Any,
        prepared: PreparedGripperGoal,
        outcome: ExecutionOutcome,
    ):
        if outcome.state is TerminalState.SUCCEEDED:
            self._setpoint_state.commit(prepared.full_positions)
            goal_handle.succeed()
            return self._result(prepared.target_position, reached_goal=True)
        if outcome.state is TerminalState.CANCELED:
            self._setpoint_state.reset()
            goal_handle.canceled()
            return self._result(prepared.actual_position, reached_goal=False)
        self._setpoint_state.reset()
        goal_handle.abort()
        return self._result(prepared.actual_position, reached_goal=False)

    def _abort_without_execution(
        self,
        goal_handle: Any,
        reason: str,
        position: float = 0.0,
    ):
        self._setpoint_state.reset()
        self._node.get_logger().error(f"gripper goal aborted: {reason}")
        goal_handle.abort()
        return self._result(position, reached_goal=False)

    def notify_connection_loss(self, reason: str) -> None:
        self._setpoint_state.reset()
        if self._motion_arbiter.owner != "gripper":
            return
        outcome = self._execution_core.handle_connection_loss(reason)
        if outcome is None:
            return
        with self._state_lock:
            self._external_outcome = outcome

    def destroy(self) -> None:
        self._setpoint_state.reset()
        self._motion_arbiter.release("gripper")
        self._server.destroy()
