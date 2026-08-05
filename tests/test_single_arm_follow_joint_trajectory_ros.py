import sys
import threading
import time
import unittest
import uuid
from pathlib import Path


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectoryPoint

    from single_arm_bridge.action_execution import MotionExecutionCore
    from single_arm_bridge.action_execution import ExecutionOutcome, TerminalState
    from single_arm_bridge.calibration import load_calibration
    from single_arm_bridge.commanded_setpoint_state import CommandedSetpointState
    from single_arm_bridge.follow_joint_trajectory_server import (
        FollowJointTrajectoryActionAdapter,
        RECOVERY_FEEDBACK_OVERRUN_RAW,
        RECOVERY_TARGET_MARGIN_RAW,
    )
    from single_arm_bridge.motion_goal_arbiter import MotionGoalArbiter
    from single_arm_bridge.protocol import Hello, MotionResult

    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


if ROS_AVAILABLE:

    class FakeRosActionTransport:
        def __init__(self) -> None:
            self.next_sequence = 500
            self.send_calls = []
            self.results = []
            self.safe_stop_calls = 0
            self.auto_status = None
            self.auto_detail = 0

        def send_setpoint(self, positions_urad, duration_ms):
            sequence = self.next_sequence
            self.next_sequence += 1
            self.send_calls.append((tuple(positions_urad), duration_ms))
            if self.auto_status is not None:
                self.results.append(
                    MotionResult(
                        self.auto_status,
                        1,
                        3,
                        self.auto_detail,
                        sequence,
                        1200,
                        0x8AD27897,
                    )
                )
            return MotionResult(
                0,
                1,
                3,
                0,
                sequence,
                1200,
                0x8AD27897,
            )

        def drain_motion_results(self):
            results = list(self.results)
            self.results.clear()
            return results

        def safe_stop(self):
            self.safe_stop_calls += 1


    class FakeBufferedExecutionCore:
        def __init__(self) -> None:
            self.blocked = False
            self.active = False
            self.start_calls = []

        def start_goal(self, trajectory, *, preserved_gripper_rad):
            self.start_calls.append((trajectory, preserved_gripper_rad))
            self.active = True

        def poll(self):
            self.active = False
            return ExecutionOutcome(
                TerminalState.SUCCEEDED,
                900,
                6,
                2,
                "buffered integration test completed",
            )

        def cancel_active_goal(self):
            self.active = False
            self.blocked = True
            return ExecutionOutcome(
                TerminalState.CANCELED,
                900,
                8,
                None,
                "buffered integration test canceled",
            )

        def handle_connection_loss(self, reason):
            self.active = False
            self.blocked = True
            return ExecutionOutcome(
                TerminalState.ABORTED,
                900,
                None,
                None,
                str(reason),
            )


@unittest.skipUnless(ROS_AVAILABLE, "ROS Jazzy environment is not sourced")
class FollowJointTrajectoryRosIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.calibration = load_calibration(CALIBRATION_PATH)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        suffix = uuid.uuid4().hex
        self.action_name = f"/test_left_arm_controller_{suffix}/follow"
        self.server_node = Node(f"test_server_{suffix}")
        self.client_node = Node(f"test_client_{suffix}")
        self.transport = FakeRosActionTransport()
        hello = Hello(
            1,
            6,
            False,
            0x00022900,
            0x8AD27897,
            0x00000FFF,
            0,
        )
        self.core = MotionExecutionCore(
            self.transport,
            hello,
            self.calibration,
        )
        self.ready = True
        self.positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        self.motion_arbiter = MotionGoalArbiter()
        self.setpoint_state = CommandedSetpointState()
        self.buffered_core = FakeBufferedExecutionCore()
        self.adapter = FollowJointTrajectoryActionAdapter(
            self.server_node,
            self.core,
            self.calibration,
            lambda: self.ready,
            lambda: self.positions,
            motion_arbiter=self.motion_arbiter,
            setpoint_state=self.setpoint_state,
            buffered_execution_core=self.buffered_core,
            action_name=self.action_name,
            poll_interval_s=0.005,
            completion_timeout_s=0.2,
        )
        self.client = ActionClient(
            self.client_node,
            FollowJointTrajectory,
            self.action_name,
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server_node)
        self.executor.add_node(self.client_node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=2.0))

    def tearDown(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.client.destroy()
        self.adapter.destroy()
        self.client_node.destroy_node()
        self.server_node.destroy_node()

    def wait_future(self, future, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(future.done(), "ROS Action future timed out")
        result = future.result()
        if result is None and future.exception() is not None:
            raise future.exception()
        return result

    def goal(self, positions=None, names=None, duration_ms=300):
        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = list(
            names or self.calibration.ros_joint_names[:5]
        )
        point = JointTrajectoryPoint()
        point.positions = list(positions or [0.0] * 5)
        point.time_from_start.sec = duration_ms // 1000
        point.time_from_start.nanosec = (duration_ms % 1000) * 1_000_000
        request.trajectory.points = [point]
        return request

    def buffered_goal(self):
        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = list(
            self.calibration.ros_joint_names[:5]
        )
        start = JointTrajectoryPoint()
        start.positions = [0.0] * 5
        middle = JointTrajectoryPoint()
        middle.positions = [0.02] * 5
        middle.time_from_start.nanosec = 200_000_000
        final = JointTrajectoryPoint()
        final.positions = [0.04] * 5
        final.time_from_start.nanosec = 400_000_000
        request.trajectory.points = [start, middle, final]
        return request

    def send_goal(self, goal, feedback_callback=None):
        return self.wait_future(
            self.client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            )
        )

    def test_success_result_feedback_and_gripper_preservation(self) -> None:
        self.transport.auto_status = 6
        self.transport.auto_detail = 20
        feedback = []
        goal_handle = self.send_goal(
            self.goal(),
            feedback_callback=lambda message: feedback.append(message.feedback),
        )
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            response.result.error_code,
            FollowJointTrajectory.Result.SUCCESSFUL,
        )
        self.assertEqual(len(self.transport.send_calls), 1)
        positions_urad, duration_ms = self.transport.send_calls[0]
        self.assertEqual(positions_urad[:5], (0, 0, 0, 0, 0))
        self.assertEqual(positions_urad[5], 100_000)
        self.assertEqual(duration_ms, 300)
        self.assertTrue(feedback)
        self.assertEqual(feedback[0].joint_names, self.calibration.ros_joint_names[:5])

    def test_arm_goal_preserves_commanded_gripper_not_contact_feedback(self) -> None:
        self.positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.098)
        self.setpoint_state.commit(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.13)
        )
        self.transport.auto_status = 6
        self.transport.auto_detail = 20

        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)
        response = self.wait_future(goal_handle.get_result_async())

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.transport.send_calls[0][0][5], 130_000)
        self.assertAlmostEqual(self.setpoint_state.snapshot()[5], 0.13)

    def test_multi_point_goal_routes_once_to_buffered_core(self) -> None:
        self.setpoint_state.commit((0.0, 0.0, 0.0, 0.0, 0.0, 0.13))
        goal_handle = self.send_goal(self.buffered_goal())
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.transport.send_calls, [])
        self.assertEqual(len(self.buffered_core.start_calls), 1)
        trajectory, gripper = self.buffered_core.start_calls[0]
        self.assertEqual(trajectory.duration_ms, 400)
        self.assertEqual(trajectory.ordered_points[-1], (0.04,) * 5)
        self.assertAlmostEqual(gripper, 0.13)
        self.assertEqual(
            self.setpoint_state.snapshot(),
            (0.04, 0.04, 0.04, 0.04, 0.04, 0.13),
        )

    def test_moveit_nanosecond_duration_is_rounded_to_next_sample(self) -> None:
        request = self.buffered_goal()
        request.trajectory.points[-1].time_from_start.nanosec = 401_000_000

        goal_handle = self.send_goal(request)

        self.assertTrue(goal_handle.accepted)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            self.buffered_core.start_calls[0][0].duration_ms,
            420,
        )
        self.assertEqual(self.transport.send_calls, [])

    def test_recovery_thresholds_remain_20_raw(self) -> None:
        self.assertEqual(RECOVERY_FEEDBACK_OVERRUN_RAW, 20)
        self.assertEqual(RECOVERY_TARGET_MARGIN_RAW, 20)

    def test_invalid_unready_and_feedbackless_goals_are_rejected(self) -> None:
        invalid = self.goal(names=["unknown_joint"] * 5)
        self.assertFalse(self.send_goal(invalid).accepted)

        self.ready = False
        self.assertFalse(self.send_goal(self.goal()).accepted)

        self.ready = True
        self.positions = None
        self.assertFalse(self.send_goal(self.goal()).accepted)

        self.positions = (-0.1, 0.0, 0.0, 0.0, 0.0, 0.1)
        self.assertFalse(self.send_goal(self.goal()).accepted)

        self.positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        scheduled = self.goal()
        scheduled.trajectory.header.stamp.sec = 1
        self.assertFalse(self.send_goal(scheduled).accepted)

        custom_tolerance = self.goal()
        tolerance = JointTolerance()
        tolerance.name = self.calibration.ros_joint_names[0]
        tolerance.position = 0.01
        custom_tolerance.path_tolerance = [tolerance]
        self.assertFalse(self.send_goal(custom_tolerance).accepted)
        self.assertEqual(self.transport.send_calls, [])

    def test_boundary_feedback_only_allows_bounded_inward_recovery(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2070, 2043, 2041, 2071, 2080, 1965)
            )
        )

        self.assertFalse(
            self.send_goal(
                self.goal(positions=[0.01] * 5, duration_ms=2000)
            ).accepted
        )
        self.assertFalse(
            self.send_goal(
                self.goal(positions=[0.05] * 5, duration_ms=1000)
            ).accepted
        )
        self.assertFalse(
            self.send_goal(
                self.goal(positions=[0.10] * 5, duration_ms=2000)
            ).accepted
        )
        self.assertEqual(self.transport.send_calls, [])

        self.transport.auto_status = 6
        self.transport.auto_detail = 20
        goal_handle = self.send_goal(
            self.goal(positions=[0.05] * 5, duration_ms=2000)
        )
        self.assertTrue(goal_handle.accepted)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(self.transport.send_calls), 1)
        positions_urad, duration_ms = self.transport.send_calls[0]
        self.assertEqual(positions_urad[:5], (50_000,) * 5)
        self.assertEqual(
            positions_urad[5],
            round(self.positions[5] * 1_000_000),
        )
        self.assertEqual(duration_ms, 2000)

    def test_current_elbow_residual_accepts_verified_inward_pose(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2279, 2051, 2069, 2048, 2054, 1959)
            )
        )
        self.transport.auto_status = 6
        self.transport.auto_detail = 20

        goal_handle = self.send_goal(
            self.goal(
                positions=[0.45, 0.10, 0.05, 0.05, 0.05],
                duration_ms=2000,
            )
        )
        self.assertTrue(goal_handle.accepted)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(self.transport.send_calls), 1)
        positions_urad, duration_ms = self.transport.send_calls[0]
        self.assertEqual(
            positions_urad[:5],
            (450_000, 100_000, 50_000, 50_000, 50_000),
        )
        self.assertEqual(positions_urad[5], 136_524)
        self.assertEqual(duration_ms, 2000)

    def test_bounded_arm_recovery_rejects_out_of_range_gripper(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2070, 2051, 2069, 2048, 2054, 2069)
            )
        )
        self.assertFalse(
            self.send_goal(
                self.goal(positions=[0.05] * 5, duration_ms=2000)
            ).accepted
        )
        self.assertEqual(self.transport.send_calls, [])

    def test_final_error_residual_allows_next_strict_arm_goal(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2051, 2043, 2051, 2057, 2053, 1965)
            )
        )
        self.transport.auto_status = 6
        self.transport.auto_detail = 20

        goal_handle = self.send_goal(
            self.goal(positions=[0.01] * 5, duration_ms=300)
        )
        self.assertTrue(goal_handle.accepted)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(self.transport.send_calls), 1)
        positions_urad, duration_ms = self.transport.send_calls[0]
        self.assertEqual(positions_urad[:5], (10_000,) * 5)
        self.assertEqual(
            positions_urad[5],
            round(self.positions[5] * 1_000_000),
        )
        self.assertEqual(duration_ms, 300)

    def test_feedback_beyond_firmware_recovery_envelope_is_rejected(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2070, 2048 - 41, 2041, 2071, 2080, 1965)
            )
        )
        self.assertFalse(
            self.send_goal(
                self.goal(positions=[0.0] * 5, duration_ms=2000)
            ).accepted
        )
        self.assertEqual(self.transport.send_calls, [])

    def test_cancel_is_canceled_and_latches_safe_stop(self) -> None:
        goal_handle = self.send_goal(self.goal(duration_ms=1000))
        self.assertTrue(goal_handle.accepted)
        deadline = time.monotonic() + 1.0
        while not self.transport.send_calls and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.transport.send_calls)
        self.assertEqual(self.motion_arbiter.owner, "arm")

        cancel_response = self.wait_future(goal_handle.cancel_goal_async())
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(self.transport.safe_stop_calls, 1)
        self.assertTrue(self.core.blocked)
        self.assertIsNone(self.motion_arbiter.owner)
        retry = self.send_goal(self.goal())
        self.assertFalse(retry.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

    def test_firmware_failure_aborts_and_propagates_error(self) -> None:
        self.transport.auto_status = 7
        self.transport.auto_detail = 6
        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            response.result.error_code,
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
        )
        self.assertIn("status=7", response.result.error_string)
        self.assertEqual(self.transport.safe_stop_calls, 1)

    def test_final_tracking_error_soft_aborts_without_safe_stop(self) -> None:
        self.transport.auto_status = 6
        self.transport.auto_detail = 31
        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertIn(
            "soft abort without safety latch",
            response.result.error_string,
        )
        self.assertEqual(self.transport.safe_stop_calls, 0)
        self.assertFalse(self.core.blocked)

        self.transport.auto_detail = 0
        retry = self.send_goal(self.goal())
        self.assertTrue(retry.accepted)
        retry_response = self.wait_future(retry.get_result_async())
        self.assertEqual(retry_response.status, GoalStatus.STATUS_SUCCEEDED)

    def test_connection_loss_aborts_without_resending_goal(self) -> None:
        goal_handle = self.send_goal(self.goal(duration_ms=1000))
        self.assertTrue(goal_handle.accepted)
        deadline = time.monotonic() + 1.0
        while not self.transport.send_calls and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.transport.send_calls)

        self.adapter.notify_connection_loss("serial disconnected")
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            response.result.error_code,
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
        )
        self.assertIn("serial disconnected", response.result.error_string)
        self.assertEqual(len(self.transport.send_calls), 1)
        self.assertEqual(self.transport.safe_stop_calls, 1)

    def test_concurrent_goal_is_rejected_before_second_transport_call(self) -> None:
        first = self.send_goal(self.goal(duration_ms=1000))
        self.assertTrue(first.accepted)
        deadline = time.monotonic() + 1.0
        while not self.transport.send_calls and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.transport.send_calls)

        second = self.send_goal(self.goal(duration_ms=1000))
        self.assertFalse(second.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

        self.wait_future(first.cancel_goal_async())
        response = self.wait_future(first.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)


if __name__ == "__main__":
    unittest.main()
