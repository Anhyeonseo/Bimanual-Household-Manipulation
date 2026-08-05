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
    from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectoryPoint

    from single_arm_bridge.action_execution import MotionExecutionCore
    from single_arm_bridge.calibration import load_calibration
    from single_arm_bridge.commanded_setpoint_state import CommandedSetpointState
    from single_arm_bridge.follow_joint_trajectory_server import (
        FollowJointTrajectoryActionAdapter,
    )
    from single_arm_bridge.motion_goal_arbiter import MotionGoalArbiter
    from single_arm_bridge.parallel_gripper_command_server import (
        ParallelGripperCommandActionAdapter,
    )
    from single_arm_bridge.protocol import Hello, MotionResult

    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


if ROS_AVAILABLE:

    class FakeGripperTransport:
        def __init__(self) -> None:
            self.next_sequence = 700
            self.send_calls = []
            self.results = []
            self.safe_stop_calls = 0
            self.auto_status = None
            self.auto_detail = 0
            self.on_send = None

        def send_setpoint(self, positions_urad, duration_ms):
            sequence = self.next_sequence
            self.next_sequence += 1
            self.send_calls.append((tuple(positions_urad), duration_ms))
            if self.on_send is not None:
                self.on_send(tuple(positions_urad))
            if self.auto_status is not None:
                self.results.append(
                    MotionResult(
                        self.auto_status,
                        1,
                        3,
                        self.auto_detail,
                        sequence,
                        1200,
                        0xB317C672,
                    )
                )
            return MotionResult(
                0,
                1,
                3,
                0,
                sequence,
                1200,
                0xB317C672,
            )

        def drain_motion_results(self):
            results = list(self.results)
            self.results.clear()
            return results

        def safe_stop(self):
            self.safe_stop_calls += 1


@unittest.skipUnless(ROS_AVAILABLE, "ROS Jazzy environment is not sourced")
class ParallelGripperRosIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.calibration = load_calibration(CALIBRATION_PATH)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        suffix = uuid.uuid4().hex
        self.action_name = f"/test_left_gripper_{suffix}/gripper_cmd"
        self.arm_action_name = f"/test_left_arm_{suffix}/follow"
        self.server_node = Node(f"test_gripper_server_{suffix}")
        self.client_node = Node(f"test_gripper_client_{suffix}")
        self.transport = FakeGripperTransport()
        hello = Hello(
            1,
            6,
            False,
            0x00022A00,
            0xB317C672,
            0x00000FFF,
            0,
        )
        self.core = MotionExecutionCore(
            self.transport,
            hello,
            self.calibration,
        )
        self.ready = True
        self.positions = (0.01, 0.01, 0.01, 0.01, 0.01, 0.0)
        self.motion_arbiter = MotionGoalArbiter()
        self.setpoint_state = CommandedSetpointState()
        self.adapter = ParallelGripperCommandActionAdapter(
            self.server_node,
            self.core,
            self.calibration,
            lambda: self.ready,
            lambda: self.positions,
            motion_arbiter=self.motion_arbiter,
            setpoint_state=self.setpoint_state,
            action_name=self.action_name,
            poll_interval_s=0.005,
            completion_timeout_s=0.2,
        )
        self.arm_adapter = FollowJointTrajectoryActionAdapter(
            self.server_node,
            self.core,
            self.calibration,
            lambda: self.ready,
            lambda: self.positions,
            motion_arbiter=self.motion_arbiter,
            setpoint_state=self.setpoint_state,
            action_name=self.arm_action_name,
            poll_interval_s=0.005,
            completion_timeout_s=0.2,
        )
        self.client = ActionClient(
            self.client_node,
            ParallelGripperCommand,
            self.action_name,
        )
        self.arm_client = ActionClient(
            self.client_node,
            FollowJointTrajectory,
            self.arm_action_name,
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
        self.assertTrue(self.arm_client.wait_for_server(timeout_sec=2.0))

    def tearDown(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.client.destroy()
        self.arm_client.destroy()
        self.adapter.destroy()
        self.arm_adapter.destroy()
        self.client_node.destroy_node()
        self.server_node.destroy_node()

    def wait_future(self, future, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(future.done(), "ROS gripper Action future timed out")
        result = future.result()
        if result is None and future.exception() is not None:
            raise future.exception()
        return result

    def goal(self, position=0.1, name=None, velocity=(), effort=()):
        request = ParallelGripperCommand.Goal()
        request.command.name = [
            name or self.calibration.ros_joint_names[5]
        ]
        request.command.position = [position]
        request.command.velocity = list(velocity)
        request.command.effort = list(effort)
        return request

    def send_goal(self, goal, feedback_callback=None):
        return self.wait_future(
            self.client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            )
        )

    def arm_goal(self):
        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = self.calibration.ros_joint_names[:5]
        point = JointTrajectoryPoint()
        point.positions = [0.02] * 5
        point.time_from_start.sec = 1
        request.trajectory.points = [point]
        return request

    def send_arm_goal(self):
        return self.wait_future(
            self.arm_client.send_goal_async(self.arm_goal())
        )

    def wait_until_sent(self) -> None:
        deadline = time.monotonic() + 1.0
        while not self.transport.send_calls and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.transport.send_calls)

    def test_success_result_feedback_and_arm_preservation(self) -> None:
        self.transport.auto_status = 6
        self.transport.auto_detail = 20
        self.transport.on_send = self._apply_fake_feedback
        feedback = []
        goal_handle = self.send_goal(
            self.goal(0.1),
            feedback_callback=lambda message: feedback.append(message.feedback),
        )
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.reached_goal)
        self.assertFalse(response.result.stalled)
        self.assertEqual(response.result.state.name, ["left_gripper_joint"])
        self.assertAlmostEqual(response.result.state.position[0], 0.1)
        positions_urad, duration_ms = self.transport.send_calls[0]
        self.assertEqual(positions_urad[:5], (10_000,) * 5)
        self.assertEqual(positions_urad[5], 100_000)
        self.assertEqual(duration_ms, 1000)
        self.assertTrue(feedback)
        self.assertIsNone(self.motion_arbiter.owner)

    def test_contact_gripper_target_survives_following_arm_goal(self) -> None:
        self.transport.auto_status = 6
        self.transport.auto_detail = 20

        def apply_contact_feedback(positions_urad) -> None:
            feedback = [value / 1_000_000.0 for value in positions_urad]
            feedback[5] = 0.07
            self.positions = tuple(feedback)

        self.transport.on_send = apply_contact_feedback
        gripper_handle = self.send_goal(self.goal(0.1))
        self.assertTrue(gripper_handle.accepted)
        gripper_response = self.wait_future(gripper_handle.get_result_async())
        self.assertEqual(gripper_response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(self.positions[5], 0.07)
        self.assertAlmostEqual(self.setpoint_state.snapshot()[5], 0.1)

        self.transport.on_send = None
        arm_handle = self.send_arm_goal()
        self.assertTrue(arm_handle.accepted)
        arm_response = self.wait_future(arm_handle.get_result_async())
        self.assertEqual(arm_response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(self.transport.send_calls), 2)
        self.assertEqual(self.transport.send_calls[1][0][5], 100_000)

    def _apply_fake_feedback(self, positions_urad) -> None:
        self.positions = tuple(value / 1_000_000.0 for value in positions_urad)

    def test_invalid_and_unsafe_goals_never_reach_transport(self) -> None:
        self.assertFalse(self.send_goal(self.goal(1.91986)).accepted)
        self.assertFalse(self.send_goal(self.goal(name="unknown_joint")).accepted)
        self.assertFalse(self.send_goal(self.goal(velocity=(0.1,))).accepted)
        self.assertFalse(self.send_goal(self.goal(effort=(1.0,))).accepted)

        scheduled = self.goal()
        scheduled.command.header.stamp.sec = 1
        self.assertFalse(self.send_goal(scheduled).accepted)

        self.ready = False
        self.assertFalse(self.send_goal(self.goal()).accepted)
        self.ready = True
        self.positions = None
        self.assertFalse(self.send_goal(self.goal()).accepted)
        self.positions = (-0.1, 0.01, 0.01, 0.01, 0.01, 0.0)
        self.assertFalse(self.send_goal(self.goal()).accepted)

        self.positions = (0.01, 0.01, 0.01, 0.01, 0.01, 0.0)
        self.assertTrue(self.motion_arbiter.try_reserve("arm"))
        self.assertFalse(self.send_goal(self.goal()).accepted)
        self.assertTrue(self.motion_arbiter.release("arm"))
        self.assertEqual(self.transport.send_calls, [])

    def test_gripper_is_blocked_while_arm_feedback_needs_recovery(self) -> None:
        self.positions = tuple(
            self.calibration.raw_feedback_to_radians(
                (2070, 2043, 2041, 2131, 2080, 1965)
            )
        )
        self.assertFalse(self.send_goal(self.goal()).accepted)
        self.assertEqual(self.transport.send_calls, [])

    def test_cancel_latches_safe_stop_and_releases_owner(self) -> None:
        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)
        self.wait_until_sent()
        self.assertEqual(self.motion_arbiter.owner, "gripper")

        cancel_response = self.wait_future(goal_handle.cancel_goal_async())
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(response.result.reached_goal)
        self.assertEqual(self.transport.safe_stop_calls, 1)
        self.assertIsNone(self.motion_arbiter.owner)

        retry = self.send_goal(self.goal())
        self.assertFalse(retry.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

    def test_firmware_failure_aborts(self) -> None:
        self.transport.auto_status = 7
        self.transport.auto_detail = 6
        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)

        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.reached_goal)
        self.assertEqual(self.transport.safe_stop_calls, 1)
        self.assertIsNone(self.motion_arbiter.owner)
        self.assertIsNone(self.setpoint_state.snapshot())

    def test_connection_loss_aborts_without_resend(self) -> None:
        goal_handle = self.send_goal(self.goal())
        self.assertTrue(goal_handle.accepted)
        self.wait_until_sent()

        self.adapter.notify_connection_loss("serial disconnected")
        response = self.wait_future(goal_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.reached_goal)
        self.assertEqual(len(self.transport.send_calls), 1)
        self.assertEqual(self.transport.safe_stop_calls, 1)
        self.assertIsNone(self.motion_arbiter.owner)

    def test_concurrent_goal_is_rejected(self) -> None:
        first = self.send_goal(self.goal())
        self.assertTrue(first.accepted)
        self.wait_until_sent()

        second = self.send_goal(self.goal())
        self.assertFalse(second.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

        self.wait_future(first.cancel_goal_async())
        response = self.wait_future(first.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)

    def test_active_arm_action_rejects_gripper_action(self) -> None:
        arm_handle = self.send_arm_goal()
        self.assertTrue(arm_handle.accepted)
        self.wait_until_sent()
        self.assertEqual(self.motion_arbiter.owner, "arm")

        gripper_handle = self.send_goal(self.goal())
        self.assertFalse(gripper_handle.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

        self.wait_future(arm_handle.cancel_goal_async())
        response = self.wait_future(arm_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertIsNone(self.motion_arbiter.owner)

    def test_active_gripper_action_rejects_arm_action(self) -> None:
        gripper_handle = self.send_goal(self.goal())
        self.assertTrue(gripper_handle.accepted)
        self.wait_until_sent()
        self.assertEqual(self.motion_arbiter.owner, "gripper")

        arm_handle = self.send_arm_goal()
        self.assertFalse(arm_handle.accepted)
        self.assertEqual(len(self.transport.send_calls), 1)

        self.wait_future(gripper_handle.cancel_goal_async())
        response = self.wait_future(gripper_handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertIsNone(self.motion_arbiter.owner)


if __name__ == "__main__":
    unittest.main()
