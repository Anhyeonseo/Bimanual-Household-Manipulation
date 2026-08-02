import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.action_execution import (  # noqa: E402
    MAX_FINAL_ERROR_RAW,
    ExecutionError,
    ExecutionOutcome,
    MotionExecutionCore,
    TerminalState,
    format_execution_outcome,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.hardware_identity import (  # noqa: E402
    HardwareIdentityError,
)
from single_arm_bridge.protocol import Hello, MotionResult  # noqa: E402


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


class FakeExecutionTransport:
    def __init__(self) -> None:
        self.next_sequence = 100
        self.send_calls = []
        self.safe_stop_calls = 0
        self.results = []
        self.send_error = None
        self.safe_stop_error = None
        self.drain_error = None
        self.accepted_status = 0
        self.accepted_sample_count = 1
        self.accepted_calibration_hash = 0x8AD27897

    def send_setpoint(self, positions_urad, duration_ms):
        if self.send_error is not None:
            raise self.send_error
        sequence = self.next_sequence
        self.next_sequence += 1
        self.send_calls.append((tuple(positions_urad), duration_ms))
        return MotionResult(
            self.accepted_status,
            self.accepted_sample_count,
            3,
            0,
            sequence,
            1200,
            self.accepted_calibration_hash,
        )

    def drain_motion_results(self):
        if self.drain_error is not None:
            raise self.drain_error
        results = list(self.results)
        self.results.clear()
        return results

    def safe_stop(self):
        self.safe_stop_calls += 1
        if self.safe_stop_error is not None:
            raise self.safe_stop_error

    def queue_result(self, sequence, status=6, detail=0):
        self.results.append(
            MotionResult(
                status,
                1,
                3,
                detail,
                sequence,
                1200,
                self.accepted_calibration_hash,
            )
        )


class SingleArmActionExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.calibration = load_calibration(CALIBRATION_PATH)

    @staticmethod
    def hello(
        calibration_hash=0x8AD27897,
        firmware_version=0x00021900,
        protocol_version=1,
        joint_count=6,
        capabilities=0x000007FF,
        stop_latched=False,
    ) -> Hello:
        return Hello(
            protocol_version,
            joint_count,
            stop_latched,
            firmware_version,
            calibration_hash,
            capabilities,
            0,
        )

    def make_core(self, transport=None, hello=None):
        selected_transport = transport or FakeExecutionTransport()
        selected_hello = hello or self.hello()
        return MotionExecutionCore(
            selected_transport,
            selected_hello,
            self.calibration,
        )

    def test_terminal_diagnostic_preserves_status_and_detail(self) -> None:
        message = format_execution_outcome(
            ExecutionOutcome(
                TerminalState.ABORTED,
                42,
                6,
                31,
                "final error 31 exceeds 30 raw",
            )
        )
        self.assertEqual(
            message,
            "ARM_EXECUTION_TERMINAL state=aborted sequence=42 "
            "status=6 detail=31 reason=final error 31 exceeds 30 raw",
        )

    def test_completion_tolerance_is_30_raw(self) -> None:
        self.assertEqual(MAX_FINAL_ERROR_RAW, 30)

    def test_identity_mismatch_blocks_before_any_setpoint(self) -> None:
        invalid_hellos = (
            self.hello(calibration_hash=0xDEADBEEF),
            self.hello(firmware_version=0x00020600),
            self.hello(protocol_version=2),
            self.hello(joint_count=5),
            self.hello(capabilities=0),
        )
        for hello in invalid_hellos:
            with self.subTest(hello=hello):
                transport = FakeExecutionTransport()
                with self.assertRaises(HardwareIdentityError):
                    self.make_core(transport, hello)
                self.assertEqual(transport.send_calls, [])

    def test_latched_stop_blocks_goal_until_explicit_recovery(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport, self.hello(stop_latched=True))
        self.assertTrue(core.blocked)
        with self.assertRaisesRegex(ExecutionError, "explicit recovery"):
            core.start_goal([0.0] * 6, 1000)
        self.assertEqual(transport.send_calls, [])

    def test_goal_succeeds_only_after_completion_within_tolerance(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport)
        sequence = core.start_goal([0.0] * 6, 1000)
        self.assertTrue(core.active)
        self.assertIsNone(core.poll())

        transport.queue_result(sequence, status=6, detail=30)
        outcome = core.poll()
        self.assertIsNotNone(outcome)
        self.assertEqual(outcome.state, TerminalState.SUCCEEDED)
        self.assertEqual(outcome.final_error_raw, 30)
        self.assertFalse(core.active)
        self.assertFalse(core.blocked)

    def test_completion_above_error_tolerance_soft_aborts(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport)
        sequence = core.start_goal([0.0] * 6, 1000)
        transport.queue_result(sequence, status=6, detail=31)
        outcome = core.poll()
        self.assertEqual(outcome.state, TerminalState.ABORTED)
        self.assertIn("soft abort without safety latch", outcome.reason)
        self.assertFalse(core.blocked)
        self.assertEqual(transport.safe_stop_calls, 0)
        core.start_goal([0.0] * 6, 1000)
        self.assertEqual(len(transport.send_calls), 2)

    def test_firmware_failure_status_aborts_and_requests_safe_stop(self) -> None:
        for status in (7, 8, 9):
            with self.subTest(status=status):
                transport = FakeExecutionTransport()
                core = self.make_core(transport)
                sequence = core.start_goal([0.0] * 6, 1000)
                transport.queue_result(sequence, status=status, detail=6)
                outcome = core.poll()
                self.assertEqual(outcome.state, TerminalState.ABORTED)
                self.assertEqual(outcome.status_code, status)
                self.assertTrue(core.blocked)
                self.assertEqual(transport.safe_stop_calls, 1)

    def test_cancel_latches_stop_and_late_success_is_stale(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport)
        sequence = core.start_goal([0.0] * 6, 1000)
        outcome = core.cancel_active_goal()
        self.assertEqual(outcome.state, TerminalState.CANCELED)
        self.assertEqual(transport.safe_stop_calls, 1)
        self.assertTrue(core.blocked)
        self.assertFalse(core.active)

        transport.queue_result(sequence, status=6, detail=0)
        self.assertIsNone(core.poll())
        self.assertEqual(core.stale_result_count, 1)

    def test_cancel_without_stop_acknowledgement_aborts(self) -> None:
        transport = FakeExecutionTransport()
        transport.safe_stop_error = RuntimeError("no acknowledgement")
        core = self.make_core(transport)
        core.start_goal([0.0] * 6, 1000)
        outcome = core.cancel_active_goal()
        self.assertEqual(outcome.state, TerminalState.ABORTED)
        self.assertIn("acknowledgement failed", outcome.reason)
        self.assertTrue(core.blocked)

    def test_connection_loss_never_resends_old_goal_after_recovery(self) -> None:
        old_transport = FakeExecutionTransport()
        core = self.make_core(old_transport)
        sequence = core.start_goal([0.0] * 6, 1000)
        outcome = core.handle_connection_loss("serial disconnected")
        self.assertEqual(outcome.state, TerminalState.ABORTED)
        self.assertEqual(outcome.request_sequence, sequence)
        self.assertTrue(core.blocked)
        self.assertEqual(old_transport.safe_stop_calls, 1)

        new_transport = FakeExecutionTransport()
        core.replace_transport_after_explicit_recovery(
            new_transport,
            self.hello(),
        )
        self.assertFalse(core.blocked)
        self.assertEqual(new_transport.send_calls, [])
        core.start_goal([0.0] * 6, 1000)
        self.assertEqual(len(new_transport.send_calls), 1)
        self.assertEqual(len(old_transport.send_calls), 1)

    def test_recovery_revalidates_identity_before_unblocking(self) -> None:
        core = self.make_core()
        core.handle_connection_loss("serial disconnected")
        replacement = FakeExecutionTransport()
        with self.assertRaises(HardwareIdentityError):
            core.replace_transport_after_explicit_recovery(
                replacement,
                self.hello(calibration_hash=0xDEADBEEF),
            )
        self.assertTrue(core.blocked)
        self.assertEqual(replacement.send_calls, [])

    def test_recovery_rejects_latched_replacement(self) -> None:
        core = self.make_core()
        core.handle_connection_loss("serial disconnected")
        replacement = FakeExecutionTransport()
        with self.assertRaisesRegex(ExecutionError, "stop is latched"):
            core.replace_transport_after_explicit_recovery(
                replacement,
                self.hello(stop_latched=True),
            )
        self.assertTrue(core.blocked)
        self.assertEqual(replacement.send_calls, [])

    def test_concurrent_goal_is_rejected_without_second_send(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport)
        core.start_goal([0.0] * 6, 1000)
        with self.assertRaisesRegex(ExecutionError, "another motion goal"):
            core.start_goal([0.0] * 6, 1000)
        self.assertEqual(len(transport.send_calls), 1)

    def test_invalid_acceptance_response_blocks_execution(self) -> None:
        transports = []
        for field, value in (
            ("accepted_status", 2),
            ("accepted_sample_count", 2),
            ("accepted_calibration_hash", 0xDEADBEEF),
        ):
            with self.subTest(field=field):
                transport = FakeExecutionTransport()
                setattr(transport, field, value)
                transports.append(transport)
                core = self.make_core(transport)
                with self.assertRaises(ExecutionError):
                    core.start_goal([0.0] * 6, 1000)
                self.assertTrue(core.blocked)
                self.assertEqual(transport.safe_stop_calls, 1)
        self.assertTrue(all(len(item.send_calls) == 1 for item in transports))

    def test_transport_send_failure_blocks_execution(self) -> None:
        transport = FakeExecutionTransport()
        transport.send_error = RuntimeError("serial write failed")
        core = self.make_core(transport)
        with self.assertRaisesRegex(ExecutionError, "transport failed"):
            core.start_goal([0.0] * 6, 1000)
        self.assertTrue(core.blocked)
        self.assertEqual(transport.safe_stop_calls, 1)

    def test_poll_transport_failure_aborts_without_resend(self) -> None:
        transport = FakeExecutionTransport()
        core = self.make_core(transport)
        sequence = core.start_goal([0.0] * 6, 1000)
        transport.drain_error = RuntimeError("serial read failed")
        outcome = core.poll()
        self.assertEqual(outcome.state, TerminalState.ABORTED)
        self.assertEqual(outcome.request_sequence, sequence)
        self.assertTrue(core.blocked)
        self.assertEqual(len(transport.send_calls), 1)
        self.assertEqual(transport.safe_stop_calls, 1)

    def test_duration_is_checked_before_transport(self) -> None:
        for duration in (299, 2001):
            with self.subTest(duration=duration):
                transport = FakeExecutionTransport()
                core = self.make_core(transport)
                with self.assertRaisesRegex(ExecutionError, "300..2000"):
                    core.start_goal([0.0] * 6, duration)
                self.assertFalse(core.blocked)
                self.assertEqual(transport.send_calls, [])


if __name__ == "__main__":
    unittest.main()
