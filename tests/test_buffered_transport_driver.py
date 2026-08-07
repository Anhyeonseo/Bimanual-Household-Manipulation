from pathlib import Path
import struct
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.action_validation import (  # noqa: E402
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (  # noqa: E402
    BufferedAdapterState,
    BufferedBatchScheduler,
    BufferedExecutorState,
    BufferedTerminalReason,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.buffered_transport_driver import (  # noqa: E402
    BufferedExchangeResponse,
    BufferedTransportDriver,
    BufferedTransportDriverError,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.protocol import (  # noqa: E402
    BUFFERED_SETPOINT_HEADER,
    BufferedSetpointFlags,
    MotionResult,
)


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


def make_scheduler(duration_ms: int = 800):
    calibration = load_calibration(CALIBRATION_PATH)
    names = tuple(calibration.ros_joint_names[:5])
    trajectory = validate_buffered_trajectory(
        names,
        (
            TrajectoryPointData((0.0,) * 5, 0),
            TrajectoryPointData((0.08,) * 5, duration_ms * 1_000_000),
        ),
        names,
        {name: calibration.ros_radian_limits[name] for name in names},
        (0.0,) * 5,
        {name: 0.5 for name in names},
        {name: 1.0 for name in names},
        start_tolerance_rad=0.01,
    )
    plan = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=0.06,
        current_tick_ms=1_000,
    )
    return plan, BufferedBatchScheduler(plan)


def result_for(command, *, sequence: int, state: int, applied: int = 0):
    accepted = command.accepted_samples_after_ack
    return MotionResult(
        status_code=0,
        sample_count=command.sample_count,
        safety_state=3,
        detail=0,
        request_sequence=sequence,
        apply_tick_ms=command.first_apply_tick_ms,
        calibration_hash=0xB317C672,
        executor_state=state,
        terminal_reason=BufferedTerminalReason.NONE.value,
        safe_stop_required=False,
        queue_result=0,
        queued_samples=accepted - applied,
        peak_queued_samples=16,
        accepted_samples=accepted,
        applied_samples=applied,
    )


class ScriptedPort:
    def __init__(self, scripts):
        self.scripts = list(scripts)
        self.commands = []
        self.timeouts = []

    def exchange_buffered_command(self, command, *, timeout_s):
        self.commands.append(command)
        self.timeouts.append(timeout_s)
        script = self.scripts.pop(0)
        if isinstance(script, BaseException):
            raise script
        return script(command)


def response_script(sequence: int, state: int, *, applied: int = 0):
    return lambda command: BufferedExchangeResponse(
        sequence,
        result_for(command, sequence=sequence, state=state, applied=applied),
    )


def test_driver_encodes_and_exchanges_prime_9_plus_7_once_each() -> None:
    _, scheduler = make_scheduler()
    port = ScriptedPort(
        [
            response_script(101, BufferedExecutorState.PRIMING.value),
            response_script(102, BufferedExecutorState.RUNNING.value),
        ]
    )
    driver = BufferedTransportDriver(scheduler, port)

    first = driver.service_once(current_tick_ms=1_000)
    second = driver.service_once(current_tick_ms=1_120)

    assert first is not None and first.sample_count == 9
    assert second is not None and second.sample_count == 7
    assert first.flags == int(
        BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.BEGIN
    )
    assert second.flags == int(
        BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.START
    )
    assert first.flags & int(BufferedSetpointFlags.VALIDATION_ONLY) == 0
    assert BUFFERED_SETPOINT_HEADER.unpack_from(first.payload)[:3] == (
        first.first_apply_tick_ms,
        9,
        1,
    )
    assert driver.commands_sent == 2
    assert len(port.commands) == 2
    assert scheduler.state is BufferedAdapterState.RUNNING


def test_timeout_transmits_once_discards_pending_and_never_retries() -> None:
    _, scheduler = make_scheduler()
    port = ScriptedPort([TimeoutError("injected")])
    driver = BufferedTransportDriver(scheduler, port)

    with pytest.raises(BufferedTransportDriverError, match="timeout"):
        driver.service_once(current_tick_ms=1_000)
    assert driver.commands_sent == 1
    assert len(port.commands) == 1
    snapshot = scheduler.snapshot()
    assert snapshot.state is BufferedAdapterState.ABORTED
    assert snapshot.pending_batch is False
    assert snapshot.reason == "setpoint_status_timeout"
    with pytest.raises(BufferedTransportDriverError, match="refused"):
        driver.service_once(current_tick_ms=1_020)
    assert len(port.commands) == 1


def test_response_sequence_mismatch_is_fail_closed_before_ack() -> None:
    _, scheduler = make_scheduler()

    def mismatch(command):
        return BufferedExchangeResponse(
            200,
            result_for(
                command,
                sequence=201,
                state=BufferedExecutorState.PRIMING.value,
            ),
        )

    port = ScriptedPort([mismatch])
    driver = BufferedTransportDriver(scheduler, port)
    with pytest.raises(BufferedTransportDriverError, match="sequence mismatch"):
        driver.service_once(current_tick_ms=1_000)
    assert scheduler.snapshot().reason == "response_sequence_mismatch"
    assert len(port.commands) == 1


def test_legacy_16_byte_result_is_rejected_without_second_exchange() -> None:
    _, scheduler = make_scheduler()

    def legacy(command):
        return BufferedExchangeResponse(
            300,
            MotionResult(
                status_code=0,
                sample_count=command.sample_count,
                safety_state=3,
                detail=0,
                request_sequence=300,
                apply_tick_ms=command.first_apply_tick_ms,
                calibration_hash=0xB317C672,
            ),
        )

    port = ScriptedPort([legacy])
    driver = BufferedTransportDriver(scheduler, port)
    with pytest.raises(BufferedTransportDriverError, match="host admission"):
        driver.service_once(current_tick_ms=1_000)
    assert scheduler.state is BufferedAdapterState.ABORTED
    assert len(port.commands) == 1


def test_terminal_before_pending_ack_aborts_and_does_not_exchange_again() -> None:
    plan, scheduler = make_scheduler()
    port = ScriptedPort([])
    driver = BufferedTransportDriver(scheduler, port)
    scheduler.next_batch(current_tick_ms=1_000)
    terminal = MotionResult(
        status_code=6,
        sample_count=0,
        safety_state=3,
        detail=0,
        request_sequence=400,
        apply_tick_ms=plan.samples[0].apply_tick_ms,
        calibration_hash=0xB317C672,
        executor_state=BufferedExecutorState.ABORTED.value,
        terminal_reason=BufferedTerminalReason.CONNECTION_LOSS.value,
        safe_stop_required=True,
        queue_result=0,
        queued_samples=0,
        peak_queued_samples=0,
        accepted_samples=0,
        applied_samples=0,
    )
    with pytest.raises(BufferedTransportDriverError, match="host admission"):
        driver.observe_terminal(BufferedExchangeResponse(400, terminal))
    assert scheduler.snapshot().reason == "terminal_while_batch_pending"
    assert driver.commands_sent == 0


def test_malformed_payload_header_cannot_be_created_by_driver() -> None:
    _, scheduler = make_scheduler()
    port = ScriptedPort(
        [response_script(500, BufferedExecutorState.PRIMING.value)]
    )
    driver = BufferedTransportDriver(scheduler, port)
    command = driver.service_once(current_tick_ms=1_000)
    assert command is not None
    first_tick, count, arm_mask, reserved = struct.unpack_from(
        "<IBBH", command.payload
    )
    assert (first_tick, count, arm_mask, reserved) == (
        command.first_apply_tick_ms,
        command.sample_count,
        1,
        0,
    )


def test_driver_refills_at_watermark_without_reusing_prime_frames() -> None:
    _, scheduler = make_scheduler()
    port = ScriptedPort(
        [
            response_script(601, BufferedExecutorState.PRIMING.value),
            response_script(602, BufferedExecutorState.RUNNING.value),
            response_script(603, BufferedExecutorState.RUNNING.value, applied=6),
        ]
    )
    driver = BufferedTransportDriver(scheduler, port)
    driver.service_once(current_tick_ms=1_000)
    driver.service_once(current_tick_ms=1_120)
    scheduler.record_applied(6)
    refill = driver.service_once(current_tick_ms=1_261)

    assert refill is not None
    assert refill.first_sample_index == 17
    assert refill.sample_count == 6
    assert refill.accepted_samples_after_ack == 22
    assert driver.commands_sent == 3
    assert [(item.first_sample_index, item.sample_count) for item in port.commands] == [
        (1, 9),
        (10, 7),
        (17, 6),
    ]
    assert scheduler.snapshot().queued_samples == 16
