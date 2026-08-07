from pathlib import Path
from types import SimpleNamespace
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
    BufferedExecutorState,
    BufferedSetpointFlags,
    BufferedTerminalReason,
    UINT32_MAX,
)
from single_arm_bridge.buffered_action_execution import (  # noqa: E402
    BufferedActionExecutionCore,
    POST_SETTLE_TIMEOUT_S,
    conservative_applied_count,
)
from single_arm_bridge.buffered_transport_driver import (  # noqa: E402
    BufferedExchangeResponse,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.protocol import Hello, MotionResult  # noqa: E402


CALIBRATION = load_calibration(
    PACKAGE_ROOT / "config" / "single_arm_calibration.json"
)


def trajectory(duration_ms: int = 800):
    names = tuple(CALIBRATION.ros_joint_names[:5])
    return validate_buffered_trajectory(
        names,
        (
            TrajectoryPointData((0.0,) * 5, 0),
            TrajectoryPointData((0.08,) * 5, duration_ms * 1_000_000),
        ),
        names,
        {name: CALIBRATION.ros_radian_limits[name] for name in names},
        (0.0,) * 5,
        {name: 0.5 for name in names},
        {name: 1.0 for name in names},
        start_tolerance_rad=0.055,
    )


class SimulatedBufferedTransport:
    def __init__(
        self,
        *,
        settle_error_raw: int = 0,
        position_errors_raw: tuple[int, ...] | None = None,
        position_error_vectors_raw: tuple[tuple[int, ...], ...] | None = None,
        diagnostics_torque_enabled: bool = True,
        heartbeat_error_after_terminal: Exception | None = None,
        position_error: Exception | None = None,
        diagnostics_error: Exception | None = None,
        heartbeat_step_ms: int = 20,
    ) -> None:
        self.tick = 1_000
        self.sequence = 100
        self.accepted = 0
        self.applied = 0
        self.last_sequence = 0
        self.last_apply_tick = 0
        self.total_samples = 41
        self.terminal_sent = False
        self.started = False
        self.commands = []
        self.safe_stop_calls = 0
        self.settle_error_raw = settle_error_raw
        self.position_errors_raw = position_errors_raw
        self.position_error_vectors_raw = position_error_vectors_raw
        self.position_snapshot_calls = 0
        self.diagnostics_calls = 0
        self.diagnostics_torque_enabled = diagnostics_torque_enabled
        self.heartbeat_error_after_terminal = heartbeat_error_after_terminal
        self.position_error = position_error
        self.diagnostics_error = diagnostics_error
        self.heartbeat_step_ms = heartbeat_step_ms
        self.events = []
        self.target_raw = tuple(joint.zero_raw for joint in CALIBRATION.joints)

    def heartbeat(self):
        self.events.append("heartbeat")
        if (
            self.terminal_sent
            and self.heartbeat_error_after_terminal is not None
        ):
            raise self.heartbeat_error_after_terminal
        self.tick = (self.tick + self.heartbeat_step_ms) & UINT32_MAX
        self._update_applied()
        return SimpleNamespace(last_heartbeat_ms=self.tick)

    def exchange_buffered_command(self, command, *, timeout_s):
        del timeout_s
        self._update_applied()
        self.sequence += 1
        self.last_sequence = self.sequence
        self.last_apply_tick = command.first_apply_tick_ms
        self.accepted = command.accepted_samples_after_ack
        self.commands.append(command)
        if command.flags & int(BufferedSetpointFlags.START):
            self.started = True
        if command.flags & int(BufferedSetpointFlags.END):
            self.total_samples = self.accepted
            last_positions = command.payload[-24:]
            del last_positions
        state = (
            BufferedExecutorState.RUNNING.value
            if self.started
            else BufferedExecutorState.PRIMING.value
        )
        result = MotionResult(
            status_code=0,
            sample_count=command.sample_count,
            safety_state=3,
            detail=0,
            request_sequence=self.sequence,
            apply_tick_ms=command.first_apply_tick_ms,
            calibration_hash=CALIBRATION.calibration_hash,
            executor_state=state,
            terminal_reason=BufferedTerminalReason.NONE.value,
            safe_stop_required=False,
            queue_result=0,
            queued_samples=self.accepted - self.applied,
            peak_queued_samples=16,
            accepted_samples=self.accepted,
            applied_samples=self.applied,
        )
        return BufferedExchangeResponse(self.sequence, result)

    def drain_buffered_motion_results(self):
        self._update_applied()
        if (
            self.started
            and self.accepted == self.total_samples
            and self.applied == self.total_samples
            and not self.terminal_sent
        ):
            self.terminal_sent = True
            result = MotionResult(
                status_code=6,
                sample_count=0,
                safety_state=3,
                detail=2,
                request_sequence=self.last_sequence,
                apply_tick_ms=self.last_apply_tick,
                calibration_hash=CALIBRATION.calibration_hash,
                executor_state=BufferedExecutorState.SUCCEEDED.value,
                terminal_reason=BufferedTerminalReason.NONE.value,
                safe_stop_required=False,
                queue_result=0,
                queued_samples=0,
                peak_queued_samples=16,
                accepted_samples=self.accepted,
                applied_samples=self.applied,
            )
            return [BufferedExchangeResponse(self.last_sequence, result)]
        return []

    def get_state(self, include_positions=True):
        assert include_positions is True
        self.events.append("get_state")
        self.position_snapshot_calls += 1
        if self.position_error is not None:
            raise self.position_error
        error = self.settle_error_raw
        if self.position_errors_raw:
            index = min(
                self.position_snapshot_calls - 1,
                len(self.position_errors_raw) - 1,
            )
            error = self.position_errors_raw[index]
        if self.position_error_vectors_raw:
            index = min(
                self.position_snapshot_calls - 1,
                len(self.position_error_vectors_raw) - 1,
            )
            errors = self.position_error_vectors_raw[index]
            assert len(errors) == len(self.target_raw)
            return SimpleNamespace(
                raw_positions=tuple(
                    target + axis_error
                    for target, axis_error in zip(
                        self.target_raw,
                        errors,
                        strict=True,
                    )
                )
            )
        return SimpleNamespace(
            raw_positions=tuple(target + error for target in self.target_raw)
        )

    def get_diagnostics(self):
        self.events.append("get_diagnostics")
        self.diagnostics_calls += 1
        if self.diagnostics_error is not None:
            raise self.diagnostics_error
        joints = tuple(
            SimpleNamespace(
                position_raw=target + self.settle_error_raw,
                torque_enabled=self.diagnostics_torque_enabled,
            )
            for target in self.target_raw
        )
        return SimpleNamespace(joints=joints)

    def safe_stop(self):
        self.events.append("safe_stop")
        self.safe_stop_calls += 1

    def _update_applied(self):
        if not self.started or not self.commands:
            return
        first_tick = self.commands[0].first_apply_tick_ms
        elapsed = (self.tick - first_tick) & UINT32_MAX
        if elapsed > 0x7FFFFFFF:
            return
        due = elapsed // 20 + 1
        self.applied = min(self.accepted, max(self.applied, due))


def hello():
    return Hello(
        protocol_version=1,
        joint_count=6,
        stop_latched=False,
        firmware_version=0x00022C00,
        calibration_hash=CALIBRATION.calibration_hash,
        capabilities=0x00000FFF,
        rejected_frame_count=0,
    )


def test_continuous_runtime_refills_and_requires_terminal_and_settle() -> None:
    transport = SimulatedBufferedTransport()
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "succeeded"
    assert "maximum_apply_lateness_ms=2" in outcome.reason
    assert "first_sample_lead_ms=100" in outcome.reason
    assert "prime_heartbeat_gates=6" in outcome.reason
    assert "prime_frames=2 accepted=16 applied=0 queued=16" in outcome.reason
    assert len(transport.commands) > 2
    assert transport.commands[0].sample_count == 9
    assert transport.commands[1].sample_count == 7
    assert not (
        transport.commands[0].flags & int(BufferedSetpointFlags.START)
    )
    assert transport.commands[1].flags & int(BufferedSetpointFlags.START)
    assert transport.accepted == len(plan.samples)
    assert transport.position_snapshot_calls == 2
    assert transport.diagnostics_calls == 1
    assert transport.events[-7:] == [
        "heartbeat",
        "heartbeat",
        "get_state",
        "heartbeat",
        "get_state",
        "heartbeat",
        "get_diagnostics",
    ]
    assert core.active is False
    assert core.blocked is False


@pytest.mark.parametrize(
    ("elapsed_ms", "expected_first_sample_lead_ms"),
    ((120, 100), (121, 99), (140, 80)),
)
def test_startup_prime_accepts_reviewed_elapsed_window_boundaries(
    elapsed_ms: int,
    expected_first_sample_lead_ms: int,
) -> None:
    transport = SimulatedBufferedTransport(heartbeat_step_ms=elapsed_ms)
    core = BufferedActionExecutionCore(transport, hello(), CALIBRATION)

    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    assert len(transport.commands) == 2
    assert not (
        transport.commands[0].flags & int(BufferedSetpointFlags.START)
    )
    assert transport.commands[1].flags & int(BufferedSetpointFlags.START)
    assert transport.accepted == 16
    assert (
        plan.samples[0].apply_tick_ms - transport.tick
    ) & UINT32_MAX == expected_first_sample_lead_ms
    assert core.blocked is False


@pytest.mark.parametrize("elapsed_ms", (119, 141))
def test_startup_prime_rejects_elapsed_outside_reviewed_window(
    elapsed_ms: int,
) -> None:
    transport = SimulatedBufferedTransport(heartbeat_step_ms=elapsed_ms)
    core = BufferedActionExecutionCore(transport, hello(), CALIBRATION)

    with pytest.raises(Exception, match="buffered start failed"):
        core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    assert core.active is False
    assert core.blocked is True
    assert transport.safe_stop_calls == 1
    assert len(transport.commands) == 1
    assert not (
        transport.commands[0].flags & int(BufferedSetpointFlags.START)
    )


def test_startup_prime_121ms_elapsed_handles_uint32_wraparound() -> None:
    transport = SimulatedBufferedTransport(heartbeat_step_ms=121)
    transport.tick = UINT32_MAX - 30
    core = BufferedActionExecutionCore(transport, hello(), CALIBRATION)

    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    assert plan.samples[0].apply_tick_ms == 310
    assert transport.tick == 211
    assert (plan.samples[0].apply_tick_ms - transport.tick) & UINT32_MAX == 99
    assert len(transport.commands) == 2
    assert transport.commands[1].flags & int(BufferedSetpointFlags.START)
    assert transport.accepted == 16
    assert core.blocked is False


def test_startup_lead_gate_fails_closed_before_start_frame() -> None:
    transport = SimulatedBufferedTransport(heartbeat_step_ms=180)
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
    )

    with pytest.raises(
        Exception,
        match=(
            r"stage=prime_frame_2_heartbeat_1 .*"
            r"startup first-sample lead gate failed: lead_ms=40"
        ),
    ):
        core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    assert core.active is False
    assert core.blocked is True
    assert transport.safe_stop_calls == 1
    assert len(transport.commands) == 1
    assert not (
        transport.commands[0].flags & int(BufferedSetpointFlags.START)
    )


def test_post_settle_failure_is_fail_closed() -> None:
    transport = SimulatedBufferedTransport(settle_error_raw=31)
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert "post-settle" in outcome.reason
    assert transport.safe_stop_calls == 1
    assert transport.position_snapshot_calls >= 1
    assert transport.diagnostics_calls == 0
    assert "heartbeat_gates=" in outcome.reason
    assert "elapsed_ms=" in outcome.reason
    assert "error_trace_raw=1:[31,31,31,31,31,31]" in outcome.reason
    assert "per_axis_minimum_error_raw=[31,31,31,31,31,31]" in outcome.reason
    assert "final_errors_raw=[31,31,31,31,31,31]" in outcome.reason
    assert "best_maximum_error_raw=31" in outcome.reason
    assert core.blocked is True


def test_post_settle_default_timeout_is_extended_without_relaxing_error_gate():
    assert POST_SETTLE_TIMEOUT_S == 2.5


def test_post_settle_failure_records_per_axis_trace_minimum_and_final_errors():
    transport = SimulatedBufferedTransport(
        position_error_vectors_raw=(
            (40, 20, 10, 5, 1, 0),
            (35, 25, 9, 4, 1, 0),
        ),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert (
        "error_trace_raw=1:[40,20,10,5,1,0]|2:[35,25,9,4,1,0]"
        in outcome.reason
    )
    assert "per_axis_minimum_error_raw=[35,20,9,4,1,0]" in outcome.reason
    assert "final_errors_raw=[35,25,9,4,1,0]" in outcome.reason
    assert "best_maximum_error_raw=35" in outcome.reason
    assert transport.safe_stop_calls == 1
    assert core.blocked is True


def test_post_settle_recovers_after_one_outlier_without_full_sweep_retries() -> None:
    transport = SimulatedBufferedTransport(
        settle_error_raw=19,
        position_errors_raw=(31, 19, 19),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "succeeded"
    assert "post_settle_max_error_raw=19" in outcome.reason
    assert transport.position_snapshot_calls == 3
    assert transport.diagnostics_calls == 1


def test_post_settle_extends_deadline_while_strictly_improving() -> None:
    """관절이 목표를 계속 따라잡는 중이면 기본 시한을 넘겨도 기다린다.

    2026-08-07 실기 120 raw/s 램프에서 한 관절이 283 raw 벗어난 채 시작해
    매 관측마다 줄어들다가 고정 2.5s 시한에 걸려 실패했다 — 추세선대로면
    완주까지 더 걸렸을 뿐이었다.
    """
    transport = SimulatedBufferedTransport(
        settle_error_raw=20,
        position_errors_raw=(100, 80, 60, 40, 20, 20),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.12,
        post_settle_poll_interval_s=0.05,
        post_settle_maximum_timeout_s=0.5,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "succeeded"
    assert transport.position_snapshot_calls == 6


def test_post_settle_does_not_extend_when_error_plateaus() -> None:
    """정체 사례(2026-08-06 SHOULDER 32 raw 평형)는 여전히 기본 시한에서 실패한다.

    연장은 매 관측이 직전보다 엄격히 좋아질 때만 일어난다 — 그렇지 않으면
    '기다리면 온다'는 보장이 없는 정체를 계속 기다리게 된다.
    """
    transport = SimulatedBufferedTransport(settle_error_raw=100)
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.12,
        post_settle_poll_interval_s=0.05,
        post_settle_maximum_timeout_s=0.5,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    # 기본 시한(120ms)이 상한(500ms)까지 연장됐다면 훨씬 더 많은 관측이
    # 쌓였을 것이다 — 정체는 몇 회 안에서 그대로 끝나야 한다.
    assert transport.position_snapshot_calls <= 4


def test_post_settle_extension_is_capped_at_the_absolute_ceiling() -> None:
    """개선이 계속돼도 상한을 넘겨 무한정 기다리지는 않는다."""
    transport = SimulatedBufferedTransport(
        settle_error_raw=100,
        position_errors_raw=tuple(range(200, 31, -1)),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.05,
        post_settle_poll_interval_s=0.03,
        post_settle_maximum_timeout_s=0.2,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"


def test_terminal_heartbeat_gate_failure_is_fail_closed_without_position_read():
    transport = SimulatedBufferedTransport(
        heartbeat_error_after_terminal=RuntimeError(
            "HEARTBEAT rejected: status=0 latched=1"
        ),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert "stage=terminal_received" in outcome.reason
    assert "observations=0" in outcome.reason
    assert "heartbeat_gates=0" in outcome.reason
    assert "latched=1" in outcome.reason
    assert transport.position_snapshot_calls == 0
    assert transport.diagnostics_calls == 0
    assert transport.safe_stop_calls == 1
    assert core.blocked is True


def test_position_read_failure_reports_stage_and_is_fail_closed():
    transport = SimulatedBufferedTransport(
        position_error=RuntimeError(
            "GET_STATE rejected: status=0 latched=1"
        ),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    core.start_goal(trajectory(), preserved_gripper_rad=0.0)

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert "stage=position_snapshot_1" in outcome.reason
    assert "observations=0" in outcome.reason
    assert "heartbeat_gates=2" in outcome.reason
    assert "latched=1" in outcome.reason
    assert transport.position_snapshot_calls == 1
    assert transport.diagnostics_calls == 0
    assert transport.safe_stop_calls == 1
    assert core.blocked is True


def test_full_diagnostics_transport_failure_reports_heartbeat_aligned_stage():
    transport = SimulatedBufferedTransport(
        diagnostics_error=RuntimeError("diagnostics timeout"),
    )
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert "stage=full_diagnostics" in outcome.reason
    assert "observations=2" in outcome.reason
    assert "heartbeat_gates=4" in outcome.reason
    assert "diagnostics timeout" in outcome.reason
    assert transport.position_snapshot_calls == 2
    assert transport.diagnostics_calls == 1
    assert transport.safe_stop_calls == 1
    assert core.blocked is True


def test_full_diagnostics_after_position_settle_remains_fail_closed() -> None:
    transport = SimulatedBufferedTransport(diagnostics_torque_enabled=False)
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    transport.target_raw = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break

    assert outcome is not None
    assert outcome.state.value == "aborted"
    assert "final full diagnostics" in outcome.reason
    assert transport.position_snapshot_calls == 2
    assert transport.diagnostics_calls == 1
    assert transport.safe_stop_calls == 1


def test_conservative_progress_handles_uint32_wrap() -> None:
    transport = SimulatedBufferedTransport()
    transport.tick = UINT32_MAX - 80
    core = BufferedActionExecutionCore(transport, hello(), CALIBRATION)
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    first_tick = plan.samples[0].apply_tick_ms

    assert conservative_applied_count(
        plan,
        accepted_samples=16,
        current_tick_ms=(first_tick + 4) & UINT32_MAX,
    ) == 0
    assert conservative_applied_count(
        plan,
        accepted_samples=16,
        current_tick_ms=(first_tick + 5) & UINT32_MAX,
    ) == 1


def test_the_terminal_carries_the_per_joint_measurement_to_the_sender() -> None:
    """실행 코어가 만든 진짜 종단 문자열을 sender 파서에 그대로 넣어본다.

    2026-08-06 A4 성공 회차와 A4.5 실패 회차는 최대값 하나만 남아 있어서
    도달 자세를 되살릴 수 없었다. 어느 관절이 얼마나 못 갔는지 모르면 그
    오차가 TCP 에서 몇 mm 였는지도 계산할 수 없다. 여기서 그 경로를 끝까지
    확인한다 — 두 파일이 각자의 가정으로만 문자열을 만들면 드리프트가
    보이지 않기 때문이다.
    """
    import importlib.util
    import sys
    from pathlib import Path
    from types import SimpleNamespace

    transport = SimulatedBufferedTransport(settle_error_raw=19)
    core = BufferedActionExecutionCore(
        transport,
        hello(),
        CALIBRATION,
        post_settle_timeout_s=0.01,
        post_settle_poll_interval_s=0.0,
    )
    plan = core.start_goal(trajectory(), preserved_gripper_rad=0.0)
    targets = tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            CALIBRATION.joints,
            (*plan.final_arm_positions_rad, plan.preserved_gripper_rad),
            strict=True,
        )
    )
    transport.target_raw = targets

    outcome = None
    for _ in range(80):
        outcome = core.poll()
        if outcome is not None:
            break
    assert outcome is not None and outcome.state.value == "succeeded"

    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "execute_buffered_action_plan_once_vector_path",
        root / "tools" / "execute_buffered_action_plan_once.py",
    )
    sender = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = sender
    spec.loader.exec_module(sender)

    evidence = sender.validate_action_terminal(
        sender.ACTION_STATUS_SUCCEEDED,
        SimpleNamespace(
            error_code=sender.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
            error_string=outcome.reason,
        ),
    )
    assert evidence.post_settle_target_raw == targets
    assert evidence.post_settle_error_raw is not None
    assert max(evidence.post_settle_error_raw) == 19
    for target, measured, error in zip(
        evidence.post_settle_target_raw,
        evidence.post_settle_measured_raw,
        evidence.post_settle_error_raw,
        strict=True,
    ):
        assert abs(measured - target) == error
