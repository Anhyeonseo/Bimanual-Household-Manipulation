from pathlib import Path
from types import SimpleNamespace

import pytest

from single_arm_bridge.bimanual_stream_adapter import (
    AdapterState,
    BimanualTransientFeedbackError,
    BimanualStreamContractError,
    BimanualStreamError,
    BimanualStreamOwnershipError,
    CALIBRATION_HASH,
    CANONICAL_JOINT_NAMES,
    F8_CAPABILITIES,
    F8_FIRMWARE_VERSION,
    ResidentBimanualStreamAdapter,
    TimedJointPoint,
    load_operational_limits,
    normalize_joint_positions,
)
from single_arm_bridge.stream_transport_v2 import (
    StreamResponseTimeoutError,
    StreamTransportV2Error,
)
from single_arm_bridge.stream_protocol_v2 import (
    ARM_MASK_BOTH,
    StreamContractResultV2,
    StreamExecutorStateV2,
    StreamStatusCodeV2,
    StreamTerminalReasonV2,
)


ROOT = Path(__file__).resolve().parents[1]
LIMITS_PATH = ROOT / "config/bimanual_operational_limits.json"


def namespace(**values):
    return SimpleNamespace(**values)


class FakeTransport:
    def __init__(self):
        self.tick = 1000
        self.calls = []
        self.opened = None
        self.appended = []
        self.spliced = []
        self.safe_stop_count = 0
        self.reject_append = False
        self.executor_state = StreamExecutorStateV2.RUNNING
        self.executor_safe_stop = False
        self.dispatch_failures = 0
        self.dispatch_active = False
        self.tracking_failures = 0
        self.tracking_active = False
        self.tracking_pending = False
        self.arm_error = None
        self.shadow_status = 0
        self.shadow_positions_urad = (0,) * 12
        self.feedback_positions = (0,) * 12
        self.feedback_age_ms = tuple(range(12))
        self.feedback_transport_errors_remaining = 0

    def enter_binary_mode(self):
        self.calls.append("hello")
        return namespace(
            protocol_version=2,
            joint_count=12,
            stop_latched=False,
            firmware_version=F8_FIRMWARE_VERSION,
            left_calibration_hash=CALIBRATION_HASH,
            right_calibration_hash=CALIBRATION_HASH,
            capabilities=F8_CAPABILITIES,
        )

    def heartbeat(self):
        self.calls.append("heartbeat")
        return namespace(
            stop_latched=False,
            status_code=0,
            last_heartbeat_ms=self.tick,
        )

    get_state = heartbeat

    def prepare_shadow(self):
        self.calls.append("shadow")
        return namespace(
            status_code=self.shadow_status,
            joint_count=12,
            left_present_mask=0x3F,
            right_present_mask=0x3F,
            positions_raw=(2048,) * 12,
            unwrapped_positions_raw=(2048,) * 12,
            anchor_positions_urad=self.shadow_positions_urad,
        )

    def get_dispatch_diagnostics(self):
        self.calls.append("dispatch")
        return namespace(
            status_code=0,
            active=self.dispatch_active,
            faulted=bool(self.dispatch_failures),
            ready=not self.dispatch_active,
            failure_count=self.dispatch_failures,
        )

    def get_tracking_diagnostics(self):
        self.calls.append("tracking")
        return namespace(
            status_code=0,
            active=self.tracking_active,
            pending=self.tracking_pending,
            failed_pairs=self.tracking_failures,
        )

    def get_feedback_snapshot(self):
        self.calls.append("feedback")
        if self.feedback_transport_errors_remaining:
            self.feedback_transport_errors_remaining -= 1
            raise StreamResponseTimeoutError(
                "timeout waiting for FEEDBACK_SNAPSHOT sequence=2189 "
                "observed=none"
            )
        return namespace(
            status_code=0,
            joint_count=12,
            present_mask=0x0FFF,
            firmware_tick_ms=self.tick,
            completed_pairs=12,
            positions_urad=self.feedback_positions,
            sample_age_ms=self.feedback_age_ms,
        )

    def get_executor_diagnostics(self):
        self.calls.append("executor")
        return namespace(
            state=self.executor_state,
            terminal_reason=StreamTerminalReasonV2.NONE,
            safe_stop_required=self.executor_safe_stop,
        )

    def arm(self, calibration_hash):
        self.calls.append(("arm", calibration_hash))
        if self.arm_error is not None:
            raise self.arm_error

    def enable(self):
        self.calls.append("enable")
        return namespace(stop_latched=False, status_code=0)

    @staticmethod
    def accepted(epoch=0):
        return namespace(
            status_code=StreamStatusCodeV2.OK,
            contract_result=StreamContractResultV2.OK,
            arm_mask=ARM_MASK_BOTH,
            arbiter_epoch=epoch,
        )

    def open_stream(self, policy):
        self.calls.append("open")
        self.opened = policy
        return self.accepted()

    def append(self, batch):
        self.calls.append("append")
        self.appended.append(batch)
        if self.reject_append:
            return namespace(
                status_code=StreamStatusCodeV2.CONTRACT_REJECTED,
                contract_result=StreamContractResultV2.SAMPLE_DISCONTINUITY,
                arm_mask=ARM_MASK_BOTH,
            )
        return self.accepted(batch.arbiter_epoch)

    def splice(self, batch):
        self.calls.append("splice")
        self.spliced.append(batch)
        return self.accepted(batch.arbiter_epoch)

    def safe_stop(self):
        self.calls.append("safe_stop")
        self.safe_stop_count += 1
        return namespace(stop_latched=True, status_code=0)


def points(*offsets, value=0.0):
    return tuple(
        TimedJointPoint(offset, (value,) * 12)
        for offset in offsets
    )


def adapter(*, motion_authorized=True):
    transport = FakeTransport()
    instance = ResidentBimanualStreamAdapter(
        transport,
        load_operational_limits(LIMITS_PATH),
        motion_authorized=motion_authorized,
    )
    return instance, transport


def test_loads_only_the_approved_full_task_envelope() -> None:
    limits = load_operational_limits(LIMITS_PATH)
    assert limits.joint_names == CANONICAL_JOINT_NAMES
    assert len(limits.minimum_urad) == len(limits.maximum_urad) == 12
    assert limits.source_sha256 == (
        "c1d6d41c402de15c0ac03ceca7c9eeb2d2ffe166dd794599e3fdc8b2db87a48e"
    )


def test_prepare_reports_firmware_shadow_failure_reason() -> None:
    instance, transport = adapter()
    transport.shadow_status = 2

    with pytest.raises(
        BimanualStreamError,
        match="left-arm torque-disable command failed.*STM32 reset is required",
    ):
        instance.prepare()

    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 0


def test_active_feedback_refreshes_heartbeat_before_snapshot() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("feedback_owner", points(80, 130), finite=False)
    instance._last_heartbeat_monotonic = None
    call_start = len(transport.calls)

    instance.feedback_snapshot()

    assert transport.calls[call_start:] == ["heartbeat", "feedback"]


def test_armed_ready_feedback_keeps_watchdog_alive_between_finite_legs() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("fsm", points(80, 130), finite=True)
    transport.executor_state = StreamExecutorStateV2.SUCCEEDED
    instance.poll("fsm")
    assert instance.state is AdapterState.READY
    assert instance.heartbeat_required

    instance._last_heartbeat_monotonic = None
    call_start = len(transport.calls)
    instance.feedback_snapshot()

    assert transport.calls[call_start:] == ["heartbeat", "feedback"]


def test_reorders_an_exact_joint_vector_and_rejects_duplicates() -> None:
    reversed_names = tuple(reversed(CANONICAL_JOINT_NAMES))
    reordered = normalize_joint_positions(reversed_names, tuple(range(12)))
    assert reordered == tuple(reversed(range(12)))
    with pytest.raises(BimanualStreamContractError, match="exactly once"):
        normalize_joint_positions(
            CANONICAL_JOINT_NAMES[:-1] + (CANONICAL_JOINT_NAMES[0],),
            (0.0,) * 12,
        )


def test_prepare_is_read_only_and_requires_exact_f8_identity() -> None:
    instance, transport = adapter()
    prepared = instance.prepare()
    assert instance.state is AdapterState.READY
    assert prepared.positions_urad == (0,) * 12
    assert "arm" not in transport.calls
    assert "enable" not in transport.calls

    bad, bad_transport = adapter()
    original = bad_transport.enter_binary_mode
    bad_transport.enter_binary_mode = lambda: namespace(
        **{**vars(original()), "firmware_version": F8_FIRMWARE_VERSION - 1}
    )
    with pytest.raises(BimanualStreamError, match="unexpected F8 identity"):
        bad.prepare()
    assert bad.state is AdapterState.FAULTED
    assert bad_transport.safe_stop_count == 0


def test_refresh_unarmed_anchor_recaptures_the_current_torque_off_pose() -> None:
    instance, transport = adapter()
    instance.prepare()
    refreshed = (0, 320_000) + (0,) * 10
    transport.shadow_positions_urad = refreshed

    prepared = instance.refresh_unarmed_anchor()

    assert prepared.positions_urad == refreshed
    assert instance.prepared_state.positions_urad == refreshed
    assert transport.calls.count("shadow") == 2
    assert not any(
        isinstance(call, tuple) and call[0] == "arm"
        for call in transport.calls
    )


def test_first_start_rejects_a_route_based_on_a_stale_anchor_before_arm() -> None:
    instance, transport = adapter()
    instance.prepare()
    transport.shadow_positions_urad = (0, 320_000) + (0,) * 10

    with pytest.raises(
        BimanualStreamContractError,
        match="joint step exceeds host policy: joint=left_shoulder_joint",
    ):
        instance.start("policy", points(80, 130), finite=True)

    assert instance.state is AdapterState.READY
    assert instance.owner is None
    assert instance.epoch == 0
    assert transport.calls.count("shadow") == 2
    assert "enable" not in transport.calls
    assert transport.safe_stop_count == 0
    assert not any(
        isinstance(call, tuple) and call[0] == "arm"
        for call in transport.calls
    )


def test_feedback_snapshot_requires_all_twelve_freshness_slots() -> None:
    instance, transport = adapter()
    instance.prepare()
    snapshot = instance.feedback_snapshot()
    assert snapshot.present_mask == 0x0FFF
    assert snapshot.positions_urad == (0,) * 12
    assert snapshot.sample_age_ms == tuple(range(12))
    assert transport.calls[-1] == "feedback"

    transport.get_feedback_snapshot = lambda: namespace(
        status_code=1,
        joint_count=12,
        present_mask=0x003F,
        positions_urad=(0,) * 12,
        sample_age_ms=(0,) * 12,
    )
    with pytest.raises(BimanualStreamError, match="incomplete 12-axis"):
        instance.feedback_snapshot()
    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 0


def test_non_timeout_feedback_transport_error_faults_immediately() -> None:
    instance, transport = adapter()
    instance.prepare()
    transport.get_feedback_snapshot = lambda: (_ for _ in ()).throw(
        StreamTransportV2Error("feedback snapshot sequence echo mismatch")
    )

    with pytest.raises(BimanualStreamError, match="sequence echo mismatch"):
        instance.feedback_snapshot()

    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 0


def test_ready_feedback_tolerates_two_transport_timeouts_and_recovers() -> None:
    instance, transport = adapter()
    instance.prepare()
    transport.feedback_transport_errors_remaining = 2

    for streak in (1, 2):
        with pytest.raises(
            BimanualTransientFeedbackError,
            match=rf"transient feedback transport delay \({streak}/3\)",
        ):
            instance.feedback_snapshot()
        assert instance.state is AdapterState.READY
        assert transport.safe_stop_count == 0

    snapshot = instance.feedback_snapshot()
    assert snapshot.present_mask == 0x0FFF
    assert instance.state is AdapterState.READY
    assert instance._feedback_transport_failure_streak == 0


def test_active_feedback_faults_after_three_consecutive_transport_timeouts() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("feedback_owner", points(80, 130), finite=False)
    transport.feedback_transport_errors_remaining = 3

    for _streak in (1, 2):
        with pytest.raises(BimanualTransientFeedbackError):
            instance.feedback_snapshot()
        assert instance.state is AdapterState.ACTIVE
        assert transport.safe_stop_count == 0

    with pytest.raises(
        BimanualStreamError,
        match=r"feedback transport failed consecutively \(3/3\)",
    ):
        instance.feedback_snapshot()
    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 1


def test_motion_requires_explicit_authorization() -> None:
    instance, transport = adapter(motion_authorized=False)
    instance.prepare()
    with pytest.raises(BimanualStreamError, match="motion_authorized"):
        instance.start("policy", points(80, 130, 180), finite=False)
    assert not any(
        isinstance(call, tuple) and call[0] == "arm"
        for call in transport.calls
    )


def test_finite_start_maps_relative_times_and_uses_one_epoch() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("moveit", points(80, 130, 180), finite=True)
    assert instance.state is AdapterState.ACTIVE
    assert instance.owner == "moveit"
    assert instance.epoch == 1
    assert transport.opened.horizon_end_tick == 1180
    assert transport.opened.command_timeout_ms == 500
    batch = transport.appended[-1]
    assert batch.arbiter_epoch == 1
    assert batch.horizon_end_tick == 1180
    assert tuple(sample.apply_tick for sample in batch.samples) == (1080, 1130, 1180)
    assert all(sample.positions_urad == (0,) * 12 for sample in batch.samples)
    arm_index = transport.calls.index(("arm", CALIBRATION_HASH))
    enable_index = transport.calls.index("enable")
    assert "heartbeat" in transport.calls[arm_index + 1:enable_index]


def test_long_finite_plan_is_fed_in_bounded_wire_batches() -> None:
    instance, transport = adapter()
    instance.prepare()
    route = points(*range(50, 1050, 50))

    instance.start("moveit", route, finite=True)

    assert transport.opened.horizon_end_tick == 2000
    assert tuple(
        sample.apply_tick for sample in transport.appended[0].samples
    ) == tuple(range(1050, 1450, 50))
    assert transport.appended[0].horizon_end_tick == 2000
    assert len(instance._pending_finite_samples) == 12

    transport.tick = 1200
    instance._pump_finite_plan()

    assert tuple(
        sample.apply_tick for sample in transport.appended[1].samples
    ) == (1450, 1500, 1550, 1600)
    assert transport.appended[1].horizon_end_tick == 2000
    assert len(instance._pending_finite_samples) == 8


def test_invalid_range_or_time_is_rejected_before_arm() -> None:
    instance, transport = adapter()
    instance.prepare()
    bad_position = (2.0,) + (0.0,) * 11
    with pytest.raises(BimanualStreamContractError, match="outside approved"):
        instance.start(
            "policy",
            (
                TimedJointPoint(80, bad_position),
                TimedJointPoint(130, bad_position),
            ),
            finite=False,
        )
    assert not any(
        isinstance(call, tuple) and call[0] == "arm"
        for call in transport.calls
    )

    fresh, fresh_transport = adapter()
    fresh.prepare()
    with pytest.raises(BimanualStreamContractError, match="multiples of 5"):
        fresh.start("policy", points(81, 130), finite=False)
    assert not any(
        isinstance(call, tuple) and call[0] == "arm"
        for call in fresh_transport.calls
    )


def test_open_stream_append_and_splice_preserve_owner_and_epoch() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130, 180), finite=False)
    instance.append("policy", points(200, 250))
    assert transport.appended[-1].arbiter_epoch == 1
    assert tuple(
        sample.apply_tick for sample in transport.appended[-1].samples
    ) == (1200, 1250)

    transport.tick = 1100
    instance.splice("policy", points(150), splice_offset_ms=100)
    replacement = transport.spliced[-1]
    assert instance.epoch == 2
    assert replacement.arbiter_epoch == 2
    assert replacement.splice_at_tick == 1200
    assert tuple(sample.apply_tick for sample in replacement.samples) == (1200, 1250)

    with pytest.raises(BimanualStreamOwnershipError, match="owned by"):
        instance.append("moveit", points(200, 250))


def test_append_and_splice_align_to_the_existing_five_ms_grid() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130, 180), finite=False)

    transport.tick = 1003
    instance.append("policy", points(200, 250))
    assert tuple(
        sample.apply_tick for sample in transport.appended[-1].samples
    ) == (1205, 1255)

    transport.tick = 1102
    instance.splice("policy", points(150), splice_offset_ms=100)
    assert tuple(
        sample.apply_tick for sample in transport.spliced[-1].samples
    ) == (1205, 1255)


def test_splice_synthesizes_interpolated_continuity_point() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start(
        "policy",
        (
            TimedJointPoint(80, (0.0,) * 12),
            TimedJointPoint(130, (0.045,) * 12),
            TimedJointPoint(180, (0.09,) * 12),
        ),
        finite=False,
    )
    transport.tick = 1002
    instance.splice(
        "policy",
        (TimedJointPoint(150, (0.05,) * 12),),
        splice_offset_ms=100,
    )
    replacement = transport.spliced[-1]
    assert replacement.splice_at_tick == 1105
    assert tuple(sample.apply_tick for sample in replacement.samples) == (
        1105,
        1155,
    )
    assert replacement.samples[0].positions_urad == (22_500,) * 12
    assert replacement.samples[1].positions_urad == (50_000,) * 12


def test_splice_rejects_targets_at_or_before_the_splice_point() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130, 180), finite=False)
    with pytest.raises(BimanualStreamContractError, match="must occur after"):
        instance.splice("policy", points(100), splice_offset_ms=100)
    assert not transport.spliced


def test_ambiguous_arm_transport_failure_attempts_safe_stop() -> None:
    instance, transport = adapter()
    instance.prepare()
    transport.arm_error = RuntimeError("arm response timeout")
    with pytest.raises(BimanualStreamError, match="arm response timeout"):
        instance.start("policy", points(80, 130), finite=False)
    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 1


def test_rejected_motion_batch_latches_safe_stop() -> None:
    instance, transport = adapter()
    instance.prepare()
    transport.reject_append = True
    with pytest.raises(BimanualStreamError, match="initial append rejected"):
        instance.start("policy", points(80, 130), finite=False)
    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 1


def test_finite_terminal_reanchors_and_allows_next_resident_leg() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("fsm", points(80, 130), finite=True)
    transport.executor_state = StreamExecutorStateV2.SUCCEEDED
    transport.tracking_pending = True
    instance.poll("fsm")
    assert instance.state is AdapterState.ACTIVE
    assert transport.calls.count("shadow") == 2

    transport.tracking_pending = False
    call_start = len(transport.calls)
    instance.poll("fsm")
    assert instance.state is AdapterState.READY
    assert transport.safe_stop_count == 0
    assert transport.calls.count("shadow") == 2
    terminal_calls = transport.calls[call_start:]
    assert terminal_calls.index("feedback") < terminal_calls.index("dispatch")
    assert terminal_calls.index("feedback") < terminal_calls.index("tracking")

    transport.executor_state = StreamExecutorStateV2.RUNNING
    instance.start("fsm", points(80, 130), finite=True)
    assert instance.epoch == 2
    assert transport.calls.count("enable") == 1
    assert len([call for call in transport.calls if isinstance(call, tuple)]) == 1


def test_finite_terminal_requires_fresh_measured_target() -> None:
    instance, transport = adapter()
    instance.prepare()
    target = 0.03
    instance.start("fsm", points(80, 130, value=target), finite=True)
    transport.executor_state = StreamExecutorStateV2.SUCCEEDED
    transport.feedback_positions = (30_000,) * 12
    transport.feedback_age_ms = (20,) * 12

    instance.poll("fsm")

    assert instance.state is AdapterState.READY
    assert instance._prepared.positions_urad == (30_000,) * 12
    assert instance.prepared_state.positions_urad == (30_000,) * 12


def test_finite_terminal_accepts_contact_residual_only_on_grippers() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("fsm", points(80, 130, value=0.03), finite=True)
    transport.executor_state = StreamExecutorStateV2.SUCCEEDED
    transport.feedback_positions = (
        (30_000,) * 5
        + (80_000,)
        + (30_000,) * 5
        + (80_000,)
    )
    transport.feedback_age_ms = (20,) * 12

    instance.poll("fsm")

    assert instance.state is AdapterState.READY
    assert instance._prepared.positions_urad[5] == 80_000
    assert instance._prepared.positions_urad[11] == 80_000


@pytest.mark.parametrize(
    ("positions", "ages", "message"),
    (
        ((100_000,) + (30_000,) * 11, (20,) * 12, "terminal settle error"),
        ((30_000,) * 12, (151,) + (20,) * 11, "terminal feedback is stale"),
    ),
)
def test_finite_terminal_feedback_failure_stops_both_arms(
    positions, ages, message
) -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("fsm", points(80, 130, value=0.03), finite=True)
    transport.executor_state = StreamExecutorStateV2.SUCCEEDED
    transport.feedback_positions = positions
    transport.feedback_age_ms = ages

    with pytest.raises(BimanualStreamError, match=message):
        instance.poll("fsm")

    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 1


def test_poll_fails_closed_on_dispatch_fault() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130), finite=False)
    transport.dispatch_failures = 1
    with pytest.raises(BimanualStreamError, match="dispatch fault") as caught:
        instance.poll("policy")
    assert instance.state is AdapterState.FAULTED
    assert transport.safe_stop_count == 1
    assert "post_stop_diagnostics: executor=" in str(caught.value)
    assert instance.fault_diagnostic == str(caught.value)


def test_poll_keeps_running_after_recovered_tracking_read_failure() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130, 180), finite=False)
    transport.dispatch_active = True
    transport.tracking_active = True
    transport.tracking_failures = 1

    instance.poll("policy")

    assert instance.state is AdapterState.ACTIVE
    assert transport.safe_stop_count == 0



def test_poll_accepts_a_normal_in_flight_dispatch() -> None:
    instance, transport = adapter()
    instance.prepare()
    instance.start("policy", points(80, 130, 180), finite=False)
    transport.dispatch_active = True

    instance.poll("policy")

    assert instance.state is AdapterState.ACTIVE
    assert transport.safe_stop_count == 0
