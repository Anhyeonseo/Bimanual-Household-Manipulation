from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STM32 = ROOT / "firmware/stm32_g474_single_arm"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_resident_finite_candidate_has_isolated_identity() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    assert "BIMANUAL_RESIDENT_FINITE_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024703UL" in cmake
    assert "HOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U" in cmake
    assert "BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024800UL" in cmake
    assert "BIMANUAL_TERMINAL_SETTLE_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024806UL" in cmake
    assert "HOST_BIMANUAL_TERMINAL_SETTLE_BUILD=1U" in cmake
    assert "BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024809UL" in cmake
    assert "HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD=1U" in cmake


def test_finite_success_ends_tracking_only_after_final_pair() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    service = source[
        source.index("static void Host_ServiceV2TrackingFeedback"):
        source.index("static void Host_ServiceV2Executor")
    ]
    assert "#if HOST_BIMANUAL_RESIDENT_FINITE_BUILD" in service
    end = service.rindex("BimanualTrackingFeedback_End();")
    for required in (
        "host_v2_executor_clock_active == 0U",
        "ACTUATOR_V2_EXECUTOR_SUCCEEDED",
        "tracking->pending == 0U",
        "tracking->requested_pairs == tracking->completed_pairs",
        "dispatch->launch_count == dispatch->completed_count",
        "host_v2_tracking_last_dispatch_completed",
        "host_v2_shadow_executor_anchor_urad",
        "host_v2_output_urad",
        "host_v2_shadow_anchor_ready = 1U",
    ):
        assert required in service[:end]


def test_terminal_settle_requires_twelve_consecutive_fresh_pairs() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    service = source[
        source.index("static void Host_ServiceV2TrackingFeedback"):
        source.index("static void Host_ServiceV2Executor")
    ]
    for required in (
        "HOST_BIMANUAL_TERMINAL_SETTLE_BUILD",
        "HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS",
        "HOST_BIMANUAL_TERMINAL_SETTLE_ARM_TOLERANCE_URAD",
        "HOST_BIMANUAL_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD",
        "host_v2_terminal_settle_baseline_completed_pairs",
        "Host_V2TrackingCompletedPairs",
        "BimanualFeedbackSnapshot_Copy",
        "snapshot.positions_urad",
    ):
        assert required in service or required in config
    assert "UINT8_C(12)" in config
    assert "INT32_C(46020)" in config
    assert "INT32_C(150000)" in config
    assert "INT32_C(160000)" in config
    assert "sample.joint_index == (SINGLE_ARM_JOINT_COUNT - 1U)" in service
    assert "Host_V2TrackingCompletedPairs() -" in service
    assert "host_v2_terminal_settle_active == 0U" in service
    assert (
        "((host_v2_terminal_settle_active == 0U) &&\n"
        "           (dispatch->completed_count >\n"
        "            host_v2_tracking_last_dispatch_completed))"
    ) in service
    assert (
        "host_v2_terminal_settle_baseline_completed_pairs) >=\n"
        "             HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS"
    ) in service



def test_f89_gripper_contact_tracking_cap_is_axis_specific() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    hard_caps = source[
        source.index("static actuator_v2_stream_hard_caps_t Host_V2HardCaps"):
        source.index("static void Host_V2JointLimits")
    ]
    assert "HOST_BIMANUAL_GRIPPER_TRACKING_HARD_CAP_URAD" in hard_caps
    assert "joint == (SINGLE_ARM_JOINT_COUNT - 1U)" in hard_caps
    assert "joint == (ACTUATOR_V2_JOINT_COUNT - 1U)" in hard_caps
    assert "INT32_C(160000)" in config
    assert "INT32_C(100000)" in hard_caps

def test_resident_completion_does_not_disable_torque_to_reanchor() -> None:
    adapter = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_adapter.py"
    )
    poll = adapter[adapter.index("    def poll("):adapter.index("    def stop(")]
    assert "self._tail_positions_urad" in poll
    assert "prepare_shadow" not in poll


def test_resident_host_contract_requires_finite_completion_firmware() -> None:
    adapter = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_adapter.py"
    )
    node = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_node.py"
    )
    no_motion = text(
        ROOT / "tools/validate_resident_bimanual_adapter_no_motion.py"
    )
    hold = text(
        ROOT / "tools/execute_resident_bimanual_current_pose_hold_twice.py"
    )
    roundtrip = text(
        ROOT
        / "tools/execute_resident_bimanual_base_small_roundtrip_once.py"
    )
    rolling = text(
        ROOT
        / "tools/execute_resident_bimanual_rolling_horizon_no_motion_once.py"
    )
    rolling_motion = text(
        ROOT
        / "tools/"
        "execute_resident_bimanual_rolling_base_small_roundtrip_once.py"
    )
    assert "F8_FIRMWARE_VERSION = 0x00024809" in adapter
    assert "F8_FIRMWARE_VERSION" in node
    for source in (no_motion, hold, roundtrip, rolling, rolling_motion):
        assert "0x00024809" in source
    assert '"~/refresh_anchor"' in node
    assert "refresh_unarmed_anchor" in node
    assert "prepared_positions_rad" in node
    assert "prepared_epoch" in node
    assert "torque_hold_active" in node
    assert "prepared = adapter.prepared_state" in node
    assert "BASE_DELTA_RAD = 0.03" in roundtrip
    assert "BASE_INDICES = (0, 6)" in roundtrip
    assert "leg_routes" in roundtrip
    assert "BimanualStreamCommand.Request.STOP" in roundtrip
    for operation in ("START_OPEN", "APPEND", "SPLICE", "STOP"):
        assert operation in rolling
        assert operation in rolling_motion
    assert "KEEPALIVE_COUNT = 3" in rolling
    assert "ready_soak_s" in hold
    assert "time.sleep(args.ready_soak_s)" in hold
    assert 'ready_soak_status.get("torque_hold_active") is not True' in hold
    assert "BASE_DELTA_RAD = 0.03" in rolling_motion
    assert "BASE_INDICES = (0, 6)" in rolling_motion
    assert "splice_offset_ms=splice_offset_ms" in rolling_motion


def test_resident_node_owns_an_independent_armed_keepalive() -> None:
    node = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_node.py"
    )
    adapter = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_adapter.py"
    )
    assert 'self.declare_parameter("heartbeat_period_s", 0.1)' in node
    assert 'self.declare_parameter("response_timeout_s", 0.12)' in node
    assert 'self._heartbeat_timer = self.create_timer(' in node
    status = node[
        node.index("    def _on_status"):
        node.index("    def _publish_feedback")
    ]
    assert "adapter.keepalive()" in status
    assert "def heartbeat_required" in adapter
    heartbeat_contract = adapter[
        adapter.index("def heartbeat_required"):
        adapter.index("    def _fault")
    ]
    assert "AdapterState.READY" in heartbeat_contract
    assert "post_stop_diagnostics" in adapter
    assert "def prepared_state" in adapter
    assert "def _publish_anchor" in node
    assert "adapter.prepared_state" in node
    assert '"fault_diagnostic": adapter.fault_diagnostic' in node


def test_resident_node_prioritizes_armed_heartbeat_over_feedback() -> None:
    node = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_node.py"
    )
    feedback = node[
        node.index("    def _publish_feedback"):
        node.index("    def _poll_active")
    ]
    armed_guard = "if adapter.heartbeat_required:"
    assert armed_guard in feedback
    assert feedback.index(armed_guard) < feedback.index(
        "snapshot = adapter.feedback_snapshot()"
    )


def test_resident_splice_api_synthesizes_continuity_in_adapter() -> None:
    service = text(
        ROOT / "ros2_ws/src/so101_interfaces/srv/BimanualStreamCommand.srv"
    )
    adapter = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_adapter.py"
    )
    node = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_node.py"
    )
    class_start = adapter.index("class ResidentBimanualStreamAdapter")
    splice = adapter[
        adapter.index("    def splice(", class_start):
        adapter.index("    def poll(", class_start)
    ]
    assert "uint32 splice_offset_ms" in service
    assert "_interpolate_route" in splice
    assert "StreamSampleV2(splice_tick, continuity_positions)" in splice
    assert "splice_offset_ms=int(request.splice_offset_ms)" in node
