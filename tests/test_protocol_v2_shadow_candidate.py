"""The 0x23F01 shadow candidate reads feedback but emits no goal output."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text()
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()
TOOL = (ROOT / "tools/validate_protocol_v2_shadow_no_output.py").read_text()


def function_body(name: str, next_name: str) -> str:
    return BINARY.split(f"static void {name}", 1)[1].split(
        f"static void {next_name}", 1
    )[0]


def test_shadow_candidate_has_unique_fail_closed_identity() -> None:
    assert "PROTOCOL_V2_SHADOW_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00023F01UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x07802000UL" in CMAKE
    assert "HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD=1U" in CMAKE
    assert "HOST_PROTOCOL_V2_SHADOW_CAPABILITY UINT32_C(0x04000000)" in CONFIG


def test_shadow_snapshot_verifies_disable_before_feedback() -> None:
    body = function_body("Host_SendV2ShadowSnapshot", "Host_HandleBinaryFrame")
    left_disable = body.index("Servo_DisableTorqueAll()")
    right_disable = body.index("RightServoBus_DisableTorqueAllVerified()")
    left_read = body.index("Servo_ReadAllPositions(left_raw)")
    right_read = body.index("RightServoBus_Discover()")
    assert left_disable < left_read
    assert right_disable < right_read
    assert "Host_ShadowFeedbackRawToUrad" in body
    assert "host_v2_shadow_anchor_ready = 1U" in body


def test_shadow_feedback_margin_does_not_expand_command_limits() -> None:
    helper = BINARY.split(
        "static actuator_calibration_result_t Host_ShadowFeedbackRawToUrad", 1
    )[1].split("static void Host_SendV2ShadowSnapshot", 1)[0]
    assert "HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW" in CONFIG
    assert "UINT16_C(30)" in CONFIG
    assert "feedback_calibration.minimum_raw" in helper
    assert "feedback_calibration.maximum_raw" in helper
    assert "actuator_raw_to_urad" in helper
    limits = BINARY.split("static void Host_V2JointLimits", 1)[1].split(
        "static actuator_v2_stream_session_result_t "
        "Host_V2ExecutorSessionResult",
        1,
    )[0]
    assert "HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW" not in limits
    body = function_body("Host_SendV2ShadowSnapshot", "Host_HandleBinaryFrame")
    assert "host_v2_shadow_anchor_urad[joint]" in body
    assert "host_v2_shadow_executor_anchor_urad[joint]" in body
    assert "executor_raw = calibration.minimum_raw" in body
    assert "executor_raw = calibration.maximum_raw" in body


def test_shadow_executor_output_remains_discarded() -> None:
    service = function_body(
        "Host_ServiceV2Executor", "Host_SendV2ExecutorDiagnostics"
    )
    assert "host_v2_discarded_output_urad" in service
    for forbidden in ("Servo_", "RightServoBus_", "SyncWrite", "Torque"):
        assert forbidden not in service


def test_shadow_tool_exposes_no_motion_api() -> None:
    assert "PROTOCOL_V2_SHADOW_BOTH_ARMS_TORQUE_OFF_NO_GOAL_OUTPUT" in TOOL
    assert '"motion_authorized": False' in TOOL
    assert '"executor_goal_output_connected": False' in TOOL
    assert "independently_computed_anchor" in TOOL
    for forbidden in (
        ".arm_and_enable(",
        ".enable(",
        ".hold(",
        ".safe_stop(",
        ".disable(",
        ".clear_fault(",
    ):
        assert forbidden not in TOOL
