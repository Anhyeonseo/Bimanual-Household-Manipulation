"""The J1-W candidate binds real raw feedback to explicit branches only."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text()
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()


def function_body(name: str, next_name: str) -> str:
    return BINARY.split(f"static void {name}", 1)[1].split(
        f"static void {next_name}", 1
    )[0]


def test_candidate_has_unique_no_output_identity() -> None:
    assert "PROTOCOL_V2_UNWRAP_SHADOW_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024000UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x0F802000UL" in CMAKE
    assert "HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U" in CMAKE
    assert "HOST_PROTOCOL_V2_UNWRAP_SHADOW_CAPABILITY" in CONFIG


def test_prepare_requires_explicit_branch_references() -> None:
    route = BINARY.split("case ACTUATOR_V2_MSG_PREPARE_SHADOW:", 1)[1]
    assert "request->payload_length == 52U" in route
    body = function_body("Host_SendV2ShadowSnapshot", "Host_HandleBinaryFrame")
    assert "maximum_reference_delta_raw" in body
    assert "reference_unwrapped_raw" in body
    assert "actuator_joint_unwrapper_bind" in body
    assert "ACTUATOR_UNWRAP_HALF_TURN_RAW" in body
    assert "host_v2_shadow_unwrapped_raw" in body
    assert "request->payload_length == 0U" in route
    assert "actuator_joint_unwrapper_update" in body
    assert "host_v2_shadow_unwrappers" in body
    assert body.index("candidate_unwrappers") < body.index(
        "host_v2_shadow_unwrappers,"
    )


def test_unwrapped_executor_uses_synthetic_no_output_limits_only() -> None:
    limits = BINARY.split("static void Host_V2JointLimits", 1)[1].split(
        "static actuator_v2_stream_session_result_t", 1
    )[0]
    assert "HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD" in limits
    assert "limits[joint].minimum_urad = -6400000" in limits
    assert "limits[joint].maximum_urad = 6400000" in limits
    assert "no value reaches either servo bus" in limits


def test_candidate_still_verifies_torque_off_and_discards_output() -> None:
    body = function_body("Host_SendV2ShadowSnapshot", "Host_HandleBinaryFrame")
    assert body.index("Servo_DisableTorqueAll()") < body.index(
        "Servo_ReadAllPositions(left_raw)"
    )
    assert body.index("RightServoBus_DisableTorqueAllVerified()") < body.index(
        "RightServoBus_Discover()"
    )
    service = function_body(
        "Host_ServiceV2Executor", "Host_SendV2ExecutorDiagnostics"
    )
    assert "host_v2_discarded_output_urad" in service
    for forbidden in ("Servo_", "RightServoBus_", "SyncWrite", "Torque"):
        assert forbidden not in service
