"""The 0x23E00 executor candidate advances state but cannot move a servo."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text()
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()
PROTOCOL = (ROOT / "firmware/stm32_actuator/src/protocol.c").read_text()
TOOL = (ROOT / "tools/contract_evidence/validate_protocol_v2_executor_no_motion.py").read_text()


def _function_body(name: str, next_name: str) -> str:
    return BINARY.split(f"static void {name}", 1)[1].split(
        f"static void {next_name}", 1
    )[0]


def test_executor_candidate_has_unique_identity_and_capability() -> None:
    assert "PROTOCOL_V2_EXECUTOR_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00023E00UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x03802000UL" in CMAKE
    assert "ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U" in CMAKE
    assert "HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U" in CMAKE
    assert "HOST_PROTOCOL_V2_EXECUTOR_CAPABILITY UINT32_C(0x02000000)" in CONFIG


def test_executor_output_is_discarded_without_any_servo_call() -> None:
    service = _function_body("Host_ServiceV2Executor", "Host_SendV2ExecutorDiagnostics")
    assert "host_v2_discarded_output_urad" in service
    for forbidden in ("Servo_", "RightServoBus_", "SyncWrite", "Torque"):
        assert forbidden not in service
    assert "HOST_F25_VALIDATION_ONLY_BUILD=1U" in CMAKE
    assert "case ACTUATOR_MSG_ARM_REQUEST:" in BINARY
    assert "case ACTUATOR_MSG_ENABLE:" in BINARY


def test_executor_diagnostics_messages_are_candidate_only() -> None:
    assert "ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES" in PROTOCOL
    assert "ACTUATOR_V2_MSG_GET_EXECUTOR_DIAGNOSTICS" in BINARY
    assert "ACTUATOR_V2_MSG_EXECUTOR_DIAGNOSTICS" in BINARY
    assert "ACTUATOR_V2_EXECUTOR_DIAGNOSTICS_WIRE_SIZE" in BINARY


def test_hardware_tool_requires_both_rails_off_and_no_motion() -> None:
    assert "PROTOCOL_V2_EXECUTOR_NO_MOTION_BOTH_SERVO_12V_OFF" in TOOL
    assert '"motion_authorized": False' in TOOL
    assert '"synthetic_anchor": True' in TOOL
    assert '"discarded_executor_output": True' in TOOL
    for forbidden in (
        ".arm_and_enable(",
        ".enable(",
        ".hold(",
        ".safe_stop(",
        ".disable(",
        ".clear_fault(",
    ):
        assert forbidden not in TOOL


def test_open_timeout_probe_stays_within_firmware_hard_cap() -> None:
    assert "command_timeout_ms=100" in TOOL
    assert "for offset in (70, 90)" in TOOL
    assert "command_timeout_ms=180" not in TOOL
