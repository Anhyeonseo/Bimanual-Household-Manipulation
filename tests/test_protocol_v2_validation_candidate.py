"""The first wire-connected protocol-v2 candidate remains motion-free."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text()
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()
PROTOCOL = (ROOT / "firmware/stm32_actuator/src/protocol.c").read_text()
TOOL = (ROOT / "tools/validate_protocol_v2_no_motion.py").read_text()
EXECUTOR_V2 = (
    ROOT / "firmware/stm32_actuator/src/stream_executor_v2.c"
).read_text().lower()
EXECUTOR_V2_HEADER = (
    ROOT
    / "firmware/stm32_actuator/include/actuator_core/stream_executor_v2.h"
).read_text().lower()


def test_candidate_has_unique_protocol_v2_identity() -> None:
    assert "PROTOCOL_V2_VALIDATION_CANDIDATE" in CMAKE
    assert "ACTUATOR_PROTOCOL_VERSION=2U" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00023D00UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x01802000UL" in CMAKE
    assert "HOST_BINARY_JOINT_COUNT=12U" in CMAKE
    assert "HOST_BINARY_UART_BAUD=921600UL" in CMAKE
    assert "HOST_PROTOCOL_V2_VALIDATION_CAPABILITY UINT32_C(0x01000000)" in CONFIG


def test_candidate_messages_are_compile_time_isolated_from_r4() -> None:
    assert "ACTUATOR_ENABLE_STREAM_V2_MESSAGES" in PROTOCOL
    assert "#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD" in BINARY
    assert "Host_ValidateV2StreamOpen" in BINARY
    assert "Host_ValidateV2Batch" in BINARY
    assert "ACTUATOR_V2_MSG_SPLICE" in BINARY


def test_candidate_stream_status_proves_no_executable_output() -> None:
    assert "ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY" in BINARY
    assert "Host_WriteU32Le(&response.payload[24], 0U);" in BINARY
    assert "Host_WriteU32Le(&response.payload[28], 0U);" in BINARY
    assert "Host_WriteU32Le(&response.payload[32], 0U);" in BINARY
    assert "both servo 12 V domains must be off" in TOOL
    assert '"motion_authorized": False' in TOOL
    for forbidden in (
        ".arm_and_enable(",
        ".enable(",
        ".hold(",
        ".safe_stop(",
        ".disable(",
        ".clear_fault(",
    ):
        assert forbidden not in TOOL


def test_default_r4_values_remain_present() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x007FFFFF)" in CONFIG
    assert "HOST_BINARY_UART_BAUD UINT32_C(115200)" in CONFIG


def test_v2_executor_has_no_upstream_command_source_concepts() -> None:
    executor_text = EXECUTOR_V2 + EXECUTOR_V2_HEADER
    for forbidden in (
        "moveit",
        "pretrained",
        "inference",
        "residual",
        "track_a",
        "track_b",
        "rl_policy",
    ):
        assert forbidden not in executor_text
    assert "actuator_v2_joint_count" in executor_text
    assert "horizon_end_tick" in executor_text
    assert "stream_session_v2" in executor_text
