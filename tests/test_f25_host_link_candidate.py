"""F2.5 921600-baud entry gate stays isolated and motion-free."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
MAIN = (FIRMWARE / "Core/Src/main.c").read_text()
BINARY_CONTROL = (FIRMWARE / "Core/Src/binary_control.c").read_text()
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text()
BRIDGE_IDENTITY = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
).read_text()
TOOL_PATH = ROOT / "tools/stress_f25_host_link_no_motion.py"
SPEC = importlib.util.spec_from_file_location("f25_host_link", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)
TOOL_SOURCE = TOOL_PATH.read_text()


def test_default_r4_build_identity_and_baud_are_unchanged() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x007FFFFF)" in CONFIG
    assert "HOST_BINARY_UART_BAUD UINT32_C(115200)" in CONFIG
    assert "hlpuart1.Init.BaudRate = HOST_BINARY_UART_BAUD;" in MAIN


def test_candidate_is_a_separate_cmake_build_identity() -> None:
    assert "option(\n    F25_HOST_LINK_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00023C01UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x00802408UL" in CMAKE
    assert "HOST_BINARY_UART_BAUD=921600UL" in CMAKE
    assert "HOST_F25_VALIDATION_ONLY_BUILD=1U" in CMAKE
    assert "HOST_F25_HIGH_BAUD_VALIDATION_CAPABILITY UINT32_C(0x00800000)" in CONFIG


def test_candidate_router_rejects_torque_and_motion_requests() -> None:
    assert "#if HOST_F25_VALIDATION_ONLY_BUILD" in BINARY_CONTROL
    assert "Host_F25RejectsRequest(request)" in BINARY_CONTROL
    assert "ACTUATOR_MSG_ARM_REQUEST" in BINARY_CONTROL
    assert "ACTUATOR_MSG_ENABLE" in BINARY_CONTROL
    assert "ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_REQUEST" in BINARY_CONTROL
    assert "ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST" in BINARY_CONTROL
    assert "ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_REQUEST" in BINARY_CONTROL
    assert "ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY" in BINARY_CONTROL
    assert "Host_SendBinaryState(request->sequence, 8U);" in BINARY_CONTROL


def test_normal_bridge_does_not_accept_candidate_identity() -> None:
    assert "0x00023C01" not in BRIDGE_IDENTITY


def test_stress_load_is_derived_and_exceeds_two_times_planned_load() -> None:
    request_bytes, response_bytes = TOOL.representative_wire_bytes(9)
    assert request_bytes > 400
    assert response_bytes > 100
    stress_bps = (request_bytes + response_bytes) * (
        1000.0 / TOOL.DEFAULT_INTERVAL_MS
    )
    assert stress_bps >= (
        TOOL.planned_worst_case_wire_bps()
        * TOOL.WIRE_TRAFFIC_MULTIPLIER_GATE
    )


def test_stress_tool_has_only_no_motion_protocol_calls() -> None:
    assert TOOL.CONFIRMATION.endswith("SERVO_12V_OFF")
    assert TOOL.EXPECTED_FIRMWARE_VERSION == 0x00023C01
    assert TOOL.EXPECTED_CAPABILITIES == 0x00802408
    assert TOOL.HOST_BAUD == 921_600
    assert "hello.capabilities != EXPECTED_CAPABILITIES" in TOOL_SOURCE
    assert '"validation_only": True' in TOOL_SOURCE
    assert '"motion_authorized": False' in TOOL_SOURCE
    assert "validate_buffered_candidate" in TOOL_SOURCE
    for forbidden in (
        ".arm_and_enable(",
        ".enable(",
        ".hold(",
        ".safe_stop(",
        ".disable(",
        ".clear_fault(",
        "exchange_buffered_command",
        "send_goal_async",
        "create_publisher",
    ):
        assert forbidden not in TOOL_SOURCE


def test_compile_time_overrides_are_guarded_in_header() -> None:
    for name in (
        "HOST_BINARY_FIRMWARE_VERSION",
        "HOST_BINARY_CAPABILITIES",
        "HOST_BINARY_UART_BAUD",
    ):
        pattern = rf"#ifndef {name}\s+#define {name}"
        assert re.search(pattern, CONFIG)
