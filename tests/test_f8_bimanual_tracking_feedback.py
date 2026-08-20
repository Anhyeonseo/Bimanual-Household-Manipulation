from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STM32 = ROOT / "firmware/stm32_g474_single_arm"
ACTUATOR = ROOT / "firmware/stm32_actuator"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_f8_has_isolated_identity_and_right_rx_dma() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    main = text(STM32 / "Core/Src/main.c")
    msp = text(STM32 / "Core/Src/stm32g4xx_hal_msp.c")
    interrupts = text(STM32 / "Core/Src/stm32g4xx_it.c")
    assert "BIMANUAL_TRACKING_FEEDBACK_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024700UL" in cmake
    assert "HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U" in cmake
    assert "DMA_HandleTypeDef hdma_uart4_rx" in main
    assert "DMA1_Channel5" in msp
    assert "DMA_REQUEST_UART4_RX" in msp
    assert "hdma_uart4_rx.Init.Mode = DMA_CIRCULAR" in msp
    assert "HAL_DMA_IRQHandler(&hdma_uart4_rx)" in interrupts


def test_f8_right_sampler_reuses_shared_parser_and_is_nonblocking() -> None:
    source = text(STM32 / "Core/Src/right_servo_bus.c")
    header = text(STM32 / "Core/Inc/right_servo_bus.h")
    tracking = source[source.index("HAL_StatusTypeDef RightServoBus_InMotionTelemetryBegin") :]
    assert '#include "servo_rx_window.h"' in source
    assert "HAL_UARTEx_ReceiveToIdle_DMA(" in tracking
    assert "HAL_UART_Transmit_IT(" in tracking
    assert "ServoRxWindow_Consume(" in tracking
    assert "RIGHT_SERVO_TELEMETRY_TIMEOUT_MS UINT32_C(4)" in source
    assert "HAL_UART_Receive(" not in tracking
    assert "RightServoBus_InMotionTelemetryEnd" in header
    assert "right_servo_telemetry.enabled != 0U" in source


def test_f8_pairs_left_and_right_samples_with_captured_commands() -> None:
    source = text(STM32 / "Core/Src/bimanual_tracking_feedback.c")
    assert "Servo_InMotionTelemetryStart(joint_index" in source
    assert "RightServoBus_InMotionTelemetryStart(" in source
    assert "Servo_InMotionTelemetryPoll(" in source
    assert "RightServoBus_InMotionTelemetryPoll(" in source
    assert "left_commanded_urad" in source
    assert "right_commanded_urad" in source
    assert "maximum_reply_latency_ms" in source


def test_f8_requires_feedback_before_next_tick_and_stops_both() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    executor = source[source.index("static void Host_ServiceV2Executor") :]
    service = source[source.index("void BinaryControl_Service") :]
    stop = source[
        source.index("static uint8_t Host_PerformV2CoordinatedStop") :
        source.index("static uint8_t Host_ConfigureBimanualForTrajectory")
    ]
    assert "BimanualTrackingFeedback_Pending() != 0U" in executor
    assert service.index("Host_ServiceV2TrackingFeedback();") < service.index(
        "Host_ServiceV2Executor();"
    )
    assert stop.index("BimanualTrackingFeedback_End();") < stop.index(
        "Servo_DisableTorqueAll()"
    )
    assert stop.index("BimanualTrackingFeedback_End();") < stop.index(
        "RightServoBus_DisableTorqueAll()"
    )
    assert "actuator_v2_stream_executor_check_joint_feedback(" in source
    assert "Host_RequestV2CoordinatedStop();" in source


def test_f8_protocol_and_tools_expose_tracking_evidence() -> None:
    contract = text(ACTUATOR / "include/actuator_core/stream_contract_v2.h")
    transport = text(
        ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/stream_transport_v2.py"
    )
    no_output = text(ROOT / "tools/contract_evidence/validate_f8_bimanual_tracking_no_output.py")
    hold = text(ROOT / "tools/contract_evidence/execute_f8_bimanual_tracking_hold_once.py")
    assert "ACTUATOR_V2_MSG_TRACKING_DIAGNOSTICS UINT8_C(60)" in contract
    assert "ACTUATOR_V2_TRACKING_DIAGNOSTICS_WIRE_SIZE 76u" in contract
    assert "def get_tracking_diagnostics" in transport
    for source in (no_output, hold):
        assert "EXPECTED_FIRMWARE_VERSION = 0x00024700" in source
    assert "tracking.requested_pairs != launch_delta" in hold
    assert "tracking.completed_pairs != launch_delta" in hold
    assert "tracking.maximum_reply_latency_ms >= 5" in hold
    assert "maximum_tracking_error_urad" in hold


def test_f8_tracking_fault_injection_is_isolated_and_fail_closed() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    source = text(STM32 / "Core/Src/binary_control.c")
    tool = text(ROOT / "tools/contract_evidence/validate_f8_bimanual_tracking_fault_stop_once.py")
    assert "BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024701UL" in cmake
    assert "HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD=1U" in cmake
    assert "#define HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD 0U" in config
    assert "tracking->completed_pairs >= 8U" in source
    assert "sample.right_commanded_urad + INT32_C(100000)" in source
    assert source.index("Host_ServiceV2TrackingFeedback();") < source.index(
        "Host_ServiceV2Executor();"
    )
    assert "EXPECTED_FIRMWARE_VERSION = 0x00024701" in tool
    assert "EXPECTED_PAIRS_BEFORE_FAULT = 8" in tool
    assert "StreamTerminalReasonV2.TRACKING_ERROR" in tool
    assert "verified_left_torque_disabled" in tool
    assert "verified_right_torque_disabled" in tool
