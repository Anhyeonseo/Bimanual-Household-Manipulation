from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STM32 = ROOT / "firmware/stm32_g474_single_arm"
ACTUATOR = ROOT / "firmware/stm32_actuator"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_f7_has_unique_v2_identity_and_dispatch_capability() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    assert "BIMANUAL_DMA_DISPATCH_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024604UL" in cmake
    assert "HOST_BINARY_CAPABILITIES=0xEFFFFFFFUL" in cmake
    assert "HOST_BINARY_JOINT_COUNT=12U" in cmake
    assert "HOST_BINARY_UART_BAUD=921600UL" in cmake
    assert "ACTUATOR_ENABLE_BIMANUAL_DISPATCH_MESSAGES=1U" in cmake
    assert "HOST_BIMANUAL_DMA_DISPATCH_CAPABILITY" in config


def test_f7_launches_one_packet_per_arm_with_dma_and_rolls_back() -> None:
    source = text(STM32 / "Core/Src/bimanual_servo_dispatch.c")
    assert source.count("actuator_sts3215_build_sync_write_positions(") == 2
    assert source.count("HAL_UART_Transmit_DMA(") == 2
    left = source.index("left_bus_uart, left_packet")
    right = source.index("right_bus_uart, right_packet")
    assert left < right
    assert "HAL_UART_AbortTransmit(left_bus_uart)" in source
    assert "actuator_bimanual_dispatch_begin(" in source


def test_f7_ignores_unrelated_uart_callbacks_outside_active_dispatch() -> None:
    source = text(STM32 / "Core/Src/bimanual_servo_dispatch.c")
    tx_complete = source[
        source.index("void BimanualServoDispatch_OnTxComplete") :
        source.index("void BimanualServoDispatch_OnUartError")
    ]
    uart_error = source[
        source.index("void BimanualServoDispatch_OnUartError") :
        source.index("void BimanualServoDispatch_Stop")
    ]
    assert "!snapshot->active" in tx_complete
    assert "launch_in_progress != 0U" in uart_error
    assert "snapshot->active" in uart_error


def test_f7_connects_uart4_tc_irq_after_dma_completion() -> None:
    msp = text(STM32 / "Core/Src/stm32g4xx_hal_msp.c")
    interrupts = text(STM32 / "Core/Src/stm32g4xx_it.c")
    header = text(STM32 / "Core/Inc/stm32g4xx_it.h")
    assert "HAL_NVIC_EnableIRQ(UART4_IRQn);" in msp
    assert "void UART4_IRQHandler(void)" in interrupts
    assert "HAL_UART_IRQHandler(&huart4);" in interrupts
    assert "void UART4_IRQHandler(void);" in header


def test_f7_arm_watchdog_grace_starts_after_blocking_configuration() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    configure = source[
        source.index("static uint8_t Host_ConfigureBimanualForTrajectory") :
        source.index("static void Host_ServiceV2Executor")
    ]
    service = source[source.index("void BinaryControl_Service") :]
    assert "host_bimanual_arm_watchdog_grace_started_ms = HAL_GetTick();" in configure
    assert configure.index("host_bimanual_arm_watchdog_grace_started_ms") < configure.index(
        "host_right_arm_output_active = 1U;"
    )
    assert "now_ms -\n              host_bimanual_arm_watchdog_grace_started_ms" in service
    assert "now_ms - host_binary_last_heartbeat_ms" in service


def test_f7_uses_one_tick_event_and_full_twelve_axis_goal_map() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    candidate = source[source.index("static void Host_ServiceV2Executor") :]
    assert "ControlTick_TakeEvent(&event)" in candidate
    assert "event.missed_count != 0U" in candidate
    assert "BimanualOperationalLimits_MapExecutorOutput(" in candidate
    assert "BimanualServoDispatch_Launch(" in candidate
    assert "Host_RequestV2CoordinatedStop();" in candidate
    admission = source[
        source.index("static void Host_ValidateV2Batch") :
        source.index("static void Host_RequestV2CoordinatedStop")
    ]
    assert "host_v2_executor_start_pending = 1U;" in admission
    assert "ControlTick_ClearPending();" in admission
    assert "host_v2_executor_start_pending != 0U" in candidate
    start = candidate.index("actuator_v2_stream_executor_start(")
    event_epoch = candidate.index("event.tick_ms", start)
    step = candidate.index("actuator_v2_stream_executor_step(")
    assert start < event_epoch < step


def test_f7_dispatch_messages_are_compile_gated() -> None:
    protocol = text(ACTUATOR / "src/protocol.c")
    contract = text(ACTUATOR / "include/actuator_core/stream_contract_v2.h")
    assert "ACTUATOR_ENABLE_BIMANUAL_DISPATCH_MESSAGES" in protocol
    assert "ACTUATOR_V2_MSG_GET_DISPATCH_DIAGNOSTICS" in protocol
    assert "ACTUATOR_V2_MSG_DISPATCH_DIAGNOSTICS" in protocol
    assert "ACTUATOR_V2_DISPATCH_DIAGNOSTICS_WIRE_SIZE 44u" in contract


def test_f7_fault_candidate_is_build_isolated_and_deterministic() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    dispatch = text(STM32 / "Core/Src/bimanual_servo_dispatch.c")
    control = text(STM32 / "Core/Src/binary_control.c")
    tool = text(ROOT / "tools/contract_evidence/validate_f7_bimanual_right_dma_fault_stop_once.py")
    assert "BIMANUAL_DMA_FAULT_INJECTION_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024605UL" in cmake
    assert "HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD=1U" in cmake
    assert "#define HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD 0U" in config
    assert "snapshot->completed_count >= 8U" in dispatch
    assert "HAL_UART_AbortTransmit(left_bus_uart)" in dispatch
    assert "right_dma_fault_injection_consumed = 1U" in dispatch
    assert "host_v2_last_coordinated_stop_status = status" in control
    assert "host_v2_last_coordinated_stop_status = 2U" in control
    assert "EXPECTED_FIRMWARE_VERSION = 0x00024605" in tool
    assert "launch_delta != 8" in tool
    assert "completed_delta != 8" in tool
    assert "failure_delta != 1" in tool
    assert "verified_left_torque_disabled" in tool
    assert "verified_right_torque_disabled" in tool


def test_f7_hardware_tools_freeze_no_output_then_zero_delta_hold() -> None:
    no_output = text(ROOT / "tools/contract_evidence/validate_f7_bimanual_dma_no_output.py")
    hold = text(ROOT / "tools/contract_evidence/execute_f7_bimanual_current_pose_hold_once.py")
    roundtrip = text(
        ROOT / "tools/contract_evidence/execute_f7_bimanual_base_small_roundtrip_once.py"
    )
    for source in (no_output, hold, roundtrip):
        assert "EXPECTED_FIRMWARE_VERSION = 0x00024604" in source
        assert "EXPECTED_CAPABILITIES = 0xEFFFFFFF" in source
    assert 'CONFIRMATION = "F7_BIMANUAL_DMA_NO_OUTPUT"' in no_output
    assert '"dma_launches": 0' in no_output
    assert 'CONFIRMATION = "F7_BIMANUAL_CURRENT_POSE_HOLD_ONCE"' in hold
    assert "positions_urad=shadow.anchor_positions_urad" in hold
    assert '"commanded_motion_delta_urad": [0] * JOINT_COUNT' in hold
    assert "transport.safe_stop()" in hold
    assert "failure_executor = transport.get_executor_diagnostics()" in hold
    assert "failure_dispatch = transport.get_dispatch_diagnostics()" in hold
    assert 'CONFIRMATION = "F7_BIMANUAL_BASE_SMALL_ROUNDTRIP_ONCE"' in roundtrip
    assert 'parser.add_argument("--delta-rad", type=float, default=0.03)' in roundtrip
    assert "target[0] += delta_urad" in roundtrip
    assert "target[6] += delta_urad" in roundtrip
    assert "(anchor, target, anchor)[index]" in roundtrip
    assert "transport.safe_stop()" in roundtrip
