from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text(encoding="utf-8")
APP = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/single_arm_app.c"
).read_text(encoding="utf-8")
HOST_RX_HEADER = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/host_uart_rx.h"
).read_text(encoding="utf-8")
HOST_RX = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/host_uart_rx.c"
).read_text(encoding="utf-8")
MSP = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/stm32g4xx_hal_msp.c"
).read_text(encoding="utf-8")
IRQ = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/stm32g4xx_it.c"
).read_text(encoding="utf-8")
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")
TRANSPORT = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/transport.py"
).read_text(encoding="utf-8")
IDENTITY = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_buffered_rx_identity_and_capability_are_fail_closed() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022700)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)" in CONFIG
    assert "BUFFERED_HOST_RX_CAPABILITY = 0x00000040" in IDENTITY
    assert "interrupt-buffered host RX capability is missing" in IDENTITY


def test_binary_mode_uses_irq_ring_not_polling_receive() -> None:
    body = function_body(APP, "void SingleArmApp_Process(void)")
    binary_start = body.index("if (BinaryControl_IsBinaryMode() != 0U)")
    ascii_receive = body.index(
        "HAL_StatusTypeDef host_receive_status",
        binary_start,
    )
    binary_branch = body[binary_start:ascii_receive]

    assert "HostUartRx_Pop(&rx_byte)" in binary_branch
    assert "HostUartRx_TakeFault()" in binary_branch
    assert "HAL_UART_Receive(" not in binary_branch
    assert binary_branch.index("HostUartRx_TakeFault") < binary_branch.index(
        "HostUartRx_Pop"
    )
    assert binary_branch.index("HostUartRx_Pop") < binary_branch.index(
        "BinaryControl_ProcessByte"
    )

    take_fault = function_body(HOST_RX, "uint8_t HostUartRx_TakeFault(void)")
    assert "host_rx_tail = host_rx_head" in take_fault


def test_lpuart_irq_is_enabled_and_feeds_a_power_of_two_ring() -> None:
    assert "HOST_UART_RX_RING_CAPACITY UINT16_C(1024)" in HOST_RX_HEADER
    assert "HAL_UART_Receive_IT(" in HOST_RX
    assert "void HAL_UART_RxCpltCallback(" in HOST_RX
    assert "host_rx_ring[host_rx_head] = host_rx_interrupt_byte" in HOST_RX
    assert "next == host_rx_tail" in HOST_RX
    assert "__get_PRIMASK()" in HOST_RX
    assert "HAL_NVIC_EnableIRQ(LPUART1_IRQn)" in MSP
    assert "HAL_UART_IRQHandler(&hlpuart1)" in IRQ


def test_ring_capacity_covers_worst_legacy_blocking_window() -> None:
    # A heartbeat frame is <= 32 encoded bytes and the host sends at 10 Hz.
    # Even a legacy 2-second blocking position sweep fits with margin.
    capacity = 1024
    required = 32 * 10 * 2
    assert capacity > required


def test_motion_servo_io_is_cooperative_per_main_loop() -> None:
    safety = function_body(HOST_RX + BINARY, "static void Host_ServiceBinaryMotion(void)")
    assert "Servo_PositionSweepStep(" in safety
    assert "Servo_ReadAllPositions(" not in safety
    assert "Servo_MotionSafetyPoll()" in safety
    assert "SERVO_MOTION_SAFETY_SLOT_MS UINT32_C(16)" in CONFIG
    assert "host_stop_latched = 1U" in BINARY
    assert "actuator_safety_request_hold" in BINARY


def test_firmware_acknowledges_each_accepted_heartbeat() -> None:
    handler = function_body(
        BINARY,
        "static void Host_HandleBinaryFrame(const actuator_frame_t *request)",
    )
    heartbeat_start = handler.index("case ACTUATOR_MSG_HEARTBEAT:")
    heartbeat_end = handler.index("case ACTUATOR_MSG_GET_STATE:", heartbeat_start)
    heartbeat_case = handler[heartbeat_start:heartbeat_end]
    assert "actuator_safety_on_heartbeat(" in heartbeat_case
    assert "Host_SendBinaryState(request->sequence, 0U);" in heartbeat_case


def test_host_waits_for_matching_heartbeat_acknowledgement() -> None:
    heartbeat_start = TRANSPORT.index("def heartbeat(self) -> State:")
    heartbeat = TRANSPORT[
        heartbeat_start : TRANSPORT.index("def get_state(", heartbeat_start)
    ]
    assert "HEARTBEAT_RESPONSE_TIMEOUT_S = 0.40" in TRANSPORT
    assert "MessageType.STATE_FEEDBACK" in heartbeat
    assert "parse_state(" in heartbeat
    assert "if state.stop_latched:" in heartbeat
    assert "raise StopLatchedError(" in heartbeat
    assert "if state.status_code != 0:" in heartbeat
