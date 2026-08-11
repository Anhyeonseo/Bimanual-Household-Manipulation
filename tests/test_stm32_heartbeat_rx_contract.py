from pathlib import Path
import subprocess
import textwrap


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
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG
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

    assert "HostUartRx_Pop(&rx_byte, &received_at_ms)" in binary_branch
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
        "static void Host_HandleBinaryFrame(",
    )
    heartbeat_start = handler.index("case ACTUATOR_MSG_HEARTBEAT:")
    heartbeat_end = handler.index("case ACTUATOR_MSG_GET_STATE:", heartbeat_start)
    heartbeat_case = handler[heartbeat_start:heartbeat_end]
    assert "actuator_safety_on_heartbeat(" in heartbeat_case
    assert "Host_SendBinaryState(request->sequence, 0U);" in heartbeat_case


def test_f1_uses_frame_completion_isr_tick_for_heartbeat_freshness() -> None:
    callback = function_body(HOST_RX, "void HAL_UART_RxCpltCallback(")
    pop = function_body(
        HOST_RX,
        "uint8_t HostUartRx_Pop(uint8_t *byte, uint32_t *received_at_ms)",
    )
    process = function_body(
        BINARY,
        "static void Host_ProcessBinaryByte(uint8_t byte, uint32_t received_at_ms)",
    )
    handler = function_body(BINARY, "static void Host_HandleBinaryFrame(")
    heartbeat_start = handler.index("case ACTUATOR_MSG_HEARTBEAT:")
    heartbeat_end = handler.index("case ACTUATOR_MSG_GET_STATE:", heartbeat_start)
    heartbeat_case = handler[heartbeat_start:heartbeat_end]

    assert "host_rx_received_at_ms[host_rx_head] = HAL_GetTick();" in callback
    assert callback.index("host_rx_received_at_ms[host_rx_head]") < callback.index(
        "host_rx_head = next"
    )
    assert "*received_at_ms = host_rx_received_at_ms[host_rx_tail];" in pop
    assert "Host_HandleBinaryFrame(&request, received_at_ms);" in process
    assert "host_binary_last_heartbeat_ms = received_at_ms;" in heartbeat_case
    assert "host_binary_last_heartbeat_ms = HAL_GetTick();" not in heartbeat_case
    assert "HOST_F1_HEARTBEAT_RX_TIMESTAMP_CAPABILITY UINT32_C(0x00004000)" in CONFIG
    assert "F1_HEARTBEAT_RX_TIMESTAMP_CANDIDATE_FIRMWARE_VERSION = 0x00023200" in IDENTITY
    assert "heartbeat RX timestamp capability is missing" in IDENTITY


def test_f1_400ms_background_stall_gate_keeps_received_heartbeat_fresh(
    tmp_path: Path,
) -> None:
    (tmp_path / "stm32g4xx_hal.h").write_text(
        textwrap.dedent(
            """
            #ifndef STM32G4XX_HAL_H
            #define STM32G4XX_HAL_H
            #include <stdint.h>
            typedef struct { uint32_t instance; } UART_HandleTypeDef;
            typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
            #define RESET 0U
            #define UART_RXDATA_FLUSH_REQUEST 0U
            #define __HAL_UART_CLEAR_OREFLAG(huart) ((void)(huart))
            #define __HAL_UART_SEND_REQ(huart, request) \\
                do { (void)(huart); (void)(request); } while (0)
            HAL_StatusTypeDef HAL_UART_Receive_IT(
                UART_HandleTypeDef *huart,
                uint8_t *data,
                uint16_t length
            );
            uint32_t HAL_GetTick(void);
            uint32_t __get_PRIMASK(void);
            void __disable_irq(void);
            void __enable_irq(void);
            #endif
            """
        ),
        encoding="utf-8",
    )
    (tmp_path / "host_uart_rx.h").write_text(
        textwrap.dedent(
            """
            #ifndef HOST_UART_RX_H
            #define HOST_UART_RX_H
            #include "stm32g4xx_hal.h"
            #include <stdint.h>
            #define HOST_UART_RX_RING_CAPACITY UINT16_C(1024)
            void HostUartRx_Init(UART_HandleTypeDef *host_uart);
            HAL_StatusTypeDef HostUartRx_Start(void);
            uint8_t HostUartRx_Pop(uint8_t *byte, uint32_t *received_at_ms);
            uint8_t HostUartRx_TakeFault(void);
            uint16_t HostUartRx_Count(void);
            #endif
            """
        ),
        encoding="utf-8",
    )
    (tmp_path / "host_uart_tx.h").write_text(
        textwrap.dedent(
            """
            #ifndef HOST_UART_TX_H
            #define HOST_UART_TX_H
            #include "stm32g4xx_hal.h"
            void HostUartTx_OnError(UART_HandleTypeDef *huart);
            #endif
            """
        ),
        encoding="utf-8",
    )
    (tmp_path / "servo_bus.h").write_text(
        textwrap.dedent(
            """
            #ifndef SERVO_BUS_H
            #define SERVO_BUS_H
            #include "stm32g4xx_hal.h"
            void ServoBus_HandleUartError(UART_HandleTypeDef *huart);
            #endif
            """
        ),
        encoding="utf-8",
    )
    harness = tmp_path / "f1_stall_harness.c"
    harness.write_text(
        textwrap.dedent(
            """
            #include "host_uart_rx.h"
            #include "actuator_core/safety.h"
            #include <assert.h>
            #include <stdbool.h>
            #include <stdint.h>

            static uint32_t fake_tick_ms;
            static uint8_t *interrupt_target;

            HAL_StatusTypeDef HAL_UART_Receive_IT(
                UART_HandleTypeDef *huart,
                uint8_t *data,
                uint16_t length
            ) {
                (void)huart;
                assert(length == 1U);
                interrupt_target = data;
                return HAL_OK;
            }

            uint32_t HAL_GetTick(void) { return fake_tick_ms; }
            uint32_t __get_PRIMASK(void) { return 0U; }
            void __disable_irq(void) {}
            void __enable_irq(void) {}
            void ServoBus_HandleUartError(UART_HandleTypeDef *huart) {
                (void)huart;
            }
            void HostUartTx_OnError(UART_HandleTypeDef *huart) {
                (void)huart;
            }
            void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

            int main(void) {
                UART_HandleTypeDef uart = {0U};
                actuator_safety_t safety;
                uint8_t byte = UINT8_C(0xFF);
                uint32_t received_at_ms = UINT32_MAX;

                HostUartRx_Init(&uart);
                assert(HostUartRx_Start() == HAL_OK);
                actuator_safety_init(&safety, UINT32_C(500));
                assert(actuator_safety_complete_boot(&safety, true)
                    == ACTUATOR_SAFETY_OK);
                actuator_safety_on_heartbeat(&safety, 0U);
                assert(actuator_safety_request_arm(&safety, true, true)
                    == ACTUATOR_SAFETY_OK);
                assert(actuator_safety_request_enable(&safety, 0U)
                    == ACTUATOR_SAFETY_OK);

                fake_tick_ms = UINT32_C(100);
                *interrupt_target = 0U;
                HAL_UART_RxCpltCallback(&uart);

                fake_tick_ms = UINT32_C(500);
                assert(HostUartRx_Pop(&byte, &received_at_ms) == 1U);
                assert(byte == 0U);
                assert(received_at_ms == UINT32_C(100));
                actuator_safety_on_heartbeat(&safety, received_at_ms);
                actuator_safety_tick(&safety, fake_tick_ms);
                assert(safety.state == ACTUATOR_STATE_ACTIVE);
                assert(!safety.hold_requested);
                return 0;
            }
            """
        ),
        encoding="utf-8",
    )
    executable = tmp_path / "f1_stall_harness"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(tmp_path),
            "-I",
            str(ROOT / "firmware/stm32_actuator/include"),
            str(ROOT / "firmware/stm32_g474_single_arm/Core/Src/host_uart_rx.c"),
            str(ROOT / "firmware/stm32_actuator/src/safety.c"),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


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
