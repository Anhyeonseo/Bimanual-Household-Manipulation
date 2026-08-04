from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SERVO_BUS = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/servo_bus.c"
).read_text(encoding="utf-8")
CONFIG = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")


def test_native_response_parser_fault_injection(tmp_path: Path) -> None:
    compiler = shutil.which("cc")
    if compiler is None:
        pytest.skip("native C compiler is unavailable")

    executable = tmp_path / "servo_response_parser_test"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            "-I",
            str(ROOT / "firmware/stm32_g474_single_arm/Core/Inc"),
            str(ROOT / "firmware/stm32_g474_single_arm/Core/Src/servo_response_parser.c"),
            str(ROOT / "tests/native/test_servo_response_parser.c"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=ROOT,
    )
    subprocess.run([str(executable)], check=True, cwd=ROOT)


def test_uart_recovery_clears_all_blocking_rx_faults() -> None:
    assert "HAL_UART_Abort(servo_uart_handle)" in SERVO_BUS
    for flag in (
        "UART_CLEAR_OREF",
        "UART_CLEAR_NEF",
        "UART_CLEAR_PEF",
        "UART_CLEAR_FEF",
        "UART_CLEAR_RTOF",
    ):
        assert flag in SERVO_BUS
    assert "UART_RXDATA_FLUSH_REQUEST" in SERVO_BUS
    assert "SERVO_BUS_RECOVERY_QUIET_MS" in SERVO_BUS


def test_reads_are_bounded_burst_and_resynchronized() -> None:
    assert "SERVO_BUS_READ_TIMEOUT_MS UINT32_C(50)" in SERVO_BUS
    assert "SERVO_BUS_MAX_RX_BYTES UINT16_C(64)" in SERVO_BUS
    assert "HAL_UARTEx_ReceiveToIdle(" in SERVO_BUS
    read_start = SERVO_BUS.rindex("HAL_StatusTypeDef Servo_ReadData(")
    write_start = SERVO_BUS.index("HAL_StatusTypeDef Servo_WriteData(", read_start)
    read_body = SERVO_BUS[read_start:write_start]
    assert "HAL_UART_Receive(" not in read_body
    assert "ServoResponseParser_Push(&parser, burst[index])" in read_body
    assert "received_total < SERVO_BUS_MAX_RX_BYTES" in read_body
    assert "ServoBus_MapRejectReason(parser.last_reject)" in read_body
    assert "(void)ServoBus_Recover();" in read_body


def test_uart_diagnostics_map_all_receive_faults() -> None:
    mappings = {
        "UART_FLAG_PE": "HAL_UART_ERROR_PE",
        "UART_FLAG_NE": "HAL_UART_ERROR_NE",
        "UART_FLAG_FE": "HAL_UART_ERROR_FE",
        "UART_FLAG_ORE": "HAL_UART_ERROR_ORE",
        "UART_FLAG_RTOF": "HAL_UART_ERROR_RTO",
    }
    for flag, error in mappings.items():
        assert flag in SERVO_BUS
        assert error in SERVO_BUS
    assert "servo_bus_diagnostics.uart_error_code = uart_errors" in SERVO_BUS


def test_partial_write_reply_drain_is_removed() -> None:
    write_start = SERVO_BUS.rindex("HAL_StatusTypeDef Servo_WriteData(")
    write_end = SERVO_BUS.rindex("HAL_StatusTypeDef Servo_ReadPosition(")
    write_body = SERVO_BUS[write_start:write_end]
    assert "HAL_UART_Receive(" not in write_body
    assert "SERVO_BUS_WRITE_REPLY_SETTLE_MS" in write_body
    assert "ServoBus_ClearRxState();" in write_body


def test_identity_and_failure_payload_advertise_recovery_diagnostics() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022200)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)" in CONFIG
    assert "response.payload_length = 40U;" in BINARY
    assert "ServoBus_GetDiagnostics()" in BINARY
    assert "bus->uart_error_code" in BINARY
    assert "bus->uart_isr" in BINARY
