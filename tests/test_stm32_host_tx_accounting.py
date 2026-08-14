"""
host frame 전송 결과가 버려지지 않고 기록되는지 강제한다.

`Host_SendBinaryFrame` 은 100 ms timeout 의 blocking `HAL_UART_Transmit` 하나로
프레임을 보내는데, 여덟 개 호출부가 모두 반환값을 `(void)` 로 버렸다. 그래서
전송이 중간에 잘려도 firmware 에 아무 흔적이 남지 않았다.

2026-08-06 buffered 실행에서 42 바이트짜리 STATE_FEEDBACK 하나가 24 + 18 로
쪼개져 host 의 120 ms read timeout 경계를 넘었다. host 는 앞 24 바이트를
구분자 없는 부분 패킷으로 버리고 뒤 18 바이트를 해독 불가 프레임으로 버렸으며,
결국 응답을 잃고 예산을 전부 소진했다. MCU 는 그 순간 `HAL_TIMEOUT` 을 이미
받고 있었지만 버리고 있었다.
"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")
PROTOCOL = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/protocol.py"
).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.rindex(signature)
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


def test_enqueue_result_is_captured_not_discarded() -> None:
    body = function_body(BINARY, "static HAL_StatusTypeDef Host_SendBinaryFrame(")
    assert "HAL_StatusTypeDef status = HostUartTx_Enqueue(" in body
    assert "HAL_UART_Transmit(" not in body
    assert "host_tx_last_status = (uint8_t)status;" in body
    assert "if (status != HAL_OK)" in body
    assert "host_tx_failure_count++" in body


def test_dma_or_queue_failure_is_latched_for_fail_closed_service() -> None:
    assert "HostUartTx_TakeFault()" in BINARY
    assert "ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS" in BINARY


def test_transmit_elapsed_time_is_measured() -> None:
    """
    F2 뒤 이 값은 wire duration이 아니라 enqueue에 걸린 control-path
    시간이다. DMA 전송은 ISR에서 완료되므로 이 수치가 apply lateness를
    차지하지 않는다는 것을 terminal F0 계측으로 다시 확인한다.
    """
    body = function_body(BINARY, "static HAL_StatusTypeDef Host_SendBinaryFrame(")
    assert "uint32_t started = HAL_GetTick();" in body
    assert "HAL_GetTick() - started" in body
    assert "host_tx_maximum_ms" in body


def test_counters_are_exposed_in_diagnostics() -> None:
    assert "Host_WriteU16Le(&response.payload[138], host_tx_failure_count);" in BINARY
    assert "Host_WriteU16Le(&response.payload[140], host_tx_timeout_count);" in BINARY
    assert "Host_WriteU16Le(&response.payload[142], host_tx_maximum_ms);" in BINARY
    assert "response.payload[144] = host_tx_last_status;" in BINARY
    assert "response.payload_length = 146U;" in BINARY


def test_counters_reset_on_binary_mode_restart() -> None:
    assert "host_tx_failure_count = 0U;" in BINARY
    assert "host_tx_timeout_count = 0U;" in BINARY
    assert "host_tx_maximum_ms = 0U;" in BINARY


def test_host_parses_the_wider_diagnostics_payload() -> None:
    assert 'DIAGNOSTICS_HOST_TX = struct.Struct("<3HBx")' in PROTOCOL
    assert "host_tx_length = extended_length + DIAGNOSTICS_HOST_TX.size" in PROTOCOL
    for field in (
        "host_tx_failure_count",
        "host_tx_timeout_count",
        "host_tx_maximum_ms",
        "host_tx_last_status",
    ):
        assert field in PROTOCOL


def test_older_payload_lengths_still_decode() -> None:
    """구버전 firmware 가 계속 해석되어야 한다."""
    # parse_setpoint_status 에도 같은 모양의 검사가 있으므로 진단 파서로 한정한다.
    start = PROTOCOL.index("def parse_servo_diagnostic(")
    match = re.search(
        r"if len\(payload\) not in \(\s*(.*?)\s*\):",
        PROTOCOL[start:],
        re.DOTALL,
    )
    assert match is not None
    accepted = match.group(1)
    for name in (
        "legacy_length",
        "legacy_health_length",
        "extended_length",
        "host_tx_length",
    ):
        assert name in accepted


def test_wide_payload_sizes_agree_between_sides() -> None:
    """firmware payload 길이와 host struct 합계가 같아야 한다."""
    import struct

    base = struct.calcsize("<BBBBII")
    joint = struct.calcsize("<8B7H2B2H2BH4B")
    health = struct.calcsize("<8B11I6H2IBB16s")
    host_tx = struct.calcsize("<3HBx")
    assert base + joint + health + host_tx == 146
