"""버퍼드 응답 프레임의 전송시간이 apply lateness 예산 안에 있어야 한다.

Host_SendBinaryFrame 은 blocking HAL_UART_Transmit 이고, 그것을 호출하는
협조적 루프가 곧 executor 를 stepping 하는 루프다. 따라서 응답 프레임의
인코딩된 길이를 host baud 로 나눈 값이 apply lateness 로 그대로 청구된다.
이건 부수효과가 아니라 산술이다.

측정된 근거:

  0x00022500  status payload 32 B -> 전선 54 B -> 4.688 ms
              Motion-11 실기 통과, 관측된 max apply lateness 정확히 5 ms
  0x00022800  status payload 60 B -> 전선 82 B -> 7.118 ms
              2026-08-06 q0 복귀가 applied=0 / missed_apply_tick 으로 중단

두 번째 값은 lateness histogram 을 refill 응답에 실으면서 생겼다. 계측이
계측 대상을 바꾼 경우이며, 이 시험은 그 회귀가 조용히 돌아오지 못하게 한다.

여유는 5 - 4.688 = 0.312 ms 로 얇다. 이 시험은 그 얇음을 숨기지 않고
수치로 드러낸다. 근본 해결은 host TX 를 non-blocking 으로 돌리는 것이며
docs/CURRENT_STATE_AND_NEXT_ROADMAP.md 에 선행 조건으로 기록되어 있다.
"""

from __future__ import annotations

import math
import pathlib
import re
import sys

FIRMWARE = pathlib.Path(__file__).resolve().parents[1] / "firmware"
sys.path.insert(0, str(FIRMWARE.parents[0] / "tools"))

from actuator_protocol import (  # noqa: E402
    CRC,
    HEADER,
    Frame,
    MessageType,
    encode_frame,
)

CONFIG = (
    FIRMWARE / "stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text()
MAIN = (FIRMWARE / "stm32_g474_single_arm/Core/Src/main.c").read_text()
BINARY_CONTROL = (
    FIRMWARE / "stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text()
ROUTE_HEADER = (
    FIRMWARE / "stm32_actuator/include/actuator_core/buffered_command_route.h"
).read_text()
ROUTE_SOURCE = (
    FIRMWARE / "stm32_actuator/src/buffered_command_route.c"
).read_text()


def _u32(source: str, name: str) -> int:
    match = re.search(rf"#define {name} UINT32_C\((\d+)\)", source)
    assert match is not None, f"{name} 정의를 찾지 못했다"
    return int(match.group(1))


def _size(source: str, name: str) -> int:
    match = re.search(rf"#define {name} (\d+)u", source)
    assert match is not None, f"{name} 정의를 찾지 못했다"
    return int(match.group(1))


HOST_BAUD = _u32(CONFIG, "HOST_BINARY_UART_BAUD")
BITS_PER_BYTE = _u32(CONFIG, "HOST_BINARY_UART_BITS_PER_BYTE")
MAXIMUM_APPLY_LATENESS_MS = _u32(
    CONFIG, "HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS"
)
EXTENDED_SIZE = _size(ROUTE_HEADER, "ACTUATOR_BUFFERED_STATUS_EXTENDED_SIZE")
LATENESS_SIZE = _size(ROUTE_HEADER, "ACTUATOR_BUFFERED_STATUS_LATENESS_SIZE")


def wire_bytes(payload_bytes: int) -> int:
    """저장소의 실제 인코더로 전선 길이를 만든다. 공식을 신뢰하지 않는다."""
    frame = Frame(
        message_type=int(MessageType.SETPOINT_STATUS),
        flags=0,
        sequence=0xFFFFFFFF,
        sender_time_ms=0xFFFFFFFF,
        payload=bytes(payload_bytes),
    )
    return len(encode_frame(frame))


def transmit_ms(payload_bytes: int) -> float:
    return wire_bytes(payload_bytes) * BITS_PER_BYTE * 1000.0 / HOST_BAUD


def test_declared_host_baud_matches_the_lpuart_initialiser() -> None:
    """예산 계산이 실제로 설정된 속도를 쓰고 있어야 한다."""
    match = re.search(r"hlpuart1\.Init\.BaudRate = (\d+);", MAIN)
    assert match is not None
    assert int(match.group(1)) == HOST_BAUD


def test_wire_length_formula_agrees_with_the_real_encoder() -> None:
    """C 매크로의 산술이 COBS/CRC 실제 결과와 같아야 한다."""
    for payload in (0, EXTENDED_SIZE, LATENESS_SIZE, 128, 253):
        declared = payload + HEADER.size + CRC.size + 1 + 1
        assert declared == wire_bytes(payload), payload


def test_acknowledgement_transmit_fits_the_apply_lateness_allowance() -> None:
    """refill 응답 하나가 lateness 예산을 넘기면 첫 sample 에서 죽는다."""
    assert math.ceil(transmit_ms(EXTENDED_SIZE)) <= MAXIMUM_APPLY_LATENESS_MS


def test_the_lateness_block_would_not_fit_an_acknowledgement() -> None:
    """terminal 전용으로 둔 이유를 수치로 고정한다."""
    assert math.ceil(transmit_ms(LATENESS_SIZE)) > MAXIMUM_APPLY_LATENESS_MS


def test_measured_transmit_times_match_the_recorded_evidence() -> None:
    """실기 판정에 쓰인 두 숫자를 회귀로 붙잡는다."""
    assert round(transmit_ms(EXTENDED_SIZE), 3) == 4.688
    assert round(transmit_ms(LATENESS_SIZE), 3) == 7.118


def test_remaining_acknowledgement_margin_is_reported_as_thin() -> None:
    """여유가 1 ms 미만이라는 사실 자체를 시험으로 남긴다.

    이 단언이 깨진다면 non-blocking TX 가 들어왔거나 baud 가 올라간 것이고,
    둘 다 이 파일의 서술을 다시 써야 하는 변경이다.
    """
    margin_ms = MAXIMUM_APPLY_LATENESS_MS - transmit_ms(EXTENDED_SIZE)
    assert 0.0 < margin_ms < 1.0


def test_only_terminal_frames_carry_the_lateness_distribution() -> None:
    match = re.search(
        r"actuator_buffered_status_encode\((.*?)\)\)", BINARY_CONTROL, re.S
    )
    assert match is not None
    assert "(status_code == HOST_BUFFERED_STATUS_TERMINAL)" in match.group(1)


def test_encoder_omits_the_lateness_block_when_not_requested() -> None:
    assert "bool include_lateness" in ROUTE_HEADER
    assert "if (include_lateness) {" in ROUTE_SOURCE
    assert "const size_t encoded_size = include_lateness ?" in ROUTE_SOURCE


def test_build_refuses_an_acknowledgement_that_outgrows_the_allowance() -> None:
    """시험은 건너뛸 수 있지만 컴파일은 못 건너뛴다."""
    guard = re.search(
        r"#if HOST_BINARY_FRAME_TRANSMIT_MS\(\s*"
        r"ACTUATOR_BUFFERED_STATUS_EXTENDED_SIZE\s*\)\s*>\s*\\?\s*"
        r"HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS\s*\n#error",
        BINARY_CONTROL,
    )
    assert guard is not None
