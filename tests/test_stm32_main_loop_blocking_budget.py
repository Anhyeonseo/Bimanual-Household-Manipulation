"""
main loop 한 바퀴의 blocking 예산을 소스 상수에서 계산해 강제한다.

이 저장소의 firmware 는 협조적 단일 루프다. `SingleArmApp_Process` 한 바퀴가
host 바이트를 처리하고 `BinaryControl_Service` 를 호출한다. 어느 한 호출이
길게 블로킹하면 host UART 처리가 멈추고, heartbeat 는 *수신* 이 아니라
*처리* 시점에 기록되므로(`binary_control.c` 의
`host_binary_last_heartbeat_ms = HAL_GetTick()`) 응답도 안 가고 MCU 자신의
`HOST_BINARY_HEARTBEAT_TIMEOUT_MS` watchdog 도 먹지 못한다. 즉 굶김은 곧
자체 latch 위험이다.

이 불변식은 지금까지 어디에도 명시되지 않았고 두 번 조용히 깨졌다.

  1. 0x00022500 이 모든 servo write 에 ServoBus_PrepareTransaction 을 붙여
     6축 DISABLE 봉투가 늘었다. 산술로 발견했다.
  2. 0x00022800 에서 같은 비용이 buffered 실행 중 motion-safety 폴링에 붙어
     host 를 굶겼다. 실기에서 발견했다. 관측 침묵 365 ms, 한계 500 ms.

두 번째 실패의 구조는 단순하다. 폴링 slot 은 16 ms 인데 poll 1회 최악 비용이
79 ms 였다. slot 보다 비용이 크면 poll 이 밀리고 루프가 포화된다.

여기서는 상수를 소스에서 읽어 계산한다. 값을 복사하지 않으므로 firmware 가
바뀌면 이 시험이 따라 깨진다.
"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
SERVO_BUS = (FIRMWARE / "Core/Src/servo_bus.c").read_text(encoding="utf-8")
BINARY_CONTROL = (
    FIRMWARE / "Core/Src/binary_control.c"
).read_text(encoding="utf-8")
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")


def constant(source: str, name: str) -> int:
    match = re.search(rf"#define\s+{name}\s+UINT(?:8|16|32)_C\((\d+)\)", source)
    assert match is not None, f"{name} 상수를 찾지 못했다"
    return int(match.group(1))


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


def servo_read_worst_case_ms() -> int:
    """PrepareTransaction 을 포함한 servo read 1회 최악 비용."""
    return (
        constant(SERVO_BUS, "SERVO_BUS_IDLE_HIGH_TIMEOUT_MS")
        + constant(SERVO_BUS, "SERVO_BUS_PREFLIGHT_IDLE_TIMEOUT_MS")
        + constant(SERVO_BUS, "SERVO_BUS_READ_TX_TIMEOUT_MS")
        + constant(SERVO_BUS, "SERVO_BUS_READ_TIMEOUT_MS")
        + constant(SERVO_BUS, "SERVO_BUS_RECOVERY_QUIET_MS")
    )


def test_servo_read_cost_is_derived_not_assumed() -> None:
    """비용이 바뀌면 아래 예산 시험들이 따라 움직여야 한다."""
    assert servo_read_worst_case_ms() == 79


def test_motion_safety_poll_cost_exceeds_its_own_slot() -> None:
    """
    이것이 2026-08-06 실패의 구조다. 기록으로 남긴다.

    poll 1회 최악 비용이 slot 주기보다 크면 poll 이 밀린다. 이 사실 자체는
    firmware 상수의 성질이므로 고칠 수 없고, 그래서 buffered 실행 경로에서
    폴링을 아예 하지 않는 것이 답이었다.
    """
    slot_ms = constant(CONFIG, "SERVO_MOTION_SAFETY_SLOT_MS")
    assert servo_read_worst_case_ms() > slot_ms


def test_buffered_execution_performs_no_servo_reads() -> None:
    """
    buffered 실행 경로에 blocking servo read 가 하나도 없어야 한다.

    executor 가 5 ms sync-write 주기로 버스를 소유하는 동안 read 를 끼워 넣으면
    idle-high 대기와 경합해 루프를 포화시킨다. host 요청 servo read 는 이미
    전부 거부되고 있었으므로, 내부 폴링 제거는 그 불변식의 복원이다.
    """
    body = function_body(
        BINARY_CONTROL, "static void Host_ServiceBufferedExecution(void)"
    )
    for blocking_call in (
        "Servo_MotionSafetyPoll(",
        "Servo_ReadData(",
        "Servo_ReadPosition(",
        "Servo_ReadAllPositions(",
        "Servo_ReadTelemetry(",
        "HAL_Delay(",
    ):
        assert blocking_call not in body, (
            f"buffered 실행 경로에 {blocking_call} 가 있다. "
            "host UART 처리를 굶겨 MCU watchdog 까지 위협한다"
        )


def test_buffered_start_does_not_arm_motion_safety_polling() -> None:
    """
    START 에서 폴링을 켜면 위 불변식이 다음 tick 부터 깨진다.

    폴링을 켜는 곳은 단 한 곳, 단일점 motion 경로여야 한다. 그 위치가
    buffered command route 코드보다 앞선다는 것으로 확인한다.
    """
    assert BINARY_CONTROL.count("Servo_MotionSafetyBegin(") == 1
    arming = BINARY_CONTROL.index("Servo_MotionSafetyBegin(")
    buffered_start = BINARY_CONTROL.index(
        "actuator_buffered_command_route_start("
    )
    assert arming < buffered_start


def test_single_point_motion_keeps_its_monitoring() -> None:
    """
    단일점 motion 은 executor 와 버스를 다투지 않으므로 폴링을 유지한다.
    이번 변경이 안전 감시를 통째로 없앤 것이 아님을 고정한다.
    """
    assert "Servo_MotionSafetyBegin(" in BINARY_CONTROL
    assert BINARY_CONTROL.count("Servo_MotionSafetyPoll()") >= 2


def test_buffered_hot_path_write_is_transmit_only() -> None:
    """5 ms sync-write 는 arm/recover/delay 를 지불하지 않아야 한다."""
    body = function_body(
        SERVO_BUS, "HAL_StatusTypeDef Servo_SyncWritePositions("
    )
    for blocking_call in (
        "ServoBus_PrepareTransaction(",
        "ServoBus_Recover(",
        "ServoBus_ArmReceiver(",
        "ServoBus_WaitForIdleHighStable(",
        "HAL_Delay(",
    ):
        assert blocking_call not in body


def test_host_budget_stays_under_the_mcu_watchdog() -> None:
    """
    host 가 MCU 보다 먼저 포기하면 안 된다.

    MCU watchdog 은 heartbeat 를 처리할 때 먹으므로, 응답이 늦는 상황은
    watchdog 이 이미 흐르고 있는 상황과 같다. host 예산이 그보다 짧으면
    바쁜 링크를 죽은 링크로 오판한다.
    """
    transport = (
        ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/transport.py"
    ).read_text(encoding="utf-8")
    host_budget_s = float(
        re.search(r"HEARTBEAT_RESPONSE_TIMEOUT_S = ([0-9.]+)", transport).group(1)
    )
    declared_mcu_s = float(
        re.search(
            r"MCU_HEARTBEAT_WATCHDOG_TIMEOUT_S = ([0-9.]+)", transport
        ).group(1)
    )
    firmware_ms = constant(CONFIG, "HOST_BINARY_HEARTBEAT_TIMEOUT_MS")

    # host 가 적어둔 MCU 한계가 실제 firmware 값과 같아야 한다.
    assert declared_mcu_s * 1000.0 == firmware_ms
    # 그리고 host 예산은 그 아래에 있어야 한다.
    assert host_budget_s < declared_mcu_s
    # 다음 heartbeat 송신 여유도 남겨야 한다. timer 주기는 100 ms 다.
    assert host_budget_s + 0.1 <= declared_mcu_s
