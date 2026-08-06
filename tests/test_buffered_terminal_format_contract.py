"""
bridge 가 만드는 성공 terminal 문자열과 sender 가 파싱하는 패턴이 일치하는지
양쪽 소스를 파싱해 강제한다.

2026-08-06 Motion-11 실기에서 실제로 갈라졌다. bridge 는
`buffered_action_execution.py` 에서 성공 terminal 뒤에 `"; {startup}"` 로
startup 진단을 덧붙이는데, sender 의 TERMINAL_PATTERN 은
`post_settle_max_error_raw=(\\d+)$` 로 문자열 끝을 요구했다. 그 결과
47초 경로가 물리적으로 완주하고 post-settle 도 통과했는데도 sender 가
"terminal evidence is missing" 으로 실패했다.

두 시험 파일이 각자의 가정으로만 문자열을 만들었기 때문에 이 드리프트가
보이지 않았다. 여기서는 bridge 소스의 실제 f-string 을 재구성해 sender 의
패턴에 넣는다.

소스 파싱 시험 선례: tests/test_left_arm_q0_contract.py,
tests/test_stm32_servo_uart_circular_dma_contract.py
"""

import importlib.util
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXECUTION_SOURCE = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "buffered_action_execution.py"
).read_text(encoding="utf-8")

SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_action_plan_once_terminal_contract",
    ROOT / "tools" / "execute_buffered_action_plan_once.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_bridge_success_terminal_literal_is_unchanged() -> None:
    """bridge 쪽 f-string 이 바뀌면 이 시험이 먼저 깨져야 한다."""
    assert '"buffered trajectory completed; "' in EXECUTION_SOURCE
    assert 'f"maximum_apply_lateness_ms={result.detail} "' in EXECUTION_SOURCE
    assert (
        'f"post_settle_max_error_raw={settle.max_error_raw}; "' in EXECUTION_SOURCE
    )
    assert (
        'f"{startup}; {lateness}; {settle.terminal_summary()}",'
        in EXECUTION_SOURCE
    )


def build_measurement(settle: int) -> str:
    """bridge 의 PostSettleMeasurement.terminal_summary() 를 재구성한다."""
    target = (2276, 3437, 1550, 1201, 2112, 2003)
    error = (settle, 3, 2, 5, 1, 0)
    measured = tuple(t + e for t, e in zip(target, error))
    joined = lambda values: ",".join(str(value) for value in values)
    return (
        f"post_settle_target_raw={joined(target)} "
        f"post_settle_measured_raw={joined(measured)} "
        f"post_settle_error_raw={joined(error)}"
    )


def build_bridge_terminal(
    lateness: int,
    settle: int,
    startup: str,
    profile: str = "lateness_buckets=2340,0,0,0,0,0 lateness_worst_sample=none",
    measurement: str | None = None,
) -> str:
    """bridge 가 실제로 만드는 문자열을 그대로 재구성한다."""
    if measurement is None:
        measurement = build_measurement(settle)
    return (
        "buffered trajectory completed; "
        f"maximum_apply_lateness_ms={lateness} "
        f"post_settle_max_error_raw={settle}; {startup}; {profile}; "
        f"{measurement}"
    )


def test_sender_pattern_accepts_the_real_bridge_terminal() -> None:
    startup = (
        "startup=prime_depth=16 first_lead_ms=160 "
        "firmware_elapsed_ms=61 heartbeat_gates=2"
    )
    profile = "lateness_buckets=2330,8,1,0,0,1 lateness_worst_sample=1841"
    text = build_bridge_terminal(4, 18, startup, profile)
    match = MODULE.TERMINAL_PATTERN.fullmatch(text)
    assert match is not None, "sender 패턴이 실제 bridge terminal 을 못 읽는다"
    assert int(match.group(1)) == 4
    assert int(match.group(2)) == 18
    assert match.group("diagnostics") == (
        f"{startup}; {profile}; {build_measurement(18)}"
    )


def test_sender_pattern_still_accepts_the_legacy_terminal() -> None:
    """startup 접미사가 없던 옛 형식도 계속 읽혀야 한다."""
    text = (
        "buffered trajectory completed; "
        "maximum_apply_lateness_ms=1 post_settle_max_error_raw=30"
    )
    match = MODULE.TERMINAL_PATTERN.fullmatch(text)
    assert match is not None
    assert int(match.group(1)) == 1
    assert int(match.group(2)) == 30
    assert match.group("diagnostics") is None


def test_startup_unavailable_fallback_parses() -> None:
    assert 'or "startup=unavailable"' in EXECUTION_SOURCE
    match = MODULE.TERMINAL_PATTERN.fullmatch(
        build_bridge_terminal(0, 0, "startup=unavailable")
    )
    assert match is not None
    assert match.group("diagnostics").startswith("startup=unavailable")


def test_validate_action_terminal_preserves_startup_evidence() -> None:
    from types import SimpleNamespace

    startup = "startup=prime_depth=16 first_lead_ms=160"
    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(4, 18, startup),
    )
    evidence = MODULE.validate_action_terminal(
        MODULE.ACTION_STATUS_SUCCEEDED,
        result,
    )
    assert evidence.maximum_apply_lateness_ms == 4
    assert evidence.post_settle_max_error_raw == 18
    assert evidence.terminal_diagnostics is not None
    assert evidence.terminal_diagnostics.startswith(startup)
    assert "lateness_buckets=" in evidence.terminal_diagnostics


def test_bounds_still_reject_out_of_range_values() -> None:
    from types import SimpleNamespace
    import pytest

    over_lateness = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(6, 0, "startup=unavailable"),
    )
    with pytest.raises(RuntimeError, match=r"outside 0\.\.5"):
        MODULE.validate_action_terminal(
            MODULE.ACTION_STATUS_SUCCEEDED, over_lateness
        )

    over_settle = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(0, 31, "startup=unavailable"),
    )
    with pytest.raises(RuntimeError, match=r"outside 0\.\.30"):
        MODULE.validate_action_terminal(
            MODULE.ACTION_STATUS_SUCCEEDED, over_settle
        )


def test_pattern_rejects_a_truncated_terminal() -> None:
    for text in (
        "buffered trajectory completed; maximum_apply_lateness_ms=4",
        "buffered trajectory completed; post_settle_max_error_raw=18",
        "buffered trajectory aborted; "
        "maximum_apply_lateness_ms=4 post_settle_max_error_raw=18",
    ):
        assert MODULE.TERMINAL_PATTERN.fullmatch(text) is None


def test_the_per_joint_measurement_survives_to_the_sender() -> None:
    """최대값 하나로는 어느 관절이 못 갔는지 알 수 없다.

    2026-08-06 A4 성공 회차와 A4.5 실패 회차의 도달 자세를 되살릴 수 없었던
    이유가 정확히 이것이다. 벡터가 종단 문자열을 타고 sender 까지 와야
    수렴 계층이 TCP 잔차를 mm 로 계산할 수 있다.
    """
    from types import SimpleNamespace

    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(4, 19, "startup=unavailable"),
    )
    evidence = MODULE.validate_action_terminal(
        MODULE.ACTION_STATUS_SUCCEEDED, result
    )
    assert evidence.post_settle_target_raw == (
        2276, 3437, 1550, 1201, 2112, 2003
    )
    assert evidence.post_settle_error_raw == (19, 3, 2, 5, 1, 0)
    assert evidence.post_settle_measured_raw == (
        2295, 3440, 1552, 1206, 2113, 2003
    )
    assert max(evidence.post_settle_error_raw) == 19


def test_a_terminal_without_vectors_still_parses() -> None:
    """옛 형식으로 기록된 증거를 다시 읽을 수 있어야 한다."""
    from types import SimpleNamespace

    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=(
            "buffered trajectory completed; "
            "maximum_apply_lateness_ms=4 post_settle_max_error_raw=18; "
            "startup=unavailable"
        ),
    )
    evidence = MODULE.validate_action_terminal(
        MODULE.ACTION_STATUS_SUCCEEDED, result
    )
    assert evidence.post_settle_error_raw is None
    assert evidence.post_settle_max_error_raw == 18


def test_inconsistent_vectors_are_refused() -> None:
    """|실측 - 목표| 가 오차와 다르면 세 벡터 중 하나가 거짓이다."""
    import pytest

    with pytest.raises(RuntimeError, match="inconsistent"):
        MODULE.parse_post_settle_vectors(
            "post_settle_target_raw=2276,3437 "
            "post_settle_measured_raw=2295,3440 "
            "post_settle_error_raw=19,99"
        )


def test_a_vector_maximum_above_the_reported_maximum_is_refused() -> None:
    """보고된 최대값은 두 관측의 큰 쪽이므로 벡터 최대를 넘을 수는 없다."""
    from types import SimpleNamespace
    import pytest

    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(
            4, 5, "startup=unavailable", measurement=build_measurement(19)
        ),
    )
    with pytest.raises(RuntimeError, match="exceeds the reported"):
        MODULE.validate_action_terminal(MODULE.ACTION_STATUS_SUCCEEDED, result)


def test_the_vector_maximum_may_be_below_the_reported_maximum() -> None:
    """최종 full diagnostics 가 스냅샷보다 좋을 수 있다. 그건 정상이다."""
    from types import SimpleNamespace

    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=build_bridge_terminal(
            4, 22, "startup=unavailable", measurement=build_measurement(19)
        ),
    )
    evidence = MODULE.validate_action_terminal(
        MODULE.ACTION_STATUS_SUCCEEDED, result
    )
    assert evidence.post_settle_max_error_raw == 22
    assert max(evidence.post_settle_error_raw) == 19
