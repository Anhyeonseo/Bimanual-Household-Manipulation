"""leg startup 진단 수집 계층의 계약.

**이 계층이 없어서 생긴 일을 먼저 적어 둔다.** 실행기는 leg 마다
`precompute_ms` 와 `first_sample_lead_ms` 를 이미 찍고 있었는데 pilot 이 그
stdout 을 버렸다. 그래서 "buffered leg 을 6~7회 반복하면 첫 프레임이
거부된다"는 관찰이 **한 번도 수치로 기록되지 않았다.**

여기서 지키는 것은 두 가지다.

  1. 성공 출력과 **실패 출력**을 똑같이 읽어야 한다. 우리가 쫓는 사건은
     실패 경로에만 나타나고, 실패 경로는 이름이 다르다(`lead_ms`).
  2. leg 정체와 회차를 섞지 않아야 한다. `precompute_ms` 는 sample 수에
     비례하므로 긴 leg 과 짧은 leg 을 한 줄에 놓고 기울기를 재면 회차
     추세가 아니라 leg 종류를 재게 된다.

ROS 없이, 직렬 포트 없이 검증한다.
"""

from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "single_arm_bridge"))

import buffered_leg_telemetry as TELEMETRY  # noqa: E402


# 실행기가 실제로 찍는 형태. `execute_buffered_segment_leg_once.py` 의
# 출력 순서를 그대로 따른다.
SUCCESS_OUTPUT = """PLAN_SHA256=deadbeef
SEGMENT_COUNT=7
PLAN_DURATION_MS=35000
PLAN_SAMPLE_COUNT=1751
PLAN_GATE=PASS
FRESH_START_MAX_ERROR_RAD=0.004120
FRESH_START_GATE=PASS
ACTION_SEND_COUNT=1
ACTION_TERMINAL_PASS status=4 error_code=0 \
maximum_apply_lateness_ms=3 post_settle_max_error_raw=6
TERMINAL_DIAGNOSTICS=precompute_ms=412.318 reanchor_ms=18.402 \
prime_frame_1_ms=5.110 fresh_tick=100523 \
prime_tick=100587 first_sample_lead_ms=136 prime_heartbeat_gates=1 \
prime_frames=2 accepted=16 applied=0 queued=16; lateness_buckets=1746/5/0/0; \
post_settle_error_raw=[2, 6, 3, 1, 0, 0]
TARGET_MAX_ERROR_RAD=0.003100
AUTOMATIC_RETRY_COUNT=0
MOTION14_FRESH_SEGMENT_LEG_ONCE_PASS
"""

# `start_goal` 이 lead 게이트에서 fail-closed 할 때의 형태. 이때
# `_startup_diagnostics` 는 아직 없으므로 precompute 만 실려 나오고,
# lead 는 `lead_ms` 라는 **다른 이름**으로 예외 메시지에 들어간다.
FAILURE_OUTPUT = """PLAN_DURATION_MS=35000
PLAN_SAMPLE_COUNT=1751
PLAN_GATE=PASS
ACTION_SEND_COUNT=1
RuntimeError: buffered start failed: stage=prime_frame_2_heartbeat_3 \
precompute_ms=980.412 reanchor_ms=94.220 prime_frame_1_ms=6.030 \
error=startup first-sample lead gate failed: \
lead_ms=57 required=80..160 heartbeat_gates=3
"""


# ---------------------------------------------------------------------------
# 성공 출력을 읽는다
# ---------------------------------------------------------------------------


def test_the_startup_numbers_are_recovered_from_a_passing_leg() -> None:
    record = TELEMETRY.parse_leg_telemetry(SUCCESS_OUTPUT)
    assert record["precompute_ms"] == pytest.approx(412.318)
    assert record["first_sample_lead_ms"] == 136
    assert record["prime_heartbeat_gates"] == 1
    assert record["fresh_tick_ms"] == 100523
    assert record["prime_tick_ms"] == 100587


def test_the_leg_size_is_recovered_so_precompute_can_be_normalised() -> None:
    """precompute 는 sample 수에 비례한다. 나누지 않으면 비교가 안 된다."""
    record = TELEMETRY.parse_leg_telemetry(SUCCESS_OUTPUT)
    assert record["sample_count"] == 1751
    assert record["duration_ms"] == 35000


def test_only_the_phases_after_the_heartbeat_count_against_the_lead() -> None:
    """`precompute` 는 fresh heartbeat 앞에서 끝난다. lead 를 못 깎는다.

    2026-08-06 에 precompute 가 두 배로 늘어난 것을 멈춤의 원인으로
    지목했다가 코드 구조로 반증됐다. 합계에 섞으면 그 오해가 되살아난다.
    """
    record = TELEMETRY.parse_leg_telemetry(SUCCESS_OUTPUT)
    assert record["reanchor_ms"] == pytest.approx(18.402)
    assert record["prime_frame_1_ms"] == pytest.approx(5.110)
    assert record["lead_consuming_ms"] == pytest.approx(23.512)
    assert record["precompute_ms"] not in (
        record["lead_consuming_ms"],
    )


def test_the_rejected_leg_shows_which_phase_ate_the_lead() -> None:
    """실패 경로도 구간을 갈라 보고해야 원인을 지목할 수 있다."""
    record = TELEMETRY.parse_leg_telemetry(FAILURE_OUTPUT)
    assert record["reanchor_ms"] == pytest.approx(94.220)
    assert record["lead_consuming_ms"] == pytest.approx(100.25)


def test_the_bridge_emits_the_phase_breakdown_this_module_reads() -> None:
    """두 파일이 갈라지면 이 계층이 조용히 빈 값을 읽는다."""
    source = (
        ROOT / "ros2_ws" / "src" / "single_arm_bridge" / "single_arm_bridge"
        / "buffered_action_execution.py"
    ).read_text(encoding="utf-8")
    for field in ("reanchor_ms", "prime_frame_1_ms", "precompute_ms"):
        assert f'f"{field}=' in source or f'"{field}",' in source, field
    # 계측은 fresh heartbeat 뒤에서 시작해야 한다. 앞에서 재면 lead 와
    # 무관한 시간을 lead 소비로 세게 된다.
    assert source.index("heartbeat = self._transport.heartbeat()") < (
        source.index("reanchor_started = time.monotonic()")
    )


def test_the_motion_quality_numbers_come_along_for_free() -> None:
    """속도를 올릴 때 이 두 값이 안전 게이트다. 같은 자리에서 거둔다."""
    record = TELEMETRY.parse_leg_telemetry(SUCCESS_OUTPUT)
    assert record["maximum_apply_lateness_ms"] == 3
    assert record["post_settle_max_error_raw"] == 6


# ---------------------------------------------------------------------------
# 실패 출력이 진짜 증거다
# ---------------------------------------------------------------------------


def test_the_rejected_lead_is_recovered_under_its_other_name() -> None:
    """실패 경로는 `lead_ms` 라고 쓴다. 이걸 놓치면 사건 자체를 놓친다."""
    record = TELEMETRY.parse_leg_telemetry(FAILURE_OUTPUT)
    assert record["first_sample_lead_ms"] == 57
    assert record["prime_heartbeat_gates"] == 3
    assert record["precompute_ms"] == pytest.approx(980.412)


def test_the_gate_bound_in_the_message_is_not_mistaken_for_a_measurement() -> None:
    """`required=80..160` 은 게이트지 관측이 아니다."""
    record = TELEMETRY.parse_leg_telemetry(FAILURE_OUTPUT)
    assert record["first_sample_lead_ms"] == 57


def test_the_canonical_name_wins_over_the_alias() -> None:
    """둘 다 있으면 성공 경로의 이름을 쓴다."""
    record = TELEMETRY.parse_leg_telemetry(
        "first_sample_lead_ms=136 lead_ms=57"
    )
    assert record["first_sample_lead_ms"] == 136


def test_an_output_with_no_diagnostics_yields_nothing_rather_than_zeros() -> None:
    """없는 값을 0 으로 채우면 추세가 조작된다."""
    assert TELEMETRY.parse_leg_telemetry("PLAN_GATE=PASS\n") == {}


# ---------------------------------------------------------------------------
# 추세 — 반복할수록 커지는가
# ---------------------------------------------------------------------------


def legs(precompute: list[float], leads: list[int], tag: str = "pregrasp"):
    return [
        {
            "ordinal": index,
            "run": index,
            "tag": tag,
            "ok": True,
            "precompute_ms": value,
            "first_sample_lead_ms": lead,
            "sample_count": 1751,
        }
        for index, (value, lead) in enumerate(
            zip(precompute, leads, strict=True), start=1
        )
    ]


def test_a_rising_precompute_shows_up_as_a_positive_slope() -> None:
    """가설이 맞다면 이 모양이다 — precompute 가 오르고 lead 가 내린다."""
    summary = TELEMETRY.summarise_leg_telemetry(
        legs([400.0, 420.0, 440.0, 460.0], [140, 130, 120, 110])
    )
    assert summary["precompute_ms"]["slope_per_leg"] == pytest.approx(20.0)
    assert summary["first_sample_lead_ms"]["slope_per_leg"] == pytest.approx(-10.0)
    assert summary["precompute_ms"]["drift"] == pytest.approx(60.0)


def test_a_flat_precompute_refutes_the_hypothesis() -> None:
    """커지지 않으면 원인이 다른 데 있다. 그것도 결론이다."""
    summary = TELEMETRY.summarise_leg_telemetry(
        legs([410.0, 409.0, 411.0, 410.0], [135, 136, 134, 135])
    )
    assert summary["precompute_ms"]["slope_per_leg"] == pytest.approx(0.0, abs=1.0)


def test_the_margin_to_both_gates_is_reported() -> None:
    """host 게이트(80)가 펌웨어 하한(60)보다 먼저 걸린다. 둘 다 봐야 한다."""
    summary = TELEMETRY.summarise_leg_telemetry(
        legs([400.0, 500.0], [140, 85])
    )
    gate = summary["lead_gate"]
    assert gate["minimum_observed_ms"] == 85
    assert gate["minimum_host_margin_ms"] == 5
    assert gate["minimum_firmware_margin_ms"] == 25


def test_the_host_gate_matches_the_bridge_that_enforces_it() -> None:
    """두 곳에 적힌 값이 갈라지면 여유 계산이 조용히 틀린다."""
    from single_arm_bridge.buffered_action_adapter import (
        INITIAL_FIRST_SAMPLE_LEAD_MS,
        MINIMUM_LEAD_MS,
    )
    from single_arm_bridge.buffered_action_execution import (
        STARTUP_FIRST_SAMPLE_LEAD_GATE_MS,
    )

    assert TELEMETRY.HOST_LEAD_GATE_FLOOR_MS == STARTUP_FIRST_SAMPLE_LEAD_GATE_MS
    assert TELEMETRY.HOST_LEAD_GATE_CEILING_MS == INITIAL_FIRST_SAMPLE_LEAD_MS
    assert TELEMETRY.FIRMWARE_LEAD_FLOOR_MS == MINIMUM_LEAD_MS


# ---------------------------------------------------------------------------
# leg 정체를 회차와 섞지 않는다
# ---------------------------------------------------------------------------


def test_mixed_leg_kinds_are_separated_before_the_slope_is_read() -> None:
    """긴 leg 과 짧은 leg 을 번갈아 돌면 전체 기울기는 leg 종류를 잰다.

    아래 자료에서 두 종류 다 **완전히 평탄**한데, 섞어 놓으면 전체 기울기가
    0 이 아니게 나온다. tag 별로 갈라야 "반복해도 안 커진다"가 보인다.
    """
    mixed = []
    for index in range(1, 7):
        long_leg, short_leg = (400.0, 90.0)
        mixed.append(
            {
                "ordinal": len(mixed) + 1, "run": index, "tag": "pregrasp",
                "ok": True, "precompute_ms": long_leg, "sample_count": 1751,
                "first_sample_lead_ms": 136,
            }
        )
        mixed.append(
            {
                "ordinal": len(mixed) + 1, "run": index, "tag": "lift",
                "ok": True, "precompute_ms": short_leg, "sample_count": 401,
                "first_sample_lead_ms": 150,
            }
        )
    summary = TELEMETRY.summarise_leg_telemetry(mixed)
    for tag in ("pregrasp", "lift"):
        assert summary["by_tag"][tag]["precompute_ms"]["slope_per_leg"] == (
            pytest.approx(0.0)
        )
        assert summary["by_tag"][tag]["leg_count"] == 6


def test_precompute_is_also_reported_per_sample() -> None:
    """leg 길이가 다른 경로끼리 비교하려면 비례분을 나눠야 한다."""
    summary = TELEMETRY.summarise_leg_telemetry(legs([400.0, 800.0], [140, 100]))
    per_sample = summary["precompute_ms_per_sample"]
    assert per_sample["first"] == pytest.approx(400.0 / 1751)
    assert per_sample["last"] == pytest.approx(800.0 / 1751)


def test_a_single_leg_of_a_kind_has_no_slope() -> None:
    """점 하나로 기울기를 내면 없는 추세를 만들어낸다."""
    summary = TELEMETRY.summarise_leg_telemetry(legs([400.0], [140]))
    assert "slope_per_leg" not in summary["precompute_ms"]
    assert "by_tag" not in summary


def test_failed_legs_are_counted_and_kept() -> None:
    """실패한 leg 을 빼면 우리가 찾던 바로 그 점이 사라진다."""
    records = legs([400.0, 420.0], [140, 57])
    records[-1]["ok"] = False
    summary = TELEMETRY.summarise_leg_telemetry(records)
    assert summary["leg_count"] == 2
    assert summary["failed_leg_count"] == 1
    assert summary["lead_gate"]["minimum_observed_ms"] == 57
    assert summary["lead_gate"]["minimum_host_margin_ms"] == -23


def test_the_module_never_touches_hardware() -> None:
    source = (ROOT / "tools" / "buffered_leg_telemetry.py").read_text(
        encoding="utf-8"
    )
    for forbidden in ("serial", "rclpy", "subprocess", "ActuatorTransport"):
        assert forbidden not in source


def test_the_trend_fits_on_a_terminal() -> None:
    """파일을 열지 않고도 추세가 보여야 물리 시험 중에 판단할 수 있다."""
    summary = TELEMETRY.summarise_leg_telemetry(
        legs([400.0, 420.0, 440.0], [140, 120, 100])
    )
    lines = TELEMETRY.format_leg_trend(summary)
    body = "\n".join(lines)
    assert "PRECOMPUTE_MS=" in body
    assert "FIRST_SAMPLE_LEAD_MS=" in body
    assert "LEAD_MARGIN_MS" in body
