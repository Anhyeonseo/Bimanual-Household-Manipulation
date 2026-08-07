"""추종률 가정 — 이 저장소의 속도 손잡이가 지켜야 할 것들.

**먼저 잘못 알려진 것을 적어 둔다.** MoveIt 의 `max_velocity_scaling_factor`
(`0.15`/`0.20`)는 buffered leg 경로의 속도와 **무관하다**.
`plan_buffered_segment_leg` 가 MoveIt 타이밍을 통째로 버리고
`select_duration_ms` 로 시간을 다시 정하기 때문이다. 진짜 손잡이는
`CONSERVATIVE_TRACKING_RATE_RAW_S` 하나다.

이 손잡이는 위험하다. peak/terminal 오차 게이트를 **같은 rate 로** 계산하기
때문에, rate 를 올리면 게이트가 같이 느슨해져 self-consistent 하게 통과한다.
모델은 아무것도 막아주지 않는다. 그래서 여기서 지키는 것은:

  1. 기본값이 조용히 바뀌지 않을 것 — 실측 없이 빨라지면 안 된다
  2. 검토된 상한이 계획기·실행기 **양쪽**에서 독립으로 걸릴 것
  3. 실행기가 계획이 스스로 밝힌 rate 로 재계산할 것 — 아니면 rate 를 바꾼
     계획은 실행 자체가 불가능해진다

ROS 없이, 직렬 포트 없이 검증한다.
"""

from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "single_arm_bridge"))

import plan_buffered_pick_place_leg as LEG  # noqa: E402
import plan_buffered_pick_pregrasp as PREGRASP  # noqa: E402

SEGMENT_SOURCE = (ROOT / "tools" / "plan_buffered_segment_leg.py").read_text(
    encoding="utf-8"
)
EXECUTE_SOURCE = (
    ROOT / "tools" / "execute_buffered_segment_leg_once.py"
).read_text(encoding="utf-8")

START = (2048,) * 6
TARGET = (2848, 2048, 2048, 2048, 2048, 2048)
ACTUAL = tuple(float(value) for value in START)


# ---------------------------------------------------------------------------
# 손잡이가 실제로 먹는가
# ---------------------------------------------------------------------------


def test_raising_the_assumed_rate_shortens_the_leg() -> None:
    """이 관계가 깨지면 손잡이가 아니라 장식이다."""
    durations = [
        LEG.select_duration_ms(ACTUAL, START, TARGET, rate)
        for rate in (50.0, 70.0, 90.0, 120.0, 150.0)
    ]
    assert durations == sorted(durations, reverse=True)
    # 800 raw 이동에서 50 → 150 raw/s 는 3배 언저리다.
    assert durations[0] / durations[-1] == pytest.approx(3.0, abs=0.5)


def test_the_default_is_still_the_conservative_2026_08_04_value() -> None:
    """실측으로 대체하기 전까지 기본값이 움직이면 안 된다.

    `50 raw/s` 는 Motion-11 이 관측한 **post-terminal** 추종률 `60 raw/s` 를
    깎은 값이다. 서보의 능력치가 아니라 뒤처진 팔이 따라잡던 속도다.
    """
    assert PREGRASP.CONSERVATIVE_TRACKING_RATE_RAW_S == 50.0
    assert LEG.select_duration_ms(ACTUAL, START, TARGET) == (
        LEG.select_duration_ms(
            ACTUAL, START, TARGET, PREGRASP.CONSERVATIVE_TRACKING_RATE_RAW_S
        )
    )


# ---------------------------------------------------------------------------
# 상한 — 모델이 못 막으므로 값 자체를 막는다
# ---------------------------------------------------------------------------


def test_the_ceiling_sits_below_the_independent_joint_velocity_limit() -> None:
    """`validate_buffered_trajectory` 의 `0.5 rad/s` 가 유일한 독립 게이트다.

    0.5 rad/s = 4096 raw / 2π rad × 0.5 ≈ 326 raw/s. 상한이 그 위로 가면
    rate 가정을 막아 주는 것이 아무것도 남지 않는다.
    """
    import math

    joint_limit_raw_s = 0.5 * 4096.0 / (2.0 * math.pi)
    assert LEG.MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S < joint_limit_raw_s


@pytest.mark.parametrize("rate", [0.0, -10.0, 301.0, 1000.0])
def test_the_simulation_refuses_an_unreviewed_rate(rate: float) -> None:
    with pytest.raises(ValueError, match="tracking rate"):
        LEG.simulate_stage_tracking(ACTUAL, START, TARGET, 10_000, rate)


def test_the_planner_cli_enforces_the_same_ceiling() -> None:
    assert "--tracking-rate-raw-s" in SEGMENT_SOURCE
    assert "MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S" in SEGMENT_SOURCE


def test_the_executor_enforces_the_ceiling_independently() -> None:
    """계획 파일이 자기일관적이기만 하면 통과해 버리는 것을 막는다.

    rate 는 duration 을 통해 모든 sample 에 퍼지므로 값을 손으로 고치면
    재현 검사가 깨진다 — 그러나 **처음부터** 검토되지 않은 rate 로 만든
    계획은 자기일관적이라 재현 검사만으로는 걸리지 않는다.
    """
    assert "MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S" in EXECUTE_SOURCE
    assert "plan tracking rate is outside the reviewed range" in EXECUTE_SOURCE


# ---------------------------------------------------------------------------
# 계획과 실행이 같은 rate 를 쓴다
# ---------------------------------------------------------------------------


def test_the_plan_records_the_rate_it_was_built_with() -> None:
    assert '"conservative_rate_raw_s": float(tracking_rate_raw_s)' in (
        SEGMENT_SOURCE
    )
    assert '"default_rate_raw_s"' in SEGMENT_SOURCE
    assert '"maximum_authorized_rate_raw_s"' in SEGMENT_SOURCE


def test_the_executor_rebuilds_with_the_rate_the_plan_declares() -> None:
    """기본값으로 재계산하면 rate 를 바꾼 계획은 전부 실행 불가가 된다."""
    assert "float(tracking_rate_raw_s)," in EXECUTE_SOURCE
    rebuild = EXECUTE_SOURCE.index("rebuilt = build_plan(")
    check = EXECUTE_SOURCE.index("plan tracking rate is outside")
    # 상한 검사가 재계산보다 먼저다. 검토되지 않은 rate 로 2000개 sample 을
    # 만들어 본 뒤에 거부할 이유가 없다.
    assert check < rebuild


def test_each_leg_gets_its_own_independent_rate_flag() -> None:
    """leg 마다 관절 이동량이 다르다 -- 하나의 rate 를 셋 다에 적용하면
    안 된다(2026-08-07 밤, q0_return 에 300 을 기본값으로 흘려보냈다가
    관절 속도 하드 게이트를 넘겨 계획이 거부된 실기 사례)."""
    pilot = (ROOT / "tools" / "run_grasp_repeatability_pilot.py").read_text(
        encoding="utf-8"
    )
    assert "--pregrasp-tracking-rate-raw-s" in pilot
    assert "--grasp-tracking-rate-raw-s" in pilot
    assert "--q0-return-tracking-rate-raw-s" in pilot


# ---------------------------------------------------------------------------
# 램프 도구
# ---------------------------------------------------------------------------


def test_the_ramp_refuses_a_non_increasing_schedule() -> None:
    """순서가 뒤섞이면 "어느 rate 에서 무너졌는가"를 말할 수 없다."""
    import run_speed_ramp_pilot as RAMP

    assert RAMP.parse_rates("50,70,90") == (50.0, 70.0, 90.0)
    for bad in ("90,70", "50,50", "0,70", "50,500"):
        with pytest.raises(ValueError):
            RAMP.parse_rates(bad)


def test_the_ramp_stops_below_the_bridge_gate() -> None:
    """정지선이 경성 게이트 위면 bridge 가 먼저 fail-closed 된다."""
    import run_speed_ramp_pilot as RAMP

    assert RAMP.BRIDGE_POST_SETTLE_GATE_RAW == 30
    assert RAMP.DEFAULT_STOP_RAW < RAMP.BRIDGE_POST_SETTLE_GATE_RAW
    source = (ROOT / "tools" / "run_speed_ramp_pilot.py").read_text(
        encoding="utf-8"
    )
    assert "strictly below the bridge gate" in source


def test_the_ramp_reports_the_speedup_the_operator_will_feel() -> None:
    import run_speed_ramp_pilot as RAMP

    steps = [
        {"step": 1, "rate_raw_s": 50.0, "ok": True, "duration_ms": 24000,
         "post_settle_max_error_raw": 6, "maximum_apply_lateness_ms": 3},
        {"step": 2, "rate_raw_s": 150.0, "ok": True, "duration_ms": 8000,
         "post_settle_max_error_raw": 14, "maximum_apply_lateness_ms": 4},
    ]
    summary = RAMP.summarise(steps, RAMP.DEFAULT_STOP_RAW)
    assert summary["highest_passing_rate_raw_s"] == 150.0
    assert summary["duration_ms"]["speedup_factor"] == pytest.approx(3.0)
    settle = summary["post_settle_max_error_raw"]
    assert settle["margin_to_stop_raw"] == 6
    assert settle["margin_to_bridge_gate_raw"] == 16


def test_an_unmeasured_post_settle_stops_the_ramp() -> None:
    """근거를 못 읽었으면 올리지 않는다. 없는 값은 통과가 아니다."""
    source = (ROOT / "tools" / "run_speed_ramp_pilot.py").read_text(
        encoding="utf-8"
    )
    assert "post-settle 값을 읽지 못했다" in source
    assert "if settle is None:" in source


def test_the_ramp_never_opens_a_serial_port() -> None:
    source = (ROOT / "tools" / "run_speed_ramp_pilot.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "open_exclusive_serial",
        "serial.Serial",
        "ActuatorTransport",
        "ActionClient",
    ):
        assert forbidden not in source
    assert '"serial_port_opened": False' in source


def test_the_ramp_has_no_automatic_retry() -> None:
    source = (ROOT / "tools" / "run_speed_ramp_pilot.py").read_text(
        encoding="utf-8"
    )
    assert '"automatic_retry_count": 0' in source
    assert "stopped_reason" in source
