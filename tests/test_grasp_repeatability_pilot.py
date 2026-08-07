"""A5 재현성 pilot 의 계약.

**이 도구는 성공률을 재지 않는다.** 회차별 잔차의 흩어짐을 잰다.

2026-08-06 C2 는 파지가 잔차 `10.17 mm` 와 `7.75 mm` 에서 성립하는 것을 봤다.
과제 허용치 `4 mm` 는 필요조건이 아니었고, 남은 잔차는
`PICK_GRASP_OFFSET_M` 이 흡수한다. 그것이 성립하려면 잔차가 회차 간
재현되어야 한다 — A4 가 실패한 이유도 잔차가 컸기 때문이 아니라 회차마다
달랐기 때문이다.

ROS 없이 검증한다. 통계와 판정은 순수 함수이고 나머지는 소스 검사다.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

PILOT_PATH = ROOT / "tools" / "run_grasp_repeatability_pilot.py"
_spec = importlib.util.spec_from_file_location(
    "run_grasp_repeatability_pilot", PILOT_PATH
)
PILOT = importlib.util.module_from_spec(_spec)
sys.modules["run_grasp_repeatability_pilot"] = PILOT
_spec.loader.exec_module(PILOT)

SOURCE = PILOT_PATH.read_text(encoding="utf-8")


def record(residual_mm: float, gap: int, run: int = 1) -> dict:
    return {
        "run": run,
        "convergence_residual_mm": residual_mm,
        "residual_gap_raw": gap,
        "held": gap >= PILOT.CONTACT_THRESHOLD_RAW,
    }


# ---------------------------------------------------------------------------
# 무엇을 재는가
# ---------------------------------------------------------------------------


def test_the_summary_reports_the_spread_not_just_the_mean() -> None:
    """흩어짐이 결론이다. 평균만 보면 A4 의 실패를 못 본다."""
    runs = [record(r, 20, i) for i, r in enumerate((7.7, 8.1, 7.4, 9.0), 1)]
    summary = PILOT.summarise(runs)
    assert summary["residual_mm"]["spread"] == pytest.approx(1.6)
    assert summary["residual_mm"]["minimum"] == pytest.approx(7.4)
    assert summary["residual_mm"]["maximum"] == pytest.approx(9.0)
    assert "stdev" in summary["residual_mm"]


def test_a_single_run_has_no_stdev() -> None:
    summary = PILOT.summarise([record(7.7, 20)])
    assert "stdev" not in summary["residual_mm"]
    assert summary["residual_mm"]["spread"] == 0


def test_holds_are_counted_by_the_calibrated_threshold() -> None:
    """`reached_goal` 이 아니라 잔여 간격으로 판정한다."""
    runs = [record(7.7, 22, 1), record(8.0, 5, 2), record(7.9, 14, 3)]
    summary = PILOT.summarise(runs)
    assert summary["held_count"] == 2
    assert summary["contact_threshold_raw"] == 14


def test_the_threshold_matches_the_contract() -> None:
    import json

    contract = json.loads(
        (
            ROOT
            / "ros2_ws/src/single_arm_bridge/config"
            / "buffered_trajectory_contract.json"
        ).read_text(encoding="utf-8")
    )
    offsets = contract["tcp_contact_offsets"]
    assert PILOT.CONTACT_THRESHOLD_RAW == offsets["contact_threshold_raw"]
    assert (
        offsets["control_close_residual_raw"] < PILOT.CONTACT_THRESHOLD_RAW
    )


# ---------------------------------------------------------------------------
# 규율
# ---------------------------------------------------------------------------


def test_the_pilot_requires_its_own_exact_confirmation() -> None:
    assert PILOT.CONFIRMATION == "RUN_A5_GRASP_REPEATABILITY_PILOT"
    assert "exact pilot confirmation is required" in SOURCE


def test_a_repeatability_pilot_needs_more_than_one_run() -> None:
    assert "at least two runs" in SOURCE


def test_there_is_no_automatic_retry() -> None:
    """한 회차라도 실패하면 멈추고 그때까지의 기록을 남긴다."""
    assert '"automatic_retry_count": 0' in SOURCE
    assert "stopped_reason" in SOURCE
    assert "break" in SOURCE


def test_a_stopped_pilot_still_writes_what_it_measured() -> None:
    """한 회차도 못 채웠어도 기록은 나와야 한다.

    첫 회차에서 멈췄다면 **그 멈춤 자체가** 이번 시험이 찾던 것일 수 있다.
    예전 구현은 `runs` 가 비면 아무것도 쓰지 않고 나갔다.
    """
    assert "'STOPPED'" in SOURCE
    assert "summary = summarise(runs) if runs else None" in SOURCE
    assert "arguments.output.write_text(text" in SOURCE
    # 파일을 쓴 뒤에 실패를 알린다. 순서가 뒤집히면 다시 증거를 잃는다.
    assert SOURCE.index("arguments.output.write_text(text") < SOURCE.index(
        "A5_PILOT_FAIL"
    )


def test_convergence_shortfall_does_not_stop_the_pilot() -> None:
    """수렴이 허용치를 못 채워도 그 잔차가 측정 대상이다."""
    assert "수렴이 허용치를 못 채워도 계속한다" in SOURCE


def test_the_pilot_never_opens_a_serial_port() -> None:
    for forbidden in (
        "open_exclusive_serial",
        "serial.Serial",
        "ActuatorTransport",
    ):
        assert forbidden not in SOURCE
    assert '"serial_port_opened": False' in SOURCE


def test_every_move_goes_through_the_validated_pipeline() -> None:
    for tool in (
        "ros_moveit_plan_pregrasp_segments.py",
        "plan_buffered_segment_leg.py",
        "execute_buffered_segment_leg_once.py",
        "execute_grasp_convergence_once.py",
        "execute_gripper_command_once.py",
    ):
        assert tool in SOURCE
    for forbidden in ("ActionClient", "FollowJointTrajectory"):
        assert forbidden not in SOURCE


def test_each_leg_is_digest_pinned() -> None:
    assert "--segments-sha256" in SOURCE
    assert "--expected-sha256" in SOURCE


def test_the_pilot_names_no_arm_side_in_its_logic() -> None:
    body = "\n".join(
        line
        for line in SOURCE.splitlines()
        if "default=" not in line and not line.lstrip().startswith("#")
    )
    assert "left_" not in body


def test_the_gripper_verdict_uses_the_gap_not_reached_goal() -> None:
    """`reached_goal` 은 SUCCEEDED 이면 항상 True 라 파지를 말해주지 못한다."""
    assert "RESIDUAL_GAP_RAW" in SOURCE
    assert "reached_goal" not in SOURCE
    assert '"--expect", "report"' in SOURCE


def test_each_cycle_returns_to_q0() -> None:
    """q0 경유는 취향이 아니라 유일하게 검증된 상승 경로다.

    2026-08-06 A5 1회차는 파지 자세에서 pregrasp 로 펼친 채 드는 이동에서
    post-settle `32 raw` 로 중단됐다(안전 허용치 30). 같은 날
    `grasp -> q0` 은 `1536 raw` 를 들고도 post-settle `6 raw` 로 통과했다.
    """
    assert PILOT.Q0_RETURN_CONFIRMATION == "EXECUTE_MOTION12_Q0_RETURN_ONCE"
    assert "plan_buffered_q0_return.py" in SOURCE
    assert "execute_buffered_q0_return_once.py" in SOURCE
    assert "return_to_q0(\n                work, arm_names, calibration, leg_log, index," in SOURCE


def test_the_cycle_start_is_not_inherited_from_the_previous_run() -> None:
    """앞 회차의 최종 자세가 스며들면 재현성이 아니라 누적을 재게 된다."""
    assert "앞 회차의 최종 자세가" in SOURCE


def test_short_term_repeatability_and_session_drift_are_separate() -> None:
    """평탄역 안에서는 결정론적이어도 세션 동안 계단이 생길 수 있다.

    2026-08-06 두 회차 실측:
      A: 8.303, 11.620, 10.503, 10.503, 10.597, 10.503
      B: 8.966,  8.852,  8.852, 11.073, 11.079, 11.050
    둘 다 낮은 값에서 높은 값으로 가지만 계단 지점이 다르다. 고정된
    워밍업 회차로 자르면 한 데이터에만 맞는다.
    """
    values = (8.966, 8.852, 8.852, 11.073, 11.079, 11.050)
    runs = [record(v, 22, i) for i, v in enumerate(values, 1)]
    summary = PILOT.summarise(runs)
    adjacent = summary["adjacent_difference_mm"]
    # 평탄역 안은 거의 0, 계단 하나만 크다.
    assert adjacent["maximum"] == pytest.approx(2.221)
    assert adjacent["median"] < 0.15
    assert summary["session_drift_mm"] == pytest.approx(2.227)


# ---------------------------------------------------------------------------
# leg startup 진단 — 여태 출력되고도 버려지던 것
# ---------------------------------------------------------------------------


def leg_calls(source: str) -> list:
    """`move_to`/`return_to_q0` 호출을 구문으로 찾는다.

    문자열로 찾으면 인자가 줄바꿈될 때 조용히 놓친다 — 놓치면 그 leg 만
    기록되지 않고, 빠진 자리는 추세에서 보이지 않는다.
    """
    import ast

    found = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "id", None)
        if name in ("move_to", "return_to_q0"):
            found.append((name, node))
    return found


def test_every_leg_execution_is_logged() -> None:
    """leg_log 없이 불리는 leg 이 하나라도 있으면 증거가 다시 사라진다."""
    calls = leg_calls(SOURCE)
    assert calls
    for name, node in calls:
        passed = [
            argument for argument in node.args
            if getattr(argument, "id", None) == "leg_log"
        ] + [
            keyword for keyword in node.keywords if keyword.arg == "log"
        ]
        assert passed, f"{name} 호출이 leg_log 를 받지 않는다"


def test_the_executor_output_is_saved_not_discarded() -> None:
    """실행기가 찍는 `TERMINAL_DIAGNOSTICS=` 가 이 파일에 남는다."""
    assert '(workdir / f"{tag}_execute.txt").write_text' in SOURCE
    assert "parse_leg_telemetry" in SOURCE


def test_a_failed_leg_keeps_its_output() -> None:
    """거부 사유는 실패 경로에만 있다. 종료 코드만 던지면 사라진다."""
    assert "class StepFailure" in SOURCE
    assert "failure.output" in SOURCE
    assert "ok=False" in SOURCE


def test_the_evidence_carries_the_leg_trend() -> None:
    assert '"startup_trend": startup' in SOURCE
    assert '"legs": leg_log.legs' in SOURCE
    assert '"schema_version": 2' in SOURCE


def test_the_leg_ordinal_counts_executions_not_cycles() -> None:
    """가설은 회차가 아니라 leg 실행 횟수에 걸려 있다."""
    log = PILOT.LegLog()
    log.record(tag="pregrasp", run_index=1, text="precompute_ms=1", ok=True)
    log.record(tag="grasp", run_index=1, text="precompute_ms=2", ok=True)
    log.record(tag="q0_return", run_index=1, text="precompute_ms=3", ok=True)
    log.record(tag="pregrasp", run_index=2, text="precompute_ms=4", ok=True)
    assert [leg["ordinal"] for leg in log.legs] == [1, 2, 3, 4]
    assert [leg["run"] for leg in log.legs] == [1, 1, 1, 2]


def test_a_single_run_has_no_adjacent_difference() -> None:
    summary = PILOT.summarise([record(8.3, 22, 1)])
    assert "adjacent_difference_mm" not in summary
    assert "session_drift_mm" not in summary
