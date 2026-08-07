"""수렴 적용 Pick/Place pilot 의 계약.

Motion-13 은 3-leg 경로를 완주했지만 물체를 집지 못했다. C1/C2 가 도달 층을
닫고 파지 offset 을 다시 쟀으므로 그 조건으로 전 주기를 다시 돈다.

**주기 설계에 2026-08-06 실측이 들어가 있다.** 파지 자세에서 pregrasp 로
올리는 이동이 post-settle `32 raw` 로 안전 게이트를 넘겼으므로, 그 상승을
주기에서 제거하고 `20 mm` 만 들어 옆으로 간다.

ROS 없이 검증한다.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "single_arm_bridge"))

PILOT_PATH = ROOT / "tools" / "run_pick_place_pilot.py"
_spec = importlib.util.spec_from_file_location("run_pick_place_pilot", PILOT_PATH)
PILOT = importlib.util.module_from_spec(_spec)
sys.modules["run_pick_place_pilot"] = PILOT
_spec.loader.exec_module(PILOT)

SOURCE = PILOT_PATH.read_text(encoding="utf-8")


def record(run: int, pick_mm: float, gap: int, place_mm: float) -> dict:
    return {
        "run": run,
        "pick_residual_mm": pick_mm,
        "pick_gap_raw": gap,
        "picked": gap >= PILOT.CONTACT_THRESHOLD_RAW,
        "place_residual_mm": place_mm,
        "release_gap_raw": 6,
    }


# ---------------------------------------------------------------------------
# 주기 설계 — 오늘 실측이 제약이다
# ---------------------------------------------------------------------------


def test_the_cycle_never_lifts_from_grasp_to_pregrasp() -> None:
    """2026-08-06 A5 1회차가 정확히 그 이동에서 중단됐다.

    SHOULDER 259 raw 상승, post-settle 32 raw (안전 허용치 30 초과),
    14회 관측이 2685 ms 동안 동일한 평형이었다.
    """
    assert "lift20" in SOURCE
    assert "파지 자세에서 pregrasp 로 올리지 않는다" in SOURCE
    # 놓는 자리는 pregrasp 를 거치지 않고 곧장 grasp 로 간다.
    assert '"place_grasp", target_plan, "grasp"' in SOURCE
    assert '"place_pregrasp"' not in SOURCE


def test_the_cycle_ends_by_folding_to_q0() -> None:
    """접는 방향 상승은 1536 raw 를 들고도 post-settle 6 raw 로 통과했다."""
    assert "return_to_q0(work, arm_names, calibration, leg_log, index)" in SOURCE


# ---------------------------------------------------------------------------
# 무엇을 재는가
# ---------------------------------------------------------------------------


def test_pick_and_place_residuals_are_reported_separately() -> None:
    """두 자세의 처짐은 다르다. 합쳐서 보고하면 어느 쪽 문제인지 못 가른다."""
    runs = [record(1, 10.5, 22, 9.1), record(2, 10.6, 21, 9.4)]
    summary = PILOT.summarise(runs)
    assert summary["pick_residual_mm"]["spread"] == pytest.approx(0.1)
    assert summary["place_residual_mm"]["spread"] == pytest.approx(0.3)
    assert summary["picked_count"] == 2


def test_a_missed_pick_is_counted_by_the_calibrated_threshold() -> None:
    runs = [record(1, 10.5, 22, 9.1), record(2, 10.6, 5, 9.4)]
    summary = PILOT.summarise(runs)
    assert summary["picked_count"] == 1
    assert summary["contact_threshold_raw"] == 14


def test_the_threshold_matches_the_repeatability_pilot() -> None:
    """두 도구가 같은 판별값을 써야 결과를 나란히 놓을 수 있다."""
    import run_grasp_repeatability_pilot as REPEAT

    assert PILOT.CONTACT_THRESHOLD_RAW == REPEAT.CONTACT_THRESHOLD_RAW


# ---------------------------------------------------------------------------
# 규율
# ---------------------------------------------------------------------------


def test_the_pilot_requires_its_own_exact_confirmation() -> None:
    assert PILOT.CONFIRMATION == "RUN_A5_PICK_PLACE_PILOT"
    assert "exact pick/place pilot confirmation is required" in SOURCE


def test_convergence_shortfall_does_not_stop_the_cycle() -> None:
    """허용치를 못 채워도 그 잔차가 측정 대상이다."""
    assert "허용치를 채우지 못해도 계속한다" in SOURCE


def test_there_is_no_automatic_retry() -> None:
    assert '"automatic_retry_count": 0' in SOURCE
    assert "stopped_reason" in SOURCE


def test_the_pilot_never_opens_a_serial_port() -> None:
    for forbidden in (
        "open_exclusive_serial",
        "serial.Serial",
        "ActuatorTransport",
        "ActionClient",
    ):
        assert forbidden not in SOURCE
    assert '"serial_port_opened": False' in SOURCE


def test_every_move_reuses_the_validated_helpers() -> None:
    """새 이동 경로를 만들지 않는다."""
    for helper in ("move_to", "return_to_q0", "gripper", "sha256_file"):
        assert f"    {helper}," in SOURCE or f"{helper}(" in SOURCE


def test_the_pilot_names_no_arm_side_in_its_logic() -> None:
    body = "\n".join(
        line
        for line in SOURCE.splitlines()
        if "default=" not in line and not line.lstrip().startswith("#")
    )
    assert "left_" not in body


def test_every_pose_plan_is_digest_recorded() -> None:
    assert '"pose_plan_sha256"' in SOURCE
    assert '"grasp": sha256_file(plan)' in SOURCE
    assert '"lift": sha256_file(lift)' in SOURCE


# ---------------------------------------------------------------------------
# 물체는 왕복한다
# ---------------------------------------------------------------------------


def test_the_cycle_alternates_source_and_target() -> None:
    """한 주기가 끝나면 물체는 놓은 자리에 있다.

    같은 자리를 다시 집으러 가면 빈 자리를 집는다. 회차마다 출발과 도착을
    바꿔야 사람이 물체를 되돌리지 않고 반복할 수 있다.
    """
    assert 'source_name = "A" if index % 2 == 1 else "B"' in SOURCE
    assert 'target_name = "B" if source_name == "A" else "A"' in SOURCE
    assert '"alternates": True' in SOURCE


def test_each_run_records_which_direction_it_ran() -> None:
    """두 자세의 처짐이 다르므로 방향을 알아야 자료를 나눌 수 있다."""
    assert '"source_pose": source_name' in SOURCE
    assert '"target_pose": target_name' in SOURCE


# ---------------------------------------------------------------------------
# leg startup 진단 — 왕복 주기는 leg 을 5개씩 쌓는다
# ---------------------------------------------------------------------------


def test_every_leg_execution_is_logged() -> None:
    """한 주기가 leg 을 5개 돌린다. 하나라도 빠지면 추세에 구멍이 난다."""
    import test_grasp_repeatability_pilot as REPEAT_TESTS

    calls = REPEAT_TESTS.leg_calls(SOURCE)
    assert len(calls) == 5
    for name, node in calls:
        passed = [
            argument for argument in node.args
            if getattr(argument, "id", None) == "leg_log"
        ] + [keyword for keyword in node.keywords if keyword.arg == "log"]
        assert passed, f"{name} 호출이 leg_log 를 받지 않는다"


def test_the_evidence_carries_the_leg_trend() -> None:
    assert '"startup_trend": startup' in SOURCE
    assert '"legs": leg_log.legs' in SOURCE
    assert '"schema_version": 2' in SOURCE


def test_a_stopped_pilot_still_writes_its_leg_diagnostics() -> None:
    """첫 주기에서 멈췄다면 그 멈춤 자체가 찾던 것일 수 있다."""
    assert "summary = summarise(runs) if runs else None" in SOURCE
    assert SOURCE.index("arguments.output.write_text(text") < SOURCE.index(
        "PICK_PLACE_PILOT_FAIL"
    )


def test_both_poses_need_their_own_lift_plan() -> None:
    """드는 자세는 집는 자리마다 다르다."""
    for flag in ("--pose-a-lift-plan", "--pose-b-lift-plan"):
        assert flag in SOURCE
    assert "source_lift" in SOURCE
