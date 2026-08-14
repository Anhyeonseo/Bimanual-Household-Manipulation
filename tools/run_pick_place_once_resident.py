#!/usr/bin/env python3
"""H1 gate: `run_pick_place_once.py`와 같은 8단계를 상주 세션으로 실행한다.

    q0 -> pick_pregrasp -> pick_grasp -> [수렴+close]
       -> lift20 -> place_pregrasp -> place_grasp -> [수렴+release]
       -> retreat -> q0

원본과의 유일한 차이: `move_to()`/`return_to_q0()`가 leg 마다 subprocess
3개(ROS 노드 생성 + MoveIt/action 탐색을 매번 새로)를 띄우는 대신, 이 도구는
`ResidentArmSession` 하나를 세션 내내 유지하고
`resident_pick_place.move_to_resident()`/`return_to_q0_resident()`를 쓴다.
계획·검증·실행 로직은 원본과 완전히 동일한 함수를 재사용한다
(`docs/PLAN_CONTINUOUS_EXECUTION.md` §3.1 H1).

**수렴(convergence)과 gripper 명령은 아직 subprocess 그대로다.** H1의 첫
범위는 leg 이동(segment leg + q0 복귀)만이고, 그 사실을 출력에 명시한다 —
`grasp -> q0_return` 구간(원본 기준 표준 중앙값 23189 ms)은 이 도구만으로는
완전히 좁혀지지 않는다. 단발 도구에서 직접 생기는
`pick_pregrasp -> pick_grasp`만 H1 gate 대상으로 삼는다.
`q0_return -> pregrasp`는 반복 실행기 전환 뒤 별도로 잰다.

**gate는 wall-clock 단계 완료 시각이 아니라 firmware tick 진단으로 계산한다.**
이전 leg의 첫 적용 예정 시각+duration부터 다음 leg의 fresh heartbeat까지다.
단발 1회는 표본 하나뿐이므로 최종 PASS를 만들지 않는다. 이전 실행 artifact를
`--h1-gate-evidence`로 합쳐 3표본 이상일 때 중앙값 < 1000 ms를 판정한다.
`PHASE_INTERVAL_MS`는 전체 단계 체감시간 진단일 뿐 gate가 아니다.

**자동 재시도가 없다.** 한 단계라도 실패하면 멈추고 그때까지의 기록을 남긴다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from buffered_leg_telemetry import (  # noqa: E402
    collect_transition_dead_times,
    format_leg_trend,
    summarise_transition_dead_times,
    summarise_leg_telemetry,
)
from resident_arm_session import ResidentArmSession  # noqa: E402
from resident_pick_place import (  # noqa: E402
    move_to_resident,
    return_to_q0_resident,
    sha256_file,
)
from run_grasp_repeatability_pilot import (  # noqa: E402
    CONTACT_THRESHOLD_RAW,
    GRIPPER_CLOSE_CONFIRMATION,
    GRIPPER_CLOSE_RAD,
    GRIPPER_OPEN_CONFIRMATION,
    GRIPPER_OPEN_RAD,
    ARM_MOTION_PATTERN,
    RESIDUAL_GAP_PATTERN,
    LegLog,
    gripper,
)
from run_pick_place_once import converge  # noqa: E402

CONFIRMATION = "RUN_PICK_PLACE_ONCE_RESIDENT_2026_08_11"
STATUS = "PICK_PLACE_ONCE_RESIDENT"

# 같은 firmware tick 산식을 표준 median으로 다시 계산한 기준선.
# 기존 문서의 4178 ms는 짝수 표본 10개의 '상위 중앙값'이었다. 표준 median은
# 가운데 두 값(4096, 4178)의 평균 4137 ms다.
H1_BASELINE_MEDIAN_MS = 4137
H1_GATE_TARGET_MS = 1000
H1_GATE_MINIMUM_SAMPLES = 3
H1_GATE_METRIC = (
    "next.fresh_tick_ms - (previous.prime_tick_ms + "
    "previous.first_sample_lead_ms + previous.duration_ms)"
)


def load_h1_gate_history(
    paths: list[Path],
    *,
    expected_arm: str,
    expected_plan_sha256: dict[str, str],
    expected_calibration_sha256: dict[str, str],
    expected_execution_parameters: dict[str, float | None],
) -> tuple[list[int], list[dict]]:
    """동일 조건의 이전 COMPLETE artifact에서 로컬 표본만 가져온다.

    같은 파일을 반복 지정해 표본 수를 부풀리거나 서로 다른 계획·보정·속도의
    실행을 섞는 것을 fail-closed로 막는다. artifact 경계를 가로질러 leg를
    짝짓지 않는다.
    """
    values: list[int] = []
    sources: list[dict] = []
    seen_sha256: set[str] = set()
    for path in paths:
        digest = sha256_file(path)
        if digest in seen_sha256:
            raise ValueError(f"H1 gate evidence가 중복됐다: {path}")
        seen_sha256.add(digest)

        document = json.loads(path.read_text(encoding="utf-8"))
        recorded = document.get("legs")
        valid = (
            document.get("schema_version") == 1
            and document.get("status") == f"{STATUS}_COMPLETE"
            and document.get("operator_confirmation") == CONFIRMATION
            and document.get("arm") == expected_arm
            and document.get("plan_sha256") == expected_plan_sha256
            and document.get("calibration_sha256")
            == expected_calibration_sha256
            and document.get("execution_parameters")
            == expected_execution_parameters
            and document.get("h1_gate", {}).get("metric") == H1_GATE_METRIC
            and isinstance(recorded, list)
        )
        if not valid:
            raise ValueError(f"H1 gate evidence 조건이 현재 실행과 다르다: {path}")

        local_values = collect_transition_dead_times(
            recorded,
            previous_tags=("pregrasp", "pick_pregrasp"),
            current_tags=("grasp", "pick_grasp"),
        )
        if len(local_values) != 1:
            raise ValueError(
                f"H1 gate evidence에는 전환 표본이 정확히 1개여야 한다: {path}"
            )
        values.extend(local_values)
        sources.append({
            "path": str(path),
            "sha256": digest,
            "status": document["status"],
            "sample_ms": local_values[0],
        })
    return values, sources


class PhaseClock:
    """단계 사이 간격을 잰다. 이 도구가 만드는 유일한 새 계측이다."""

    def __init__(self) -> None:
        self._marks: list[tuple[str, float]] = []

    def mark(self, label: str) -> None:
        self._marks.append((label, time.monotonic()))

    def intervals_ms(self) -> list[tuple[str, str, float]]:
        return [
            (a_label, b_label, (b_time - a_time) * 1000.0)
            for (a_label, a_time), (b_label, b_time) in zip(
                self._marks[:-1], self._marks[1:], strict=True
            )
        ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pick-plan", type=Path, required=True)
    parser.add_argument("--pick-lift-plan", type=Path, required=True)
    parser.add_argument("--place-plan", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--minimum-tcp-z-m", type=float, default=None)
    parser.add_argument("--tracking-rate-raw-s", type=float, default=None)
    parser.add_argument("--q0-swing-tracking-rate-raw-s", type=float, default=None)
    parser.add_argument(
        "--h1-gate-evidence",
        type=Path,
        action="append",
        default=[],
        help="이전 resident 실행 JSON. 여러 번 주어 3표본 이상을 만든다.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config" / "single_arm_calibration.json",
    )
    parser.add_argument(
        "--bridge-calibration",
        type=Path,
        default=PACKAGE / "config" / "single_arm_calibration.json",
    )
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact resident pick/place-once confirmation is required")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    plan_sha256 = {
        "pick": sha256_file(arguments.pick_plan),
        "pick_lift": sha256_file(arguments.pick_lift_plan),
        "place": sha256_file(arguments.place_plan),
    }
    calibration_sha256 = {
        "planner": sha256_file(arguments.calibration),
        "bridge": sha256_file(arguments.bridge_calibration),
    }
    execution_parameters = {
        "minimum_tcp_z_m": arguments.minimum_tcp_z_m,
        "tracking_rate_raw_s": arguments.tracking_rate_raw_s,
        "q0_swing_tracking_rate_raw_s": arguments.q0_swing_tracking_rate_raw_s,
    }
    # 형식이나 실행 조건이 다른 history는 팔이 움직이기 전에 거부한다.
    history_gate_values, gate_sources = load_h1_gate_history(
        arguments.h1_gate_evidence,
        expected_arm=arguments.arm,
        expected_plan_sha256=plan_sha256,
        expected_calibration_sha256=calibration_sha256,
        expected_execution_parameters=execution_parameters,
    )
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")

    for label, plan in (
        ("PICK_PLAN", arguments.pick_plan),
        ("PICK_LIFT_PLAN", arguments.pick_lift_plan),
        ("PLACE_PLAN", arguments.place_plan),
    ):
        print(f"{label}={plan} sha256={sha256_file(plan)}")
    print(
        "CYCLE=q0 -> pick_pregrasp -> pick_grasp -> [converge+close] "
        "-> lift20 -> place_pregrasp -> place_grasp -> [converge+release] "
        "-> retreat -> q0  (RESIDENT: leg 이동만. 수렴/gripper 는 아직 subprocess)"
    )

    work = arguments.workdir
    leg_log = LegLog()
    clock = PhaseClock()
    stopped: str | None = None
    result: dict = {}

    session: ResidentArmSession | None = None
    try:
        # 세션 생성 자체(MoveIt 서비스·action 탐색)도 try 안에 둔다 — 실패해도
        # `run_pick_place_once.py`처럼 STOPPED 상태의 출력 artifact를 남긴다.
        session = ResidentArmSession()
        rate = arguments.tracking_rate_raw_s
        q0_swing_rate = arguments.q0_swing_tracking_rate_raw_s
        pick_pregrasp_rate = q0_swing_rate if q0_swing_rate is not None else rate

        clock.mark("start")
        move_to_resident(
            session, work, "pick_pregrasp", arguments.pick_plan, "pregrasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=pick_pregrasp_rate,
        )
        clock.mark("pregrasp")
        move_to_resident(
            session, work, "pick_grasp", arguments.pick_plan, "grasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=rate,
        )
        clock.mark("grasp")
        pick = converge(work, "pick", arguments.pick_plan, "grasp",
                        arguments.minimum_tcp_z_m)
        clock.mark("pick_converge")
        closed = gripper(GRIPPER_CLOSE_RAD, "pick_close",
                         GRIPPER_CLOSE_CONFIRMATION)
        clock.mark("pick_close")
        (work / "pick_close.txt").write_text(closed, encoding="utf-8")
        pick_gap = int(RESIDUAL_GAP_PATTERN.search(closed).group(1))
        picked = pick_gap >= CONTACT_THRESHOLD_RAW
        print(
            f"  pick  잔차 {pick['final_residual_mm']:.3f} mm  "
            f"간격 {pick_gap} raw  {'파지' if picked else '헛닫힘'}"
        )

        move_to_resident(
            session, work, "lift", arguments.pick_lift_plan, "grasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=rate,
        )
        clock.mark("lift")
        move_to_resident(
            session, work, "place_pregrasp", arguments.place_plan, "pregrasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=rate,
        )
        clock.mark("place_pregrasp")
        move_to_resident(
            session, work, "place_grasp", arguments.place_plan, "grasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=rate,
        )
        clock.mark("place_grasp")
        place = converge(work, "place", arguments.place_plan, "grasp",
                         arguments.minimum_tcp_z_m)
        clock.mark("place_converge")
        released = gripper(GRIPPER_OPEN_RAD, "place_release",
                           GRIPPER_OPEN_CONFIRMATION)
        clock.mark("place_release")
        (work / "place_release.txt").write_text(released, encoding="utf-8")
        release_gap = int(RESIDUAL_GAP_PATTERN.search(released).group(1))
        arm_motion = float(ARM_MOTION_PATTERN.search(released).group(1))
        print(
            f"  place 잔차 {place['final_residual_mm']:.3f} mm  "
            f"놓기 잔여 {release_gap} raw"
        )

        move_to_resident(
            session, work, "retreat", arguments.place_plan, "pregrasp",
            arguments.calibration, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=rate,
        )
        clock.mark("retreat")
        return_to_q0_resident(
            session, work, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=q0_swing_rate,
        )
        clock.mark("q0_return")

        result = {
            "pick_residual_mm": pick["final_residual_mm"],
            "pick_residual_vector_mm": pick["final_residual_vector_mm"],
            "pick_gap_raw": pick_gap,
            "picked": picked,
            "place_residual_mm": place["final_residual_mm"],
            "place_residual_vector_mm": place["final_residual_vector_mm"],
            "release_gap_raw": release_gap,
            "arm_motion_during_release_rad": arm_motion,
            "pick_clamped_joints": pick.get("clamped_joints"),
            "place_clamped_joints": place.get("clamped_joints"),
        }
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")
    finally:
        if session is not None:
            session.close()

    print()
    interval_records = []
    for a_label, b_label, interval_ms in clock.intervals_ms():
        transition = f"{a_label}->{b_label}"
        print(f"PHASE_INTERVAL_MS {transition}={interval_ms:.0f}")
        interval_records.append(
            {"transition": transition, "interval_ms": round(interval_ms, 1)}
        )

    current_gate_values = collect_transition_dead_times(
        leg_log.legs,
        previous_tags=("pregrasp", "pick_pregrasp"),
        current_tags=("grasp", "pick_grasp"),
    )
    gate_values = history_gate_values + current_gate_values
    gate_summary = summarise_transition_dead_times(gate_values)
    gate_count = 0 if gate_summary is None else gate_summary["count"]
    gate_ready = gate_count >= H1_GATE_MINIMUM_SAMPLES
    gate_median = None if gate_summary is None else gate_summary["median_ms"]
    gate_pass = bool(
        stopped is None
        and gate_ready
        and gate_median is not None
        and gate_median < H1_GATE_TARGET_MS
    )
    gate_verdict = (
        "INSUFFICIENT_SAMPLES"
        if not gate_ready
        else ("PASS" if gate_pass else "FAIL")
    )
    print()
    print(f"H1_BASELINE_MEDIAN_MS={H1_BASELINE_MEDIAN_MS}")
    print(f"H1_GATE_TARGET_MS={H1_GATE_TARGET_MS}")
    print(f"H1_GATE_MINIMUM_SAMPLES={H1_GATE_MINIMUM_SAMPLES}")
    print(f"H1_GATE_SAMPLE_COUNT={gate_count}")
    print(
        "H1_GATE_MEDIAN_MS="
        f"{gate_median if gate_median is not None else 'MISSING'}"
    )
    print(f"H1_GATE_OVERALL={gate_verdict}")

    startup = summarise_leg_telemetry(leg_log.legs) if leg_log.legs else None
    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "plan_sha256": plan_sha256,
        "calibration_sha256": calibration_sha256,
        "execution_parameters": execution_parameters,
        "result": result or None,
        "legs": leg_log.legs,
        "startup_trend": startup,
        "phase_intervals_ms": interval_records,
        "h1_gate": {
            "metric": H1_GATE_METRIC,
            "baseline_median_ms": H1_BASELINE_MEDIAN_MS,
            "target_ms": H1_GATE_TARGET_MS,
            "minimum_samples": H1_GATE_MINIMUM_SAMPLES,
            "summary": gate_summary,
            "verdict": gate_verdict,
            "overall_pass": gate_pass,
            "history_sources": gate_sources,
        },
        "convergence_and_gripper_still_subprocess": True,
        "stopped_reason": stopped,
        "automatic_retry_count": 0,
        "serial_port_opened": False,
        "operator_confirmation": CONFIRMATION,
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    if startup is not None:
        print()
        for line in format_leg_trend(startup):
            print(line)

    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
