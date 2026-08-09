#!/usr/bin/env python3
"""같은 자세를 반복해 잔차가 재현되는지 잰다. **물리 이동이 회차마다 있다.**

**왜 이것을 재는가.**

2026-08-06 C2 가 남긴 유일한 큰 미지수다. 파지는 잔차 `10.17 mm` 와
`7.75 mm` 에서 성립했다 — 과제 허용치 `4 mm` 는 파지 성공의 필요조건이
아니었다. 수렴 계층의 값은 잔차를 `0` 으로 만드는 데 있지 않고 **작게 만들고
측정 가능하게** 만드는 데 있으며, 남은 잔차는 `PICK_GRASP_OFFSET_M` 이
흡수한다.

그런데 offset 이 흡수할 수 있으려면 잔차가 **회차 간 재현**되어야 한다.
A4 가 실패한 이유도 잔차가 컸기 때문이 아니라 회차마다 달랐기 때문이다.
`0.011` 이 우연인지 재현되는 값인지는 아직 아무도 모른다.

**그래서 이 도구는 성공률을 재지 않는다.** 회차별 잔차의 **흩어짐**을 잰다.
그것이 offset 이라는 방식 자체가 성립하는지를 결정한다.

**한 회차는 이렇게 돈다.**

    q0 -> pregrasp -> grasp -> 수렴 -> 닫기(관측) -> 열기 -> q0

**q0 을 거치는 것은 취향이 아니라 유일하게 검증된 상승 경로이기 때문이다.**
2026-08-06 A5 1회차는 파지 자세에서 pregrasp 로 **펼친 채 드는** 이동
(SHOULDER 259 raw 상승)에서 중단됐다. post-settle 이 `32 raw` 로 안전 허용치
`30` 을 넘었고, 14회 관측이 2685 ms 동안 전부 동일해 **평형에 있었다** —
더 기다려도 가지 않는다.

같은 날 `grasp -> q0` 은 SHOULDER 를 `1536 raw` 나 들어올리고도 post-settle
`6 raw` 로 통과했다. 접는 방향이라 부하가 줄기 때문이다. 즉 문제는 이동
거리가 아니라 **펼친 자세에서 드는 것**이다.

그리고 매 회차를 q0 에서 시작하면 앞 회차의 최종 자세가 다음 회차에
스며들지 않는다. 재현성을 재는 데는 그편이 옳다.

**자동 재시도가 없다.** 한 회차라도 실패하면 거기서 멈추고 그때까지의
기록을 남긴다. 조용히 이어가지 않는다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from buffered_leg_telemetry import (  # noqa: E402
    format_leg_trend,
    parse_leg_telemetry,
    summarise_leg_telemetry,
)

CONFIRMATION = "RUN_A5_GRASP_REPEATABILITY_PILOT"
STATUS = "GRASP_REPEATABILITY_PILOT"
SEGMENT_LEG_CONFIRMATION = "EXECUTE_MOTION14_FRESH_SEGMENT_LEG_ONCE"
CONVERGENCE_CONFIRMATION = "EXECUTE_C2_GRASP_CONVERGENCE_ONCE"
GRIPPER_CLOSE_CONFIRMATION = "EXECUTE_MOTION13_GRIPPER_PICK_CLOSE_ONCE"
GRIPPER_OPEN_CONFIRMATION = "EXECUTE_MOTION13_GRIPPER_PLACE_RELEASE_ONCE"
Q0_RETURN_CONFIRMATION = "EXECUTE_MOTION12_Q0_RETURN_ONCE"

# 계약의 접촉 판별값. 물체 없음 5 raw, 물체 있음 23 raw 로 실측됐다.
CONTACT_THRESHOLD_RAW = 14
GRIPPER_CLOSE_RAD = 0.130388
GRIPPER_OPEN_RAD = 0.059825

RESIDUAL_GAP_PATTERN = re.compile(r"RESIDUAL_GAP_RAW=(\d+)")
ARM_MOTION_PATTERN = re.compile(r"ARM_MOTION_RAD=(\S+)")


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


class StepFailure(RuntimeError):
    """실패한 단계의 출력을 예외에 실어 보낸다.

    종료 코드만 던지면 **거부 사유가 사라진다.** 우리가 쫓는 사건
    (`first_sample_lead_ms` 가 게이트 아래로 떨어져 거부되는 것)은 실패
    경로에서만 관측되므로, 그 출력이 곧 증거다.
    """

    def __init__(self, message: str, output: str) -> None:
        super().__init__(message)
        self.output = output


def run(command: list[str], label: str, quiet: bool = True) -> str:
    completed = subprocess.run(
        command, capture_output=True, text=True, cwd=str(ROOT)
    )
    output = completed.stdout + completed.stderr
    if not quiet or completed.returncode != 0:
        print(output.rstrip())
    if completed.returncode != 0:
        raise StepFailure(
            f"{label} failed with exit {completed.returncode}", output
        )
    return output


class LegLog:
    """세션에서 실행된 buffered leg 의 startup 진단을 순서대로 모은다.

    x 축이 **세션 누적 leg 번호**인 것이 핵심이다. 회차 번호가 아니다 —
    한 회차가 leg 을 여러 개 실행하고, 반복될수록 쌓인다는 가설은 회차가
    아니라 leg 실행 횟수에 걸려 있다.
    """

    def __init__(self) -> None:
        self._legs: list[dict] = []

    @property
    def legs(self) -> list[dict]:
        return list(self._legs)

    def record(self, *, tag: str, run_index: int, text: str, ok: bool) -> dict:
        record: dict = {
            "ordinal": len(self._legs) + 1,
            "run": run_index,
            "tag": tag,
            "ok": ok,
        }
        record.update(parse_leg_telemetry(text))
        self._legs.append(record)
        return record


def read_joint_state(names: tuple[str, ...], timeout_s: float = 20.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("a5_joint_state_reader")
    latest: dict[str, float] = {}
    try:
        node.create_subscription(
            JointState,
            "/joint_states",
            lambda m: latest.update(dict(zip(m.name, m.position))),
            10,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(name in latest for name in names):
                return tuple(latest[name] for name in names)
        raise TimeoutError("/joint_states did not carry every joint")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def move_to(
    workdir: Path,
    tag: str,
    endpoint: Path,
    target_name: str,
    calibration_path: Path,
    arm_names: tuple[str, ...],
    calibration,
    log: LegLog | None = None,
    run_index: int = 0,
    tracking_rate_raw_s: float | None = None,
) -> None:
    """Motion-14 파이프라인으로 한 구간 이동한다.

    `tracking_rate_raw_s` 를 주면 그 추종률 가정으로 leg 시간을 다시 고른다.
    `None` 이면 계획기의 보수적 기본값(`50 raw/s`)을 그대로 쓴다. q0 복귀는
    검증된 Motion-12 경로라 이 손잡이가 닿지 않는다 — 속도를 재는 구간과
    복귀 구간을 분리해 두는 편이 안전하다.
    """
    import math

    observed = read_joint_state((*arm_names, arm_names[0].replace(
        "base_joint", "gripper_joint")))
    start = observed[:5]
    anchor = tuple(
        round(
            joint.zero_raw
            + joint.direction * value * 4096.0 / (2.0 * math.pi)
        )
        for joint, value in zip(calibration.joints, observed, strict=True)
    )
    segments = workdir / f"{tag}_segments.json"
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"),
            "--plan-only",
            "--source-plan", str(endpoint),
            "--target-name", target_name,
            "--calibration", str(calibration_path),
            "--start=" + ",".join(f"{v:.12f}" for v in start),
            "--output", str(segments),
        ],
        f"{tag}: segments",
    )
    leg = workdir / f"{tag}_leg.json"
    plan_command = [
        sys.executable,
        str(ROOT / "tools" / "plan_buffered_segment_leg.py"),
        "--plan-only",
        "--segments", str(segments),
        "--segments-sha256", sha256_file(segments),
        "--anchor-raw", *[str(v) for v in anchor],
        "--output", str(leg),
    ]
    if tracking_rate_raw_s is not None:
        plan_command += ["--tracking-rate-raw-s", f"{tracking_rate_raw_s:g}"]
    planned = run(plan_command, f"{tag}: leg")
    _execute_leg(
        workdir,
        tag,
        [
            sys.executable,
            str(ROOT / "tools" / "execute_buffered_segment_leg_once.py"),
            str(leg),
            "--expected-sha256", sha256_file(leg),
            "--confirmation", SEGMENT_LEG_CONFIRMATION,
        ],
        planned=planned,
        log=log,
        run_index=run_index,
    )


def _execute_leg(
    workdir: Path,
    tag: str,
    command: list[str],
    *,
    planned: str,
    log: LegLog | None,
    run_index: int,
) -> None:
    """leg 을 실행하고 그 출력을 **버리지 않는다.**

    실행기는 `TERMINAL_DIAGNOSTICS=precompute_ms=… first_sample_lead_ms=…`
    를 이미 찍고 있었다. 여태 그 문자열을 받아서 버렸기 때문에 "6~7회부터
    멈춘다"가 수치로 남은 적이 없다. 계획 출력도 같이 붙인다 —
    `PLAN_SAMPLE_COUNT` 가 있어야 precompute 를 leg 길이로 정규화할 수 있다.
    """
    try:
        executed = run(command, f"{tag}: execute")
    except StepFailure as failure:
        text = planned + failure.output
        (workdir / f"{tag}_execute.txt").write_text(text, encoding="utf-8")
        if log is not None:
            log.record(tag=tag, run_index=run_index, text=text, ok=False)
        raise
    text = planned + executed
    (workdir / f"{tag}_execute.txt").write_text(text, encoding="utf-8")
    if log is not None:
        log.record(tag=tag, run_index=run_index, text=text, ok=True)


def return_to_q0(
    workdir: Path,
    arm_names: tuple[str, ...],
    calibration,
    log: LegLog | None = None,
    run_index: int = 0,
    tracking_rate_raw_s: float | None = None,
) -> None:
    """검증된 Motion-12 경로로 q0 로 접는다.

    `plan_buffered_q0_return.py` 는 추종 계약을 통과하는 최소 시간을 스스로
    탐색한다. 이 경로는 상승이지만 접는 방향이라 부하가 줄어 통과한다.

    `tracking_rate_raw_s` 를 주면 그 추종률 가정으로 이 leg 시간도 다시
    고른다 -- 기본은 검증된 보수적 값 그대로다.
    """
    import math

    observed = read_joint_state(
        (*arm_names, arm_names[0].replace("base_joint", "gripper_joint"))
    )
    anchor = tuple(
        round(
            joint.zero_raw
            + joint.direction * value * 4096.0 / (2.0 * math.pi)
        )
        for joint, value in zip(calibration.joints, observed, strict=True)
    )
    plan = workdir / "q0_return.json"
    plan_command = [
        sys.executable,
        str(ROOT / "tools" / "plan_buffered_q0_return.py"),
        "--plan-only",
        "--anchor-raw", *[str(v) for v in anchor],
        "--output", str(plan),
    ]
    if tracking_rate_raw_s is not None:
        plan_command += ["--tracking-rate-raw-s", f"{tracking_rate_raw_s:g}"]
    planned = run(plan_command, "q0 return: plan")
    _execute_leg(
        workdir,
        "q0_return",
        [
            sys.executable,
            str(ROOT / "tools" / "execute_buffered_q0_return_once.py"),
            str(plan),
            "--expected-sha256", sha256_file(plan),
            "--confirmation", Q0_RETURN_CONFIRMATION,
        ],
        planned=planned,
        log=log,
        run_index=run_index,
    )


def gripper(position_rad: float, label: str, confirmation: str) -> str:
    return run(
        [
            sys.executable,
            str(ROOT / "tools" / "execute_gripper_command_once.py"),
            "--label", label,
            "--position-rad", f"{position_rad:.6f}",
            "--confirmation", confirmation,
            "--expect", "report",
        ],
        f"gripper {label}",
    )


# **워밍업이 아니라 두 개의 평탄역이다.**
#
# 2026-08-06 에 6회씩 두 번 돌렸다.
#   A: 8.303, 11.620, 10.503, 10.503, 10.597, 10.503
#   B: 8.966,  8.852,  8.852, 11.073, 11.079, 11.050
#
# 둘 다 낮은 값(~8.7)에서 시작해 높은 값(~10.8)으로 가는데, **계단이 생기는
# 지점이 다르다** (A 는 2->3, B 는 3->4). 즉 고정된 워밍업 회차가 없다.
#
# 평탄역 안에서는 거의 결정론적이다 — 인접 회차 차이가
# `0.0, 0.0, 0.006, 0.029, 0.094, 0.114 mm` 다. 평탄역 사이는 `2.0 mm` 다.
#
# 그래서 "몇 회차까지 워밍업" 으로 자르지 않고 **인접 회차 차이**(단기
# 재현성)와 **세션 범위**(표류)를 따로 보고한다. 첫 구현은 회차 수로
# 잘랐는데 한 데이터에만 맞는 값이었다.


def summarise(runs: list[dict]) -> dict:
    """흩어짐이 결론이다. 성공률이 아니라 재현성을 본다.

    워밍업 구간과 정상상태를 갈라 보고한다. 둘을 섞으면 "재현되지 않는다"
    로 잘못 읽힌다 — 2026-08-06 에 실제로 그렇게 잘못 읽었다.
    """
    residuals = [r["convergence_residual_mm"] for r in runs]
    gaps = [r["residual_gap_raw"] for r in runs]
    held = [r for r in runs if r["held"]]
    summary = {
        "run_count": len(runs),
        "held_count": len(held),
        "residual_mm": {
            "values": residuals,
            "minimum": min(residuals),
            "maximum": max(residuals),
            "mean": statistics.fmean(residuals),
            "spread": max(residuals) - min(residuals),
        },
        "residual_gap_raw": {
            "values": gaps,
            "minimum": min(gaps),
            "maximum": max(gaps),
        },
        "contact_threshold_raw": CONTACT_THRESHOLD_RAW,
    }
    if len(residuals) >= 2:
        summary["residual_mm"]["stdev"] = statistics.stdev(residuals)

    if len(residuals) >= 2:
        adjacent = [
            abs(residuals[i + 1] - residuals[i])
            for i in range(len(residuals) - 1)
        ]
        summary["adjacent_difference_mm"] = {
            "values": adjacent,
            "maximum": max(adjacent),
            "median": statistics.median(adjacent),
        }
        # 단기 재현성과 세션 표류는 다른 질문이다. 평탄역 안에서는
        # 결정론적이어도 세션 동안 계단이 생길 수 있다.
        summary["session_drift_mm"] = max(residuals) - min(residuals)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-plan", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--minimum-tcp-z-m", type=float, default=None)
    parser.add_argument(
        "--pregrasp-tracking-rate-raw-s",
        type=float,
        default=None,
        help="q0->pregrasp leg 에만 적용된다.",
    )
    parser.add_argument(
        "--grasp-tracking-rate-raw-s",
        type=float,
        default=None,
        help="pregrasp->grasp leg 에만 적용된다.",
    )
    parser.add_argument(
        "--q0-return-tracking-rate-raw-s",
        type=float,
        default=None,
        help=(
            "grasp->q0 복귀 leg 에만 적용된다. 셋 다 각자 따로 지정해야 "
            "한다 -- leg 마다 관절 이동량이 달라 한 값을 전부에 적용하면 "
            "관절 속도 하드 게이트를 넘겨 계획이 거부될 수 있다."
        ),
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
        parser.error("exact pilot confirmation is required")
    if arguments.runs < 2:
        parser.error("a repeatability pilot needs at least two runs")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")

    print(f"ARM={arguments.arm}")
    print(f"SOURCE_PLAN={arguments.source_plan}")
    print(f"SOURCE_PLAN_SHA256={sha256_file(arguments.source_plan)}")
    print(f"RUNS={arguments.runs}")
    print(
        "PREGRASP_TRACKING_RATE_RAW_S="
        f"{arguments.pregrasp_tracking_rate_raw_s}"
    )
    print(
        "GRASP_TRACKING_RATE_RAW_S="
        f"{arguments.grasp_tracking_rate_raw_s}"
    )
    print(
        "Q0_RETURN_TRACKING_RATE_RAW_S="
        f"{arguments.q0_return_tracking_rate_raw_s}"
    )
    print(f"CONTACT_THRESHOLD_RAW={CONTACT_THRESHOLD_RAW}")
    print("QUESTION=does the residual repeat, not does the grasp succeed")
    print("CYCLE=q0 -> pregrasp -> grasp -> converge -> close -> open -> q0")

    runs: list[dict] = []
    leg_log = LegLog()
    stopped: str | None = None
    for index in range(1, arguments.runs + 1):
        work = arguments.workdir / f"run{index:02d}"
        work.mkdir(parents=True, exist_ok=True)
        print(f"\n===== run {index}/{arguments.runs} =====")
        try:
            move_to(work, "pregrasp", arguments.source_plan, "pregrasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index,
                    tracking_rate_raw_s=arguments.pregrasp_tracking_rate_raw_s)
            move_to(work, "grasp", arguments.source_plan, "grasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index,
                    tracking_rate_raw_s=arguments.grasp_tracking_rate_raw_s)

            command = [
                sys.executable,
                str(ROOT / "tools" / "execute_grasp_convergence_once.py"),
                "--source-plan", str(arguments.source_plan),
                "--target-name", "grasp",
                "--confirmation", CONVERGENCE_CONFIRMATION,
                "--workdir", str(work / "converge"),
                "--output", str(work / "converge.json"),
            ]
            if arguments.minimum_tcp_z_m is not None:
                command += ["--minimum-tcp-z-m",
                            str(arguments.minimum_tcp_z_m)]
            # 수렴이 허용치를 못 채워도 계속한다. 그 잔차가 측정 대상이다.
            subprocess.run(command, capture_output=True, text=True,
                           cwd=str(ROOT))
            convergence = json.loads(
                (work / "converge.json").read_text(encoding="utf-8")
            )["convergence"]

            closed = gripper(GRIPPER_CLOSE_RAD, "pick_close",
                             GRIPPER_CLOSE_CONFIRMATION)
            (work / "gripper_close.txt").write_text(closed, encoding="utf-8")
            gap = int(RESIDUAL_GAP_PATTERN.search(closed).group(1))
            arm_motion = float(ARM_MOTION_PATTERN.search(closed).group(1))
            gripper(GRIPPER_OPEN_RAD, "place_release",
                    GRIPPER_OPEN_CONFIRMATION)
            # 다음 회차를 같은 곳에서 시작한다. 앞 회차의 최종 자세가
            # 스며들면 재현성이 아니라 누적을 재게 된다.
            return_to_q0(
                work, arm_names, calibration, leg_log, index,
                tracking_rate_raw_s=arguments.q0_return_tracking_rate_raw_s,
            )
        except Exception as error:
            stopped = f"run {index}: {error}"
            print(f"STOPPED {stopped}")
            break

        record = {
            "run": index,
            "convergence_residual_mm": convergence["final_residual_mm"],
            "residual_mm_by_iteration": convergence[
                "residual_mm_by_iteration"
            ],
            "final_residual_vector_mm": convergence[
                "final_residual_vector_mm"
            ],
            "overshoot_used": convergence["overshoot_used"],
            "clamped_joints": convergence.get("clamped_joints"),
            "residual_gap_raw": gap,
            "held": gap >= CONTACT_THRESHOLD_RAW,
            "arm_motion_during_close_rad": arm_motion,
        }
        runs.append(record)
        print(
            f"  잔차 {record['convergence_residual_mm']:.3f} mm  "
            f"잔여 간격 {gap} raw  "
            f"{'파지' if record['held'] else '헛닫힘'}"
        )

    # 한 회차도 못 채웠어도 leg 진단은 남긴다. 첫 회차에서 멈췄다면 그
    # 멈춤 자체가 이번 시험이 찾던 것일 수 있다.
    summary = summarise(runs) if runs else None
    startup = summarise_leg_telemetry(leg_log.legs) if leg_log.legs else None
    document = {
        "schema_version": 2,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "source_plan": str(arguments.source_plan),
        "source_plan_sha256": sha256_file(arguments.source_plan),
        "requested_runs": arguments.runs,
        "summary": summary,
        "runs": runs,
        "startup_trend": startup,
        "legs": leg_log.legs,
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

    if summary is None:
        print(f"OUTPUT={arguments.output}")
        print("A5_PILOT_FAIL no run completed")
        return 2

    residual = summary["residual_mm"]
    print()
    print(f"RUNS_COMPLETED={summary['run_count']}/{arguments.runs}")
    print(f"HELD={summary['held_count']}/{summary['run_count']}")
    print(f"RESIDUAL_MM={[round(v, 3) for v in residual['values']]}")
    print(
        f"RESIDUAL_MM_RANGE={residual['minimum']:.3f}.."
        f"{residual['maximum']:.3f}  spread={residual['spread']:.3f}"
    )
    if "stdev" in residual:
        print(f"RESIDUAL_MM_STDEV={residual['stdev']:.3f}")
    adjacent = summary.get("adjacent_difference_mm")
    if adjacent:
        print(
            f"ADJACENT_DIFF_MM={[round(v, 3) for v in adjacent['values']]}"
        )
        print(
            f"ADJACENT_MAX_MM={adjacent['maximum']:.3f}  "
            f"median={adjacent['median']:.3f}   (단기 재현성)"
        )
        print(f"SESSION_DRIFT_MM={summary['session_drift_mm']:.3f}   (세션 표류)")
    print(f"RESIDUAL_GAP_RAW={summary['residual_gap_raw']['values']}")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
