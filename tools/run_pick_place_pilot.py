#!/usr/bin/env python3
"""수렴을 적용한 Pick/Place 주기를 반복한다. **회차마다 물리 이동이 있다.**

**무엇이 달라졌나.**

Motion-13 은 3-leg 연속 경로를 완주했지만 **물체를 집지 못했다**
(`pick_close` 잔여 `3 raw`, 대조군 5). 그 뒤 C1/C2 가 도달 층을 닫았고
파지 offset 을 수렴 적용 상태에서 다시 쟀다 (`0.017 → 0.011`). 이 도구는
그 조건으로 전 주기를 다시 돈다.

**물체는 왕복한다.** 한 주기가 끝나면 물체는 놓은 자리에 있다. 다음 주기가
같은 자리를 집으러 가면 빈 자리를 집는다. 그래서 회차마다 출발과 도착을
바꾼다 — `A→B`, `B→A`, `A→B`... 사람이 물체를 되돌릴 필요도, 인식에 의존할
필요도 없다.

기하도 성립한다. B 에 놓을 때 펜은 그 자세의 손가락 축과 **수직**으로 눕고,
다시 B 에서 집을 때 같은 자세를 쓰면 손가락이 또 수직으로 가로지른다.
A4.5 의 "펜과 나란히 닫힘" 문제가 생기지 않는다.

덤으로 **두 자세의 잔차를 번갈아 재게 되어** 자세별 처짐 자료가 같이 나온다.

**주기 설계에 오늘 실측이 들어가 있다.**

    q0 → src_pregrasp → src_grasp → [수렴 + 닫기]
       → src_lift20 → dst_grasp → [수렴 + 놓기] → q0

`pick_grasp` 에서 `place_pregrasp` 로 올리지 않는다. 2026-08-06 A5 1회차가
파지 자세에서 pregrasp 로 **펼친 채 드는** 이동(SHOULDER 259 raw 상승)에서
post-settle `32 raw` 로 중단됐다 — 안전 허용치 `30` 초과이고 14회 관측이
`2685 ms` 동안 동일한 평형이었다.

대신 `20 mm` 만 들어 낮은 높이에서 옆으로 간다. 마지막 `place → q0` 은
접는 방향이라 같은 날 `1536 raw` 를 들고도 post-settle `6 raw` 로 통과했다.

**Place offset 예측.** `0.025` 는 그 회차의 처짐을 흡수하던 값이었다. 펜은
같은 테이블에 놓이므로 그리퍼가 있어야 할 높이는 Pick 과 같고, 수렴이 처짐을
없애면 Place offset 도 `0.011` 로 수렴해야 한다. 이 도구는 그 예측을
시험한다 — 틀리면 Place 스윕이 따로 필요하다.

**자동 재시도가 없다.** 한 회차라도 실패하면 멈추고 그때까지의 기록을 남긴다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import statistics
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from buffered_leg_telemetry import (  # noqa: E402
    format_leg_trend,
    summarise_leg_telemetry,
)
from run_grasp_repeatability_pilot import (  # noqa: E402
    CONTACT_THRESHOLD_RAW,
    GRIPPER_CLOSE_CONFIRMATION,
    GRIPPER_CLOSE_RAD,
    GRIPPER_OPEN_CONFIRMATION,
    GRIPPER_OPEN_RAD,
    CONVERGENCE_CONFIRMATION,
    ARM_MOTION_PATTERN,
    RESIDUAL_GAP_PATTERN,
    LegLog,
    gripper,
    move_to,
    return_to_q0,
    sha256_file,
)

CONFIRMATION = "RUN_A5_PICK_PLACE_PILOT"
STATUS = "PICK_PLACE_PILOT"


def converge(
    workdir: Path,
    tag: str,
    endpoint: Path,
    target_name: str,
    minimum_tcp_z_m: float | None,
) -> dict:
    """수렴시키고 그 기록을 돌려준다.

    허용치를 채우지 못해도 계속한다 — 그 잔차가 측정 대상이다.
    """
    output = workdir / f"{tag}_converge.json"
    command = [
        sys.executable,
        str(ROOT / "tools" / "execute_grasp_convergence_once.py"),
        "--source-plan", str(endpoint),
        "--target-name", target_name,
        "--confirmation", CONVERGENCE_CONFIRMATION,
        "--workdir", str(workdir / f"{tag}_converge"),
        "--output", str(output),
    ]
    if minimum_tcp_z_m is not None:
        command += ["--minimum-tcp-z-m", str(minimum_tcp_z_m)]
    subprocess.run(command, capture_output=True, text=True, cwd=str(ROOT))
    if not output.exists():
        raise RuntimeError(f"{tag}: 수렴 기록이 생성되지 않았다")
    return json.loads(output.read_text(encoding="utf-8"))["convergence"]


def summarise(runs: list[dict]) -> dict:
    def stats(values: list[float]) -> dict:
        result = {
            "values": values,
            "minimum": min(values),
            "maximum": max(values),
            "mean": statistics.fmean(values),
            "spread": max(values) - min(values),
        }
        if len(values) >= 2:
            result["stdev"] = statistics.stdev(values)
        return result

    picked = [r for r in runs if r["picked"]]
    return {
        "run_count": len(runs),
        "picked_count": len(picked),
        "pick_residual_mm": stats([r["pick_residual_mm"] for r in runs]),
        "place_residual_mm": stats([r["place_residual_mm"] for r in runs]),
        "pick_gap_raw": {
            "values": [r["pick_gap_raw"] for r in runs],
            "minimum": min(r["pick_gap_raw"] for r in runs),
            "maximum": max(r["pick_gap_raw"] for r in runs),
        },
        "release_gap_raw": {
            "values": [r["release_gap_raw"] for r in runs],
            "maximum": max(r["release_gap_raw"] for r in runs),
        },
        "contact_threshold_raw": CONTACT_THRESHOLD_RAW,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-a-plan", type=Path, required=True)
    parser.add_argument("--pose-a-lift-plan", type=Path, required=True)
    parser.add_argument("--pose-b-plan", type=Path, required=True)
    parser.add_argument("--pose-b-lift-plan", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--minimum-tcp-z-m", type=float, default=None)
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
        parser.error("exact pick/place pilot confirmation is required")
    if arguments.runs < 1:
        parser.error("runs must be at least 1")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")

    poses = {
        "A": (arguments.pose_a_plan, arguments.pose_a_lift_plan),
        "B": (arguments.pose_b_plan, arguments.pose_b_lift_plan),
    }
    print(f"ARM={arguments.arm}")
    for name, (plan, lift) in poses.items():
        print(f"POSE_{name}_PLAN={plan} sha256={sha256_file(plan)}")
        print(f"POSE_{name}_LIFT={lift} sha256={sha256_file(lift)}")
    print(f"RUNS={arguments.runs}")
    print(
        "CYCLE=q0 -> src_pregrasp -> src_grasp -> [converge+close] "
        "-> src_lift20 -> dst_grasp -> [converge+release] -> q0"
    )
    print("ALTERNATES=A->B, B->A, A->B, ...  (물체가 왕복하므로)")
    print(
        "NOTE=파지 자세에서 pregrasp 로 올리지 않는다 — 그 상승이 "
        "2026-08-06 에 안전 게이트를 넘겼다"
    )

    runs: list[dict] = []
    leg_log = LegLog()
    stopped: str | None = None
    for index in range(1, arguments.runs + 1):
        work = arguments.workdir / f"run{index:02d}"
        work.mkdir(parents=True, exist_ok=True)
        source_name = "A" if index % 2 == 1 else "B"
        target_name = "B" if source_name == "A" else "A"
        source_plan, source_lift = poses[source_name]
        target_plan, _ = poses[target_name]
        print(
            f"\n===== run {index}/{arguments.runs}  "
            f"{source_name} -> {target_name} ====="
        )
        try:
            move_to(work, "pick_pregrasp", source_plan, "pregrasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index)
            move_to(work, "pick_grasp", source_plan, "grasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index)
            pick = converge(work, "pick", source_plan, "grasp",
                            arguments.minimum_tcp_z_m)
            closed = gripper(GRIPPER_CLOSE_RAD, "pick_close",
                             GRIPPER_CLOSE_CONFIRMATION)
            (work / "pick_close.txt").write_text(closed, encoding="utf-8")
            pick_gap = int(RESIDUAL_GAP_PATTERN.search(closed).group(1))
            picked = pick_gap >= CONTACT_THRESHOLD_RAW
            print(
                f"  pick  잔차 {pick['final_residual_mm']:.3f} mm  "
                f"간격 {pick_gap} raw  {'파지' if picked else '헛닫힘'}"
            )

            move_to(work, "lift", source_lift, "grasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index)
            move_to(work, "place_grasp", target_plan, "grasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index)
            place = converge(work, "place", target_plan, "grasp",
                             arguments.minimum_tcp_z_m)
            released = gripper(GRIPPER_OPEN_RAD, "place_release",
                               GRIPPER_OPEN_CONFIRMATION)
            (work / "place_release.txt").write_text(released, encoding="utf-8")
            release_gap = int(RESIDUAL_GAP_PATTERN.search(released).group(1))
            arm_motion = float(ARM_MOTION_PATTERN.search(released).group(1))
            print(
                f"  place 잔차 {place['final_residual_mm']:.3f} mm  "
                f"놓기 잔여 {release_gap} raw"
            )

            return_to_q0(work, arm_names, calibration, leg_log, index)
        except Exception as error:
            stopped = f"run {index}: {error}"
            print(f"STOPPED {stopped}")
            break

        runs.append(
            {
                "run": index,
                "source_pose": source_name,
                "target_pose": target_name,
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
        )

    # 한 주기도 못 채웠어도 leg 진단은 남긴다. 첫 주기에서 멈췄다면 그
    # 멈춤 자체가 이번 시험이 찾던 것일 수 있다.
    summary = summarise(runs) if runs else None
    startup = summarise_leg_telemetry(leg_log.legs) if leg_log.legs else None
    document = {
        "schema_version": 2,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "pose_plan_sha256": {
            name: {
                "grasp": sha256_file(plan),
                "lift": sha256_file(lift),
            }
            for name, (plan, lift) in poses.items()
        },
        "alternates": True,
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
        print("PICK_PLACE_PILOT_FAIL no cycle completed")
        return 2

    print()
    print(f"RUNS_COMPLETED={summary['run_count']}/{arguments.runs}")
    print(f"PICKED={summary['picked_count']}/{summary['run_count']}")
    for key in ("pick_residual_mm", "place_residual_mm"):
        stats = summary[key]
        print(
            f"{key.upper()}={[round(v, 3) for v in stats['values']]}  "
            f"spread={stats['spread']:.3f}"
        )
    print(f"PICK_GAP_RAW={summary['pick_gap_raw']['values']}")
    print(f"RELEASE_GAP_RAW={summary['release_gap_raw']['values']}")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
