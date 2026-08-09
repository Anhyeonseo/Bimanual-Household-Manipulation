#!/usr/bin/env python3
"""한 번의 완전한 Pick/Place 주기를 8단계로 실행한다. **물리 이동이 있다.**

    q0 -> pick_pregrasp -> pick_grasp -> [수렴+close]
       -> lift20 -> place_pregrasp -> place_grasp -> [수렴+release]
       -> retreat -> q0

`run_pick_place_pilot.py`(2026-08-07, A/B 왕복 반복 시험)와 같은 `move_to()`
기반 실행기를 쓴다 — 매 leg 를 **현재 실측 관절값에서** 새로 계획해서 단일
buffered leg 로 실행하므로 Motion-13 의 SHA-pinned 7-phase manifest 나
`buffered_trajectory_contract.json` 의 `continuous_pick_place_candidate`
(Pi 에 배포된 계약)를 건드리지 않는다.

`run_pick_place_pilot.py` 는 의도적으로 `place_pregrasp`/`retreat` 를
건너뛴다(2026-08-06 A5 가 다른 자세에서 그 상승으로 안전 게이트를 넘었기
때문). 이 도구는 그 두 단계를 되살린다 — 2026-08-07 밤 이후 tracking rate
300 raw/s 검증으로 여유가 생겼고, 오늘 목표가 정확히 이 8단계이기 때문이다.
**retreat 는 place_pregrasp 와 좌표가 같다** — 물체를 놓은 자세에서 다시
그 pregrasp 높이로 올라가는 것뿐이라 별도 IK 가 필요 없다.

**자동 재시도가 없다.** 한 단계라도 실패하면 멈추고 그때까지의 기록을 남긴다.
회차 반복이 아니라 단발 실행이므로 `run_pick_place_pilot.py` 의 A/B 왕복
로직은 없다 — pick 물체는 매번 그 자리에 있어야 한다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
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

CONFIRMATION = "RUN_PICK_PLACE_ONCE_2026_08_10"
STATUS = "PICK_PLACE_ONCE"


def converge(
    workdir: Path,
    tag: str,
    endpoint: Path,
    target_name: str,
    minimum_tcp_z_m: float | None,
) -> dict:
    import subprocess

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
    parser.add_argument(
        "--tracking-rate-raw-s",
        type=float,
        default=None,
        help=(
            "pick_grasp/lift/place_pregrasp/place_grasp/retreat 의 추종률 "
            "가정. None 이면 계획기의 보수적 기본값(50 raw/s)을 그대로 "
            "쓴다. q0<->pregrasp 큰 swing 두 leg(pick_pregrasp, q0 복귀)는 "
            "--q0-swing-tracking-rate-raw-s 로 별도 지정한다."
        ),
    )
    parser.add_argument(
        "--q0-swing-tracking-rate-raw-s",
        type=float,
        default=None,
        help=(
            "pick_pregrasp(q0->pregrasp)와 마지막 q0 복귀, 두 큰 swing "
            "leg 전용 추종률. None 이면 각자의 기본 정책을 따른다 "
            "(pick_pregrasp 는 --tracking-rate-raw-s 미지정 시와 동일하게 "
            "계획기 기본값, q0 복귀는 return_to_q0 자체 규율)."
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
        parser.error("exact pick/place-once confirmation is required")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
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
        "-> retreat -> q0"
    )

    work = arguments.workdir
    leg_log = LegLog()
    stopped: str | None = None
    result: dict = {}
    try:
        rate = arguments.tracking_rate_raw_s
        q0_swing_rate = arguments.q0_swing_tracking_rate_raw_s
        pick_pregrasp_rate = q0_swing_rate if q0_swing_rate is not None else rate
        move_to(work, "pick_pregrasp", arguments.pick_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=pick_pregrasp_rate)
        move_to(work, "pick_grasp", arguments.pick_plan, "grasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        pick = converge(work, "pick", arguments.pick_plan, "grasp",
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

        move_to(work, "lift", arguments.pick_lift_plan, "grasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        move_to(work, "place_pregrasp", arguments.place_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        move_to(work, "place_grasp", arguments.place_plan, "grasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        place = converge(work, "place", arguments.place_plan, "grasp",
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

        move_to(work, "retreat", arguments.place_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        return_to_q0(work, arm_names, calibration, leg_log, 1,
                     tracking_rate_raw_s=q0_swing_rate)

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

    startup = summarise_leg_telemetry(leg_log.legs) if leg_log.legs else None
    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "plan_sha256": {
            "pick": sha256_file(arguments.pick_plan),
            "pick_lift": sha256_file(arguments.pick_lift_plan),
            "place": sha256_file(arguments.place_plan),
        },
        "result": result or None,
        "legs": leg_log.legs,
        "startup_trend": startup,
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
