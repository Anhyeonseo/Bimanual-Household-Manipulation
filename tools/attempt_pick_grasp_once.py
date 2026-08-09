#!/usr/bin/env python3
"""grasp offset 후보 하나를 시험한다. **물리 이동이 있다.**

    (현재 자세) -> pregrasp -> grasp -> [수렴+close] -> 판정 -> 다시 pregrasp

offset 재스윕 전용 — `pick-plan` 에 이미 원하는 `--grasp-offset` 이 baked
in 되어 있다고 가정한다(`ros_moveit_plan_grasp.py --grasp-offset <값>` 으로
새로 만든 파일을 넣는다). 잡았든 못 잡았든 **항상 pregrasp 로 복귀**한다 —
못 잡았으면 빈 채로, 잡았으면 문 채로. 다음 후보를 바로 이어서 시도할 수
있는 상태로 남긴다. 자동으로 다음 offset 을 시도하지 않는다 — 그건
운영자가 결정한다.
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

from run_grasp_repeatability_pilot import (  # noqa: E402
    CONTACT_THRESHOLD_RAW,
    GRIPPER_CLOSE_CONFIRMATION,
    GRIPPER_CLOSE_RAD,
    GRIPPER_OPEN_CONFIRMATION,
    GRIPPER_OPEN_RAD,
    CONVERGENCE_CONFIRMATION,
    RESIDUAL_GAP_PATTERN,
    LegLog,
    gripper,
    move_to,
    sha256_file,
)

CONFIRMATION = "ATTEMPT_PICK_GRASP_ONCE_2026_08_10"
STATUS = "ATTEMPT_PICK_GRASP_ONCE"


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
    parser.add_argument("--grasp-offset-m", type=float, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--minimum-tcp-z-m", type=float, default=None)
    parser.add_argument("--tracking-rate-raw-s", type=float, default=None)
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
        parser.error("exact grasp-attempt confirmation is required")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")
    rate = arguments.tracking_rate_raw_s

    print(f"PICK_PLAN={arguments.pick_plan} sha256={sha256_file(arguments.pick_plan)}")
    print(f"GRASP_OFFSET_M={arguments.grasp_offset_m:.6f}")

    work = arguments.workdir
    leg_log = LegLog()
    stopped: str | None = None
    result: dict = {}
    try:
        move_to(work, "pregrasp", arguments.pick_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        move_to(work, "grasp", arguments.pick_plan, "grasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        converged = converge(work, "attempt", arguments.pick_plan, "grasp",
                             arguments.minimum_tcp_z_m)
        closed = gripper(GRIPPER_CLOSE_RAD, "pick_close", GRIPPER_CLOSE_CONFIRMATION)
        (work / "close.txt").write_text(closed, encoding="utf-8")
        gap = int(RESIDUAL_GAP_PATTERN.search(closed).group(1))
        picked = gap >= CONTACT_THRESHOLD_RAW
        print(
            f"  잔차 {converged['final_residual_mm']:.3f} mm  간격 {gap} raw  "
            f"{'파지' if picked else '헛닫힘'}"
        )
        if not picked:
            released = gripper(GRIPPER_OPEN_RAD, "place_release", GRIPPER_OPEN_CONFIRMATION)
            (work / "release.txt").write_text(released, encoding="utf-8")
        move_to(work, "pregrasp_return", arguments.pick_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
        result = {
            "grasp_offset_m": arguments.grasp_offset_m,
            "residual_mm": converged["final_residual_mm"],
            "residual_vector_mm": converged["final_residual_vector_mm"],
            "gap_raw": gap,
            "picked": picked,
            "contact_threshold_raw": CONTACT_THRESHOLD_RAW,
            "clamped_joints": converged.get("clamped_joints"),
        }
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")

    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "pick_plan_sha256": sha256_file(arguments.pick_plan),
        "result": result or None,
        "legs": leg_log.legs,
        "stopped_reason": stopped,
        "automatic_retry_count": 0,
        "serial_port_opened": False,
        "operator_confirmation": CONFIRMATION,
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
