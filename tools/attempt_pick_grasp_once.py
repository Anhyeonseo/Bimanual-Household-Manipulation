#!/usr/bin/env python3
"""grasp offset 후보 하나를 시험하거나, 열린 그리퍼 위치를 확인한다.

    (현재 자세) -> pregrasp -> grasp -> [수렴+close] -> 판정 -> 다시 pregrasp

offset 재스윕 전용 — `pick-plan` 에 이미 원하는 `--grasp-offset` 이 baked
in 되어 있다고 가정한다(`ros_moveit_plan_grasp.py --grasp-offset <값>` 으로
새로 만든 파일을 넣는다). 잡았든 못 잡았든 **항상 pregrasp 로 복귀**한다 —
못 잡았으면 빈 채로, 잡았으면 문 채로. 다음 후보를 바로 이어서 시도할 수
있는 상태로 남긴다. 자동으로 다음 offset 을 시도하지 않는다 — 그건
운영자가 결정한다.

``--open-position-check`` 는 더 작은 범위의 안전 확인 모드다:

    (현재 자세) -> pregrasp -> grasp -> pregrasp

수렴과 모든 gripper 명령을 생략한다. 새 Z 후보가 물체보다 얼마나 높거나
낮은지 육안으로 확인하는 용도이며, 별도의 확인 문자열이 없으면 실행되지
않는다.

``--held-object-position-check`` 는 같은 이동을 물체를 문 상태에서 한다.
그리퍼 상태를 바꾸지 않고, place 높이에서 물체 바닥의 여유를 확인한 뒤
pregrasp로 되돌아간다.
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
OPEN_POSITION_CHECK_CONFIRMATION = "OPEN_PICK_POSITION_CHECK_ONCE_2026_08_12"
OPEN_POSITION_CHECK_STATUS = "OPEN_PICK_POSITION_CHECK"
HELD_OBJECT_POSITION_CHECK_CONFIRMATION = (
    "HELD_OBJECT_POSITION_CHECK_ONCE_2026_08_12"
)
HELD_OBJECT_POSITION_CHECK_STATUS = "HELD_OBJECT_POSITION_CHECK"


def confirmation_for(arguments: argparse.Namespace) -> str:
    if arguments.held_object_position_check:
        return HELD_OBJECT_POSITION_CHECK_CONFIRMATION
    if arguments.open_position_check:
        return OPEN_POSITION_CHECK_CONFIRMATION
    return CONFIRMATION


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
    parser.add_argument(
        "--open-position-check",
        action="store_true",
        help="move to grasp and back while skipping convergence and every gripper command",
    )
    parser.add_argument(
        "--held-object-position-check",
        action="store_true",
        help="same no-gripper position check while an object is already held",
    )
    parser.add_argument(
        "--hold-at-grasp-s",
        type=float,
        default=5.0,
        help="open-position-check only: visible hold at grasp, limited to 0..10 seconds",
    )
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
    if arguments.open_position_check and arguments.held_object_position_check:
        parser.error("choose at most one position-check mode")
    expected_confirmation = confirmation_for(arguments)
    if arguments.confirmation != expected_confirmation:
        parser.error("exact confirmation for the selected pick-check mode is required")
    if not 0.0 <= arguments.hold_at_grasp_s <= 10.0:
        parser.error("--hold-at-grasp-s must be within 0..10 seconds")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")
    rate = arguments.tracking_rate_raw_s
    position_check = (
        arguments.open_position_check or arguments.held_object_position_check
    )

    print(f"PICK_PLAN={arguments.pick_plan} sha256={sha256_file(arguments.pick_plan)}")
    print(f"GRASP_OFFSET_M={arguments.grasp_offset_m:.6f}")
    if position_check:
        print(
            "POSITION_CHECK=true "
            f"held_object={str(arguments.held_object_position_check).lower()} "
            "convergence=false gripper_commands=false"
        )

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
        if position_check:
            print(f"OPEN_POSITION_CHECK_HOLD_S={arguments.hold_at_grasp_s:.1f}")
            time.sleep(arguments.hold_at_grasp_s)
            result = {
                "grasp_offset_m": arguments.grasp_offset_m,
                "position_check_only": True,
                "held_object_position_check": arguments.held_object_position_check,
                "convergence_executed": False,
                "gripper_command_sent": False,
                "hold_at_grasp_s": arguments.hold_at_grasp_s,
            }
        else:
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
            result = {
                "grasp_offset_m": arguments.grasp_offset_m,
                "position_check_only": False,
                "convergence_executed": True,
                "gripper_command_sent": True,
                "residual_mm": converged["final_residual_mm"],
                "residual_vector_mm": converged["final_residual_vector_mm"],
                "gap_raw": gap,
                "picked": picked,
                "contact_threshold_raw": CONTACT_THRESHOLD_RAW,
                "clamped_joints": converged.get("clamped_joints"),
            }
        move_to(work, "pregrasp_return", arguments.pick_plan, "pregrasp",
                arguments.calibration, arm_names, calibration, leg_log, 1,
                tracking_rate_raw_s=rate)
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")

    document = {
        "schema_version": 1,
        "status": (
            (
                HELD_OBJECT_POSITION_CHECK_STATUS
                if arguments.held_object_position_check
                else OPEN_POSITION_CHECK_STATUS
            )
            if position_check
            else STATUS
        ) + f"_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "pick_plan_sha256": sha256_file(arguments.pick_plan),
        "result": result or None,
        "legs": leg_log.legs,
        "stopped_reason": stopped,
        "automatic_retry_count": 0,
        "serial_port_opened": False,
        "operator_confirmation": confirmation_for(arguments),
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
