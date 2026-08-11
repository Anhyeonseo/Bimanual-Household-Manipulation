#!/usr/bin/env python3
"""Move a held object from its current pose to a place pregrasp target once.

This is deliberately a single Motion-14 buffered leg.  It reads the current
joint state, requests fresh plan-only segments from MoveIt, then sends the
validated buffered leg.  It never invokes convergence, close, open, lift, or
release; the gripper stays exactly as it was when this program started.
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

from run_grasp_repeatability_pilot import LegLog, move_to, sha256_file  # noqa: E402


CONFIRMATION = "EXECUTE_HELD_OBJECT_TO_PLACE_PREGRASP_ONCE_2026_08_12"
STATUS = "HELD_OBJECT_TO_PLACE_PREGRASP_ONCE"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--place-plan", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--tracking-rate-raw-s", type=float, default=200.0)
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
        parser.error("exact held-object transfer confirmation is required")
    if arguments.tracking_rate_raw_s <= 0.0:
        parser.error("--tracking-rate-raw-s must be positive")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")
    plan_sha256 = sha256_file(arguments.place_plan)

    print(f"PLACE_PLAN={arguments.place_plan} sha256={plan_sha256}")
    print(f"TARGET=place_pregrasp rate_raw_s={arguments.tracking_rate_raw_s:g}")
    print("GRIPPER_COMMANDS=false convergence=false release=false")

    legs = LegLog()
    stopped: str | None = None
    try:
        move_to(
            arguments.workdir,
            "place_pregrasp",
            arguments.place_plan,
            "pregrasp",
            arguments.calibration,
            arm_names,
            calibration,
            legs,
            1,
            tracking_rate_raw_s=arguments.tracking_rate_raw_s,
        )
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")

    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "place_plan_sha256": plan_sha256,
        "target_name": "pregrasp",
        "tracking_rate_raw_s": arguments.tracking_rate_raw_s,
        "gripper_command_sent": False,
        "convergence_executed": False,
        "release_executed": False,
        "legs": legs.legs,
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
