#!/usr/bin/env python3
"""Place one already-held object: descend, release once, then clear upward.

The caller must have visually confirmed the selected place ``grasp`` height.
This tool requests fresh Motion-14 segments from the current joint state,
executes exactly one descent, opens the gripper exactly once with the existing
arm-motion gate, and only then returns to the plan's place pregrasp target.
It never closes the gripper, runs convergence, lifts, or retries a command.
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
    GRIPPER_OPEN_CONFIRMATION,
    GRIPPER_OPEN_RAD,
    LegLog,
    StepFailure,
    move_to,
    run,
    sha256_file,
)


CONFIRMATION = "EXECUTE_HELD_OBJECT_PLACE_ONCE_2026_08_12"
STATUS = "HELD_OBJECT_PLACE_ONCE"


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
        parser.error("exact held-object place confirmation is required")
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
    print(f"RATE_RAW_S={arguments.tracking_rate_raw_s:g}")
    print("CYCLE=place_grasp -> release_once -> place_pregrasp")
    print("GRIPPER_CLOSE=false convergence=false automatic_retry_count=0")

    legs = LegLog()
    stopped: str | None = None
    release_output = ""
    release_attempted = False
    release_completed = False
    try:
        move_to(
            arguments.workdir,
            "place_grasp",
            arguments.place_plan,
            "grasp",
            arguments.calibration,
            arm_names,
            calibration,
            legs,
            1,
            tracking_rate_raw_s=arguments.tracking_rate_raw_s,
        )
        release_attempted = True
        release_output = run(
            [
                sys.executable,
                str(ROOT / "tools" / "execute_gripper_command_once.py"),
                "--label", "place_release",
                "--position-rad", f"{GRIPPER_OPEN_RAD:.6f}",
                "--confirmation", GRIPPER_OPEN_CONFIRMATION,
                "--expect", "reached",
            ],
            "place release",
            quiet=False,
        )
        release_completed = True
        (arguments.workdir / "place_release.txt").write_text(
            release_output, encoding="utf-8"
        )
        move_to(
            arguments.workdir,
            "place_pregrasp_return",
            arguments.place_plan,
            "pregrasp",
            arguments.calibration,
            arm_names,
            calibration,
            legs,
            1,
            tracking_rate_raw_s=arguments.tracking_rate_raw_s,
        )
    except StepFailure as error:
        stopped = str(error)
        if release_attempted and error.output:
            release_output = error.output
            (arguments.workdir / "place_release.txt").write_text(
                release_output, encoding="utf-8"
            )
        print(f"STOPPED {stopped}")
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")

    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "place_plan_sha256": plan_sha256,
        "tracking_rate_raw_s": arguments.tracking_rate_raw_s,
        "gripper_close_sent": False,
        "gripper_release_attempted": release_attempted,
        "gripper_release_completed": release_completed,
        "gripper_release_expectation": "reached",
        "convergence_executed": False,
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
