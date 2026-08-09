#!/usr/bin/env python3
"""검증된 Motion-12 경로로 현재 자세에서 q0 로 복귀한다. **물리 이동이 있다.**

`run_grasp_repeatability_pilot.return_to_q0()` 를 그대로 감싼 CLI. 중간에
멈췄거나 리셋이 필요할 때 단독으로 부를 수 있게 만든다.
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

from run_grasp_repeatability_pilot import LegLog, return_to_q0  # noqa: E402

CONFIRMATION = "RETURN_TO_Q0_ONCE_2026_08_10"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--tracking-rate-raw-s", type=float, default=None)
    parser.add_argument(
        "--bridge-calibration",
        type=Path,
        default=PACKAGE / "config" / "single_arm_calibration.json",
    )
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact return-to-q0 confirmation is required")
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")

    leg_log = LegLog()
    stopped: str | None = None
    try:
        return_to_q0(
            arguments.workdir, arm_names, calibration, leg_log, 1,
            tracking_rate_raw_s=arguments.tracking_rate_raw_s,
        )
    except Exception as error:
        stopped = str(error)
        print(f"STOPPED {stopped}")

    document = {
        "schema_version": 1,
        "status": "RETURN_TO_Q0_ONCE_" + ("COMPLETE" if stopped is None else "STOPPED"),
        "arm": arguments.arm,
        "legs": leg_log.legs,
        "stopped_reason": stopped,
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
