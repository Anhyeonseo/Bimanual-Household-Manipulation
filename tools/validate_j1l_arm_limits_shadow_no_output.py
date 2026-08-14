#!/usr/bin/env python3
"""Run J1-W shadow validation with approved J1-L arm limits enabled."""

from __future__ import annotations

from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path

import validate_protocol_v2_unwrap_shadow_no_output as j1w


CONFIRMATION = "J1L_ARM_LIMITS_BOTH_ARMS_TORQUE_OFF_NO_GOAL_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00024100
APPROVED_SHA256 = (
    "ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff"
)
ARM_JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
STATUS = "J1L_ARM_LIMITS_SHADOW_NO_OUTPUT_PASS"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_approved_limits(root: Path) -> tuple[Path, dict]:
    path = root / "config/bimanual_j1_operational_limits.approved.json"
    actual = file_sha256(path)
    if actual != APPROVED_SHA256:
        raise RuntimeError(
            f"approved J1-L SHA mismatch expected={APPROVED_SHA256} "
            f"actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        document.get("status") != "J1_L_ARM_5_APPROVED_FOR_PARITY_ONLY"
        or document.get("operator_approved") is not True
        or document.get("motion_authorized") is not False
        or document.get("runtime_change_authorized") is not False
    ):
        raise RuntimeError("approved J1-L manifest is not fail-closed")
    return path, document


def verify_anchor_inside_arm_limits(
    anchor_urad: tuple[int, ...], approved: dict
) -> None:
    if len(anchor_urad) != 12:
        raise RuntimeError("J1-L shadow anchor must contain 12 joints")
    index = 0
    for arm in ("left", "right"):
        for name in ARM_JOINTS:
            joint = approved["arms"][arm][name]
            value = anchor_urad[index]
            lower = int(joint["minimum_urad"])
            upper = int(joint["maximum_urad"])
            if not lower <= value <= upper:
                raise RuntimeError(
                    f"{arm}_{name} anchor {value} outside {lower}..{upper}"
                )
            index += 1
        index += 1  # Gripper mapping is deliberately not approved.


def main() -> int:
    args = j1w.parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("confirmation mismatch")
    root = Path(__file__).resolve().parents[1]
    approved_path, approved = load_approved_limits(root)

    j1w.CONFIRMATION = CONFIRMATION
    j1w.EXPECTED_FIRMWARE_VERSION = EXPECTED_FIRMWARE_VERSION
    hidden_output = io.StringIO()
    with redirect_stdout(hidden_output):
        result = j1w.main()
    if result != 0:
        return result

    output = args.output.resolve()
    document = json.loads(output.read_text(encoding="utf-8"))
    if document["hello"]["firmware_version"] != EXPECTED_FIRMWARE_VERSION:
        raise RuntimeError("J1-L firmware identity changed")
    anchor = tuple(document["independently_computed_anchor_urad"])
    verify_anchor_inside_arm_limits(anchor, approved)
    document["record_kind"] = "j1l_arm_limits_shadow_no_output"
    document["overall_verdict"] = STATUS
    document["approved_limits"] = {
        "path": str(approved_path),
        "sha256": APPROVED_SHA256,
        "arm_joint_count": 10,
        "grippers_excluded": True,
    }
    document["firmware_arm_limit_admission_verified"] = True
    output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = file_sha256(output)
    print(
        f"{STATUS} firmware=0x{EXPECTED_FIRMWARE_VERSION:08X} "
        f"arm_joints=10 grippers=blocked applied="
        f"{document['terminal']['applied_samples']} "
        f"motion_authorized=false output={output} sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
