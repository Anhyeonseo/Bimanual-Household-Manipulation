#!/usr/bin/env python3
"""Validate the motion-locked square-towel physical/contact candidate."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_task_runtime import (  # noqa: E402
    TowelTaskContractError,
    load_towel_contract,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=Path("config/towel_task_contract.candidate.yaml"),
    )
    args = parser.parse_args()
    try:
        contract = load_towel_contract(args.path)
        artifacts = contract.get("software_artifacts")
        if not isinstance(artifacts, dict) or not artifacts:
            raise TowelTaskContractError("software_artifacts must be an object")
        resolved_root = ROOT.resolve()
        for name, raw_path in artifacts.items():
            if not isinstance(raw_path, str) or not raw_path:
                raise TowelTaskContractError(
                    f"software artifact {name} must be a path"
                )
            relative_path = Path(raw_path)
            resolved_path = (resolved_root / relative_path).resolve()
            if (
                relative_path.is_absolute()
                or resolved_root not in resolved_path.parents
                or not resolved_path.is_file()
            ):
                raise TowelTaskContractError(
                    f"software artifact {name} does not resolve to a repository file"
                )
        verified_hold_artifacts = 0
        contact = contract["cloth_contact_candidate"]
        for side in ("left", "right"):
            for layer in ("one_layer", "four_layer"):
                sample = contact[side][layer]
                artifact_path = ROOT / sample["artifact"]
                if not artifact_path.is_file():
                    continue
                payload = artifact_path.read_bytes()
                if sha256(payload).hexdigest() != sample["sha256"]:
                    raise TowelTaskContractError(
                        f"cloth-contact artifact digest mismatch: {artifact_path}"
                    )
                try:
                    hold = json.loads(payload)
                except json.JSONDecodeError as exc:
                    raise TowelTaskContractError(
                        f"cloth-contact artifact is invalid JSON: {artifact_path}"
                    ) from exc
                if (
                    hold.get("overall_verdict")
                    != "RESIDENT_BIMANUAL_CURRENT_POSE_HOLD_TWICE_PASS"
                    or hold.get("commanded_motion_delta_rad")
                    != [0.0] * 12
                    or hold.get("coordinated_stop_verified") is not True
                    or hold.get("final_status", {}).get("state") != "stopped"
                    or hold.get("final_status", {}).get("torque_hold_active")
                    is not False
                ):
                    raise TowelTaskContractError(
                        f"cloth-contact artifact does not prove a stopped hold: "
                        f"{artifact_path}"
                    )
                gripper_index = 5 if side == "left" else 11
                anchor_positions = hold.get("anchor_positions_rad")
                if (
                    not isinstance(anchor_positions, list)
                    or len(anchor_positions) != 12
                    or anchor_positions[gripper_index]
                    != sample["validated_hold_anchor_rad"]
                ):
                    raise TowelTaskContractError(
                        f"cloth-contact anchor mismatch: {artifact_path}"
                    )
                verified_hold_artifacts += 1
    except TowelTaskContractError as exc:
        print(f"[FAIL] {exc}")
        return 1
    null_hardware = sum(
        value is None
        for name, value in contract["hardware_limits"].items()
        if name != "provenance"
    )
    print(
        "[PASS] Towel contract records measured geometry/material and four "
        "static-retention checks while remaining motion-locked; "
        f"{verified_hold_artifacts}/4 local hold artifacts verified; "
        f"{null_hardware} automatic/dynamic hardware limits remain uncommissioned."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
