#!/usr/bin/env python3
"""Create a fail-closed, plan-only J2 right-Base acceptance report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


STATUS = "J2_RIGHT_BASE_BIDIRECTIONAL_75_PERCENT_ACTIVE_PASS"
EXPECTED_MANIFEST_SHA256 = (
    "dfbfaf6c7138fab30afebc1f3e69c7d53edb01060bd349f65c6f048f150dff34"
)
EXPECTED_FIRMWARE = "0x00024200"
EXPECTED_NO_MOTION_SHA256 = (
    "e64422beccb552f8bf25de14e94e01479c5439c440f7238789dd9cd9b6d5da97"
)
ACTIVE_EVIDENCE = {
    ("upper", 75): (
        "j2_right_base_upper75_run01.json",
        "3cf8e4c4ae4a5c5f1f9926a436ed2245885375220e8035bea89e1e0169de4151",
        2711,
    ),
    ("lower", 25): (
        "j2_right_base_lower25_run01.json",
        "5f231aa887e96663d60e364e16bdd8415ebe2489e5583dec2ead44167b6483c8",
        1829,
    ),
    ("lower", 50): (
        "j2_right_base_lower50_run01.json",
        "af294547e29bcdf65da72336e936caeb97de2bff844d2e08588b5bea9712b5dc",
        1610,
    ),
    ("lower", 75): (
        "j2_right_base_lower75_run01.json",
        "bd7d818bba77a7869c94fffc079ddf1212545f9268065f552c179d46ffe91bd7",
        1391,
    ),
}
SUPPLEMENTAL_BASELINE = {
    ("upper", 25): (
        "j2_right_base_upper25_run03.json",
        "f22fa0c1c0d35d0d24e71a5646340297ec931c665ec40775c4f5ba3509d0d5a0",
    ),
    ("upper", 50): (
        "j2_right_base_upper50_run01.json",
        "e3411ff63ca625ebad1ba8ed7ce8759855dba934f1d9e512d91f2150f5d1209f",
    ),
}


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bound(path: Path, expected: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected:
        raise RuntimeError(
            f"SHA mismatch path={path} expected={expected} actual={actual}"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def validate_health(document: dict[str, Any]) -> None:
    for leg in document["legs"]:
        for cycle in leg["cycles"]:
            for snapshot in cycle["snapshots"]:
                if not all(snapshot["checks"].values()):
                    raise RuntimeError("recorded servo health check is not all-pass")
                if (
                    int(snapshot["servo_id"]) != 1
                    and int(snapshot.get("drift_from_preflight_raw", 0)) > 10
                ):
                    raise RuntimeError("nonselected joint drift exceeded 10 raw")


def validate_active(
    document: dict[str, Any],
    direction: str,
    fraction: int,
    target: int,
) -> dict[str, Any]:
    selected = document["selected"]
    if document.get("overall_verdict") != "J2_RIGHT_ARM_AXIS_ROUNDTRIP_PASS":
        raise RuntimeError("active evidence did not pass")
    if (
        selected.get("arm") != "right"
        or selected.get("joint") != "base"
        or selected.get("direction") != direction
        or int(selected.get("fraction_percent", -1)) != fraction
        or int(selected.get("target_unwrapped_raw", -1)) != target
    ):
        raise RuntimeError("active evidence selected target changed")
    identity = document.get("j2b_identity") or {}
    if (
        identity.get("firmware_version") != EXPECTED_FIRMWARE
        or identity.get("manifest_sha256") != EXPECTED_MANIFEST_SHA256
    ):
        raise RuntimeError("active evidence is not exact J2-B identity")
    command_limits = document.get("inputs", {}).get("command_limits", {})
    if command_limits.get("sha256") != EXPECTED_MANIFEST_SHA256:
        raise RuntimeError("active evidence command-limit SHA changed")
    if document.get("safe_stop") is not None:
        raise RuntimeError("active evidence required a safe stop")
    if not document.get("verified_disable", {}).get("success"):
        raise RuntimeError("active evidence did not end verified torque-off")
    maximum_tracking_error = int(document["maximum_tracking_error_raw"])
    if maximum_tracking_error > 10:
        raise RuntimeError("active evidence tracking error exceeded 10 raw")
    legs = document["legs"]
    if len(legs) != 2:
        raise RuntimeError("active evidence must contain outbound and return legs")
    if (
        int(legs[0]["goal_raw"]) != target
        or int(legs[0]["final_residual_raw"]) > 10
        or int(legs[1]["goal_raw"]) != 2048
        or int(legs[1]["final_residual_raw"]) > 10
    ):
        raise RuntimeError("active evidence terminal residual changed")
    validate_health(document)
    positions = [
        int(cycle["selected_position_raw"])
        for leg in legs
        for cycle in leg["cycles"]
    ]
    return {
        "direction": direction,
        "fraction_percent": fraction,
        "target_raw": target,
        "observed_minimum_raw": min(positions),
        "observed_maximum_raw": max(positions),
        "target_residual_raw": int(legs[0]["final_residual_raw"]),
        "q0_return_residual_raw": int(legs[1]["final_residual_raw"]),
        "maximum_tracking_error_raw": maximum_tracking_error,
        "verified_torque_off": True,
    }


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--evidence-directory",
        type=Path,
        default=root / "artifacts/joint_ranges/2026-08-14",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")

    no_motion_path = args.evidence_directory / "j2b_no_motion_run01.json"
    no_motion = load_bound(no_motion_path, EXPECTED_NO_MOTION_SHA256)
    if (
        no_motion.get("overall_verdict") != "PASS"
        or no_motion.get("motion_commands_sent") != 0
        or no_motion.get("torque_enable_requests_sent") != 0
        or no_motion.get("identity", {}).get("firmware_version")
        != EXPECTED_FIRMWARE
        or no_motion.get("identity", {}).get("manifest_sha256")
        != EXPECTED_MANIFEST_SHA256
        or not no_motion.get("verified_disable", {}).get("success")
    ):
        raise RuntimeError("J2-B no-motion evidence contract changed")

    active_results = []
    active_inputs = []
    for (direction, fraction), (name, digest, target) in ACTIVE_EVIDENCE.items():
        path = args.evidence_directory / name
        document = load_bound(path, digest)
        active_results.append(
            validate_active(document, direction, fraction, target)
        )
        active_inputs.append({"path": str(path), "sha256": digest})

    upper75 = next(
        item
        for item in active_results
        if item["direction"] == "upper" and item["fraction_percent"] == 75
    )
    lower75 = next(
        item
        for item in active_results
        if item["direction"] == "lower" and item["fraction_percent"] == 75
    )
    if upper75["observed_maximum_raw"] <= 2610:
        raise RuntimeError("J2-B upper traversal did not cross the legacy limit")
    if lower75["observed_minimum_raw"] >= 1988:
        raise RuntimeError("J2-B lower traversal did not cross the legacy limit")

    supplemental_inputs = []
    for (direction, fraction), (name, digest) in SUPPLEMENTAL_BASELINE.items():
        path = args.evidence_directory / name
        document = load_bound(path, digest)
        if (
            document.get("overall_verdict")
            != "J2_RIGHT_ARM_AXIS_ROUNDTRIP_PASS"
            or document.get("j2b_identity") is not None
        ):
            raise RuntimeError("supplemental pre-J2-B baseline contract changed")
        supplemental_inputs.append(
            {
                "path": str(path),
                "sha256": digest,
                "direction": direction,
                "fraction_percent": fraction,
                "role": "pre-J2-B supplemental baseline only",
            }
        )

    report = {
        "schema_version": 1,
        "record_kind": "right_arm_j2_base_acceptance_plan_only",
        "status": STATUS,
        "motion_authorized": False,
        "general_trajectory_authorized": False,
        "runtime_limit_promotion_authorized": False,
        "endpoint_commands_forbidden": True,
        "firmware_version": EXPECTED_FIRMWARE,
        "command_limit_manifest_sha256": EXPECTED_MANIFEST_SHA256,
        "approved_command_limits_raw": [1172, 2932],
        "actively_validated_targets_raw": [1391, 2711],
        "active_coverage": active_results,
        "inputs": {
            "no_motion": {
                "path": str(no_motion_path),
                "sha256": EXPECTED_NO_MOTION_SHA256,
            },
            "active_j2b": active_inputs,
            "supplemental_pre_j2b": supplemental_inputs,
        },
        "interpretation": {
            "upper_25_50_covered_by_j2b_upper75_traversal": True,
            "full_approved_endpoints_physically_tested": False,
            "claim": "bidirectional 75-percent interior active envelope passed",
        },
        "next_required_gate": (
            "gravity-sensitive axis procedure with nonselected joints held at q0"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = file_sha256(args.output)
    print(
        f"{STATUS} output={args.output} sha256={digest}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
