#!/usr/bin/env python3
"""Check left nominal trajectory points against a SHA-bound J1 candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


LEFT_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
SHORT_NAMES = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
STATUS = "J1_LEFT_NOMINAL_ROUTE_COVERAGE_PASS"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bound(path: Path, expected_sha256: str, label: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise ValueError(
            f"{label} SHA mismatch expected={expected_sha256.lower()} "
            f"actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("motion_authorized") is not False:
        raise ValueError(f"{label} must keep motion_authorized=false")
    return document


def candidate_bounds(candidate: dict[str, Any]) -> tuple[tuple[float, float], ...]:
    if candidate.get("status") != (
        "J1_OPERATIONAL_LIMIT_CANDIDATE_REVIEW_REQUIRED"
    ):
        raise ValueError("unexpected J1 candidate status")
    if candidate.get("runtime_change_authorized") is not False:
        raise ValueError("candidate must not authorize runtime changes")
    joints = candidate["arms"]["left"]["joints"]
    bounds: list[tuple[float, float]] = []
    for name in SHORT_NAMES:
        joint = joints[name]
        if joint.get("status") != "PLAN_ONLY_CONTRACTED_CANDIDATE":
            raise ValueError(f"left {name} has no plan-only candidate")
        lower = float(joint["candidate_lower_rad"])
        upper = float(joint["candidate_upper_rad"])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"left {name} has invalid radian bounds")
        bounds.append((lower, upper))
    return tuple(bounds)


def resolve_source(root: Path, source: str) -> Path:
    path = Path(source)
    if path.is_absolute():
        return path
    return root / path


def check_routes(
    root: Path,
    candidate: dict[str, Any],
    manifest: dict[str, Any],
) -> dict[str, Any]:
    if manifest.get("status") != "FULL_PICK_PLACE_PLAN_ONLY_PASS":
        raise ValueError("nominal route manifest did not pass")
    if manifest.get("execution_api_used") is not False:
        raise ValueError("nominal route used an execution API")
    if tuple(manifest.get("joint_names", ())) != LEFT_JOINTS:
        raise ValueError("nominal route joint identity/order changed")

    bounds = candidate_bounds(candidate)
    minima = [math.inf] * 5
    maxima = [-math.inf] * 5
    minimum_clearance = [math.inf] * 5
    phase_results: list[dict[str, Any]] = []
    total_points = 0
    for phase in manifest.get("phase_summaries", ()):
        source = resolve_source(root, str(phase["source"]))
        actual_sha = file_sha256(source)
        if actual_sha != str(phase["source_sha256"]).lower():
            raise ValueError(f"phase SHA mismatch: {source}")
        document = json.loads(source.read_text(encoding="utf-8"))
        if (
            document.get("motion_authorized") is not False
            or document.get("execution_api_used") is not False
        ):
            raise ValueError(f"phase is not plan-only: {source}")
        if tuple(document.get("joint_names", ())) != LEFT_JOINTS:
            raise ValueError(f"phase joint order changed: {source}")

        phase_points = 0
        for segment in document.get("segments", ()):
            if tuple(segment.get("trajectory_joint_names", ())) != LEFT_JOINTS:
                raise ValueError(f"segment joint order changed: {source}")
            positions = segment.get("trajectory_positions_rad")
            if not isinstance(positions, list) or not positions:
                raise ValueError(f"segment has no trajectory points: {source}")
            for point in positions:
                if len(point) != 5 or not all(
                    math.isfinite(float(value)) for value in point
                ):
                    raise ValueError(f"invalid trajectory point: {source}")
                for index, value in enumerate(map(float, point)):
                    lower, upper = bounds[index]
                    if value < lower or value > upper:
                        raise ValueError(
                            "trajectory outside J1 candidate: "
                            f"phase={phase['name']} "
                            f"joint={LEFT_JOINTS[index]} value={value:.9f} "
                            f"bounds={lower:.9f}..{upper:.9f}"
                        )
                    minima[index] = min(minima[index], value)
                    maxima[index] = max(maxima[index], value)
                    minimum_clearance[index] = min(
                        minimum_clearance[index],
                        value - lower,
                        upper - value,
                    )
                phase_points += 1
                total_points += 1
        phase_results.append(
            {
                "name": phase["name"],
                "source": phase["source"],
                "sha256": actual_sha,
                "trajectory_point_count": phase_points,
                "verdict": "PASS",
            }
        )
    if total_points == 0:
        raise ValueError("nominal route contained no trajectory points")

    joints = {
        name: {
            "candidate_lower_rad": bounds[index][0],
            "candidate_upper_rad": bounds[index][1],
            "route_minimum_rad": minima[index],
            "route_maximum_rad": maxima[index],
            "minimum_limit_clearance_rad": minimum_clearance[index],
            "verdict": "PASS",
        }
        for index, name in enumerate(LEFT_JOINTS)
    }
    return {
        "status": STATUS,
        "motion_authorized": False,
        "runtime_change_authorized": False,
        "execution_api_used": False,
        "left_route_coverage": True,
        "right_route_coverage": False,
        "trajectory_point_count": total_points,
        "phases": phase_results,
        "joints": joints,
        "remaining_blockers": [
            "right-arm representative routes do not exist yet",
            "gripper semantic mapping is unresolved",
            "physical zero and model alignment are unresolved",
            "candidate has not been applied to any runtime consumer",
        ],
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--candidate-sha256", required=True)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.set_defaults(root=root)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")
    candidate = load_bound(
        args.candidate, args.candidate_sha256, "J1 candidate"
    )
    manifest = load_bound(
        args.manifest, args.manifest_sha256, "route manifest"
    )
    report = check_routes(args.root, candidate, manifest)
    report["inputs"] = {
        "candidate": {
            "path": str(args.candidate),
            "sha256": args.candidate_sha256.lower(),
        },
        "manifest": {
            "path": str(args.manifest),
            "sha256": args.manifest_sha256.lower(),
        },
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = file_sha256(output)
    for name in LEFT_JOINTS:
        joint = report["joints"][name]
        print(
            "J1_ROUTE_COVERAGE "
            f"joint={name} "
            f"route={joint['route_minimum_rad']:.6f}.."
            f"{joint['route_maximum_rad']:.6f} "
            f"clearance_rad={joint['minimum_limit_clearance_rad']:.6f}"
        )
    print(
        f"STATUS={STATUS} points={report['trajectory_point_count']} "
        f"motion_authorized=false output={output} sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
