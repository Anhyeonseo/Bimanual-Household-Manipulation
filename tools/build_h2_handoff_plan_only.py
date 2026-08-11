#!/usr/bin/env python3
"""Inspect H1 segment-leg handoffs without opening ROS, serial, or Actions.

H2 may remove an intermediate stop only when the preceding collision-checked
segment terminates at exactly the next segment's collision-checked anchor.
This tool deliberately does *not* fill a gap with interpolation: a connector
without a fresh collision check would turn a plan-only review into motion
authority.  Its output is a SHA-pinned rejection/ready report for the next
MoveIt collision-checking builder.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
from typing import Any


STATUS_REJECTED = "H2_HANDOFF_PLAN_ONLY_REJECTED"
STATUS_READY = "H2_HANDOFF_PLAN_ONLY_READY_FOR_COLLISION_CHECK"
JOINT_COUNT = 5
EXACT_HANDOFF_TOLERANCE_RAD = 1.0e-12


@dataclass(frozen=True, slots=True)
class PinnedLeg:
    label: str
    path: Path
    sha256: str
    start_rad: tuple[float, ...]
    end_rad: tuple[float, ...]
    segment_count: int


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _positions(value: Any, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != JOINT_COUNT:
        raise ValueError(f"{field} must contain exactly {JOINT_COUNT} joints")
    result = tuple(float(item) for item in value)
    if not all(abs(item) < 10.0 for item in result):
        raise ValueError(f"{field} contains an implausible joint position")
    return result


def load_pinned_leg(label: str, path: Path, expected_sha256: str) -> PinnedLeg:
    actual_sha256 = sha256_file(path)
    if actual_sha256.lower() != expected_sha256.lower():
        raise ValueError(
            f"{label} sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={actual_sha256}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    status = document.get("status")
    if not isinstance(status, str) or not status.endswith("_SEGMENT_PLAN_ONLY_PASS"):
        raise ValueError(f"{label} is not a passing segment plan")
    if document.get("execution_api_used") is not False:
        raise ValueError(f"{label} must not use an execution API")
    if document.get("motion_authorized") is not False:
        raise ValueError(f"{label} must remain plan-only")
    segments = document.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError(f"{label} has no collision-checked segments")

    first = _positions(segments[0].get("expected_start_positions_rad"), f"{label}.start")
    previous_end: tuple[float, ...] | None = None
    for index, segment in enumerate(segments, start=1):
        if segment.get("success") is not True or segment.get("moveit_error_code") != 1:
            raise ValueError(f"{label} segment {index} is not MoveIt-successful")
        start = _positions(
            segment.get("expected_start_positions_rad"), f"{label}.segment[{index}].start"
        )
        end = _positions(
            segment.get("target_positions_rad"), f"{label}.segment[{index}].end"
        )
        if previous_end is not None and any(
            abs(a - b) > EXACT_HANDOFF_TOLERANCE_RAD
            for a, b in zip(previous_end, start, strict=True)
        ):
            raise ValueError(f"{label} breaks inside segment chain at {index}")
        previous_end = end
    assert previous_end is not None
    return PinnedLeg(label, path, actual_sha256, first, previous_end, len(segments))


def load_tracking_envelope(paths: list[Path]) -> dict[str, Any] | None:
    """Combine only complete H2 leg telemetry already persisted by H1.

    A missing envelope is a reportable H2 blocker, not a reason to invent a
    tracking margin.  Each input digest is retained so a later collision check
    can trace exactly which execution evidence was used.
    """
    records: list[tuple[Path, str, dict[str, Any]]] = []
    for path in paths:
        digest = sha256_file(path)
        document = json.loads(path.read_text(encoding="utf-8"))
        envelope = document.get("startup_trend", {}).get("h2_tracking_envelope")
        if not isinstance(envelope, dict):
            continue
        maximum = envelope.get("maximum_error_raw")
        count = envelope.get("valid_leg_count")
        if (
            not isinstance(maximum, list)
            or len(maximum) != 6
            or not all(isinstance(value, int) and value >= 0 for value in maximum)
            or not isinstance(count, int)
            or count <= 0
        ):
            continue
        records.append((path, digest, envelope))
    if not records:
        return None
    return {
        "source_count": len(records),
        "sources": [
            {"path": str(path), "sha256": digest}
            for path, digest, _ in records
        ],
        "maximum_error_raw": [
            max(envelope["maximum_error_raw"][index] for _, _, envelope in records)
            for index in range(6)
        ],
        "valid_leg_count": sum(envelope["valid_leg_count"] for _, _, envelope in records),
    }


def build_report(legs: list[PinnedLeg], envelope: dict[str, Any] | None) -> dict[str, Any]:
    if len(legs) < 2:
        raise ValueError("at least two ordered legs are required")
    handoffs = []
    rejection_reasons: list[str] = []
    for previous, following in zip(legs[:-1], legs[1:], strict=True):
        per_joint = [
            abs(end - start)
            for end, start in zip(previous.end_rad, following.start_rad, strict=True)
        ]
        maximum = max(per_joint)
        continuous = maximum <= EXACT_HANDOFF_TOLERANCE_RAD
        handoffs.append(
            {
                "from": previous.label,
                "to": following.label,
                "per_joint_gap_rad": per_joint,
                "maximum_gap_rad": maximum,
                "exactly_continuous": continuous,
            }
        )
        if not continuous:
            rejection_reasons.append(
                f"handoff {previous.label}->{following.label} has an unchecked gap"
            )
    if envelope is None:
        rejection_reasons.append("complete H2 tracking envelope evidence is missing")
    return {
        "schema_version": 1,
        "status": STATUS_REJECTED if rejection_reasons else STATUS_READY,
        "execution_api_used": False,
        "motion_authorized": False,
        "collision_checked": False,
        "exact_handoff_tolerance_rad": EXACT_HANDOFF_TOLERANCE_RAD,
        "legs": [
            {
                "label": leg.label,
                "path": str(leg.path),
                "sha256": leg.sha256,
                "segment_count": leg.segment_count,
                "start_rad": list(leg.start_rad),
                "end_rad": list(leg.end_rad),
            }
            for leg in legs
        ],
        "handoffs": handoffs,
        "tracking_envelope": envelope,
        "rejection_reasons": rejection_reasons,
        "next_required_step": (
            "fresh collision-checked connector planning with expanded geometry"
            if rejection_reasons
            else "fresh collision check with tracking-expanded geometry"
        ),
    }


def parse_pinned_argument(value: str) -> tuple[str, Path, str]:
    try:
        label, source = value.split("=", 1)
        path_text, digest = source.rsplit("@", 1)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected LABEL=PATH@SHA256") from error
    if not label or len(digest) != 64 or any(character not in "0123456789abcdefABCDEF" for character in digest):
        raise argparse.ArgumentTypeError("expected LABEL=PATH@64-hex-SHA256")
    return label, Path(path_text), digest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--leg", action="append", type=parse_pinned_argument, required=True,
        help="ordered LABEL=PATH@SHA256; repeated at least twice",
    )
    parser.add_argument("--h1-evidence", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    labels = [label for label, _, _ in arguments.leg]
    if len(set(labels)) != len(labels):
        parser.error("--leg labels must be unique")
    try:
        legs = [load_pinned_leg(label, path, digest) for label, path, digest in arguments.leg]
        report = build_report(legs, load_tracking_envelope(arguments.h1_evidence))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"STATUS={report['status']}")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256_file(arguments.output)}")
    for reason in report["rejection_reasons"]:
        print(f"REJECTED={reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
