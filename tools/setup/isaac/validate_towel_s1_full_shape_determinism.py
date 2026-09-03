#!/usr/bin/env python3
"""Compare independent S1 runs using every final cloth node in env 0."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


PASS_STATUS = "S1_FULL_SHAPE_REPEAT_DETERMINISM_PASS"
EXPECTED_GRASP_MODE_MARKER = "contact_gated_no_slip_retention_used"
DEFAULT_MAXIMUM_NODE_DISTANCE_M = 0.001


def load_result(path: Path) -> dict[str, object]:
    if not path.is_file():
        raise ValueError(f"result does not exist: {path}")
    result = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(result, dict):
        raise ValueError(f"result must be an object: {path}")
    shape = result.get("final_cloth_shape_local_m_env_0")
    if not isinstance(shape, list) or len(shape) != 1024:
        raise ValueError(f"result must contain 1,024 final env-0 nodes: {path}")
    for node in shape:
        if (
            not isinstance(node, list)
            or len(node) != 3
            or any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in node)
        ):
            raise ValueError(f"result contains an invalid final node: {path}")
    attachment = result.get("attachment", {})
    if attachment.get(EXPECTED_GRASP_MODE_MARKER) is not True:
        raise ValueError(f"result did not use contact-gated retention: {path}")
    if result.get("cloth", {}).get("self_collision_enabled") is not True:
        raise ValueError(f"result did not enable cloth self-collision: {path}")
    if result.get("place_release") is None:
        raise ValueError(f"result did not complete place/release: {path}")
    return result


def node_distances_m(
    reference: list[list[float]], repeat: list[list[float]]
) -> list[float]:
    return [
        math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))
        for left, right in zip(reference, repeat, strict=True)
    ]


def percentile_nearest_rank(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    return ordered[max(0, math.ceil(probability * len(ordered)) - 1)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("repeat", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--maximum-node-distance-m",
        type=float,
        default=DEFAULT_MAXIMUM_NODE_DISTANCE_M,
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if not math.isfinite(args.maximum_node_distance_m) or args.maximum_node_distance_m <= 0:
        parser.error("--maximum-node-distance-m must be finite and positive")

    reference = load_result(args.reference)
    repeat = load_result(args.repeat)
    identity_fields = (
        ("identity", reference["identity"], repeat["identity"]),
        (
            "material_sha256",
            reference["material_candidate"]["sha256"],
            repeat["material_candidate"]["sha256"],
        ),
        (
            "gripper_sha256",
            reference["gripper_candidate"]["sha256"],
            repeat["gripper_candidate"]["sha256"],
        ),
        ("urdf_sha256", reference["urdf_sha256"], repeat["urdf_sha256"]),
    )
    mismatches = [name for name, left, right in identity_fields if left != right]
    if mismatches:
        raise ValueError(f"repeat identity mismatch: {mismatches}")

    distances = node_distances_m(
        reference["final_cloth_shape_local_m_env_0"],
        repeat["final_cloth_shape_local_m_env_0"],
    )
    maximum_distance_m = max(distances)
    p95_distance_m = percentile_nearest_rank(distances, 0.95)
    median_distance_m = percentile_nearest_rank(distances, 0.50)
    rms_distance_m = math.sqrt(sum(value * value for value in distances) / len(distances))
    if maximum_distance_m > args.maximum_node_distance_m:
        raise RuntimeError(
            f"full-shape repeat drift {maximum_distance_m:.9f} m exceeds "
            f"{args.maximum_node_distance_m:.9f} m"
        )

    output = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s1_full_shape_repeat_determinism_result",
        "status": PASS_STATUS,
        "reference": {
            "path": str(args.reference),
            "sha256": hashlib.sha256(args.reference.read_bytes()).hexdigest(),
        },
        "repeat": {
            "path": str(args.repeat),
            "sha256": hashlib.sha256(args.repeat.read_bytes()).hexdigest(),
        },
        "node_count": len(distances),
        "maximum_node_distance_m": maximum_distance_m,
        "p95_node_distance_m": p95_distance_m,
        "median_node_distance_m": median_distance_m,
        "rms_node_distance_m": rms_distance_m,
        "maximum_node_distance_limit_m": args.maximum_node_distance_m,
        "identity_match": True,
        "contact_gated_vertical_pinch": True,
        "self_collision_after_closed_jaw_gate": True,
        "full_shape_repeat_determinism_passed": True,
        "physical_robot_motion_authorized": False,
        "motion_commands": 0,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{PASS_STATUS} maximum_node_distance_m={maximum_distance_m:.9f} "
        f"p95_node_distance_m={p95_distance_m:.9f} output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
