#!/usr/bin/env python3
"""Check a pinned H2 route against a temporary, conservative table collider.

This is deliberately plan-only: it never creates an Action client, publishes
no trajectory and sends no serial command.  It temporarily adds one table box
to MoveIt's planning scene, checks every retained MoveIt waypoint at every
vertex of the measured joint-error envelope, writes an immutable report, then
removes that box even if a check fails.

The check is fail-closed.  It is a waypoint-envelope audit, not permission to
execute a merged H2 trajectory: intended pick/place contact and continuous
between-waypoint certification remain separate gates.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
from itertools import product
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
ARM_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
GRIPPER_JOINT = "left_gripper_joint"
ALL_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)
TABLE_OBJECT_ID = "h2_temporary_verified_table"
TABLE_FRAME = "left_base_link"
MAX_CONTACTS_TO_RECORD = 8
PHASE_TO_H2_LEG = {
    "q0_to_pick_pregrasp": "pick_pregrasp",
    "pick_pregrasp_to_grasp": "pick_grasp",
    "pick_grasp_to_lift20": "lift",
    "lift_to_place_pregrasp": "place_pregrasp",
    "place_pregrasp_to_place": "place_grasp",
    "place_to_retreat": "retreat",
    "place_pregrasp_to_q0": "q0_return",
}


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def read_pinned_json(path: Path, expected_sha256: str, label: str) -> dict[str, Any]:
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(f"{label} sha256 mismatch expected={expected_sha256} actual={actual}")
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{label} must be a JSON object")
    return document


def finite_vector(value: Any, count: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must contain exactly {count} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def load_manifest(path: Path, expected_sha256: str) -> tuple[dict[str, Any], str]:
    document = read_pinned_json(path, expected_sha256, "manifest")
    if (
        document.get("status") != "FULL_PICK_PLACE_PLAN_ONLY_PASS"
        or document.get("execution_api_used") is not False
        or document.get("motion_authorized") is not False
        or tuple(document.get("joint_names", ())) != ARM_JOINTS
    ):
        raise ValueError("complete plan-only five-joint manifest is required")
    return document, sha256_file(path)


def load_h2_envelope(path: Path, expected_sha256: str) -> tuple[dict[str, Any], str]:
    document = read_pinned_json(path, expected_sha256, "H2 evidence")
    envelope = (document.get("startup_trend") or {}).get("h2_tracking_envelope")
    if document.get("status") != "PICK_PLACE_ONCE_RESIDENT_COMPLETE" or not isinstance(envelope, dict):
        raise ValueError("complete H2 resident evidence is required")
    maximum = envelope.get("maximum_error_raw")
    if (
        not isinstance(maximum, list)
        or len(maximum) != len(ALL_JOINTS)
        or not all(isinstance(value, int) and value >= 0 for value in maximum)
        or envelope.get("valid_leg_count", 0) <= 0
        or envelope.get("requested_completed_samples", 0) <= 0
    ):
        raise ValueError("H2 error envelope is incomplete")
    return envelope, sha256_file(path)


def load_phase_envelopes(document: dict[str, Any]) -> dict[str, list[int]]:
    """Return one coordinate-wise maximum error bound for each route phase.

    A global maximum mixes unrelated postures: e.g. a shoulder error observed
    while transferring to place must not be asserted at the pick descent.  Each
    map entry still uses the full coordinate-wise max of its own completed H2
    leg, so it remains a fail-closed bound for that phase.
    """
    by_tag: dict[str, list[int]] = {}
    for leg in document.get("legs", []):
        tag = leg.get("tag")
        maximum = leg.get("h2_tracking_error_max_raw")
        if (
            isinstance(tag, str)
            and isinstance(maximum, list)
            and len(maximum) == len(ALL_JOINTS)
            and all(isinstance(value, int) and value >= 0 for value in maximum)
            and leg.get("ok") is True
        ):
            by_tag[tag] = maximum
    missing = sorted(set(PHASE_TO_H2_LEG.values()) - set(by_tag))
    if missing:
        raise ValueError("H2 evidence lacks completed phase envelopes: " + ",".join(missing))
    return {phase: by_tag[tag] for phase, tag in PHASE_TO_H2_LEG.items()}


def validate_route_report(
    path: Path,
    expected_sha256: str,
    *,
    manifest_sha256: str,
    h2_evidence_sha256: str,
) -> str:
    document = read_pinned_json(path, expected_sha256, "nominal route report")
    if (
        document.get("status")
        != "H2_NOMINAL_ROUTES_PLAN_ONLY_PASS_AWAITING_ROUTE_ENVELOPE_AND_ROBUST_COLLISION_CHECK"
        or document.get("tracking_envelope_route_matches_inputs") is not True
        or document.get("manifest", {}).get("sha256") != manifest_sha256
        or document.get("tracking_envelope", {}).get("sha256") != h2_evidence_sha256
    ):
        raise ValueError("nominal route report does not pin this manifest and H2 evidence")
    return sha256_file(path)


def load_table_top(path: Path, expected_sha256: str) -> tuple[float, str]:
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(f"table validation sha256 mismatch expected={expected_sha256} actual={actual}")
    # The evidence is YAML but its required table height is intentionally a
    # simple, stable scalar.  Avoid a new YAML runtime dependency.
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("table_z_in_left_base_link_m:"):
            value = float(line.split(":", 1)[1].strip())
            if math.isfinite(value):
                return value, actual
    raise ValueError("table validation has no finite table_z_in_left_base_link_m")


def load_calibration_limits(path: Path) -> tuple[tuple[float, float], ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != len(ALL_JOINTS):
        raise ValueError("calibration must contain six joints")
    scale = 2.0 * math.pi / float(document.get("raw_units_per_turn", 4096))
    result = []
    for joint in joints:
        direction = float(joint["positive_raw_direction"])
        zero = float(joint["zero_raw"])
        endpoints = [
            direction * (float(joint[key]) - zero) * scale
            for key in ("minimum_raw", "maximum_raw")
        ]
        result.append((min(endpoints), max(endpoints)))
    return tuple(result)


def error_vertices(raw_errors: Iterable[int]) -> list[tuple[float, ...]]:
    scale = 2.0 * math.pi / 4096.0
    axes = [(-value * scale, value * scale) if value else (0.0,) for value in raw_errors]
    return [tuple(values) for values in product(*axes)]


def phase_reverse_flags(manifest: dict[str, Any]) -> dict[tuple[str, str], bool]:
    result: dict[tuple[str, str], bool] = {}
    for phase in manifest.get("phase_summaries", []):
        source = phase.get("source")
        digest = phase.get("source_sha256")
        if not isinstance(source, str) or not isinstance(digest, str):
            raise ValueError("manifest phase source is incomplete")
        result[(source, digest)] = bool(phase.get("reversed"))
    return result


def retained_waypoints(manifest: dict[str, Any], root: Path) -> list[dict[str, Any]]:
    """Load the exact stored MoveIt joint waypoints in manifest order."""
    reverse_flags = phase_reverse_flags(manifest)
    records: list[dict[str, Any]] = []
    gripper_open = next(
        (float(step["target_position_rad"]) for step in manifest.get("steps", [])
         if step.get("kind") == "gripper" and step.get("phase") == "place_release"),
        None,
    )
    if gripper_open is None:
        raise ValueError("manifest lacks place_release gripper position")
    gripper = gripper_open
    for step in manifest.get("steps", []):
        if step.get("kind") == "gripper":
            gripper = float(step["target_position_rad"])
            continue
        if step.get("kind") != "arm":
            raise ValueError("manifest contains an unknown command step")
        source = step.get("source")
        digest = step.get("source_sha256")
        index = step.get("source_segment_index")
        if not isinstance(source, str) or not isinstance(digest, str) or not isinstance(index, int):
            raise ValueError("arm step provenance is incomplete")
        source_path = Path(source)
        if not source_path.is_absolute():
            source_path = root / source_path
        if sha256_file(source_path) != digest:
            raise ValueError(f"phase source changed: {source_path}")
        phase = json.loads(source_path.read_text(encoding="utf-8"))
        candidates = [segment for segment in phase.get("segments", []) if segment.get("index") == index]
        if len(candidates) != 1:
            raise ValueError(f"phase source segment is missing: {source_path}:{index}")
        segment = candidates[0]
        if tuple(segment.get("trajectory_joint_names", ())) != ARM_JOINTS:
            raise ValueError("phase was made before exact MoveIt waypoints were retained")
        points = [finite_vector(point, len(ARM_JOINTS), "trajectory point") for point in segment.get("trajectory_positions_rad", [])]
        if not points:
            raise ValueError("phase segment has no retained trajectory points")
        if reverse_flags.get((source, digest), False):
            points.reverse()
        for point_index, point in enumerate(points):
            records.append({
                "manifest_step": int(step["index"]),
                "phase": str(step["phase"]),
                "source_segment_index": index,
                "trajectory_point_index": point_index,
                "positions_rad": (*point, gripper),
            })
    if not records:
        raise ValueError("manifest has no arm waypoints")
    return records


def clamp_variant(
    nominal: tuple[float, ...], error: tuple[float, ...], limits: tuple[tuple[float, float], ...]
) -> tuple[float, ...]:
    return tuple(
        min(upper, max(lower, value + delta))
        for value, delta, (lower, upper) in zip(nominal, error, limits, strict=True)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--h2-evidence", type=Path, required=True)
    parser.add_argument("--h2-evidence-sha256", required=True)
    parser.add_argument("--route-report", type=Path, required=True)
    parser.add_argument("--route-report-sha256", required=True)
    parser.add_argument("--table-validation", type=Path, required=True)
    parser.add_argument("--table-validation-sha256", required=True)
    parser.add_argument("--calibration", type=Path, default=ROOT / "config" / "single_arm_calibration.json")
    parser.add_argument("--table-x-min-m", type=float, required=True)
    parser.add_argument("--table-x-max-m", type=float, required=True)
    parser.add_argument("--table-y-min-m", type=float, required=True)
    parser.add_argument("--table-y-max-m", type=float, required=True)
    parser.add_argument("--table-thickness-m", type=float, default=0.10)
    parser.add_argument("--table-registration-margin-m", type=float, default=0.010)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    if not args.plan_only:
        parser.error("--plan-only is required; this tool never executes motion")
    if not all(math.isfinite(value) for value in (
        args.table_x_min_m, args.table_x_max_m, args.table_y_min_m,
        args.table_y_max_m, args.table_thickness_m, args.table_registration_margin_m,
    )):
        parser.error("table dimensions must be finite")
    if args.table_x_min_m >= args.table_x_max_m or args.table_y_min_m >= args.table_y_max_m:
        parser.error("table minimum bounds must be less than maximum bounds")
    if args.table_thickness_m <= 0.0 or args.table_registration_margin_m < 0.0:
        parser.error("table thickness must be positive and margin non-negative")
    for field in ("manifest_sha256", "h2_evidence_sha256", "route_report_sha256", "table_validation_sha256"):
        value = getattr(args, field)
        if len(value) != 64 or any(char not in "0123456789abcdefABCDEF" for char in value):
            parser.error(f"--{field.replace('_', '-')} must be a 64-hex SHA256")
    return args


def wait_future(node: Any, future: Any, timeout_s: float = 5.0) -> Any:
    import rclpy
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done() or future.result() is None:
        raise TimeoutError("MoveIt service response timeout")
    return future.result()


def scene_message(*, operation: int, args: argparse.Namespace, table_top_m: float) -> Any:
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    object_message = CollisionObject()
    object_message.id = TABLE_OBJECT_ID
    object_message.header.frame_id = TABLE_FRAME
    object_message.operation = operation
    if operation == CollisionObject.REMOVE:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [object_message]
        return scene
    margin = args.table_registration_margin_m
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        args.table_x_max_m - args.table_x_min_m + 2.0 * margin,
        args.table_y_max_m - args.table_y_min_m + 2.0 * margin,
        args.table_thickness_m + margin,
    ]
    pose = Pose()
    pose.position.x = (args.table_x_min_m + args.table_x_max_m) / 2.0
    pose.position.y = (args.table_y_min_m + args.table_y_max_m) / 2.0
    # Raise the top by the registration margin; all thickness lies below it.
    pose.position.z = table_top_m + margin - primitive.dimensions[2] / 2.0
    pose.orientation.w = 1.0
    object_message.primitives = [primitive]
    object_message.primitive_poses = [pose]
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [object_message]
    return scene


def check_state(node: Any, client: Any, positions: tuple[float, ...]) -> tuple[bool, list[tuple[str, str]]]:
    from moveit_msgs.srv import GetStateValidity
    request = GetStateValidity.Request()
    request.group_name = "left_arm"
    request.robot_state.is_diff = False
    request.robot_state.joint_state.name = list(ALL_JOINTS)
    request.robot_state.joint_state.position = list(positions)
    response = wait_future(node, client.call_async(request))
    contacts = [
        (contact.contact_body_1, contact.contact_body_2)
        for contact in response.contacts[:MAX_CONTACTS_TO_RECORD]
    ]
    return bool(response.valid), contacts


def apply_scene(node: Any, client: Any, scene: Any) -> bool:
    from moveit_msgs.srv import ApplyPlanningScene
    request = ApplyPlanningScene.Request()
    request.scene = scene
    return bool(wait_future(node, client.call_async(request)).success)


def main() -> int:
    args = parse_args()
    try:
        manifest, manifest_sha = load_manifest(args.manifest, args.manifest_sha256)
        envelope, evidence_sha = load_h2_envelope(args.h2_evidence, args.h2_evidence_sha256)
        evidence_document = read_pinned_json(args.h2_evidence, args.h2_evidence_sha256, "H2 evidence")
        phase_envelopes = load_phase_envelopes(evidence_document)
        route_report_sha = validate_route_report(
            args.route_report,
            args.route_report_sha256,
            manifest_sha256=manifest_sha,
            h2_evidence_sha256=evidence_sha,
        )
        table_top, table_sha = load_table_top(args.table_validation, args.table_validation_sha256)
        limits = load_calibration_limits(args.calibration)
        waypoints = retained_waypoints(manifest, ROOT)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"H2_TABLE_ENVELOPE_PLAN_ONLY_FAIL reason={error}")
        return 2

    import rclpy
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity

    rclpy.init()
    node = rclpy.create_node("h2_table_tracking_envelope_check")
    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    added = False
    checked = 0
    collision: dict[str, Any] | None = None
    failure: str | None = None
    stage = "service_discovery"
    try:
        if not apply_client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("MoveIt apply_planning_scene service unavailable")
        if not validity_client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("MoveIt check_state_validity service unavailable")
        stage = "add_temporary_table"
        applied = apply_scene(
            node, apply_client,
            scene_message(operation=CollisionObject.ADD, args=args, table_top_m=table_top),
        )
        if not applied:
            raise RuntimeError("MoveIt rejected temporary table collision object")
        added = True
        for waypoint in waypoints:
            nominal = tuple(waypoint["positions_rad"])
            phase_error_raw = phase_envelopes.get(str(waypoint["phase"]))
            if phase_error_raw is None:
                raise ValueError(f"no H2 envelope is mapped to phase={waypoint['phase']}")
            variants = error_vertices(phase_error_raw)
            for vertex_index, error in enumerate(variants):
                stage = "state_validity"
                candidate = clamp_variant(nominal, error, limits)
                valid, contacts = check_state(node, validity_client, candidate)
                checked += 1
                if not valid:
                    collision = {
                        **waypoint,
                        "vertex_index": vertex_index,
                        "joint_error_rad": list(error),
                        "checked_positions_rad": list(candidate),
                        "contacts": contacts,
                    }
                    break
            if collision is not None:
                break
    except Exception as error:  # preserve a fail-closed report below
        failure = f"{type(error).__name__}: {error!r}"
    finally:
        cleanup_succeeded = False
        if added:
            try:
                cleanup_succeeded = apply_scene(
                    node, apply_client,
                    scene_message(operation=CollisionObject.REMOVE, args=args, table_top_m=table_top),
                )
            except Exception as error:
                failure = f"{failure}; cleanup failed: {error}" if failure else f"cleanup failed: {error}"
        else:
            cleanup_succeeded = True
        node.destroy_node()
        rclpy.shutdown()

    passed = failure is None and collision is None and cleanup_succeeded
    report = {
        "schema_version": 1,
        "status": "H2_TABLE_ENVELOPE_WAYPOINT_CHECK_PASS" if passed else "H2_TABLE_ENVELOPE_WAYPOINT_CHECK_FAIL",
        "execution_api_used": False,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "manifest": {"path": str(args.manifest), "sha256": manifest_sha},
        "h2_evidence": {
            "path": str(args.h2_evidence), "sha256": evidence_sha,
            "global_maximum_error_raw": envelope["maximum_error_raw"],
            "phase_maximum_error_raw": phase_envelopes,
        },
        "nominal_route_report": {"path": str(args.route_report), "sha256": route_report_sha},
        "table_validation": {"path": str(args.table_validation), "sha256": table_sha, "top_z_m": table_top},
        "table_box": {
            "frame": TABLE_FRAME,
            "x_min_m": args.table_x_min_m,
            "x_max_m": args.table_x_max_m,
            "y_min_m": args.table_y_min_m,
            "y_max_m": args.table_y_max_m,
            "thickness_m": args.table_thickness_m,
            "registration_margin_m": args.table_registration_margin_m,
        },
        "waypoint_count": len(waypoints),
        "error_vertex_count_per_phase": {
            phase: len(error_vertices(values)) for phase, values in phase_envelopes.items()
        },
        "state_validity_requests_completed": checked,
        "temporary_scene_cleanup_succeeded": cleanup_succeeded,
        "collision": collision,
        "failure_stage": None if failure is None else stage,
        "failure": failure,
        "continuous_between_waypoints_certified": False,
        "intended_pick_place_contact_modeled": False,
        "next_required_step": (
            "review waypoint-envelope result; retain explicit manual contact gates and "
            "add continuous collision certification before any automatic H2 execution"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    args.output.write_text(encoded, encoding="utf-8")
    print(f"STATUS={report['status']}")
    print(f"WAYPOINTS={len(waypoints)} ERROR_VERTICES_PER_PHASE=64 REQUESTS={checked}")
    print(f"TEMPORARY_SCENE_CLEANUP_SUCCEEDED={str(cleanup_succeeded).lower()}")
    if collision is not None:
        print("COLLISION=detected")
    if failure:
        print(f"FAILURE={failure}")
    print(f"OUTPUT={args.output} SHA256={sha256(encoded.encode()).hexdigest()}")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
