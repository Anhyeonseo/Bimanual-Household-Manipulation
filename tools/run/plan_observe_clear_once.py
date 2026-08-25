#!/usr/bin/env python3
"""Create one exact dual-arm OBSERVE_CLEAR MoveIt plan without execution."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.desk_task_runtime import CANONICAL_JOINTS  # noqa: E402


STATUS = "OBSERVE_CLEAR_PLAN_ONLY_PASS"
JOINT_TOPIC = "/joint_states"
MAX_GOAL_RESIDUAL_RAD = 0.001
APPLY_SCENE_SERVICE = "/apply_planning_scene"
GET_SCENE_SERVICE = "/get_planning_scene"
STATE_VALIDITY_SERVICE = "/check_state_validity"
TABLE_COLLISION_THICKNESS_M = 0.020
PATH_VALIDATION_STEP_RAD = 0.01
MAXIMUM_EXCEPTION_DEPTH_M = 0.004
RIGHT_CLEARANCE_SHOULDER_RAD = 0.0
RIGHT_CLEARANCE_ELBOW_RAD = -0.2
OBSERVE_CLEAR_CONTACT_EXCEPTIONS = frozenset(
    {
        frozenset(("left_shoulder_link", "left_gripper_link")),
        frozenset(("left_shoulder_link", "left_moving_jaw_link")),
        frozenset(("right_shoulder_link", "right_lower_arm_link")),
        frozenset(("right_shoulder_link", "right_gripper_link")),
        frozenset(("right_shoulder_link", "right_moving_jaw_link")),
    }
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_inputs(contract_path: Path, shadow_path: Path, manifest_path: Path):
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    shadow = yaml.safe_load(shadow_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    observation = contract.get("workcell_observation_candidate", {})
    clear = observation.get("observe_clear", {})
    clear_candidate = (
        clear.get("status") == "VISUAL_CANDIDATE"
        and clear.get("motion_reproducibility_validated") is False
    )
    clear_validated = (
        clear.get("status") == "SUPERVISED_ROUNDTRIP_VALIDATED"
        and clear.get("motion_reproducibility_validated") is True
        and clear.get("all_four_towel_corners_visible") is True
        and clear.get("coordinated_stop_verified") is True
    )
    if (
        contract.get("motion_authorized") is not False
        or observation.get("motion_authorized") is not False
        or not (clear_candidate or clear_validated)
        or clear.get("visual_towel_occlusion") is not False
        or tuple(clear.get("joint_names", ())) != CANONICAL_JOINTS
        or len(clear.get("joint_positions_rad", ())) != 12
    ):
        raise RuntimeError("OBSERVE_CLEAR task contract is not a fail-closed visual candidate")
    if (
        shadow.get("status") != "RIGHT_REGISTRATION_WORKCELL_SHADOW_VALIDATED"
        or shadow.get("motion_authorized") is not False
        or shadow.get("robot_target_available") is not False
        or shadow.get("tabletop_object_validation_performed") is not False
    ):
        raise RuntimeError("right registration shadow gate is invalid")
    registration_source = shadow.get("sources", {}).get("candidate", {})
    manifest_registration = manifest.get("right_registration_candidate", {})
    if (
        manifest.get("simulation_only") is not True
        or manifest.get("motion_authorized") is not False
        or manifest_registration.get("runtime_promotion_authorized") is not False
        or manifest_registration.get("sha256") != registration_source.get("sha256")
    ):
        raise RuntimeError("registered preview manifest does not match the shadow gate")
    urdf_path = Path(str(manifest.get("urdf", ""))).resolve()
    configured = os.environ.get("SO101_DUAL_URDF_PATH", "")
    if not configured or Path(configured).resolve() != urdf_path:
        raise RuntimeError("SO101_DUAL_URDF_PATH is not the registered preview URDF")
    if not urdf_path.is_file() or sha256_file(urdf_path) != manifest.get("urdf_sha256"):
        raise RuntimeError("registered preview URDF SHA mismatch")
    worktable_source = shadow.get("sources", {}).get("worktable", {})
    worktable_path = Path(str(worktable_source.get("path", ""))).resolve()
    if (
        not worktable_path.is_file()
        or sha256_file(worktable_path) != worktable_source.get("sha256")
    ):
        raise RuntimeError("shadow worktable source is missing or changed")
    worktable = yaml.safe_load(worktable_path.read_text(encoding="utf-8"))
    if (
        worktable.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or worktable.get("base_registration", {}).get("transform_validated") is not True
    ):
        raise RuntimeError("validated worktable source is invalid")
    target = tuple(float(value) for value in clear["joint_positions_rad"])
    if not all(math.isfinite(value) for value in target):
        raise RuntimeError("OBSERVE_CLEAR target contains a non-finite value")
    return contract, shadow, manifest, worktable, target


def staged_right_clearance_targets(start, target):
    """Keep the right jaw clear of the table/shoulder while unfolding to clear."""
    current = list(start)
    result = []
    for index, value in (
        (6, target[6]),
        (7, RIGHT_CLEARANCE_SHOULDER_RAD),
        (8, RIGHT_CLEARANCE_ELBOW_RAD),
        (7, target[7]),
        (8, target[8]),
        (9, target[9]),
    ):
        current[index] = float(value)
        result.append(tuple(current))
    result.append(tuple(target))
    return tuple(result)


def validated_table_scene(worktable: dict) -> PlanningScene:
    board = worktable["board"]
    span = tuple(float(value) for value in board["calibrated_span_m"])
    origin = tuple(float(value) for value in board["origin_in_left_base_link_xy_m"])
    table_z = float(board["table_z_in_left_base_link_m"])
    if (
        len(span) != 2
        or len(origin) != 2
        or min(span) <= 0.30
        or not all(math.isfinite(value) for value in (*span, *origin, table_z))
    ):
        raise RuntimeError("validated worktable collision dimensions are invalid")
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [span[0], span[1], TABLE_COLLISION_THICKNESS_M]
    pose = Pose()
    pose.position.x = origin[0] + span[0] / 2.0
    pose.position.y = origin[1] + span[1] / 2.0
    pose.position.z = table_z - TABLE_COLLISION_THICKNESS_M / 2.0
    pose.orientation.w = 1.0
    collision = CollisionObject()
    collision.header.frame_id = "left_base_link"
    collision.id = "validated_worktable_region"
    collision.primitives = [primitive]
    collision.primitive_poses = [pose]
    collision.operation = CollisionObject.ADD
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [collision]
    return scene


def collision_matrix_with_exceptions(matrix, enabled: bool):
    """Toggle only the measured shallow folded-pose mesh contacts."""
    result = copy.deepcopy(matrix)
    names = list(result.entry_names)
    required = sorted({name for pair in OBSERVE_CLEAR_CONTACT_EXCEPTIONS for name in pair})
    for name in required:
        if name in names:
            continue
        names.append(name)
        for row in result.entry_values:
            row.enabled.append(False)
        new = AllowedCollisionEntry()
        new.enabled = [False] * len(names)
        result.entry_values.append(new)
    if len(result.entry_values) != len(names):
        raise RuntimeError("MoveIt allowed-collision matrix is not square")
    for row in result.entry_values:
        if len(row.enabled) != len(names):
            raise RuntimeError("MoveIt allowed-collision matrix row is not square")
    result.entry_names = names
    index = {name: offset for offset, name in enumerate(names)}
    for pair in OBSERVE_CLEAR_CONTACT_EXCEPTIONS:
        first, second = tuple(pair)
        result.entry_values[index[first]].enabled[index[second]] = enabled
        result.entry_values[index[second]].enabled[index[first]] = enabled
    return result


def scene_with_collision_matrix(matrix) -> PlanningScene:
    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = matrix
    return scene


def dense_arm_path(start, trajectory_names, trajectory, step_rad=PATH_VALIDATION_STEP_RAD):
    start_by_name = dict(zip(CANONICAL_JOINTS, start, strict=True))
    previous = tuple(float(start_by_name[name]) for name in trajectory_names)
    samples = [previous]
    for item in trajectory:
        target = tuple(float(value) for value in item.positions)
        maximum = max(abs(b - a) for a, b in zip(previous, target, strict=True))
        count = max(1, math.ceil(maximum / step_rad))
        for index in range(1, count + 1):
            fraction = index / count
            samples.append(
                tuple(
                    a + (b - a) * fraction
                    for a, b in zip(previous, target, strict=True)
                )
            )
        previous = target
    return samples


def validate_contact_set(contacts) -> float:
    maximum = 0.0
    for contact in contacts:
        pair = frozenset((contact.contact_body_1, contact.contact_body_2))
        depth = float(contact.depth)
        if pair not in OBSERVE_CLEAR_CONTACT_EXCEPTIONS:
            raise RuntimeError(
                "strict path validation found an unapproved contact: "
                f"{contact.contact_body_1}--{contact.contact_body_2} depth={depth}"
            )
        if not math.isfinite(depth) or depth > MAXIMUM_EXCEPTION_DEPTH_M:
            raise RuntimeError(
                "folded-pose mesh contact exceeds the measured shallow bound: "
                f"{contact.contact_body_1}--{contact.contact_body_2} depth={depth}"
            )
        maximum = max(maximum, depth)
    return maximum


def wait_joint_state(node: Node, messages: list[JointState], timeout_s: float) -> tuple[float, ...]:
    deadline = time.monotonic() + timeout_s
    while not messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not messages:
        raise RuntimeError(f"timeout waiting for {JOINT_TOPIC}")
    message = messages[-1]
    if tuple(message.name) != CANONICAL_JOINTS or len(message.position) != 12:
        raise RuntimeError("joint state is not the canonical complete 12-axis vector")
    result = tuple(float(value) for value in message.position)
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError("joint state contains a non-finite value")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true", required=True)
    parser.add_argument(
        "--contract", type=Path, default=ROOT / "config/towel_task_contract.candidate.yaml"
    )
    parser.add_argument("--shadow-validation", type=Path, required=True)
    parser.add_argument("--registered-urdf-manifest", type=Path, required=True)
    parser.add_argument(
        "--staged-right-clearance",
        action="store_true",
        help="Use the validated base/shoulder/elbow/wrist ordering",
    )
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing plan: {args.output}")
    return args


def main() -> int:
    args = parse_args()
    contract, shadow, manifest, worktable, target = load_inputs(
        args.contract, args.shadow_validation, args.registered_urdf_manifest
    )
    rclpy.init()
    node = Node("observe_clear_plan_only")
    messages: list[JointState] = []
    node.create_subscription(JointState, JOINT_TOPIC, messages.append, 10)
    client = node.create_client(planning.GetMotionPlan, planning.PLAN_SERVICE)
    scene_client = node.create_client(ApplyPlanningScene, APPLY_SCENE_SERVICE)
    get_scene_client = node.create_client(GetPlanningScene, GET_SCENE_SERVICE)
    validity_client = node.create_client(GetStateValidity, STATE_VALIDITY_SERVICE)
    strict_matrix = None
    exceptions_enabled = False
    try:
        for name, service_client in (
            (planning.PLAN_SERVICE, client),
            (APPLY_SCENE_SERVICE, scene_client),
            (GET_SCENE_SERVICE, get_scene_client),
            (STATE_VALIDITY_SERVICE, validity_client),
        ):
            if not service_client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")
        scene_request = ApplyPlanningScene.Request()
        scene_request.scene = validated_table_scene(worktable)
        scene_response = planning.wait_future(
            node, scene_client.call_async(scene_request), args.timeout_s
        )
        if not scene_response.success:
            raise RuntimeError("MoveIt rejected the validated worktable collision scene")
        get_scene_request = GetPlanningScene.Request()
        get_scene_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        strict_scene = planning.wait_future(
            node, get_scene_client.call_async(get_scene_request), args.timeout_s
        ).scene
        strict_matrix = strict_scene.allowed_collision_matrix
        exception_matrix = collision_matrix_with_exceptions(strict_matrix, True)
        exception_request = ApplyPlanningScene.Request()
        exception_request.scene = scene_with_collision_matrix(exception_matrix)
        exception_response = planning.wait_future(
            node, scene_client.call_async(exception_request), args.timeout_s
        )
        if not exception_response.success:
            raise RuntimeError("MoveIt rejected the bounded folded-pose collision exceptions")
        exceptions_enabled = True
        start = wait_joint_state(node, messages, args.timeout_s)
        if args.staged_right_clearance:
            segment_targets = staged_right_clearance_targets(start, target)
        else:
            segment_targets = (target,)
        segment_start = start
        segments = []
        for segment_target in segment_targets:
            request = planning.both_arms_joint_request(segment_start, segment_target)
            response = planning.wait_future(
                node, client.call_async(request), args.timeout_s
            ).motion_plan_response
            trajectory = response.trajectory.joint_trajectory
            names = tuple(trajectory.joint_names)
            if (
                int(response.error_code.val) != MoveItErrorCodes.SUCCESS
                or not trajectory.points
                or set(names) != set(planning.BOTH_ARM_JOINTS)
            ):
                raise RuntimeError(
                    "OBSERVE_CLEAR MoveIt segment failed: "
                    f"code={response.error_code.val} names={names}"
                )
            segments.append((segment_start, response))
            final_by_name = dict(
                zip(names, trajectory.points[-1].positions, strict=True)
            )
            segment_start = tuple(
                float(final_by_name.get(name, start[index]))
                for index, name in enumerate(CANONICAL_JOINTS)
            )
        restore_request = ApplyPlanningScene.Request()
        restore_request.scene = scene_with_collision_matrix(strict_matrix)
        restore_response = planning.wait_future(
            node, scene_client.call_async(restore_request), args.timeout_s
        )
        if not restore_response.success:
            raise RuntimeError("failed to restore the strict MoveIt collision matrix")
        exceptions_enabled = False

        strict_sample_count = 0
        maximum_exception_depth = 0.0
        for segment_start, segment_response in segments:
            trajectory = segment_response.trajectory.joint_trajectory
            for positions in dense_arm_path(
                segment_start, tuple(trajectory.joint_names), trajectory.points
            ):
                validity_request = GetStateValidity.Request()
                validity_request.group_name = "both_arms"
                validity_request.robot_state = RobotState()
                validity_request.robot_state.joint_state = JointState(
                    name=list(trajectory.joint_names), position=list(positions)
                )
                validity = planning.wait_future(
                    node,
                    validity_client.call_async(validity_request),
                    args.timeout_s,
                )
                maximum_exception_depth = max(
                    maximum_exception_depth,
                    validate_contact_set(validity.contacts),
                )
                strict_sample_count += 1
    finally:
        if exceptions_enabled and strict_matrix is not None:
            try:
                emergency_restore = ApplyPlanningScene.Request()
                emergency_restore.scene = scene_with_collision_matrix(strict_matrix)
                planning.wait_future(
                    node,
                    scene_client.call_async(emergency_restore),
                    min(args.timeout_s, 5.0),
                )
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

    trajectory = segments[-1][1].trajectory.joint_trajectory
    names = tuple(trajectory.joint_names)
    target_by_name = dict(zip(CANONICAL_JOINTS, target, strict=True))
    final_by_name = dict(zip(names, trajectory.points[-1].positions, strict=True))
    residual = max(
        abs(float(final_by_name[name]) - target_by_name[name])
        for name in planning.BOTH_ARM_JOINTS
    )
    if residual > MAX_GOAL_RESIDUAL_RAD:
        raise RuntimeError(f"OBSERVE_CLEAR plan goal residual is too large: {residual}")

    document = {
        "schema_version": 1,
        "record_kind": "observe_clear_plan_only",
        "status": STATUS,
        "generated_at_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "planning_group": "both_arms",
        "planning_scene": {
            "collision_object_id": "validated_worktable_region",
            "frame_id": "left_base_link",
            "span_m": worktable["board"]["calibrated_span_m"],
            "table_z_m": worktable["board"]["table_z_in_left_base_link_m"],
            "thickness_m": TABLE_COLLISION_THICKNESS_M,
            "apply_planning_scene_success": True,
            "temporary_contact_exceptions": [
                sorted(pair) for pair in sorted(
                    OBSERVE_CLEAR_CONTACT_EXCEPTIONS,
                    key=lambda value: sorted(value),
                )
            ],
            "temporary_exceptions_restored_before_strict_validation": True,
            "strict_path_sample_step_rad": PATH_VALIDATION_STEP_RAD,
            "strict_path_sample_count": strict_sample_count,
            "strict_unapproved_contact_count": 0,
            "strict_maximum_exception_depth_m": maximum_exception_depth,
            "strict_maximum_exception_depth_limit_m": MAXIMUM_EXCEPTION_DEPTH_M,
        },
        "joint_names": list(CANONICAL_JOINTS),
        "arm_joint_names": list(names),
        "start_positions_rad": list(start),
        "target_positions_rad": list(target),
        "maximum_start_to_target_delta_rad": max(
            abs(b - a) for a, b in zip(start, target, strict=True)
        ),
        "moveit_error_code": int(segments[-1][1].error_code.val),
        "planning_time_s": sum(
            float(item[1].planning_time) for item in segments
        ),
        "planning_segment_count": len(segments),
        "planning_strategy": (
            "staged_right_clearance"
            if args.staged_right_clearance
            else "direct"
        ),
        "joint_goal_residual_rad": residual,
        "trajectory": [
            {
                "positions_rad": list(point.positions),
                "time_from_start_s": float(index),
            }
            for index, point in enumerate(
                [
                    point
                    for _, segment_response in segments
                    for point in segment_response.trajectory.joint_trajectory.points
                ],
                start=1,
            )
        ],
        "sources": {
            "contract": {"path": str(args.contract.resolve()), "sha256": sha256_file(args.contract)},
            "shadow_validation": {"path": str(args.shadow_validation.resolve()), "sha256": sha256_file(args.shadow_validation)},
            "registered_urdf_manifest": {"path": str(args.registered_urdf_manifest.resolve()), "sha256": sha256_file(args.registered_urdf_manifest)},
            "registered_urdf": {"path": str(Path(manifest["urdf"]).resolve()), "sha256": manifest["urdf_sha256"]},
        },
        "right_shadow_metrics": shadow["metrics"],
        "operator_visual_candidate_image_sha256": contract[
            "workcell_observation_candidate"
        ]["observe_clear"]["image_sha256"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"{STATUS} points={len(trajectory.points)} residual_rad={residual:.6f} "
        f"motion_commands=0 output={args.output} sha256={sha256_file(args.output)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
