#!/usr/bin/env python3
"""Plan the canonical bimanual-then-single towel sequence with zero motion commands.

This is the final R0 reachability gate.  It combines deterministic task-pose
IK with MoveIt planning, dense strict collision checks, the registered wrist
camera mount mesh, the validated worktable, and the operator-reviewed
cable-safe joint envelope.  It never connects to a controller or resident
motion service.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Iterable

import rclpy
import yaml
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneComponents, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.desk_task_runtime import CANONICAL_JOINTS  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_task_pose_planning import (  # noqa: E402
    TowelPlanningError,
    CandidateSpec,
    CorrectionProbe,
    PhaseSpec,
    TaskPose,
    build_correction_probes,
    phase_to_dict,
    point_segment_distance_m,
    solve_task_pose_branches,
    towel_bounds_from_worktable,
    validate_phase_contract,
)
from tools.lib.towel_bimanual_then_single_planning import (  # noqa: E402
    build_bimanual_then_single_candidates,
)
from tools.run.plan_observe_clear_once import (  # noqa: E402
    APPLY_SCENE_SERVICE,
    GET_SCENE_SERVICE,
    MAXIMUM_EXCEPTION_DEPTH_M,
    OBSERVE_CLEAR_CONTACT_EXCEPTIONS,
    STATE_VALIDITY_SERVICE,
    collision_matrix_with_exceptions,
    dense_arm_path,
    scene_with_collision_matrix,
    validate_contact_set,
    validated_table_scene,
)


STATUS = "TOWEL_BIMANUAL_THEN_SINGLE_TASK_POSE_PLAN_ONLY_PASS"
PATH_VALIDATION_STEP_RAD = 0.020
MAXIMUM_DENSE_TCP_PATH_DEVIATION_M = 0.004
MAXIMUM_GOAL_RESIDUAL_RAD = 0.001
PLANNING_ATTEMPTS_PER_IK_BRANCH = 8
MAXIMUM_INTENDED_TABLE_CONTACT_DEPTH_M = 0.0001
RIGHT_CLEARANCE_SHOULDER_RAD = -0.12
RIGHT_CLEARANCE_ELBOW_RAD = -0.2
DEFAULT_CONTRACT = ROOT / "config/towel_task_contract.candidate.yaml"
DEFAULT_WORKTABLE = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml"
)
DEFAULT_OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
DEFAULT_CABLE_REVIEW = ROOT / "config/bimanual_j0_desired_envelope.reviewed.json"
DEFAULT_MANIFEST = (
    ROOT
    / "artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g.manifest.json"
)
DEFAULT_SHADOW = (
    ROOT
    / "artifacts/calibration/top_eye_to_hand_20260825_r0c/"
    "right_workcell_shadow_validation.yaml"
)
DEFAULT_RIGHT_TABLETOP = (
    ROOT
    / "artifacts/calibration/right_tabletop_target_staged_20260826_r0/candidate.yaml"
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_yaml(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return value


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def validate_inputs(args: argparse.Namespace) -> dict[str, object]:
    for path in (
        args.contract,
        args.worktable,
        args.operational_limits,
        args.cable_review,
        args.registered_urdf_manifest,
        args.right_registration_shadow,
        args.right_tabletop_validation,
    ):
        if not path.is_file():
            raise RuntimeError(f"required source does not exist: {path}")
    contract = load_yaml(args.contract)
    observe = contract.get("workcell_observation_candidate", {}).get(
        "observe_clear", {}
    )
    if (
        contract.get("motion_authorized") is not False
        or observe.get("status") != "SUPERVISED_ROUNDTRIP_VALIDATED"
        or observe.get("motion_reproducibility_validated") is not True
        or observe.get("all_four_towel_corners_visible") is not True
        or tuple(observe.get("joint_names", ())) != CANONICAL_JOINTS
        or len(observe.get("joint_positions_rad", ())) != 12
    ):
        raise RuntimeError("towel contract does not contain the validated clear pose")
    worktable = load_yaml(args.worktable)
    if (
        worktable.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or worktable.get("base_registration", {}).get("transform_validated") is not True
    ):
        raise RuntimeError("worktable source is not independently validated")
    manifest = load_json(args.registered_urdf_manifest)
    registration = manifest.get("right_registration_candidate", {})
    urdf_path = Path(str(manifest.get("urdf", ""))).resolve()
    if (
        manifest.get("simulation_only") is not True
        or manifest.get("motion_authorized") is not False
        or registration.get("runtime_promotion_authorized") is not False
        or manifest.get("wrist_camera_mount_geometry", {}).get("left") is not True
        or manifest.get("wrist_camera_mount_geometry", {}).get("right") is not True
        or not urdf_path.is_file()
        or sha256_file(urdf_path) != manifest.get("urdf_sha256")
    ):
        raise RuntimeError("registered preview manifest is invalid or stale")
    configured = os.environ.get(planning.DUAL_URDF_ENVIRONMENT, "")
    if not configured or Path(configured).resolve() != urdf_path:
        raise RuntimeError(
            f"{planning.DUAL_URDF_ENVIRONMENT} must select the registered preview URDF"
        )
    shadow = load_yaml(args.right_registration_shadow)
    if (
        shadow.get("status") != "RIGHT_REGISTRATION_WORKCELL_SHADOW_VALIDATED"
        or shadow.get("motion_authorized") is not False
        or shadow.get("robot_target_available") is not False
        or shadow.get("sources", {}).get("candidate", {}).get("sha256")
        != registration.get("sha256")
    ):
        raise RuntimeError("right workcell shadow does not match the URDF registration")
    tabletop = load_yaml(args.right_tabletop_validation)
    validation_metrics = tabletop.get("metrics", {}).get("validation", {})
    if (
        tabletop.get("status")
        != "RIGHT_TABLETOP_TARGETS_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or tabletop.get("motion_authorized") is not False
        or int(validation_metrics.get("capture_count", 0)) < 2
        or float(validation_metrics.get("xy_max_m", math.inf)) > 0.015
    ):
        raise RuntimeError("right tabletop target validation is missing or too inaccurate")
    limits = load_json(args.operational_limits)
    if (
        limits.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or limits.get("operator_approved") is not True
        or limits.get("firmware_limit_authorized") is not True
        or manifest.get("joint_limits", {}).get("approved_sha256")
        != sha256_file(args.operational_limits)
    ):
        raise RuntimeError("operational joint envelope is not approved")
    cable = load_json(args.cable_review)
    confirmation = cable.get("operator_confirmation", {})
    if (
        cable.get("status") != "J0_D_REVIEWED_PASS_J0_M_NOT_MEASURED"
        or cable.get("motion_authorized") is not False
        or confirmation.get("all_sweeps_cable_safe") is not True
        or confirmation.get("cable_or_connector_issue_observed") is not False
    ):
        raise RuntimeError("operator-reviewed cable-safe envelope is not available")
    return {
        "contract": contract,
        "worktable": worktable,
        "manifest": manifest,
        "shadow": shadow,
        "tabletop": tabletop,
        "limits": limits,
        "cable": cable,
        "urdf_path": urdf_path,
    }


def gripper_modes(contract: dict, cable_review: dict) -> dict[str, tuple[float, float]]:
    contact = contract["cloth_contact_candidate"]
    clear_positions = contract["workcell_observation_candidate"]["observe_clear"][
        "joint_positions_rad"
    ]
    result = {
        "task_open": (
            float(clear_positions[5]),
            float(clear_positions[11]),
        ),
        "one_layer_contact": (
            float(contact["left"]["one_layer"]["operational_candidate_rad"]),
            float(contact["right"]["one_layer"]["operational_candidate_rad"]),
        ),
        "four_layer_contact": (
            float(contact["left"]["four_layer"]["operational_candidate_rad"]),
            float(contact["right"]["four_layer"]["operational_candidate_rad"]),
        ),
    }
    if not all(math.isfinite(value) for pair in result.values() for value in pair):
        raise RuntimeError("gripper collision modes contain a non-finite value")
    if not all(0.0 <= value <= 1.91986 for pair in result.values() for value in pair):
        raise RuntimeError("gripper collision modes exceed the registered URDF bounds")
    return result


def arm_values(full: Iterable[float], side: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in full)
    indices = (0, 1, 2, 3, 4) if side == "left" else (6, 7, 8, 9, 10)
    return tuple(values[index] for index in indices)


def replace_arm(full: Iterable[float], side: str, arm: Iterable[float]) -> tuple[float, ...]:
    result = list(float(value) for value in full)
    indices = (0, 1, 2, 3, 4) if side == "left" else (6, 7, 8, 9, 10)
    values = tuple(float(value) for value in arm)
    if len(values) != 5:
        raise RuntimeError("arm replacement requires five joints")
    for index, value in zip(indices, values, strict=True):
        result[index] = value
    return tuple(result)


def with_grippers(full: Iterable[float], pair: tuple[float, float]) -> tuple[float, ...]:
    result = list(float(value) for value in full)
    result[5], result[11] = pair
    return tuple(result)


def phase_gripper_modes(
    phase: PhaseSpec,
    configured: dict[str, tuple[float, float]],
) -> dict[str, tuple[float, float]]:
    """Return only gripper widths physically present during one phase.

    Applying every measured width to every arm pose creates impossible states
    (for example a closed jaw during an open pregrasp).  Contact and release
    phases include both sides of the state change; a two-layer bundle is
    checked at both measured one- and four-layer bounds.
    """
    open_pair = configured["task_open"]
    if phase.clear_pose:
        return {"task_open": open_pair}
    semantics = {target.semantic for target in phase.targets}
    if not semantics:
        raise RuntimeError(f"{phase.name}: non-clear phase has no task targets")
    if all(
        semantic in {"pregrasp_open", "released_retreat"}
        for semantic in semantics
    ):
        return {"task_open": open_pair}

    active_arms = {target.arm for target in phase.targets}

    def mixed(mode: str) -> tuple[float, float]:
        values = list(open_pair)
        source = configured[mode]
        if "left" in active_arms:
            values[0] = source[0]
        if "right" in active_arms:
            values[1] = source[1]
        return float(values[0]), float(values[1])

    layers = {target.layer for target in phase.targets}
    modes: dict[str, tuple[float, float]] = {}
    if "two_layer_bundle" in layers:
        modes["two_layer_one_layer_bound"] = mixed("one_layer_contact")
        modes["two_layer_four_layer_bound"] = mixed("four_layer_contact")
    else:
        modes["one_layer_contact"] = mixed("one_layer_contact")
    if "contact" in semantics or any(
        semantic in {"attached_laydown"} for semantic in semantics
    ):
        modes["task_open"] = open_pair
    return modes


def needs_validated_right_departure(
    phase: PhaseSpec,
    current: tuple[float, ...],
    clear: tuple[float, ...],
) -> bool:
    """Identify a right-arm task departure from the shallow-contact clear pose."""
    if current != clear or len(phase.targets) != 1:
        return False
    first_target = phase.targets[0]
    return (
        first_target.arm == "right"
        and "departure" in first_target.name
        and first_target.semantic == "pregrasp_open"
    )


def wait_future(node: Node, future, timeout_s: float):
    return planning.wait_future(node, future, timeout_s)


class MoveItPlanOnlyGate:
    def __init__(
        self,
        node: Node,
        timeout_s: float,
        grippers: dict[str, tuple[float, float]],
        collision_reference: tuple[float, ...],
        kinematics: dict[str, GraspYawKinematics],
    ):
        self.node = node
        self.timeout_s = timeout_s
        self.grippers = grippers
        self.kinematics = kinematics
        self.collision_reference = tuple(float(value) for value in collision_reference)
        if len(self.collision_reference) != len(CANONICAL_JOINTS):
            raise RuntimeError("collision reference must be the canonical 12-axis state")
        self.plan_client = node.create_client(planning.GetMotionPlan, planning.PLAN_SERVICE)
        self.scene_client = node.create_client(ApplyPlanningScene, APPLY_SCENE_SERVICE)
        self.get_scene_client = node.create_client(GetPlanningScene, GET_SCENE_SERVICE)
        self.validity_client = node.create_client(GetStateValidity, STATE_VALIDITY_SERVICE)
        self.strict_matrix = None
        self.exceptions_enabled = False
        self.validated_path_cache: dict[
            str,
            tuple[
                tuple[float, ...],
                list[dict[str, object]],
                dict[str, object],
                list[dict[str, object]],
            ],
        ] = {}

    def wait(self) -> None:
        for name, client in (
            (planning.PLAN_SERVICE, self.plan_client),
            (APPLY_SCENE_SERVICE, self.scene_client),
            (GET_SCENE_SERVICE, self.get_scene_client),
            (STATE_VALIDITY_SERVICE, self.validity_client),
        ):
            if not client.wait_for_service(timeout_sec=self.timeout_s):
                raise RuntimeError(f"MoveIt service unavailable: {name}")

    def apply_table_and_read_matrix(self, worktable: dict) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = validated_table_scene(worktable)
        if not wait_future(
            self.node, self.scene_client.call_async(request), self.timeout_s
        ).success:
            raise RuntimeError("MoveIt rejected the validated table scene")
        get_request = GetPlanningScene.Request()
        get_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.strict_matrix = wait_future(
            self.node, self.get_scene_client.call_async(get_request), self.timeout_s
        ).scene.allowed_collision_matrix

    def set_exceptions(self, enabled: bool) -> None:
        if self.strict_matrix is None:
            raise RuntimeError("strict collision matrix has not been loaded")
        matrix = (
            collision_matrix_with_exceptions(self.strict_matrix, True)
            if enabled else self.strict_matrix
        )
        request = ApplyPlanningScene.Request()
        request.scene = scene_with_collision_matrix(matrix)
        response = wait_future(
            self.node, self.scene_client.call_async(request), self.timeout_s
        )
        if not response.success:
            raise RuntimeError("MoveIt rejected a collision-matrix update")
        self.exceptions_enabled = enabled

    def restore(self) -> None:
        if self.exceptions_enabled:
            self.set_exceptions(False)

    def endpoint_valid(self, positions: tuple[float, ...]) -> tuple[bool, str]:
        request = GetStateValidity.Request()
        request.group_name = "both_arms"
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(
            name=list(CANONICAL_JOINTS),
            position=list(with_grippers(positions, self.grippers["task_open"])),
        )
        response = wait_future(
            self.node, self.validity_client.call_async(request), self.timeout_s
        )
        if response.valid:
            return True, ""
        if response.contacts:
            try:
                validate_contact_set(response.contacts)
            except RuntimeError:
                pass
            else:
                return True, ""
        contacts = ", ".join(
            f"{item.contact_body_1}--{item.contact_body_2}:{item.depth:.6f}"
            for item in response.contacts
        )
        return False, contacts or "invalid state without contacts"

    def canonical_approved_contact_depth(
        self,
        by_name: dict[str, float],
        contact_pair: frozenset[str],
        collision_check_group: str,
    ) -> float:
        """Requery one active-arm mesh pair with a canonical inactive arm.

        FCL may select a different representative triangle intersection for
        the same active-arm geometry when unrelated inactive-arm transforms
        change.  This query leaves every active-arm joint and gripper exactly
        unchanged and only restores the inactive arm to OBSERVE_CLEAR.  The
        original full-state query still owns all unapproved and inter-arm
        contacts.
        """
        if collision_check_group not in {"left_arm", "right_arm"}:
            return math.inf
        active_prefix = "left_" if collision_check_group == "left_arm" else "right_"
        if not all(name.startswith(active_prefix) for name in contact_pair):
            return math.inf
        reference = dict(
            zip(CANONICAL_JOINTS, self.collision_reference, strict=True)
        )
        canonical = dict(by_name)
        inactive_prefix = "right_" if active_prefix == "left_" else "left_"
        for joint in CANONICAL_JOINTS:
            if joint.startswith(inactive_prefix):
                canonical[joint] = reference[joint]
        request = GetStateValidity.Request()
        request.group_name = collision_check_group
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(
            name=list(CANONICAL_JOINTS),
            position=[float(canonical[name]) for name in CANONICAL_JOINTS],
        )
        response = wait_future(
            self.node,
            self.validity_client.call_async(request),
            self.timeout_s,
        )
        depths = [
            float(contact.depth)
            for contact in response.contacts
            if frozenset((contact.contact_body_1, contact.contact_body_2))
            == contact_pair
        ]
        if not depths:
            return 0.0
        if not all(math.isfinite(depth) for depth in depths):
            return math.inf
        return max(depths)

    def plan_segment(
        self, name: str, start: tuple[float, ...], target: tuple[float, ...]
    ) -> dict[str, object]:
        start_with_grippers = with_grippers(start, self.grippers["task_open"])
        target_with_grippers = with_grippers(target, self.grippers["task_open"])
        changed_arms = [
            side
            for side in ("left", "right")
            if any(
                abs(a - b) > 1.0e-9
                for a, b in zip(
                    arm_values(start, side),
                    arm_values(target, side),
                    strict=True,
                )
            )
        ]
        if not changed_arms:
            raise RuntimeError(f"{name}: planning segment has no arm motion")
        request = planning.both_arms_joint_request(
            start_with_grippers,
            target_with_grippers,
        )
        if len(changed_arms) == 1:
            side = changed_arms[0]
            group_name, group_joints, _ = planning.arm_contract(side)
            group_arm_joints = set(group_joints[:5])
            request.motion_plan_request.group_name = group_name
            constraints = request.motion_plan_request.goal_constraints[0]
            constraints.joint_constraints = [
                item
                for item in constraints.joint_constraints
                if item.joint_name in group_arm_joints
            ]
            expected_joints = group_arm_joints
        else:
            group_name = "both_arms"
            expected_joints = set(planning.BOTH_ARM_JOINTS)
        response = wait_future(
            self.node, self.plan_client.call_async(request), self.timeout_s
        ).motion_plan_response
        trajectory = response.trajectory.joint_trajectory
        names = tuple(trajectory.joint_names)
        if (
            int(response.error_code.val) != MoveItErrorCodes.SUCCESS
            or not trajectory.points
            or set(names) != expected_joints
        ):
            raise RuntimeError(
                f"{name}: MoveIt failed group={group_name} "
                f"code={response.error_code.val} names={names}"
            )
        final = dict(zip(names, trajectory.points[-1].positions, strict=True))
        target_by_name = dict(zip(CANONICAL_JOINTS, target, strict=True))
        residual = max(
            abs(float(final[joint]) - float(target_by_name[joint]))
            for joint in names
        )
        if residual > MAXIMUM_GOAL_RESIDUAL_RAD:
            raise RuntimeError(f"{name}: joint goal residual {residual:.6f} rad")
        return {
            "name": name,
            "start_positions_rad": list(start),
            "target_positions_rad": list(target),
            "planning_group": group_name,
            "moveit_error_code": int(response.error_code.val),
            "planning_time_s": float(response.planning_time),
            "trajectory_joint_names": list(names),
            "trajectory_positions_rad": [
                [float(value) for value in point.positions]
                for point in trajectory.points
            ],
            "trajectory_point_count": len(trajectory.points),
            "joint_goal_residual_rad": residual,
            "_trajectory": trajectory,
        }

    def deterministic_arm_segment(
        self,
        name: str,
        start: tuple[float, ...],
        target: tuple[float, ...],
        side: str,
    ) -> dict[str, object]:
        """Build an exact joint chord for an independently validated route."""
        group_name, group_joints, _ = planning.arm_contract(side)
        names = tuple(group_joints[:5])
        indices = tuple(CANONICAL_JOINTS.index(joint) for joint in names)
        start_arm = tuple(float(start[index]) for index in indices)
        target_arm = tuple(float(target[index]) for index in indices)
        if not any(abs(a - b) > 1.0e-12 for a, b in zip(start_arm, target_arm)):
            raise RuntimeError(f"{name}: deterministic segment has no arm motion")
        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)
        trajectory.points = [
            JointTrajectoryPoint(positions=list(start_arm)),
            JointTrajectoryPoint(positions=list(target_arm)),
        ]
        return {
            "name": name,
            "start_positions_rad": list(start),
            "target_positions_rad": list(target),
            "planning_group": group_name,
            "moveit_error_code": int(MoveItErrorCodes.SUCCESS),
            "planning_time_s": 0.0,
            "trajectory_joint_names": list(names),
            "trajectory_positions_rad": [list(start_arm), list(target_arm)],
            "trajectory_point_count": 2,
            "joint_goal_residual_rad": 0.0,
            "planning_method": "validated_single_joint_linear_interpolation",
            "_trajectory": trajectory,
        }

    def reverse_segment(
        self,
        name: str,
        start: tuple[float, ...],
        target: tuple[float, ...],
        source: dict[str, object],
    ) -> dict[str, object]:
        """Reverse an already planned single-arm path without replanning it.

        A fresh OMPL request is not guaranteed to retrace a collision-free
        approach.  Reusing the exact accepted path is deterministic; the
        result is still densely checked with the other arm at its current
        pose before it can pass this plan-only gate.
        """
        trajectory = copy.deepcopy(source["_trajectory"])
        names = tuple(trajectory.joint_names)
        if not names or not trajectory.points:
            raise RuntimeError(f"{name}: reverse source has no trajectory")
        source_start = tuple(float(value) for value in source["start_positions_rad"])
        source_target = tuple(float(value) for value in source["target_positions_rad"])
        source_by_name = dict(zip(CANONICAL_JOINTS, source_target, strict=True))
        if max(
            abs(float(start[CANONICAL_JOINTS.index(joint)]) - source_by_name[joint])
            for joint in names
        ) > MAXIMUM_GOAL_RESIDUAL_RAD:
            raise RuntimeError(f"{name}: current arm does not match reverse source goal")

        original_points = list(trajectory.points)
        total_ns = (
            int(original_points[-1].time_from_start.sec) * 1_000_000_000
            + int(original_points[-1].time_from_start.nanosec)
        )
        reversed_points = []
        for original in reversed(original_points):
            point = copy.deepcopy(original)
            original_ns = (
                int(original.time_from_start.sec) * 1_000_000_000
                + int(original.time_from_start.nanosec)
            )
            reversed_ns = max(0, total_ns - original_ns)
            point.time_from_start.sec = reversed_ns // 1_000_000_000
            point.time_from_start.nanosec = reversed_ns % 1_000_000_000
            if point.velocities:
                point.velocities = [-float(value) for value in point.velocities]
            reversed_points.append(point)
        trajectory.points = reversed_points

        target_by_name = dict(zip(CANONICAL_JOINTS, target, strict=True))
        final = dict(zip(names, trajectory.points[-1].positions, strict=True))
        residual = max(
            abs(float(final[joint]) - float(target_by_name[joint]))
            for joint in names
        )
        if residual > MAXIMUM_GOAL_RESIDUAL_RAD:
            source_start_by_name = dict(
                zip(CANONICAL_JOINTS, source_start, strict=True)
            )
            source_residual = max(
                abs(float(final[joint]) - float(source_start_by_name[joint]))
                for joint in names
            )
            raise RuntimeError(
                f"{name}: reversed goal residual {residual:.6f} rad "
                f"source_residual={source_residual:.6f}"
            )
        return {
            "name": name,
            "start_positions_rad": list(start),
            "target_positions_rad": list(target),
            "planning_group": str(source["planning_group"]),
            "moveit_error_code": int(MoveItErrorCodes.SUCCESS),
            "planning_time_s": 0.0,
            "trajectory_joint_names": list(names),
            "trajectory_positions_rad": [
                [float(value) for value in point.positions]
                for point in trajectory.points
            ],
            "trajectory_point_count": len(trajectory.points),
            "joint_goal_residual_rad": residual,
            "reversed_validated_segment": str(source["name"]),
            "_trajectory": trajectory,
        }

    def validated_right_departure(
        self,
        phase_name: str,
        start: tuple[float, ...],
        target: tuple[float, ...],
        task_targets: tuple[TaskPose, ...],
    ) -> tuple[list[dict[str, object]], dict[str, object]]:
        """Leave OBSERVE_CLEAR through a bounded seven-stage joint route.

        OBSERVE_CLEAR contains only the measured shallow same-arm mesh contacts.
        Planning directly with those pairs allowed lets OMPL choose paths that
        deepen the right jaw/shoulder overlap.  The registered-r0g strict sweep
        selected shoulder=-0.12 before elbow=-0.2; each deterministic joint
        chord is independently audited against the unchanged 4 mm bound.
        """
        current = start
        route = (
            ("shoulder_clearance", 7, RIGHT_CLEARANCE_SHOULDER_RAD),
            ("elbow_clearance", 8, RIGHT_CLEARANCE_ELBOW_RAD),
            ("wrist_roll", 10, target[10]),
            ("wrist_flex", 9, target[9]),
            ("target_elbow", 8, target[8]),
            ("target_shoulder", 7, target[7]),
            ("target_base", 6, target[6]),
        )
        segments: list[dict[str, object]] = []
        self.set_exceptions(False)
        for route_name, index, value in route:
            if abs(current[index] - value) <= 1.0e-12:
                continue
            name = f"{phase_name}_route_{route_name}"
            target_positions = list(current)
            target_positions[index] = float(value)
            target_state = tuple(target_positions)
            segment = self.deterministic_arm_segment(
                name, current, target_state, "right"
            )
            is_final = route_name == "target_base"
            segment["strict_validation"] = self.dense_validate(
                segment,
                {"task_open": self.grippers["task_open"]},
                False,
                task_targets if is_final else (),
            )
            segment["planning_attempt"] = 1
            segment["validated_clear_departure_route"] = True
            segments.append(segment)
            current = target_state
        if current != target:
            raise RuntimeError(f"{phase_name}: staged departure missed its target")
        if not segments:
            raise RuntimeError(f"{phase_name}: staged departure has no segments")
        final_segment = segments[-1]
        final_segment["name"] = phase_name
        prefix_records = [
            {
                "name": str(segment["name"]),
                "targets": [],
                "validated_clearance_route": True,
                "moveit": segment,
            }
            for segment in segments[:-1]
        ]
        return prefix_records, final_segment

    def staged_bimanual_reobserve_clear(
        self,
        phase_name: str,
        start: tuple[float, ...],
        clear: tuple[float, ...],
    ) -> tuple[list[dict[str, object]], dict[str, object]]:
        """Return from the first retreat through strict r0g route stages.

        The registered-r0g state grid and every connecting single-joint chord
        were checked through MoveIt's strict state-validity service.  Restoring
        shoulder and elbow before rotating the base keeps the right arm below
        and away from the overhead-camera mount.  Each actual candidate route
        is audited again below, so a different IK branch cannot inherit the
        diagnostic result without passing the same dense checks.
        """
        current = start
        prefix_records: list[dict[str, object]] = []
        route = (
            ("target_shoulder", 7, clear[7]),
            ("target_elbow", 8, clear[8]),
            ("target_base", 6, clear[6]),
            ("target_wrist_roll", 10, clear[10]),
            ("target_wrist_flex", 9, clear[9]),
        )
        for route_name, index, value in route:
            if abs(current[index] - value) <= 1.0e-12:
                continue
            name = f"{phase_name}_route_{route_name}"
            target_state = list(current)
            target_state[index] = float(value)
            target_state_tuple = tuple(target_state)
            previous_exceptions = self.exceptions_enabled
            try:
                self.set_exceptions(False)
                segment = self.deterministic_arm_segment(
                    name, current, target_state_tuple, "right"
                )
                segment["strict_validation"] = self.dense_validate(
                    segment,
                    {"task_open": self.grippers["task_open"]},
                    False,
                    (),
                )
            finally:
                if self.exceptions_enabled != previous_exceptions:
                    self.set_exceptions(previous_exceptions)
            segment["planning_attempt"] = 1
            segment["staged_reobserve_clear_route"] = True
            prefix_records.append(
                {
                    "name": name,
                    "targets": [],
                    "staged_reobserve_clear_route": True,
                    "moveit": segment,
                }
            )
            current = target_state_tuple

        if current == clear:
            if not prefix_records:
                raise RuntimeError(f"{phase_name}: staged clear return has no motion")
            final_record = prefix_records.pop()
            final_segment = final_record["moveit"]
            final_segment["name"] = phase_name
            final_segment["completed_clear_without_final_ompl_segment"] = True
            return prefix_records, final_segment

        failures = []
        for attempt in range(1, PLANNING_ATTEMPTS_PER_IK_BRANCH + 1):
            previous_exceptions = self.exceptions_enabled
            try:
                self.set_exceptions(True)
                final_segment = self.plan_segment(phase_name, current, clear)
                self.set_exceptions(False)
                final_segment["strict_validation"] = self.dense_validate(
                    final_segment,
                    {"task_open": self.grippers["task_open"]},
                    False,
                    (),
                )
            except RuntimeError as exc:
                failures.append(f"attempt={attempt}: {exc}")
                continue
            finally:
                if self.exceptions_enabled != previous_exceptions:
                    self.set_exceptions(previous_exceptions)
            final_segment["planning_attempt"] = attempt
            final_segment["staged_reobserve_clear_route"] = True
            return prefix_records, final_segment
        reason = failures[0] if failures else "no final clear segment"
        raise RuntimeError(f"{phase_name}: staged clear return failed; {reason}")

    def dense_validate(
        self,
        segment: dict[str, object],
        applicable_grippers: dict[str, tuple[float, float]],
        intended_table_contact: bool,
        task_targets: tuple[TaskPose, ...],
    ) -> dict[str, object]:
        trajectory = segment["_trajectory"]
        names = tuple(trajectory.joint_names)
        collision_check_group = str(segment["planning_group"])
        if collision_check_group not in {"left_arm", "right_arm", "both_arms"}:
            raise RuntimeError(
                f"{segment['name']}: invalid collision group "
                f"{collision_check_group}"
            )
        start = tuple(float(value) for value in segment["start_positions_rad"])
        samples = dense_arm_path(
            start, names, trajectory.points, step_rad=PATH_VALIDATION_STEP_RAD
        )
        sample_count = 0
        accepted_exception_contacts = 0
        maximum_depth = 0.0
        normalized_approved_contacts = 0
        maximum_raw_nondeterministic_depth = 0.0
        intended_table_contacts = 0
        maximum_table_contact_depth = 0.0
        base_by_name = dict(zip(CANONICAL_JOINTS, start, strict=True))
        target_by_arm = {target.arm: target for target in task_targets}
        start_tcp_by_arm = {
            arm: self.kinematics[arm].tcp_pose_in_root(base_by_name)[1]
            for arm in target_by_arm
        }
        maximum_tcp_path_deviation_by_arm = {
            arm: 0.0 for arm in target_by_arm
        }
        for arm_sample in samples:
            by_name = dict(base_by_name)
            by_name.update(zip(names, arm_sample, strict=True))
            for arm, target in target_by_arm.items():
                actual_tcp = self.kinematics[arm].tcp_pose_in_root(by_name)[1]
                deviation = point_segment_distance_m(
                    actual_tcp, start_tcp_by_arm[arm], target.xyz_m
                )
                maximum_tcp_path_deviation_by_arm[arm] = max(
                    maximum_tcp_path_deviation_by_arm[arm], deviation
                )
                if deviation > MAXIMUM_DENSE_TCP_PATH_DEVIATION_M:
                    raise RuntimeError(
                        f"{segment['name']}: {arm} TCP path deviation "
                        f"{deviation:.6f} m exceeds "
                        f"{MAXIMUM_DENSE_TCP_PATH_DEVIATION_M:.6f} m"
                    )
            for mode_name, pair in applicable_grippers.items():
                by_name["left_gripper_joint"] = pair[0]
                by_name["right_gripper_joint"] = pair[1]
                request = GetStateValidity.Request()
                # Match collision checking to the moving planning group.  The
                # inactive arm remains in the complete RobotState and is still
                # an obstacle.
                request.group_name = collision_check_group
                request.robot_state = RobotState()
                request.robot_state.joint_state = JointState(
                    name=list(CANONICAL_JOINTS),
                    position=[float(by_name[name]) for name in CANONICAL_JOINTS],
                )
                response = wait_future(
                    self.node,
                    self.validity_client.call_async(request),
                    self.timeout_s,
                )
                if not response.valid and not response.contacts:
                    raise RuntimeError(
                        f"{segment['name']}: invalid {mode_name} state without contacts"
                    )
                if response.contacts:
                    remaining_contacts = []
                    for contact in response.contacts:
                        pair_names = {
                            contact.contact_body_1,
                            contact.contact_body_2,
                        }
                        table_pair = (
                            "validated_worktable_region" in pair_names
                            and any(
                                name.endswith("_moving_jaw_link")
                                for name in pair_names
                            )
                        )
                        if intended_table_contact and table_pair:
                            depth = float(contact.depth)
                            if (
                                not math.isfinite(depth)
                                or depth > MAXIMUM_INTENDED_TABLE_CONTACT_DEPTH_M
                            ):
                                raise RuntimeError(
                                    f"{segment['name']}: intended jaw-table contact "
                                    f"depth={depth} exceeds "
                                    f"{MAXIMUM_INTENDED_TABLE_CONTACT_DEPTH_M} m"
                                )
                            intended_table_contacts += 1
                            maximum_table_contact_depth = max(
                                maximum_table_contact_depth, depth
                            )
                        else:
                            normalized = contact
                            contact_pair = frozenset(
                                (contact.contact_body_1, contact.contact_body_2)
                            )
                            raw_depth = float(contact.depth)
                            if (
                                contact_pair in OBSERVE_CLEAR_CONTACT_EXCEPTIONS
                                and math.isfinite(raw_depth)
                                and raw_depth > MAXIMUM_EXCEPTION_DEPTH_M
                            ):
                                canonical_depth = (
                                    self.canonical_approved_contact_depth(
                                        by_name,
                                        contact_pair,
                                        collision_check_group,
                                    )
                                )
                                if canonical_depth <= MAXIMUM_EXCEPTION_DEPTH_M:
                                    normalized = copy.deepcopy(contact)
                                    normalized.depth = canonical_depth
                                    normalized_approved_contacts += 1
                                    maximum_raw_nondeterministic_depth = max(
                                        maximum_raw_nondeterministic_depth,
                                        raw_depth,
                                    )
                            remaining_contacts.append(normalized)
                    try:
                        contact_depth = validate_contact_set(remaining_contacts)
                    except RuntimeError as exc:
                        raise RuntimeError(
                            f"{segment['name']}: strict {mode_name} collision: {exc}"
                        ) from exc
                    maximum_depth = max(maximum_depth, contact_depth)
                    accepted_exception_contacts += len(response.contacts)
                sample_count += 1
        return {
            "strict_state_samples": sample_count,
            "strict_arm_sample_count": len(samples),
            "collision_check_group": collision_check_group,
            "gripper_collision_modes": sorted(applicable_grippers),
            "unapproved_contact_count": 0,
            "accepted_shallow_mesh_contact_count": accepted_exception_contacts,
            "maximum_accepted_mesh_contact_depth_m": maximum_depth,
            "maximum_accepted_mesh_contact_depth_limit_m": MAXIMUM_EXCEPTION_DEPTH_M,
            "canonicalized_active_arm_contact_count": normalized_approved_contacts,
            "maximum_raw_nondeterministic_contact_depth_m": (
                maximum_raw_nondeterministic_depth
            ),
            "intended_jaw_table_contact_count": intended_table_contacts,
            "maximum_intended_jaw_table_contact_depth_m": maximum_table_contact_depth,
            "maximum_intended_jaw_table_contact_depth_limit_m": (
                MAXIMUM_INTENDED_TABLE_CONTACT_DEPTH_M
            ),
            "dense_tcp_path_audit_target": "nearest_point_on_adjacent_task_chord",
            "maximum_dense_tcp_path_deviation_m_by_arm": (
                maximum_tcp_path_deviation_by_arm
            ),
            "maximum_dense_tcp_path_deviation_limit_m": (
                MAXIMUM_DENSE_TCP_PATH_DEVIATION_M
            ),
        }


def solve_and_plan_phases(
    gate: MoveItPlanOnlyGate,
    phases: tuple[PhaseSpec, ...],
    start: tuple[float, ...],
    clear: tuple[float, ...],
    kinematics: dict[str, GraspYawKinematics],
    bounds: dict[str, tuple[object, object]],
) -> tuple[tuple[float, ...], list[dict[str, object]]]:
    current = start
    records = []
    for phase in phases:
        phase_record: dict[str, object] = phase_to_dict(phase)
        if (
            phase.path_cache_key is not None
            and phase.path_cache_key in gate.validated_path_cache
        ):
            (
                cached_target,
                cached_evaluations,
                cached_segment,
                cached_prefix_records,
            ) = (
                gate.validated_path_cache[phase.path_cache_key]
            )
            cached_start_segment = (
                cached_prefix_records[0]["moveit"]
                if cached_prefix_records
                else cached_segment
            )
            cached_start = tuple(
                float(value)
                for value in cached_start_segment["start_positions_rad"]
            )
            if current != cached_start:
                raise RuntimeError(
                    f"{phase.name}: cached path start state changed"
                )
            segment = copy.deepcopy(cached_segment)
            prefix_records = copy.deepcopy(cached_prefix_records)
            renamed_prefixes = []
            for prefix_index, prefix_record in enumerate(prefix_records, start=1):
                old_name = str(prefix_record["name"])
                new_name = f"{phase.name}_cached_route_{prefix_index:02d}"
                prefix_record["name"] = new_name
                prefix_record["cached_path_reused_from"] = old_name
                prefix_record["moveit"]["name"] = new_name
                prefix_record["moveit"][
                    "strict_validation_reused_from"
                ] = old_name
                renamed_prefixes.append(new_name)
            segment["name"] = phase.name
            segment["strict_validation_reused_from"] = str(
                cached_segment["name"]
            )
            if renamed_prefixes:
                segment["phase_prefix_record_names"] = renamed_prefixes
            phase_record["task_pose_evaluations"] = copy.deepcopy(
                cached_evaluations
            )
            phase_record["moveit"] = segment
            records.extend(prefix_records)
            records.append(phase_record)
            current = cached_target
            continue
        if phase.reverse_of is not None:
            source_record = next(
                (item for item in records if item["name"] == phase.reverse_of),
                None,
            )
            if source_record is None:
                raise RuntimeError(
                    f"{phase.name}: reverse source not found: {phase.reverse_of}"
                )
            source_segment = source_record["moveit"]
            prefix_names = tuple(
                str(value)
                for value in source_segment.get(
                    "phase_prefix_record_names", ()
                )
            )
            source_components = []
            for prefix_name in prefix_names:
                prefix_record = next(
                    (item for item in records if item["name"] == prefix_name),
                    None,
                )
                if prefix_record is None:
                    raise RuntimeError(
                        f"{phase.name}: reverse prefix not found: {prefix_name}"
                    )
                source_components.append(prefix_record["moveit"])
            source_components.append(source_segment)
            if phase.clear_arm is not None:
                active_arm = phase.clear_arm
            else:
                active_arms = {target.arm for target in phase.targets}
                if len(active_arms) != 1:
                    raise RuntimeError(
                        f"{phase.name}: reverse phase requires one active arm"
                    )
                active_arm = next(iter(active_arms))
            target_positions = current
            reverse_segments = []
            for component_index, source_component in enumerate(
                reversed(source_components), start=1
            ):
                source_start = tuple(
                    float(value)
                    for value in source_component["start_positions_rad"]
                )
                component_target = replace_arm(
                    target_positions,
                    active_arm,
                    arm_values(source_start, active_arm),
                )
                final_component = component_index == len(source_components)
                component_name = (
                    phase.name
                    if final_component
                    else f"{phase.name}_route_{component_index:02d}"
                )
                reverse_segment = gate.reverse_segment(
                    component_name,
                    target_positions,
                    component_target,
                    source_component,
                )
                moving_names = set(reverse_segment["trajectory_joint_names"])
                source_target = tuple(
                    float(value)
                    for value in source_component["target_positions_rad"]
                )
                inactive_state_unchanged = all(
                    abs(target_positions[index] - source_target[index]) <= 1.0e-12
                    for index, joint in enumerate(CANONICAL_JOINTS)
                    if joint not in moving_names
                )
                if inactive_state_unchanged:
                    reverse_segment["strict_validation"] = copy.deepcopy(
                        source_component["strict_validation"]
                    )
                    reverse_segment["strict_validation"][
                        "validation_reused_for_exact_reverse"
                    ] = True
                else:
                    previous_exceptions = gate.exceptions_enabled
                    gate.set_exceptions(False)
                    try:
                        reverse_segment["strict_validation"] = gate.dense_validate(
                            reverse_segment,
                            phase_gripper_modes(phase, gate.grippers),
                            False,
                            phase.targets,
                        )
                    finally:
                        if gate.exceptions_enabled != previous_exceptions:
                            gate.set_exceptions(previous_exceptions)
                reverse_segments.append(reverse_segment)
                target_positions = component_target
            if phase.clear_pose:
                expected_clear = replace_arm(
                    current,
                    active_arm,
                    arm_values(clear, active_arm),
                )
                if target_positions != expected_clear:
                    raise RuntimeError(
                        f"{phase.name}: reverse source does not return to "
                        "OBSERVE_CLEAR"
                    )
            for component in reverse_segments[:-1]:
                records.append(
                    {
                        "name": str(component["name"]),
                        "targets": [],
                        "exact_composite_reverse_route": True,
                        "moveit": component,
                    }
                )
            segment = reverse_segments[-1]
            segment["exact_composite_reverse_component_count"] = len(
                reverse_segments
            )
            phase_record["task_pose_evaluations"] = []
            phase_record["moveit"] = segment
            records.append(phase_record)
            current = target_positions
            continue
        if phase.reuse_target_of is not None:
            source_record = next(
                (item for item in records if item["name"] == phase.reuse_target_of),
                None,
            )
            if source_record is None:
                raise RuntimeError(
                    f"{phase.name}: reused target source not found: "
                    f"{phase.reuse_target_of}"
                )
            source_segment = source_record["moveit"]
            source_target = tuple(
                float(value) for value in source_segment["target_positions_rad"]
            )
            active_arms = {target.arm for target in phase.targets}
            if len(active_arms) != 1:
                raise RuntimeError(
                    f"{phase.name}: reused target requires exactly one active arm"
                )
            active_arm = next(iter(active_arms))
            target_positions = replace_arm(
                current,
                active_arm,
                arm_values(source_target, active_arm),
            )
            failures = []
            segment = None
            for attempt in range(1, PLANNING_ATTEMPTS_PER_IK_BRANCH + 1):
                previous_exceptions = gate.exceptions_enabled
                try:
                    candidate_segment = gate.plan_segment(
                        phase.name, current, target_positions
                    )
                    gate.set_exceptions(False)
                    candidate_segment["strict_validation"] = gate.dense_validate(
                        candidate_segment,
                        phase_gripper_modes(phase, gate.grippers),
                        False,
                        phase.targets,
                    )
                except RuntimeError as exc:
                    failures.append(f"attempt={attempt}: {exc}")
                    continue
                finally:
                    if gate.exceptions_enabled != previous_exceptions:
                        gate.set_exceptions(previous_exceptions)
                candidate_segment["planning_attempt"] = attempt
                candidate_segment["reused_joint_target_of"] = phase.reuse_target_of
                segment = candidate_segment
                break
            if segment is None:
                reason = failures[0] if failures else "no reused target branch"
                raise RuntimeError(f"{phase.name}: reused target failed; {reason}")
            phase_record["task_pose_evaluations"] = [
                {
                    "target_source_phase": phase.reuse_target_of,
                    "exact_joint_target_reused": True,
                }
            ]
            phase_record["moveit"] = segment
            records.append(phase_record)
            current = target_positions
            continue
        if phase.clear_pose:
            gate.set_exceptions(True)
            target_clear = clear
            if phase.clear_arm is not None:
                target_clear = replace_arm(
                    current,
                    phase.clear_arm,
                    arm_values(clear, phase.clear_arm),
                )
            candidates = [(target_clear, [])]
        else:
            seen_arms = [target.arm for target in phase.targets]
            if len(seen_arms) != len(set(seen_arms)):
                raise RuntimeError(f"{phase.name}: duplicate arm target")
            branch_sets = []
            for target in phase.targets:
                lower, upper = bounds[target.arm]
                current_arm = arm_values(current, target.arm)
                clear_arm = arm_values(clear, target.arm)
                clear_anchored_departure = (
                    any(
                        target.name.startswith(prefix)
                        for prefix in (
                            "first_departure_",
                            "first_return_departure_",
                            "second_departure_",
                            "second_return_departure_",
                        )
                    )
                    and not target.name.endswith("workspace")
                    and current_arm == clear_arm
                ) or target.name == "second_left_return_pregrasp"
                branches = solve_task_pose_branches(
                    kinematics[target.arm],
                    target,
                    lower,
                    upper,
                    clear_arm if clear_anchored_departure else current_arm,
                    current_arm if clear_anchored_departure else clear_arm,
                )
                if not branches:
                    raise RuntimeError(f"{phase.name}: no task-pose IK for {target.arm}")
                branch_sets.append(branches)
            candidates = []
            for combination in itertools.product(*branch_sets):
                combined = current
                for target, branch in zip(phase.targets, combination, strict=True):
                    combined = replace_arm(combined, target.arm, branch["positions_rad"])
                candidates.append((combined, list(combination)))

            active_arms = {target.arm for target in phase.targets}
            if any(
                arm_values(current, arm) == arm_values(clear, arm)
                for arm in active_arms
            ):
                gate.set_exceptions(True)

        failures = []
        selected = None
        for target_positions, evaluations in candidates:
            staged_right_departure = needs_validated_right_departure(
                phase, current, clear
            )
            if staged_right_departure:
                gate.set_exceptions(True)
            valid, contact_reason = gate.endpoint_valid(target_positions)
            if not valid:
                failures.append(f"endpoint collision: {contact_reason}")
                continue
            staged_right_reobserve = phase.clear_pose and (
                (
                    phase.name == "first_reobserve_clear"
                    and phase.clear_arm is None
                )
                or (
                    phase.clear_arm == "right"
                    and phase.name.endswith("_reobserve_clear")
                )
            )
            if staged_right_reobserve:
                try:
                    prefix_records, segment = gate.staged_bimanual_reobserve_clear(
                        phase.name, current, target_positions
                    )
                except RuntimeError as exc:
                    failures.append(f"staged reobserve clear: {exc}")
                    continue
                selected = (
                    target_positions,
                    evaluations,
                    segment,
                    prefix_records,
                )
                break
            if staged_right_departure:
                try:
                    prefix_records, segment = gate.validated_right_departure(
                        phase.name,
                        current,
                        target_positions,
                        phase.targets,
                    )
                except RuntimeError as exc:
                    failures.append(
                        "validated departure "
                        f"right_target={arm_values(target_positions, 'right')}: {exc}"
                    )
                    continue
                selected = (
                    target_positions,
                    evaluations,
                    segment,
                    prefix_records,
                )
                break
            deterministic_departure = (
                len(phase.targets) == 1
                and phase.targets[0].semantic == "pregrasp_open"
                and (
                    "departure" in phase.name
                    or (
                        phase.path_cache_key is not None
                        and phase.path_cache_key.startswith(
                            "correction_pregrasp_"
                        )
                    )
                )
            )
            planning_start = current
            departure_prefix_records: list[dict[str, object]] = []
            if deterministic_departure:
                side = phase.targets[0].arm
                wrist_roll_index = CANONICAL_JOINTS.index(
                    f"{side}_wrist_roll_joint"
                )
                wrist_roll_delta = (
                    target_positions[wrist_roll_index]
                    - current[wrist_roll_index]
                )
                if abs(wrist_roll_delta) > math.pi / 2.0:
                    rebranched = list(current)
                    rebranched[wrist_roll_index] = target_positions[wrist_roll_index]
                    rebranched_state = tuple(rebranched)
                    rebranch_name = f"{phase.name}_wrist_roll_rebranch"
                    try:
                        gate.set_exceptions(False)
                        rebranch_segment = gate.deterministic_arm_segment(
                            rebranch_name,
                            current,
                            rebranched_state,
                            side,
                        )
                        rebranch_segment["strict_validation"] = gate.dense_validate(
                            rebranch_segment,
                            phase_gripper_modes(phase, gate.grippers),
                            False,
                            (),
                        )
                    except RuntimeError as exc:
                        failures.append(f"wrist-roll rebranch: {exc}")
                        continue
                    rebranch_segment["planning_attempt"] = 1
                    rebranch_segment["jaw_line_equivalent_rebranch"] = True
                    departure_prefix_records.append(
                        {
                            "name": rebranch_name,
                            "targets": [],
                            "jaw_line_equivalent_rebranch": True,
                            "moveit": rebranch_segment,
                        }
                    )
                    planning_start = rebranched_state
            attempt_count = (
                1 if deterministic_departure else PLANNING_ATTEMPTS_PER_IK_BRANCH
            )
            for attempt in range(1, attempt_count + 1):
                try:
                    if deterministic_departure:
                        segment = gate.deterministic_arm_segment(
                            phase.name,
                            planning_start,
                            target_positions,
                            phase.targets[0].arm,
                        )
                    else:
                        segment = gate.plan_segment(
                            phase.name, current, target_positions
                        )
                except RuntimeError as exc:
                    failures.append(f"attempt={attempt}: {exc}")
                    continue
                try:
                    previous_exceptions = gate.exceptions_enabled
                    gate.set_exceptions(False)
                    segment["strict_validation"] = gate.dense_validate(
                        segment,
                        phase_gripper_modes(phase, gate.grippers),
                        any(
                            target.semantic
                            in {
                                "contact",
                                "attached_lift",
                                "attached_laydown",
                                "attached_correction",
                                "released_retreat",
                            }
                            for target in phase.targets
                        ),
                        phase.targets,
                    )
                except RuntimeError as exc:
                    maximum_joint_delta = max(
                        abs(a - b)
                        for a, b in zip(
                            planning_start, target_positions, strict=True
                        )
                    )
                    pose_errors = [
                        float(item.get("tcp_position_error_m", math.nan))
                        for item in evaluations
                    ]
                    joint_deltas = {
                        name: float(target - start)
                        for name, start, target in zip(
                            CANONICAL_JOINTS,
                            current,
                            target_positions,
                            strict=True,
                        )
                        if abs(target - start) > 0.1
                    }
                    failures.append(
                        f"attempt={attempt}: {exc}; "
                        f"max_joint_delta={maximum_joint_delta:.6f} "
                        f"tcp_endpoint_errors={pose_errors} "
                        f"joint_deltas={joint_deltas}"
                    )
                    continue
                finally:
                    if gate.exceptions_enabled != previous_exceptions:
                        gate.set_exceptions(previous_exceptions)
                segment["planning_attempt"] = attempt
                selected = (
                    target_positions,
                    evaluations,
                    segment,
                    departure_prefix_records,
                )
                break
            if selected is not None:
                break
        if selected is None:
            unique_failures = list(dict.fromkeys(failures))
            reason = (
                " | ".join(unique_failures[:8])
                if unique_failures
                else "no branch combination"
            )
            raise RuntimeError(f"{phase.name}: all branches failed; {reason}")
        current, evaluations, segment, prefix_records = selected
        if prefix_records:
            segment["phase_prefix_record_names"] = [
                str(record["name"]) for record in prefix_records
            ]
        records.extend(prefix_records)
        if phase.path_cache_key is not None:
            gate.validated_path_cache[phase.path_cache_key] = (
                tuple(current),
                copy.deepcopy(evaluations),
                copy.deepcopy(segment),
                copy.deepcopy(prefix_records),
            )
        phase_record["task_pose_evaluations"] = evaluations
        phase_record["moveit"] = segment
        records.append(phase_record)
    return current, records


def strip_runtime_values(records: list[dict[str, object]]) -> None:
    for phase in records:
        moveit = phase.get("moveit")
        if isinstance(moveit, dict):
            moveit.pop("_trajectory", None)


def summarize_strict_records(
    groups: Iterable[list[dict[str, object]]],
) -> dict[str, object]:
    total_states = 0
    total_segments = 0
    maximum_depth = 0.0
    accepted_contacts = 0
    canonicalized_contacts = 0
    maximum_raw_nondeterministic_depth = 0.0
    intended_table_contacts = 0
    maximum_table_depth = 0.0
    maximum_tcp_path_deviation = 0.0
    for records in groups:
        for phase in records:
            moveit = phase["moveit"]
            validation = moveit["strict_validation"]
            total_states += int(validation["strict_state_samples"])
            total_segments += 1
            maximum_depth = max(
                maximum_depth,
                float(validation["maximum_accepted_mesh_contact_depth_m"]),
            )
            accepted_contacts += int(validation["accepted_shallow_mesh_contact_count"])
            canonicalized_contacts += int(
                validation["canonicalized_active_arm_contact_count"]
            )
            maximum_raw_nondeterministic_depth = max(
                maximum_raw_nondeterministic_depth,
                float(validation["maximum_raw_nondeterministic_contact_depth_m"]),
            )
            intended_table_contacts += int(
                validation["intended_jaw_table_contact_count"]
            )
            maximum_table_depth = max(
                maximum_table_depth,
                float(validation["maximum_intended_jaw_table_contact_depth_m"]),
            )
            deviations = validation.get(
                "maximum_dense_tcp_path_deviation_m_by_arm", {}
            )
            if isinstance(deviations, dict) and deviations:
                maximum_tcp_path_deviation = max(
                    maximum_tcp_path_deviation,
                    *(float(value) for value in deviations.values()),
                )
    return {
        "planning_segment_count": total_segments,
        "strict_state_sample_count": total_states,
        "unapproved_contact_count": 0,
        "accepted_shallow_mesh_contact_count": accepted_contacts,
        "maximum_accepted_mesh_contact_depth_m": maximum_depth,
        "maximum_accepted_mesh_contact_depth_limit_m": MAXIMUM_EXCEPTION_DEPTH_M,
        "canonicalized_active_arm_contact_count": canonicalized_contacts,
        "maximum_raw_nondeterministic_contact_depth_m": (
            maximum_raw_nondeterministic_depth
        ),
        "intended_jaw_table_contact_count": intended_table_contacts,
        "maximum_intended_jaw_table_contact_depth_m": maximum_table_depth,
        "maximum_intended_jaw_table_contact_depth_limit_m": (
            MAXIMUM_INTENDED_TABLE_CONTACT_DEPTH_M
        ),
        "maximum_dense_tcp_path_deviation_m": maximum_tcp_path_deviation,
        "maximum_dense_tcp_path_deviation_limit_m": (
            MAXIMUM_DENSE_TCP_PATH_DEVIATION_M
        ),
    }


def evaluate_candidate(
    gate: MoveItPlanOnlyGate,
    candidate: CandidateSpec,
    corrections: tuple[CorrectionProbe, ...],
    clear: tuple[float, ...],
    kinematics: dict[str, GraspYawKinematics],
    bounds: dict[str, tuple[object, object]],
) -> dict[str, object]:
    validate_phase_contract(candidate.first_fold_phases)
    validate_phase_contract(candidate.second_fold_phases)
    for probe in corrections:
        validate_phase_contract(probe.phases)
    gate.set_exceptions(True)
    try:
        first_end, first_records = solve_and_plan_phases(
            gate,
            candidate.first_fold_phases,
            clear,
            clear,
            kinematics,
            bounds,
        )
        if first_end != clear:
            raise RuntimeError("first fold did not end at OBSERVE_CLEAR")
        second_end, second_records = solve_and_plan_phases(
            gate,
            candidate.second_fold_phases,
            clear,
            clear,
            kinematics,
            bounds,
        )
        if second_end != clear:
            raise RuntimeError("second fold did not end at OBSERVE_CLEAR")
        correction_records = []
        for probe in corrections:
            end, records = solve_and_plan_phases(
                gate, probe.phases, clear, clear, kinematics, bounds
            )
            if end != clear:
                raise RuntimeError(f"{probe.probe_id} did not return to OBSERVE_CLEAR")
            correction_records.append(
                {
                    "probe_id": probe.probe_id,
                    "primitive": probe.primitive,
                    "arm": probe.arm,
                    "corner": probe.corner,
                    "offset_xy_m": list(probe.offset_xy_m),
                    "phases": records,
                }
            )
    finally:
        gate.restore()

    all_groups = [first_records, second_records]
    all_groups.extend(item["phases"] for item in correction_records)
    strict = summarize_strict_records(all_groups)
    planning_time = sum(
        float(phase["moveit"]["planning_time_s"])
        for group in all_groups
        for phase in group
    )
    evaluated_joint_margins = [
        float(evaluation["minimum_joint_limit_margin_rad"])
        for group in all_groups
        for phase in group
        for evaluation in phase.get("task_pose_evaluations", [])
        if "minimum_joint_limit_margin_rad" in evaluation
    ]
    if not evaluated_joint_margins:
        raise RuntimeError("candidate contained no direct task-pose evaluations")
    minimum_margin = min(evaluated_joint_margins)
    result = {
        "candidate_id": candidate.candidate_id,
        "first_arm_assignment": candidate.first_arm_assignment,
        "first_axis": candidate.first_axis,
        "first_direction": candidate.first_direction,
        "second_axis": candidate.second_axis,
        "second_direction": candidate.second_direction,
        "second_active_arm": candidate.second_active_arm,
        "second_grasp_strategy": "single_arm_moving_edge_midpoint_multilayer",
        "final_footprint_requires_bounded_visual_correction": True,
        "first_expected_footprint_xyxy_m": list(
            candidate.first_expected_footprint_xyxy_m
        ),
        "final_expected_footprint_xyxy_m": list(
            candidate.final_expected_footprint_xyxy_m
        ),
        "first_fold": first_records,
        "second_fold": second_records,
        "correction_envelope": correction_records,
        "minimum_joint_limit_margin_rad": minimum_margin,
        "total_moveit_planning_time_s": planning_time,
        "strict_validation": strict,
    }
    strip_runtime_values(first_records)
    strip_runtime_values(second_records)
    for item in correction_records:
        strip_runtime_values(item["phases"])
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true", required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument(
        "--operational-limits", type=Path, default=DEFAULT_OPERATIONAL_LIMITS
    )
    parser.add_argument("--cable-review", type=Path, default=DEFAULT_CABLE_REVIEW)
    parser.add_argument(
        "--registered-urdf-manifest", type=Path, default=DEFAULT_MANIFEST
    )
    parser.add_argument(
        "--right-registration-shadow", type=Path, default=DEFAULT_SHADOW
    )
    parser.add_argument(
        "--right-tabletop-validation", type=Path, default=DEFAULT_RIGHT_TABLETOP
    )
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive and finite")
    return args


def main() -> int:
    args = parse_args()
    inputs = validate_inputs(args)
    contract = inputs["contract"]
    worktable = inputs["worktable"]
    board = worktable["board"]
    towel_bounds = towel_bounds_from_worktable(
        board["calibrated_span_m"], board["origin_in_left_base_link_xy_m"]
    )
    table_z = float(board["table_z_in_left_base_link_m"])
    candidates = build_bimanual_then_single_candidates(towel_bounds, table_z)
    corrections = build_correction_probes(
        candidates[0].first_expected_footprint_xyxy_m, table_z
    )
    clear = tuple(
        float(value)
        for value in contract["workcell_observation_candidate"]["observe_clear"][
            "joint_positions_rad"
        ]
    )
    kinematics = {
        side: GraspYawKinematics(inputs["urdf_path"], prefix=f"{side}_")
        for side in ("left", "right")
    }
    bounds = {
        side: planning.load_arm_joint_bounds(side)
        for side in ("left", "right")
    }
    grippers = gripper_modes(contract, inputs["cable"])

    rclpy.init()
    node = Node("towel_fold_sequence_plan_only")
    gate = MoveItPlanOnlyGate(
        node, args.timeout_s, grippers, clear, kinematics
    )
    selected = None
    rejected = []
    try:
        gate.wait()
        gate.apply_table_and_read_matrix(worktable)
        for candidate in candidates:
            try:
                selected = evaluate_candidate(
                    gate,
                    candidate,
                    corrections,
                    clear,
                    kinematics,
                    bounds,
                )
                break
            except (RuntimeError, TowelPlanningError) as exc:
                rejected.append(
                    {"candidate_id": candidate.candidate_id, "reason": str(exc)}
                )
                print(
                    f"TOWEL_FOLD_CANDIDATE_REJECTED "
                    f"id={candidate.candidate_id} reason={exc}",
                    flush=True,
                )
    finally:
        try:
            gate.restore()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if selected is None:
        raise RuntimeError("no canonical towel task-pose candidate passed MoveIt")

    selected_index = next(
        index
        for index, candidate in enumerate(candidates)
        if candidate.candidate_id == selected["candidate_id"]
    )
    sources = {}
    for name, path in (
        ("contract", args.contract),
        ("worktable", args.worktable),
        ("operational_limits", args.operational_limits),
        ("cable_review", args.cable_review),
        ("registered_urdf_manifest", args.registered_urdf_manifest),
        ("right_registration_shadow", args.right_registration_shadow),
        ("right_tabletop_validation", args.right_tabletop_validation),
        ("registered_urdf", inputs["urdf_path"]),
    ):
        sources[name] = {"path": str(path.resolve()), "sha256": sha256_file(path)}
    document = {
        "schema_version": 1,
        "record_kind": "towel_bimanual_then_single_task_pose_plan_only",
        "status": STATUS,
        "generated_at_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "planning_group": "both_arms",
        "kinematic_contract": {
            "arm_dof": 5,
            "arbitrary_exact_6d_pose_claimed": False,
            "required_constraints": [
                "tcp_xyz",
                "phase_semantic_approach_cone",
                "jaw_opening_line_yaw",
                "full_6d_fk_recorded",
            ],
            "position_only_ik_accepted": False,
        },
        "towel_placement": {
            "nominal_side_m": 0.300,
            "bounds_xyxy_m": list(towel_bounds),
            "table_z_m": table_z,
            "placement_basis": "centered_in_independently_validated_metric_worktable",
        },
        "selected_candidate": selected,
        "candidate_search": {
            "ordered_candidate_ids": [item.candidate_id for item in candidates],
            "rejected_before_selection": rejected,
            "not_evaluated_after_selection": [
                item.candidate_id for item in candidates[selected_index + 1 :]
            ],
        },
        "planning_scene": {
            "validated_worktable_applied": True,
            "wrist_camera_mount_collision_meshes_present": True,
            "temporary_shallow_mesh_contact_exceptions": [
                sorted(pair)
                for pair in sorted(
                    OBSERVE_CLEAR_CONTACT_EXCEPTIONS,
                    key=lambda value: sorted(value),
                )
            ],
            "temporary_exceptions_restored_before_dense_validation": True,
            "strict_path_sample_step_rad": PATH_VALIDATION_STEP_RAD,
            "dense_tcp_path_deviation_limit_m": (
                MAXIMUM_DENSE_TCP_PATH_DEVIATION_M
            ),
            "dense_tcp_path_reference": (
                "nearest point on each adjacent task-waypoint chord"
            ),
            "cable_safety_basis": (
                "all joints remain inside operator-reviewed manually swept "
                "cable-safe desired envelope; no cable mesh claim"
            ),
        },
        "gripper_collision_modes_rad": {
            name: list(values) for name, values in grippers.items()
        },
        "gripper_collision_mode_basis": {
            "task_open": (
                "validated OBSERVE_CLEAR gripper positions; automatic jaw-open "
                "command remains uncommissioned"
            ),
            "one_layer_contact": "static retention operational candidates",
            "four_layer_contact": "static retention operational candidates",
        },
        "sources": sources,
        "limitations": [
            "surface-cloth attachment and deformation are not validated in R0",
            "automatic jaw close-to-contact remains uncommissioned",
            "cable safety uses the reviewed joint envelope, not an explicit cable mesh",
            "no real motion is authorized by this artifact",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{STATUS} candidate={selected['candidate_id']} "
        f"segments={selected['strict_validation']['planning_segment_count']} "
        f"strict_states={selected['strict_validation']['strict_state_sample_count']} "
        f"motion_commands=0 output={args.output} sha256={sha256_file(args.output)}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, TowelPlanningError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1) from None
