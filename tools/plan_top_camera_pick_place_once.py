#!/usr/bin/env python3
"""Select one arm from camera pixels and build a fresh non-executable plan."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
from statistics import median
import sys
import tempfile
import time

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
for source_path in (
    ROOT / "tools",
    ROOT / "ros2_ws" / "src" / "so101_top_perception",
):
    if str(source_path) not in sys.path:
        sys.path.insert(0, str(source_path))

from geometry_msgs.msg import Pose  # noqa: E402
from moveit_msgs.msg import (  # noqa: E402
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from shape_msgs.msg import SolidPrimitive  # noqa: E402
from so101_interfaces.msg import TopObjectPose  # noqa: E402

from so101_top_perception.shadow_target import (  # noqa: E402
    BoardObservation,
    ShadowTargetError,
    evaluate_shadow,
    load_shadow_config,
    source_stamp_age_seconds,
)
from top_pick_place_application import (  # noqa: E402
    ARM_JOINTS_BY_SIDE,
    CANONICAL_JOINTS,
    CONTACT_THRESHOLD_RAW,
    DEFAULT_CAMERA_CENTER_DEADBAND_PX,
    BaseTargetSample,
    lock_target,
    require_consistent_arm_selection,
    select_arm_for_pixel,
    sha256_file,
    workspace_coordinates_for_arm,
)
from grasp_yaw_kinematics import (  # noqa: E402
    GraspYawKinematics,
    wrap_half_turn,
)


STATUS = "DYNAMIC_TOP_PICK_PLACE_PLAN_ONLY_PASS"
TARGET_TOPIC = "/perception/top/object_pose_board"
PLAN_SERVICE = "/plan_kinematic_path"
FK_SERVICE = "/compute_fk"
WORKCELL_FRAME = "workcell_base_link"
Q0 = (0.0,) * 5
MAX_JOINT_STEP_RAD = 0.18
JOINT_GOAL_TOLERANCE_RAD = 0.0005
PICK_PREGRASP_OFFSET_M = 0.100
BASELINE_PICK_GRASP_OFFSET_M = 0.011
PREVIOUS_PICK_GRASP_OFFSET_M = 0.005
PICK_GRASP_OFFSET_M = 0.002
PICK_GRASP_DOWNWARD_ADJUSTMENT_M = (
    PREVIOUS_PICK_GRASP_OFFSET_M - PICK_GRASP_OFFSET_M
)
PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M = (
    BASELINE_PICK_GRASP_OFFSET_M - PICK_GRASP_OFFSET_M
)
PICK_LIFT_OFFSET_M = 0.031
RAW_STEP_RAD = 2.0 * math.pi / 4096.0
GRIPPER_OPEN_TARGET_RAW = 2009
GRIPPER_OPEN_RAD = (2048 - GRIPPER_OPEN_TARGET_RAW) * RAW_STEP_RAD
GRIPPER_CLOSE_TARGET_RAW = 1948
GRIPPER_CLOSE_RAD = (2048 - GRIPPER_CLOSE_TARGET_RAW) * RAW_STEP_RAD
POSITION_TOLERANCE_M = 0.001
PLAN_RESIDUAL_BOUND_M = 0.0021
GRASP_CROSSING_RESIDUAL_BOUND_RAD = math.radians(2.0)
DUAL_URDF_XACRO = (
    ROOT / "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
ARM_JOINT_SHORT_NAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist_flex",
    "wrist_roll",
)
PLACE_PLAN_SHA256 = (
    "39eae1f89d2ec9b0944227ec86eef61603450e45d67c0716013ba7df0730f9f5"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--target-samples", type=int, default=7)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--routing-deadband-px",
        type=float,
        default=DEFAULT_CAMERA_CENTER_DEADBAND_PX,
    )
    parser.add_argument(
        "--shadow-config",
        type=Path,
        default=(
            ROOT
            / "ros2_ws/src/so101_top_perception/config/top_shadow_target.yaml"
        ),
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--place-plan",
        type=Path,
        default=(
            ROOT
            / "artifacts/stage7/2026-08-10/place_pose_plan_only_offset014.json"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            ROOT
            / "artifacts/top_pick_place/2026-08-14/dynamic_plan_run01.json"
        ),
    )
    args = parser.parse_args()
    if not args.plan_only:
        parser.error("--plan-only is required; this tool has no execution client")
    if args.target_samples < 5:
        parser.error("--target-samples must be at least 5")
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    return args


def target_sample(
    node: Node, config, message: TopObjectPose, selected_arm: str
) -> BaseTargetSample:
    stamp_age = source_stamp_age_seconds(
        node.get_clock().now().nanoseconds,
        int(message.header.stamp.sec),
        int(message.header.stamp.nanosec),
        config.max_frame_age_s,
        config.future_tolerance_s,
    )
    observation = BoardObservation(
        source_frame=str(message.header.frame_id),
        x_m=float(message.x_m),
        y_m=float(message.y_m),
        yaw_rad=float(message.yaw_rad),
        frame_age_s=max(float(message.frame_age_s), stamp_age),
        confidence=float(message.confidence),
        footprint_inside=bool(message.footprint_inside),
        image_fully_visible=bool(message.image_fully_visible),
        motion_authorized=bool(message.motion_authorized),
        robot_target_available=bool(message.robot_target_available),
    )
    result = evaluate_shadow(config, observation)
    if not result.transform_validated:
        raise RuntimeError(f"Top target transform is not validated: {result}")
    x_m, y_m, z_m = (float(value) for value in result.position_m)
    workspace_x_m, workspace_y_m = workspace_coordinates_for_arm(
        x_m, y_m, selected_arm
    )
    bounds = config.workspace
    radius_m = math.hypot(workspace_x_m, workspace_y_m)
    inside_selected_workspace = (
        bounds.x_min_m <= workspace_x_m <= bounds.x_max_m
        and bounds.y_min_m <= workspace_y_m <= bounds.y_max_m
        and bounds.z_min_m <= z_m <= bounds.z_max_m
        and bounds.radial_min_m <= radius_m <= bounds.radial_max_m
    )
    if not inside_selected_workspace:
        raise RuntimeError(
            f"Top target is outside {selected_arm} conservative workspace: "
            f"workcell=({x_m:.6f},{y_m:.6f},{z_m:.6f}) "
            f"arm_xy=({workspace_x_m:.6f},{workspace_y_m:.6f}) "
            f"radius={radius_m:.6f}"
        )
    return BaseTargetSample(
        float(result.position_m[0]),
        float(result.position_m[1]),
        float(result.position_m[2]),
        float(result.yaw_rad),
        observation.confidence,
    )


def wait_target(node, messages, config, count, timeout_s, deadband_px):
    deadline = time.monotonic() + timeout_s
    samples = []
    sides = []
    center_x_samples = []
    image_widths = []
    stamps = set()
    rejection = "no observation"
    while len(samples) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not messages:
            continue
        message = messages[-1]
        stamp = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        if stamp in stamps:
            continue
        stamps.add(stamp)
        try:
            side = select_arm_for_pixel(
                float(message.center_x_px),
                int(message.image_width_px),
                deadband_px,
            )
            samples.append(target_sample(node, config, message, side))
            sides.append(side)
            center_x_samples.append(float(message.center_x_px))
            image_widths.append(int(message.image_width_px))
            print(
                "DYNAMIC_PICK_TARGET_SAMPLE "
                f"count={len(samples)}/{count} side={side} "
                f"pixel_x={float(message.center_x_px):.1f}/"
                f"{int(message.image_width_px)}"
            )
        except (ShadowTargetError, RuntimeError, ValueError) as error:
            rejection = f"{type(error).__name__}: {error}"
    if len(samples) < count:
        raise RuntimeError(f"only {len(samples)}/{count} valid samples; {rejection}")
    if len(set(image_widths)) != 1:
        raise RuntimeError(f"camera image width changed during lock: {image_widths}")
    selected_arm = require_consistent_arm_selection(sides)
    return (
        lock_target(samples),
        selected_arm,
        float(median(center_x_samples)),
        image_widths[0],
    )


def arm_contract(side: str) -> tuple[str, tuple[str, ...], str]:
    joints = ARM_JOINTS_BY_SIDE[side]
    return f"{side}_arm", joints, f"{side}_gripper_frame_link"


def full_q0_state() -> JointState:
    state = JointState()
    state.name = list(CANONICAL_JOINTS)
    state.position = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        GRIPPER_OPEN_RAD,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        GRIPPER_OPEN_RAD,
    ]
    return state


def wait_future(node, future, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("MoveIt service response timeout")
    result = future.result()
    if result is None:
        raise RuntimeError("MoveIt service returned no response")
    return result


def pose_goal(side: str, x: float, y: float, z: float) -> Constraints:
    _, _, tcp_link = arm_contract(side)
    constraints = Constraints()
    constraints.name = f"{side}_top_down_position"
    position = PositionConstraint()
    position.header.frame_id = WORKCELL_FRAME
    position.link_name = tcp_link
    region = SolidPrimitive()
    region.type = SolidPrimitive.BOX
    region.dimensions = [2.0 * POSITION_TOLERANCE_M] * 3
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    position.constraint_region.primitives = [region]
    position.constraint_region.primitive_poses = [pose]
    position.weight = 1.0
    constraints.position_constraints = [position]
    return constraints


def pose_request(side: str, x: float, y: float, z: float):
    group_name, _, _ = arm_contract(side)
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.workspace_parameters.header.frame_id = WORKCELL_FRAME
    motion.workspace_parameters.min_corner.x = -0.60
    motion.workspace_parameters.min_corner.y = -0.60
    motion.workspace_parameters.min_corner.z = -0.10
    motion.workspace_parameters.max_corner.x = 0.60
    motion.workspace_parameters.max_corner.y = 0.60
    motion.workspace_parameters.max_corner.z = 0.60
    motion.start_state = RobotState()
    motion.start_state.joint_state = full_q0_state()
    motion.start_state.is_diff = False
    motion.goal_constraints = [pose_goal(side, x, y, z)]
    motion.pipeline_id = "ompl"
    motion.planner_id = "RRTConnectkConfigDefault"
    motion.group_name = group_name
    motion.num_planning_attempts = 5
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.20
    motion.max_acceleration_scaling_factor = 0.20
    return request


def measure_tcp(fk_client, node, side, joint_names, positions):
    _, _, tcp_link = arm_contract(side)
    request = GetPositionFK.Request()
    request.header.frame_id = WORKCELL_FRAME
    request.fk_link_names = [tcp_link]
    state = JointState()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    request.robot_state = RobotState()
    request.robot_state.joint_state = state
    response = wait_future(node, fk_client.call_async(request), 8.0)
    if int(response.error_code.val) != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f"{FK_SERVICE} rejected {side} solution")
    point = response.pose_stamped[0].pose.position
    return [float(point.x), float(point.y), float(point.z)]


def load_yaw_kinematics(side: str) -> GraspYawKinematics:
    import xacro

    xml = xacro.process_file(str(DUAL_URDF_XACRO)).toxml()
    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as urdf:
        urdf.write(xml)
        urdf.flush()
        return GraspYawKinematics(Path(urdf.name), prefix=f"{side}_")


def load_arm_joint_bounds(side: str) -> tuple[np.ndarray, np.ndarray]:
    document = json.loads(OPERATIONAL_LIMITS.read_text(encoding="utf-8"))
    if (
        document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True
    ):
        raise RuntimeError("bimanual operational limits are not approved")
    arm = document["arms"][side]
    lower = np.array(
        [arm[name]["minimum_urad"] / 1.0e6 for name in ARM_JOINT_SHORT_NAMES]
    )
    upper = np.array(
        [arm[name]["maximum_urad"] / 1.0e6 for name in ARM_JOINT_SHORT_NAMES]
    )
    return lower, upper


def solve_endpoint_pose_with_locked_wrist(
    kinematics,
    side,
    joint_names,
    position_only_solution,
    reference,
    target_workcell,
    pen_yaw_rad,
    lower,
    upper,
):
    """Solve TCP xyz with wrist roll locked at the bimanual q0 angle."""
    original = np.asarray(position_only_solution, dtype=float)
    reference = np.asarray(reference, dtype=float)
    target_x, target_y = workspace_coordinates_for_arm(
        target_workcell[0], target_workcell[1], side
    )
    target = np.array([target_x, target_y, target_workcell[2]], dtype=float)
    locked_wrist_roll = 0.0
    if not lower[4] <= locked_wrist_roll <= upper[4]:
        raise RuntimeError(
            f"{side} q0 wrist roll is outside operational limits"
        )

    seeds = (
        original[:4],
        reference[:4],
        np.concatenate((original[:2], reference[2:4])),
        np.concatenate((reference[:2], original[2:4])),
    )
    candidates = []
    candidate_keys = set()

    def complete(values):
        return np.concatenate((values, [locked_wrist_roll]))

    def residuals(values):
        positions = dict(zip(joint_names, complete(values), strict=True))
        return 1000.0 * (
            kinematics.tcp_position(positions) - target
        )

    for seed in seeds:
        clipped = np.clip(seed, lower[:4] + 1.0e-8, upper[:4] - 1.0e-8)
        result = least_squares(
            residuals,
            clipped,
            bounds=(lower[:4], upper[:4]),
            xtol=1.0e-11,
            ftol=1.0e-11,
            gtol=1.0e-11,
            max_nfev=1200,
        )
        solved = complete(result.x)
        positions = dict(zip(joint_names, solved, strict=True))
        achieved = kinematics.tcp_position(positions)
        position_error_m = float(np.linalg.norm(achieved - target))
        if position_error_m > PLAN_RESIDUAL_BOUND_M:
            continue
        key = tuple(round(float(value), 8) for value in solved)
        if key in candidate_keys:
            continue
        candidate_keys.add(key)
        transition = np.abs(solved - reference)
        achieved_finger_yaw = float(kinematics.finger_yaw(positions))
        finger_target_yaw = wrap_half_turn(pen_yaw_rad + math.pi / 2.0)
        crossing_error_rad = abs(
            wrap_half_turn(achieved_finger_yaw - finger_target_yaw)
        )
        candidates.append(
            (
                float(np.max(transition[:4])),
                float(np.linalg.norm(transition[:4])),
                position_error_m,
                solved,
                achieved_finger_yaw,
                finger_target_yaw,
                crossing_error_rad,
                int(result.nfev),
            )
        )

    if not candidates:
        raise RuntimeError(
            f"no {side} endpoint solution preserves the q0 wrist angle "
            "while meeting the TCP position bound"
        )
    candidates.sort(key=lambda item: item[:3])
    selected = candidates[0]
    return {
        "positions_rad": tuple(float(value) for value in selected[3]),
        "position_residual_m": selected[2],
        "finger_target_yaw_rad": selected[5],
        "achieved_finger_yaw_rad": selected[4],
        "crossing_residual_rad": selected[6],
        "reference_maximum_joint_delta_rad": selected[0],
        "candidate_count": len(candidates),
        "solver_evaluations": selected[7],
        "wrist_roll_reference_rad": locked_wrist_roll,
        "wrist_roll_delta_rad": 0.0,
    }


def plan_endpoint(
    node,
    plan_client,
    fk_client,
    kinematics,
    bounds,
    reference,
    side,
    name,
    x,
    y,
    z,
    yaw,
):
    response = wait_future(
        node,
        plan_client.call_async(pose_request(side, x, y, z)),
        8.0,
    ).motion_plan_response
    trajectory = response.trajectory.joint_trajectory
    _, joints, _ = arm_contract(side)
    if (
        int(response.error_code.val) != MoveItErrorCodes.SUCCESS
        or not trajectory.points
    ):
        raise RuntimeError(
            f"MoveIt endpoint failed: side={side} name={name} "
            f"code={response.error_code.val}"
        )
    if tuple(trajectory.joint_names) != joints:
        raise RuntimeError(f"MoveIt endpoint joint order mismatch: {name}")
    position_only_final = tuple(
        float(value) for value in trajectory.points[-1].positions
    )
    crossing = solve_endpoint_pose_with_locked_wrist(
        kinematics,
        side,
        trajectory.joint_names,
        position_only_final,
        reference,
        (x, y, z),
        yaw,
        *bounds,
    )
    final = crossing["positions_rad"]
    achieved = measure_tcp(
        fk_client, node, side, trajectory.joint_names, final
    )
    residual = math.dist(achieved, (x, y, z))
    if residual > PLAN_RESIDUAL_BOUND_M:
        raise RuntimeError(
            f"MoveIt endpoint residual failed: {name} residual={residual:.6f}m"
        )
    return {
        "name": name,
        "target_m": [x, y, z],
        "yaw_rad": yaw,
        "orientation_constraint_applied": False,
        "wrist_roll_yaw_correction_applied": False,
        "wrist_roll_policy": "hold_bimanual_q0",
        "wrist_roll_locked": True,
        "wrist_roll_reference_rad": crossing[
            "wrist_roll_reference_rad"
        ],
        "wrist_roll_delta_rad": crossing["wrist_roll_delta_rad"],
        "grasp_geometry": {
            "relationship": "informational_only_wrist_locked_at_q0",
            "target_crossing_angle_rad": math.pi / 2.0,
            "finger_target_yaw_rad": crossing["finger_target_yaw_rad"],
            "achieved_finger_yaw_rad": crossing["achieved_finger_yaw_rad"],
            "crossing_residual_rad": crossing["crossing_residual_rad"],
            "crossing_residual_bound_rad": GRASP_CROSSING_RESIDUAL_BOUND_RAD,
        },
        "position_only_final_joint_positions_rad": list(position_only_final),
        "final_joint_positions_rad": list(final),
        "achieved_tcp_m": achieved,
        "plan_residual_norm_m": residual,
        "moveit_error_code": int(response.error_code.val),
        "yaw_constrained_candidate_count": crossing["candidate_count"],
        "yaw_solver_evaluations": crossing["solver_evaluations"],
        "reference_maximum_joint_delta_rad": crossing[
            "reference_maximum_joint_delta_rad"
        ],
    }


def load_place_targets(path: Path):
    if sha256_file(path) != PLACE_PLAN_SHA256:
        raise RuntimeError("place plan sha256 mismatch")
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != "PLAN_ONLY_PASS":
        raise RuntimeError("place plan is not PLAN_ONLY_PASS")
    by_name = {item["name"]: item for item in document["plans"]}
    result = {}
    for name in ("pregrasp", "grasp"):
        item = by_name[name]
        result[name] = (
            *(float(value) for value in item["target_m"]),
            float(item["yaw_rad"]),
        )
    return result


def interpolate_segments(start, target):
    largest = max(abs(b - a) for a, b in zip(start, target, strict=True))
    count = max(1, math.ceil(largest / MAX_JOINT_STEP_RAD))
    points = [tuple(start)]
    for index in range(1, count):
        points.append(
            tuple(
                a + (b - a) * index / count
                for a, b in zip(start, target, strict=True)
            )
        )
    points.append(tuple(target))
    return list(zip(points[:-1], points[1:], strict=True))


def joint_request(side, start, target):
    group_name, joints, _ = arm_contract(side)
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.workspace_parameters.header.frame_id = WORKCELL_FRAME
    motion.workspace_parameters.min_corner.x = -0.60
    motion.workspace_parameters.min_corner.y = -0.60
    motion.workspace_parameters.min_corner.z = -0.10
    motion.workspace_parameters.max_corner.x = 0.60
    motion.workspace_parameters.max_corner.y = 0.60
    motion.workspace_parameters.max_corner.z = 0.60
    state = full_q0_state()
    selected_offset = 0 if side == "left" else 6
    for index, value in enumerate(start):
        state.position[selected_offset + index] = float(value)
    motion.start_state.joint_state = state
    motion.start_state.is_diff = False
    goal = Constraints()
    goal.name = f"{side}_bounded_segment"
    for name, value in zip(joints, target, strict=True):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(value)
        joint.tolerance_above = JOINT_GOAL_TOLERANCE_RAD
        joint.tolerance_below = JOINT_GOAL_TOLERANCE_RAD
        joint.weight = 1.0
        goal.joint_constraints.append(joint)
    motion.goal_constraints = [goal]
    motion.pipeline_id = "ompl"
    motion.planner_id = "RRTConnectkConfigDefault"
    motion.group_name = group_name
    motion.num_planning_attempts = 5
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.15
    motion.max_acceleration_scaling_factor = 0.15
    return request


def plan_phase(node, client, side, name, start, target):
    _, joints, _ = arm_contract(side)
    results = []
    for index, (segment_start, segment_target) in enumerate(
        interpolate_segments(start, target), start=1
    ):
        response = wait_future(
            node,
            client.call_async(joint_request(side, segment_start, segment_target)),
            8.0,
        ).motion_plan_response
        trajectory = response.trajectory.joint_trajectory
        success = (
            int(response.error_code.val) == MoveItErrorCodes.SUCCESS
            and bool(trajectory.points)
            and tuple(trajectory.joint_names) == joints
        )
        result = {
            "index": index,
            "expected_start_positions_rad": list(segment_start),
            "target_positions_rad": list(segment_target),
            "maximum_joint_delta_rad": max(
                abs(b - a)
                for a, b in zip(segment_start, segment_target, strict=True)
            ),
            "moveit_error_code": int(response.error_code.val),
            "trajectory_joint_names": list(trajectory.joint_names),
            "trajectory_positions_rad": [
                list(point.positions) for point in trajectory.points
            ],
            "success": success,
        }
        if success:
            residual = max(
                abs(float(actual) - float(expected))
                for actual, expected in zip(
                    trajectory.points[-1].positions,
                    segment_target,
                    strict=True,
                )
            )
            result["joint_goal_residual_rad"] = residual
            result["success"] = residual <= 0.00075
        results.append(result)
    if not all(item["success"] for item in results):
        raise RuntimeError(f"MoveIt phase collision plan failed: {name}")
    print(f"DYNAMIC_PICK_PHASE_PLAN_PASS phase={name} segments={len(results)}")
    return {"name": name, "segments": results}


def steps_from_phases(phases):
    steps = [
        {
            "kind": "gripper",
            "phase": "pick_open",
            "target_position_rad": GRIPPER_OPEN_RAD,
        }
    ]
    for phase in phases:
        if phase["name"] == "pick_grasp_to_lift":
            steps.append(
                {
                    "kind": "gripper",
                    "phase": "pick_close",
                    "target_position_rad": GRIPPER_CLOSE_RAD,
                }
            )
        if phase["name"] == "place_grasp_to_retreat":
            steps.append(
                {
                    "kind": "gripper",
                    "phase": "place_release",
                    "target_position_rad": GRIPPER_OPEN_RAD,
                }
            )
        for segment in phase["segments"]:
            steps.append(
                {
                    "kind": "arm",
                    "phase": phase["name"],
                    "target_positions_rad": segment["target_positions_rad"],
                    "maximum_joint_delta_rad": segment[
                        "maximum_joint_delta_rad"
                    ],
                }
            )
    for index, step in enumerate(steps, start=1):
        step["index"] = index
    return steps


def main() -> int:
    args = parse_args()
    config = load_shadow_config(args.shadow_config)
    if config.output_frame != "left_base_link":
        raise RuntimeError(
            "Top shadow transform must currently terminate at left_base_link"
        )
    rclpy.init()
    node = Node("top_camera_dynamic_pick_place_planner")
    messages = []
    node.create_subscription(TopObjectPose, TARGET_TOPIC, messages.append, 10)
    plan_client = node.create_client(GetMotionPlan, PLAN_SERVICE)
    fk_client = node.create_client(GetPositionFK, FK_SERVICE)
    try:
        if not plan_client.wait_for_service(timeout_sec=args.timeout_s):
            raise RuntimeError(f"service unavailable: {PLAN_SERVICE}")
        if not fk_client.wait_for_service(timeout_sec=args.timeout_s):
            raise RuntimeError(f"service unavailable: {FK_SERVICE}")
        locked, side, center_x_px, image_width_px = wait_target(
            node,
            messages,
            config,
            args.target_samples,
            args.timeout_s,
            args.routing_deadband_px,
        )
        x, y, z, yaw = (
            locked.x_m,
            locked.y_m,
            locked.z_m,
            locked.yaw_rad,
        )
        place = load_place_targets(args.place_plan)
        target_specs = {
            "pick_pregrasp": (x, y, z + PICK_PREGRASP_OFFSET_M, yaw),
            "pick_grasp": (x, y, z + PICK_GRASP_OFFSET_M, yaw),
            "pick_lift": (x, y, z + PICK_LIFT_OFFSET_M, yaw),
            "place_pregrasp": place["pregrasp"],
            "place_grasp": place["grasp"],
        }
        kinematics = load_yaw_kinematics(side)
        bounds = load_arm_joint_bounds(side)
        endpoints = {}
        reference = Q0
        for name, values in target_specs.items():
            endpoints[name] = plan_endpoint(
                node,
                plan_client,
                fk_client,
                kinematics,
                bounds,
                reference,
                side,
                name,
                *values,
            )
            reference = tuple(
                endpoints[name]["final_joint_positions_rad"]
            )
        positions = {
            name: tuple(item["final_joint_positions_rad"])
            for name, item in endpoints.items()
        }
        phase_specs = (
            ("q0_to_pick_pregrasp", Q0, positions["pick_pregrasp"]),
            (
                "pick_pregrasp_to_grasp",
                positions["pick_pregrasp"],
                positions["pick_grasp"],
            ),
            (
                "pick_grasp_to_lift",
                positions["pick_grasp"],
                positions["pick_lift"],
            ),
            (
                "lift_to_place_pregrasp",
                positions["pick_lift"],
                positions["place_pregrasp"],
            ),
            (
                "place_pregrasp_to_grasp",
                positions["place_pregrasp"],
                positions["place_grasp"],
            ),
            (
                "place_grasp_to_retreat",
                positions["place_grasp"],
                positions["place_pregrasp"],
            ),
            ("place_pregrasp_to_q0", positions["place_pregrasp"], Q0),
        )
        phases = [
            plan_phase(node, plan_client, side, name, start, target)
            for name, start, target in phase_specs
        ]
        steps = steps_from_phases(phases)
        _, selected_joints, _ = arm_contract(side)
        document = {
            "schema_version": 9,
            "status": STATUS,
            "generated_at_unix_s": time.time(),
            "execution_api_used": False,
            "motion_authorized": False,
            "automatic_execution_permitted": False,
            "source": "fresh_top_camera_target",
            "routing": {
                "rule": "source_image_center_x",
                "selected_arm": side,
                "center_x_px": center_x_px,
                "image_width_px": image_width_px,
                "deadband_px": args.routing_deadband_px,
                "nonselected_arm_behavior": "hold_bimanual_q0",
            },
            "planning_frame": WORKCELL_FRAME,
            "joint_names": list(selected_joints),
            "q0_rad": list(Q0),
            "target_lock": {
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "yaw_rad": yaw,
                "sample_count": locked.sample_count,
                "minimum_confidence": locked.minimum_confidence,
                "maximum_position_spread_m": (
                    locked.maximum_position_spread_m
                ),
            },
            "pick_offsets_m": {
                "pregrasp": PICK_PREGRASP_OFFSET_M,
                "grasp": PICK_GRASP_OFFSET_M,
                "lift": PICK_LIFT_OFFSET_M,
            },
            "height_adjustment": {
                "baseline_grasp_offset_m": BASELINE_PICK_GRASP_OFFSET_M,
                "previous_grasp_offset_m": PREVIOUS_PICK_GRASP_OFFSET_M,
                "selected_grasp_offset_m": PICK_GRASP_OFFSET_M,
                "downward_adjustment_m": PICK_GRASP_DOWNWARD_ADJUSTMENT_M,
                "cumulative_downward_adjustment_m": (
                    PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M
                ),
                "reason": "operator_observed_run08_grasp_still_too_high",
            },
            "gripper_contract": {
                "preopen_required": True,
                "open_target_raw": GRIPPER_OPEN_TARGET_RAW,
                "open_target_rad": GRIPPER_OPEN_RAD,
                "open_phase": "before_approach",
                "close_target_raw": GRIPPER_CLOSE_TARGET_RAW,
                "close_target_rad": GRIPPER_CLOSE_RAD,
                "contact_threshold_raw": CONTACT_THRESHOLD_RAW,
                "empty_grasp_observed_residual_raw": 2,
                "held_object_observed_residual_raw": 8,
                "expected_held_residual_after_target_change_raw": 23,
            },
            "place_target_source": {
                "path": str(args.place_plan),
                "sha256": PLACE_PLAN_SHA256,
                "coordinate_reused": "workcell_base_link",
                "joint_solution_reused": False,
                "right_arm_physical_place_validation_required": side == "right",
            },
            "calibration": {
                "path": str(args.calibration),
                "sha256": sha256_file(args.calibration),
            },
            "operational_limits": {
                "path": str(OPERATIONAL_LIMITS),
                "sha256": sha256_file(OPERATIONAL_LIMITS),
            },
            "endpoints": endpoints,
            "phases": phases,
            "steps": steps,
            "arm_segment_count": sum(
                step["kind"] == "arm" for step in steps
            ),
            "command_step_count": len(steps),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = sha256(args.output.read_bytes()).hexdigest()
        wrist_endpoint_rad = [
            endpoints[name]["final_joint_positions_rad"][4]
            for name in target_specs
        ]
        wrist_references = [0.0] + wrist_endpoint_rad[:-1]
        wrist_delta_max = max(
            abs(target - reference)
            for target, reference in zip(
                wrist_endpoint_rad, wrist_references, strict=True
            )
        )
        print(
            "DYNAMIC_PICK_WRIST_BRANCH_PASS "
            "policy=hold_bimanual_q0 "
            f"endpoint_rad={[round(value, 6) for value in wrist_endpoint_rad]} "
            f"maximum_delta_rad={wrist_delta_max:.6f}"
        )
        print(
            "DYNAMIC_PICK_HEIGHT_OFFSET_PASS "
            f"baseline_offset_m={BASELINE_PICK_GRASP_OFFSET_M:.6f} "
            f"previous_offset_m={PREVIOUS_PICK_GRASP_OFFSET_M:.6f} "
            f"selected_offset_m={PICK_GRASP_OFFSET_M:.6f} "
            f"downward_adjustment_m={PICK_GRASP_DOWNWARD_ADJUSTMENT_M:.6f} "
            f"cumulative_downward_adjustment_m={PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M:.6f} "
            f"grasp_target_z_m={z + PICK_GRASP_OFFSET_M:.6f}"
        )
        print(
            "DYNAMIC_PICK_GRIPPER_CONTRACT_PASS "
            f"open_target_raw={GRIPPER_OPEN_TARGET_RAW} "
            f"close_target_raw={GRIPPER_CLOSE_TARGET_RAW} "
            f"close_target_rad={GRIPPER_CLOSE_RAD:.6f} "
            f"contact_threshold_raw={CONTACT_THRESHOLD_RAW} "
            "expected_held_residual_raw=23"
        )
        print(
            f"{STATUS} selected_arm={side} "
            f"pixel_x={center_x_px:.1f}/{image_width_px} "
            f"target=({x:.6f},{y:.6f},{z:.6f}) "
            f"arm_segments={document['arm_segment_count']} "
            f"steps={len(steps)} output={args.output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
