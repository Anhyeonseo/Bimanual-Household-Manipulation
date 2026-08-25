"""MoveIt and camera helpers shared by desk-organization task planners."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from statistics import median
import sys
import tempfile
import time

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from sensor_msgs.msg import JointState
from so101_interfaces.msg import TopObjectPose

ROOT = Path(__file__).resolve().parents[2]
for source_path in (
    ROOT,
    ROOT / "ros2_ws" / "src" / "so101_top_perception",
):
    if str(source_path) not in sys.path:
        sys.path.insert(0, str(source_path))

from tools.lib.desk_task_runtime import (  # noqa: E402
    ARM_JOINTS_BY_SIDE, CANONICAL_JOINTS, BaseTargetSample,
    lock_target, sha256_file, workspace_coordinates_for_arm,
)
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from so101_top_perception.shadow_target import (  # noqa: E402
    BoardObservation, evaluate_shadow, load_shadow_config,
    source_stamp_age_seconds,
)

PLAN_SERVICE = "/plan_kinematic_path"
FK_SERVICE = "/compute_fk"
WORKCELL_FRAME = "workcell_base_link"
MAX_JOINT_STEP_RAD = 0.18
JOINT_GOAL_TOLERANCE_RAD = 0.0005
DEFAULT_DUAL_URDF_PATH = (
    ROOT / "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
DUAL_URDF_ENVIRONMENT = "SO101_DUAL_URDF_PATH"
TOP_HOMOGRAPHY_PATH = (
    ROOT / "ros2_ws/src/manipulation_camera_manager/config/"
    "top_worktable_homography.yaml"
)
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
ARM_JOINT_SHORT_NAMES = (
    "base", "shoulder", "elbow", "wrist_flex", "wrist_roll",
)
BOTH_ARM_JOINTS = tuple(
    name
    for side in ("left", "right")
    for name in ARM_JOINTS_BY_SIDE[side][:5]
)


def dual_urdf_path() -> Path:
    path = Path(os.environ.get(DUAL_URDF_ENVIRONMENT, DEFAULT_DUAL_URDF_PATH))
    if not path.is_file():
        raise RuntimeError(f"dual robot description does not exist: {path}")
    return path


def target_sample(node, config, message: TopObjectPose, selected_arm: str):
    stamp_age = source_stamp_age_seconds(
        node.get_clock().now().nanoseconds,
        int(message.header.stamp.sec), int(message.header.stamp.nanosec),
        config.max_frame_age_s, config.future_tolerance_s,
    )
    observation = BoardObservation(
        source_frame=str(message.header.frame_id),
        x_m=float(message.x_m), y_m=float(message.y_m),
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
        x_m, y_m, selected_arm, z_m
    )
    bounds = config.workspace
    radius_m = math.hypot(workspace_x_m, workspace_y_m)
    inside = (
        bounds.x_min_m <= workspace_x_m <= bounds.x_max_m
        and bounds.y_min_m <= workspace_y_m <= bounds.y_max_m
        and bounds.z_min_m <= z_m <= bounds.z_max_m
        and bounds.radial_min_m <= radius_m <= bounds.radial_max_m
    )
    if not inside:
        raise RuntimeError(f"target is outside the {selected_arm} workspace")
    return BaseTargetSample(
        x_m=x_m, y_m=y_m, z_m=z_m,
        yaw_rad=float(result.yaw_rad), confidence=float(message.confidence),
    )


def screen_positive_x_unit_workcell(
    homography_path: Path, center_x_px: float, center_y_px: float
) -> tuple[float, float]:
    document = yaml.safe_load(homography_path.read_text(encoding="utf-8"))
    pixel_to_board = np.asarray(
        document["homography"]["rectified_pixel_to_board_m"]["data"], dtype=float
    )
    base_from_board = np.asarray(
        document["base_registration"]["base_from_board"]["data"], dtype=float
    )
    if pixel_to_board.shape != (3, 3) or base_from_board.shape != (4, 4):
        raise RuntimeError("top homography matrix dimensions are invalid")

    def project(pixel_x: float) -> np.ndarray:
        homogeneous = pixel_to_board @ np.asarray(
            (pixel_x, center_y_px, 1.0), dtype=float
        )
        if abs(float(homogeneous[2])) < 1.0e-12:
            raise RuntimeError("top homography screen-x direction is singular")
        return homogeneous[:2] / homogeneous[2]

    board_delta = project(center_x_px + 1.0) - project(center_x_px)
    workcell_delta = base_from_board[:2, :2] @ board_delta
    length = float(np.linalg.norm(workcell_delta))
    if not math.isfinite(length) or length < 1.0e-12:
        raise RuntimeError("top homography screen-x direction is invalid")
    unit = workcell_delta / length
    return float(unit[0]), float(unit[1])


def arm_contract(side: str) -> tuple[str, tuple[str, ...], str]:
    joints = ARM_JOINTS_BY_SIDE[side]
    return f"{side}_arm", joints, f"{side}_gripper_frame_link"


def full_q0_state() -> JointState:
    state = JointState()
    state.name = list(CANONICAL_JOINTS)
    state.position = [0.0] * 12
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
    xml = xacro.process_file(str(dual_urdf_path())).toxml()
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


def interpolate_segments(start, target):
    largest = max(abs(b - a) for a, b in zip(start, target, strict=True))
    count = max(1, math.ceil(largest / MAX_JOINT_STEP_RAD))
    points = [tuple(start)]
    for index in range(1, count):
        points.append(tuple(
            a + (b - a) * index / count
            for a, b in zip(start, target, strict=True)
        ))
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


def both_arms_joint_request(start_positions, target_positions):
    """Build one non-executing collision plan to an exact dual-arm pose."""
    if len(start_positions) != len(CANONICAL_JOINTS):
        raise ValueError("dual-arm start must contain all 12 canonical joints")
    if len(target_positions) != len(CANONICAL_JOINTS):
        raise ValueError("dual-arm target must contain all 12 canonical joints")
    if not all(math.isfinite(float(value)) for value in start_positions):
        raise ValueError("dual-arm start contains a non-finite value")
    if not all(math.isfinite(float(value)) for value in target_positions):
        raise ValueError("dual-arm target contains a non-finite value")

    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.workspace_parameters.header.frame_id = WORKCELL_FRAME
    motion.workspace_parameters.min_corner.x = -0.60
    motion.workspace_parameters.min_corner.y = -0.60
    motion.workspace_parameters.min_corner.z = -0.10
    motion.workspace_parameters.max_corner.x = 0.60
    motion.workspace_parameters.max_corner.y = 0.60
    motion.workspace_parameters.max_corner.z = 0.60
    state = JointState()
    state.name = list(CANONICAL_JOINTS)
    state.position = [float(value) for value in start_positions]
    motion.start_state.joint_state = state
    motion.start_state.is_diff = False
    target_by_name = dict(zip(CANONICAL_JOINTS, target_positions, strict=True))
    goal = Constraints()
    goal.name = "observe_clear_both_arms"
    for name in BOTH_ARM_JOINTS:
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(target_by_name[name])
        joint.tolerance_above = JOINT_GOAL_TOLERANCE_RAD
        joint.tolerance_below = JOINT_GOAL_TOLERANCE_RAD
        joint.weight = 1.0
        goal.joint_constraints.append(joint)
    motion.goal_constraints = [goal]
    motion.pipeline_id = "ompl"
    motion.planner_id = "RRTConnectkConfigDefault"
    motion.group_name = "both_arms"
    motion.num_planning_attempts = 5
    motion.allowed_planning_time = 8.0
    motion.max_velocity_scaling_factor = 0.10
    motion.max_acceleration_scaling_factor = 0.10
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
                abs(b - a) for a, b in zip(segment_start, segment_target, strict=True)
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
                    trajectory.points[-1].positions, segment_target, strict=True
                )
            )
            result["joint_goal_residual_rad"] = residual
            result["success"] = residual <= 0.00075
        results.append(result)
    if not all(item["success"] for item in results):
        raise RuntimeError(f"MoveIt phase collision plan failed: {name}")
    print(f"DESK_TASK_PHASE_PLAN_PASS phase={name} segments={len(results)}")
    return {"name": name, "segments": results}
