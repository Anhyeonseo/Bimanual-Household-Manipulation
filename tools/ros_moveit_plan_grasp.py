#!/usr/bin/env python3
"""Request a non-executable MoveIt plan for a top-down grasp candidate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, MoveItErrorCodes, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
import rclpy
from shape_msgs.msg import SolidPrimitive


PLAN_SERVICE = "/plan_kinematic_path"
GROUP_NAME = "left_arm"
TCP_LINK = "left_gripper_frame_link"
BASE_FRAME = "left_base_link"


def top_down_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return Rz(yaw) * Rx(pi), with TCP local +Z pointing down."""
    return (
        math.cos(yaw_rad / 2.0),
        math.sin(yaw_rad / 2.0),
        0.0,
        0.0,
    )


def pose_constraints(
    x_m: float,
    y_m: float,
    z_m: float,
    yaw_rad: float,
    position_tolerance_m: float,
    tilt_tolerance_rad: float,
) -> Constraints:
    constraint = Constraints()
    constraint.name = "top_down_grasp_candidate"

    position = PositionConstraint()
    position.header.frame_id = BASE_FRAME
    position.link_name = TCP_LINK
    region = SolidPrimitive()
    region.type = SolidPrimitive.BOX
    region.dimensions = [2.0 * position_tolerance_m] * 3
    region_pose = Pose()
    region_pose.position.x = x_m
    region_pose.position.y = y_m
    region_pose.position.z = z_m
    region_pose.orientation.w = 1.0
    position.constraint_region.primitives = [region]
    position.constraint_region.primitive_poses = [region_pose]
    position.weight = 1.0
    constraint.position_constraints = [position]

    # The current five-DOF MoveIt configuration deliberately uses
    # position_only_ik. Preserve yaw as candidate metadata, but do not pretend
    # the active IK plugin validated an orientation it does not solve.
    del yaw_rad, tilt_tolerance_rad
    return constraint


def build_request(
    x_m: float,
    y_m: float,
    z_m: float,
    yaw_rad: float,
    position_tolerance_m: float,
    tilt_tolerance_rad: float,
) -> GetMotionPlan.Request:
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.workspace_parameters.header.frame_id = BASE_FRAME
    motion.workspace_parameters.min_corner.x = -0.60
    motion.workspace_parameters.min_corner.y = -0.60
    motion.workspace_parameters.min_corner.z = -0.10
    motion.workspace_parameters.max_corner.x = 0.60
    motion.workspace_parameters.max_corner.y = 0.60
    motion.workspace_parameters.max_corner.z = 0.60
    motion.start_state.is_diff = True
    motion.goal_constraints = [
        pose_constraints(
            x_m,
            y_m,
            z_m,
            yaw_rad,
            position_tolerance_m,
            tilt_tolerance_rad,
        )
    ]
    motion.pipeline_id = "ompl"
    motion.planner_id = "RRTConnectkConfigDefault"
    motion.group_name = GROUP_NAME
    motion.num_planning_attempts = 5
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.20
    motion.max_acceleration_scaling_factor = 0.20
    return request


def wait_future(node, future, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("MoveIt plan service response timeout")
    result = future.result()
    if result is None:
        raise RuntimeError("MoveIt plan service returned no response")
    return result


def plan_one(client, node, name: str, args, z_m: float) -> dict:
    response = wait_future(
        node,
        client.call_async(
            build_request(
                args.x,
                args.y,
                z_m,
                args.yaw,
                args.position_tolerance,
                args.tilt_tolerance,
            )
        ),
        timeout_s=8.0,
    ).motion_plan_response
    code = int(response.error_code.val)
    trajectory = response.trajectory.joint_trajectory
    points = trajectory.points
    result = {
        "name": name,
        "target_m": [args.x, args.y, z_m],
        "yaw_rad": args.yaw,
        "orientation_constraint_applied": False,
        "moveit_error_code": code,
        "planning_time_s": float(response.planning_time),
        "joint_names": list(trajectory.joint_names),
        "trajectory_point_count": len(points),
    }
    if points:
        result["final_joint_positions_rad"] = list(points[-1].positions)
        final_time = points[-1].time_from_start
        result["trajectory_duration_s"] = (
            float(final_time.sec) + float(final_time.nanosec) / 1e9
        )
    result["success"] = code == MoveItErrorCodes.SUCCESS and bool(points)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan top-down pre-grasp and grasp candidates without calling "
            "any MoveIt execution Action."
        )
    )
    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--object-z", required=True, type=float)
    parser.add_argument("--yaw", required=True, type=float)
    parser.add_argument("--pregrasp-offset", type=float, default=0.10)
    parser.add_argument("--grasp-offset", type=float, default=0.025)
    parser.add_argument("--position-tolerance", type=float, default=0.006)
    parser.add_argument(
        "--tilt-tolerance",
        type=float,
        default=math.radians(25.0),
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="required acknowledgement; no execution API exists in this tool",
    )
    args = parser.parse_args()
    if not args.plan_only:
        parser.error("--plan-only is required; no planning request was sent")
    if args.pregrasp_offset <= args.grasp_offset or args.grasp_offset <= 0.0:
        parser.error("offsets must satisfy pregrasp > grasp > 0")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("so101_moveit_plan_grasp")
    client = node.create_client(GetMotionPlan, PLAN_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"MoveIt plan service unavailable: {PLAN_SERVICE}")
        plans = [
            plan_one(
                client,
                node,
                "pregrasp",
                args,
                args.object_z + args.pregrasp_offset,
            ),
            plan_one(
                client,
                node,
                "grasp",
                args,
                args.object_z + args.grasp_offset,
            ),
        ]
        result = {
            "schema_version": 1,
            "status": (
                "PLAN_ONLY_PASS"
                if all(plan["success"] for plan in plans)
                else "PLAN_ONLY_FAIL"
            ),
            "execution_api_used": False,
            "motion_authorized": False,
            "robot_target_available": False,
            "service": PLAN_SERVICE,
            "group": GROUP_NAME,
            "tcp_link": TCP_LINK,
            "ik_contract": "position_only",
            "plans": plans,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            f"{result['status']} output={args.output.resolve()} "
            f"pregrasp_points={plans[0]['trajectory_point_count']} "
            f"grasp_points={plans[1]['trajectory_point_count']} "
            "execution_api_used=false"
        )
        return 0 if result["status"] == "PLAN_ONLY_PASS" else 2
    except Exception as error:
        print(f"PLAN_ONLY_FAIL reason={error}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
