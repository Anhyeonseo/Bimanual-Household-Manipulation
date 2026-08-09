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
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
import rclpy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


PLAN_SERVICE = "/plan_kinematic_path"
FK_SERVICE = "/compute_fk"
GROUP_NAME = "left_arm"
TCP_LINK = "left_gripper_frame_link"
BASE_FRAME = "left_base_link"

# 목표는 점이 아니라 상자다. `pose_constraints` 가 한 변 `2 x tolerance` 인
# BOX 영역을 만들고, MoveIt 은 그 안 아무 데나 들어가면 success 를 돌려준다.
# 따라서 이 값은 곧 **계획이 틀려도 되는 양** 이다.
#
# 2026-08-06 실측(작업영역 5개 좌표 x 허용치 7단계 x 3회 = 105회 계획 요청):
#
#   허용치 mm   성공     축 최대 잔차 mm
#     6.00     15/15        5.67
#     4.00     15/15        3.71
#     2.00     15/15        1.92
#     1.00     15/15        0.98
#     0.20     15/15        0.19
#
# **조인다고 IK 가 실패하지 않았다.** 105회 전부 성공했고 잔차는 허용치를
# 그대로 따라갔다. 종전 기본값 `0.006` 은 이득 없이 축당 6 mm, 모서리로는
# 10.4 mm 의 오차를 계획 시점에 심고 있었다. A4 파지 창 전체가 8 mm 였다.
DEFAULT_POSITION_TOLERANCE_M = 0.001

# 해가 정말 그 상자 안에 있었는지 되재는 값이다. 두 번째 허용치가 아니다.
# 상자의 모서리 거리는 `tolerance x sqrt(3)` 이므로 그보다 커야 정당한 해를
# 거부하지 않는다. 이것을 넘으면 solver 가 제약을 지키지 않았거나 FK 모델이
# 계획 모델과 다른 것이고, 둘 다 조용히 넘어가면 안 되는 사건이다.
PLAN_RESIDUAL_MARGIN = 1.16  # sqrt(3) = 1.732 에 여유를 더한 배수
MINIMUM_PLAN_RESIDUAL_BOUND_M = 0.0005


def plan_residual_bound_m(position_tolerance_m: float) -> float:
    """상자 기하가 허용하는 최악 잔차. 허용치에서 유도하며 손으로 정하지 않는다."""
    return max(
        MINIMUM_PLAN_RESIDUAL_BOUND_M,
        position_tolerance_m * math.sqrt(3.0) * PLAN_RESIDUAL_MARGIN,
    )


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


def measure_tcp(fk_client, node, joint_names, positions) -> list[float]:
    """MoveIt 자신에게 이 관절 해의 TCP 를 묻는다.

    별도 FK 구현을 두지 않는다. 계획을 만든 모델과 잔차를 재는 모델이 다르면
    그 차이가 잔차로 둔갑한다. 같은 서비스에 물어보면 그럴 일이 없다.
    """
    request = GetPositionFK.Request()
    request.header.frame_id = BASE_FRAME
    request.fk_link_names = [TCP_LINK]
    state = JointState()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    request.robot_state = RobotState()
    request.robot_state.joint_state = state
    response = wait_future(node, fk_client.call_async(request), timeout_s=8.0)
    if int(response.error_code.val) != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"{FK_SERVICE} refused the planned solution: "
            f"error_code={response.error_code.val}"
        )
    point = response.pose_stamped[0].pose.position
    return [float(point.x), float(point.y), float(point.z)]


def plan_one(client, node, name: str, args, z_m: float, fk_client) -> dict:
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
    bound = plan_residual_bound_m(args.position_tolerance)
    result["position_tolerance_m"] = args.position_tolerance
    result["plan_residual_bound_m"] = bound
    planned = code == MoveItErrorCodes.SUCCESS and bool(points)
    if points:
        result["final_joint_positions_rad"] = list(points[-1].positions)
        final_time = points[-1].time_from_start
        result["trajectory_duration_s"] = (
            float(final_time.sec) + float(final_time.nanosec) / 1e9
        )
    if planned:
        # 계획이 성공을 반환했다고 목표에 갔다는 뜻이 아니다. 상자 안이면
        # success 다. 실제로 어디에 섰는지 재서 기록하고, 상자 밖이면 거부한다.
        achieved = measure_tcp(
            fk_client, node, trajectory.joint_names, points[-1].positions
        )
        target = result["target_m"]
        residual = [a - t for a, t in zip(achieved, target)]
        norm = math.sqrt(sum(value * value for value in residual))
        result["achieved_tcp_m"] = achieved
        result["plan_residual_m"] = residual
        result["plan_residual_norm_m"] = norm
        result["plan_residual_axis_maximum_m"] = max(
            abs(value) for value in residual
        )
        result["within_plan_residual_bound"] = norm <= bound
        planned = result["within_plan_residual_bound"]
    result["success"] = planned
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
    parser.add_argument(
        "--grasp-offset",
        type=float,
        required=True,
        help=(
            "no default on purpose — this value drifts with convergence "
            "and calibration changes. Read the current measured value from "
            "ros2_ws/src/single_arm_bridge/config/buffered_trajectory_contract.json "
            "(tcp_contact_offsets.pick_grasp_offset_m / place_grasp_offset_m) "
            "before calling this tool; do not hardcode a caller-side default "
            "either, or it will go stale the same way the old 0.025 default did"
        ),
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=DEFAULT_POSITION_TOLERANCE_M,
    )
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
    fk_client = node.create_client(GetPositionFK, FK_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"MoveIt plan service unavailable: {PLAN_SERVICE}")
        if not fk_client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"MoveIt FK service unavailable: {FK_SERVICE}")
        plans = [
            plan_one(
                client,
                node,
                "pregrasp",
                args,
                args.object_z + args.pregrasp_offset,
                fk_client,
            ),
            plan_one(
                client,
                node,
                "grasp",
                args,
                args.object_z + args.grasp_offset,
                fk_client,
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
            "position_tolerance_m": args.position_tolerance,
            "plan_residual_bound_m": plan_residual_bound_m(
                args.position_tolerance
            ),
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
            f"position_tolerance_m={args.position_tolerance:.6f} "
            "plan_residual_norm_m="
            + ",".join(
                f"{plan.get('plan_residual_norm_m', float('nan')):.6f}"
                for plan in plans
            )
            + " execution_api_used=false"
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
