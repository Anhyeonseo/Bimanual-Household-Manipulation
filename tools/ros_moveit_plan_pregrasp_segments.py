#!/usr/bin/env python3
"""Plan bounded joint-space segments to a validated pregrasp or grasp."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy

from tools.joint_calibration import load_calibration, raw_to_urad  # noqa: E402


PLAN_SERVICE = "/plan_kinematic_path"
GROUP_NAME = "left_arm"
ARM_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
DEFAULT_MAX_JOINT_STEP_RAD = 0.30
DEFAULT_DURATION_S = 2.0
# 관절 목표도 점이 아니라 구간이다. MoveIt 은 이 허용치 안에 들어오면
# success 를 돌려주므로 이 값이 곧 **계획이 틀려도 되는 양** 이다.
#
# 2026-08-06 실측(경로 3종 x 허용치 6단계 x 3회 = 54회 계획 요청):
#
#   허용치 rad   성공     관절 잔차 raw   TCP 잔차 mm
#     0.0050    9/9          3.05          2.81
#     0.0020    9/9          1.28          0.95
#     0.0010    9/9          0.64          0.57
#     0.0005    9/9          0.33          0.34
#     0.0001    9/9          0.06          0.04
#
# **54회 전부 성공했다.** 조인다고 계획이 실패하지 않았다. 종전 `0.005` 는
# 서보가 움직이기도 전에 TCP 를 최대 `2.81 mm` 어긋나게 하고 있었고, 그것은
# 과제 허용치 `4 mm` 의 대부분이다. 그 상태로 수렴 루프를 돌리면 팔의 처짐이
# 아니라 이 허용치를 재게 된다.
#
# `0.0005 rad` 는 `0.33 raw` 로 서보의 1-raw 양자화(`2*pi/4096 = 0.001534 rad`)
# 보다 작다. 그 아래로는 하드웨어가 구분하지 못하므로 더 조일 이유가 없다.
JOINT_GOAL_TOLERANCE_RAD = 0.0005

# 해가 정말 그 구간 안에 있었는지 되재는 값이다. 두 번째 허용치가 아니다.
JOINT_GOAL_RESIDUAL_MARGIN = 1.5

TARGET_NAMES = ("pregrasp", "grasp")
# 넘겨명령 목표는 어떤 `PLAN_ONLY_PASS` 파일에도 없다. 그 목표를 담은 합성
# 문서를 만들어 MoveIt 이 내지 않은 것에 `PLAN_ONLY_PASS` 를 붙이는 대신,
# 명시적 관절 목표를 정직하게 받는다. 한계 검사와 충돌 검사는 동일하다.
EXPLICIT_TARGET_NAME = "explicit"


def parse_joint_vector(value: str) -> tuple[float, ...]:
    try:
        positions = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("joint vector must contain numbers") from error
    if len(positions) != len(ARM_JOINTS):
        raise argparse.ArgumentTypeError("joint vector must contain exactly 5 values")
    if not all(math.isfinite(position) for position in positions):
        raise argparse.ArgumentTypeError("joint vector must be finite")
    return positions


def load_target(path: Path, target_name: str) -> tuple[float, ...]:
    if target_name not in TARGET_NAMES:
        raise ValueError(f"target_name must be one of {','.join(TARGET_NAMES)}")
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != "PLAN_ONLY_PASS":
        raise ValueError("source grasp plan is not PLAN_ONLY_PASS")
    if document.get("execution_api_used") is not False:
        raise ValueError("source grasp plan must prove execution_api_used=false")

    plans = document.get("plans")
    if not isinstance(plans, list):
        raise ValueError("source grasp plan has no plans")
    matches = [plan for plan in plans if plan.get("name") == target_name]
    if len(matches) != 1 or matches[0].get("success") is not True:
        raise ValueError(
            f"source grasp plan has no unique successful {target_name}"
        )
    plan = matches[0]
    names = tuple(plan.get("joint_names", ()))
    positions = tuple(float(value) for value in plan.get("final_joint_positions_rad", ()))
    if len(names) != len(ARM_JOINTS) or len(positions) != len(ARM_JOINTS):
        raise ValueError(f"{target_name} joint vector is incomplete")
    if set(names) != set(ARM_JOINTS) or len(set(names)) != len(names):
        raise ValueError(f"{target_name} joint names do not match the left arm")
    if not all(math.isfinite(position) for position in positions):
        raise ValueError(f"{target_name} target contains a non-finite value")
    by_name = dict(zip(names, positions, strict=True))
    return tuple(by_name[name] for name in ARM_JOINTS)


def load_pregrasp_target(path: Path) -> tuple[float, ...]:
    """Backward-compatible pregrasp loader for existing evidence tests."""
    return load_target(path, "pregrasp")


def arm_limits(calibration_path: Path) -> dict[str, tuple[float, float]]:
    calibration = load_calibration(calibration_path)
    limits: dict[str, tuple[float, float]] = {}
    for index, (name, joint) in enumerate(
        zip(ARM_JOINTS, calibration["joints"][:5], strict=True)
    ):
        endpoint_a = raw_to_urad(calibration, index, joint["minimum_raw"]) / 1e6
        endpoint_b = raw_to_urad(calibration, index, joint["maximum_raw"]) / 1e6
        limits[name] = (min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b))
    return limits


def validate_positions(
    label: str,
    positions: tuple[float, ...],
    limits: dict[str, tuple[float, float]],
) -> None:
    if len(positions) != len(ARM_JOINTS):
        raise ValueError(f"{label} must contain exactly 5 joints")
    for name, position in zip(ARM_JOINTS, positions, strict=True):
        lower, upper = limits[name]
        if not math.isfinite(position) or not lower <= position <= upper:
            raise ValueError(
                f"{label} {name}={position:.6f} outside {lower:.6f}..{upper:.6f}"
            )


def interpolate_segments(
    start: tuple[float, ...],
    target: tuple[float, ...],
    max_joint_step_rad: float,
) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
    if len(start) != len(ARM_JOINTS) or len(target) != len(ARM_JOINTS):
        raise ValueError("start and target must contain exactly 5 joints")
    if not math.isfinite(max_joint_step_rad) or not 0.0 < max_joint_step_rad <= 0.30:
        raise ValueError("max_joint_step_rad must be within (0, 0.30]")
    largest_delta = max(abs(goal - current) for current, goal in zip(start, target))
    segment_count = max(1, math.ceil(largest_delta / max_joint_step_rad))
    waypoints = [start]
    for index in range(1, segment_count):
        waypoints.append(
            tuple(
                current + (goal - current) * index / segment_count
                for current, goal in zip(start, target, strict=True)
            )
        )
    waypoints.append(target)
    return list(zip(waypoints[:-1], waypoints[1:], strict=True))


def joint_goal_constraints(
    target: tuple[float, ...],
    target_name: str = "pregrasp",
) -> Constraints:
    constraints = Constraints()
    constraints.name = f"bounded_{target_name}_segment"
    for name, position in zip(ARM_JOINTS, target, strict=True):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = position
        joint.tolerance_above = JOINT_GOAL_TOLERANCE_RAD
        joint.tolerance_below = JOINT_GOAL_TOLERANCE_RAD
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def build_request(
    start: tuple[float, ...],
    target: tuple[float, ...],
    target_name: str = "pregrasp",
) -> GetMotionPlan.Request:
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.workspace_parameters.header.frame_id = "left_base_link"
    motion.workspace_parameters.min_corner.x = -0.60
    motion.workspace_parameters.min_corner.y = -0.60
    motion.workspace_parameters.min_corner.z = -0.10
    motion.workspace_parameters.max_corner.x = 0.60
    motion.workspace_parameters.max_corner.y = 0.60
    motion.workspace_parameters.max_corner.z = 0.60
    motion.start_state.joint_state.name = list(ARM_JOINTS)
    motion.start_state.joint_state.position = list(start)
    motion.start_state.is_diff = False
    motion.goal_constraints = [joint_goal_constraints(target, target_name)]
    motion.pipeline_id = "ompl"
    motion.planner_id = "RRTConnectkConfigDefault"
    motion.group_name = GROUP_NAME
    motion.num_planning_attempts = 5
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.15
    motion.max_acceleration_scaling_factor = 0.15
    return request


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("MoveIt plan service response timeout")
    result = future.result()
    if result is None:
        raise RuntimeError("MoveIt plan service returned no response")
    return result


def plan_segment(
    client: Any,
    node: Any,
    index: int,
    start,
    target,
    target_name: str = "pregrasp",
) -> dict:
    response = wait_future(
        node,
        client.call_async(build_request(start, target, target_name)),
        timeout_s=8.0,
    ).motion_plan_response
    code = int(response.error_code.val)
    trajectory = response.trajectory.joint_trajectory
    points = trajectory.points
    trajectory_names = tuple(trajectory.joint_names)
    result = {
        "index": index,
        "expected_start_positions_rad": list(start),
        "target_positions_rad": list(target),
        "maximum_joint_delta_rad": max(
            abs(goal - current)
            for current, goal in zip(start, target, strict=True)
        ),
        "moveit_error_code": code,
        "planning_time_s": float(response.planning_time),
        "trajectory_point_count": len(points),
        # Keep the exact MoveIt-returned joint waypoints.  A later robust
        # collision audit must not substitute straight-line interpolation for
        # a path that MoveIt planned around an obstacle.
        "trajectory_joint_names": list(trajectory_names),
        "trajectory_positions_rad": [list(point.positions) for point in points],
        "success": code == MoveItErrorCodes.SUCCESS and bool(points),
    }
    result["joint_goal_tolerance_rad"] = JOINT_GOAL_TOLERANCE_RAD
    bound = JOINT_GOAL_TOLERANCE_RAD * JOINT_GOAL_RESIDUAL_MARGIN
    result["joint_goal_residual_bound_rad"] = bound
    if points:
        if trajectory_names != ARM_JOINTS or any(
            len(point.positions) != len(ARM_JOINTS) for point in points
        ):
            result["success"] = False
            result["trajectory_shape_error"] = (
                "MoveIt trajectory joint names or point widths are inconsistent"
            )
            return result
        final = list(points[-1].positions)
        result["planned_final_positions_rad"] = final
        final_time = points[-1].time_from_start
        result["planned_trajectory_duration_s"] = (
            float(final_time.sec) + float(final_time.nanosec) / 1e9
        )
        # 계획이 success 를 돌려줬다고 목표에 갔다는 뜻이 아니다. 구간 안이면
        # success 다. 실제로 어디에 섰는지 재서 기록하고, 구간 밖이면 거부한다.
        residual = max(
            abs(value - goal)
            for value, goal in zip(final, target, strict=True)
        )
        result["joint_goal_residual_rad"] = residual
        result["within_joint_goal_residual_bound"] = residual <= bound
        result["success"] = result["success"] and residual <= bound
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Split a validated pregrasp or grasp joint solution into bounded "
            "segments and collision-check every segment without execution."
        )
    )
    parser.add_argument("--source-plan", type=Path, default=None)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--start", required=True, type=parse_joint_vector)
    parser.add_argument(
        "--target-name", choices=TARGET_NAMES, default="pregrasp"
    )
    parser.add_argument(
        "--target-joints",
        type=parse_joint_vector,
        default=None,
        help=(
            "explicit joint target instead of a named plan entry. 수렴 "
            "넘겨명령 목표는 어떤 PLAN_ONLY_PASS 파일에도 없다."
        ),
    )
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=DEFAULT_MAX_JOINT_STEP_RAD,
        help="maximum planned checkpoint-to-checkpoint joint delta",
    )
    parser.add_argument(
        "--execution-step-limit",
        type=float,
        default=None,
        help=(
            "maximum fresh measured-state-to-target delta accepted by the "
            "executor; defaults to --max-joint-step and must not be smaller"
        ),
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="required acknowledgement; this tool has no execution Action client",
    )
    args = parser.parse_args()
    if not args.plan_only:
        parser.error("--plan-only is required; no planning request was sent")
    if (args.source_plan is None) == (args.target_joints is None):
        parser.error(
            "give exactly one of --source-plan or --target-joints"
        )
    if args.execution_step_limit is None:
        args.execution_step_limit = args.max_joint_step
    if (
        not math.isfinite(args.execution_step_limit)
        or not args.max_joint_step <= args.execution_step_limit <= 0.30
    ):
        parser.error(
            "--execution-step-limit must be within "
            "[--max-joint-step, 0.30]"
        )
    return args


def main() -> int:
    args = parse_args()
    if args.target_joints is not None:
        args.target_name = EXPLICIT_TARGET_NAME
    try:
        target = (
            args.target_joints
            if args.target_joints is not None
            else load_target(args.source_plan, args.target_name)
        )
        limits = arm_limits(args.calibration)
        validate_positions("start", args.start, limits)
        validate_positions(args.target_name, target, limits)
        candidates = interpolate_segments(args.start, target, args.max_joint_step)
    except Exception as error:
        print(f"{args.target_name.upper()}_SEGMENT_PLAN_FAIL reason={error}")
        return 2

    rclpy.init()
    node = rclpy.create_node(f"so101_moveit_plan_{args.target_name}_segments")
    client = node.create_client(GetMotionPlan, PLAN_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"MoveIt plan service unavailable: {PLAN_SERVICE}")
        segments = [
            plan_segment(client, node, index, start, goal, args.target_name)
            for index, (start, goal) in enumerate(candidates, start=1)
        ]
        passed = all(segment["success"] for segment in segments)
        status_prefix = args.target_name.upper()
        result = {
            "schema_version": 1,
            "status": (
                f"{status_prefix}_SEGMENT_PLAN_ONLY_PASS"
                if passed
                else f"{status_prefix}_SEGMENT_PLAN_ONLY_FAIL"
            ),
            "execution_api_used": False,
            "motion_authorized": False,
            "robot_target_available": False,
            "service": PLAN_SERVICE,
            "group": GROUP_NAME,
            "target_name": args.target_name,
            "joint_names": list(ARM_JOINTS),
            "source_plan": (
                None if args.source_plan is None else str(args.source_plan)
            ),
            "explicit_target_positions_rad": (
                None
                if args.target_joints is None
                else list(args.target_joints)
            ),
            "joint_goal_tolerance_rad": JOINT_GOAL_TOLERANCE_RAD,
            "calibration": str(args.calibration),
            "interpolation_joint_step_rad": args.max_joint_step,
            "max_joint_step_rad": args.execution_step_limit,
            "recommended_execution_duration_s": DEFAULT_DURATION_S,
            "segments": segments,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            f"{result['status']} segments={len(segments)} "
            f"output={args.output.resolve()} execution_api_used=false"
        )
        return 0 if passed else 2
    except Exception as error:
        print(f"{args.target_name.upper()}_SEGMENT_PLAN_FAIL reason={error}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
