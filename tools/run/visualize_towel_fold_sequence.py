#!/usr/bin/env python3
"""Visualize the canonical first and second towel folds in RViz."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.desk_task_runtime import CANONICAL_JOINTS  # noqa: E402


MARKER_TOPIC = "/towel_fold_markers"
TRAJECTORY_TOPIC = "/display_planned_path"
FRAME_ID = "workcell_base_link"
MODEL_ID = "so101_dual_preview"
LEFT_COLOR = (0.10, 0.45, 1.00, 1.00)
RIGHT_COLOR = (1.00, 0.45, 0.05, 1.00)
FIRST_OUTLINE_COLOR = (0.90, 0.90, 0.90, 1.00)
SECOND_OUTLINE_COLOR = (0.20, 0.90, 0.90, 1.00)
FINAL_OUTLINE_COLOR = (0.30, 1.00, 0.35, 1.00)
FOLD_LINE_COLOR = (0.85, 0.20, 1.00, 1.00)
PASS_COLOR = (0.10, 0.90, 0.20, 1.00)
WARNING_COLOR = (1.00, 0.75, 0.05, 1.00)
SUPPORTED_RECORD_KINDS = {
    "canonical_towel_fold_full_fk_diagnostic",
    "towel_bimanual_then_single_task_pose_plan_only",
}


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1.0e9))


def _point(values) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def _color(marker: Marker, rgba) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def _marker(namespace: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


def load_artifact(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(value, dict)
        or value.get("record_kind") not in SUPPORTED_RECORD_KINDS
        or value.get("motion_authorized") is not False
        or value.get("execution_api_used") is not False
        or value.get("motion_commands") != 0
    ):
        raise RuntimeError("artifact is not a supported motion-locked result")
    candidate = value.get("selected_candidate")
    if not isinstance(candidate, dict):
        raise RuntimeError("artifact has no selected canonical candidate")
    if not isinstance(candidate.get("first_fold"), list) or not isinstance(
        candidate.get("second_fold"), list
    ):
        raise RuntimeError("artifact does not contain both fold stages")
    return value


def stage_records(document: dict, stage: str) -> list[tuple[str, dict]]:
    candidate = document["selected_candidate"]
    stages = ("first", "second") if stage == "both" else (stage,)
    result = []
    for name in stages:
        for record in candidate[f"{name}_fold"]:
            result.append((name, record))
    return result


def visible_target_records(
    document: dict, stage: str, include_departure: bool
) -> list[tuple[str, dict]]:
    result = []
    for stage_name, record in stage_records(document, stage):
        if not include_departure and "departure" in str(record.get("name", "")):
            continue
        if record.get("targets"):
            result.append((stage_name, record))
    return result


def _outline_marker(
    namespace: str,
    marker_id: int,
    bounds: list[float],
    z_m: float,
    rgba,
) -> Marker:
    left, right, bottom, top = (float(value) for value in bounds)
    marker = _marker(namespace, marker_id, Marker.LINE_STRIP)
    marker.scale.x = 0.004
    _color(marker, rgba)
    corners = (
        (left, bottom, z_m),
        (right, bottom, z_m),
        (right, top, z_m),
        (left, top, z_m),
        (left, bottom, z_m),
    )
    marker.points = [_point(point) for point in corners]
    return marker


def marker_array(
    document: dict,
    *,
    stage: str = "both",
    include_departure: bool = False,
) -> MarkerArray:
    if stage not in {"first", "second", "both"}:
        raise RuntimeError(f"invalid visualization stage: {stage}")
    candidate = document["selected_candidate"]
    strict = (
        document["record_kind"]
        == "towel_bimanual_then_single_task_pose_plan_only"
    )
    waypoint_color = PASS_COLOR if strict else WARNING_COLOR
    placement = document.get("towel_placement", {})
    bounds = placement.get("bounds_xyxy_m")
    if not isinstance(bounds, list) or len(bounds) != 4:
        raise RuntimeError("artifact has no towel placement bounds")
    table_z = float(placement.get("table_z_m", math.nan))
    if not math.isfinite(table_z):
        raise RuntimeError("artifact table height is invalid")
    records = visible_target_records(document, stage, include_departure)
    if not records:
        raise RuntimeError("selected stage has no visible task targets")

    result = MarkerArray()
    clear = _marker("clear_previous_towel_fold", 0, Marker.SPHERE)
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    z_outline = table_z + 0.003
    if stage in {"first", "both"}:
        result.markers.append(
            _outline_marker(
                "initial_towel", 0, bounds, z_outline, FIRST_OUTLINE_COLOR
            )
        )
    first_bounds = candidate["first_expected_footprint_xyxy_m"]
    if stage in {"second", "both"}:
        result.markers.append(
            _outline_marker(
                "after_first_fold",
                0,
                first_bounds,
                z_outline + 0.002,
                SECOND_OUTLINE_COLOR,
            )
        )
    if stage == "both":
        result.markers.append(
            _outline_marker(
                "final_footprint",
                0,
                candidate["final_expected_footprint_xyxy_m"],
                z_outline + 0.004,
                FINAL_OUTLINE_COLOR,
            )
        )

    initial_left, initial_right, initial_bottom, initial_top = (
        float(value) for value in bounds
    )
    first_fold_line = _marker("first_fold_line", 0, Marker.LINE_STRIP)
    first_fold_line.scale.x = 0.003
    _color(first_fold_line, FOLD_LINE_COLOR)
    first_x = 0.5 * (initial_left + initial_right)
    first_fold_line.points = [
        _point((first_x, initial_bottom, z_outline + 0.006)),
        _point((first_x, initial_top, z_outline + 0.006)),
    ]
    if stage in {"first", "both"}:
        result.markers.append(first_fold_line)

    first_left, first_right, first_bottom, first_top = (
        float(value) for value in first_bounds
    )
    second_fold_line = _marker("second_fold_line", 0, Marker.LINE_STRIP)
    second_fold_line.scale.x = 0.003
    _color(second_fold_line, FOLD_LINE_COLOR)
    second_y = 0.5 * (first_bottom + first_top)
    second_fold_line.points = [
        _point((first_left, second_y, z_outline + 0.008)),
        _point((first_right, second_y, z_outline + 0.008)),
    ]
    if stage in {"second", "both"}:
        result.markers.append(second_fold_line)

    paths: dict[tuple[str, str], list[Point]] = {}
    label_id = 0
    for stage_name, record in records:
        targets = record.get("targets", ())
        for target in targets:
            arm = str(target["arm"])
            paths.setdefault((stage_name, arm), []).append(
                _point(target["xyz_m"])
            )
            sphere = _marker(
                f"{stage_name}_waypoints", label_id * 2 + (arm == "right"), Marker.SPHERE
            )
            sphere.pose.position = _point(target["xyz_m"])
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.016
            _color(sphere, waypoint_color)
            result.markers.append(sphere)
        midpoint = [
            sum(float(target["xyz_m"][axis]) for target in targets) / len(targets)
            for axis in range(3)
        ]
        label = _marker(f"{stage_name}_labels", label_id, Marker.TEXT_VIEW_FACING)
        label.pose.position = _point(
            (midpoint[0], midpoint[1], midpoint[2] + 0.023)
        )
        label.scale.z = 0.015
        _color(label, waypoint_color)
        label.text = str(record["name"])
        result.markers.append(label)
        label_id += 1

    for path_id, ((stage_name, arm), points) in enumerate(sorted(paths.items())):
        line = _marker(f"{stage_name}_{arm}_path", path_id, Marker.LINE_STRIP)
        line.scale.x = 0.006
        _color(line, LEFT_COLOR if arm == "left" else RIGHT_COLOR)
        line.points = points
        result.markers.append(line)

    second_arm = {
        "right": "오른팔",
        "left": "왼팔",
    }.get(str(candidate.get("second_active_arm")), "한 팔")
    second_direction = {
        "right_to_left": "오른쪽→왼쪽",
        "left_to_right": "왼쪽→오른쪽",
    }.get(str(candidate.get("second_direction")), "가로 방향")
    first_direction = {
        "robot_near_to_far": "아래→위",
    }.get(str(candidate.get("first_direction")), "진행 방향 미확정")
    title = _marker("canonical_fold_title", 0, Marker.TEXT_VIEW_FACING)
    title.pose.position = _point(
        (
            0.5 * (initial_left + initial_right),
            initial_top + 0.055,
            table_z + 0.18,
        )
    )
    title.scale.z = 0.025
    _color(title, PASS_COLOR if strict else WARNING_COLOR)
    title.text = (
        f"1차: 양팔 {first_direction} | 2차: {second_arm} {second_direction}"
        + (" | STRICT MOVEIT" if strict else " | FULL-FK ONLY / COLLISION UNCHECKED")
    )
    result.markers.append(title)
    return result


def trajectory_positions(
    document: dict, stage: str
) -> tuple[list[list[float]], bool]:
    strict = document["record_kind"] == "towel_bimanual_then_single_task_pose_plan_only"
    positions: list[list[float]] = []
    if strict:
        joint_index = {name: index for index, name in enumerate(CANONICAL_JOINTS)}
        for _, record in stage_records(document, stage):
            moveit = record.get("moveit")
            if not isinstance(moveit, dict):
                continue
            current = [float(value) for value in moveit["start_positions_rad"]]
            if not positions:
                positions.append(list(current))
            names = [str(name) for name in moveit["trajectory_joint_names"]]
            for arm_point in moveit["trajectory_positions_rad"]:
                for name, value in zip(names, arm_point, strict=True):
                    current[joint_index[name]] = float(value)
                if not positions or current != positions[-1]:
                    positions.append(list(current))
        return positions, True

    clear = [float(value) for value in document["clear_joint_positions_rad"]]
    positions.append(clear)
    for _, record in stage_records(document, stage):
        state = record.get("joint_positions_rad")
        if isinstance(state, list) and state != positions[-1]:
            positions.append([float(value) for value in state])
    return positions, False


def display_trajectory(
    document: dict, stage: str, seconds_per_pose: float
) -> tuple[DisplayTrajectory | None, bool]:
    positions, collision_certified = trajectory_positions(document, stage)
    if not positions:
        return None, collision_certified
    message = DisplayTrajectory()
    message.model_id = MODEL_ID
    message.trajectory_start = RobotState()
    message.trajectory_start.joint_state = JointState(
        name=list(CANONICAL_JOINTS), position=positions[0]
    )
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.header.frame_id = FRAME_ID
    trajectory.joint_trajectory.joint_names = list(CANONICAL_JOINTS)
    for index, state in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = state
        point.time_from_start = _duration(index * seconds_per_pose)
        trajectory.joint_trajectory.points.append(point)
    message.trajectory = [trajectory]
    return message, collision_certified


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path)
    parser.add_argument(
        "--stage", choices=("first", "second", "both"), default="both"
    )
    parser.add_argument("--include-departure", action="store_true")
    parser.add_argument("--seconds-per-pose", type=float, default=0.7)
    parser.add_argument(
        "--publish-ik-animation",
        action="store_true",
        help="animate full-FK-only poses even though transitions are not collision checked",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="0 keeps publishing until Ctrl-C",
    )
    args = parser.parse_args()
    if not args.artifact.is_file():
        parser.error(f"artifact does not exist: {args.artifact}")
    if not math.isfinite(args.seconds_per_pose) or args.seconds_per_pose <= 0.0:
        parser.error("--seconds-per-pose must be finite and positive")
    if not math.isfinite(args.hold_seconds) or args.hold_seconds < 0.0:
        parser.error("--hold-seconds must be finite and nonnegative")
    return args


def main() -> int:
    args = parse_args()
    document = load_artifact(args.artifact)
    markers = marker_array(
        document,
        stage=args.stage,
        include_departure=args.include_departure,
    )
    trajectory, collision_certified = display_trajectory(
        document, args.stage, args.seconds_per_pose
    )
    publish_trajectory = collision_certified or args.publish_ik_animation

    rclpy.init()
    node = Node("canonical_towel_fold_visualizer")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    marker_publisher = node.create_publisher(MarkerArray, MARKER_TOPIC, qos)
    trajectory_publisher = node.create_publisher(
        DisplayTrajectory, TRAJECTORY_TOPIC, qos
    )

    def publish_markers() -> None:
        stamp = node.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = stamp
        marker_publisher.publish(markers)

    marker_timer = node.create_timer(1.0, publish_markers)
    publish_markers()
    trajectory_published = False

    def publish_trajectory_when_ready() -> None:
        nonlocal trajectory_published
        if (
            trajectory_published
            or trajectory is None
            or not publish_trajectory
            or trajectory_publisher.get_subscription_count() == 0
        ):
            return
        trajectory_publisher.publish(trajectory)
        trajectory_published = True
        print(
            "TOWEL_FOLD_RVIZ_TRAJECTORY_PUBLISHED "
            f"collision_certified={collision_certified}"
        )

    trajectory_timer = node.create_timer(0.2, publish_trajectory_when_ready)
    print(
        "TOWEL_FOLD_RVIZ_READY "
        f"stage={args.stage} collision_certified={collision_certified} "
        f"ik_animation_enabled={publish_trajectory} motion_commands=0"
    )
    if not collision_certified and not args.publish_ik_animation:
        print(
            "TOWEL_FOLD_RVIZ_MARKERS_ONLY full-FK pose transitions are not "
            "collision checked; pass --publish-ik-animation to inspect them"
        )
    try:
        if args.hold_seconds == 0.0:
            rclpy.spin(node)
        else:
            deadline = node.get_clock().now().nanoseconds / 1.0e9 + args.hold_seconds
            while rclpy.ok() and node.get_clock().now().nanoseconds / 1.0e9 < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        marker_timer.cancel()
        trajectory_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, json.JSONDecodeError, RuntimeError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from None
