#!/usr/bin/env python3
"""Apply one read-only ROS bimanual observation to the loaded Isaac USD.

Run this file in Isaac Sim 6.0.1's Script Editor with the timeline stopped.
It subscribes once to ``/bimanual_joint_states`` and changes only the USD
session layer.  It never publishes, calls a service/action, or commands either
physical arm.
"""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Iterable, Sequence


SIMULATION_ONLY = True
MOTION_AUTHORIZED = False
HARDWARE_SYNCHRONOUS = False
BIMANUAL_TOPIC = "/bimanual_joint_states"
SNAPSHOT_TIMEOUT_S = 10.0
# The accepted q0 settle tolerance is 10 raw counts (about 0.88 degrees).
# Preserve the measured value rather than clamping it, but reject excursions
# larger than this small mechanical/quantization allowance past a URDF limit.
IMPORTED_LIMIT_TOLERANCE_DEG = 1.0

EXPECTED_JOINT_NAMES = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
    "left_gripper_joint",
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
    "right_gripper_joint",
)

ARM_ROOT_LINK_SUFFIX = {
    "left": "/left_mount_arm_base_link",
    "right": "/right_mount_arm_base_link",
}


def validate_snapshot(
    names: Sequence[str],
    positions: Sequence[float],
) -> dict[str, float]:
    """Validate the exact R4 topic contract and retain ROS radians verbatim."""

    actual_names = tuple(str(name) for name in names)
    if actual_names != EXPECTED_JOINT_NAMES:
        raise ValueError(
            "unexpected /bimanual_joint_states identity/order: "
            f"expected={EXPECTED_JOINT_NAMES!r} actual={actual_names!r}"
        )
    actual_positions = tuple(float(value) for value in positions)
    if len(actual_positions) != len(EXPECTED_JOINT_NAMES):
        raise ValueError("bimanual snapshot must contain exactly 12 positions")
    if not all(math.isfinite(value) for value in actual_positions):
        raise ValueError("bimanual snapshot positions must be finite radians")
    return dict(zip(EXPECTED_JOINT_NAMES, actual_positions, strict=True))


def map_joint_paths(
    joint_paths: Iterable[object],
) -> dict[str, str]:
    """Map all 12 project joint names to unique imported USD joint paths."""

    paths = tuple(str(path) for path in joint_paths)
    mapped: dict[str, str] = {}
    for name in EXPECTED_JOINT_NAMES:
        matches = tuple(path for path in paths if path.endswith(f"/{name}"))
        if len(matches) != 1:
            raise RuntimeError(
                f"expected exactly one imported joint named {name}, "
                f"found {len(matches)}"
            )
        mapped[name] = matches[0]
    if len(set(mapped.values())) != len(EXPECTED_JOINT_NAMES):
        raise RuntimeError("imported bimanual joint paths are not unique")
    return mapped


def branch_first_link_paths(
    link_paths: Sequence[object],
    arm: str,
) -> list[object]:
    """Put one branch root first for Isaac 6.0.1 stopped-timeline FK.

    The imported fixed-base workcell has two disjoint rigid-body branches
    connected to the robot root.  Robot Poser's kinematic tree starts from the
    first ``robotLinks`` target, so the branches must be applied separately.
    """

    if arm not in ARM_ROOT_LINK_SUFFIX:
        raise ValueError(f"unsupported arm: {arm}")
    suffix = ARM_ROOT_LINK_SUFFIX[arm]
    matches = [path for path in link_paths if str(path).endswith(suffix)]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {arm} branch root ending {suffix}, "
            f"found {len(matches)}"
        )
    root = matches[0]
    return [root, *(path for path in link_paths if path != root)]


def _validate_imported_joint_limits(stage, path_by_name, positions) -> None:
    """Reject a ROS value outside the imported revolute-joint limit."""

    from pxr import UsdPhysics

    for name, value_rad in positions.items():
        prim = stage.GetPrimAtPath(path_by_name[name])
        if not prim or not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
            raise RuntimeError(f"imported joint is not revolute: {name}")
        joint = UsdPhysics.RevoluteJoint(prim)
        lower = joint.GetLowerLimitAttr().Get()
        upper = joint.GetUpperLimitAttr().Get()
        if lower is None or upper is None:
            raise RuntimeError(f"imported joint has no limits: {name}")
        value_deg = math.degrees(value_rad)
        if (
            value_deg < float(lower) - IMPORTED_LIMIT_TOLERANCE_DEG
            or value_deg > float(upper) + IMPORTED_LIMIT_TOLERANCE_DEG
        ):
            raise ValueError(
                f"snapshot exceeds imported limit for {name}: "
                f"value={value_deg:.3f}deg limits=[{float(lower):.3f}, "
                f"{float(upper):.3f}]deg"
            )


def _apply_two_branches(stage, robot, path_by_name, positions, apply_joint_state) -> None:
    """Apply left/right FK independently while authoring only the session layer."""

    links = robot.GetRelationship("isaac:physics:robotLinks")
    original_paths = list(links.GetTargets())
    if not original_paths:
        raise RuntimeError("imported robot has no isaac:physics:robotLinks")

    previous_edit_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(stage.GetSessionLayer())
        try:
            for arm in ("left", "right"):
                links.SetTargets(branch_first_link_paths(original_paths, arm))
                commands = {
                    path_by_name[name]: positions[name]
                    for name in EXPECTED_JOINT_NAMES
                    if name.startswith(f"{arm}_")
                }
                apply_joint_state(stage, robot, commands)
        finally:
            links.SetTargets(original_paths)
    finally:
        stage.SetEditTarget(previous_edit_target)


async def apply_one_snapshot() -> None:
    """Wait for one valid ROS sample and apply it to the loaded USD once."""

    import omni.kit.app
    import omni.timeline
    import omni.usd

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("stop the Isaac timeline before applying the snapshot")

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.robot.poser", True)
    from isaacsim.robot.poser import apply_joint_state, validate_robot_schema

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no USD stage is open")
    robots = [prim for prim in stage.Traverse() if validate_robot_schema(prim)]
    if len(robots) != 1:
        raise RuntimeError(f"expected exactly one imported robot, found {len(robots)}")
    robot = robots[0]
    joint_paths = robot.GetRelationship("isaac:physics:robotJoints").GetTargets()
    path_by_name = map_joint_paths(joint_paths)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init(args=None)
    node = rclpy.create_node(
        f"isaac_bimanual_snapshot_{time.monotonic_ns()}"
    )
    received: list[dict[str, float]] = []
    callback_error: list[Exception] = []

    def on_snapshot(message: JointState) -> None:
        if received or callback_error:
            return
        try:
            received.append(validate_snapshot(message.name, message.position))
        except Exception as error:
            callback_error.append(error)

    subscription = node.create_subscription(
        JointState,
        BIMANUAL_TOPIC,
        on_snapshot,
        qos_profile_sensor_data,
    )
    print(
        "ISAAC_BIMANUAL_SNAPSHOT_WAITING "
        f"topic={BIMANUAL_TOPIC} timeout_s={SNAPSHOT_TIMEOUT_S:.1f} "
        "SIMULATION_ONLY motion_authorized=false"
    )
    started = time.monotonic()
    try:
        while not received and not callback_error:
            if time.monotonic() - started >= SNAPSHOT_TIMEOUT_S:
                raise TimeoutError(
                    f"no valid sample received on {BIMANUAL_TOPIC} within "
                    f"{SNAPSHOT_TIMEOUT_S:.1f}s"
                )
            rclpy.spin_once(node, timeout_sec=0.0)
            await omni.kit.app.get_app().next_update_async()
        if callback_error:
            raise RuntimeError(f"R4 snapshot validation failed: {callback_error[0]}")
        if timeline.is_playing():
            raise RuntimeError("timeline started while waiting; snapshot was not applied")

        positions = received[0]
        _validate_imported_joint_limits(stage, path_by_name, positions)
        _apply_two_branches(
            stage,
            robot,
            path_by_name,
            positions,
            apply_joint_state,
        )
        degrees = " ".join(
            f"{name}={math.degrees(positions[name]):+.2f}deg"
            for name in EXPECTED_JOINT_NAMES
        )
        print(
            "ISAAC_BIMANUAL_SNAPSHOT_APPLIED joints=12 apply_count=1 "
            "SIMULATION_ONLY motion_authorized=false "
            "hardware_synchronous=false source=r4_read_only "
            f"{degrees}"
        )
    finally:
        del subscription
        node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    """Schedule the one-shot coroutine without blocking Isaac's UI loop."""

    asyncio.ensure_future(apply_one_snapshot())


if __name__ == "__main__":
    main()
