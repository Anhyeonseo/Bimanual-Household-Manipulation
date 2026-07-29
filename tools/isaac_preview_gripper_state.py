#!/usr/bin/env python3
"""Apply one simulation-only gripper state in Isaac Sim 6.0.1.

Run in Isaac Script Editor with the timeline stopped. The script only edits
the loaded USD stage through Robot Poser; it never publishes ROS commands.
"""

from __future__ import annotations

import math


SIMULATION_ONLY = True
MOTION_AUTHORIZED = False

PROJECT_GRIPPER_RAD = {
    "closed": 0.0,
    "open": 1.91986,
}
PROJECT_TO_ISAAC_OFFSET_RAD = math.radians(10.0)


def project_to_isaac_gripper(value: float) -> float:
    return value - PROJECT_TO_ISAAC_OFFSET_RAD


def requested_isaac_position(state_name: str) -> float:
    if state_name not in PROJECT_GRIPPER_RAD:
        raise ValueError("state must be 'open' or 'closed'")
    return project_to_isaac_gripper(PROJECT_GRIPPER_RAD[state_name])


def main(state_name: str) -> None:
    import omni.kit.app
    import omni.usd

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.robot.poser", True)

    from isaacsim.robot.poser import apply_joint_state, validate_robot_schema

    stage = omni.usd.get_context().get_stage()
    robots = [prim for prim in stage.Traverse() if validate_robot_schema(prim)]
    if len(robots) != 1:
        raise RuntimeError(f"Expected exactly 1 robot, found {len(robots)}")

    robot = robots[0]
    joint_paths = robot.GetRelationship("isaac:physics:robotJoints").GetTargets()
    matches = [path for path in joint_paths if str(path).endswith("/gripper")]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one Isaac gripper joint, found {len(matches)}")

    isaac_value = requested_isaac_position(state_name)
    apply_joint_state(stage, robot, {str(matches[0]): isaac_value})
    print(
        "ISAAC_GRIPPER_PREVIEW_APPLIED "
        f"state={state_name} "
        f"project_rad={PROJECT_GRIPPER_RAD[state_name]:.6f} "
        f"isaac_deg={math.degrees(isaac_value):.3f} "
        "SIMULATION_ONLY motion_authorized=false"
    )


if __name__ == "__main__":
    requested_state = globals().get("GRIPPER_PREVIEW_STATE")
    if requested_state is None:
        raise RuntimeError(
            "Set GRIPPER_PREVIEW_STATE to 'open' or 'closed' before exec"
        )
    main(str(requested_state))
