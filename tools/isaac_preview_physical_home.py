#!/usr/bin/env python3
"""Preview the accepted physical Home baseline in Isaac Sim 6.0.1.

Run this file only in Isaac Sim's Script Editor with the timeline stopped.
The upstream-model values record how the canonical q0 was established. They
are simulation-only metadata, never hardware commands.
"""

from __future__ import annotations

import math


SIMULATION_ONLY = True
MOTION_AUTHORIZED = False

# Initially accepted on 2026-07-26 from top/side/front photographs and an
# interactive Isaac preview. Wrist flex was refined on 2026-07-30 by external
# eye-to-hand metrology. These are the upstream model coordinates absorbed
# into the model joint origins; canonical project q0 remains zero.
UPSTREAM_POSE_ABSORBED_INTO_Q0_RAD = {
    "left_base_joint": 0.0,
    "left_shoulder_joint": math.radians(90.0),
    "left_elbow_joint": math.radians(-55.0),
    "left_wrist_flex_joint": math.radians(-64.898281239),
    "left_wrist_roll_joint": math.radians(-90.0),
}

# Keep this local because Isaac Script Editor does not load the ROS workspace
# Python path. These names/signs match so101_isaac_bridge.mapping.JOINT_SPECS.
PROJECT_TO_ISAAC = {
    "left_base_joint": ("shoulder_pan", -1.0, 0.0),
    "left_shoulder_joint": ("shoulder_lift", -1.0, 0.0),
    "left_elbow_joint": ("elbow_flex", -1.0, 0.0),
    "left_wrist_flex_joint": ("wrist_flex", -1.0, 0.0),
    "left_wrist_roll_joint": ("wrist_roll", -1.0, 0.0),
}


def base_first_link_paths(link_paths):
    """Return robot link paths with the unique base_link first.

    Isaac Sim 6.0.1's kinematic tree builder uses the first robotLinks target
    as the tree root. URDF import may author the relationship in reverse
    order, which makes stopped-timeline FK pull the rigid links apart.
    """
    base_paths = [
        path for path in link_paths if str(path).endswith("/base_link")
    ]
    if len(base_paths) != 1:
        raise RuntimeError(
            f"Expected exactly one base_link, found {len(base_paths)}"
        )
    base_path = base_paths[0]
    return [base_path, *(path for path in link_paths if path != base_path)]


def upstream_registration_pose() -> dict[str, float]:
    """Return the upstream pose used to establish canonical q0."""
    return dict(UPSTREAM_POSE_ABSORBED_INTO_Q0_RAD)


def upstream_registration_pose_isaac() -> dict[str, float]:
    """Convert the historical registration pose to upstream Isaac names."""
    result = {}
    for project_name, project_value in upstream_registration_pose().items():
        isaac_name, sign, offset = PROJECT_TO_ISAAC[project_name]
        result[isaac_name] = (project_value - offset) / sign
    return result


def main() -> None:
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
    link_relationship = robot.GetRelationship("isaac:physics:robotLinks")
    link_paths = list(link_relationship.GetTargets())
    ordered_link_paths = base_first_link_paths(link_paths)
    if ordered_link_paths != link_paths:
        previous_edit_target = stage.GetEditTarget()
        try:
            # Keep this compatibility correction in the anonymous session
            # layer so previewing a pose never dirties the USD asset.
            stage.SetEditTarget(stage.GetSessionLayer())
            link_relationship.SetTargets(ordered_link_paths)
        finally:
            stage.SetEditTarget(previous_edit_target)

    joint_paths = robot.GetRelationship("isaac:physics:robotJoints").GetTargets()
    project_candidate = {
        project_name: 0.0 for project_name in PROJECT_TO_ISAAC
    }
    candidate = {
        isaac_name: 0.0
        for isaac_name, _sign, _offset in PROJECT_TO_ISAAC.values()
    }
    commands: dict[str, float] = {}
    for joint_name, value in candidate.items():
        matches = [
            path
            for path in joint_paths
            if str(path).endswith(f"/{joint_name}")
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"Expected one Isaac joint for {joint_name}, found {len(matches)}"
            )
        commands[str(matches[0])] = value

    apply_joint_state(stage, robot, commands)
    degrees = ", ".join(
        f"{name}={math.degrees(value):+.1f}deg"
        for name, value in project_candidate.items()
    )
    print(
        "CANONICAL_Q0_PREVIEW_APPLIED "
        f"SIMULATION_ONLY motion_authorized=false "
        f"KINEMATIC_ROOT=base_link {degrees}"
    )
    absorbed = ", ".join(
        f"{name}={math.degrees(value):+.1f}deg"
        for name, value in upstream_registration_pose().items()
    )
    print(f"UPSTREAM_POSE_ABSORBED_INTO_Q0 {absorbed}")


if __name__ == "__main__":
    main()
