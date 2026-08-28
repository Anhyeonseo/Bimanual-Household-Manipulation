"""Pure helpers for the Isaac S0 transition-collision gate."""

from __future__ import annotations

import math
import re
from typing import Mapping, Sequence


MAXIMUM_JOINT_STEP_RAD = 0.02
MAXIMUM_SHALLOW_MESH_PENETRATION_M = 0.004
_ENV_PREFIX = re.compile(r"^/World/envs/env_\d+")

# Neighboring links share joint hardware and are expected to overlap at the
# joint boundary. PhysX contacts between any other robot links are forbidden.
ADJACENT_ROBOT_LINKS = frozenset(
    {
        frozenset(("workcell_base_link", "left_base_link")),
        frozenset(("workcell_base_link", "left_shoulder_link")),
        frozenset(("left_base_link", "left_shoulder_link")),
        frozenset(("left_shoulder_link", "left_upper_arm_link")),
        frozenset(("left_upper_arm_link", "left_lower_arm_link")),
        frozenset(("left_lower_arm_link", "left_wrist_link")),
        frozenset(("left_wrist_link", "left_gripper_link")),
        frozenset(("left_gripper_link", "left_moving_jaw_link")),
        frozenset(("workcell_base_link", "right_base_link")),
        frozenset(("workcell_base_link", "right_shoulder_link")),
        frozenset(("right_base_link", "right_shoulder_link")),
        frozenset(("right_shoulder_link", "right_upper_arm_link")),
        frozenset(("right_upper_arm_link", "right_lower_arm_link")),
        frozenset(("right_lower_arm_link", "right_wrist_link")),
        frozenset(("right_wrist_link", "right_gripper_link")),
        frozenset(("right_gripper_link", "right_moving_jaw_link")),
    }
)

# These pairs are explicitly disabled as ``Never`` in so101_dual.srdf. Fixed
# base links are merged into workcell_base_link by the Isaac URDF importer.
SRDF_DISABLED_ROBOT_LINKS = frozenset(
    {
        frozenset(("workcell_base_link", "left_upper_arm_link")),
        frozenset(("left_lower_arm_link", "left_gripper_link")),
        frozenset(("left_lower_arm_link", "left_moving_jaw_link")),
        frozenset(("left_wrist_link", "left_moving_jaw_link")),
        frozenset(("workcell_base_link", "right_upper_arm_link")),
        frozenset(("right_lower_arm_link", "right_gripper_link")),
        frozenset(("right_lower_arm_link", "right_moving_jaw_link")),
        frozenset(("right_wrist_link", "right_moving_jaw_link")),
    }
)

# The strict MoveIt replay accepts only these measured collision-mesh
# approximations, and only up to a 4 mm penetration bound. They are distinct
# from SRDF ``Never`` pairs and must therefore remain depth audited in PhysX.
SHALLOW_MESH_CONTACT_EXCEPTIONS = frozenset(
    {
        frozenset(("left_gripper_link", "left_shoulder_link")),
        frozenset(("left_moving_jaw_link", "left_shoulder_link")),
        frozenset(("right_gripper_link", "right_shoulder_link")),
        frozenset(("right_lower_arm_link", "right_shoulder_link")),
        frozenset(("right_moving_jaw_link", "right_shoulder_link")),
    }
)


def normalized_prim_path(path: str) -> str:
    """Remove the vectorized environment prefix from a USD prim path."""
    return _ENV_PREFIX.sub("{ENV}", path)


def robot_link_from_actor(path: str) -> str | None:
    """Return the rigid-body link name from a cloned Robot actor path."""
    marker = "/Robot/"
    if marker not in path:
        return None
    suffix = path.split(marker, 1)[1]
    # Tensor paths are direct, while low-level PhysX reports preserve the
    # converted URDF's nested fixed-joint hierarchy below ``Robot/Geometry``.
    links = [segment for segment in suffix.split("/") if segment.endswith("_link")]
    return links[-1] if links else None


def classify_contact_pair(actor0: str, actor1: str) -> str:
    """Classify a PhysX actor pair for the rigid-proxy collision gate."""
    link0 = robot_link_from_actor(actor0)
    link1 = robot_link_from_actor(actor1)
    has_table0 = "/Table" in actor0
    has_table1 = "/Table" in actor1
    has_proxy0 = "/TowelProxy" in actor0
    has_proxy1 = "/TowelProxy" in actor1
    has_probe0 = "/ContactProbe" in actor0
    has_probe1 = "/ContactProbe" in actor1

    if link0 is not None and link1 is not None:
        if link0 == link1:
            return "allowed_same_robot_body"
        pair = frozenset((link0, link1))
        if pair in ADJACENT_ROBOT_LINKS:
            return "allowed_adjacent_robot_links"
        if pair in SRDF_DISABLED_ROBOT_LINKS:
            return "allowed_srdf_disabled_robot_links"
        if pair in SHALLOW_MESH_CONTACT_EXCEPTIONS:
            return "bounded_shallow_mesh_contact"
        return "forbidden_robot_self_collision"
    if (link0 is not None and has_table1) or (link1 is not None and has_table0):
        robot_link = link0 if link0 is not None else link1
        if robot_link == "workcell_base_link":
            return "allowed_robot_mount_table_contact"
        return "forbidden_robot_table_collision"
    if (link0 is not None and has_proxy1) or (link1 is not None and has_proxy0):
        return "excluded_robot_proxy_contact"
    if (link0 is not None and has_probe1) or (link1 is not None and has_probe0):
        return "sensor_liveness_robot_probe_contact"
    if (has_probe0 and has_table1) or (has_probe1 and has_table0):
        return "sensor_liveness_probe_table_contact"
    return "excluded_out_of_scope_contact"


def classify_contact_separation(
    actor0: str,
    actor1: str,
    separation_m: float,
) -> str:
    """Classify one PhysX point under the pinned strict-MoveIt contract.

    Positive PhysX separation is contact-offset proximity, not penetration.
    The known shallow-mesh pairs are owned by the source MoveIt/FCL audit;
    their importer-specific PhysX depth remains diagnostic evidence.
    """
    category = classify_contact_pair(actor0, actor1)
    if not math.isfinite(separation_m):
        return "forbidden_nonfinite_contact_separation"
    if category == "bounded_shallow_mesh_contact":
        return "accepted_moveit_bounded_shallow_mesh_contact"
    if category.startswith("forbidden_") and separation_m > 0.0:
        return "allowed_physx_contact_offset_proximity"
    return category


def interpolation_step_count(
    start: Sequence[float],
    target: Sequence[float],
    *,
    maximum_joint_step_rad: float = MAXIMUM_JOINT_STEP_RAD,
) -> int:
    """Return the samples needed to bound every joint increment."""
    if len(start) != len(target) or not start:
        raise ValueError("start and target must be equally sized non-empty vectors")
    if not math.isfinite(maximum_joint_step_rad) or maximum_joint_step_rad <= 0.0:
        raise ValueError("maximum_joint_step_rad must be finite and positive")
    values = [abs(float(end) - float(begin)) for begin, end in zip(start, target)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("joint vectors must contain only finite values")
    return max(1, math.ceil(max(values) / maximum_joint_step_rad))


def expanded_phase_waypoints(
    phase: Mapping[str, object],
    *,
    canonical_joint_names: Sequence[str],
    current_positions_rad: Sequence[float],
    continuity_tolerance_rad: float = 1.0e-3,
    terminal_tolerance_rad: float = 1.0e-3,
) -> list[list[float]]:
    """Expand a strict partial-joint MoveIt trajectory into full joint states.

    MoveIt records the final trajectory state inside its configured goal
    tolerance instead of copying the requested target bit-for-bit.  The two
    tolerances remain explicit and bounded so a stale or discontinuous replay
    cannot be hidden by that normal planner residual.
    """
    if len(canonical_joint_names) != len(current_positions_rad):
        raise ValueError("canonical names and current positions must have equal size")
    target = phase.get("joint_positions_rad")
    if not isinstance(target, list) or len(target) != len(canonical_joint_names):
        raise ValueError("phase target is not canonical")
    points = phase.get("trajectory_positions_rad")
    if points is None:
        return [[float(value) for value in target]]
    names = phase.get("trajectory_joint_names")
    start = phase.get("start_positions_rad")
    if (
        not isinstance(names, list)
        or not names
        or not isinstance(start, list)
        or len(start) != len(canonical_joint_names)
        or not isinstance(points, list)
        or not points
    ):
        raise ValueError("strict phase trajectory metadata is incomplete")
    if max(
        abs(float(actual) - float(expected))
        for actual, expected in zip(current_positions_rad, start, strict=True)
    ) > continuity_tolerance_rad:
        raise ValueError("strict phase start is discontinuous")
    indices = []
    for name in names:
        if name not in canonical_joint_names:
            raise ValueError(f"strict phase has unknown joint: {name}")
        indices.append(canonical_joint_names.index(name))
    result = []
    for point in points:
        if not isinstance(point, list) or len(point) != len(indices):
            raise ValueError("strict trajectory point width is invalid")
        full = [float(value) for value in start]
        for index, value in zip(indices, point, strict=True):
            full[index] = float(value)
        result.append(full)
    if max(
        abs(float(actual) - float(expected))
        for actual, expected in zip(result[-1], target, strict=True)
    ) > terminal_tolerance_rad:
        raise ValueError("strict trajectory terminal state does not match phase target")
    return result
