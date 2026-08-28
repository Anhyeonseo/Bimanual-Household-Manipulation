"""Fail-closed host preflight for the R2 Isaac Lab S0 rigid-proxy replay."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any

import yaml

from tools.lib.towel_isaac_collision import (
    MAXIMUM_SHALLOW_MESH_PENETRATION_M,
    SHALLOW_MESH_CONTACT_EXCEPTIONS,
)


class TowelIsaacS0Error(RuntimeError):
    """The pinned S0 inputs are stale, unsafe, or semantically incompatible."""


S0_STATUS = "S0_HOST_CONTRACT_READY_SIMULATION_NOT_RUN"
CANONICAL_JOINTS = [
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
]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TowelIsaacS0Error(f"{name} must be an object")
    return value


def _load_json(path: Path) -> dict[str, Any]:
    return _mapping(json.loads(path.read_text(encoding="utf-8")), str(path))


def _load_yaml(path: Path) -> dict[str, Any]:
    return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), str(path))


def _digest_json(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _canonical_phases(
    document: dict[str, Any], fold: str
) -> list[dict[str, Any]]:
    selected = _mapping(document.get("selected_candidate"), "selected_candidate")
    records = selected.get(fold)
    if not isinstance(records, list) or not records:
        raise TowelIsaacS0Error(f"source has no {fold} records")
    result = []
    for record in records:
        item = _mapping(record, f"{fold} phase")
        positions = item.get("joint_positions_rad")
        moveit: dict[str, Any] | None = None
        if positions is None:
            moveit = _mapping(item.get("moveit"), f"{fold} moveit")
            positions = moveit.get("target_positions_rad")
        if (
            not isinstance(positions, list)
            or len(positions) != len(CANONICAL_JOINTS)
        ):
            raise TowelIsaacS0Error(f"{fold} phase has no canonical joint target")
        phase = {
            "name": str(item.get("name", "")),
            "joint_positions_rad": [float(value) for value in positions],
        }
        if moveit is not None:
            start = moveit.get("start_positions_rad")
            names = moveit.get("trajectory_joint_names")
            points = moveit.get("trajectory_positions_rad")
            if (
                not isinstance(start, list)
                or len(start) != len(CANONICAL_JOINTS)
                or not isinstance(names, list)
                or not names
                or len(set(names)) != len(names)
                or any(name not in CANONICAL_JOINTS for name in names)
                or not isinstance(points, list)
                or not points
                or any(
                    not isinstance(point, list) or len(point) != len(names)
                    for point in points
                )
            ):
                raise TowelIsaacS0Error(
                    f"{fold} phase has no replayable strict trajectory"
                )
            phase.update(
                {
                    "start_positions_rad": [float(value) for value in start],
                    "trajectory_joint_names": [str(name) for name in names],
                    "trajectory_positions_rad": [
                        [float(value) for value in point] for point in points
                    ],
                }
            )
        result.append(phase)
    return result


def build_s0_vectorized_manifest(
    document: dict[str, Any],
    *,
    source_sha256: str,
    environment_count: int,
    seed: int,
) -> dict[str, Any]:
    """Build the Isaac S0 runner manifest from strict MoveIt plan-only evidence."""
    if environment_count < 2 or not isinstance(seed, int):
        raise TowelIsaacS0Error("S0 requires at least two deterministic environments")
    strict_source = (
        document.get("record_kind")
        == "towel_bimanual_then_single_task_pose_plan_only"
    )
    if not strict_source:
        raise TowelIsaacS0Error("S0 requires strict MoveIt plan-only evidence")
    expected_status = "TOWEL_BIMANUAL_THEN_SINGLE_TASK_POSE_PLAN_ONLY_PASS"
    if (
        document.get("status") != expected_status
        or document.get("motion_authorized") is not False
        or document.get("execution_api_used") is not False
        or document.get("motion_commands") != 0
    ):
        raise TowelIsaacS0Error("source is not motion-locked canonical evidence")
    selected = _mapping(document.get("selected_candidate"), "selected_candidate")
    if (
        selected.get("candidate_id")
        != "first_bimanual_robot_near_to_far__second_right_right_to_left_edge_midpoint"
        or selected.get("first_axis") != "x"
        or selected.get("first_direction") != "robot_near_to_far"
        or selected.get("first_arm_assignment") != "left_high_y_right_low_y"
        or selected.get("second_axis") != "y"
        or selected.get("second_direction") != "right_to_left"
        or selected.get("second_active_arm") != "right"
    ):
        raise TowelIsaacS0Error("source fold strategy is not the approved topology")
    source_entries = _mapping(document.get("sources"), "sources")
    urdf_entry = _mapping(
        source_entries.get("registered_urdf" if strict_source else "urdf"),
        "source URDF",
    )
    urdf_path = Path(str(urdf_entry.get("path", ""))).resolve()
    urdf_sha256 = str(urdf_entry.get("sha256", ""))
    if not urdf_path.is_file() or _sha256(urdf_path) != urdf_sha256:
        raise TowelIsaacS0Error("source URDF is missing or stale")
    first = _canonical_phases(document, "first_fold")
    second = _canonical_phases(document, "second_fold")
    replay = {"first_fold": first, "second_fold": second}
    placement = _mapping(document.get("towel_placement"), "towel_placement")
    bounds = placement.get("bounds_xyxy_m")
    if not isinstance(bounds, list) or len(bounds) != 4:
        raise TowelIsaacS0Error("source towel placement is invalid")
    left, right, bottom, top = (float(value) for value in bounds)
    table_z = float(placement.get("table_z_m"))
    proxy_size = [0.3, 0.3, 0.003]
    proxy_pose = [
        [
            0.5 * (left + right),
            0.5 * (bottom + top),
            table_z + 0.5 * proxy_size[2],
            0.0,
        ]
        for _ in range(environment_count)
    ]
    clear = (
        document.get("clear_joint_positions_rad")
        if not strict_source
        else document["selected_candidate"]["first_fold"][0]["moveit"][
            "start_positions_rad"
        ]
    )
    if not isinstance(clear, list) or len(clear) != 12:
        raise TowelIsaacS0Error("source clear state is invalid")
    strict_validation = _mapping(
        selected.get("strict_validation"), "strict MoveIt validation"
    )
    planning_scene = _mapping(document.get("planning_scene"), "planning_scene")
    expected_exceptions = sorted(
        (sorted(pair) for pair in SHALLOW_MESH_CONTACT_EXCEPTIONS),
        key=lambda pair: tuple(pair),
    )
    actual_exceptions = planning_scene.get(
        "temporary_shallow_mesh_contact_exceptions"
    )
    maximum_mesh_limit = float(
        strict_validation.get("maximum_accepted_mesh_contact_depth_limit_m")
    )
    maximum_mesh_depth = float(
        strict_validation.get("maximum_accepted_mesh_contact_depth_m")
    )
    if (
        actual_exceptions != expected_exceptions
        or strict_validation.get("unapproved_contact_count") != 0
        or maximum_mesh_limit != MAXIMUM_SHALLOW_MESH_PENETRATION_M
        or maximum_mesh_depth > maximum_mesh_limit
    ):
        raise TowelIsaacS0Error("strict MoveIt collision contract is unsafe")
    moveit_collision_contract = {
        "authority": "strict_moveit_fcl_source_plan",
        "shallow_mesh_contact_exceptions": actual_exceptions,
        "maximum_accepted_mesh_contact_depth_limit_m": maximum_mesh_limit,
        "maximum_accepted_mesh_contact_depth_m": maximum_mesh_depth,
        "unapproved_contact_count": 0,
    }
    source_entries = _mapping(document.get("sources"), "sources")
    worktable_entry = _mapping(source_entries.get("worktable"), "source worktable")
    worktable_path = Path(str(worktable_entry.get("path", ""))).resolve()
    worktable_sha256 = str(worktable_entry.get("sha256", ""))
    if (
        not worktable_path.is_file()
        or _sha256(worktable_path) != worktable_sha256
    ):
        raise TowelIsaacS0Error("source worktable is missing or stale")
    worktable = _load_yaml(worktable_path)
    board = _mapping(worktable.get("board"), "source worktable board")
    span = board.get("calibrated_span_m")
    origin = board.get("origin_in_left_base_link_xy_m")
    if (
        not isinstance(span, list)
        or len(span) != 2
        or not isinstance(origin, list)
        or len(origin) != 2
    ):
        raise TowelIsaacS0Error("source worktable geometry is invalid")
    table_thickness = 0.020
    table_z = float(board.get("table_z_in_left_base_link_m"))
    worktable_size = [float(span[0]), float(span[1]), table_thickness]
    worktable_pose = [
        float(origin[0]) + 0.5 * worktable_size[0],
        float(origin[1]) + 0.5 * worktable_size[1],
        table_z - 0.5 * table_thickness,
    ]
    worktable_geometry = {
        "path": str(worktable_path),
        "sha256": worktable_sha256,
        "size_xyz_m": worktable_size,
        "pose_xyz_m": worktable_pose,
    }
    identity = {
        "source_sha256": source_sha256,
        "canonical_replay_sha256": _digest_json(replay),
        "reset_batch_sha256": _digest_json(proxy_pose),
        "worktable_geometry_sha256": _digest_json(worktable_geometry),
        "moveit_collision_contract_sha256": _digest_json(
            moveit_collision_contract
        ),
    }
    return {
        "schema_version": 1,
        "record_kind": "towel_isaac_s0_host_manifest",
        "status": S0_STATUS,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": environment_count,
        "base_seed": seed,
        "environment_seeds": [seed + index for index in range(environment_count)],
        "joint_names": list(CANONICAL_JOINTS),
        "urdf_path": str(urdf_path),
        "urdf_sha256": urdf_sha256,
        "clear_joint_positions_rad": [float(value) for value in clear],
        "proxy_size_xyz_m": proxy_size,
        "rigid_proxy_pose_xyz_yaw_rad": proxy_pose,
        "worktable_geometry": worktable_geometry,
        "moveit_collision_contract": moveit_collision_contract,
        "canonical_replay": replay,
        "identity": identity,
        "simulation_checks": {
            "isaac_stage_loaded": False,
            "isaaclab_vectorized_reset_executed": False,
            "canonical_replay_executed": False,
            "simulated_camera_fov_checked": False,
            "simulated_robot_collision_checked": False,
            "simulated_table_collision_checked": False,
        },
        "completion_claim": {
            "s0_smoke_test_passed": False,
            "blocking_reason": "isaac_runner_not_executed_for_this_manifest",
        },
    }


def validate_s0_host_manifest(document: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize the manifest consumed by all Isaac S0 runners."""
    if (
        document.get("schema_version") != 1
        or document.get("record_kind") != "towel_isaac_s0_host_manifest"
        or document.get("status") != S0_STATUS
        or document.get("motion_authorized") is not False
        or document.get("automatic_execution_permitted") is not False
        or document.get("execution_api_used") is not False
        or document.get("motion_commands") != 0
    ):
        raise TowelIsaacS0Error("invalid or motion-capable S0 host manifest")
    environment_count = int(document.get("environment_count", 0))
    poses = document.get("rigid_proxy_pose_xyz_yaw_rad")
    clear = document.get("clear_joint_positions_rad")
    replay = _mapping(document.get("canonical_replay"), "canonical_replay")
    identity = _mapping(document.get("identity"), "identity")
    urdf_path = Path(str(document.get("urdf_path", ""))).resolve()
    worktable_geometry = document.get("worktable_geometry")
    moveit_collision_contract = document.get("moveit_collision_contract")
    worktable_path = (
        Path(str(worktable_geometry.get("path", ""))).resolve()
        if isinstance(worktable_geometry, dict)
        else Path()
    )
    if (
        environment_count < 2
        or not isinstance(poses, list)
        or len(poses) != environment_count
        or document.get("joint_names") != CANONICAL_JOINTS
        or not isinstance(clear, list)
        or len(clear) != len(CANONICAL_JOINTS)
        or not urdf_path.is_file()
        or _sha256(urdf_path) != document.get("urdf_sha256")
        or document.get("proxy_size_xyz_m") != [0.3, 0.3, 0.003]
        or not isinstance(worktable_geometry, dict)
        or not isinstance(moveit_collision_contract, dict)
        or len(worktable_geometry.get("size_xyz_m", [])) != 3
        or len(worktable_geometry.get("pose_xyz_m", [])) != 3
        or not worktable_path.is_file()
        or _sha256(worktable_path) != worktable_geometry.get("sha256")
        or min(float(value) for value in worktable_geometry["size_xyz_m"])
        <= 0.0
        or _digest_json(worktable_geometry)
        != identity.get("worktable_geometry_sha256")
        or _digest_json(moveit_collision_contract)
        != identity.get("moveit_collision_contract_sha256")
        or moveit_collision_contract.get("authority")
        != "strict_moveit_fcl_source_plan"
        or moveit_collision_contract.get("unapproved_contact_count") != 0
        or float(
            moveit_collision_contract.get(
                "maximum_accepted_mesh_contact_depth_m", float("inf")
            )
        )
        > float(
            moveit_collision_contract.get(
                "maximum_accepted_mesh_contact_depth_limit_m", float("-inf")
            )
        )
        or _digest_json(replay) != identity.get("canonical_replay_sha256")
        or _digest_json(poses) != identity.get("reset_batch_sha256")
    ):
        raise TowelIsaacS0Error("S0 host manifest identity or shape is invalid")
    for fold in ("first_fold", "second_fold"):
        phases = replay.get(fold)
        if not isinstance(phases, list) or not phases:
            raise TowelIsaacS0Error(f"manifest has no {fold}")
        if any(len(item.get("joint_positions_rad", [])) != 12 for item in phases):
            raise TowelIsaacS0Error(f"manifest {fold} joint target is invalid")
        strict_flags = ["trajectory_positions_rad" in item for item in phases]
        if any(strict_flags) and not all(strict_flags):
            raise TowelIsaacS0Error(f"manifest {fold} mixes strict and endpoint replay")
        for item in phases:
            if "trajectory_positions_rad" not in item:
                continue
            names = item.get("trajectory_joint_names")
            points = item.get("trajectory_positions_rad")
            if (
                len(item.get("start_positions_rad", [])) != 12
                or not isinstance(names, list)
                or not names
                or len(set(names)) != len(names)
                or any(name not in CANONICAL_JOINTS for name in names)
                or not isinstance(points, list)
                or not points
                or any(
                    not isinstance(point, list) or len(point) != len(names)
                    for point in points
                )
            ):
                raise TowelIsaacS0Error(
                    f"manifest {fold} strict trajectory is invalid"
                )
    return {
        "environment_count": environment_count,
        "joint_names": list(CANONICAL_JOINTS),
        "urdf_path": str(urdf_path),
        "urdf_sha256": str(document["urdf_sha256"]),
        "clear_joint_positions_rad": [float(value) for value in clear],
        "proxy_size_xyz_m": list(document["proxy_size_xyz_m"]),
        "rigid_proxy_pose_xyz_yaw_rad": poses,
        "worktable_geometry": worktable_geometry,
        "moveit_collision_contract": moveit_collision_contract,
        "canonical_replay": replay,
        "identity": identity,
    }


def validate_s0_contract(contract_path: Path, root: Path) -> dict[str, Any]:
    contract_path = contract_path.resolve()
    root = root.resolve()
    document = _load_json(contract_path)
    if (
        document.get("schema_version") != 1
        or document.get("status") != "R2_S0_HOST_PREFLIGHT_CANDIDATE"
        or document.get("motion_authorized") is not False
        or document.get("simulation_success_claimed") is not False
    ):
        raise TowelIsaacS0Error("S0 contract authorization/status is invalid")

    stage = _mapping(document.get("stage"), "stage")
    if (
        stage.get("name") != "rigid_proxy"
        or stage.get("cloth_dynamics_enabled") is not False
        or stage.get("robot_controller_enabled") is not False
        or stage.get("proxy_size_m") != [0.3, 0.3, 0.003]
    ):
        raise TowelIsaacS0Error("S0 must remain a controller-free rigid proxy")

    replay = _mapping(document.get("vectorized_replay"), "vectorized_replay")
    seeds = replay.get("seeds")
    if (
        not isinstance(seeds, list)
        or replay.get("environment_count") != len(seeds)
        or len(seeds) < 2
        or len(set(seeds)) != len(seeds)
        or replay.get("reset_randomization_enabled") is not False
        or replay.get("trajectory_source") != "strict_moveit_plan_only"
    ):
        raise TowelIsaacS0Error("vectorized replay seeds/reset policy is invalid")

    resolved: dict[str, Path] = {}
    sources = _mapping(document.get("sources"), "sources")
    for name in (
        "urdf",
        "urdf_manifest",
        "top_camera_info",
        "worktable",
        "task_contract",
        "plan",
    ):
        source = _mapping(sources.get(name), f"sources.{name}")
        path = (root / str(source.get("path", ""))).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise TowelIsaacS0Error(f"source escapes repository: {name}") from exc
        if not path.is_file() or _sha256(path) != source.get("sha256"):
            raise TowelIsaacS0Error(f"source is missing or stale: {name}")
        resolved[name] = path

    camera = _load_yaml(resolved["top_camera_info"])
    if camera.get("image_width") != 1280 or camera.get("image_height") != 960:
        raise TowelIsaacS0Error("S0 requires the validated 1280x960 Top camera")
    worktable = _load_yaml(resolved["worktable"])
    if worktable.get("status") != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED":
        raise TowelIsaacS0Error("worktable is not the validated metric candidate")

    task = _load_yaml(resolved["task_contract"])
    policy = _mapping(task.get("fold_policy"), "fold_policy")
    expected = _mapping(document.get("expected_strategy"), "expected_strategy")
    if (
        policy.get("first_direction") != "robot_near_to_far"
        or policy.get("second_direction_candidates", [None])[0] != "right_to_left"
        or policy.get("second_active_arm_candidates", [None])[0] != "right"
        or "second_relay_fallback" in policy
    ):
        raise TowelIsaacS0Error("task contract does not match the bounded fold policy")

    plan = _load_json(resolved["plan"])
    selected = _mapping(plan.get("selected_candidate"), "selected_candidate")
    for key in (
        "candidate_id",
        "first_axis",
        "first_direction",
        "first_arm_assignment",
        "second_axis",
        "second_direction",
        "second_active_arm",
    ):
        if selected.get(key) != expected.get(key):
            raise TowelIsaacS0Error(f"plan strategy mismatch: {key}")
    strict = _mapping(selected.get("strict_validation"), "strict_validation")
    if (
        plan.get("status") != "TOWEL_BIMANUAL_THEN_SINGLE_TASK_POSE_PLAN_ONLY_PASS"
        or plan.get("record_kind") != "towel_bimanual_then_single_task_pose_plan_only"
        or plan.get("motion_authorized") is not False
        or plan.get("automatic_execution_permitted") is not False
        or plan.get("execution_api_used") is not False
        or plan.get("motion_commands") != 0
        or strict.get("unapproved_contact_count") != 0
        or int(strict.get("planning_segment_count", 0)) < 1
        or int(strict.get("strict_state_sample_count", 0)) < 1
        or not selected.get("first_fold")
        or not selected.get("second_fold")
        or len(selected.get("correction_envelope", [])) != 8
    ):
        raise TowelIsaacS0Error("strict plan-only evidence is incomplete or unsafe")

    isaac_sim_available = importlib.util.find_spec("isaacsim") is not None
    isaac_lab_available = importlib.util.find_spec("isaaclab") is not None
    return {
        "status": "R2_S0_HOST_PREFLIGHT_PASS_SIMULATION_NOT_RUN",
        "simulation_executed": False,
        "simulation_success_claimed": False,
        "environment_count": len(seeds),
        "seeds": seeds,
        "plan_sha256": _sha256(resolved["plan"]),
        "planning_segment_count": strict["planning_segment_count"],
        "strict_state_sample_count": strict["strict_state_sample_count"],
        "isaac_sim_available_in_current_python": isaac_sim_available,
        "isaac_lab_available_in_current_python": isaac_lab_available,
    }
