#!/usr/bin/env python3
"""Densely replay canonical S0 transitions with robot collisions enabled."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import traceback


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("manifest", type=Path)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--maximum-joint-step-rad", type=float, default=0.02)
parser.add_argument("--keep-open", action="store_true")
parser.add_argument("--stop-on-first-forbidden", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if (
    not math.isfinite(args.maximum_joint_step_rad)
    or args.maximum_joint_step_rad <= 0.0
):
    parser.error("--maximum-joint-step-rad must be finite and positive")

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass
from omni.physx import get_physx_simulation_interface
import omni.usd
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics

from tools.lib.towel_isaac_collision import (
    classify_contact_pair,
    classify_contact_separation,
    expanded_phase_waypoints,
    interpolation_step_count,
    normalized_prim_path,
    robot_link_from_actor,
)
from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


PASS_STATUS = "S0_ISAACLAB_TRANSITION_COLLISION_PASS"
BLOCKED_STATUS = "S0_ISAACLAB_TRANSITION_COLLISION_BLOCKED"
ENVIRONMENT_SPACING_M = 1.0
ROS_PACKAGE = ROOT / "ros2_ws/src/so101_description"
ROBOT_BODY_NAMES = (
    "workcell_base_link",
    "left_shoulder_link",
    "right_shoulder_link",
    "left_upper_arm_link",
    "right_upper_arm_link",
    "left_lower_arm_link",
    "right_lower_arm_link",
    "left_wrist_link",
    "right_wrist_link",
    "left_gripper_link",
    "right_gripper_link",
    "left_moving_jaw_link",
    "right_moving_jaw_link",
)
CONTACT_FORCE_THRESHOLD_N = 1.0e-4
PROXY_POSITION_TOLERANCE_M = 1.0e-6


def load_manifest(path: Path) -> tuple[dict[str, object], dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    return document, validate_s0_host_manifest(document)


def scene_config(source: dict[str, object]) -> InteractiveSceneCfg:
    proxy_size = tuple(float(value) for value in source["proxy_size_xyz_m"])
    table_geometry = source["worktable_geometry"]
    table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
    table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])

    @configclass
    class TowelS0CollisionSceneCfg(InteractiveSceneCfg):
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=1500.0),
        )
        table = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            spawn=sim_utils.CuboidCfg(
                size=table_size,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True,
                    kinematic_enabled=True,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=10.0),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.30, 0.30, 0.30)
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=table_pose
            ),
        )
        # The rigid towel is shown for context but excluded from this gate: a
        # cuboid cannot represent intended gripper/cloth contact faithfully.
        towel_proxy = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/TowelProxy",
            spawn=sim_utils.CuboidCfg(
                size=proxy_size,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.100),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=False
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.20, 0.80)
                ),
            ),
        )
        contact_probe = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/ContactProbe",
            spawn=sim_utils.SphereCfg(
                # Deliberately overlap the native table collider. Robot-body
                # report coverage is audited separately below; this proves
                # that the global PhysX callback itself is live.
                radius=0.012,
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(1.0, 0.05, 0.05)
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(0.0, 0.0, 1.0)
            ),
        )
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(source["urdf_path"]),
                fix_base=True,
                merge_fixed_joints=True,
                # Contact-report APIs and collision overrides cannot be applied
                # below an instance proxy. S0 collision evidence therefore uses
                # non-instanceable geometry even though the kinematic replay can
                # safely use instances.
                make_instanceable=False,
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                ros_package_paths=[
                    {"name": "so101_description", "path": str(ROS_PACKAGE)}
                ],
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=0,
                ),
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                        stiffness=1000.0,
                        damping=100.0,
                    )
                ),
            ),
            actuators={
                "all_joints": ImplicitActuatorCfg(
                    joint_names_expr=[".*"],
                    effort_limit_sim=10.0,
                    velocity_limit_sim=10.0,
                    stiffness=1000.0,
                    damping=100.0,
                )
            },
        )
        probe_contacts = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/ContactProbe",
            update_period=0.0,
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Table"],
        )

    return TowelS0CollisionSceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        replicate_physics=True,
    )


def phase_records(source: dict[str, object]) -> list[dict[str, object]]:
    replay = source["canonical_replay"]
    return [*replay["first_fold"], *replay["second_fold"]]


def reset_proxy(scene: InteractiveScene, source: dict[str, object]) -> float:
    """Place every rigid proxy at its manifest pose, relative to its env."""
    local_pose = torch.tensor(
        source["rigid_proxy_pose_xyz_yaw_rad"],
        dtype=torch.float32,
        device=scene.device,
    )
    world_position = local_pose[:, :3] + scene.env_origins
    half_yaw = 0.5 * local_pose[:, 3]
    zeros = torch.zeros_like(half_yaw)
    orientation_wxyz = torch.stack(
        (torch.cos(half_yaw), zeros, zeros, torch.sin(half_yaw)), dim=-1
    )
    proxy = scene["towel_proxy"]
    proxy.write_root_pose_to_sim_index(
        root_pose=torch.cat((world_position, orientation_wxyz), dim=-1)
    )
    proxy.write_root_velocity_to_sim_index(
        root_velocity=torch.zeros(
            (int(source["environment_count"]), 6), device=scene.device
        )
    )
    scene.reset()
    scene.write_data_to_sim()
    measured_local = proxy.data.root_pos_w.torch - scene.env_origins
    return float(torch.max(torch.abs(measured_local - local_pose[:, :3])).item())


def forbidden_pair_specs() -> list[tuple[str, str, str]]:
    """Return unambiguous one-source/one-partner collision sensor specs."""
    specs = []
    for source_index, source_body in enumerate(ROBOT_BODY_NAMES):
        source_path = f"{{ENV}}/Robot/{source_body}"
        table_path = "{ENV}/Table"
        category = classify_contact_pair(source_path, table_path)
        if category.startswith("forbidden_"):
            specs.append((source_body, "Table", category))
        for partner_body in ROBOT_BODY_NAMES[source_index + 1 :]:
            partner_path = f"{{ENV}}/Robot/{partner_body}"
            category = classify_contact_pair(source_path, partner_path)
            if category.startswith("forbidden_"):
                specs.append((source_body, partner_body, category))
    return specs


def draw_forbidden_contacts(records, robot, table_top_z_m: float) -> None:
    points = [
        tuple(float(value) for value in item["position_m"])
        for item in records
        if "position_m" in item
    ]
    from isaacsim.util.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    body_positions = {
        name: tuple(float(value) for value in robot.data.body_pos_w.torch[0, index])
        for index, name in enumerate(robot.body_names)
    }
    starts = []
    ends = []
    for item in records[:1]:
        link0 = robot_link_from_actor(str(item["actor0"]))
        link1 = robot_link_from_actor(str(item["actor1"]))
        start = body_positions.get(link0) if link0 is not None else None
        end = body_positions.get(link1) if link1 is not None else None
        if start is not None and end is None and "/Table" in str(item["actor1"]):
            end = (start[0], start[1], table_top_z_m)
        if start is not None and end is not None:
            starts.append(start)
            ends.append(end)
            points.extend((start, end))
    if points:
        draw.draw_points(
            points,
            [(1.0, 0.05, 0.05, 1.0)] * len(points),
            [14.0] * len(points),
        )
    if starts:
        draw.draw_lines(
            starts,
            ends,
            [(1.0, 0.05, 0.05, 1.0)] * len(starts),
            [6.0] * len(starts),
        )


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    print(
        f"S0_COLLISION_START environments={source['environment_count']} "
        f"maximum_joint_step_rad={args.maximum_joint_step_rad:.6f} "
        "motion_commands=0",
        flush=True,
    )
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.01, device=args.device))
    scene = InteractiveScene(scene_config(source))
    sim.set_camera_view(eye=(0.75, 0.55, 0.65), target=(0.18, -0.12, 0.08))
    # URDF conversion finishes while InteractiveScene is built.  Isaac Lab's
    # high-level helper only reached one converted body in this asset, so apply
    # the report API directly to every articulation rigid body before physics
    # starts.  A zero threshold is required because this is a geometric gate,
    # not a force-thresholded contact detector.
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if "/Robot/" not in path and "/ContactProbe" not in path:
            continue
        report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        report_api.CreateThresholdAttr().Set(0.0)
    pair_sensors = []
    for source_body, partner_name, category in forbidden_pair_specs():
        partner_expression = (
            "/World/envs/env_.*/Table"
            if partner_name == "Table"
            else f"/World/envs/env_.*/Robot/{partner_name}"
        )
        sensor = ContactSensor(
            ContactSensorCfg(
                prim_path=f"/World/envs/env_.*/Robot/{source_body}",
                update_period=0.0,
                filter_prim_paths_expr=[partner_expression],
            )
        )
        pair_sensors.append((source_body, partner_name, category, sensor))
    sim.reset()
    robot = scene["robot"]
    joint_ids, imported_names = robot.find_joints(
        source["joint_names"], preserve_order=True
    )
    if imported_names != source["joint_names"] or len(joint_ids) != 12:
        raise RuntimeError(
            "imported articulation does not match the canonical 12-joint order"
        )
    proxy_reset_error_m = reset_proxy(scene, source)
    if (
        not math.isfinite(proxy_reset_error_m)
        or proxy_reset_error_m > PROXY_POSITION_TOLERANCE_M
    ):
        raise RuntimeError(
            f"rigid proxy reset error {proxy_reset_error_m:.9f} m exceeds "
            f"{PROXY_POSITION_TOLERANCE_M:.9f} m"
        )
    sim.step()
    scene.update(sim.get_physics_dt())
    print(
        "S0_COLLISION_PROXY_READY "
        f"maximum_position_error_m={proxy_reset_error_m:.9f}",
        flush=True,
    )
    print(
        "S0_COLLISION_STAGE_READY bodies=" + ",".join(robot.body_names),
        flush=True,
    )
    rigid_body_paths: list[str] = []
    report_api_paths: list[str] = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/env_0/Robot/" not in path or not prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            continue
        rigid_body_paths.append(path)
        if prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            report_api_paths.append(path)
    contact_api_coverage_passed = bool(rigid_body_paths) and (
        len(report_api_paths) == len(rigid_body_paths)
    )
    print(
        "S0_COLLISION_CONTACT_API "
        f"rigid_bodies={len(rigid_body_paths)} "
        f"report_apis={len(report_api_paths)}",
        flush=True,
    )

    forbidden: list[dict[str, object]] = []
    allowed_counts: dict[str, int] = {}
    shallow_mesh_diagnostic = {
        "physx_contact_point_count": 0,
        "physx_maximum_imported_mesh_penetration_m": 0.0,
    }
    seen_forbidden: set[tuple[object, ...]] = set()
    replay_state = {"phase": "initial_reset", "sample": 0}

    def collect_tensor_contacts(phase: str, sample: int) -> None:
        for source_body, partner_name, category, sensor in pair_sensors:
            sensor.update(sim.get_physics_dt(), force_recompute=True)
            force_matrix = sensor.data.force_matrix_w
            if force_matrix is None:
                raise RuntimeError(
                    f"pair sensor {source_body}/{partner_name} has no force matrix"
                )
            forces = force_matrix.torch
            if forces.shape[1:3] != (1, 1):
                raise RuntimeError(
                    f"pair sensor {source_body}/{partner_name} has ambiguous "
                    f"force matrix shape {tuple(forces.shape)}"
                )
            magnitudes = torch.linalg.vector_norm(forces[:, 0, 0, :], dim=-1)
            active_count = int(
                torch.sum(magnitudes > CONTACT_FORCE_THRESHOLD_N).item()
            )
            if active_count == 0:
                continue
            actor0 = f"{{ENV}}/Robot/{source_body}"
            actor1 = (
                "{ENV}/Table"
                if partner_name == "Table"
                else f"{{ENV}}/Robot/{partner_name}"
            )
            allowed_counts[category] = allowed_counts.get(category, 0) + active_count
            pair = tuple(sorted((actor0, actor1)))
            key = (phase, category, *pair)
            if key in seen_forbidden:
                continue
            seen_forbidden.add(key)
            forbidden.append(
                {
                    "phase": phase,
                    "sample": sample,
                    "category": category,
                    "actor0": actor0,
                    "actor1": actor1,
                    "maximum_contact_force_n": float(torch.max(magnitudes).item()),
                    "evidence_source": "single_partner_contact_sensor_force_matrix",
                }
            )

    def on_contact_report(headers, contact_data) -> None:
        for header in headers:
            actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            count = int(header.num_contact_data)
            offset = int(header.contact_data_offset)
            for index in range(offset, offset + count):
                contact = contact_data[index]
                separation = float(contact.separation)
                base_category = classify_contact_pair(actor0, actor1)
                if base_category == "bounded_shallow_mesh_contact":
                    shallow_mesh_diagnostic["physx_contact_point_count"] += 1
                    shallow_mesh_diagnostic[
                        "physx_maximum_imported_mesh_penetration_m"
                    ] = max(
                        shallow_mesh_diagnostic[
                            "physx_maximum_imported_mesh_penetration_m"
                        ],
                        max(0.0, -separation),
                    )
                category = classify_contact_separation(
                    actor0, actor1, separation
                )
                if not category.startswith("forbidden_"):
                    allowed_counts[category] = allowed_counts.get(category, 0) + 1
                    continue
                position = tuple(float(value) for value in contact.position)
                key = (
                    replay_state["phase"],
                    replay_state["sample"],
                    category,
                    normalized_prim_path(actor0),
                    normalized_prim_path(actor1),
                    *(round(value, 5) for value in position),
                )
                if key in seen_forbidden:
                    continue
                seen_forbidden.add(key)
                forbidden.append(
                    {
                        "phase": replay_state["phase"],
                        "sample": replay_state["sample"],
                        "category": category,
                        "actor0": normalized_prim_path(actor0),
                        "actor1": normalized_prim_path(actor1),
                        "position_m": list(position),
                        "separation_m": separation,
                    }
                )

    def on_full_contact_report(headers, contact_data, _friction_anchors) -> None:
        on_contact_report(headers, contact_data)

    subscription = (
        get_physx_simulation_interface().subscribe_full_contact_report_events(
            on_full_contact_report
        )
    )

    # Prove that the global contact callback is live, then park the probe above
    # the workcell. Complete report-API coverage on all robot rigid bodies is
    # checked independently above.
    probe = scene["contact_probe"]
    contact_positions = []
    for environment_index in range(int(source["environment_count"])):
        table_prim = stage.GetPrimAtPath(
            f"/World/envs/env_{environment_index}/Table"
        )
        if not table_prim.IsValid():
            raise RuntimeError("vectorized collision table prim is missing")
        world = UsdGeom.Xformable(table_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        translation = world.ExtractTranslation()
        contact_positions.append(
            (
                float(translation[0]),
                float(translation[1]),
                float(translation[2])
                + 0.5 * float(source["worktable_geometry"]["size_xyz_m"][2])
                + 0.050,
            )
        )
    contact_position = torch.tensor(
        contact_positions, dtype=torch.float32, device=scene.device
    )
    probe_orientation = torch.tensor(
        (0.0, 0.0, 0.0, 1.0), dtype=torch.float32, device=scene.device
    ).repeat(int(source["environment_count"]), 1)
    probe.write_root_pose_to_sim_index(
        root_pose=torch.cat((contact_position, probe_orientation), dim=-1)
    )
    probe.write_root_velocity_to_sim_index(
        root_velocity=torch.tensor(
            (0.0, 0.0, -0.2, 0.0, 0.0, 0.0),
            dtype=torch.float32,
            device=scene.device,
        ).repeat(int(source["environment_count"]), 1)
    )
    scene.write_data_to_sim()
    replay_state["phase"] = "contact_sensor_liveness"
    liveness_maximum_force_n = 0.0
    liveness_contact_count = 0
    for sample in range(1, 101):
        replay_state["sample"] = sample
        sim.step()
        scene.update(sim.get_physics_dt())
        probe_forces = scene["probe_contacts"].data.force_matrix_w
        if probe_forces is None:
            raise RuntimeError("probe contact sensor did not create a force matrix")
        probe_magnitudes = torch.linalg.vector_norm(
            probe_forces.torch[:, 0, :, :], dim=-1
        )
        liveness_maximum_force_n = max(
            liveness_maximum_force_n,
            float(torch.max(probe_magnitudes).item()),
        )
        liveness_contact_count += int(
            torch.sum(probe_magnitudes > CONTACT_FORCE_THRESHOLD_N).item()
        )
    parked_position = scene.env_origins + torch.tensor(
        (0.45, 0.45, 1.0), dtype=torch.float32, device=scene.device
    )
    probe.write_root_pose_to_sim_index(
        root_pose=torch.cat((parked_position, probe_orientation), dim=-1)
    )
    probe.write_root_velocity_to_sim_index(
        root_velocity=torch.zeros(
            (int(source["environment_count"]), 6), device=scene.device
        )
    )
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())
    print(
        f"S0_COLLISION_CONTACT_SENSOR_LIVE contacts={liveness_contact_count}",
        flush=True,
    )

    clear = torch.tensor(
        source["clear_joint_positions_rad"],
        dtype=torch.float32,
        device=sim.device,
    )
    current = clear.clone()
    environment_count = int(source["environment_count"])
    zero_velocity = torch.zeros((environment_count, 12), device=sim.device)
    phases = phase_records(source)
    sample_count = 0
    stopped_on_first_forbidden = False

    for phase_index, phase in enumerate(phases, start=1):
        waypoints = expanded_phase_waypoints(
            phase,
            canonical_joint_names=source["joint_names"],
            current_positions_rad=current.tolist(),
        )
        phase_sample_count = 0
        for waypoint in waypoints:
            target = torch.tensor(
                waypoint, dtype=torch.float32, device=sim.device
            )
            steps = interpolation_step_count(
                current.tolist(),
                target.tolist(),
                maximum_joint_step_rad=args.maximum_joint_step_rad,
            )
            start = current.clone()
            for interpolation_index in range(1, steps + 1):
                if not simulation_app.is_running():
                    raise RuntimeError(
                        "Isaac application closed during collision replay"
                    )
                phase_sample_count += 1
                replay_state["phase"] = phase["name"]
                replay_state["sample"] = phase_sample_count
                ratio = float(interpolation_index) / float(steps)
                row = start + ratio * (target - start)
                batch = row.repeat(environment_count, 1)
                robot.set_joint_position_target_index(
                    target=batch, joint_ids=joint_ids
                )
                robot.write_joint_state_to_sim_index(
                    position=batch,
                    velocity=zero_velocity,
                    joint_ids=joint_ids,
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim.get_physics_dt())
                collect_tensor_contacts(phase["name"], phase_sample_count)
                sample_count += 1
                if args.stop_on_first_forbidden and forbidden:
                    stopped_on_first_forbidden = True
                    break
            if stopped_on_first_forbidden:
                break
            current = target
        print(
            f"S0_COLLISION phase={phase_index:02d}/{len(phases):02d} "
            f"name={phase['name']} samples={phase_sample_count} "
            f"forbidden_contacts={len(forbidden)}",
            flush=True,
        )
        if stopped_on_first_forbidden:
            break

    del subscription
    contact_sensor_live = (
        liveness_maximum_force_n > CONTACT_FORCE_THRESHOLD_N
        and contact_api_coverage_passed
    )
    status = PASS_STATUS if not forbidden and contact_sensor_live else BLOCKED_STATUS
    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s0_transition_collision_result",
        "status": status,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": environment_count,
        "device": str(sim.device),
        "identity": source["identity"],
        "urdf": str(source["urdf_path"]),
        "urdf_sha256": str(source["urdf_sha256"]),
        "joint_names": list(imported_names),
        "phase_count": len(phases),
        "interpolation_sample_count": sample_count,
        "stopped_on_first_forbidden": stopped_on_first_forbidden,
        "maximum_joint_step_rad": args.maximum_joint_step_rad,
        "maximum_proxy_reset_position_error_m": proxy_reset_error_m,
        "proxy_position_tolerance_m": PROXY_POSITION_TOLERANCE_M,
        "collision_scope": {
            "robot_self_collision_enabled": True,
            "robot_table_collision_enabled": True,
            "robot_proxy_collision_enabled": False,
            "gravity_enabled": False,
            "adjacent_robot_links_allowed": True,
            "fixed_mount_table_contact_allowed": True,
            "shallow_mesh_exception_authority": (
                "pinned_strict_moveit_fcl_source_plan"
            ),
        },
        "moveit_collision_contract": source["moveit_collision_contract"],
        "physx_shallow_mesh_diagnostic": shallow_mesh_diagnostic,
        "allowed_contact_point_counts": allowed_counts,
        "contact_report_liveness": {
            "probe_contact_count": liveness_contact_count,
            "probe_maximum_contact_force_n": liveness_maximum_force_n,
            "contact_force_threshold_n": CONTACT_FORCE_THRESHOLD_N,
            "robot_rigid_body_count": len(rigid_body_paths),
            "robot_contact_report_api_count": len(report_api_paths),
            "robot_contact_report_api_coverage_passed": (
                contact_api_coverage_passed
            ),
            "passed": contact_sensor_live,
        },
        "forbidden_contact_count": len(forbidden),
        "forbidden_contacts": forbidden,
        "simulation_checks": {
            "isaac_stage_loaded": True,
            "bimanual_articulation_loaded": True,
            "canonical_dense_transition_replay_executed": True,
            "simulated_robot_self_collision_checked": True,
            "simulated_robot_table_collision_checked": True,
            "simulated_robot_proxy_collision_checked": False,
        },
        "completion_claim": {
            "transition_collision_passed": not forbidden and contact_sensor_live,
            "s0_smoke_test_passed": not forbidden and contact_sensor_live,
            "blocking_reason": (
                None
                if not forbidden and contact_sensor_live
                else (
                    "forbidden_contact_detected"
                    if forbidden
                    else "contact_report_liveness_failed"
                )
            ),
        },
        "source_status": manifest["status"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{status} environments={environment_count} phases={len(phases)} "
        f"samples={sample_count} forbidden_contacts={len(forbidden)} "
        f"motion_commands=0 output={args.output}",
        flush=True,
    )
    if args.keep_open:
        proxy_pose = source["rigid_proxy_pose_xyz_yaw_rad"][0]
        table_top_z_m = float(proxy_pose[2]) - 0.5 * float(
            source["proxy_size_xyz_m"][2]
        )
        draw_forbidden_contacts(forbidden, robot, table_top_z_m)
        if forbidden:
            first = forbidden[0]
            print(
                "S0_COLLISION_FIRST_FORBIDDEN "
                f"phase={first['phase']} sample={first['sample']} "
                f"actor0={first['actor0']} actor1={first['actor1']}",
                flush=True,
            )
        print("S0_COLLISION_GUI_KEEP_OPEN red=forbidden contact", flush=True)
        while simulation_app.is_running():
            sim.step()
    return 0 if not forbidden and contact_sensor_live else 2


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S0_COLLISION_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
