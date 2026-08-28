#!/usr/bin/env python3
"""Replay all canonical R2 S0 poses on the simulated bimanual articulation."""

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
parser.add_argument("--phase-seconds", type=float, default=0.45)
parser.add_argument(
    "--keep-open",
    action="store_true",
    help="keep the GUI open on the final pose until the window is closed",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if not math.isfinite(args.phase_seconds) or args.phase_seconds < 0.0:
    parser.error("--phase-seconds must be finite and non-negative")

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


STATUS = "S0_ISAACLAB_ARTICULATION_REPLAY_PASS_COLLISION_NOT_RUN"
JOINT_TOLERANCE_RAD = 1.0e-5
ENVIRONMENT_SPACING_M = 1.0
ROS_PACKAGE = ROOT / "ros2_ws/src/so101_description"


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
    class TowelS0ReplaySceneCfg(InteractiveSceneCfg):
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(
                intensity=1500.0,
                color=(0.85, 0.85, 0.85),
            ),
        )
        table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            spawn=sim_utils.CuboidCfg(
                size=table_size,
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.30, 0.30, 0.30)
                ),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=table_pose
            ),
        )
        towel_proxy = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/TowelProxy",
            spawn=sim_utils.CuboidCfg(
                size=proxy_size,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.100),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.20, 0.80)
                ),
            ),
        )
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(source["urdf_path"]),
                fix_base=True,
                merge_fixed_joints=True,
                make_instanceable=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=False
                ),
                ros_package_paths=[
                    {"name": "so101_description", "path": str(ROS_PACKAGE)}
                ],
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
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

    return TowelS0ReplaySceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        replicate_physics=True,
    )


def reset_proxy(scene: InteractiveScene, source: dict[str, object]) -> None:
    local_pose = torch.tensor(
        source["rigid_proxy_pose_xyz_yaw_rad"],
        dtype=torch.float32,
        device=scene.device,
    )
    world_position = local_pose[:, :3] + scene.env_origins
    half_yaw = 0.5 * local_pose[:, 3]
    zeros = torch.zeros_like(half_yaw)
    orientation = torch.stack(
        (torch.cos(half_yaw), zeros, zeros, torch.sin(half_yaw)), dim=-1
    )
    root_pose = torch.cat((world_position, orientation), dim=-1)
    root_velocity = torch.zeros(
        (int(source["environment_count"]), 6),
        device=scene.device,
    )
    proxy = scene["towel_proxy"]
    proxy.write_root_pose_to_sim_index(root_pose=root_pose)
    proxy.write_root_velocity_to_sim_index(root_velocity=root_velocity)


def phase_records(source: dict[str, object]) -> list[dict[str, object]]:
    replay = source["canonical_replay"]
    return [*replay["first_fold"], *replay["second_fold"]]


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    print(
        f"S0_REPLAY_START environments={source['environment_count']} "
        "motion_commands=0",
        flush=True,
    )
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.01, device=args.device))
    scene = InteractiveScene(scene_config(source))
    sim.set_camera_view(eye=(0.75, 0.55, 0.65), target=(0.18, -0.12, 0.08))
    sim.reset()
    print("S0_REPLAY_STAGE_READY", flush=True)

    robot = scene["robot"]
    joint_ids, imported_names = robot.find_joints(
        source["joint_names"], preserve_order=True
    )
    if imported_names != source["joint_names"] or len(joint_ids) != 12:
        raise RuntimeError(
            "imported articulation does not match the canonical 12-joint order"
        )
    print("S0_REPLAY_JOINT_MAP_READY joints=12", flush=True)
    reset_proxy(scene, source)
    scene.reset()
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    phases = phase_records(source)
    hold_steps = max(1, round(args.phase_seconds / sim.get_physics_dt()))
    maximum_error = 0.0
    for phase_index, phase in enumerate(phases, start=1):
        target_row = torch.tensor(
            phase["joint_positions_rad"],
            dtype=torch.float32,
            device=sim.device,
        ).repeat(int(source["environment_count"]), 1)
        zero_velocity = torch.zeros_like(target_row)
        robot.set_joint_position_target_index(
            target=target_row,
            joint_ids=joint_ids,
        )
        for _ in range(hold_steps):
            if not simulation_app.is_running():
                raise RuntimeError("Isaac application closed before replay completed")
            robot.write_joint_state_to_sim_index(
                position=target_row,
                velocity=zero_velocity,
                joint_ids=joint_ids,
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
        robot.write_joint_state_to_sim_index(
            position=target_row,
            velocity=zero_velocity,
            joint_ids=joint_ids,
        )
        measured = robot.data.joint_pos.torch[:, joint_ids]
        phase_error = float(torch.max(torch.abs(measured - target_row)).item())
        maximum_error = max(maximum_error, phase_error)
        if not math.isfinite(phase_error) or phase_error > JOINT_TOLERANCE_RAD:
            raise RuntimeError(
                f"phase {phase['name']} joint error {phase_error:.9f} rad "
                f"exceeds {JOINT_TOLERANCE_RAD:.9f} rad"
            )
        print(
            f"S0_REPLAY phase={phase_index:02d}/{len(phases):02d} "
            f"name={phase['name']} max_error_rad={phase_error:.9f}",
            flush=True,
        )

    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s0_articulation_replay_result",
        "status": STATUS,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": int(source["environment_count"]),
        "device": str(sim.device),
        "identity": source["identity"],
        "urdf": str(source["urdf_path"]),
        "urdf_sha256": str(source["urdf_sha256"]),
        "joint_names": list(imported_names),
        "phase_count": len(phases),
        "maximum_joint_position_error_rad": maximum_error,
        "joint_position_tolerance_rad": JOINT_TOLERANCE_RAD,
        "replay_physics": {
            "gravity_enabled": False,
            "robot_collision_enabled": False,
            "purpose": "kinematic_pose_replay_only",
        },
        "simulation_checks": {
            "isaac_stage_loaded": True,
            "isaaclab_vectorized_reset_executed": True,
            "bimanual_articulation_loaded": True,
            "canonical_replay_executed": True,
            "simulated_camera_fov_checked": False,
            "simulated_task_pose_access_checked": True,
            "simulated_robot_collision_checked": False,
            "simulated_table_collision_checked": False,
        },
        "completion_claim": {
            "articulation_replay_passed": True,
            "s0_smoke_test_passed": False,
            "blocking_reason": "camera_fov_and_collision_not_executed",
        },
        "source_status": manifest["status"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{STATUS} environments={source['environment_count']} "
        f"phases={len(phases)} max_joint_error_rad={maximum_error:.9f} "
        f"motion_commands=0 output={args.output}"
    )
    if args.keep_open:
        print("S0_GUI_KEEP_OPEN close the Isaac Sim window when inspection is done")
        while simulation_app.is_running():
            robot.write_joint_state_to_sim_index(
                position=target_row,
                velocity=zero_velocity,
                joint_ids=joint_ids,
            )
            scene.write_data_to_sim()
            sim.step()
    return 0


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S0_REPLAY_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
