#!/usr/bin/env python3
"""Run the first R2 S0 Isaac Lab vectorized rigid-proxy reset gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("manifest", type=Path)
parser.add_argument("--output", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


STATUS = "S0_ISAACLAB_VECTORIZED_RESET_PASS_REPLAY_NOT_RUN"
POSITION_TOLERANCE_M = 1.0e-5
ENVIRONMENT_SPACING_M = 1.0


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
    class TowelS0SceneCfg(InteractiveSceneCfg):
        table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            spawn=sim_utils.CuboidCfg(
                size=table_size,
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.25, 0.25, 0.25)
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
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.100),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.20, 0.80)
                ),
            ),
        )

    return TowelS0SceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        replicate_physics=True,
    )


def yaw_to_wxyz(yaws: torch.Tensor) -> torch.Tensor:
    half = 0.5 * yaws
    zeros = torch.zeros_like(half)
    return torch.stack((torch.cos(half), zeros, zeros, torch.sin(half)), dim=-1)


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.01, device=args.device))
    scene = InteractiveScene(scene_config(source))
    sim.reset()

    proxy = scene["towel_proxy"]
    local_pose = torch.tensor(
        source["rigid_proxy_pose_xyz_yaw_rad"],
        dtype=torch.float32,
        device=sim.device,
    )
    world_position = local_pose[:, :3] + scene.env_origins
    orientation = yaw_to_wxyz(local_pose[:, 3])
    root_pose = torch.cat((world_position, orientation), dim=-1)
    root_velocity = torch.zeros(
        (int(source["environment_count"]), 6), device=sim.device
    )
    proxy.write_root_pose_to_sim_index(root_pose=root_pose)
    proxy.write_root_velocity_to_sim_index(root_velocity=root_velocity)
    scene.reset()
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    measured_local = proxy.data.root_pos_w.torch - scene.env_origins
    maximum_error = float(
        torch.max(torch.abs(measured_local - local_pose[:, :3])).item()
    )
    if not math.isfinite(maximum_error) or maximum_error > POSITION_TOLERANCE_M:
        raise RuntimeError(
            f"vectorized reset position error {maximum_error:.9f} m exceeds "
            f"{POSITION_TOLERANCE_M:.9f} m"
        )
    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s0_vectorized_reset_result",
        "status": STATUS,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": int(source["environment_count"]),
        "device": str(sim.device),
        "identity": source["identity"],
        "maximum_reset_position_error_m": maximum_error,
        "position_tolerance_m": POSITION_TOLERANCE_M,
        "simulation_checks": {
            "isaac_stage_loaded": True,
            "isaaclab_vectorized_reset_executed": True,
            "canonical_replay_executed": False,
            "simulated_camera_fov_checked": False,
            "simulated_task_pose_access_checked": False,
            "simulated_robot_collision_checked": False,
            "simulated_table_collision_checked": False,
        },
        "completion_claim": {
            "vectorized_reset_passed": True,
            "s0_smoke_test_passed": False,
            "blocking_reason": "canonical_robot_replay_fov_and_collision_not_executed",
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
        f"max_reset_error_m={maximum_error:.9f} motion_commands=0 "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    finally:
        simulation_app.close()
