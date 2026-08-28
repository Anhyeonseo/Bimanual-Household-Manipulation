#!/usr/bin/env python3
"""Validate and visualize the calibrated Top-camera S0 containment gate."""

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
parser.add_argument(
    "--camera-info",
    type=Path,
    default=ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml",
)
parser.add_argument(
    "--homography",
    type=Path,
    default=ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml",
)
parser.add_argument(
    "--task-contract",
    type=Path,
    default=ROOT / "config/towel_task_contract.candidate.yaml",
)
parser.add_argument("--keep-open", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
for path in (args.manifest, args.camera_info, args.homography, args.task_contract):
    if not path.is_file():
        parser.error(f"input does not exist: {path}")
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

from tools.lib.towel_isaac_fov import validate_calibrated_top_fov
from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


STATUS = "S0_ISAACLAB_CALIBRATED_FOV_PASS_COLLISION_NOT_RUN"
ENVIRONMENT_SPACING_M = 1.0


def load_inputs() -> tuple[dict[str, object], dict[str, object], dict[str, object]]:
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("manifest root must be a mapping")
    source = validate_s0_host_manifest(manifest)
    fov = validate_calibrated_top_fov(
        source,
        camera_info_path=args.camera_info,
        homography_path=args.homography,
        task_contract_path=args.task_contract,
    )
    return manifest, source, fov


def scene_config(
    source: dict[str, object], fov: dict[str, object]
) -> InteractiveSceneCfg:
    proxy_size = tuple(float(value) for value in source["proxy_size_xyz_m"])
    table_geometry = source["worktable_geometry"]
    table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
    table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])
    camera_position = tuple(float(value) for value in fov["camera_position_workcell_m"])

    @configclass
    class TowelS0FovSceneCfg(InteractiveSceneCfg):
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=1500.0),
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
        camera_marker = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/CalibratedTopCamera",
            spawn=sim_utils.SphereCfg(
                radius=0.015,
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(1.0, 0.75, 0.05)
                ),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=camera_position),
        )

    return TowelS0FovSceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        replicate_physics=True,
    )


def reset_proxy(scene: InteractiveScene, source: dict[str, object]) -> float:
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
    proxy = scene["towel_proxy"]
    proxy.write_root_pose_to_sim_index(
        root_pose=torch.cat((world_position, orientation), dim=-1)
    )
    proxy.write_root_velocity_to_sim_index(
        root_velocity=torch.zeros(
            (int(source["environment_count"]), 6), device=scene.device
        )
    )
    scene.reset()
    scene.write_data_to_sim()
    return float(
        torch.max(torch.abs(world_position - proxy.data.root_pos_w.torch)).item()
    )


def draw_fov(fov: dict[str, object], table_z_m: float) -> None:
    from isaacsim.util.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    camera = tuple(float(value) for value in fov["camera_position_workcell_m"])
    footprint = [
        (float(point[0]), float(point[1]), table_z_m + 0.004)
        for point in fov["image_footprint_workcell_xy_m"]
    ]
    envelope = [
        (float(point[0]), float(point[1]), table_z_m + 0.007)
        for point in fov["envelope_workcell_xy_m"]
    ]

    starts: list[tuple[float, float, float]] = []
    ends: list[tuple[float, float, float]] = []
    colors: list[tuple[float, float, float, float]] = []
    widths: list[float] = []

    def add_loop(points, color, width):
        for index, point in enumerate(points):
            starts.append(point)
            ends.append(points[(index + 1) % len(points)])
            colors.append(color)
            widths.append(width)

    add_loop(footprint, (1.0, 0.75, 0.05, 1.0), 3.0)
    add_loop(envelope, (0.10, 1.0, 0.20, 1.0), 5.0)
    for point in footprint:
        starts.append(camera)
        ends.append(point)
        colors.append((0.0, 0.80, 1.0, 1.0))
        widths.append(2.0)
    draw.draw_lines(starts, ends, colors, widths)


def run() -> int:
    manifest, source, fov = load_inputs()
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.01, device=args.device))
    scene = InteractiveScene(scene_config(source, fov))
    sim.set_camera_view(eye=(0.78, 0.56, 0.72), target=(0.30, -0.12, 0.0))
    sim.reset()
    reset_error = reset_proxy(scene, source)
    sim.step()
    scene.update(sim.get_physics_dt())
    if not math.isfinite(reset_error) or reset_error > 1.0e-5:
        raise RuntimeError(f"FOV stage proxy reset error is {reset_error:.9f} m")

    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s0_calibrated_fov_result",
        "status": STATUS,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": int(source["environment_count"]),
        "device": str(sim.device),
        "identity": {**source["identity"], **fov["identity"]},
        "maximum_reset_position_error_m": reset_error,
        "fov": {key: value for key, value in fov.items() if key != "identity"},
        "simulation_checks": {
            "isaac_stage_loaded": True,
            "isaaclab_vectorized_reset_executed": True,
            "bimanual_articulation_loaded": False,
            "canonical_replay_executed": False,
            "simulated_camera_fov_checked": True,
            "simulated_task_pose_access_checked": True,
            "simulated_robot_collision_checked": False,
            "simulated_table_collision_checked": False,
        },
        "completion_claim": {
            "calibrated_fov_passed": True,
            "s0_smoke_test_passed": False,
            "blocking_reason": "robot_and_table_collision_not_executed",
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
        f"minimum_image_margin_px={fov['minimum_image_margin_px']:.3f} "
        f"minimum_board_margin_m={fov['minimum_calibrated_board_margin_m']:.6f} "
        f"motion_commands=0 output={args.output}",
        flush=True,
    )
    if args.keep_open:
        table_z_m = float(source["rigid_proxy_pose_xyz_yaw_rad"][0][2]) - 0.5 * float(
            source["proxy_size_xyz_m"][2]
        )
        draw_fov(fov, table_z_m)
        print(
            "S0_FOV_GUI_KEEP_OPEN yellow=image footprint green=360mm envelope "
            "cyan=calibrated rays",
            flush=True,
        )
        while simulation_app.is_running():
            sim.step()
    return 0


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S0_FOV_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
