#!/usr/bin/env python3
"""Drop a surface-deformable towel onto the S0 worktable and audit settling."""

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
parser.add_argument("--settle-timeout-s", type=float, default=8.0)
parser.add_argument("--keep-open", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if not math.isfinite(args.settle_timeout_s) or args.settle_timeout_s <= 0.0:
    parser.error("--settle-timeout-s must be finite and positive")

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass
from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import (
    PhysxSurfaceDeformableBodyMaterialCfg,
)
import omni.usd
from pxr import PhysxSchema

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


PASS_STATUS = "S1_ISAACLAB_SURFACE_DROP_SETTLE_SMOKE_PASS_MATERIAL_UNCOMMISSIONED"
BLOCKED_STATUS = "S1_ISAACLAB_SURFACE_DROP_SETTLE_BLOCKED"
ENVIRONMENT_SPACING_M = 1.0
PHYSICS_DT_S = 1.0 / 120.0
CLOTH_SIZE_XY_M = (0.300, 0.300)
CLOTH_RESOLUTION = (31, 31)
CLOTH_DROP_HEIGHT_M = 0.050
CLOTH_MASS_CANDIDATE_KG = 0.100
CLOTH_THICKNESS_CANDIDATE_M = 0.003
SETTLE_SPEED_THRESHOLD_M_S = 0.010
SETTLE_CONSECUTIVE_STEPS = 30
MINIMUM_DROP_DISTANCE_M = 0.020
MAXIMUM_TABLE_PENETRATION_M = 0.004
MAXIMUM_TABLE_HOVER_CLEARANCE_M = 0.006
MAXIMUM_FINAL_HEIGHT_SPAN_M = 0.020
MAXIMUM_ENVIRONMENT_DIVERGENCE_M = 1.0e-5
MAXIMUM_TABLE_RESET_ERROR_M = 1.0e-6


def load_manifest(path: Path) -> tuple[dict[str, object], dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    return document, validate_s0_host_manifest(document)


def scene_config(source: dict[str, object]) -> InteractiveSceneCfg:
    table_geometry = source["worktable_geometry"]
    table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
    table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])
    proxy_pose = source["rigid_proxy_pose_xyz_yaw_rad"][0]
    table_top_z_m = table_pose[2] + 0.5 * table_size[2]
    cloth_position = (
        float(proxy_pose[0]),
        float(proxy_pose[1]),
        table_top_z_m + CLOTH_DROP_HEIGHT_M,
    )

    @configclass
    class TowelS1SurfaceSceneCfg(InteractiveSceneCfg):
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
                    collision_enabled=True,
                    contact_offset=0.002,
                    rest_offset=0.0,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.30, 0.30, 0.30)
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=table_pose),
        )
        cloth = DeformableObjectCfg(
            prim_path="{ENV_REGEX_NS}/TowelCloth",
            spawn=sim_utils.MeshRectangleCfg(
                size=CLOTH_SIZE_XY_M,
                resolution=CLOTH_RESOLUTION,
                deformable_props=PhysxDeformableBodyPropertiesCfg(
                    mass=CLOTH_MASS_CANDIDATE_KG,
                    solver_position_iteration_count=24,
                    linear_damping=0.10,
                    settling_damping=1.0,
                    settling_threshold=0.02,
                    sleep_threshold=0.005,
                    max_depenetration_velocity=0.5,
                    self_collision=True,
                    contact_offset=0.003,
                    rest_offset=0.0015,
                    collision_pair_update_frequency=4,
                    collision_iteration_multiplier=2.0,
                ),
                physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
                    density=1000.0,
                    static_friction=0.50,
                    dynamic_friction=0.40,
                    youngs_modulus=1.0e6,
                    poissons_ratio=0.30,
                    elasticity_damping=0.05,
                    surface_thickness=CLOTH_THICKNESS_CANDIDATE_M,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.45, 0.85)
                ),
            ),
            init_state=DeformableObjectCfg.InitialStateCfg(pos=cloth_position),
        )

    return TowelS1SurfaceSceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        # PhysX deformable schemas cannot use the rigid replication shortcut.
        replicate_physics=False,
    )


def local_nodal_positions(
    scene: InteractiveScene, positions_w: torch.Tensor
) -> torch.Tensor:
    return positions_w - scene.env_origins[:, None, :]


def apply_shape_contact_offsets(environment_count: int) -> None:
    """Author offsets on the concrete collision shapes consumed by PhysX."""
    stage = omni.usd.get_context().get_stage()
    for environment_index in range(environment_count):
        for path, contact_offset, rest_offset in (
            (
                f"/World/envs/env_{environment_index}/Table/geometry/mesh",
                0.002,
                0.0,
            ),
            (
                f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh",
                0.003,
                0.0015,
            ),
        ):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"missing S1 collision shape: {path}")
            collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision_api.CreateContactOffsetAttr().Set(contact_offset)
            collision_api.CreateRestOffsetAttr().Set(rest_offset)


def authored_contact_offsets() -> dict[str, dict[str, float | None]]:
    """Read the exact env-0 USD collision offsets consumed by PhysX."""
    stage = omni.usd.get_context().get_stage()
    result = {}
    for label, path in (
        ("table_shape", "/World/envs/env_0/Table/geometry/mesh"),
        ("cloth_root", "/World/envs/env_0/TowelCloth"),
        ("cloth_sim_mesh", "/World/envs/env_0/TowelCloth/sim_mesh"),
    ):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"missing S1 collision prim: {path}")
        values = {}
        for name in ("physxCollision:contactOffset", "physxCollision:restOffset"):
            attribute = prim.GetAttribute(name)
            values[name] = (
                None
                if not attribute.IsValid() or attribute.Get() is None
                else float(attribute.Get())
            )
        result[label] = values
    return result


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    environment_count = int(source["environment_count"])
    print(
        f"S1_SURFACE_START environments={environment_count} "
        f"resolution={CLOTH_RESOLUTION[0]}x{CLOTH_RESOLUTION[1]} "
        "material_physical_fidelity_validated=false motion_commands=0",
        flush=True,
    )
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=PHYSICS_DT_S, device=args.device)
    )
    scene = InteractiveScene(scene_config(source))
    sim.set_camera_view(eye=(0.72, 0.48, 0.52), target=(0.32, -0.12, 0.0))
    apply_shape_contact_offsets(environment_count)
    contact_offsets = authored_contact_offsets()
    print(f"S1_SURFACE_CONTACT_OFFSETS {json.dumps(contact_offsets, sort_keys=True)}", flush=True)
    sim.reset()
    scene.reset()
    scene.write_data_to_sim()
    sim.step()
    scene.update(PHYSICS_DT_S)
    cloth = scene["cloth"]
    table = scene["table"]
    table_geometry = source["worktable_geometry"]
    expected_table_position = torch.tensor(
        table_geometry["pose_xyz_m"], dtype=torch.float32, device=sim.device
    ).unsqueeze(0)
    measured_table_position = table.data.root_pos_w.torch - scene.env_origins
    maximum_table_reset_error_m = float(
        torch.max(torch.abs(measured_table_position - expected_table_position)).item()
    )
    if maximum_table_reset_error_m > MAXIMUM_TABLE_RESET_ERROR_M:
        raise RuntimeError(
            "worktable reset error "
            f"{maximum_table_reset_error_m:.9f} m exceeds "
            f"{MAXIMUM_TABLE_RESET_ERROR_M:.9f} m"
        )
    initial = local_nodal_positions(scene, cloth.data.nodal_pos_w.torch).clone()
    if not torch.all(torch.isfinite(initial)):
        raise RuntimeError("surface cloth initial state contains non-finite nodes")

    initial_center_z_m = float(torch.mean(initial[..., 2]).item())
    maximum_speed_m_s = math.inf
    settled_steps = 0
    settled_step = None
    maximum_steps = max(1, math.ceil(args.settle_timeout_s / PHYSICS_DT_S))
    final = initial
    for step in range(1, maximum_steps + 1):
        if not simulation_app.is_running():
            raise RuntimeError("Isaac application closed during surface settle smoke")
        sim.step()
        scene.update(PHYSICS_DT_S)
        final = local_nodal_positions(scene, cloth.data.nodal_pos_w.torch)
        velocities = cloth.data.nodal_vel_w.torch
        if not torch.all(torch.isfinite(final)) or not torch.all(torch.isfinite(velocities)):
            raise RuntimeError("surface cloth produced a non-finite state")
        maximum_speed_m_s = float(
            torch.max(torch.linalg.vector_norm(velocities, dim=-1)).item()
        )
        current_center_z_m = float(torch.mean(final[..., 2]).item())
        dropped = initial_center_z_m - current_center_z_m >= MINIMUM_DROP_DISTANCE_M
        if dropped and maximum_speed_m_s <= SETTLE_SPEED_THRESHOLD_M_S:
            settled_steps += 1
        else:
            settled_steps = 0
        if settled_steps >= SETTLE_CONSECUTIVE_STEPS:
            settled_step = step
            break
        if step % 120 == 0:
            print(
                f"S1_SURFACE_SETTLE step={step}/{maximum_steps} "
                f"center_z_m={current_center_z_m:.6f} "
                f"maximum_speed_m_s={maximum_speed_m_s:.6f}",
                flush=True,
            )

    final_center_z_m = float(torch.mean(final[..., 2]).item())
    drop_distance_m = initial_center_z_m - final_center_z_m
    minimum_z_m = float(torch.min(final[..., 2]).item())
    maximum_z_m = float(torch.max(final[..., 2]).item())
    maximum_height_span_m = maximum_z_m - minimum_z_m
    table_pose = table_geometry["pose_xyz_m"]
    table_size = table_geometry["size_xyz_m"]
    table_top_z_m = float(table_pose[2]) + 0.5 * float(table_size[2])
    table_min_x = float(table_pose[0]) - 0.5 * float(table_size[0])
    table_max_x = float(table_pose[0]) + 0.5 * float(table_size[0])
    table_min_y = float(table_pose[1]) - 0.5 * float(table_size[1])
    table_max_y = float(table_pose[1]) + 0.5 * float(table_size[1])
    minimum_x_m = float(torch.min(final[..., 0]).item())
    maximum_x_m = float(torch.max(final[..., 0]).item())
    minimum_y_m = float(torch.min(final[..., 1]).item())
    maximum_y_m = float(torch.max(final[..., 1]).item())
    contained = (
        minimum_x_m >= table_min_x
        and maximum_x_m <= table_max_x
        and minimum_y_m >= table_min_y
        and maximum_y_m <= table_max_y
    )
    minimum_table_clearance_m = minimum_z_m - table_top_z_m
    reference = final[0:1]
    environment_divergence_m = float(torch.max(torch.abs(final - reference)).item())
    passed = (
        settled_step is not None
        and drop_distance_m >= MINIMUM_DROP_DISTANCE_M
        and minimum_table_clearance_m >= -MAXIMUM_TABLE_PENETRATION_M
        and minimum_table_clearance_m <= MAXIMUM_TABLE_HOVER_CLEARANCE_M
        and maximum_height_span_m <= MAXIMUM_FINAL_HEIGHT_SPAN_M
        and environment_divergence_m <= MAXIMUM_ENVIRONMENT_DIVERGENCE_M
        and contained
    )
    status = PASS_STATUS if passed else BLOCKED_STATUS
    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s1_surface_drop_settle_smoke_result",
        "status": status,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": environment_count,
        "device": str(sim.device),
        "identity": source["identity"],
        "source_status": manifest["status"],
        "authored_contact_offsets": contact_offsets,
        "cloth": {
            "size_xy_m": list(CLOTH_SIZE_XY_M),
            "resolution": list(CLOTH_RESOLUTION),
            "node_count": int(cloth.max_sim_vertices_per_body),
            "drop_height_m": CLOTH_DROP_HEIGHT_M,
            "mass_candidate_kg": CLOTH_MASS_CANDIDATE_KG,
            "surface_thickness_candidate_m": CLOTH_THICKNESS_CANDIDATE_M,
            "material_physical_fidelity_validated": False,
            "material_basis": "simulation_smoke_placeholder_not_measured_towel",
        },
        "settle": {
            "physics_dt_s": PHYSICS_DT_S,
            "timeout_s": args.settle_timeout_s,
            "settled_step": settled_step,
            "settle_time_s": (
                None if settled_step is None else settled_step * PHYSICS_DT_S
            ),
            "settle_speed_threshold_m_s": SETTLE_SPEED_THRESHOLD_M_S,
            "settle_consecutive_steps": SETTLE_CONSECUTIVE_STEPS,
            "final_maximum_speed_m_s": maximum_speed_m_s,
            "drop_distance_m": drop_distance_m,
            "minimum_drop_distance_m": MINIMUM_DROP_DISTANCE_M,
            "minimum_table_clearance_m": minimum_table_clearance_m,
            "maximum_table_penetration_m": MAXIMUM_TABLE_PENETRATION_M,
            "maximum_table_hover_clearance_m": (
                MAXIMUM_TABLE_HOVER_CLEARANCE_M
            ),
            "final_height_span_m": maximum_height_span_m,
            "maximum_final_height_span_m": MAXIMUM_FINAL_HEIGHT_SPAN_M,
            "tabletop_contained": contained,
            "maximum_environment_divergence_m": environment_divergence_m,
            "environment_divergence_tolerance_m": MAXIMUM_ENVIRONMENT_DIVERGENCE_M,
            "maximum_table_reset_error_m": maximum_table_reset_error_m,
            "table_reset_tolerance_m": MAXIMUM_TABLE_RESET_ERROR_M,
        },
        "completion_claim": {
            "surface_drop_settle_smoke_passed": passed,
            "vertex_patch_attachment_tested": False,
            "lift_place_release_tested": False,
            "material_physical_fidelity_validated": False,
            "s1_completed": False,
            "blocking_reason": (
                "vertex_patch_attachment_and_material_measurement_not_run"
                if passed
                else "surface_drop_settle_gate_failed"
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{status} environments={environment_count} "
        f"nodes={cloth.max_sim_vertices_per_body} "
        f"settle_step={settled_step} drop_distance_m={drop_distance_m:.6f} "
        f"minimum_table_clearance_m={minimum_table_clearance_m:.6f} "
        f"environment_divergence_m={environment_divergence_m:.9f} "
        f"motion_commands=0 output={args.output}",
        flush=True,
    )
    if args.keep_open:
        print("S1_SURFACE_GUI_KEEP_OPEN", flush=True)
        while simulation_app.is_running():
            sim.step()
            scene.update(PHYSICS_DT_S)
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S1_SURFACE_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
