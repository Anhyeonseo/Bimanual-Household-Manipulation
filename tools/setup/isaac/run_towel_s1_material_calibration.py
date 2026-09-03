#!/usr/bin/env python3
"""Measure Isaac towel cantilever and two-corner edge-release responses.

This is a calibration diagnostic, not an S1 completion gate.  It reproduces the
geometry of the home measurements and records simulator-native observables for
later parameter selection.  No robot command or hardware API is imported.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@dataclass(frozen=True, slots=True)
class MaterialCandidate:
    config_path: Path
    config_sha256: str
    mass_kg: float
    density_kg_m3: float
    static_friction: float
    dynamic_friction: float
    youngs_modulus_pa: float
    poissons_ratio: float
    elasticity_damping: float
    surface_bend_stiffness_pa: float
    bend_damping_s_inv: float
    surface_thickness_m: float
    contact_offset_m: float
    rest_offset_m: float
    linear_damping_s_inv: float
    settling_damping_s_inv: float
    settling_threshold_m_s: float


def load_material_candidate(path: Path) -> tuple[dict[str, object], MaterialCandidate]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("record_kind") != "towel_isaac_s1_material_candidate"
        or document.get("motion_authorized") is not False
        or document.get("physical_fidelity_validated") is not False
    ):
        raise ValueError("material config identity or motion lock is invalid")
    model = document.get("model_candidate")
    if not isinstance(model, dict):
        raise ValueError("material config model_candidate must be a mapping")

    def finite(name: str, *, allow_zero: bool = False) -> float:
        value = float(model[name])
        if not math.isfinite(value) or (value < 0.0 if allow_zero else value <= 0.0):
            raise ValueError(f"invalid material parameter {name}: {value}")
        return value

    candidate = MaterialCandidate(
        config_path=path.resolve(),
        config_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        mass_kg=finite("mass_kg"),
        density_kg_m3=finite("density_kg_m3"),
        static_friction=finite("static_friction", allow_zero=True),
        dynamic_friction=finite("dynamic_friction", allow_zero=True),
        youngs_modulus_pa=finite("youngs_modulus_pa"),
        poissons_ratio=finite("poissons_ratio"),
        elasticity_damping=finite("elasticity_damping", allow_zero=True),
        surface_bend_stiffness_pa=finite(
            "surface_bend_stiffness_pa", allow_zero=True
        ),
        bend_damping_s_inv=finite("bend_damping_s_inv", allow_zero=True),
        surface_thickness_m=finite("surface_thickness_m"),
        contact_offset_m=finite("contact_offset_m"),
        rest_offset_m=finite("rest_offset_m", allow_zero=True),
        linear_damping_s_inv=finite("linear_damping_s_inv", allow_zero=True),
        settling_damping_s_inv=finite("settling_damping_s_inv", allow_zero=True),
        settling_threshold_m_s=finite("settling_threshold_m_s"),
    )
    if candidate.static_friction < candidate.dynamic_friction:
        raise ValueError("static friction must not be below dynamic friction")
    if not 0.0 < candidate.poissons_ratio < 0.5:
        raise ValueError("Poisson ratio must be between zero and 0.5")
    return document, candidate


from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("manifest", type=Path)
parser.add_argument("--material-config", type=Path, required=True)
parser.add_argument("--experiment", choices=("cantilever", "edge-release"), required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--youngs-modulus-pa", type=float)
parser.add_argument("--elasticity-damping", type=float)
parser.add_argument("--surface-bend-stiffness-pa", type=float)
parser.add_argument("--bend-damping-s-inv", type=float)
parser.add_argument("--linear-damping-s-inv", type=float)
parser.add_argument("--settling-damping-s-inv", type=float)
parser.add_argument("--settle-timeout-s", type=float, default=12.0)
parser.add_argument("--keep-open", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if not args.material_config.is_file():
    parser.error(f"material config does not exist: {args.material_config}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if not math.isfinite(args.settle_timeout_s) or args.settle_timeout_s <= 0.0:
    parser.error("--settle-timeout-s must be finite and positive")

material_document, material = load_material_candidate(args.material_config)


def override(name: str, supplied: float | None, default: float, *, zero_ok: bool) -> float:
    value = default if supplied is None else float(supplied)
    if not math.isfinite(value) or (value < 0.0 if zero_ok else value <= 0.0):
        parser.error(f"--{name.replace('_', '-')} is invalid: {value}")
    return value


YOUNGS_MODULUS_PA = override(
    "youngs_modulus_pa", args.youngs_modulus_pa, material.youngs_modulus_pa, zero_ok=False
)
ELASTICITY_DAMPING = override(
    "elasticity_damping", args.elasticity_damping, material.elasticity_damping, zero_ok=True
)
SURFACE_BEND_STIFFNESS_PA = override(
    "surface_bend_stiffness_pa",
    args.surface_bend_stiffness_pa,
    material.surface_bend_stiffness_pa,
    zero_ok=True,
)
BEND_DAMPING_S_INV = override(
    "bend_damping_s_inv",
    args.bend_damping_s_inv,
    material.bend_damping_s_inv,
    zero_ok=True,
)
LINEAR_DAMPING_S_INV = override(
    "linear_damping_s_inv", args.linear_damping_s_inv, material.linear_damping_s_inv, zero_ok=True
)
SETTLING_DAMPING_S_INV = override(
    "settling_damping_s_inv",
    args.settling_damping_s_inv,
    material.settling_damping_s_inv,
    zero_ok=True,
)

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass
from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
from isaaclab_physx.physics import PhysxCfg
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


PHYSICS_DT_S = 1.0 / 240.0
VIDEO_FPS = 24.0
VIDEO_SAMPLE_STEPS = round((1.0 / VIDEO_FPS) / PHYSICS_DT_S)
CLOTH_SIZE_XY_M = (0.300, 0.300)
CLOTH_RESOLUTION = (31, 31)
CLOTH_INITIAL_CLEARANCE_M = 0.0015
ENVIRONMENT_SPACING_M = 1.0
SETTLE_SPEED_THRESHOLD_M_S = 0.010
SETTLE_CONSECUTIVE_STEPS = 30
SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD = 2
SELF_COLLISION_FILTER_DISTANCE_M = (
    SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD * CLOTH_SIZE_XY_M[0] / CLOTH_RESOLUTION[0]
)
CANTILEVER_OVERHANG_TARGET_M = 0.0363333333333
CANTILEVER_ANGLE_TARGET_DEG = 45.0
CANTILEVER_ANGLE_TOLERANCE_DEG = 5.0
CANTILEVER_FRAME_DISPLACEMENT_THRESHOLD_M = 0.0010
CANTILEVER_FRAME_ANGLE_DELTA_THRESHOLD_DEG = 0.5
CANTILEVER_SHAPE_CONSECUTIVE_VIDEO_FRAMES = 5
MAXIMUM_SUPPORTED_REGION_SLIP_M = 0.003
EDGE_RELEASE_LIFT_HEIGHT_M = 0.1001
EDGE_RELEASE_LIFT_PREPARATION_S = 3.0
EDGE_RELEASE_TARGET_SETTLE_S = 0.1805555556
EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S = 1.0 / VIDEO_FPS
EDGE_RELEASE_OBSERVATION_THRESHOLDS_M = (0.0005, 0.0010, 0.0015, 0.0020)
EDGE_RELEASE_PRIMARY_THRESHOLD_M = 0.0010
EDGE_RELEASE_CONSECUTIVE_VIDEO_FRAMES = 2
EDGE_RELEASE_TIMEOUT_S = 2.0
CORNER_PATCH_GRID_WIDTH = 3
HOLDER_DRIVE_STIFFNESS_N_M = 5000.0
HOLDER_DRIVE_DAMPING_N_S_M = 150.0
HOLDER_DRIVE_MAX_FORCE_N = 100.0


def load_manifest(path: Path) -> tuple[dict[str, object], dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    return document, validate_s0_host_manifest(document)


manifest, source = load_manifest(args.manifest)
table_geometry = source["worktable_geometry"]
table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])
table_top_z_m = table_pose[2] + 0.5 * table_size[2]
table_max_x_m = table_pose[0] + 0.5 * table_size[0]
proxy_pose = source["rigid_proxy_pose_xyz_yaw_rad"][0]

if args.experiment == "cantilever":
    cloth_center_x_m = table_max_x_m - (
        0.5 * CLOTH_SIZE_XY_M[0] - CANTILEVER_OVERHANG_TARGET_M
    )
else:
    cloth_center_x_m = float(proxy_pose[0])
cloth_center_y_m = float(proxy_pose[1])
cloth_position = (
    cloth_center_x_m,
    cloth_center_y_m,
    table_top_z_m + CLOTH_INITIAL_CLEARANCE_M,
)
holder_position = (
    cloth_center_x_m + 0.5 * CLOTH_SIZE_XY_M[0],
    cloth_center_y_m,
    table_top_z_m + CLOTH_INITIAL_CLEARANCE_M,
)


@configclass
class TowelS1MaterialCalibrationSceneCfg(InteractiveSceneCfg):
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=1500.0),
    )
    table = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=table_size,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True, kinematic_enabled=True
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=10.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True, contact_offset=0.002, rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.30, 0.30, 0.30)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=table_pose),
    )
    holder = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Holder",
        spawn=sim_utils.CuboidCfg(
            size=(0.006, 0.310, 0.006),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                # Surface vertex attachments are validated against dynamic
                # articulation links. Keep this holder dynamic as well, while
                # explicitly writing its pose and zero velocity every step.
                disable_gravity=True, kinematic_enabled=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.20, 0.10)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=holder_position),
    )
    cloth = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/TowelCloth",
        spawn=sim_utils.MeshRectangleCfg(
            size=CLOTH_SIZE_XY_M,
            resolution=CLOTH_RESOLUTION,
            deformable_props=PhysxDeformableBodyPropertiesCfg(
                mass=material.mass_kg,
                solver_position_iteration_count=24,
                linear_damping=LINEAR_DAMPING_S_INV,
                settling_damping=SETTLING_DAMPING_S_INV,
                settling_threshold=material.settling_threshold_m_s,
                sleep_threshold=0.005,
                max_depenetration_velocity=0.5,
                # The cantilever shape has no non-neighboring surface contact;
                # enabling it there imports the known production chatter into
                # the bending observable.  Edge release retains self-contact.
                self_collision=args.experiment == "edge-release",
                self_collision_filter_distance=(
                    SELF_COLLISION_FILTER_DISTANCE_M
                    if args.experiment == "edge-release"
                    else None
                ),
                contact_offset=material.contact_offset_m,
                rest_offset=material.rest_offset_m,
                collision_pair_update_frequency=4,
                collision_iteration_multiplier=2.0,
            ),
            physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
                density=material.density_kg_m3,
                static_friction=material.static_friction,
                dynamic_friction=material.dynamic_friction,
                youngs_modulus=YOUNGS_MODULUS_PA,
                poissons_ratio=material.poissons_ratio,
                elasticity_damping=ELASTICITY_DAMPING,
                surface_bend_stiffness=SURFACE_BEND_STIFFNESS_PA,
                bend_damping=BEND_DAMPING_S_INV,
                surface_thickness=material.surface_thickness_m,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.65, 0.85)),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=cloth_position),
    )


def scene_config() -> InteractiveSceneCfg:
    return TowelS1MaterialCalibrationSceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        replicate_physics=False,
    )


def local_nodes(scene: InteractiveScene, cloth: object) -> torch.Tensor:
    return cloth.data.nodal_pos_w.torch - scene.env_origins[:, None, :]


def apply_shape_contact_offsets(environment_count: int) -> None:
    stage = omni.usd.get_context().get_stage()
    for environment_index in range(environment_count):
        for path, contact_offset, rest_offset in (
            (f"/World/envs/env_{environment_index}/Table/geometry/mesh", 0.002, 0.0),
            (
                f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh",
                material.contact_offset_m,
                material.rest_offset_m,
            ),
        ):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"missing calibration collision shape: {path}")
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr().Set(contact_offset)
            api.CreateRestOffsetAttr().Set(rest_offset)


def holder_root_pose(scene: InteractiveScene, local_position: torch.Tensor) -> torch.Tensor:
    environment_count = int(source["environment_count"])
    world_position = scene.env_origins + local_position.repeat(environment_count, 1)
    orientation = torch.zeros((environment_count, 4), dtype=torch.float32, device=scene.device)
    orientation[:, 0] = 1.0
    return torch.cat((world_position, orientation), dim=-1)


def author_holder_prismatic_drives(scene: InteractiveScene) -> list[object]:
    """Constrain dynamic holders to vertical motion without teleporting them."""
    stage = omni.usd.get_context().get_stage()
    target_attributes = []
    for environment_index in range(int(source["environment_count"])):
        holder_path = Sdf.Path(f"/World/envs/env_{environment_index}/Holder")
        joint = UsdPhysics.PrismaticJoint.Define(
            stage, f"/World/envs/env_{environment_index}/HolderLiftJoint"
        )
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(EDGE_RELEASE_LIFT_HEIGHT_M + 0.005)
        joint.CreateBody1Rel().SetTargets([holder_path])
        initial_world = scene.env_origins[environment_index] + torch.tensor(
            holder_position, dtype=torch.float32, device=scene.device
        )
        joint.CreateLocalPos0Attr().Set(
            Gf.Vec3f(*[float(value) for value in initial_world.tolist()])
        )
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        target = drive.CreateTargetPositionAttr(0.0)
        drive.CreateStiffnessAttr(HOLDER_DRIVE_STIFFNESS_N_M)
        drive.CreateDampingAttr(HOLDER_DRIVE_DAMPING_N_S_M)
        drive.CreateMaxForceAttr(HOLDER_DRIVE_MAX_FORCE_N)
        target_attributes.append(target)
    simulation_app.update()
    return target_attributes


def step_scene(
    sim: SimulationContext,
    scene: InteractiveScene,
    holder: object,
    holder_pose: torch.Tensor | None,
) -> None:
    if holder_pose is not None:
        holder.write_root_pose_to_sim_index(root_pose=holder_pose)
        holder.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros(
                (int(source["environment_count"]), 6),
                dtype=torch.float32,
                device=scene.device,
            )
        )
    scene.write_data_to_sim()
    sim.step()
    scene.update(PHYSICS_DT_S)


def settle_cloth(
    sim: SimulationContext,
    scene: InteractiveScene,
    cloth: object,
    holder: object,
    holder_pose: torch.Tensor | None,
    *,
    timeout_s: float,
) -> tuple[int | None, float]:
    consecutive = 0
    final_speed = math.inf
    for step in range(1, math.ceil(timeout_s / PHYSICS_DT_S) + 1):
        step_scene(sim, scene, holder, holder_pose)
        velocities = cloth.data.nodal_vel_w.torch
        if not torch.all(torch.isfinite(velocities)):
            raise RuntimeError("cloth produced non-finite velocity")
        final_speed = float(
            torch.max(torch.linalg.vector_norm(velocities, dim=-1)).item()
        )
        consecutive = consecutive + 1 if final_speed <= SETTLE_SPEED_THRESHOLD_M_S else 0
        if consecutive >= SETTLE_CONSECUTIVE_STEPS:
            return step, final_speed
    return None, final_speed


def settle_shape_at_video_rate(
    sim: SimulationContext,
    scene: InteractiveScene,
    cloth: object,
    holder: object,
    *,
    timeout_s: float,
    displacement_threshold_m: float = 0.001,
    consecutive_frames: int = 5,
) -> tuple[int | None, float, list[dict[str, float]]]:
    """Wait for visually observable cloth shape motion, not one fastest vertex."""
    previous = local_nodes(scene, cloth).clone()
    quiet_frames = 0
    final_speed = math.inf
    samples = []
    for step in range(1, math.ceil(timeout_s / PHYSICS_DT_S) + 1):
        step_scene(sim, scene, holder, None)
        final_speed = float(
            torch.max(
                torch.linalg.vector_norm(cloth.data.nodal_vel_w.torch, dim=-1)
            ).item()
        )
        if step % VIDEO_SAMPLE_STEPS != 0:
            continue
        current = local_nodes(scene, cloth).clone()
        displacement = torch.linalg.vector_norm(current - previous, dim=-1)
        p95_m = float(
            torch.max(torch.quantile(displacement, 0.95, dim=1)).item()
        )
        samples.append(
            {
                "time_s": step * PHYSICS_DT_S,
                "maximum_environment_p95_displacement_m": p95_m,
            }
        )
        quiet_frames = quiet_frames + 1 if p95_m <= displacement_threshold_m else 0
        previous = current
        if quiet_frames >= consecutive_frames:
            return step, final_speed, samples
    return None, final_speed, samples


def grid_corner_patch_indices(initial_local_nodes: torch.Tensor) -> list[int]:
    points = initial_local_nodes[0]
    unique_x = torch.unique(points[:, 0]).sort().values
    unique_y = torch.unique(points[:, 1]).sort().values
    if len(unique_x) != CLOTH_RESOLUTION[0] + 1 or len(unique_y) != CLOTH_RESOLUTION[1] + 1:
        raise RuntimeError(
            f"unexpected surface grid dimensions: x={len(unique_x)} y={len(unique_y)}"
        )
    x_cut = unique_x[-CORNER_PATCH_GRID_WIDTH]
    low_y_cut = unique_y[CORNER_PATCH_GRID_WIDTH - 1]
    high_y_cut = unique_y[-CORNER_PATCH_GRID_WIDTH]
    mask = (points[:, 0] >= x_cut - 1.0e-6) & (
        (points[:, 1] <= low_y_cut + 1.0e-6)
        | (points[:, 1] >= high_y_cut - 1.0e-6)
    )
    indices = torch.nonzero(mask, as_tuple=False).flatten().tolist()
    expected_count = 2 * CORNER_PATCH_GRID_WIDTH * CORNER_PATCH_GRID_WIDTH
    if len(indices) != expected_count:
        raise RuntimeError(
            f"expected {expected_count} two-corner patch nodes, found {len(indices)}"
        )
    return [int(index) for index in indices]


def author_edge_release_attachments(
    scene: InteractiveScene,
    nodes_w: torch.Tensor,
    holder: object,
    indices: list[int],
) -> list[str]:
    stage = omni.usd.get_context().get_stage()
    paths = []
    holder_positions_w = holder.data.root_pos_w.torch
    for environment_index in range(int(source["environment_count"])):
        holder_path = Sdf.Path(f"/World/envs/env_{environment_index}/Holder")
        holder_prim = stage.GetPrimAtPath(holder_path)
        if not holder_prim.IsValid() or not holder_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"holder is not a rigid body: {holder_path}")
        local_positions = [
            Gf.Vec3f(
                *[
                    float(value)
                    for value in (
                        nodes_w[environment_index, index]
                        - holder_positions_w[environment_index]
                    ).tolist()
                ]
            )
            for index in indices
        ]
        attachment_path = Sdf.Path(
            f"/World/envs/env_{environment_index}/Attachments/two_corner_edge_patch"
        )
        prim = stage.DefinePrim(attachment_path, "OmniPhysicsVtxXformAttachment")
        prim.GetAttribute("omniphysics:attachmentEnabled").Set(True)
        prim.GetRelationship("omniphysics:src0").SetTargets(
            [Sdf.Path(f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh")]
        )
        # Target the concrete kinematic rigid actor directly. A runtime child
        # Xform can exist under a rigid ancestor yet fail to receive tensor-pose
        # updates as an attachment frame.
        prim.GetRelationship("omniphysics:src1").SetTargets([holder_path])
        prim.GetAttribute("omniphysics:vtxIndicesSrc0").Set(indices)
        prim.GetAttribute("omniphysics:localPositionsSrc1").Set(local_positions)
        paths.append(str(attachment_path))
    simulation_app.update()
    for environment_index, path in enumerate(paths):
        prim = stage.GetPrimAtPath(path)
        expected_target = Sdf.Path(f"/World/envs/env_{environment_index}/Holder")
        if prim.GetRelationship("omniphysics:src1").GetTargets() != [expected_target]:
            raise RuntimeError(f"edge-release attachment target changed: {path}")
        if prim.GetAttribute("omniphysics:attachmentEnabled").Get() is not True:
            raise RuntimeError(f"edge-release attachment is not enabled: {path}")
    return paths


def disable_attachments(paths: list[str]) -> None:
    stage = omni.usd.get_context().get_stage()
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"missing edge-release attachment: {path}")
        prim.GetAttribute("omniphysics:attachmentEnabled").Set(False)
    simulation_app.update()


def minimum_nonlocal_node_separation_m(nodes: torch.Tensor) -> float:
    node_count = int(nodes.shape[1])
    side_nodes = int(round(math.sqrt(node_count)))
    flat = torch.arange(node_count, device=nodes.device)
    rows = torch.div(flat, side_nodes, rounding_mode="floor")
    columns = flat % side_nodes
    neighbor = (
        (torch.abs(rows[:, None] - rows[None, :]) <= SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD)
        & (
            torch.abs(columns[:, None] - columns[None, :])
            <= SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD
        )
    )
    result = math.inf
    for environment_nodes in nodes:
        distances = torch.cdist(environment_nodes, environment_nodes)
        distances[neighbor] = math.inf
        result = min(result, float(torch.min(distances).item()))
    return result


def run_cantilever(
    sim: SimulationContext,
    scene: InteractiveScene,
    cloth: object,
    holder: object,
    parked_holder_pose: torch.Tensor,
    initial: torch.Tensor,
) -> tuple[str, dict[str, object]]:
    initial_env0 = initial[0]
    free_x = torch.max(initial_env0[:, 0])
    free_edge_indices = torch.nonzero(
        torch.isclose(initial_env0[:, 0], free_x, atol=1.0e-6), as_tuple=False
    ).flatten()
    supported_indices = torch.nonzero(
        initial_env0[:, 0] <= table_max_x_m - 0.020, as_tuple=False
    ).flatten()
    previous_sample = initial.clone()
    previous_angles_deg = None
    motion_observed = False
    shape_settle_run = 0
    shape_settled_step = None
    frame_samples = []
    final_speed = math.inf
    maximum_steps = math.ceil(args.settle_timeout_s / PHYSICS_DT_S)
    for step in range(1, maximum_steps + 1):
        step_scene(sim, scene, holder, parked_holder_pose)
        final_speed = float(
            torch.max(
                torch.linalg.vector_norm(cloth.data.nodal_vel_w.torch, dim=-1)
            ).item()
        )
        if step % VIDEO_SAMPLE_STEPS != 0:
            continue
        current = local_nodes(scene, cloth).clone()
        displacement = torch.linalg.vector_norm(current - previous_sample, dim=-1)
        p95_m = float(
            torch.max(torch.quantile(displacement, 0.95, dim=1)).item()
        )
        free_edge_sample = torch.mean(current[:, free_edge_indices], dim=1)
        horizontal_sample_m = free_edge_sample[:, 0] - table_max_x_m
        vertical_sample_m = (
            table_top_z_m + CLOTH_INITIAL_CLEARANCE_M - free_edge_sample[:, 2]
        )
        angles_sample_deg = torch.rad2deg(
            torch.atan2(
                torch.clamp(vertical_sample_m, min=0.0),
                torch.clamp(horizontal_sample_m, min=1.0e-9),
            )
        )
        angle_delta_deg = (
            math.inf
            if previous_angles_deg is None
            else float(torch.max(torch.abs(angles_sample_deg - previous_angles_deg)).item())
        )
        motion_observed = motion_observed or p95_m >= CANTILEVER_FRAME_DISPLACEMENT_THRESHOLD_M
        shape_quiet = (
            motion_observed
            and p95_m <= CANTILEVER_FRAME_DISPLACEMENT_THRESHOLD_M
            and angle_delta_deg <= CANTILEVER_FRAME_ANGLE_DELTA_THRESHOLD_DEG
        )
        shape_settle_run = shape_settle_run + 1 if shape_quiet else 0
        frame_samples.append(
            {
                "time_s": step * PHYSICS_DT_S,
                "maximum_environment_p95_displacement_m": p95_m,
                "maximum_environment_angle_delta_deg": (
                    None if not math.isfinite(angle_delta_deg) else angle_delta_deg
                ),
            }
        )
        previous_sample = current
        previous_angles_deg = angles_sample_deg
        if shape_settle_run >= CANTILEVER_SHAPE_CONSECUTIVE_VIDEO_FRAMES:
            shape_settled_step = step
            break
    final = local_nodes(scene, cloth).clone()
    free_edge = torch.mean(final[:, free_edge_indices], dim=1)
    horizontal_m = free_edge[:, 0] - table_max_x_m
    vertical_drop_m = (
        table_top_z_m + CLOTH_INITIAL_CLEARANCE_M - free_edge[:, 2]
    )
    angles_deg = torch.rad2deg(
        torch.atan2(torch.clamp(vertical_drop_m, min=0.0), torch.clamp(horizontal_m, min=1.0e-9))
    )
    supported_displacement_m = torch.linalg.vector_norm(
        final[:, supported_indices, :2] - initial[:, supported_indices, :2], dim=-1
    )
    maximum_supported_slip_m = float(torch.max(supported_displacement_m).item())
    maximum_angle_error_deg = float(
        torch.max(torch.abs(angles_deg - CANTILEVER_ANGLE_TARGET_DEG)).item()
    )
    initial_overhang_m = float(free_x.item() - table_max_x_m)
    matched = (
        shape_settled_step is not None
        and maximum_supported_slip_m <= MAXIMUM_SUPPORTED_REGION_SLIP_M
        and maximum_angle_error_deg <= CANTILEVER_ANGLE_TOLERANCE_DEG
    )
    return (
        "R2_S1_CANTILEVER_CALIBRATION_MATCH" if matched else "R2_S1_CANTILEVER_CALIBRATION_RECORDED_NOT_MATCHED",
        {
            "target_overhang_m": CANTILEVER_OVERHANG_TARGET_M,
            "initial_overhang_m": initial_overhang_m,
            "target_angle_deg": CANTILEVER_ANGLE_TARGET_DEG,
            "angle_tolerance_deg": CANTILEVER_ANGLE_TOLERANCE_DEG,
            "per_environment_chord_angle_deg": angles_deg.tolist(),
            "maximum_angle_error_deg": maximum_angle_error_deg,
            "maximum_supported_region_slip_m": maximum_supported_slip_m,
            "maximum_supported_region_slip_limit_m": MAXIMUM_SUPPORTED_REGION_SLIP_M,
            "shape_settled_step": shape_settled_step,
            "shape_settle_time_s": (
                None
                if shape_settled_step is None
                else shape_settled_step * PHYSICS_DT_S
            ),
            "video_fps": VIDEO_FPS,
            "shape_observable": "maximum_environment_node_displacement_p95_and_chord_angle_delta_per_video_frame",
            "frame_displacement_threshold_m": CANTILEVER_FRAME_DISPLACEMENT_THRESHOLD_M,
            "frame_angle_delta_threshold_deg": CANTILEVER_FRAME_ANGLE_DELTA_THRESHOLD_DEG,
            "consecutive_frames_required": CANTILEVER_SHAPE_CONSECUTIVE_VIDEO_FRAMES,
            "frame_samples": frame_samples,
            "final_maximum_vertex_speed_m_s_diagnostic_only": final_speed,
            "maximum_vertex_speed_is_match_gate": False,
            "matched": matched,
        },
    )


def run_edge_release(
    sim: SimulationContext,
    scene: InteractiveScene,
    cloth: object,
    holder: object,
    initial_holder_local: torch.Tensor,
    holder_drive_targets: list[object],
    initial: torch.Tensor,
) -> tuple[str, dict[str, object]]:
    pre_attach_settled_step, pre_attach_speed = settle_cloth(
        sim,
        scene,
        cloth,
        holder,
        None,
        timeout_s=args.settle_timeout_s,
    )
    if pre_attach_settled_step is None:
        raise RuntimeError(
            f"edge-release cloth did not settle before attachment: {pre_attach_speed:.6f} m/s"
        )
    nodes_before_attachment_w = cloth.data.nodal_pos_w.torch.clone()
    indices = grid_corner_patch_indices(initial)
    attachment_paths = author_edge_release_attachments(
        scene, nodes_before_attachment_w, holder, indices
    )
    lift_steps = round(EDGE_RELEASE_LIFT_PREPARATION_S / PHYSICS_DT_S)
    for step in range(1, lift_steps + 1):
        alpha = step / lift_steps
        for target in holder_drive_targets:
            target.Set(alpha * EDGE_RELEASE_LIFT_HEIGHT_M)
        step_scene(sim, scene, holder, None)
    held_settled_step, held_speed, held_frame_samples = settle_shape_at_video_rate(
        sim, scene, cloth, holder, timeout_s=args.settle_timeout_s
    )
    if held_settled_step is None:
        raise RuntimeError(
            f"edge-release cloth did not settle while held: {held_speed:.6f} m/s"
        )

    held_nodes_w = cloth.data.nodal_pos_w.torch.clone()
    measured_holder_lift_m = float(
        torch.min(
            holder.data.root_pos_w.torch[:, 2]
            - (scene.env_origins[:, 2] + initial_holder_local[2])
        ).item()
    )
    if measured_holder_lift_m < EDGE_RELEASE_LIFT_HEIGHT_M - 1.0e-4:
        raise RuntimeError(
            "edge-release holder did not reach requested height: "
            f"{measured_holder_lift_m:.6f} m"
        )
    per_environment_patch_lift_m = torch.min(
        held_nodes_w[:, indices, 2] - nodes_before_attachment_w[:, indices, 2],
        dim=1,
    ).values
    minimum_patch_lift_m = float(torch.min(per_environment_patch_lift_m).item())
    if minimum_patch_lift_m < EDGE_RELEASE_LIFT_HEIGHT_M - 0.005:
        raise RuntimeError(
            "edge-release corner patches did not reach requested height: "
            f"{minimum_patch_lift_m:.6f} m while holder moved "
            f"{measured_holder_lift_m:.6f} m"
        )

    disable_attachments(attachment_paths)
    # Disabling an attachment does not necessarily wake a sleeping PhysX
    # deformable. Re-author the identical held state with exactly zero velocity;
    # this changes no physical initial condition and uses the public state API.
    zero_release_velocity = torch.zeros_like(cloth.data.nodal_vel_w.torch)
    release_state_w = torch.cat((held_nodes_w, zero_release_velocity), dim=-1)
    cloth.write_nodal_state_to_sim_index(release_state_w)
    scene.write_data_to_sim()
    previous_sample = local_nodes(scene, cloth).clone()
    threshold_counters = {threshold: 0 for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M}
    threshold_onset_times_s: dict[float, float | None] = {
        threshold: None for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M
    }
    threshold_confirmation_times_s: dict[float, float | None] = {
        threshold: None for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M
    }
    motion_observed = False
    frame_samples = []
    maximum_steps = math.ceil(EDGE_RELEASE_TIMEOUT_S / PHYSICS_DT_S)
    for step in range(1, maximum_steps + 1):
        step_scene(sim, scene, holder, None)
        if step % VIDEO_SAMPLE_STEPS != 0:
            continue
        current = local_nodes(scene, cloth).clone()
        displacement = torch.linalg.vector_norm(current - previous_sample, dim=-1)
        per_environment_p95_m = torch.quantile(displacement, 0.95, dim=1)
        p95_m = float(torch.max(per_environment_p95_m).item())
        median_m = float(torch.max(torch.median(displacement, dim=1).values).item())
        motion_observed = motion_observed or p95_m >= EDGE_RELEASE_PRIMARY_THRESHOLD_M
        time_s = step * PHYSICS_DT_S
        frame_samples.append(
            {
                "time_s": time_s,
                "maximum_environment_p95_displacement_m": p95_m,
                "maximum_environment_median_displacement_m": median_m,
            }
        )
        if motion_observed:
            for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M:
                if threshold_onset_times_s[threshold] is not None:
                    continue
                threshold_counters[threshold] = (
                    threshold_counters[threshold] + 1
                    if p95_m <= threshold
                    else 0
                )
                if threshold_counters[threshold] >= EDGE_RELEASE_CONSECUTIVE_VIDEO_FRAMES:
                    # The second quiet frame confirms that motion stopped; it
                    # must not shift the observed stop itself one video frame
                    # later.  Report the first frame in the confirmed run and
                    # retain the confirmation time separately for auditability.
                    threshold_onset_times_s[threshold] = time_s - (
                        (EDGE_RELEASE_CONSECUTIVE_VIDEO_FRAMES - 1) / VIDEO_FPS
                    )
                    threshold_confirmation_times_s[threshold] = time_s
        previous_sample = current

    final = local_nodes(scene, cloth).clone()
    final_speed_m_s = float(
        torch.max(torch.linalg.vector_norm(cloth.data.nodal_vel_w.torch, dim=-1)).item()
    )
    primary_time_s = threshold_onset_times_s[EDGE_RELEASE_PRIMARY_THRESHOLD_M]
    target_min_s = EDGE_RELEASE_TARGET_SETTLE_S - EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S
    target_max_s = EDGE_RELEASE_TARGET_SETTLE_S + EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S
    simulated_min_s = (
        None
        if primary_time_s is None
        else primary_time_s - EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S
    )
    simulated_max_s = (
        None
        if primary_time_s is None
        else primary_time_s + EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S
    )
    matched = (
        motion_observed
        and primary_time_s is not None
        and simulated_min_s is not None
        and simulated_max_s is not None
        # Both the phone observation and simulator observation are sampled at
        # 24 fps.  Their uncertainty intervals overlap when the two recordings
        # cannot resolve a timing difference; do not demand sub-frame precision.
        and simulated_min_s <= target_max_s
        and target_min_s <= simulated_max_s
    )
    threshold_result = {
        f"{threshold * 1000.0:.1f}_mm": threshold_onset_times_s[threshold]
        for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M
    }
    threshold_confirmation_result = {
        f"{threshold * 1000.0:.1f}_mm": threshold_confirmation_times_s[threshold]
        for threshold in EDGE_RELEASE_OBSERVATION_THRESHOLDS_M
    }
    return (
        "R2_S1_EDGE_RELEASE_CALIBRATION_MATCH" if matched else "R2_S1_EDGE_RELEASE_CALIBRATION_RECORDED_NOT_MATCHED",
        {
            "lift_height_m": EDGE_RELEASE_LIFT_HEIGHT_M,
            "lift_preparation_s": EDGE_RELEASE_LIFT_PREPARATION_S,
            "holder_drive": {
                "type": "world_prismatic_force_drive",
                "stiffness_n_m": HOLDER_DRIVE_STIFFNESS_N_M,
                "damping_n_s_m": HOLDER_DRIVE_DAMPING_N_S_M,
                "maximum_force_n": HOLDER_DRIVE_MAX_FORCE_N,
            },
            "attachment": "two_3x3_corner_patches_on_one_edge",
            "selected_node_count": len(indices),
            "minimum_selected_patch_lift_m": minimum_patch_lift_m,
            "minimum_measured_holder_lift_m": measured_holder_lift_m,
            "requested_patch_lift_m": EDGE_RELEASE_LIFT_HEIGHT_M,
            "release_reactivated_by_identical_zero_velocity_nodal_state_write": True,
            "video_fps": VIDEO_FPS,
            "video_sample_steps": VIDEO_SAMPLE_STEPS,
            "target_settle_time_s": EDGE_RELEASE_TARGET_SETTLE_S,
            "target_timing_resolution_s": EDGE_RELEASE_TARGET_TIMING_RESOLUTION_S,
            "target_window_s": [target_min_s, target_max_s],
            "simulated_primary_window_s": [simulated_min_s, simulated_max_s],
            "observable": "maximum_across_environments_of_node_displacement_p95_per_video_frame",
            "primary_displacement_threshold_m": EDGE_RELEASE_PRIMARY_THRESHOLD_M,
            "consecutive_frames_required": EDGE_RELEASE_CONSECUTIVE_VIDEO_FRAMES,
            "settle_time_by_displacement_threshold_s": threshold_result,
            "confirmation_time_by_displacement_threshold_s": threshold_confirmation_result,
            "settle_time_definition": "first_quiet_frame_in_confirmed_consecutive_run",
            "match_rule": "target_and_simulated_24_fps_uncertainty_intervals_overlap",
            "frame_samples": frame_samples,
            "motion_observed": motion_observed,
            "pre_attachment_settle_time_s": pre_attach_settled_step * PHYSICS_DT_S,
            "held_settle_time_s": held_settled_step * PHYSICS_DT_S,
            "held_shape_observable": "maximum_environment_node_displacement_p95_per_video_frame",
            "held_shape_frame_samples": held_frame_samples,
            "held_final_maximum_vertex_speed_m_s_diagnostic_only": held_speed,
            "final_maximum_speed_m_s": final_speed_m_s,
            "minimum_nonlocal_node_separation_m": minimum_nonlocal_node_separation_m(final),
            "matched": matched,
        },
    )


def run() -> int:
    environment_count = int(source["environment_count"])
    print(
        f"S1_MATERIAL_CALIBRATION_START experiment={args.experiment} "
        f"environments={environment_count} youngs_modulus_pa={YOUNGS_MODULUS_PA:.6g} "
        f"elasticity_damping={ELASTICITY_DAMPING:.6g} "
        f"surface_bend_stiffness_pa={SURFACE_BEND_STIFFNESS_PA:.6g} "
        f"bend_damping_s_inv={BEND_DAMPING_S_INV:.6g} "
        f"linear_damping_s_inv={LINEAR_DAMPING_S_INV:.6g} motion_commands=0",
        flush=True,
    )
    sim = SimulationContext(
        sim_utils.SimulationCfg(
            dt=PHYSICS_DT_S,
            device=args.device,
            physics=PhysxCfg(
                enable_external_forces_every_iteration=True,
                enable_enhanced_determinism=True,
            ),
        )
    )
    scene = InteractiveScene(scene_config())
    sim.set_camera_view(
        eye=(table_pose[0] + 0.58, table_pose[1] + 0.48, 0.42),
        target=(table_pose[0] + 0.05, table_pose[1], table_top_z_m),
    )
    apply_shape_contact_offsets(environment_count)
    holder_drive_targets = (
        author_holder_prismatic_drives(scene)
        if args.experiment == "edge-release"
        else []
    )
    sim.reset()
    scene.reset()
    scene.write_data_to_sim()
    sim.step()
    scene.update(PHYSICS_DT_S)
    cloth = scene["cloth"]
    holder = scene["holder"]
    initial = local_nodes(scene, cloth).clone()
    if not torch.all(torch.isfinite(initial)):
        raise RuntimeError("calibration cloth initial state is non-finite")

    initial_holder_local = torch.tensor(
        holder_position, dtype=torch.float32, device=scene.device
    )
    if args.experiment == "cantilever":
        parked_local = initial_holder_local.clone()
        parked_local[2] += 0.5
        status, observation = run_cantilever(
            sim,
            scene,
            cloth,
            holder,
            holder_root_pose(scene, parked_local),
            initial,
        )
    else:
        status, observation = run_edge_release(
            sim,
            scene,
            cloth,
            holder,
            initial_holder_local,
            holder_drive_targets,
            initial,
        )

    result = {
        "schema_version": 1,
        "record_kind": "towel_isaac_s1_material_calibration_result",
        "status": status,
        "experiment": args.experiment,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": environment_count,
        "device": str(sim.device),
        "identity": source["identity"],
        "source_status": manifest["status"],
        "material": {
            "config_path": str(material.config_path),
            "config_sha256": material.config_sha256,
            "overrides": {
                "youngs_modulus_pa": YOUNGS_MODULUS_PA,
                "elasticity_damping": ELASTICITY_DAMPING,
                "surface_bend_stiffness_pa": SURFACE_BEND_STIFFNESS_PA,
                "bend_damping_s_inv": BEND_DAMPING_S_INV,
                "linear_damping_s_inv": LINEAR_DAMPING_S_INV,
                "settling_damping_s_inv": SETTLING_DAMPING_S_INV,
            },
        },
        "simulation": {
            "physics_dt_s": PHYSICS_DT_S,
            "cloth_size_xy_m": list(CLOTH_SIZE_XY_M),
            "cloth_resolution": list(CLOTH_RESOLUTION),
            "node_count": int(cloth.max_sim_vertices_per_body),
            "self_collision": args.experiment == "edge-release",
            "enhanced_determinism": True,
            "external_forces_every_iteration": True,
        },
        "observation": observation,
        "completion_claim": {
            "calibration_observation_recorded": True,
            "material_physical_fidelity_validated": False,
            "s1_completed": False,
            "blocking_reason": "held_out_place_release_self_contact_gate_not_run",
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{status} experiment={args.experiment} motion_commands=0 output={args.output}",
        flush=True,
    )
    if args.keep_open:
        print("S1_MATERIAL_CALIBRATION_GUI_KEEP_OPEN", flush=True)
        while simulation_app.is_running():
            sim.step()
            scene.update(PHYSICS_DT_S)
    return 0


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S1_MATERIAL_CALIBRATION_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
