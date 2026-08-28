#!/usr/bin/env python3
"""Attach two surface-cloth vertex patches to r0g grippers and lift them."""

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
parser.add_argument("--lift-seconds", type=float, default=1.0)
parser.add_argument(
    "--place-release",
    action="store_true",
    help="continue through the first-fold laydown, detach, retreat, and settle gate",
)
parser.add_argument(
    "--self-contact",
    action="store_true",
    help="enable cloth self-collision and gate nonlocal vertex separation",
)
parser.add_argument("--fold-phase-seconds", type=float, default=0.20)
parser.add_argument("--retreat-seconds", type=float, default=1.0)
parser.add_argument("--keep-open", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if not math.isfinite(args.settle_timeout_s) or args.settle_timeout_s <= 0.0:
    parser.error("--settle-timeout-s must be finite and positive")
if not math.isfinite(args.lift_seconds) or args.lift_seconds <= 0.0:
    parser.error("--lift-seconds must be finite and positive")
if not math.isfinite(args.fold_phase_seconds) or args.fold_phase_seconds <= 0.0:
    parser.error("--fold-phase-seconds must be finite and positive")
if not math.isfinite(args.retreat_seconds) or args.retreat_seconds <= 0.0:
    parser.error("--retreat-seconds must be finite and positive")
if args.self_contact and not args.place_release:
    parser.error("--self-contact requires --place-release")

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass
from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


PASS_STATUS = "S1_ISAACLAB_VERTEX_PATCH_LIFT_PASS_MATERIAL_UNCOMMISSIONED"
PLACE_RELEASE_PASS_STATUS = (
    "S1_ISAACLAB_VERTEX_PATCH_PLACE_RELEASE_PASS_"
    "MATERIAL_UNCOMMISSIONED_SELF_COLLISION_NOT_RUN"
)
SELF_CONTACT_PLACE_RELEASE_PASS_STATUS = (
    "S1_ISAACLAB_VERTEX_PATCH_PLACE_RELEASE_SELF_CONTACT_PASS_"
    "MATERIAL_UNCOMMISSIONED_FULL_SHAPE_DETERMINISM_NOT_PASSED"
)
PHYSICS_DT_S = 1.0 / 120.0
SELF_CONTACT_PHYSICS_DT_S = 1.0 / 240.0
ENVIRONMENT_SPACING_M = 1.0
CLOTH_SIZE_XY_M = (0.300, 0.300)
CLOTH_RESOLUTION = (31, 31)
CLOTH_MASS_KG = 0.100
CLOTH_DENSITY_KG_M3 = 1000.0
CLOTH_STATIC_FRICTION = 0.50
CLOTH_DYNAMIC_FRICTION = 0.40
CLOTH_YOUNGS_MODULUS_PA = 1.0e6
CLOTH_POISSONS_RATIO = 0.30
CLOTH_ELASTICITY_DAMPING = 0.05
CLOTH_SURFACE_THICKNESS_M = 0.003
CLOTH_CONTACT_OFFSET_M = 0.003
CLOTH_REST_OFFSET_M = 0.0015
CLOTH_LINEAR_DAMPING_S_INV = 0.10
CLOTH_SETTLING_DAMPING_S_INV = 1.0
CLOTH_SETTLING_THRESHOLD_M_S = 0.02
PATCH_MASK_RADIUS_M = 0.016
MINIMUM_PATCH_POINT_COUNT = 4
MINIMUM_LIFT_M = 0.003
MAXIMUM_ATTACHMENT_SNAP_M = 0.005
MAXIMUM_PATCH_FOLLOW_ERROR_M = 0.003
MAXIMUM_ATTACHMENT_PATCH_ENVIRONMENT_DIVERGENCE_M = 5.0e-4
MAXIMUM_PLACE_RELEASE_ENVIRONMENT_DIVERGENCE_M = 0.020
MAXIMUM_FINAL_TABLE_PENETRATION_M = 0.002
MAXIMUM_FINAL_CLOTH_HEIGHT_M = 0.030
MAXIMUM_RELEASE_PATCH_LIFT_M = 0.015
MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M = 0.015
MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M = 5.0e-4
SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD = 2
SELF_COLLISION_FILTER_DISTANCE_M = (
    SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD * CLOTH_SIZE_XY_M[0] / CLOTH_RESOLUTION[0]
)
GRIPPER_FRAME_TRANSLATION_M = (-0.0079, -0.000218121, -0.0981274)
PINCH_GRIPPER_JOINT_POSITIONS_RAD = {"left": 0.251573, "right": 0.188680}
PINCH_GAP_CENTER_TCP_X_M = {"left": -0.0107, "right": -0.0084}
MAXIMUM_JAW_TARGET_PATCH_CENTER_XY_DISTANCE_M = 0.008
MAXIMUM_ATTACHMENT_POINT_TCP_DISTANCE_M = 0.030
MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M = 0.005
SETTLE_SPEED_THRESHOLD_M_S = 0.010
RELEASE_SETTLE_SPEED_THRESHOLD_M_S = 0.015
SETTLE_CONSECUTIVE_STEPS = 30
ROS_PACKAGE = ROOT / "ros2_ws/src/so101_description"
CONTACT_GRIPPER_LOCAL_POSES = {
    "left": {
        "position": (0.1742753983, 0.0100203753, 0.0975446180),
        "orientation_xyzw": (0.0189957153, -0.2726267576, 0.0053933416, 0.9619171023),
    },
    "right": {
        "position": (0.1945362091, -0.2493891716, 0.1050371006),
        "orientation_xyzw": (-0.0337128118, -0.1653918773, -0.0056712483, 0.9856351614),
    },
}
CONTACT_GRIPPER_POSE_TOLERANCE_M = 1.0e-4
SELF_CONTACT_GRIPPER_POSE_TOLERANCE = 2.0e-4


def load_manifest(path: Path) -> tuple[dict[str, object], dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    return document, validate_s0_host_manifest(document)


def phase(source: dict[str, object], name: str) -> dict[str, object]:
    for record in source["canonical_replay"]["first_fold"]:
        if record["name"] == name:
            return record
    raise ValueError(f"missing canonical replay phase: {name}")


def scene_config(source: dict[str, object]) -> InteractiveSceneCfg:
    table_geometry = source["worktable_geometry"]
    table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
    table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])
    proxy_pose = source["rigid_proxy_pose_xyz_yaw_rad"][0]
    table_top_z_m = table_pose[2] + 0.5 * table_size[2]
    contact_joint_positions = phase(source, "first_contact")["joint_positions_rad"]
    contact_joint_map = {
        str(name): float(position)
        for name, position in zip(source["joint_names"], contact_joint_positions, strict=True)
    }

    @configclass
    class TowelS1VertexPatchSceneCfg(InteractiveSceneCfg):
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
                    mass=CLOTH_MASS_KG,
                    solver_position_iteration_count=24,
                    linear_damping=CLOTH_LINEAR_DAMPING_S_INV,
                    settling_damping=CLOTH_SETTLING_DAMPING_S_INV,
                    settling_threshold=CLOTH_SETTLING_THRESHOLD_M_S,
                    sleep_threshold=0.005,
                    max_depenetration_velocity=0.5,
                    # This runner isolates direct gripper coupling. Self-contact is
                    # commissioned separately with place/fold, where it is observable.
                    self_collision=args.self_contact,
                    self_collision_filter_distance=(
                        SELF_COLLISION_FILTER_DISTANCE_M
                        if args.self_contact
                        else None
                    ),
                    contact_offset=CLOTH_CONTACT_OFFSET_M,
                    rest_offset=CLOTH_REST_OFFSET_M,
                    collision_pair_update_frequency=4,
                    collision_iteration_multiplier=2.0,
                ),
                physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
                    density=CLOTH_DENSITY_KG_M3,
                    static_friction=CLOTH_STATIC_FRICTION,
                    dynamic_friction=CLOTH_DYNAMIC_FRICTION,
                    youngs_modulus=CLOTH_YOUNGS_MODULUS_PA,
                    poissons_ratio=CLOTH_POISSONS_RATIO,
                    elasticity_damping=CLOTH_ELASTICITY_DAMPING,
                    surface_thickness=CLOTH_SURFACE_THICKNESS_M,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.45, 0.85)
                ),
            ),
            init_state=DeformableObjectCfg.InitialStateCfg(
                pos=(
                    float(proxy_pose[0]),
                    float(proxy_pose[1]),
                    table_top_z_m + CLOTH_REST_OFFSET_M,
                )
            ),
        )
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(source["urdf_path"]),
                fix_base=True,
                merge_fixed_joints=True,
                make_instanceable=False,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
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
                        stiffness=1000.0, damping=100.0
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
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos=contact_joint_map,
                joint_vel={".*": 0.0},
            ),
        )

    return TowelS1VertexPatchSceneCfg(
        num_envs=int(source["environment_count"]),
        env_spacing=ENVIRONMENT_SPACING_M,
        # Isaac Sim 6.0.1 reports surface-deformable replication as unsupported.
        replicate_physics=False,
    )


def _author_jaw_pad(
    stage: Usd.Stage,
    path: str,
    center_parent_m: tuple[float, float, float],
    size_parent_m: tuple[float, float, float],
) -> None:
    """Author an invisible collision pad over a reviewed jaw-mesh contact face."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center_parent_m))
    cube.AddScaleOp().Set(Gf.Vec3f(*size_parent_m))
    cube.MakeInvisible()
    collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision.CreateCollisionEnabledAttr().Set(True)
    api = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    api.CreateContactOffsetAttr().Set(0.002)
    api.CreateRestOffsetAttr().Set(0.0)


def apply_shape_contact_offsets(environment_count: int) -> None:
    stage = omni.usd.get_context().get_stage()
    gripper_frame_rotation = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)
    for environment_index in range(environment_count):
        for path, contact_offset, rest_offset in (
            (f"/World/envs/env_{environment_index}/Table/geometry/mesh", 0.002, 0.0),
            (
                f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh",
                CLOTH_CONTACT_OFFSET_M,
                CLOTH_REST_OFFSET_M,
            ),
        ):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"missing collision shape: {path}")
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr().Set(contact_offset)
            api.CreateRestOffsetAttr().Set(rest_offset)
        robot_prefix = f"/World/envs/env_{environment_index}/Robot"
        for side in ("left", "right"):
            # Fixed-jaw face: TCP x=0, raw camera-mount mesh x=-7.9 mm.
            fixed_center_tcp = Gf.Vec3d(0.001, 0.0, 0.001)
            fixed_center_parent = Gf.Vec3d(
                *GRIPPER_FRAME_TRANSLATION_M
            ) + gripper_frame_rotation.TransformDir(fixed_center_tcp)
            _author_jaw_pad(
                stage,
                f"{robot_prefix}/{side}_gripper_link/TowelFixedJawCollider",
                tuple(fixed_center_parent),
                (0.002, 0.010, 0.010),
            )
            # Moving-jaw face: raw mesh x=-12.3 mm at its distal tip.
            _author_jaw_pad(
                stage,
                f"{robot_prefix}/{side}_moving_jaw_link/TowelMovingJawCollider",
                (-0.0113, -0.0765, 0.0189),
                (0.002, 0.009, 0.0095),
            )


def rigid_gripper_paths(environment_index: int) -> dict[str, str]:
    stage = omni.usd.get_context().get_stage()
    prefix = f"/World/envs/env_{environment_index}/Robot"
    result: dict[str, str] = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        for side in ("left", "right"):
            if prim.GetName() == f"{side}_gripper_link":
                result[side] = path
    if set(result) != {"left", "right"}:
        raise RuntimeError(f"could not resolve both rigid gripper links: {result}")
    return result


def author_contact_joint_states(source: dict[str, object], environment_count: int) -> None:
    """Set articulation state before PhysX creates attachment actor frames."""
    stage = omni.usd.get_context().get_stage()
    contact_positions = phase(source, "first_contact")["joint_positions_rad"]
    expected = dict(zip(source["joint_names"], contact_positions, strict=True))
    for environment_index in range(environment_count):
        prefix = f"/World/envs/env_{environment_index}/Robot"
        authored = set()
        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith(prefix):
                continue
            name = prim.GetName()
            if name not in expected or not prim.IsA(UsdPhysics.RevoluteJoint):
                continue
            position_degrees = math.degrees(float(expected[name]))
            PhysxSchema.JointStateAPI.Apply(prim, "angular").CreatePositionAttr().Set(
                position_degrees
            )
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.GetTargetPositionAttr().Set(position_degrees)
            authored.add(name)
        if authored != set(expected):
            raise RuntimeError(
                f"failed to author contact joint states in env {environment_index}: "
                f"{sorted(set(expected) - authored)}"
            )


def filter_non_gripper_robot_cloth_collisions(environment_count: int) -> None:
    """Allow jaw contact while excluding arm contact from this grasp smoke."""
    stage = omni.usd.get_context().get_stage()
    for environment_index in range(environment_count):
        cloth_path = Sdf.Path(
            f"/World/envs/env_{environment_index}/TowelCloth"
        )
        cloth_prim = stage.GetPrimAtPath(cloth_path)
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(cloth_prim)
        relationship = filtered_pairs.CreateFilteredPairsRel()
        robot_prefix = f"/World/envs/env_{environment_index}/Robot"
        targets = [
            prim.GetPath()
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(robot_prefix)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and prim.GetName()
            not in {
                "left_gripper_link",
                "left_moving_jaw_link",
                "right_gripper_link",
                "right_moving_jaw_link",
            }
        ]
        if not targets:
            raise RuntimeError(f"no robot rigid bodies found in env {environment_index}")
        relationship.SetTargets(targets)


def gripper_tcp_positions_w(
    gripper_positions_w: torch.Tensor,
    gripper_orientations_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Return the registered gripper-frame origins from rigid-link poses."""
    result = torch.empty_like(gripper_positions_w)
    for environment_index in range(gripper_positions_w.shape[0]):
        for side_index in range(gripper_positions_w.shape[1]):
            position = gripper_positions_w[environment_index, side_index].tolist()
            orientation_xyzw = gripper_orientations_xyzw[
                environment_index, side_index
            ].tolist()
            rotation = Gf.Rotation(
                Gf.Quatd(
                    orientation_xyzw[3], Gf.Vec3d(*orientation_xyzw[:3])
                )
            )
            offset = rotation.TransformDir(Gf.Vec3d(*GRIPPER_FRAME_TRANSLATION_M))
            result[environment_index, side_index] = torch.tensor(
                [position[index] + offset[index] for index in range(3)],
                dtype=result.dtype,
                device=result.device,
            )
    return result


def gripper_jaw_target_positions_w(
    gripper_positions_w: torch.Tensor,
    gripper_orientations_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Return cloth targets at the measured center of each pinched jaw gap."""
    result = torch.empty_like(gripper_positions_w)
    frame_rotation = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)
    for environment_index in range(gripper_positions_w.shape[0]):
        for side_index, side in enumerate(("left", "right")):
            position = gripper_positions_w[environment_index, side_index].tolist()
            orientation_xyzw = gripper_orientations_xyzw[
                environment_index, side_index
            ].tolist()
            link_rotation = Gf.Rotation(
                Gf.Quatd(orientation_xyzw[3], Gf.Vec3d(*orientation_xyzw[:3]))
            )
            target_parent = Gf.Vec3d(
                *GRIPPER_FRAME_TRANSLATION_M
            ) + frame_rotation.TransformDir(
                Gf.Vec3d(PINCH_GAP_CENTER_TCP_X_M[side], 0.0, 0.0)
            )
            offset = link_rotation.TransformDir(target_parent)
            result[environment_index, side_index] = torch.tensor(
                [position[index] + offset[index] for index in range(3)],
                dtype=result.dtype,
                device=result.device,
            )
    return result


def author_runtime_attachments(
    environment_count: int,
    nodes_w: torch.Tensor,
    gripper_positions_w: torch.Tensor,
    gripper_orientations_xyzw: torch.Tensor,
    selected_indices: list[list[list[int]]],
) -> None:
    """Author settled-node constraints in the registered jaw/TCP frame."""
    stage = omni.usd.get_context().get_stage()
    gripper_frame_rotation = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)
    for environment_index in range(environment_count):
        gripper_paths = rigid_gripper_paths(environment_index)
        for side_index, side in enumerate(("left", "right")):
            position = gripper_positions_w[environment_index, side_index].tolist()
            orientation_xyzw = gripper_orientations_xyzw[
                environment_index, side_index
            ].tolist()
            inverse_rotation = Gf.Rotation(
                Gf.Quatd(orientation_xyzw[3], Gf.Vec3d(*orientation_xyzw[:3]))
            ).GetInverse()
            indices = selected_indices[environment_index][side_index]
            frame_local_positions = []
            for node_index in indices:
                node = nodes_w[environment_index, node_index].tolist()
                body_local = inverse_rotation.TransformDir(
                    Gf.Vec3d(*node) - Gf.Vec3d(*position)
                )
                frame_local_positions.append(
                    gripper_frame_rotation.GetInverse().TransformDir(
                        body_local - Gf.Vec3d(*GRIPPER_FRAME_TRANSLATION_M)
                    )
                )
            frame_path = Sdf.Path(gripper_paths[side]).AppendChild(
                "TowelAttachmentFrame"
            )
            frame = UsdGeom.Xform.Define(stage, frame_path)
            frame.AddTranslateOp().Set(Gf.Vec3d(*GRIPPER_FRAME_TRANSLATION_M))
            frame.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 1.0, 0.0)))
            local_positions = [
                Gf.Vec3f(value) for value in frame_local_positions
            ]
            attachment_path = Sdf.Path(
                f"/World/envs/env_{environment_index}/Attachments/{side}_gripper_patch"
            )
            prim = stage.DefinePrim(attachment_path, "OmniPhysicsVtxXformAttachment")
            prim.GetAttribute("omniphysics:attachmentEnabled").Set(True)
            prim.GetRelationship("omniphysics:src0").SetTargets(
                [Sdf.Path(f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh")]
            )
            prim.GetRelationship("omniphysics:src1").SetTargets(
                [frame_path]
            )
            prim.GetAttribute("omniphysics:vtxIndicesSrc0").Set(indices)
            prim.GetAttribute("omniphysics:localPositionsSrc1").Set(local_positions)


def create_attachments(
    environment_count: int,
) -> tuple[list[dict[str, object]], list[list[list[int]]]]:
    stage = omni.usd.get_context().get_stage()
    records = []
    selected_indices: list[list[list[int]]] = []
    for environment_index in range(environment_count):
        environment_indices = []
        for side_index, side in enumerate(("left", "right")):
            attachment_scope_path = Sdf.Path(
                f"/World/envs/env_{environment_index}/Attachments/{side}_gripper_patch"
            )
            scope_prim = stage.GetPrimAtPath(attachment_scope_path)
            low_level_prims = [
                candidate
                for candidate in Usd.PrimRange(scope_prim)
                if candidate.GetTypeName() == "OmniPhysicsVtxXformAttachment"
            ]
            if len(low_level_prims) != 1:
                raise RuntimeError(
                    f"expected one cooked vertex attachment under {attachment_scope_path}, "
                    f"found {len(low_level_prims)}"
                )
            prim = low_level_prims[0]
            source1_targets = prim.GetRelationship("omniphysics:src1").GetTargets()
            if len(source1_targets) != 1:
                raise RuntimeError(f"attachment has invalid src1: {prim.GetPath()}")
            target_path = source1_targets[0]
            authored_indices = list(prim.GetAttribute("omniphysics:vtxIndicesSrc0").Get())
            local_positions = prim.GetAttribute("omniphysics:localPositionsSrc1").Get()
            indices = [int(value) for value in authored_indices]
            if len(local_positions) != len(indices):
                raise RuntimeError(
                    f"cooked patch arrays differ: {prim.GetPath()}"
                )
            environment_indices.append(indices)
            if prim.GetAttribute("omniphysics:attachmentEnabled").Get() is not True:
                raise RuntimeError(f"cooked attachment is not enabled: {prim.GetPath()}")
            target_prim = stage.GetPrimAtPath(target_path)
            rigid_ancestor = target_prim
            while rigid_ancestor and not rigid_ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_ancestor = rigid_ancestor.GetParent()
            if not rigid_ancestor or not rigid_ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f"attachment target has no rigid ancestor: {target_path}")
            records.append(
                {
                    "environment_index": environment_index,
                    "side": side,
                    "attachment_path": str(prim.GetPath()),
                    "gripper_path": str(rigid_ancestor.GetPath()),
                    "attachment_frame_path": str(target_path),
                    "selected_patch_point_count": len(indices),
                    "vertex_indices": indices,
                    "point_count": len(local_positions),
                }
            )
        selected_indices.append(environment_indices)
    return records, selected_indices


def local_nodes(scene: InteractiveScene, cloth: object) -> torch.Tensor:
    return cloth.data.nodal_pos_w.torch - scene.env_origins[:, None, :]


def disable_runtime_attachments(records: list[dict[str, object]]) -> None:
    """Disable every cooked vertex attachment at the laydown release gate."""
    stage = omni.usd.get_context().get_stage()
    for record in records:
        path = str(record["attachment_path"])
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"missing attachment at release: {path}")
        prim.GetAttribute("omniphysics:attachmentEnabled").Set(False)


def runtime_attachments_are_disabled(records: list[dict[str, object]]) -> bool:
    stage = omni.usd.get_context().get_stage()
    return all(
        stage.GetPrimAtPath(str(record["attachment_path"]))
        .GetAttribute("omniphysics:attachmentEnabled")
        .Get()
        is False
        for record in records
    )


def rigidly_transformed_points_w(
    points_before_w: torch.Tensor,
    body_position_before_w: torch.Tensor,
    body_orientation_before_xyzw: torch.Tensor,
    body_position_after_w: torch.Tensor,
    body_orientation_after_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Predict attached world points including rigid-link translation and rotation."""
    before_rotation = Gf.Rotation(
        Gf.Quatd(
            float(body_orientation_before_xyzw[3]),
            Gf.Vec3d(*[float(value) for value in body_orientation_before_xyzw[:3]]),
        )
    )
    after_rotation = Gf.Rotation(
        Gf.Quatd(
            float(body_orientation_after_xyzw[3]),
            Gf.Vec3d(*[float(value) for value in body_orientation_after_xyzw[:3]]),
        )
    )
    before_position = Gf.Vec3d(
        *[float(value) for value in body_position_before_w]
    )
    after_position = Gf.Vec3d(*[float(value) for value in body_position_after_w])
    expected = []
    for point in points_before_w:
        point_w = Gf.Vec3d(*[float(value) for value in point])
        body_local = before_rotation.GetInverse().TransformDir(
            point_w - before_position
        )
        transformed = after_position + after_rotation.TransformDir(body_local)
        expected.append([transformed[index] for index in range(3)])
    return torch.tensor(
        expected, dtype=points_before_w.dtype, device=points_before_w.device
    )


def minimum_nonlocal_node_separation_m(nodes: torch.Tensor) -> float:
    """Return the closest vertex pair outside a small mesh-topology neighborhood."""
    node_count = int(nodes.shape[1])
    side_nodes = int(round(math.sqrt(node_count)))
    if side_nodes * side_nodes != node_count:
        raise RuntimeError(f"cloth node grid is not square: {node_count}")
    flat_indices = torch.arange(node_count, device=nodes.device)
    rows = torch.div(flat_indices, side_nodes, rounding_mode="floor")
    columns = flat_indices % side_nodes
    topology_neighbor = (
        (torch.abs(rows[:, None] - rows[None, :]) <= SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD)
        & (
            torch.abs(columns[:, None] - columns[None, :])
            <= SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD
        )
    )
    minimum = math.inf
    for environment_nodes in nodes:
        distances = torch.cdist(environment_nodes, environment_nodes)
        distances[topology_neighbor] = math.inf
        minimum = min(minimum, float(torch.min(distances).item()))
    return minimum


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    environment_count = int(source["environment_count"])
    physics_dt_s = SELF_CONTACT_PHYSICS_DT_S if args.self_contact else PHYSICS_DT_S
    contact = phase(source, "first_contact")
    lift = phase(source, "first_fold_01")
    print(
        f"S1_VERTEX_PATCH_START environments={environment_count} "
        "attachment=surface_to_direct_gripper_links motion_commands=0",
        flush=True,
    )
    sim = SimulationContext(sim_utils.SimulationCfg(dt=physics_dt_s, device=args.device))
    scene = InteractiveScene(scene_config(source))
    first_origin = scene.env_origins[0].detach().cpu().tolist()
    sim.set_camera_view(
        eye=(first_origin[0] + 0.72, first_origin[1] + 0.48, 0.48),
        target=(first_origin[0] + 0.32, first_origin[1] - 0.12, 0.02),
    )
    apply_shape_contact_offsets(environment_count)
    author_contact_joint_states(source, environment_count)
    filter_non_gripper_robot_cloth_collisions(environment_count)
    simulation_app.update()
    sim.reset()
    robot = scene["robot"]
    cloth = scene["cloth"]
    joint_ids, imported_names = robot.find_joints(source["joint_names"], preserve_order=True)
    if imported_names != source["joint_names"] or len(joint_ids) != 12:
        raise RuntimeError("imported articulation does not match canonical 12-joint order")
    contact_row = torch.tensor(
        contact["joint_positions_rad"], dtype=torch.float32, device=sim.device
    ).repeat(environment_count, 1)
    pinch_row = contact_row.clone()
    pinch_row[:, 5] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["left"]
    pinch_row[:, 11] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["right"]
    zero_velocity = torch.zeros_like(contact_row)
    scene.reset()
    robot.write_joint_state_to_sim_index(
        position=contact_row, velocity=zero_velocity, joint_ids=joint_ids
    )
    robot.set_joint_position_target_index(target=contact_row, joint_ids=joint_ids)
    scene.write_data_to_sim()
    sim.step()
    scene.update(physics_dt_s)

    stage = omni.usd.get_context().get_stage()
    free_node_mask = torch.ones(1024, dtype=torch.bool, device=sim.device)

    settled_run = 0
    settled_step = None
    maximum_steps = math.ceil(args.settle_timeout_s / physics_dt_s)
    for step in range(1, maximum_steps + 1):
        robot.write_joint_state_to_sim_index(
            position=contact_row, velocity=zero_velocity, joint_ids=joint_ids
        )
        robot.set_joint_position_target_index(target=contact_row, joint_ids=joint_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
        speed = float(
            torch.max(
                torch.linalg.vector_norm(
                    cloth.data.nodal_vel_w.torch[:, free_node_mask], dim=-1
                )
            ).item()
        )
        if speed <= SETTLE_SPEED_THRESHOLD_M_S:
            settled_run += 1
        else:
            settled_run = 0
        if settled_run >= SETTLE_CONSECUTIVE_STEPS:
            settled_step = step
            break
    if settled_step is None:
        raise RuntimeError(
            f"cloth did not settle before attachment; final maximum speed={speed:.6f} m/s"
        )
    settled_nodes_w = cloth.data.nodal_pos_w.torch.clone()

    pinch_steps = 30
    for step in range(1, pinch_steps + 1):
        alpha = step / pinch_steps
        target = contact_row + alpha * (pinch_row - contact_row)
        robot.write_joint_state_to_sim_index(
            position=target, velocity=zero_velocity, joint_ids=joint_ids
        )
        robot.set_joint_position_target_index(target=target, joint_ids=joint_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
    for _ in range(15):
        robot.write_joint_state_to_sim_index(
            position=pinch_row, velocity=zero_velocity, joint_ids=joint_ids
        )
        robot.set_joint_position_target_index(target=pinch_row, joint_ids=joint_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)

    gripper_body_ids, body_names = robot.find_bodies(
        ["left_gripper_link", "right_gripper_link"], preserve_order=True
    )
    if body_names != ["left_gripper_link", "right_gripper_link"]:
        raise RuntimeError(f"unexpected gripper body map: {body_names}")
    gripper_before_w = robot.data.body_pos_w.torch[:, gripper_body_ids].clone()
    gripper_orientation_before_xyzw = robot.data.body_quat_w.torch[
        :, gripper_body_ids
    ].clone()
    gripper_tcp_before_w = gripper_tcp_positions_w(
        gripper_before_w, gripper_orientation_before_xyzw
    )
    gripper_jaw_target_before_w = gripper_jaw_target_positions_w(
        gripper_before_w, gripper_orientation_before_xyzw
    )
    print(
        "S1_VERTEX_PATCH_MEASURED_CONTACT_POSES "
        + json.dumps(
            {
                side: {
                    "position_local_m": [
                        float(value)
                        for value in (
                            gripper_before_w[0, index] - scene.env_origins[0]
                        ).tolist()
                    ],
                    "orientation_xyzw": [
                        float(value)
                        for value in gripper_orientation_before_xyzw[0, index].tolist()
                    ],
                }
                for index, side in enumerate(("left", "right"))
            },
            sort_keys=True,
        ),
        flush=True,
    )
    for side_index, side in enumerate(("left", "right")):
        contact_pose_tolerance = (
            SELF_CONTACT_GRIPPER_POSE_TOLERANCE
            if args.self_contact
            else CONTACT_GRIPPER_POSE_TOLERANCE_M
        )
        expected = CONTACT_GRIPPER_LOCAL_POSES[side]
        expected_position = torch.tensor(
            expected["position"], dtype=torch.float32, device=sim.device
        )
        measured_local_position = (
            gripper_before_w[:, side_index] - scene.env_origins
        )
        position_error = float(
            torch.max(torch.abs(measured_local_position - expected_position)).item()
        )
        expected_orientation = torch.tensor(
            expected["orientation_xyzw"], dtype=torch.float32, device=sim.device
        )
        measured_orientation = gripper_orientation_before_xyzw[:, side_index]
        orientation_error = float(
            torch.max(
                torch.minimum(
                    torch.linalg.vector_norm(measured_orientation - expected_orientation, dim=-1),
                    torch.linalg.vector_norm(measured_orientation + expected_orientation, dim=-1),
                )
            ).item()
        )
        if position_error > contact_pose_tolerance:
            raise RuntimeError(
                f"{side} contact position drift {position_error:.9f} m; "
                f"measured_env_0={measured_local_position[0].tolist()}"
            )
        if orientation_error > contact_pose_tolerance:
            raise RuntimeError(
                f"{side} contact orientation drift {orientation_error:.9f}; "
                f"measured_env_0={measured_orientation[0].tolist()}"
            )
    nodes_before = local_nodes(scene, cloth).clone()
    nodes_before_w = cloth.data.nodal_pos_w.torch.clone()
    maximum_pinch_induced_cloth_displacement_m = float(
        torch.max(
            torch.linalg.vector_norm(nodes_before_w - settled_nodes_w, dim=-1)
        ).item()
    )
    if (
        maximum_pinch_induced_cloth_displacement_m
        > MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M
    ):
        raise RuntimeError(
            "jaw closing displaced cloth before attachment by "
            f"{maximum_pinch_induced_cloth_displacement_m:.6f} m"
        )
    selected_indices = []
    jaw_target_patch_center_xy_distances_m = []
    attachment_point_tcp_distances_m = []
    for environment_index in range(environment_count):
        environment_indices = []
        for side_index in range(2):
            xy_delta = (
                nodes_before_w[environment_index, :, :2]
                - gripper_jaw_target_before_w[environment_index, side_index, :2]
            )
            center_index = int(
                torch.argmin(torch.sum(xy_delta * xy_delta, dim=-1)).item()
            )
            center = nodes_before_w[environment_index, center_index]
            jaw_target_patch_center_xy_distances_m.append(
                torch.linalg.vector_norm(
                    center[:2]
                    - gripper_jaw_target_before_w[environment_index, side_index, :2]
                )
            )
            indices = torch.nonzero(
                torch.linalg.vector_norm(
                    nodes_before_w[environment_index] - center, dim=-1
                )
                <= PATCH_MASK_RADIUS_M
            ).flatten().tolist()
            attachment_point_tcp_distances_m.append(
                torch.max(
                    torch.linalg.vector_norm(
                        nodes_before_w[environment_index, indices]
                        - gripper_tcp_before_w[environment_index, side_index],
                        dim=-1,
                    )
                )
            )
            environment_indices.append(indices)
        selected_indices.append(environment_indices)
    maximum_jaw_target_patch_center_xy_distance_m = float(
        torch.max(torch.stack(jaw_target_patch_center_xy_distances_m)).item()
    )
    maximum_attachment_point_tcp_distance_m = float(
        torch.max(torch.stack(attachment_point_tcp_distances_m)).item()
    )
    if (
        maximum_jaw_target_patch_center_xy_distance_m
        > MAXIMUM_JAW_TARGET_PATCH_CENTER_XY_DISTANCE_M
    ):
        raise RuntimeError(
            "nearest cloth patch is not centered in the measured jaw gap: "
            f"{maximum_jaw_target_patch_center_xy_distance_m:.6f} m"
        )
    if maximum_attachment_point_tcp_distance_m > MAXIMUM_ATTACHMENT_POINT_TCP_DISTANCE_M:
        raise RuntimeError(
            "attachment patch extends outside the jaw neighborhood: "
            f"{maximum_attachment_point_tcp_distance_m:.6f} m"
        )
    print(
        f"S1_VERTEX_PATCH_ENV0_INDICES left={selected_indices[0][0]} "
        f"right={selected_indices[0][1]}",
        flush=True,
    )
    minimum_selected_points = min(
        len(indices) for environment_indices in selected_indices for indices in environment_indices
    )
    if minimum_selected_points < MINIMUM_PATCH_POINT_COUNT:
        raise RuntimeError(
            f"vertex patch mask selected only {minimum_selected_points} nodes"
        )
    author_runtime_attachments(
        environment_count,
        nodes_before_w,
        gripper_before_w,
        gripper_orientation_before_xyzw,
        selected_indices,
    )
    simulation_app.update()
    attachment_records, authored_selected_indices = create_attachments(
        environment_count
    )
    if authored_selected_indices != selected_indices:
        raise RuntimeError("runtime attachment indices changed while authoring")
    print(
        "S1_VERTEX_PATCH_CONTACT_POSES "
        + json.dumps(
            {
                side: {
                    "gripper_link_position_w": [
                        float(value) for value in gripper_before_w[0, index].tolist()
                    ],
                    "registered_tcp_position_w": [
                        float(value) for value in gripper_tcp_before_w[0, index].tolist()
                    ],
                    "orientation_xyzw": [
                        float(value)
                        for value in gripper_orientation_before_xyzw[0, index].tolist()
                    ],
                }
                for index, side in enumerate(("left", "right"))
            },
            sort_keys=True,
        ),
        flush=True,
    )
    minimum_authored_points = min(record["point_count"] for record in attachment_records)
    print(
        f"S1_VERTEX_PATCH_ATTACHED attachments={len(attachment_records)} "
        f"minimum_selected_points={minimum_selected_points} "
        f"minimum_authored_points={minimum_authored_points}",
        flush=True,
    )

    for _ in range(2):
        robot.write_joint_state_to_sim_index(
            position=pinch_row, velocity=zero_velocity, joint_ids=joint_ids
        )
        robot.set_joint_position_target_index(target=pinch_row, joint_ids=joint_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
    nodes_after_attachment = local_nodes(scene, cloth).clone()
    maximum_attachment_snap_m = float(
        torch.max(
            torch.linalg.vector_norm(nodes_after_attachment - nodes_before, dim=-1)
        ).item()
    )
    if maximum_attachment_snap_m > MAXIMUM_ATTACHMENT_SNAP_M:
        first_index = selected_indices[0][0][0]
        first_before = nodes_before[0, first_index]
        first_after = nodes_after_attachment[0, first_index]
        selected_snap = float(
            torch.max(
                torch.linalg.vector_norm(
                    nodes_after_attachment[0, selected_indices[0][0]]
                    - nodes_before[0, selected_indices[0][0]],
                    dim=-1,
                )
            ).item()
        )
        env0_left_prim = stage.GetPrimAtPath(attachment_records[0]["attachment_path"])
        print(
            "S1_VERTEX_PATCH_FRAME_DIAGNOSTIC "
            + json.dumps(
                {
                    "local_positions": [
                        list(value)
                        for value in env0_left_prim.GetAttribute(
                            "omniphysics:localPositionsSrc1"
                        ).Get()
                    ],
                    "before_w": (
                        nodes_before[0, selected_indices[0][0]] + scene.env_origins[0]
                    ).tolist(),
                    "after_w": (
                        nodes_after_attachment[0, selected_indices[0][0]]
                        + scene.env_origins[0]
                    ).tolist(),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        raise RuntimeError(
            f"attachment snap {maximum_attachment_snap_m:.6f} m exceeds "
            f"{MAXIMUM_ATTACHMENT_SNAP_M:.6f} m; "
            f"selected_snap={selected_snap:.6f} m "
            f"first_before={first_before.tolist()} first_after={first_after.tolist()}"
        )

    gripper_positions_before_w = gripper_before_w.clone()
    lift_row = torch.tensor(
        lift["joint_positions_rad"], dtype=torch.float32, device=sim.device
    ).repeat(environment_count, 1)
    lift_row[:, 5] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["left"]
    lift_row[:, 11] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["right"]
    lift_steps = max(2, round(args.lift_seconds / physics_dt_s))
    for step in range(1, lift_steps + 1):
        alpha = step / lift_steps
        target = pinch_row + alpha * (lift_row - pinch_row)
        robot.write_joint_state_to_sim_index(
            position=target, velocity=zero_velocity, joint_ids=joint_ids
        )
        robot.set_joint_position_target_index(target=target, joint_ids=joint_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
        if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
            raise RuntimeError("cloth produced non-finite nodes during lift")

    nodes_after = local_nodes(scene, cloth)
    nodes_after_w = cloth.data.nodal_pos_w.torch
    gripper_after_w = robot.data.body_pos_w.torch[:, gripper_body_ids]
    gripper_orientation_after_xyzw = robot.data.body_quat_w.torch[
        :, gripper_body_ids
    ]
    node_lift = nodes_after[..., 2] - nodes_after_attachment[..., 2]
    maximum_node_lift_by_environment = torch.max(node_lift, dim=1).values
    minimum_maximum_node_lift_m = float(torch.min(maximum_node_lift_by_environment).item())
    if minimum_maximum_node_lift_m < MINIMUM_LIFT_M:
        gripper_now = robot.data.body_pos_w.torch[:, gripper_body_ids]
        raise RuntimeError(
            f"attached cloth did not lift: {minimum_maximum_node_lift_m:.6f} m; "
            f"env0_gripper_delta={(gripper_now[0] - gripper_before_w[0]).tolist()}"
        )
    patch_follow_errors_m = []
    selected_patch_lifts_m = []
    for environment_index in range(environment_count):
        for side_index in range(2):
            indices = selected_indices[environment_index][side_index]
            selected_displacements = (
                nodes_after[environment_index, indices]
                - nodes_after_attachment[environment_index, indices]
            )
            expected_points_w = rigidly_transformed_points_w(
                (
                    nodes_after_attachment[environment_index, indices]
                    + scene.env_origins[environment_index]
                ),
                gripper_positions_before_w[environment_index, side_index],
                gripper_orientation_before_xyzw[environment_index, side_index],
                gripper_after_w[environment_index, side_index],
                gripper_orientation_after_xyzw[environment_index, side_index],
            )
            patch_follow_errors_m.append(
                torch.max(
                    torch.linalg.vector_norm(
                        nodes_after_w[environment_index, indices]
                        - expected_points_w,
                        dim=-1,
                    )
                )
            )
            selected_patch_lifts_m.append(torch.min(selected_displacements[:, 2]))
    maximum_patch_follow_error_m = float(torch.max(torch.stack(patch_follow_errors_m)).item())
    minimum_selected_patch_lift_m = float(torch.min(torch.stack(selected_patch_lifts_m)).item())
    if minimum_selected_patch_lift_m < MINIMUM_LIFT_M:
        raise RuntimeError(
            f"selected attachment patch did not lift: {minimum_selected_patch_lift_m:.6f} m"
        )
    if maximum_patch_follow_error_m > MAXIMUM_PATCH_FOLLOW_ERROR_M:
        raise RuntimeError(
            f"selected patch failed to follow rigid gripper link: "
            f"{maximum_patch_follow_error_m:.6f} m exceeds "
            f"{MAXIMUM_PATCH_FOLLOW_ERROR_M:.6f} m"
        )
    full_cloth_environment_divergence_m = float(
        torch.max(torch.abs(nodes_after - nodes_after[0:1])).item()
    )
    attachment_patch_indices = sorted(
        set(selected_indices[0][0]) | set(selected_indices[0][1])
    )
    if any(
        sorted(set(environment[0]) | set(environment[1])) != attachment_patch_indices
        for environment in selected_indices[1:]
    ):
        raise RuntimeError("attachment patch indices differ between environments")
    attachment_patch_environment_divergence_m = float(
        torch.max(
            torch.abs(
                nodes_after[:, attachment_patch_indices]
                - nodes_after[0:1, attachment_patch_indices]
            )
        ).item()
    )
    if (
        attachment_patch_environment_divergence_m
        > MAXIMUM_ATTACHMENT_PATCH_ENVIRONMENT_DIVERGENCE_M
    ):
        per_environment_divergence_m = torch.max(
            torch.abs(nodes_after - nodes_after[0:1]).reshape(environment_count, -1),
            dim=1,
        ).values
        per_environment_attachment_snap_m = torch.max(
            torch.linalg.vector_norm(nodes_after_attachment - nodes_before, dim=-1),
            dim=1,
        ).values
        per_environment_gripper_displacements_m = (
            gripper_after_w - gripper_positions_before_w
        )
        print(
            "S1_VERTEX_PATCH_ENVIRONMENT_DIAGNOSTIC "
            + json.dumps(
                {
                    "attachment_snap_m": per_environment_attachment_snap_m.tolist(),
                    "cloth_divergence_m": per_environment_divergence_m.tolist(),
                    "gripper_displacements_m": per_environment_gripper_displacements_m.tolist(),
                    "maximum_node_lifts_m": maximum_node_lift_by_environment.tolist(),
                    "patch_follow_errors_m": [
                        float(value.item()) for value in patch_follow_errors_m
                    ],
                    "selected_patch_lifts_m": [
                        float(value.item()) for value in selected_patch_lifts_m
                    ],
                    "selected_indices": selected_indices,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        raise RuntimeError(
            "attachment patch environment divergence "
            f"{attachment_patch_environment_divergence_m:.9f} m exceeds limit"
        )

    place_release_result = None
    keep_open_row = lift_row
    result_status = PASS_STATUS
    if args.place_release:
        current_row = lift_row
        minimum_self_contact_separation_during_fold_m = math.inf
        fold_steps = max(2, round(args.fold_phase_seconds / physics_dt_s))
        for sample_index in range(2, 17):
            target_phase = phase(source, f"first_fold_{sample_index:02d}")
            target_row = torch.tensor(
                target_phase["joint_positions_rad"],
                dtype=torch.float32,
                device=sim.device,
            ).repeat(environment_count, 1)
            target_row[:, 5] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["left"]
            target_row[:, 11] = PINCH_GRIPPER_JOINT_POSITIONS_RAD["right"]
            for step in range(1, fold_steps + 1):
                alpha = step / fold_steps
                target = current_row + alpha * (target_row - current_row)
                robot.write_joint_state_to_sim_index(
                    position=target, velocity=zero_velocity, joint_ids=joint_ids
                )
                robot.set_joint_position_target_index(
                    target=target, joint_ids=joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                    raise RuntimeError(
                        f"cloth produced non-finite nodes during {target_phase['name']}"
                    )
            current_row = target_row
            if args.self_contact:
                minimum_self_contact_separation_during_fold_m = min(
                    minimum_self_contact_separation_during_fold_m,
                    minimum_nonlocal_node_separation_m(local_nodes(scene, cloth)),
                )

        nodes_at_laydown = local_nodes(scene, cloth).clone()
        disable_runtime_attachments(attachment_records)
        simulation_app.update()
        if not runtime_attachments_are_disabled(attachment_records):
            raise RuntimeError("one or more vertex attachments remained enabled")

        open_row = torch.tensor(
            phase(source, "first_fold_16")["joint_positions_rad"],
            dtype=torch.float32,
            device=sim.device,
        ).repeat(environment_count, 1)
        jaw_open_steps = 30
        for step in range(1, jaw_open_steps + 1):
            alpha = step / jaw_open_steps
            target = current_row + alpha * (open_row - current_row)
            robot.write_joint_state_to_sim_index(
                position=target, velocity=zero_velocity, joint_ids=joint_ids
            )
            robot.set_joint_position_target_index(target=target, joint_ids=joint_ids)
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
        nodes_after_release = local_nodes(scene, cloth).clone()

        retreat_row = torch.tensor(
            phase(source, "first_retreat")["joint_positions_rad"],
            dtype=torch.float32,
            device=sim.device,
        ).repeat(environment_count, 1)
        retreat_steps = max(2, round(args.retreat_seconds / physics_dt_s))
        gripper_before_retreat_w = robot.data.body_pos_w.torch[
            :, gripper_body_ids
        ].clone()
        for step in range(1, retreat_steps + 1):
            alpha = step / retreat_steps
            target = open_row + alpha * (retreat_row - open_row)
            robot.write_joint_state_to_sim_index(
                position=target, velocity=zero_velocity, joint_ids=joint_ids
            )
            robot.set_joint_position_target_index(target=target, joint_ids=joint_ids)
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError("cloth produced non-finite nodes during release retreat")

        settled_run_after_release = 0
        settled_step_after_release = None
        maximum_release_settle_steps = math.ceil(
            args.settle_timeout_s / physics_dt_s
        )
        for step in range(1, maximum_release_settle_steps + 1):
            robot.write_joint_state_to_sim_index(
                position=retreat_row, velocity=zero_velocity, joint_ids=joint_ids
            )
            robot.set_joint_position_target_index(
                target=retreat_row, joint_ids=joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            release_speed = float(
                torch.max(
                    torch.linalg.vector_norm(cloth.data.nodal_vel_w.torch, dim=-1)
                ).item()
            )
            if release_speed <= RELEASE_SETTLE_SPEED_THRESHOLD_M_S:
                settled_run_after_release += 1
            else:
                settled_run_after_release = 0
            if settled_run_after_release >= SETTLE_CONSECUTIVE_STEPS:
                settled_step_after_release = step
                break
        if settled_step_after_release is None:
            release_speeds = torch.linalg.vector_norm(
                cloth.data.nodal_vel_w.torch, dim=-1
            )
            per_environment_maximum_speed_m_s, per_environment_node_index = (
                torch.max(release_speeds, dim=1)
            )
            diagnostic_nodes = local_nodes(scene, cloth).clone()
            print(
                "S1_VERTEX_PATCH_RELEASE_SETTLE_DIAGNOSTIC "
                + json.dumps(
                    {
                        "per_environment_maximum_speed_m_s": (
                            per_environment_maximum_speed_m_s.tolist()
                        ),
                        "per_environment_fastest_node_index": (
                            per_environment_node_index.tolist()
                        ),
                        "env_0_fastest_node_position_m": diagnostic_nodes[
                            0, int(per_environment_node_index[0].item())
                        ].tolist(),
                        "minimum_nonlocal_node_separation_m": (
                            minimum_nonlocal_node_separation_m(diagnostic_nodes)
                            if args.self_contact
                            else None
                        ),
                        "minimum_node_height_m": float(
                            torch.min(diagnostic_nodes[..., 2]).item()
                        ),
                        "maximum_node_height_m": float(
                            torch.max(diagnostic_nodes[..., 2]).item()
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            raise RuntimeError(
                "cloth did not settle after release; final maximum speed="
                f"{release_speed:.6f} m/s"
            )

        nodes_final = local_nodes(scene, cloth).clone()
        nodes_final_w = cloth.data.nodal_pos_w.torch.clone()
        gripper_after_retreat_w = robot.data.body_pos_w.torch[:, gripper_body_ids]
        gripper_after_retreat_xyzw = robot.data.body_quat_w.torch[:, gripper_body_ids]
        jaw_targets_after_retreat_w = gripper_jaw_target_positions_w(
            gripper_after_retreat_w, gripper_after_retreat_xyzw
        )
        release_patch_lifts = []
        release_patch_to_jaw_distances = []
        for environment_index in range(environment_count):
            for side_index in range(2):
                indices = selected_indices[environment_index][side_index]
                release_patch_lifts.append(
                    torch.max(
                        nodes_final[environment_index, indices, 2]
                        - nodes_after_release[environment_index, indices, 2]
                    )
                )
                patch_center_w = torch.mean(
                    nodes_final_w[environment_index, indices], dim=0
                )
                release_patch_to_jaw_distances.append(
                    torch.linalg.vector_norm(
                        patch_center_w
                        - jaw_targets_after_retreat_w[environment_index, side_index]
                    )
                )
        maximum_release_patch_lift_m = float(
            torch.max(torch.stack(release_patch_lifts)).item()
        )
        minimum_release_patch_to_jaw_distance_m = float(
            torch.min(torch.stack(release_patch_to_jaw_distances)).item()
        )
        if maximum_release_patch_lift_m > MAXIMUM_RELEASE_PATCH_LIFT_M:
            raise RuntimeError(
                "released patch followed the retreating gripper by "
                f"{maximum_release_patch_lift_m:.6f} m"
            )
        if (
            minimum_release_patch_to_jaw_distance_m
            < MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
        ):
            raise RuntimeError(
                "released patch remained too close to the retreated jaw: "
                f"{minimum_release_patch_to_jaw_distance_m:.6f} m"
            )

        table_size = source["worktable_geometry"]["size_xyz_m"]
        table_pose = source["worktable_geometry"]["pose_xyz_m"]
        table_top_z_m = float(table_pose[2]) + 0.5 * float(table_size[2])
        minimum_final_clearance_m = float(
            torch.min(nodes_final_w[..., 2] - table_top_z_m).item()
        )
        maximum_final_height_m = float(
            torch.max(nodes_final_w[..., 2] - table_top_z_m).item()
        )
        if minimum_final_clearance_m < -MAXIMUM_FINAL_TABLE_PENETRATION_M:
            raise RuntimeError(
                f"released cloth penetrated table by {-minimum_final_clearance_m:.6f} m"
            )
        if maximum_final_height_m > MAXIMUM_FINAL_CLOTH_HEIGHT_M:
            raise RuntimeError(
                "released cloth did not lay down: maximum height "
                f"{maximum_final_height_m:.6f} m"
            )
        place_release_environment_divergence_m = float(
            torch.max(torch.abs(nodes_final - nodes_final[0:1])).item()
        )
        minimum_final_self_contact_separation_m = (
            minimum_nonlocal_node_separation_m(nodes_final)
            if args.self_contact
            else None
        )
        if (
            args.self_contact
            and minimum_self_contact_separation_during_fold_m
            < MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M
        ):
            raise RuntimeError(
                "cloth self-contact separation collapsed during fold: "
                f"{minimum_self_contact_separation_during_fold_m:.9f} m"
            )
        if (
            args.self_contact
            and minimum_final_self_contact_separation_m
            < MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M
        ):
            raise RuntimeError(
                "cloth self-contact separation collapsed after release: "
                f"{minimum_final_self_contact_separation_m:.9f} m"
            )
        place_release_result = {
            "source_phases": [f"first_fold_{index:02d}" for index in range(1, 17)],
            "release_event": "disable_both_vertex_patch_attachments_after_first_fold_16",
            "jaw_opened_after_attachment_disable": True,
            "retreat_phase": "first_retreat",
            "fold_phase_duration_s": args.fold_phase_seconds,
            "retreat_duration_s": args.retreat_seconds,
            "settled_step_after_release": settled_step_after_release,
            "final_maximum_node_speed_m_s": release_speed,
            "final_maximum_node_speed_limit_m_s": (
                RELEASE_SETTLE_SPEED_THRESHOLD_M_S
            ),
            "maximum_release_patch_lift_m": maximum_release_patch_lift_m,
            "maximum_release_patch_lift_limit_m": MAXIMUM_RELEASE_PATCH_LIFT_M,
            "minimum_release_patch_to_jaw_distance_m": (
                minimum_release_patch_to_jaw_distance_m
            ),
            "minimum_release_patch_to_jaw_distance_limit_m": (
                MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
            ),
            "minimum_final_table_clearance_m": minimum_final_clearance_m,
            "maximum_final_table_penetration_limit_m": (
                MAXIMUM_FINAL_TABLE_PENETRATION_M
            ),
            "maximum_final_cloth_height_m": maximum_final_height_m,
            "maximum_final_cloth_height_limit_m": MAXIMUM_FINAL_CLOTH_HEIGHT_M,
            "maximum_environment_divergence_m": (
                place_release_environment_divergence_m
            ),
            "maximum_environment_divergence_limit_m": (
                MAXIMUM_PLACE_RELEASE_ENVIRONMENT_DIVERGENCE_M
            ),
            "environment_divergence_within_exploratory_limit": (
                place_release_environment_divergence_m
                <= MAXIMUM_PLACE_RELEASE_ENVIRONMENT_DIVERGENCE_M
            ),
            "full_shape_determinism_passed": False,
            "self_contact_enabled": args.self_contact,
            "minimum_nonlocal_node_separation_during_fold_m": (
                minimum_self_contact_separation_during_fold_m
                if args.self_contact
                else None
            ),
            "minimum_final_nonlocal_node_separation_m": (
                minimum_final_self_contact_separation_m
            ),
            "minimum_nonlocal_node_separation_limit_m": (
                MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M
            ),
            "gripper_retreat_displacement_env_0_m": {
                side: [
                    float(value)
                    for value in (
                        gripper_after_retreat_w[0, index]
                        - gripper_before_retreat_w[0, index]
                    ).tolist()
                ]
                for index, side in enumerate(("left", "right"))
            },
            "laydown_to_release_maximum_node_displacement_m": float(
                torch.max(
                    torch.linalg.vector_norm(
                        nodes_after_release - nodes_at_laydown, dim=-1
                    )
                ).item()
            ),
        }
        keep_open_row = retreat_row
        result_status = (
            SELF_CONTACT_PLACE_RELEASE_PASS_STATUS
            if args.self_contact
            else PLACE_RELEASE_PASS_STATUS
        )

    result = {
        "schema_version": 1,
        "record_kind": (
            "towel_isaac_s1_vertex_patch_place_release_result"
            if args.place_release
            else "towel_isaac_s1_vertex_patch_lift_result"
        ),
        "status": result_status,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "environment_count": environment_count,
        "device": str(sim.device),
        "fabric_enabled": True,
        "suppress_readback": True,
        "replicate_physics": False,
        "identity": source["identity"],
        "urdf": str(source["urdf_path"]),
        "urdf_sha256": str(source["urdf_sha256"]),
        "cloth": {
            "size_xy_m": list(CLOTH_SIZE_XY_M),
            "resolution": list(CLOTH_RESOLUTION),
            "node_count": int(nodes_after.shape[1]),
            "mass_kg": CLOTH_MASS_KG,
            "density_kg_m3": CLOTH_DENSITY_KG_M3,
            "static_friction": CLOTH_STATIC_FRICTION,
            "dynamic_friction": CLOTH_DYNAMIC_FRICTION,
            "self_collision_enabled": args.self_contact,
            "self_collision_filter_distance_m": (
                SELF_COLLISION_FILTER_DISTANCE_M if args.self_contact else None
            ),
            "surface_thickness_m": CLOTH_SURFACE_THICKNESS_M,
            "contact_offset_m": CLOTH_CONTACT_OFFSET_M,
            "rest_offset_m": CLOTH_REST_OFFSET_M,
            "youngs_modulus_pa": CLOTH_YOUNGS_MODULUS_PA,
            "poissons_ratio": CLOTH_POISSONS_RATIO,
            "elasticity_damping": CLOTH_ELASTICITY_DAMPING,
            "linear_damping_s_inv": CLOTH_LINEAR_DAMPING_S_INV,
            "settling_damping_s_inv": CLOTH_SETTLING_DAMPING_S_INV,
            "settling_threshold_m_s": CLOTH_SETTLING_THRESHOLD_M_S,
            "physics_dt_s": physics_dt_s,
            "solver_position_iteration_count": 24,
            "collision_pair_update_frequency": 4,
            "collision_iteration_multiplier": 2.0,
            "speculative_ccd_enabled": False,
            "maximum_linear_velocity_m_s": None,
            "material_physical_fidelity_validated": False,
        },
        "attachment": {
            "type": "OmniPhysicsVtxXformAttachment",
            "target": "registered_r0g_gripper_frames_under_articulation_links",
            "direct_articulation_link_attachment": True,
            "scripted_attachment_used": True,
            "physical_frictional_grasp_validated": False,
            "patch_mask_radius_m": PATCH_MASK_RADIUS_M,
            "records": attachment_records,
            "minimum_selected_patch_point_count": minimum_selected_points,
            "minimum_authored_point_count": minimum_authored_points,
            "maximum_attachment_snap_m": maximum_attachment_snap_m,
            "maximum_attachment_snap_limit_m": MAXIMUM_ATTACHMENT_SNAP_M,
            "maximum_patch_follow_error_m": maximum_patch_follow_error_m,
            "maximum_patch_follow_error_limit_m": MAXIMUM_PATCH_FOLLOW_ERROR_M,
        },
        "jaw_alignment": {
            "registered_gripper_frame_translation_m": list(
                GRIPPER_FRAME_TRANSLATION_M
            ),
            "pinch_gripper_joint_positions_rad": PINCH_GRIPPER_JOINT_POSITIONS_RAD,
            "pinch_gap_center_tcp_x_m": PINCH_GAP_CENTER_TCP_X_M,
            "explicit_jaw_collision_proxies": {
                "fixed_pad_size_m": [0.002, 0.010, 0.010],
                "moving_pad_size_m": [0.002, 0.009, 0.0095],
                "pads_per_environment": 4,
            },
            "gripper_cloth_collision_enabled": True,
            "non_gripper_robot_cloth_collision_filtered": True,
            "maximum_jaw_target_patch_center_xy_distance_m": (
                maximum_jaw_target_patch_center_xy_distance_m
            ),
            "maximum_jaw_target_patch_center_xy_distance_limit_m": (
                MAXIMUM_JAW_TARGET_PATCH_CENTER_XY_DISTANCE_M
            ),
            "maximum_attachment_point_tcp_distance_m": (
                maximum_attachment_point_tcp_distance_m
            ),
            "maximum_attachment_point_tcp_distance_limit_m": (
                MAXIMUM_ATTACHMENT_POINT_TCP_DISTANCE_M
            ),
            "maximum_pinch_induced_cloth_displacement_m": (
                maximum_pinch_induced_cloth_displacement_m
            ),
            "maximum_pinch_induced_cloth_displacement_limit_m": (
                MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M
            ),
            "registered_tcp_positions_env_0_w_m": {
                side: [float(value) for value in gripper_tcp_before_w[0, index].tolist()]
                for index, side in enumerate(("left", "right"))
            },
            "jaw_target_positions_env_0_w_m": {
                side: [
                    float(value)
                    for value in gripper_jaw_target_before_w[0, index].tolist()
                ]
                for index, side in enumerate(("left", "right"))
            },
        },
        "lift": {
            "source_phase": "first_contact",
            "target_phase": "first_fold_01",
            "duration_s": args.lift_seconds,
            "minimum_maximum_node_lift_m": minimum_maximum_node_lift_m,
            "minimum_selected_patch_lift_m": minimum_selected_patch_lift_m,
            "minimum_required_lift_m": MINIMUM_LIFT_M,
            "gripper_displacement_env_0_m": {
                side: [
                    float(value)
                    for value in (gripper_after_w[0, index] - gripper_before_w[0, index]).tolist()
                ]
                for index, side in enumerate(("left", "right"))
            },
        },
        "place_release": place_release_result,
        "settled_step_before_attachment": settled_step,
        "maximum_attachment_patch_environment_divergence_m": (
            attachment_patch_environment_divergence_m
        ),
        "attachment_patch_environment_divergence_tolerance_m": (
            MAXIMUM_ATTACHMENT_PATCH_ENVIRONMENT_DIVERGENCE_M
        ),
        "maximum_full_cloth_environment_divergence_m": (
            full_cloth_environment_divergence_m
        ),
        "simulation_checks": {
            "surface_deformable_loaded": True,
            "r0g_bimanual_articulation_loaded": True,
            "dual_gripper_vertex_patches_created": True,
            "low_lift_executed": True,
            "direct_gripper_link_coupling_checked": True,
            "jaw_aligned_attachment_patch_checked": True,
            "gripper_cloth_collision_enabled": True,
            "physical_frictional_grasp_checked": False,
            "place_and_release_checked": args.place_release,
            "robot_cloth_collision_checked": False,
            "self_collision_checked": args.self_contact,
            "full_dynamic_cloth_shape_determinism_checked": False,
        },
        "completion_claim": {
            "vertex_patch_attachment_lift_smoke_passed": True,
            "vertex_patch_place_release_smoke_passed": args.place_release,
            "self_contact_smoke_passed": args.self_contact,
            "s1_completed": False,
            "blocking_reason": (
                "physical_frictional_grasp_full_shape_determinism_and_"
                "material_commissioning_not_run"
                if args.self_contact
                else "physical_frictional_grasp_self_collision_full_shape_"
                "determinism_and_material_commissioning_not_run"
                if args.place_release
                else "physical_frictional_grasp_place_release_self_collision_"
                "full_shape_determinism_and_material_commissioning_not_run"
            ),
        },
        "source_status": manifest["status"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"{result_status} attachments={len(attachment_records)} "
        f"minimum_points={minimum_authored_points} "
        f"minimum_lift_m={minimum_maximum_node_lift_m:.6f} "
        f"minimum_selected_patch_lift_m={minimum_selected_patch_lift_m:.6f} "
        f"maximum_patch_follow_error_m={maximum_patch_follow_error_m:.6f} "
        f"jaw_target_patch_xy_m={maximum_jaw_target_patch_center_xy_distance_m:.6f} "
        f"patch_tcp_radius_m={maximum_attachment_point_tcp_distance_m:.6f} "
        f"attachment_patch_env_divergence_m="
        f"{attachment_patch_environment_divergence_m:.9f} "
        f"full_cloth_env_divergence_m={full_cloth_environment_divergence_m:.9f} "
        "motion_commands=0 "
        f"output={args.output}",
        flush=True,
    )
    if args.keep_open:
        print("S1_VERTEX_PATCH_GUI_KEEP_OPEN close the Isaac Sim window when done", flush=True)
        while simulation_app.is_running():
            robot.write_joint_state_to_sim_index(
                position=keep_open_row, velocity=zero_velocity, joint_ids=joint_ids
            )
            robot.set_joint_position_target_index(
                target=keep_open_row, joint_ids=joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
    return 0


if __name__ == "__main__":
    try:
        exit_code = run()
    except Exception as error:
        print(f"S1_VERTEX_PATCH_FAIL {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
