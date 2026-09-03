#!/usr/bin/env python3
"""Validate a vertical dual-jaw towel pinch, retention, fold, and release."""

from __future__ import annotations

import argparse
import copy
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

from tools.lib.so101_gripper_geometry import (
    GripperGeometryCandidate,
    load_gripper_geometry_candidate,
)


@dataclass(frozen=True, slots=True)
class MaterialCandidate:
    path: Path
    sha256: str
    status: str
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
    newton_triangle_stiffness_pa: float
    newton_triangle_area_stiffness_pa: float
    newton_triangle_damping_pa_s: float
    newton_edge_stiffness_n_m: float
    newton_edge_damping_n_m_s: float
    newton_calibration_status: str


def load_material_candidate(path: Path) -> MaterialCandidate:
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("record_kind") != "towel_isaac_s1_material_candidate"
        or document.get("status") != "R2_S1_MATERIAL_CALIBRATED_CANDIDATE"
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
            raise ValueError(f"material parameter {name} is invalid: {value}")
        return value

    newton = model.get("newton_vbd_meter")
    if not isinstance(newton, dict):
        raise ValueError("material config newton_vbd_meter must be a mapping")

    def finite_newton(name: str, *, allow_zero: bool = False) -> float:
        value = float(newton[name])
        if not math.isfinite(value) or (value < 0.0 if allow_zero else value <= 0.0):
            raise ValueError(f"Newton material parameter {name} is invalid: {value}")
        return value

    if float(newton.get("world_units_per_meter", 0.0)) != 1.0:
        raise ValueError("newton_vbd_meter must use one world unit per metre")
    if tuple(int(value) for value in newton.get("cloth_resolution", ())) != (31, 31):
        raise ValueError("newton_vbd_meter must be calibrated at 31x31 resolution")
    if int(newton.get("solver_substeps", 0)) != 10:
        raise ValueError("newton_vbd_meter must be calibrated with 10 substeps")
    if int(newton.get("solver_iterations", 0)) != 10:
        raise ValueError("newton_vbd_meter must be calibrated with 10 iterations")

    candidate = MaterialCandidate(
        path=path.resolve(),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        status=str(document["status"]),
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
        newton_triangle_stiffness_pa=finite_newton("triangle_stiffness_pa"),
        newton_triangle_area_stiffness_pa=finite_newton(
            "triangle_area_stiffness_pa"
        ),
        newton_triangle_damping_pa_s=finite_newton(
            "triangle_damping_pa_s", allow_zero=True
        ),
        newton_edge_stiffness_n_m=finite_newton("edge_stiffness_n_m"),
        newton_edge_damping_n_m_s=finite_newton(
            "edge_damping_n_m_s", allow_zero=True
        ),
        newton_calibration_status=str(newton["calibration_status"]),
    )
    if candidate.static_friction < candidate.dynamic_friction:
        raise ValueError("static friction must not be below dynamic friction")
    if not 0.0 < candidate.poissons_ratio < 0.5:
        raise ValueError("poissons ratio must be between zero and 0.5")
    if candidate.contact_offset_m < candidate.rest_offset_m:
        raise ValueError("contact offset must not be below rest offset")
    return candidate

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("manifest", type=Path)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument(
    "--material-config",
    type=Path,
    default=ROOT / "config/towel_isaac_s1_material.json",
)
parser.add_argument(
    "--gripper-config",
    type=Path,
    default=ROOT / "config/so101_gripper_geometry.candidate.json",
)
parser.add_argument("--settle-timeout-s", type=float, default=8.0)
parser.add_argument(
    "--environment-count",
    type=int,
    help="override manifest replication count for exploratory geometry diagnosis",
)
parser.add_argument("--lift-seconds", type=float, default=1.0)
parser.add_argument(
    "--simulation-render-interval",
    type=int,
    default=1,
    help="render one viewport frame every N physics steps without changing physics dt",
)
parser.add_argument(
    "--physics-backend",
    choices=("physx", "newton-coupled-vbd"),
    default="physx",
    help="physics implementation used by the actual-contact A/B probe",
)
parser.add_argument(
    "--newton-contact-stiffness",
    type=float,
    default=3.0e5,
    help=(
        "body-particle and rigid-shape numerical contact stiffness; 3e5 is "
        "shared with the direct Newton fold and prevents compliant cloth from "
        "slowly tunneling through the table"
    ),
)
parser.add_argument(
    "--newton-curvature-softening",
    action="store_true",
    help=(
        "preserve the calibrated small-bend stiffness, then smoothly lower "
        "hinge stiffness only across the high-curvature fold range"
    ),
)
parser.add_argument(
    "--newton-softening-activation-angle-deg",
    type=float,
    default=55.0,
    help="per-hinge angle where smooth high-curvature softening begins",
)
parser.add_argument(
    "--newton-full-softening-angle-deg",
    type=float,
    default=145.0,
    help="per-hinge angle where smooth high-curvature softening reaches its maximum",
)
parser.add_argument(
    "--newton-softened-edge-stiffness",
    type=float,
    default=0.027,
    help="Newton edge stiffness at maximum sharp-fold softening (default is 20%% of 0.135)",
)
parser.add_argument(
    "--jaw-pad-face-size-mm",
    type=float,
    default=6.0,
    help="square jaw contact-face proxy size; 10 mm matches the STL distal cross-section",
)
parser.add_argument(
    "--actual-jaw-mesh-contact",
    action="store_true",
    help="retain both imported jaw STL colliders and add rubber only to the fixed face",
)
parser.add_argument(
    "--newton-rubber-friction",
    type=float,
    help="Newton-only fixed-pad friction; official cloth examples use 100 for no-slip grip",
)
parser.add_argument(
    "--frictional-descent-fraction",
    type=float,
    default=0.78,
    help="fraction of the 35 mm vertical pregrasp-to-deep-contact path",
)
parser.add_argument("--left-frictional-descent-fraction", type=float, default=0.90)
parser.add_argument("--right-frictional-descent-fraction", type=float, default=0.83)
parser.add_argument(
    "--grasp-mode",
    choices=("frictional", "contact-gated-retention", "legacy-attachment"),
    default="contact-gated-retention",
    help=(
        "validate vertical jaw contact before measured no-slip retention; "
        "pure friction and legacy attachment remain explicit diagnostics"
    ),
)
parser.add_argument(
    "--grasp-release-probe",
    action="store_true",
    help=(
        "after the isolated lift, hold for one second, open both jaws to the "
        "measured Q0 gap, and require the contact-gated cloth points to release"
    ),
)
parser.add_argument(
    "--place-release",
    action="store_true",
    help="continue through the first-fold laydown, detach, retreat, and settle gate",
)
parser.add_argument(
    "--calibrate-scripted-touchdown",
    action="store_true",
    help=(
        "record the simulator-only suspended free-edge trajectory and stop "
        "before touchdown; no mid-action camera control is introduced"
    ),
)
parser.add_argument(
    "--calibrate-post-touchdown-anchor",
    action="store_true",
    help=(
        "record simulator-only free-edge position and speed after the scripted "
        "touchdown feed, then stop before forward lay"
    ),
)
parser.add_argument(
    "--diagnostic-allow-airborne-free-edge",
    action="store_true",
    help=(
        "continue an archived GUI diagnostic after logging a failed free-edge "
        "table-contact gate; never counts as a passing result"
    ),
)
parser.add_argument(
    "--self-contact",
    action="store_true",
    help="enable cloth self-collision and gate nonlocal vertex separation",
)
parser.add_argument(
    "--second-contact-diagnostic",
    action="store_true",
    help="after the first release, replay open-jaw clear/departure to second_contact",
)
parser.add_argument(
    "--post-release-correction-replay",
    type=Path,
    help=(
        "after the first shape settles, execute a passing observed-boundary "
        "correction replay with a fresh actual-contact grasp"
    ),
)
parser.add_argument("--fold-phase-seconds", type=float, default=0.20)
parser.add_argument(
    "--scripted-pre-touchdown-hold-s",
    type=float,
    help="override the fixed pre-touchdown dwell for exact candidate replay",
)
parser.add_argument(
    "--scripted-post-touchdown-hold-s",
    type=float,
    help="override the fixed post-touchdown dwell for exact candidate replay",
)
parser.add_argument("--retreat-seconds", type=float, default=1.0)
parser.add_argument(
    "--arm-target-settle-timeout-s",
    type=float,
    default=2.0,
    help="maximum physical-drive hold time after each commanded motion segment",
)
parser.add_argument("--keep-open", action="store_true")
parser.add_argument(
    "--contact-pose-diagnostic",
    action="store_true",
    help="stop after jaw-close attempt and keep the achieved contact pose visible",
)
parser.add_argument(
    "--kinematic-replay",
    type=Path,
    help=(
        "override the first-fold joint replay and towel placement from a "
        "canonical full-FK diagnostic; intended for motion-free Isaac candidates"
    ),
)
parser.add_argument(
    "--urdf-override",
    type=Path,
    help=(
        "use a geometrically equivalent URDF after validating the manifest; "
        "intended for Newton-safe assets with mesh scales baked into vertices"
    ),
)
parser.add_argument(
    "--disable-cubric-visual-sync",
    action="store_true",
    help=(
        "use Fabric's CPU hierarchy propagation instead of the version-sensitive "
        "Cubric adapter; affects viewport synchronization only"
    ),
)
parser.add_argument(
    "--right-arm-kinematic-replay",
    type=Path,
    help=(
        "optional passing replay supplying only the right-arm targets; used "
        "when measured left/right Isaac TCP height biases require independent "
        "contact offsets"
    ),
)
parser.add_argument(
    "--right-arm-kinematic-replay-through",
    help=(
        "last inclusive phase receiving the right-arm replay; later phases "
        "return to the primary replay after the table-clearance correction"
    ),
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if not args.material_config.is_file():
    parser.error(f"material config does not exist: {args.material_config}")
if not args.gripper_config.is_file():
    parser.error(f"gripper config does not exist: {args.gripper_config}")
if args.kinematic_replay is not None and not args.kinematic_replay.is_file():
    parser.error(f"kinematic replay does not exist: {args.kinematic_replay}")
if args.urdf_override is not None and not args.urdf_override.is_file():
    parser.error(f"URDF override does not exist: {args.urdf_override}")
if (
    args.post_release_correction_replay is not None
    and not args.post_release_correction_replay.is_file()
):
    parser.error(
        "post-release correction replay does not exist: "
        f"{args.post_release_correction_replay}"
    )
if (
    args.right_arm_kinematic_replay is not None
    and not args.right_arm_kinematic_replay.is_file()
):
    parser.error(
        "right-arm kinematic replay does not exist: "
        f"{args.right_arm_kinematic_replay}"
    )
if args.right_arm_kinematic_replay is not None and args.kinematic_replay is None:
    parser.error("--right-arm-kinematic-replay requires --kinematic-replay")
if (
    args.right_arm_kinematic_replay_through is not None
    and args.right_arm_kinematic_replay is None
):
    parser.error(
        "--right-arm-kinematic-replay-through requires "
        "--right-arm-kinematic-replay"
    )
if args.output.exists():
    parser.error(f"refusing to overwrite existing output: {args.output}")
if not math.isfinite(args.settle_timeout_s) or args.settle_timeout_s <= 0.0:
    parser.error("--settle-timeout-s must be finite and positive")
if args.environment_count is not None and args.environment_count <= 0:
    parser.error("--environment-count must be positive")
if not math.isfinite(args.lift_seconds) or args.lift_seconds <= 0.0:
    parser.error("--lift-seconds must be finite and positive")
if args.simulation_render_interval <= 0:
    parser.error("--simulation-render-interval must be positive")
if (
    not math.isfinite(args.newton_contact_stiffness)
    or args.newton_contact_stiffness <= 0.0
):
    parser.error("--newton-contact-stiffness must be finite and positive")
if not 0.0 < args.newton_softening_activation_angle_deg < 180.0:
    parser.error("--newton-softening-activation-angle-deg must be in (0, 180)")
if not (
    args.newton_softening_activation_angle_deg
    < args.newton_full_softening_angle_deg
    < 180.0
):
    parser.error(
        "--newton-full-softening-angle-deg must be above the activation angle "
        "and below 180"
    )
if (
    not math.isfinite(args.newton_softened_edge_stiffness)
    or args.newton_softened_edge_stiffness <= 0.0
):
    parser.error("--newton-softened-edge-stiffness must be finite and positive")
if args.newton_curvature_softening and args.physics_backend != "newton-coupled-vbd":
    parser.error("--newton-curvature-softening requires --physics-backend newton-coupled-vbd")
if not math.isfinite(args.jaw_pad_face_size_mm) or not (
    4.0 <= args.jaw_pad_face_size_mm <= 10.0
):
    parser.error("--jaw-pad-face-size-mm must be in [4, 10]")
if args.newton_rubber_friction is not None and (
    not math.isfinite(args.newton_rubber_friction)
    or args.newton_rubber_friction <= 0.0
):
    parser.error("--newton-rubber-friction must be finite and positive")
if not math.isfinite(args.frictional_descent_fraction) or not (
    0.0 < args.frictional_descent_fraction <= 1.0
):
    parser.error("--frictional-descent-fraction must be in (0, 1]")
for side in ("left", "right"):
    value = getattr(args, f"{side}_frictional_descent_fraction")
    if value is not None and (
        not math.isfinite(value) or not 0.0 < value <= 1.0
    ):
        parser.error(f"--{side}-frictional-descent-fraction must be in (0, 1]")
if not math.isfinite(args.fold_phase_seconds) or args.fold_phase_seconds <= 0.0:
    parser.error("--fold-phase-seconds must be finite and positive")
for hold_name in (
    "scripted_pre_touchdown_hold_s",
    "scripted_post_touchdown_hold_s",
):
    hold_value = getattr(args, hold_name)
    if hold_value is not None and (
        not math.isfinite(hold_value) or hold_value < 0.0
    ):
        parser.error(f"--{hold_name.replace('_', '-')} must be finite and nonnegative")
if not math.isfinite(args.retreat_seconds) or args.retreat_seconds <= 0.0:
    parser.error("--retreat-seconds must be finite and positive")
if not math.isfinite(args.arm_target_settle_timeout_s) or (
    args.arm_target_settle_timeout_s <= 0.0
):
    parser.error("--arm-target-settle-timeout-s must be finite and positive")
if args.self_contact and not args.place_release:
    parser.error("--self-contact requires --place-release")
if args.calibrate_scripted_touchdown and not (
    args.place_release
    and args.physics_backend == "newton-coupled-vbd"
    and args.kinematic_replay is not None
):
    parser.error(
        "--calibrate-scripted-touchdown requires --place-release "
        "--physics-backend newton-coupled-vbd --kinematic-replay"
    )
if args.calibrate_post_touchdown_anchor and not (
    args.place_release
    and args.physics_backend == "newton-coupled-vbd"
    and args.kinematic_replay is not None
):
    parser.error(
        "--calibrate-post-touchdown-anchor requires --place-release "
        "--physics-backend newton-coupled-vbd --kinematic-replay"
    )
if args.calibrate_scripted_touchdown and args.calibrate_post_touchdown_anchor:
    parser.error("select only one scripted-touchdown calibration stage")
if args.second_contact_diagnostic and not (args.place_release and args.self_contact):
    parser.error("--second-contact-diagnostic requires --place-release --self-contact")
if args.post_release_correction_replay is not None and not (
    args.place_release
    and args.self_contact
    and args.grasp_mode == "contact-gated-retention"
    and args.physics_backend == "newton-coupled-vbd"
    and args.kinematic_replay is not None
):
    parser.error(
        "--post-release-correction-replay requires --place-release --self-contact "
        "--grasp-mode contact-gated-retention --physics-backend "
        "newton-coupled-vbd and --kinematic-replay"
    )
if args.grasp_release_probe and args.place_release:
    parser.error("--grasp-release-probe and --place-release are separate gates")
if args.grasp_release_probe and args.grasp_mode != "contact-gated-retention":
    parser.error("--grasp-release-probe requires --grasp-mode contact-gated-retention")
if args.grasp_release_probe and args.physics_backend != "newton-coupled-vbd":
    parser.error("--grasp-release-probe requires --physics-backend newton-coupled-vbd")

material_candidate = load_material_candidate(args.material_config)
gripper_candidate = load_gripper_geometry_candidate(args.gripper_config)

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch
import warp as wp

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.physics import PhysicsEvent
from isaaclab.utils.configclass import configclass
from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg, NewtonManager
from isaaclab_newton.sim.schemas import NewtonDeformableBodyPropertiesCfg
from isaaclab_newton.sim.spawners.materials import (
    NewtonSurfaceDeformableBodyMaterialCfg,
)
from isaaclab_contrib.deformable.newton_manager_cfg import (
    CoupledMJWarpVBDSolverCfg,
    NewtonModelCfg,
    VBDSolverCfg,
)
import omni.kit.actions.core
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from tools.lib.towel_isaac_s0 import validate_s0_host_manifest


@wp.kernel
def update_newton_curvature_softening(
    positions: wp.array[wp.vec3],
    edge_indices: wp.array2d[wp.int32],
    activation_angle: float,
    full_softening_angle: float,
    small_bend_stiffness: float,
    softened_stiffness: float,
    softened_edges: wp.array[wp.int32],
    ever_softened_edges: wp.array[wp.int32],
    peak_absolute_angles: wp.array[wp.float32],
    bending_properties: wp.array2d[wp.float32],
):
    """Soften only hinges that are currently sharp; restore flat cloth."""
    tid = wp.tid()
    i = edge_indices[tid, 0]
    j = edge_indices[tid, 1]
    k = edge_indices[tid, 2]
    l = edge_indices[tid, 3]
    if i < 0 or j < 0 or k < 0 or l < 0:
        return
    x1 = positions[i]
    x2 = positions[j]
    x3 = positions[k]
    x4 = positions[l]
    n1 = wp.cross(x3 - x1, x4 - x1)
    n2 = wp.cross(x4 - x2, x3 - x2)
    edge = x4 - x3
    n1_length = wp.length(n1)
    n2_length = wp.length(n2)
    edge_length = wp.length(edge)
    if n1_length < 1.0e-6 or n2_length < 1.0e-6 or edge_length < 1.0e-6:
        return
    n1_hat = n1 / n1_length
    n2_hat = n2 / n2_length
    edge_hat = edge / edge_length
    angle = wp.atan2(
        wp.dot(wp.cross(n1_hat, n2_hat), edge_hat),
        wp.dot(n1_hat, n2_hat),
    )
    absolute_angle = wp.abs(angle)
    if absolute_angle > peak_absolute_angles[tid]:
        peak_absolute_angles[tid] = absolute_angle
    if absolute_angle > activation_angle:
        blend = (absolute_angle - activation_angle) / (
            full_softening_angle - activation_angle
        )
        if blend > 1.0:
            blend = 1.0
        # Smoothstep avoids the visible hinge impulse produced by the former
        # binary 20x stiffness switch.  The measured 45-degree cantilever
        # remains entirely on the calibrated small-bend branch.
        blend = blend * blend * (3.0 - 2.0 * blend)
        softened_edges[tid] = 1
        ever_softened_edges[tid] = 1
        bending_properties[tid, 0] = small_bend_stiffness + blend * (
            softened_stiffness - small_bend_stiffness
        )
    else:
        softened_edges[tid] = 0
        bending_properties[tid, 0] = small_bend_stiffness


if args.disable_cubric_visual_sync:
    def _skip_cubric_visual_sync(cls: type[NewtonManager]) -> None:
        cls._cubric = None

    NewtonManager._setup_cubric_bindings = classmethod(_skip_cubric_visual_sync)


PASS_STATUS = "S1_ISAACLAB_VERTEX_PATCH_LIFT_PASS_MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
FRICTIONAL_LIFT_PASS_STATUS = (
    "S1_ISAACLAB_FRICTIONAL_JAW_LIFT_PASS_MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
)
CONTACT_GATED_RETENTION_LIFT_PASS_STATUS = (
    "S1_ISAACLAB_CONTACT_GATED_NO_SLIP_RETENTION_LIFT_PASS_"
    "MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
)
CONTACT_GATED_RELEASE_PASS_STATUS = (
    "S1_ISAACLAB_ACTUAL_CONTACT_GATED_NO_SLIP_LIFT_Q0_RELEASE_PASS_"
    "MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
)
PLACE_RELEASE_PASS_STATUS = (
    "S1_ISAACLAB_VERTEX_PATCH_PLACE_RELEASE_PASS_"
    "MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED_SELF_COLLISION_NOT_RUN"
)
SELF_CONTACT_PLACE_RELEASE_PASS_STATUS = (
    "S1_ISAACLAB_VERTEX_PATCH_PLACE_RELEASE_SELF_CONTACT_PASS_"
    "MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED_FULL_SHAPE_DETERMINISM_NOT_PASSED"
)
PLACE_RELEASE_SHAPE_DIAGNOSTIC_FAIL_STATUS = (
    "S1_ISAACLAB_PLACE_RELEASE_SHAPE_GATE_FAIL_DIAGNOSTIC_SAVED"
)
NOMINAL_HALF_FOLD_ACCEPTED_STATUS = (
    "S1_ISAACLAB_NOMINAL_HALF_FOLD_ACCEPTED_WITHIN_55_45_"
    "HIGH_CURVATURE_MATERIAL_NOT_PHYSICALLY_CALIBRATED"
)
NOMINAL_HALF_FOLD_MEASURED_MATERIAL_STATUS = (
    "S1_ISAACLAB_NOMINAL_HALF_FOLD_ACCEPTED_WITHIN_55_45_"
    "MEASURED_BEND_AND_DAMPING_CALIBRATED"
)
PHYSICS_DT_S = 1.0 / 120.0
SELF_CONTACT_PHYSICS_DT_S = 1.0 / 240.0
ENVIRONMENT_SPACING_M = 1.0
CLOTH_SIZE_XY_M = (0.300, 0.300)
CLOTH_RESOLUTION = (31, 31)
CLOTH_MASS_KG = material_candidate.mass_kg
CLOTH_DENSITY_KG_M3 = material_candidate.density_kg_m3
CLOTH_STATIC_FRICTION = material_candidate.static_friction
CLOTH_DYNAMIC_FRICTION = material_candidate.dynamic_friction
CLOTH_YOUNGS_MODULUS_PA = material_candidate.youngs_modulus_pa
CLOTH_POISSONS_RATIO = material_candidate.poissons_ratio
CLOTH_ELASTICITY_DAMPING = material_candidate.elasticity_damping
CLOTH_SURFACE_BEND_STIFFNESS_PA = material_candidate.surface_bend_stiffness_pa
CLOTH_BEND_DAMPING_S_INV = material_candidate.bend_damping_s_inv
CLOTH_SURFACE_THICKNESS_M = material_candidate.surface_thickness_m
CLOTH_CONTACT_OFFSET_M = material_candidate.contact_offset_m
CLOTH_REST_OFFSET_M = material_candidate.rest_offset_m
CLOTH_LINEAR_DAMPING_S_INV = material_candidate.linear_damping_s_inv
CLOTH_SETTLING_DAMPING_S_INV = material_candidate.settling_damping_s_inv
CLOTH_SETTLING_THRESHOLD_M_S = material_candidate.settling_threshold_m_s
NEWTON_TRIANGLE_STIFFNESS_PA = material_candidate.newton_triangle_stiffness_pa
NEWTON_TRIANGLE_AREA_STIFFNESS_PA = (
    material_candidate.newton_triangle_area_stiffness_pa
)
NEWTON_TRIANGLE_DAMPING_PA_S = material_candidate.newton_triangle_damping_pa_s
NEWTON_EDGE_STIFFNESS_N_M = material_candidate.newton_edge_stiffness_n_m
NEWTON_EDGE_DAMPING_N_M_S = material_candidate.newton_edge_damping_n_m_s
CLOTH_INITIAL_CLEARANCE_M = 0.5 * CLOTH_SURFACE_THICKNESS_M
NEWTON_CLOTH_AREAL_DENSITY_KG_M2 = CLOTH_MASS_KG / (
    CLOTH_SIZE_XY_M[0] * CLOTH_SIZE_XY_M[1]
)
PATCH_MASK_RADIUS_M = 0.016
MINIMUM_PATCH_POINT_COUNT = 4
MINIMUM_LIFT_M = 0.003
MAXIMUM_ATTACHMENT_SNAP_M = 0.005
MAXIMUM_PATCH_FOLLOW_ERROR_M = 0.003
MAXIMUM_ATTACHMENT_PATCH_ENVIRONMENT_DIVERGENCE_M = 5.0e-4
MAXIMUM_PLACE_RELEASE_ENVIRONMENT_DIVERGENCE_M = 0.020
MAXIMUM_FINAL_TABLE_PENETRATION_M = 0.002
MAXIMUM_FINAL_CLOTH_HEIGHT_M = 0.030
MAXIMUM_FIRST_FOLD_FOOTPRINT_WIDTH_M = 0.180
MAXIMUM_FIRST_FOLD_PAIRED_VERTEX_P95_XY_ERROR_M = 0.030
MAXIMUM_RELEASE_PATCH_LIFT_M = 0.015
MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M = 0.015
MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M = 5.0e-4
SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD = 2
SELF_COLLISION_FILTER_DISTANCE_M = (
    SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD * CLOTH_SIZE_XY_M[0] / CLOTH_RESOLUTION[0]
)
GRIPPER_FRAME_TRANSLATION_M = (-0.0079, -0.000218121, -0.0981274)
FIXED_JAW_PAD_CENTER_PARENT_M = (-0.0089, -0.000218121, -0.0991274)
MOVING_JAW_PAD_CENTER_PARENT_M = (-0.0113, -0.0765, 0.0189)
JAW_PAD_SIZE_M = (
    0.002,
    args.jaw_pad_face_size_mm * 0.001,
    args.jaw_pad_face_size_mm * 0.001,
)
JAW_PAD_NORMALS_PARENT = {
    "left": {
        "fixed": (-0.0063110869, 0.0289182037, 0.9995618579),
        "moving": (-0.2432612437, 0.9695295212, -0.0289218753),
    },
    "right": {
        "fixed": (0.8833461962, 0.0230649242, 0.4681532942),
        "moving": (0.7873451460, 0.6160807991, -0.0230666438),
    },
}
PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.grasp_project_rad[side][1]
    for side in ("left", "right")
}
PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.grasp_model_rad(side, 1)
    for side in ("left", "right")
}
RELEASE_MODEL_GRIPPER_JOINT_POSITION_RAD = gripper_candidate.release_model_rad
SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.simulation_release_model_rad(side)
    for side in ("left", "right")
}
PINCH_GAP_CENTER_TCP_X_M = {"left": -0.0107, "right": -0.0084}
MAXIMUM_JAW_TARGET_PATCH_CENTER_XY_DISTANCE_M = 0.008
MAXIMUM_ATTACHMENT_POINT_TCP_DISTANCE_M = 0.030
MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M = 0.005
MAXIMUM_GRIPPER_CLOSING_RESIDUAL_RAD = 0.01
MAXIMUM_ARM_TARGET_RESIDUAL_RAD = 0.03
PINCH_CLOSE_DURATION_S = 0.25
PINCH_HOLD_DURATION_S = 0.125
PINCH_TARGET_SETTLE_TIMEOUT_S = 0.50
JAW_OPEN_DURATION_S = 0.25
POST_OPEN_RELEASE_HOLD_S = 0.50
GRASP_RETENTION_HOLD_S = 1.0
GRASP_RELEASE_HOLD_S = 0.75
MINIMUM_CONTACT_GATED_LIFT_M = 0.007
MINIMUM_GRASP_RELEASE_DROP_M = 0.003
PINNED_LAYDOWN_HOLD_S = 1.0
# Allow the progressively deposited lower half to settle before reversing the
# arm direction.  This timed hold does not infer the hidden contact boundary
# from the top camera.
FORWARD_LAY_CONTACT_HOLD_S = 0.5
# Judge drag only after the towel has been lowered into its final, low sweep.
# The free edge swings substantially between touchdown and that point, so its
# instantaneous touchdown X is useful diagnostics but not a material anchor.
MAXIMUM_LOW_SWEEP_FREE_EDGE_DRIFT_M = 0.010
# Keep this as a diagnostic for detecting whether the deposited edge follows
# the arm.  It is not a standalone failure criterion: when an individual arm
# segment is short, a harmless millimetre-scale response produces a large
# ratio (for example 2.05 mm / 14.3 mm = 14.4%).  The physically meaningful
# gate is the accumulated free-edge displacement below.
MAXIMUM_LOW_SWEEP_INCREMENTAL_SLIP_RATIO = 0.10
MINIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M = 0.135
MAXIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M = 0.170
MAXIMUM_NOMINAL_LAYER_FRACTION = 0.55
MINIMUM_NOMINAL_PROFILE_LENGTH_M = 0.270
MAXIMUM_NOMINAL_PROFILE_LENGTH_M = 0.320
MAXIMUM_RAW_TERMINAL_CURL_AMPLITUDE_M = 0.030
MAXIMUM_RAW_TERMINAL_CURL_FRACTION = 0.125
MINIMUM_RAW_MAIN_FOLD_COLUMN = 13
MAXIMUM_RAW_MAIN_FOLD_COLUMN = 17
# At the calibrated low touchdown the free edge has already met the table, so
# this no longer represents a purely suspended verticality angle.  Permit at
# most one quarter of the measured 285 mm hanging length to lie sideways; a
# larger displacement indicates a collapsed panel rather than useful slack.
MAXIMUM_TOUCHDOWN_FREE_EDGE_HORIZONTAL_OFFSET_M = 0.070
MAXIMUM_ANCHORED_FREE_EDGE_TABLE_CLEARANCE_M = 0.010
# The fixed towel, grasp, lift, and initial placement make the suspended swing
# repeatable enough to use a scripted dwell.  Mid-action top-view feedback is
# deliberately excluded because the arms occlude the relevant boundary.
SUSPENDED_PRE_TOUCHDOWN_HOLD_S = 0.375
SUSPENDED_TOUCHDOWN_CALIBRATION_DURATION_S = 8.0
SUSPENDED_TOUCHDOWN_CALIBRATION_VIDEO_FPS = 24.0
# The suspended-forward route must not stop after touchdown: a hold lets the
# free panel swing ahead again before the lower layer is deposited.
POST_TOUCHDOWN_ANCHOR_HOLD_S = 0.0
POST_TOUCHDOWN_CALIBRATION_DURATION_S = 8.0
MAXIMUM_GRIPPER_OPENING_RESIDUAL_RAD = 0.02
MINIMUM_GRIPPER_RELEASE_TRAVEL_FRACTION = 0.50
SETTLE_SPEED_THRESHOLD_M_S = 0.010
SETTLE_CONSECUTIVE_STEPS = 30
RELEASE_SHAPE_VIDEO_FPS = 24.0
RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M = 0.001
RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES = 5
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
FRICTIONAL_CONTACT_TCP_TARGET_Z_M = -0.0035
FRICTIONAL_PREGRASP_CLEARANCE_M = 0.035
MAXIMUM_FRICTIONAL_APPROACH_TILT_RAD = math.radians(
    12.0 if args.kinematic_replay is not None else 5.0
)
MAXIMUM_PAD_CENTER_TO_CLOTH_NODE_DISTANCE_M = (
    0.5 * math.sqrt(sum(value * value for value in JAW_PAD_SIZE_M))
    + 0.002
    + CLOTH_CONTACT_OFFSET_M
)
FRICTIONAL_CONTACT_TCP_TARGET_XY_M = {
    "left": (0.21907949565061063, 0.013682368332387042),
    "right": (0.21907949568872562, -0.2563176316579439),
}
FRICTIONAL_PREGRASP_ARM_POSITIONS_RAD = {
    "left": (
        -0.07468253616127002,
        1.5778099826813894,
        0.6411983605417193,
        -0.11237060605143258,
        -1.5968056898203895,
    ),
    "right": (
        0.05799923544340844,
        1.6480293857626278,
        0.6710594486946732,
        -0.17649353077395843,
        -1.3983619895256454,
    ),
}
FRICTIONAL_CONTACT_ARM_POSITIONS_RAD = {
    "left": (
        -0.07468204523647687,
        1.699190115766021,
        0.5139145743984298,
        0.13629331337797806,
        -1.5968051988940526,
    ),
    "right": (
        0.05256603636524205,
        1.7563785714781988,
        0.5342032771841445,
        0.06887617434070878,
        -1.4037993511932296,
    ),
}
if args.kinematic_replay is not None:
    # Reviewed edge-pinch candidate: 15 mm normal/endpoint inset with the
    # 300 mm towel shifted 20 mm toward the robot.  These branches target the
    # same -3.5 mm deep-contact / +35 mm pregrasp contract used above.
    FRICTIONAL_CONTACT_TCP_TARGET_XY_M = {
        "left": (0.15743161353744506, 0.0049052779526063),
        "right": (0.15743161353744506, -0.2650947220473937),
    }
    FRICTIONAL_PREGRASP_ARM_POSITIONS_RAD = {
        "left": (
            -0.04187776498190455,
            1.0938714545440296,
            0.1501531700974283,
            -0.14420820235341192,
            -1.5639692957399582,
        ),
        "right": (
            0.18427253630947346,
            1.3638393611373805,
            0.2390251968089166,
            -0.16564592268919207,
            -1.3089290165790317,
        ),
    }
    FRICTIONAL_CONTACT_ARM_POSITIONS_RAD = {
        "left": (
            -0.04188076042648766,
            1.295104867033977,
            0.03257266576626055,
            0.1474045296393404,
            -1.5639125364339652,
        ),
        "right": (
            0.1770470142364502,
            1.5075454711914062,
            0.20423518121242523,
            -0.025976072996854782,
            -1.3168001174926758,
        ),
    }


def load_manifest(path: Path) -> tuple[dict[str, object], dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    source = validate_s0_host_manifest(document)
    if args.kinematic_replay is None:
        return document, source
    replay = json.loads(args.kinematic_replay.read_text(encoding="utf-8"))
    if not isinstance(replay, dict):
        raise ValueError("kinematic replay root is not a mapping")
    canonical_replay = (
        replay.get("record_kind") == "canonical_towel_fold_full_fk_diagnostic"
        and replay.get("status") == "CANONICAL_TOWEL_FULL_FK_DIAGNOSTIC_PASS"
    )
    suspended_gravity_replay = (
        replay.get("record_kind") == "towel_suspended_gravity_full_fk_diagnostic"
        and replay.get("status")
        == "TOWEL_SUSPENDED_GRAVITY_FULL_FK_DIAGNOSTIC_PASS"
    )
    if not (canonical_replay or suspended_gravity_replay):
        raise ValueError("kinematic replay is not a passing supported full-FK diagnostic")
    placement = replay.get("towel_placement", {})
    bounds = placement.get("bounds_xyxy_m", ())
    if len(bounds) != 4:
        raise ValueError("kinematic replay is missing towel bounds")
    selected = replay.get("selected_candidate", {})
    first_fold = copy.deepcopy(selected.get("first_fold", ()))
    if args.right_arm_kinematic_replay is not None:
        right_replay = json.loads(
            args.right_arm_kinematic_replay.read_text(encoding="utf-8")
        )
        if (
            not isinstance(right_replay, dict)
            or right_replay.get("record_kind")
            != "towel_suspended_gravity_full_fk_diagnostic"
            or right_replay.get("status")
            != "TOWEL_SUSPENDED_GRAVITY_FULL_FK_DIAGNOSTIC_PASS"
            or not suspended_gravity_replay
        ):
            raise ValueError(
                "right-arm replay is not a passing suspended-gravity diagnostic"
            )
        right_bounds = right_replay.get("towel_placement", {}).get(
            "bounds_xyxy_m", ()
        )
        if list(right_bounds) != list(bounds):
            raise ValueError("left/right replay towel bounds differ")
        right_records = {
            record.get("name"): record
            for record in right_replay.get("selected_candidate", {}).get(
                "first_fold", ()
            )
        }
        if set(right_records) != {record.get("name") for record in first_fold}:
            raise ValueError("left/right replay phase sets differ")
        if (
            args.right_arm_kinematic_replay_through is not None
            and args.right_arm_kinematic_replay_through not in right_records
        ):
            raise ValueError("right-arm replay cutoff phase does not exist")
        merge_right_arm = True
        for record in first_fold:
            right_record = right_records[record["name"]]
            if merge_right_arm and "arm_joint_positions_rad" in record:
                record["arm_joint_positions_rad"]["right"] = copy.deepcopy(
                    right_record["arm_joint_positions_rad"]["right"]
                )
            if merge_right_arm and "joint_positions_rad" in record:
                record["joint_positions_rad"][6:12] = copy.deepcopy(
                    right_record["joint_positions_rad"][6:12]
                )
            left_targets = record.get("targets", [])
            right_targets = right_record.get("targets", [])
            for index, target in enumerate(left_targets):
                if merge_right_arm and target.get("arm") == "right":
                    matching_index = next(
                        candidate_index
                        for candidate_index, candidate in enumerate(right_targets)
                        if candidate.get("arm") == "right"
                    )
                    left_targets[index] = copy.deepcopy(
                        right_targets[matching_index]
                    )
                    evaluations = record.get("task_pose_evaluations", [])
                    right_evaluations = right_record.get(
                        "task_pose_evaluations", []
                    )
                    if len(evaluations) == len(left_targets) and len(
                        right_evaluations
                    ) == len(right_targets):
                        evaluations[index] = copy.deepcopy(
                            right_evaluations[matching_index]
                        )
            if (
                record["name"] == args.right_arm_kinematic_replay_through
            ):
                merge_right_arm = False
    names = {record.get("name") for record in first_fold}
    required = (
        {
            "first_contact",
            "first_suspend_lift_01",
            "first_gravity_overcenter_03",
            "first_gravity_preopen_clearance_02",
            "first_gravity_retreat",
            "first_gravity_reobserve_clear",
        }
        if suspended_gravity_replay
        else {"first_contact", "first_retreat"}
        | {f"first_fold_{index:02d}" for index in range(1, 17)}
    )
    if not required.issubset(names):
        raise ValueError("kinematic replay is missing required first-fold phases")
    source = copy.deepcopy(source)
    source["canonical_replay"]["first_fold"] = copy.deepcopy(first_fold)
    source["suspended_gravity_replay"] = suspended_gravity_replay
    source["kinematic_replay_path"] = str(args.kinematic_replay.resolve())
    source["right_arm_kinematic_replay_path"] = (
        str(args.right_arm_kinematic_replay.resolve())
        if args.right_arm_kinematic_replay is not None
        else None
    )
    source["right_arm_kinematic_replay_through"] = (
        args.right_arm_kinematic_replay_through
    )
    center_x = 0.5 * (float(bounds[0]) + float(bounds[1]))
    center_y = 0.5 * (float(bounds[2]) + float(bounds[3]))
    for pose in source["rigid_proxy_pose_xyz_yaw_rad"]:
        pose[0] = center_x
        pose[1] = center_y
    return document, source


def phase(source: dict[str, object], name: str) -> dict[str, object]:
    for fold_name in ("first_fold", "second_fold"):
        for record in source["canonical_replay"][fold_name]:
            if record["name"] == name:
                return record
    raise ValueError(f"missing canonical replay phase: {name}")


def model_joint_positions(
    source: dict[str, object],
    project_positions_rad: list[float] | tuple[float, ...],
    *,
    gripper_project_positions_rad: dict[str, float] | None = None,
) -> list[float]:
    """Map only grippers from canonical project radians into mesh-model radians."""
    names = [str(name) for name in source["joint_names"]]
    if len(names) != len(project_positions_rad):
        raise ValueError("joint name and position counts differ")
    result = [float(value) for value in project_positions_rad]
    for side in ("left", "right"):
        name = f"{side}_gripper_joint"
        try:
            index = names.index(name)
        except ValueError as exc:
            raise ValueError(f"canonical joint order is missing {name}") from exc
        project_value = (
            float(project_positions_rad[index])
            if gripper_project_positions_rad is None
            else float(gripper_project_positions_rad[side])
        )
        result[index] = gripper_candidate.project_to_model(project_value)
    return result


def contact_model_joint_positions(source: dict[str, object]) -> list[float]:
    """Return Q0-open contact state with the vertical-pinch IK candidate."""
    project_positions = [
        float(value) for value in phase(source, "first_contact")["joint_positions_rad"]
    ]
    if args.grasp_mode != "legacy-attachment":
        project_positions[0:5] = FRICTIONAL_CONTACT_ARM_POSITIONS_RAD["left"]
        project_positions[6:11] = FRICTIONAL_CONTACT_ARM_POSITIONS_RAD["right"]
    return model_joint_positions(
        source,
        project_positions,
        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
    )


def initial_model_joint_positions(source: dict[str, object]) -> list[float]:
    """Start frictional trials vertically above contact instead of inside it."""
    if args.grasp_mode == "legacy-attachment":
        return contact_model_joint_positions(source)
    project_positions = [
        float(value) for value in phase(source, "first_contact")["joint_positions_rad"]
    ]
    project_positions[0:5] = FRICTIONAL_PREGRASP_ARM_POSITIONS_RAD["left"]
    project_positions[6:11] = FRICTIONAL_PREGRASP_ARM_POSITIONS_RAD["right"]
    return model_joint_positions(
        source,
        project_positions,
        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
    )


def scene_config(source: dict[str, object]) -> InteractiveSceneCfg:
    table_geometry = source["worktable_geometry"]
    table_size = tuple(float(value) for value in table_geometry["size_xyz_m"])
    table_pose = tuple(float(value) for value in table_geometry["pose_xyz_m"])
    proxy_pose = source["rigid_proxy_pose_xyz_yaw_rad"][0]
    table_top_z_m = table_pose[2] + 0.5 * table_size[2]
    initial_joint_positions = initial_model_joint_positions(source)
    contact_joint_map = {
        str(name): float(position)
        for name, position in zip(source["joint_names"], initial_joint_positions, strict=True)
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
                # The measured coefficient describes the towel/table pair.
                # Author it on both sides so PhysX does not combine the towel
                # candidate with its lower 0.5 default rigid material.
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=CLOTH_STATIC_FRICTION,
                    dynamic_friction=CLOTH_DYNAMIC_FRICTION,
                    restitution=0.0,
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
                deformable_props=(
                    NewtonDeformableBodyPropertiesCfg()
                    if args.physics_backend == "newton-coupled-vbd"
                    else PhysxDeformableBodyPropertiesCfg(
                        mass=CLOTH_MASS_KG,
                        solver_position_iteration_count=24,
                        linear_damping=CLOTH_LINEAR_DAMPING_S_INV,
                        settling_damping=CLOTH_SETTLING_DAMPING_S_INV,
                        settling_threshold=CLOTH_SETTLING_THRESHOLD_M_S,
                        sleep_threshold=0.005,
                        max_depenetration_velocity=0.5,
                        # A flat cloth can block the closing jaws when self-collision is
                        # active during the pinch. Stage it off, validate actual vertical
                        # jaw closure first, then enable it before lift/fold.
                        self_collision=False,
                        self_collision_filter_distance=(
                            SELF_COLLISION_FILTER_DISTANCE_M
                            if args.self_contact
                            else None
                        ),
                        contact_offset=CLOTH_CONTACT_OFFSET_M,
                        rest_offset=CLOTH_REST_OFFSET_M,
                        collision_pair_update_frequency=4,
                        collision_iteration_multiplier=2.0,
                    )
                ),
                physics_material=(
                    NewtonSurfaceDeformableBodyMaterialCfg(
                        # Newton cloth density is areal (kg/m^2), unlike the
                        # volumetric PhysX material field.
                        density=NEWTON_CLOTH_AREAL_DENSITY_KG_M2,
                        particle_radius=CLOTH_CONTACT_OFFSET_M,
                        tri_ke=NEWTON_TRIANGLE_STIFFNESS_PA,
                        tri_ka=NEWTON_TRIANGLE_AREA_STIFFNESS_PA,
                        tri_kd=NEWTON_TRIANGLE_DAMPING_PA_S,
                        edge_ke=NEWTON_EDGE_STIFFNESS_N_M,
                        edge_kd=NEWTON_EDGE_DAMPING_N_M_S,
                    )
                    if args.physics_backend == "newton-coupled-vbd"
                    else PhysxSurfaceDeformableBodyMaterialCfg(
                        density=CLOTH_DENSITY_KG_M3,
                        static_friction=CLOTH_STATIC_FRICTION,
                        dynamic_friction=CLOTH_DYNAMIC_FRICTION,
                        youngs_modulus=CLOTH_YOUNGS_MODULUS_PA,
                        poissons_ratio=CLOTH_POISSONS_RATIO,
                        elasticity_damping=CLOTH_ELASTICITY_DAMPING,
                        surface_bend_stiffness=CLOTH_SURFACE_BEND_STIFFNESS_PA,
                        bend_damping=CLOTH_BEND_DAMPING_S_INV,
                        surface_thickness=CLOTH_SURFACE_THICKNESS_M,
                    )
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.05, 0.45, 0.85)
                ),
            ),
            init_state=DeformableObjectCfg.InitialStateCfg(
                pos=(
                    float(proxy_pose[0]),
                    float(proxy_pose[1]),
                    table_top_z_m
                    + (
                        CLOTH_CONTACT_OFFSET_M + 0.002
                        if args.physics_backend == "newton-coupled-vbd"
                        else CLOTH_INITIAL_CLEARANCE_M
                    ),
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
                "arm_joints": ImplicitActuatorCfg(
                    joint_names_expr=[
                        ".*_(base|shoulder|elbow|wrist_flex|wrist_roll)_joint"
                    ],
                    effort_limit_sim=10.0,
                    velocity_limit_sim=10.0,
                    stiffness=1000.0,
                    damping=100.0,
                ),
                "gripper_joints": ImplicitActuatorCfg(
                    joint_names_expr=[".*_gripper_joint"],
                    effort_limit_sim=2.0,
                    velocity_limit_sim=2.0,
                    stiffness=20.0,
                    damping=1.0,
                ),
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
    thin_axis_parent: tuple[float, float, float],
    material: UsdShade.Material,
) -> None:
    """Author an invisible collision pad over a reviewed jaw-mesh contact face."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center_parent_m))
    pad_quaternion = Gf.Rotation(
        Gf.Vec3d(1.0, 0.0, 0.0), Gf.Vec3d(*thin_axis_parent)
    ).GetQuat()
    cube.AddOrientOp().Set(
        Gf.Quatf(
            float(pad_quaternion.GetReal()),
            Gf.Vec3f(*[float(value) for value in pad_quaternion.GetImaginary()]),
        )
    )
    cube.AddScaleOp().Set(Gf.Vec3f(*size_parent_m))
    cube.MakeInvisible()
    collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision.CreateCollisionEnabledAttr().Set(True)
    if args.physics_backend == "physx":
        api = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        api.CreateContactOffsetAttr().Set(0.002)
        api.CreateRestOffsetAttr().Set(0.0)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        material, materialPurpose="physics"
    )


def _author_rubber_material(stage: Usd.Stage, path: str) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    static_friction = (
        args.newton_rubber_friction
        if args.physics_backend == "newton-coupled-vbd"
        and args.newton_rubber_friction is not None
        else gripper_candidate.rubber_static_friction
    )
    dynamic_friction = (
        static_friction
        if args.physics_backend == "newton-coupled-vbd"
        and args.newton_rubber_friction is not None
        else gripper_candidate.rubber_dynamic_friction
    )
    physics_material.CreateStaticFrictionAttr(static_friction)
    physics_material.CreateDynamicFrictionAttr(dynamic_friction)
    physics_material.CreateRestitutionAttr(gripper_candidate.rubber_restitution)
    return material


def _disable_imported_gripper_mesh_collisions(
    stage: Usd.Stage, body_paths: tuple[str, str], side: str
) -> int:
    def instance_contains_collision(instance_prim: Usd.Prim) -> bool:
        if not instance_prim.IsInstance():
            return False
        prototype = instance_prim.GetPrototype()
        return prototype.IsValid() and any(
            descendant.HasAPI(UsdPhysics.CollisionAPI)
            for descendant in Usd.PrimRange(prototype)
        )

    disabled = 0
    for body_path in body_paths:
        body = stage.GetPrimAtPath(body_path)
        if not body.IsValid():
            raise RuntimeError(f"missing resolved {side} jaw body: {body_path}")
        for prim in Usd.PrimRange(body):
            if prim != body and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                # The moving jaw is handled from its own body root.
                continue
            if instance_contains_collision(prim):
                # URDF Converter 0.1.3 authors collision meshes inside an
                # instance prototype. Instance proxies are not editable, so
                # disable the collision-only instance root instead.
                prim.SetActive(False)
                disabled += 1
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                disabled += 1
    if disabled == 0:
        raise RuntimeError(f"no imported {side} gripper collisions were disabled")
    return disabled


def _apply_gripper_model_joint_limits(
    stage: Usd.Stage, robot_prefix: str, side: str
) -> tuple[float, float]:
    expected_name = f"{side}_gripper_joint"
    matches = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_prefix)
        and prim.GetName() == expected_name
        and prim.IsA(UsdPhysics.RevoluteJoint)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one imported {expected_name}, found {len(matches)}"
        )
    lower_rad, upper_rad = gripper_candidate.model_limits_rad(side)
    joint = UsdPhysics.RevoluteJoint(matches[0])
    joint.CreateLowerLimitAttr().Set(math.degrees(lower_rad))
    joint.CreateUpperLimitAttr().Set(math.degrees(upper_rad))
    return lower_rad, upper_rad


def apply_shape_contact_offsets(environment_count: int) -> None:
    stage = omni.usd.get_context().get_stage()
    for environment_index in range(environment_count):
        if args.physics_backend == "physx":
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
        jaw_paths = rigid_jaw_paths(environment_index)
        for side in ("left", "right"):
            fixed_body_path = jaw_paths[side]["fixed"]
            moving_body_path = jaw_paths[side]["moving"]
            if not args.actual_jaw_mesh_contact:
                _disable_imported_gripper_mesh_collisions(
                    stage, (fixed_body_path, moving_body_path), side
                )
            _apply_gripper_model_joint_limits(stage, robot_prefix, side)
            rubber_material = _author_rubber_material(
                stage, f"{robot_prefix}/{side}_TowelJawRubberMaterial"
            )
            # Fixed-jaw face: TCP x=0, raw camera-mount mesh x=-7.9 mm.
            _author_jaw_pad(
                stage,
                f"{fixed_body_path}/TowelFixedJawCollider",
                FIXED_JAW_PAD_CENTER_PARENT_M,
                (
                    gripper_candidate.fixed_jaw_rubber_pad_thickness_m,
                    JAW_PAD_SIZE_M[1],
                    JAW_PAD_SIZE_M[2],
                ),
                JAW_PAD_NORMALS_PARENT[side]["fixed"],
                rubber_material,
            )
            if not args.actual_jaw_mesh_contact:
                # Moving-jaw face proxy used only by the legacy simplified probe.
                _author_jaw_pad(
                    stage,
                    f"{moving_body_path}/TowelMovingJawCollider",
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                    JAW_PAD_SIZE_M,
                    JAW_PAD_NORMALS_PARENT[side]["moving"],
                    rubber_material,
                )


def enable_cloth_self_collision_after_pinch(environment_count: int) -> list[str]:
    """Enable deformable self-collision only after the closed-jaw gates pass."""
    if args.physics_backend == "newton-coupled-vbd":
        # Newton VBD compiles particle self-contact into the solver at model
        # finalization and does not expose a PhysX deformable USD owner here.
        return ["newton_vbd_solver:particle_self_contact"]
    stage = omni.usd.get_context().get_stage()
    authored_paths: list[str] = []
    for environment_index in range(environment_count):
        cloth_root = stage.GetPrimAtPath(
            f"/World/envs/env_{environment_index}/TowelCloth"
        )
        if not cloth_root.IsValid():
            raise RuntimeError(
                f"missing towel root for staged self-collision in env {environment_index}"
            )
        candidates = [
            prim
            for prim in Usd.PrimRange(cloth_root)
            if prim.GetAttribute("physxDeformableBody:selfCollision").IsValid()
        ]
        if len(candidates) != 1:
            raise RuntimeError(
                "expected one deformable self-collision owner in "
                f"env {environment_index}, found "
                f"{[str(prim.GetPath()) for prim in candidates]}"
            )
        prim = candidates[0]
        self_collision_attr = prim.GetAttribute(
            "physxDeformableBody:selfCollision"
        )
        filter_distance_attr = prim.GetAttribute(
            "physxDeformableBody:selfCollisionFilterDistance"
        )
        if not filter_distance_attr.IsValid():
            filter_distance_attr = prim.CreateAttribute(
                "physxDeformableBody:selfCollisionFilterDistance",
                Sdf.ValueTypeNames.Float,
            )
        filter_distance_attr.Set(SELF_COLLISION_FILTER_DISTANCE_M)
        self_collision_attr.Set(True)
        if self_collision_attr.Get() is not True:
            raise RuntimeError(
                f"failed to enable staged cloth self-collision at {prim.GetPath()}"
            )
        authored_paths.append(str(prim.GetPath()))
    return authored_paths


def rigid_jaw_paths(environment_index: int) -> dict[str, dict[str, str]]:
    """Resolve imported jaw bodies by identity instead of assuming USD nesting."""
    stage = omni.usd.get_context().get_stage()
    prefix = f"/World/envs/env_{environment_index}/Robot"
    result: dict[str, dict[str, str]] = {
        "left": {},
        "right": {},
    }
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        for side in ("left", "right"):
            if prim.GetName() == f"{side}_gripper_link":
                result[side]["fixed"] = path
            elif prim.GetName() == f"{side}_moving_jaw_link":
                result[side]["moving"] = path
    incomplete = {
        side: paths
        for side, paths in result.items()
        if set(paths) != {"fixed", "moving"}
    }
    if incomplete:
        raise RuntimeError(f"could not resolve rigid jaw bodies: {incomplete}")
    return result


def rigid_gripper_paths(environment_index: int) -> dict[str, str]:
    return {
        side: paths["fixed"]
        for side, paths in rigid_jaw_paths(environment_index).items()
    }


def author_contact_joint_states(source: dict[str, object], environment_count: int) -> None:
    """Set initial arm state and physical-Q0-open grippers before simulation."""
    if args.physics_backend == "newton-coupled-vbd":
        # ArticulationCfg.init_state and the explicit tensor write before the
        # first step are authoritative for Newton.
        return
    stage = omni.usd.get_context().get_stage()
    contact_positions = initial_model_joint_positions(source)
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


def newton_soft_contact_snapshot() -> dict[str, object] | None:
    """Return body-particle contacts with their imported USD shape labels."""
    if args.physics_backend != "newton-coupled-vbd":
        return None
    contacts = NewtonManager.get_contacts()
    model = NewtonManager.get_model()
    if contacts is None or model is None:
        return {"available": False, "reason": "missing_model_or_contact_buffer"}
    count_values = contacts.soft_contact_count.numpy().reshape(-1)
    count = int(count_values[0]) if count_values.size else 0
    shape_indices = contacts.soft_contact_shape.numpy().reshape(-1)[:count]
    particle_indices = contacts.soft_contact_particle.numpy().reshape(-1)[:count]
    labels = [
        model.shape_label[int(index)]
        if 0 <= int(index) < len(model.shape_label)
        else f"<shape:{int(index)}>"
        for index in shape_indices
    ]
    label_counts: dict[str, int] = {}
    particles_by_label: dict[str, list[int]] = {}
    for label, particle_index in zip(labels, particle_indices, strict=True):
        label_counts[label] = label_counts.get(label, 0) + 1
        particles_by_label.setdefault(label, []).append(int(particle_index))
    jaw_labels = {
        label: sorted(set(particles))
        for label, particles in particles_by_label.items()
        if "TowelFixedJawCollider" in label
        or "TowelMovingJawCollider" in label
        or "gripper_link" in label
        or "moving_jaw_link" in label
    }
    bilateral_particles_by_side: dict[str, list[int]] = {}
    for side in ("left", "right"):
        fixed_particles: set[int] = set()
        moving_particles: set[int] = set()
        for label, particles in jaw_labels.items():
            if f"/{side}_" not in label:
                continue
            if "TowelFixedJawCollider" in label or (
                f"/{side}_gripper_link/" in label
                and f"/{side}_moving_jaw_link/" not in label
            ):
                fixed_particles.update(particles)
            elif "TowelMovingJawCollider" in label or (
                f"/{side}_moving_jaw_link/" in label
            ):
                moving_particles.update(particles)
        bilateral_particles_by_side[side] = sorted(
            fixed_particles & moving_particles
        )
    return {
        "available": True,
        "soft_contact_count": count,
        "shape_contact_counts": label_counts,
        "jaw_particles_by_shape": jaw_labels,
        "bilateral_same_particle_contacts": bilateral_particles_by_side,
        "jaw_shape_friction": {
            model.shape_label[index]: float(model.shape_material_mu.numpy()[index])
            for index in range(len(model.shape_label))
            if "/Robot/" in model.shape_label[index]
            and (
                "TowelFixedJawCollider" in model.shape_label[index]
                or "moving_jaw_link" in model.shape_label[index]
            )
        },
    }


def apply_explicit_newton_fixed_pad_friction() -> dict[str, float] | None:
    """Apply the no-slip candidate to fixed rubber pads after model finalization."""
    if (
        args.physics_backend != "newton-coupled-vbd"
        or args.newton_rubber_friction is None
    ):
        return None
    model = NewtonManager.get_model()
    values = model.shape_material_mu.numpy()
    applied: dict[str, float] = {}
    for index, label in enumerate(model.shape_label):
        if "/Robot/" in label and "TowelFixedJawCollider" in label:
            values[index] = args.newton_rubber_friction
            applied[label] = float(values[index])
    if len(applied) != 2:
        raise RuntimeError(
            f"expected two finalized fixed rubber pad shapes, found {sorted(applied)}"
        )
    model.shape_material_mu.assign(values)
    return applied


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


def gripper_approach_axes_w(
    gripper_orientations_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Return physical jaw-tip approach axes; gripper-frame +Z is link -Z."""
    result = torch.empty_like(gripper_orientations_xyzw[..., :3])
    for environment_index in range(gripper_orientations_xyzw.shape[0]):
        for side_index in range(gripper_orientations_xyzw.shape[1]):
            orientation = gripper_orientations_xyzw[
                environment_index, side_index
            ].tolist()
            rotation = Gf.Rotation(
                Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
            )
            approach = rotation.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
            result[environment_index, side_index] = torch.tensor(
                [approach[index] for index in range(3)],
                dtype=result.dtype,
                device=result.device,
            )
    return result


def body_local_points_to_world(
    body_positions_w: torch.Tensor,
    body_orientations_xyzw: torch.Tensor,
    local_points: tuple[tuple[float, float, float], ...],
) -> torch.Tensor:
    """Transform one registered local point for each rigid body into world space."""
    if body_positions_w.shape[1] != len(local_points):
        raise ValueError("body count and local point count differ")
    result = torch.empty_like(body_positions_w)
    for environment_index in range(body_positions_w.shape[0]):
        for body_index, local_point in enumerate(local_points):
            position = body_positions_w[environment_index, body_index].tolist()
            orientation = body_orientations_xyzw[
                environment_index, body_index
            ].tolist()
            rotation = Gf.Rotation(
                Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
            )
            offset = rotation.TransformDir(Gf.Vec3d(*local_point))
            result[environment_index, body_index] = torch.tensor(
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


def authoritative_newton_nodes_w(
    environment_count: int, device: str
) -> torch.Tensor:
    """Read the current coupled-VBD particle state after a direct constraint write."""
    state = NewtonManager.get_state()
    values = state.particle_q.numpy().copy()
    if values.shape != (environment_count * 1024, 3):
        raise RuntimeError(f"unexpected Newton particle state shape: {values.shape}")
    return torch.as_tensor(values, dtype=torch.float32, device=device).reshape(
        environment_count, 1024, 3
    )


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


ARM_JOINT_INDICES = (0, 1, 2, 3, 4, 6, 7, 8, 9, 10)
GRIPPER_JOINT_INDICES = (5, 11)


def write_scripted_arm_state_and_drive_targets(
    robot: object,
    positions: torch.Tensor,
    zero_velocity: torch.Tensor,
    joint_ids: list[int],
    *,
    initialize_arm_state: bool = False,
    lock_gripper_state: bool = False,
) -> None:
    """Command drives, with explicit jaw state only when Newton cannot honor limits."""
    if initialize_arm_state:
        arm_joint_ids = [joint_ids[index] for index in ARM_JOINT_INDICES]
        robot.write_joint_state_to_sim_index(
            position=positions[:, ARM_JOINT_INDICES],
            velocity=zero_velocity[:, ARM_JOINT_INDICES],
            joint_ids=arm_joint_ids,
        )
    if lock_gripper_state:
        gripper_joint_ids = [joint_ids[index] for index in GRIPPER_JOINT_INDICES]
        robot.write_joint_state_to_sim_index(
            position=positions[:, GRIPPER_JOINT_INDICES],
            velocity=zero_velocity[:, GRIPPER_JOINT_INDICES],
            joint_ids=gripper_joint_ids,
        )
    robot.set_joint_position_target_index(target=positions, joint_ids=joint_ids)


def maximum_arm_target_residual_rad(
    robot: object, target: torch.Tensor, joint_ids: list[int]
) -> float:
    achieved = robot.data.joint_pos.torch[:, joint_ids]
    return float(
        torch.max(
            torch.abs(achieved[:, ARM_JOINT_INDICES] - target[:, ARM_JOINT_INDICES])
        ).item()
    )


def require_arm_target_reached(
    robot: object,
    target: torch.Tensor,
    joint_ids: list[int],
    phase_name: str,
) -> None:
    achieved = robot.data.joint_pos.torch[:, joint_ids]
    absolute_error = torch.abs(achieved - target)
    left_residual = float(torch.max(absolute_error[:, 0:5]).item())
    right_residual = float(torch.max(absolute_error[:, 6:11]).item())
    residual = max(left_residual, right_residual)
    if residual > MAXIMUM_ARM_TARGET_RESIDUAL_RAD:
        raise RuntimeError(
            f"{phase_name} arm drive residual {residual:.6f} rad exceeds "
            f"{MAXIMUM_ARM_TARGET_RESIDUAL_RAD:.6f} rad; "
            f"left={left_residual:.6f}, right={right_residual:.6f}, "
            f"achieved_env_0={achieved[0].tolist()}, "
            f"target_env_0={target[0].tolist()}; collision or tracking failure"
        )


def settle_physical_arm_drives(
    robot: object,
    target: torch.Tensor,
    zero_velocity: torch.Tensor,
    joint_ids: list[int],
    scene: object,
    sim: object,
    cloth: object,
    physics_dt_s: float,
    timeout_s: float,
    phase_name: str,
    post_step_callback: object | None = None,
    lock_gripper_state: bool = False,
) -> None:
    """Hold a drive target until reached, or preserve a real collision failure."""
    residual = maximum_arm_target_residual_rad(robot, target, joint_ids)
    maximum_steps = max(1, math.ceil(timeout_s / physics_dt_s))
    settled_step = 0
    while residual > MAXIMUM_ARM_TARGET_RESIDUAL_RAD and settled_step < maximum_steps:
        settled_step += 1
        write_scripted_arm_state_and_drive_targets(
            robot,
            target,
            zero_velocity,
            joint_ids,
            lock_gripper_state=lock_gripper_state,
        )
        if post_step_callback is not None:
            post_step_callback()
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
        if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
            raise RuntimeError(
                f"cloth produced non-finite nodes while settling {phase_name}"
            )
        residual = maximum_arm_target_residual_rad(robot, target, joint_ids)
    require_arm_target_reached(robot, target, joint_ids, phase_name)
    print(
        f"S1_ARM_TARGET_SETTLED phase={phase_name} "
        f"hold_s={settled_step * physics_dt_s:.6f} residual_rad={residual:.6f}",
        flush=True,
    )


def phase_model_tensor(
    source: dict[str, object],
    phase_record: dict[str, object],
    *,
    gripper_project_positions_rad: dict[str, float],
    environment_count: int,
    device: str,
) -> torch.Tensor:
    positions = model_joint_positions(
        source,
        phase_record["joint_positions_rad"],
        gripper_project_positions_rad=gripper_project_positions_rad,
    )
    return torch.tensor(positions, dtype=torch.float32, device=device).repeat(
        environment_count, 1
    )


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    if args.urdf_override is not None:
        source = copy.deepcopy(source)
        source["urdf_path"] = args.urdf_override.resolve()
        source["urdf_sha256"] = hashlib.sha256(
            args.urdf_override.read_bytes()
        ).hexdigest()
    scripted_pre_touchdown_hold_s = (
        SUSPENDED_PRE_TOUCHDOWN_HOLD_S
        if args.scripted_pre_touchdown_hold_s is None
        else args.scripted_pre_touchdown_hold_s
    )
    scripted_post_touchdown_hold_s = (
        POST_TOUCHDOWN_ANCHOR_HOLD_S
        if args.scripted_post_touchdown_hold_s is None
        else args.scripted_post_touchdown_hold_s
    )
    table_size_for_contact_gate = source["worktable_geometry"]["size_xyz_m"]
    table_pose_for_contact_gate = source["worktable_geometry"]["pose_xyz_m"]
    table_top_z_m_for_contact_gate = float(table_pose_for_contact_gate[2]) + 0.5 * float(
        table_size_for_contact_gate[2]
    )
    post_release_correction_replay = None
    correction_probe_id = None
    if args.post_release_correction_replay is not None:
        post_release_correction_replay = json.loads(
            args.post_release_correction_replay.read_text(encoding="utf-8")
        )
        if (
            not isinstance(post_release_correction_replay, dict)
            or post_release_correction_replay.get("record_kind")
            != "towel_free_edge_correction_full_fk_diagnostic"
            or post_release_correction_replay.get("status")
            != "TOWEL_FREE_EDGE_CORRECTION_FULL_FK_PASS"
        ):
            raise ValueError(
                "post-release correction is not a passing full-FK diagnostic"
            )
        correction_source = post_release_correction_replay.get("source", {})
        if correction_source.get("urdf_sha256") != source["urdf_sha256"]:
            raise ValueError(
                "post-release correction and Isaac manifest use different URDFs"
            )
        expected_route_sha256 = hashlib.sha256(
            args.kinematic_replay.read_bytes()
        ).hexdigest()
        if correction_source.get("route_source_sha256") != expected_route_sha256:
            raise ValueError(
                "post-release correction was not derived from the selected first fold"
            )
        correction_records = post_release_correction_replay.get("phases", [])
        correction_names = [record.get("name") for record in correction_records]
        correction_probe_id = post_release_correction_replay.get(
            "correction_envelope_entry", {}
        ).get("probe_id")
        if (
            not isinstance(correction_probe_id, str)
            or not correction_probe_id
            or any(character.isspace() for character in correction_probe_id)
        ):
            raise ValueError("post-release correction probe id is invalid")
        correction_contact_name = f"{correction_probe_id}_contact"
        correction_target_name = f"{correction_probe_id}_target"
        correction_reobserve_name = f"{correction_probe_id}_reobserve_clear"
        if (
            len(correction_records) < 5
            or len(set(correction_names)) != len(correction_names)
            or correction_contact_name not in correction_names
            or correction_target_name not in correction_names
            or correction_names[-1] != correction_reobserve_name
            or correction_names.index(correction_contact_name)
            >= correction_names.index(correction_target_name)
        ):
            raise ValueError("post-release correction phase sequence is incomplete")
    if args.environment_count is not None:
        source = copy.deepcopy(source)
        source["environment_count"] = args.environment_count
    environment_count = int(source["environment_count"])
    physics_dt_s = SELF_CONTACT_PHYSICS_DT_S if args.self_contact else PHYSICS_DT_S
    suspended_gravity_replay = bool(source.get("suspended_gravity_replay", False))
    contact = phase(source, "first_contact")
    lift = phase(
        source,
        "first_suspend_lift_01" if suspended_gravity_replay else "first_fold_01",
    )
    legacy_attachment_used = args.grasp_mode == "legacy-attachment"
    contact_gated_retention_used = args.grasp_mode == "contact-gated-retention"
    scripted_attachment_used = legacy_attachment_used
    newton_state_retention_used = contact_gated_retention_used
    vertical_grasp_used = not legacy_attachment_used
    print(
        f"S1_VERTEX_PATCH_START environments={environment_count} "
        "gripper=q0_aligned_drive_not_state_overwrite "
        f"grasp_mode={args.grasp_mode} motion_commands=0",
        flush=True,
    )
    if args.physics_backend == "newton-coupled-vbd":
        @configclass
        class TowelNewtonCfg(NewtonCfg):
            model_cfg: NewtonModelCfg | None = None

        physics_cfg = TowelNewtonCfg(
            solver_cfg=CoupledMJWarpVBDSolverCfg(
                rigid_solver_cfg=MJWarpSolverCfg(
                    njmax=128,
                    nconmax=256,
                    ls_iterations=20,
                    cone="pyramidal",
                    integrator="implicitfast",
                    ccd_iterations=100,
                ),
                soft_solver_cfg=VBDSolverCfg(
                    iterations=10,
                    integrate_with_external_rigid_solver=True,
                    particle_enable_self_contact=args.self_contact,
                    particle_self_contact_radius=CLOTH_CONTACT_OFFSET_M,
                    particle_self_contact_margin=2.0 * CLOTH_CONTACT_OFFSET_M,
                    particle_collision_detection_interval=(
                        1 if args.self_contact else -1
                    ),
                    particle_topological_contact_filter_threshold=(
                        SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD
                    ),
                ),
                coupling_mode="two_way",
            ),
            model_cfg=NewtonModelCfg(
                soft_contact_ke=args.newton_contact_stiffness,
                soft_contact_kd=1.0e-2,
                soft_contact_mu=CLOTH_STATIC_FRICTION,
                shape_material_ke=args.newton_contact_stiffness,
                shape_material_kd=1.0e-2,
                # Preserve per-shape friction: measured towel/table candidate,
                # generic plastic moving jaw, and fixed-jaw rubber material.
                shape_material_mu=None,
            ),
            num_substeps=10,
            use_cuda_graph=True,
        )
    else:
        physics_cfg = PhysxCfg(
            enable_external_forces_every_iteration=args.self_contact,
            enable_enhanced_determinism=args.self_contact,
        )
    sim = SimulationContext(
        sim_utils.SimulationCfg(
            dt=physics_dt_s,
            render_interval=args.simulation_render_interval,
            device=args.device,
            physics=physics_cfg,
        )
    )
    scene = InteractiveScene(scene_config(source))
    curvature_softening_runtime: dict[str, object] = {}
    if args.newton_curvature_softening:
        activation_angle_rad = math.radians(
            args.newton_softening_activation_angle_deg
        )
        full_softening_angle_rad = math.radians(
            args.newton_full_softening_angle_deg
        )

        def initialize_curvature_softening(_event: object) -> None:
            model = NewtonManager.get_model()
            softened_edges = wp.zeros(
                model.edge_count, dtype=wp.int32, device=args.device
            )
            ever_softened_edges = wp.zeros(
                model.edge_count, dtype=wp.int32, device=args.device
            )
            peak_absolute_angles = wp.zeros(
                model.edge_count, dtype=wp.float32, device=args.device
            )

            def apply_curvature_softening() -> None:
                state = NewtonManager.get_state()
                wp.launch(
                    update_newton_curvature_softening,
                    dim=model.edge_count,
                    inputs=[
                        state.particle_q,
                        model.edge_indices,
                        activation_angle_rad,
                        full_softening_angle_rad,
                        NEWTON_EDGE_STIFFNESS_N_M,
                        args.newton_softened_edge_stiffness,
                        softened_edges,
                        ever_softened_edges,
                        peak_absolute_angles,
                        model.edge_bending_properties,
                    ],
                    device=args.device,
                )

            curvature_softening_runtime.update(
                {
                    "model_edge_count": int(model.edge_count),
                    "softened_edges": softened_edges,
                    "ever_softened_edges": ever_softened_edges,
                    "peak_absolute_angles": peak_absolute_angles,
                }
            )
            NewtonManager.register_post_actuator_callback(
                apply_curvature_softening
            )

        NewtonManager.register_callback(
            initialize_curvature_softening,
            PhysicsEvent.PHYSICS_READY,
            name="towel_high_curvature_softening",
        )
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
    explicit_newton_pad_material = apply_explicit_newton_fixed_pad_friction()
    if explicit_newton_pad_material is not None:
        print(
            "S1_NEWTON_FIXED_PAD_FRICTION_APPLIED "
            + json.dumps(explicit_newton_pad_material, sort_keys=True),
            flush=True,
        )
    robot = scene["robot"]
    cloth = scene["cloth"]
    joint_ids, imported_names = robot.find_joints(source["joint_names"], preserve_order=True)
    if imported_names != source["joint_names"] or len(joint_ids) != 12:
        raise RuntimeError("imported articulation does not match canonical 12-joint order")
    if suspended_gravity_replay:
        deep_contact_row = phase_model_tensor(
            source,
            contact,
            gripper_project_positions_rad={"left": 0.0, "right": 0.0},
            environment_count=environment_count,
            device=sim.device,
        )
        initial_row = phase_model_tensor(
            source,
            lift,
            gripper_project_positions_rad={"left": 0.0, "right": 0.0},
            environment_count=environment_count,
            device=sim.device,
        )
    else:
        deep_contact_row = torch.tensor(
            contact_model_joint_positions(source),
            dtype=torch.float32,
            device=sim.device,
        ).repeat(environment_count, 1)
        initial_row = torch.tensor(
            initial_model_joint_positions(source),
            dtype=torch.float32,
            device=sim.device,
        ).repeat(environment_count, 1)
    descent_fraction_by_side = {
        side: (
            getattr(args, f"{side}_frictional_descent_fraction")
            if getattr(args, f"{side}_frictional_descent_fraction") is not None
            else args.frictional_descent_fraction
        )
        for side in ("left", "right")
    }
    contact_row = deep_contact_row
    if vertical_grasp_used and not suspended_gravity_replay:
        contact_row = initial_row.clone()
        contact_row[:, 0:5] = initial_row[:, 0:5] + descent_fraction_by_side[
            "left"
        ] * (deep_contact_row[:, 0:5] - initial_row[:, 0:5])
        contact_row[:, 6:11] = initial_row[:, 6:11] + descent_fraction_by_side[
            "right"
        ] * (deep_contact_row[:, 6:11] - initial_row[:, 6:11])
    pinch_row = contact_row.clone()
    pinch_row[:, 5] = PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
    pinch_row[:, 11] = PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
    zero_velocity = torch.zeros_like(contact_row)
    scene.reset()
    write_scripted_arm_state_and_drive_targets(
        robot,
        initial_row,
        zero_velocity,
        joint_ids,
        initialize_arm_state=True,
    )
    scene.write_data_to_sim()
    sim.step()
    scene.update(physics_dt_s)

    stage = omni.usd.get_context().get_stage()
    free_node_mask = torch.ones(1024, dtype=torch.bool, device=sim.device)

    settled_run = 0
    settled_step = None
    maximum_steps = math.ceil(args.settle_timeout_s / physics_dt_s)
    for step in range(1, maximum_steps + 1):
        write_scripted_arm_state_and_drive_targets(
            robot, initial_row, zero_velocity, joint_ids
        )
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
            f"cloth did not settle before approach; final maximum speed={speed:.6f} m/s"
        )
    settled_nodes_w = cloth.data.nodal_pos_w.torch.clone()
    nodes_after_vertical_approach_w = settled_nodes_w

    if vertical_grasp_used:
        approach_steps = max(2, round(1.0 / physics_dt_s))
        for step in range(1, approach_steps + 1):
            alpha = step / approach_steps
            target = initial_row + alpha * (contact_row - initial_row)
            write_scripted_arm_state_and_drive_targets(
                robot, target, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError("cloth produced non-finite nodes during vertical approach")
        nodes_after_vertical_approach_w = cloth.data.nodal_pos_w.torch.clone()
    maximum_vertical_approach_cloth_displacement_m = float(
        torch.max(
            torch.linalg.vector_norm(
                nodes_after_vertical_approach_w - settled_nodes_w, dim=-1
            )
        ).item()
    )

    pinch_steps = max(2, round(PINCH_CLOSE_DURATION_S / physics_dt_s))
    for step in range(1, pinch_steps + 1):
        alpha = step / pinch_steps
        target = contact_row + alpha * (pinch_row - contact_row)
        write_scripted_arm_state_and_drive_targets(
            robot, target, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
    pinch_hold_steps = max(2, round(PINCH_HOLD_DURATION_S / physics_dt_s))
    for _ in range(pinch_hold_steps):
        write_scripted_arm_state_and_drive_targets(
            robot, pinch_row, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)

    gripper_joint_ids = [joint_ids[index] for index in GRIPPER_JOINT_INDICES]
    target_gripper_model_rad = pinch_row[:, GRIPPER_JOINT_INDICES]
    extra_pinch_hold_step = 0
    maximum_extra_pinch_hold_steps = max(
        1, math.ceil(PINCH_TARGET_SETTLE_TIMEOUT_S / physics_dt_s)
    )
    while extra_pinch_hold_step < maximum_extra_pinch_hold_steps:
        achieved_grippers = robot.data.joint_pos.torch[:, gripper_joint_ids]
        residual = float(
            torch.max(
                torch.abs(achieved_grippers - target_gripper_model_rad)
            ).item()
        )
        if residual <= MAXIMUM_GRIPPER_CLOSING_RESIDUAL_RAD:
            break
        extra_pinch_hold_step += 1
        write_scripted_arm_state_and_drive_targets(
            robot, pinch_row, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
        if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
            raise RuntimeError(
                "cloth produced non-finite nodes while settling jaw closure"
            )
    achieved_gripper_model_rad = robot.data.joint_pos.torch[
        :, gripper_joint_ids
    ].clone()
    achieved_all_joint_positions = robot.data.joint_pos.torch[:, joint_ids].clone()
    arm_target_residual = achieved_all_joint_positions - pinch_row
    print(
        "S1_CONTACT_JOINT_TRACKING "
        + json.dumps(
            {
                "achieved_env_0_rad": achieved_all_joint_positions[0].tolist(),
                "target_env_0_rad": pinch_row[0].tolist(),
                "maximum_arm_residual_rad": float(
                    torch.max(
                        torch.abs(
                            arm_target_residual[
                                :, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10]
                            ]
                        )
                    ).item()
                ),
                "extra_pinch_hold_s": extra_pinch_hold_step * physics_dt_s,
            },
            sort_keys=True,
        ),
        flush=True,
    )
    require_arm_target_reached(robot, pinch_row, joint_ids, "first_contact")
    closing_contact_residual_rad = (
        achieved_gripper_model_rad - target_gripper_model_rad
    )
    newton_contact_snapshot = newton_soft_contact_snapshot()
    actual_bilateral_particles_by_side: dict[str, list[int]] = {}
    if newton_contact_snapshot is not None:
        print(
            "S1_NEWTON_ACTUAL_CONTACTS "
            + json.dumps(newton_contact_snapshot, sort_keys=True),
            flush=True,
        )
        bilateral = newton_contact_snapshot.get(
            "bilateral_same_particle_contacts", {}
        )
        if any(not bilateral.get(side) for side in ("left", "right")):
            raise RuntimeError(
                "Newton actual-contact gate failed: each gripper must contact "
                "at least one identical towel particle on both jaw faces"
            )
        actual_bilateral_particles_by_side = {
            side: [int(index) for index in bilateral[side]]
            for side in ("left", "right")
        }
    if contact_gated_retention_used:
        if args.physics_backend != "newton-coupled-vbd":
            raise RuntimeError(
                "contact-gated retention requires Newton's inspectable actual-contact buffer"
            )
        if environment_count != 1:
            raise RuntimeError(
                "strict actual-contact particle indexing is currently validated only "
                "with --environment-count 1"
            )
    if args.contact_pose_diagnostic:
        print(
            "S1_CONTACT_POSE_DIAGNOSTIC_KEEP_OPEN "
            f"maximum_closing_residual_rad="
            f"{float(torch.max(torch.abs(closing_contact_residual_rad)).item()):.6f} "
            "close the Isaac Sim window when done",
            flush=True,
        )
        while simulation_app.is_running():
            write_scripted_arm_state_and_drive_targets(
                robot, pinch_row, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
        return 0
    if (
        args.physics_backend == "physx"
        and vertical_grasp_used
        and float(
        torch.max(torch.abs(closing_contact_residual_rad)).item()
        ) > MAXIMUM_GRIPPER_CLOSING_RESIDUAL_RAD
    ):
        raise RuntimeError(
            "one or more grippers failed the closed-jaw contact gate: "
            f"maximum residual="
            f"{float(torch.max(torch.abs(closing_contact_residual_rad)).item()):.6f} rad"
        )

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
    jaw_body_ids, jaw_body_names = robot.find_bodies(
        [
            "left_gripper_link",
            "left_moving_jaw_link",
            "right_gripper_link",
            "right_moving_jaw_link",
        ],
        preserve_order=True,
    )
    if jaw_body_names != [
        "left_gripper_link",
        "left_moving_jaw_link",
        "right_gripper_link",
        "right_moving_jaw_link",
    ]:
        raise RuntimeError(f"unexpected jaw body map: {jaw_body_names}")
    jaw_pad_centers_before_w = body_local_points_to_world(
        robot.data.body_pos_w.torch[:, jaw_body_ids],
        robot.data.body_quat_w.torch[:, jaw_body_ids],
        (
            FIXED_JAW_PAD_CENTER_PARENT_M,
            MOVING_JAW_PAD_CENTER_PARENT_M,
            FIXED_JAW_PAD_CENTER_PARENT_M,
            MOVING_JAW_PAD_CENTER_PARENT_M,
        ),
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
    gripper_approach_before_w = gripper_approach_axes_w(
        gripper_orientation_before_xyzw
    )
    frictional_vertical_stop_by_side_m: dict[str, float] = {}
    frictional_xy_contact_error_by_side_m: dict[str, float] = {}
    for side_index, side in enumerate(("left", "right")):
        if legacy_attachment_used:
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
        else:
            contact_pose_tolerance = 0.003
            frictional_contact_target_z_m = (
                FRICTIONAL_CONTACT_TCP_TARGET_Z_M
                + (1.0 - descent_fraction_by_side[side])
                * FRICTIONAL_PREGRASP_CLEARANCE_M
            )
            expected_position = torch.tensor(
                (
                    *FRICTIONAL_CONTACT_TCP_TARGET_XY_M[side],
                    frictional_contact_target_z_m,
                ),
                dtype=torch.float32,
                device=sim.device,
            )
            measured_local_position = (
                gripper_tcp_before_w[:, side_index] - scene.env_origins
            )
        position_error = float(
            torch.max(
                torch.abs(
                    measured_local_position[
                        ..., :2 if vertical_grasp_used else 3
                    ]
                    - expected_position[:2 if vertical_grasp_used else 3]
                )
            ).item()
        )
        measured_orientation = gripper_orientation_before_xyzw[:, side_index]
        if legacy_attachment_used:
            expected_orientation = torch.tensor(
                expected["orientation_xyzw"], dtype=torch.float32, device=sim.device
            )
            orientation_error = float(
                torch.max(
                    torch.minimum(
                        torch.linalg.vector_norm(
                            measured_orientation - expected_orientation, dim=-1
                        ),
                        torch.linalg.vector_norm(
                            measured_orientation + expected_orientation, dim=-1
                        ),
                    )
                ).item()
            )
        else:
            frictional_xy_contact_error_by_side_m[side] = position_error
            vertical_stop_m = float(
                torch.max(
                    measured_local_position[:, 2] - expected_position[2]
                ).item()
            )
            frictional_vertical_stop_by_side_m[side] = vertical_stop_m
            if (
                not suspended_gravity_replay
                and not -0.001 <= vertical_stop_m <= 0.006
            ):
                raise RuntimeError(
                    f"{side} vertical contact stop {vertical_stop_m:.6f} m is "
                    "outside the table-contact allowance"
                )
            approach_tilt_rad = torch.acos(
                torch.clamp(-gripper_approach_before_w[:, side_index, 2], -1.0, 1.0)
            )
            maximum_approach_tilt_rad = float(torch.max(approach_tilt_rad).item())
            if maximum_approach_tilt_rad > MAXIMUM_FRICTIONAL_APPROACH_TILT_RAD:
                raise RuntimeError(
                    f"{side} frictional contact approach tilt "
                    f"{math.degrees(maximum_approach_tilt_rad):.3f} deg is not vertical"
                )
            orientation_error = 0.0
        if legacy_attachment_used and position_error > contact_pose_tolerance:
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
    pad_to_node_distances_m = torch.cdist(
        jaw_pad_centers_before_w, nodes_before_w
    )
    nearest_node_distance_by_pad_m, nearest_node_index_by_pad = torch.min(
        pad_to_node_distances_m, dim=2
    )
    jaw_pad_diagnostic = {
        "frictional_descent_fraction_by_side": descent_fraction_by_side,
        "maximum_vertical_approach_cloth_displacement_m": (
            maximum_vertical_approach_cloth_displacement_m
        ),
        "achieved_gripper_model_rad_env_0": [
            float(value) for value in achieved_gripper_model_rad[0]
        ],
        "closing_contact_residual_rad_env_0": [
            float(value) for value in closing_contact_residual_rad[0]
        ],
        "vertical_contact_stop_by_side_m": frictional_vertical_stop_by_side_m,
        "xy_contact_error_by_side_m": frictional_xy_contact_error_by_side_m,
        "pad_centers_env_0_w_m": {
            name: [float(value) for value in jaw_pad_centers_before_w[0, index]]
            for index, name in enumerate(
                ("left_fixed", "left_moving", "right_fixed", "right_moving")
            )
        },
        "pad_center_gap_env_0_m": {
            "left": float(
                torch.linalg.vector_norm(
                    jaw_pad_centers_before_w[0, 0]
                    - jaw_pad_centers_before_w[0, 1]
                ).item()
            ),
            "right": float(
                torch.linalg.vector_norm(
                    jaw_pad_centers_before_w[0, 2]
                    - jaw_pad_centers_before_w[0, 3]
                ).item()
            ),
        },
        "nearest_cloth_node_distance_env_0_m": {
            name: float(nearest_node_distance_by_pad_m[0, index].item())
            for index, name in enumerate(
                ("left_fixed", "left_moving", "right_fixed", "right_moving")
            )
        },
        "nearest_cloth_node_env_0_w_m": {
            name: [
                float(value)
                for value in nodes_before_w[
                    0, int(nearest_node_index_by_pad[0, index].item())
                ]
            ]
            for index, name in enumerate(
                ("left_fixed", "left_moving", "right_fixed", "right_moving")
            )
        },
        "cloth_height_env_0_m": {
            "minimum": float(torch.min(nodes_before_w[0, :, 2]).item()),
            "maximum": float(torch.max(nodes_before_w[0, :, 2]).item()),
        },
        "cloth_neighborhood_gate": (
            "actual_bilateral_same_particle_contact"
            if actual_bilateral_particles_by_side
            else "pad_center_to_nearest_node_proxy"
        ),
    }
    print(
        "S1_FRICTIONAL_JAW_GEOMETRY "
        + json.dumps(jaw_pad_diagnostic, sort_keys=True),
        flush=True,
    )
    if vertical_grasp_used:
        if maximum_vertical_approach_cloth_displacement_m > (
            MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M
        ):
            raise RuntimeError(
                "vertical approach displaced cloth before jaw close by "
                f"{maximum_vertical_approach_cloth_displacement_m:.6f} m"
            )
        if (
            not actual_bilateral_particles_by_side
            and float(torch.max(nearest_node_distance_by_pad_m).item())
            > MAXIMUM_PAD_CENTER_TO_CLOTH_NODE_DISTANCE_M
        ):
            raise RuntimeError(
                "vertical jaw pads did not enter the cloth neighborhood: "
                f"maximum center-to-node distance="
                f"{float(torch.max(nearest_node_distance_by_pad_m).item()):.6f} m"
            )
    maximum_pinch_induced_cloth_displacement_m = float(
        torch.max(
            torch.linalg.vector_norm(
                nodes_before_w - nodes_after_vertical_approach_w, dim=-1
            )
        ).item()
    )
    if (
        args.physics_backend == "physx"
        and
        maximum_pinch_induced_cloth_displacement_m
        > MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M
    ):
        raise RuntimeError(
            "jaw closing displaced cloth before grasp by "
            f"{maximum_pinch_induced_cloth_displacement_m:.6f} m"
        )
    selected_indices = []
    jaw_target_patch_center_xy_distances_m = []
    attachment_point_tcp_distances_m = []
    for environment_index in range(environment_count):
        environment_indices = []
        for side_index, side in enumerate(("left", "right")):
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
            if contact_gated_retention_used:
                indices = actual_bilateral_particles_by_side[side]
                if any(index < 0 or index >= nodes_before_w.shape[1] for index in indices):
                    raise RuntimeError(
                        f"{side} actual-contact particle index is outside the cloth: "
                        f"{indices}"
                    )
            else:
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
    if contact_gated_retention_used:
        print(
            "S1_CONTACT_GATED_RETENTION_SOURCE "
            + json.dumps(
                {
                    "activation": "same_towel_particle_on_fixed_and_moving_jaw",
                    "fallback": None,
                    "selected_actual_contact_particles": {
                        side: selected_indices[0][side_index]
                        for side_index, side in enumerate(("left", "right"))
                    },
                },
                sort_keys=True,
            ),
            flush=True,
        )
    minimum_selected_points = min(
        len(indices) for environment_indices in selected_indices for indices in environment_indices
    )
    if minimum_selected_points < MINIMUM_PATCH_POINT_COUNT:
        raise RuntimeError(
            f"vertex patch mask selected only {minimum_selected_points} nodes"
        )
    contact_gated_local_positions: dict[str, list[Gf.Vec3d]] = {}
    contact_gated_retention_active = False
    contact_gated_kinematic_targets = torch.empty(0, device=sim.device)

    def activate_newton_contact_gated_retention() -> None:
        nonlocal contact_gated_retention_active, contact_gated_kinematic_targets
        if not newton_state_retention_used:
            return
        state = NewtonManager.get_state()
        state_positions = state.particle_q.numpy()
        maximum_state_position_error_m = 0.0
        for side_index, side in enumerate(("left", "right")):
            indices = selected_indices[0][side_index]
            position = Gf.Vec3d(*gripper_before_w[0, side_index].tolist())
            orientation = gripper_orientation_before_xyzw[0, side_index].tolist()
            inverse_rotation = Gf.Rotation(
                Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
            ).GetInverse()
            contact_gated_local_positions[side] = [
                inverse_rotation.TransformDir(
                    Gf.Vec3d(*nodes_before_w[0, index].tolist()) - position
                )
                for index in indices
            ]
            for index in indices:
                maximum_state_position_error_m = max(
                    maximum_state_position_error_m,
                    float(
                        torch.linalg.vector_norm(
                            torch.tensor(
                                state_positions[index],
                                dtype=nodes_before_w.dtype,
                                device=nodes_before_w.device,
                            )
                            - nodes_before_w[0, index]
                        ).item()
                    ),
                )
        if maximum_state_position_error_m > 1.0e-4:
            raise RuntimeError(
                "Newton particle state and Isaac cloth observation disagree at capture: "
                f"{maximum_state_position_error_m:.6f} m"
            )
        contact_gated_kinematic_targets = torch.empty(
            (environment_count, nodes_before_w.shape[1], 4),
            dtype=nodes_before_w.dtype,
            device=nodes_before_w.device,
        )
        contact_gated_kinematic_targets[..., :3] = nodes_before_w
        contact_gated_kinematic_targets[..., 3] = 1.0
        contact_gated_retention_active = True
        print(
            "S1_NEWTON_CONTACT_GATED_RETENTION_ACTIVATED "
            + json.dumps(
                {
                    "actual_contact_particles": actual_bilateral_particles_by_side,
                    "fallback": None,
                    "frame": "fixed_gripper_link_while_both_jaws_closed",
                    "mechanism": "isaaclab_newton_nodal_kinematic_target",
                    "maximum_state_observation_error_m": (
                        maximum_state_position_error_m
                    ),
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def enforce_newton_contact_gated_retention() -> None:
        if not contact_gated_retention_active:
            return
        for side_index, side in enumerate(("left", "right")):
            body_position = Gf.Vec3d(
                *robot.data.body_pos_w.torch[0, gripper_body_ids[side_index]].tolist()
            )
            orientation = robot.data.body_quat_w.torch[
                0, gripper_body_ids[side_index]
            ].tolist()
            rotation = Gf.Rotation(
                Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
            )
            indices = selected_indices[0][side_index]
            for index, local_position in zip(
                indices, contact_gated_local_positions[side], strict=True
            ):
                target = body_position + rotation.TransformDir(local_position)
                contact_gated_kinematic_targets[0, index, :3] = torch.tensor(
                    [target[axis] for axis in range(3)],
                    dtype=contact_gated_kinematic_targets.dtype,
                    device=contact_gated_kinematic_targets.device,
                )
                contact_gated_kinematic_targets[0, index, 3] = 0.0
        cloth.write_nodal_kinematic_target_to_sim_index(
            contact_gated_kinematic_targets
        )

    def deactivate_newton_contact_gated_retention(reason: str) -> None:
        nonlocal contact_gated_retention_active
        if not contact_gated_retention_active:
            return
        contact_gated_kinematic_targets[..., 3] = 1.0
        cloth.write_nodal_kinematic_target_to_sim_index(
            contact_gated_kinematic_targets
        )
        contact_gated_retention_active = False
        print(
            "S1_NEWTON_CONTACT_GATED_RETENTION_RELEASED " f"reason={reason}",
            flush=True,
        )

    activate_newton_contact_gated_retention()
    enforce_newton_contact_gated_retention()
    attachment_records: list[dict[str, object]] = []
    staged_self_collision_paths: list[str] = []
    if scripted_attachment_used or args.self_contact:
        sim.pause()
        if scripted_attachment_used:
            author_runtime_attachments(
                environment_count,
                nodes_before_w,
                gripper_before_w,
                gripper_orientation_before_xyzw,
                selected_indices,
            )
        if args.self_contact:
            staged_self_collision_paths = (
                enable_cloth_self_collision_after_pinch(environment_count)
            )
        simulation_app.update()
        sim.play()
    if scripted_attachment_used:
        attachment_records, authored_selected_indices = create_attachments(
            environment_count
        )
        if authored_selected_indices != selected_indices:
            raise RuntimeError("runtime attachment indices changed while authoring")
    if args.self_contact:
        print(
            (
                "S1_NEWTON_SELF_CONTACT_CONFIGURED_AT_SOLVER_INIT "
                if args.physics_backend == "newton-coupled-vbd"
                else "S1_SELF_COLLISION_ENABLED_AFTER_PINCH "
            )
            + json.dumps(staged_self_collision_paths),
            flush=True,
        )
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
    minimum_authored_points = (
        min(record["point_count"] for record in attachment_records)
        if attachment_records
        else 0
    )
    print(
        (
            f"S1_VERTEX_PATCH_ATTACHED attachments={len(attachment_records)} "
            if scripted_attachment_used
            else "S1_FRICTIONAL_JAW_CLOSED attachments=0 "
        )
        + f"minimum_selected_points={minimum_selected_points} "
        + f"minimum_authored_points={minimum_authored_points}",
        flush=True,
    )

    for _ in range(2):
        write_scripted_arm_state_and_drive_targets(
            robot, pinch_row, zero_velocity, joint_ids
        )
        enforce_newton_contact_gated_retention()
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
    nodes_after_attachment = (
        authoritative_newton_nodes_w(environment_count, sim.device)
        - scene.env_origins[:, None, :]
        if newton_state_retention_used
        else local_nodes(scene, cloth).clone()
    )
    maximum_attachment_snap_m = float(
        torch.max(
            torch.linalg.vector_norm(nodes_after_attachment - nodes_before, dim=-1)
        ).item()
    )
    if scripted_attachment_used and maximum_attachment_snap_m > MAXIMUM_ATTACHMENT_SNAP_M:
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
    lift_row = phase_model_tensor(
        source,
        lift,
        gripper_project_positions_rad=(
            PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
        ),
        environment_count=environment_count,
        device=sim.device,
    )
    if args.physics_backend == "newton-coupled-vbd" and not suspended_gravity_replay:
        # Isolate grip retention from the asymmetric first-fold trajectory.
        # One third of the validated vertical contact-to-pregrasp IK segment
        # raises both TCPs by approximately 10 mm without changing approach
        # orientation or opening the jaws.
        vertical_lift_fraction = 1.0 / 3.0
        lift_row = pinch_row.clone()
        lift_row[:, 0:5] = contact_row[:, 0:5] + vertical_lift_fraction * (
            initial_row[:, 0:5] - contact_row[:, 0:5]
        )
        lift_row[:, 6:11] = contact_row[:, 6:11] + vertical_lift_fraction * (
            initial_row[:, 6:11] - contact_row[:, 6:11]
        )
    lift_steps = max(2, round(args.lift_seconds / physics_dt_s))
    for step in range(1, lift_steps + 1):
        alpha = step / lift_steps
        target = pinch_row + alpha * (lift_row - pinch_row)
        write_scripted_arm_state_and_drive_targets(
            robot, target, zero_velocity, joint_ids
        )
        enforce_newton_contact_gated_retention()
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
        if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
            raise RuntimeError("cloth produced non-finite nodes during lift")
    settle_physical_arm_drives(
        robot,
        lift_row,
        zero_velocity,
        joint_ids,
        scene,
        sim,
        cloth,
        physics_dt_s,
        args.arm_target_settle_timeout_s,
        "first_fold_01",
        post_step_callback=(
            enforce_newton_contact_gated_retention
            if newton_state_retention_used
            else None
        ),
    )
    enforce_newton_contact_gated_retention()

    if newton_state_retention_used:
        nodes_after_w = authoritative_newton_nodes_w(environment_count, sim.device)
        nodes_after = nodes_after_w - scene.env_origins[:, None, :]
    else:
        nodes_after = local_nodes(scene, cloth)
        nodes_after_w = cloth.data.nodal_pos_w.torch
    gripper_after_w = robot.data.body_pos_w.torch[:, gripper_body_ids]
    gripper_orientation_after_xyzw = robot.data.body_quat_w.torch[
        :, gripper_body_ids
    ]
    node_lift = nodes_after[..., 2] - nodes_after_attachment[..., 2]
    maximum_node_lift_by_environment = torch.max(node_lift, dim=1).values
    minimum_maximum_node_lift_m = float(torch.min(maximum_node_lift_by_environment).item())
    if args.physics_backend == "newton-coupled-vbd":
        print(
            "S1_NEWTON_POST_LIFT_CONTACTS "
            + json.dumps(
                {
                    "contacts": newton_soft_contact_snapshot(),
                    "maximum_node_lift_by_environment_m": (
                        maximum_node_lift_by_environment.tolist()
                    ),
                    "contact_stiffness": args.newton_contact_stiffness,
                },
                sort_keys=True,
            ),
            flush=True,
        )
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
    if (
        contact_gated_retention_used
        and minimum_selected_patch_lift_m < MINIMUM_CONTACT_GATED_LIFT_M
    ):
        raise RuntimeError(
            "actual-contact-gated patch did not follow the nominal 10 mm lift: "
            f"{minimum_selected_patch_lift_m:.6f} m is below "
            f"{MINIMUM_CONTACT_GATED_LIFT_M:.6f} m"
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

    grasp_release_probe_result = None
    place_release_result = None
    post_release_correction_result = None
    keep_open_row = lift_row
    result_status = PASS_STATUS
    if args.grasp_release_probe:
        retention_hold_steps = max(
            1, round(GRASP_RETENTION_HOLD_S / physics_dt_s)
        )
        nodes_before_retention_hold = local_nodes(scene, cloth).clone()
        for _ in range(retention_hold_steps):
            write_scripted_arm_state_and_drive_targets(
                robot, lift_row, zero_velocity, joint_ids
            )
            enforce_newton_contact_gated_retention()
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError("cloth produced non-finite nodes during retention hold")
        nodes_after_retention_hold = local_nodes(scene, cloth).clone()
        maximum_retention_slip_m = 0.0
        for side_index in range(2):
            indices = selected_indices[0][side_index]
            maximum_retention_slip_m = max(
                maximum_retention_slip_m,
                float(
                    torch.max(
                        torch.linalg.vector_norm(
                            nodes_after_retention_hold[0, indices]
                            - nodes_before_retention_hold[0, indices],
                            dim=-1,
                        )
                    ).item()
                ),
            )
        if maximum_retention_slip_m > MAXIMUM_PATCH_FOLLOW_ERROR_M:
            raise RuntimeError(
                "actual-contact-gated cloth slipped during the 1 s closed hold: "
                f"{maximum_retention_slip_m:.6f} m"
            )

        deactivate_newton_contact_gated_retention("q0_opening_started")
        if contact_gated_retention_active:
            raise RuntimeError("contact-gated retention remained active before Q0 opening")

        q0_open_row = lift_row.clone()
        q0_open_row[:, GRIPPER_JOINT_INDICES] = RELEASE_MODEL_GRIPPER_JOINT_POSITION_RAD
        jaw_open_steps = max(2, round(JAW_OPEN_DURATION_S / physics_dt_s))
        for step in range(1, jaw_open_steps + 1):
            alpha = step / jaw_open_steps
            target = lift_row + alpha * (q0_open_row - lift_row)
            write_scripted_arm_state_and_drive_targets(
                robot, target, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
        release_hold_steps = max(1, round(GRASP_RELEASE_HOLD_S / physics_dt_s))
        for _ in range(release_hold_steps):
            write_scripted_arm_state_and_drive_targets(
                robot, q0_open_row, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError("cloth produced non-finite nodes during Q0 release hold")

        achieved_q0 = robot.data.joint_pos.torch[:, gripper_joint_ids]
        maximum_q0_residual_rad = float(
            torch.max(
                torch.abs(
                    achieved_q0
                    - q0_open_row[:, GRIPPER_JOINT_INDICES]
                )
            ).item()
        )
        if maximum_q0_residual_rad > MAXIMUM_GRIPPER_OPENING_RESIDUAL_RAD:
            raise RuntimeError(
                "grippers did not reach the measured Q0 release opening: "
                f"residual={maximum_q0_residual_rad:.6f} rad"
            )
        nodes_after_q0_release = local_nodes(scene, cloth).clone()
        release_drop_by_side_m: dict[str, float] = {}
        for side_index, side in enumerate(("left", "right")):
            indices = selected_indices[0][side_index]
            release_drop_by_side_m[side] = float(
                torch.mean(
                    nodes_after_retention_hold[0, indices, 2]
                    - nodes_after_q0_release[0, indices, 2]
                ).item()
            )
        if min(release_drop_by_side_m.values()) < MINIMUM_GRASP_RELEASE_DROP_M:
            raise RuntimeError(
                "cloth did not drop from both Q0-open grippers: "
                f"{release_drop_by_side_m}"
            )
        post_open_contacts = newton_soft_contact_snapshot()
        post_open_bilateral = (
            post_open_contacts.get("bilateral_same_particle_contacts", {})
            if post_open_contacts is not None
            else {}
        )
        grasp_release_probe_result = {
            "activation": "actual_same_particle_bilateral_jaw_contact_only",
            "proximity_fallback_used": False,
            "retention_hold_s": GRASP_RETENTION_HOLD_S,
            "maximum_retention_slip_m": maximum_retention_slip_m,
            "q0_project_command_rad": 0.0,
            "q0_model_target_rad": RELEASE_MODEL_GRIPPER_JOINT_POSITION_RAD,
            "q0_measured_gap_mm": {
                "left": gripper_candidate.q0_gap_mm["left"],
                "right": gripper_candidate.q0_gap_mm["right"],
            },
            "maximum_q0_residual_rad": maximum_q0_residual_rad,
            "release_hold_s": GRASP_RELEASE_HOLD_S,
            "release_drop_by_side_m": release_drop_by_side_m,
            "post_open_contacts": post_open_contacts,
            "post_open_bilateral_contacts_diagnostic": post_open_bilateral,
            "post_open_zero_contact_required": False,
            "release_gate": (
                "constraint_disabled_and_q0_reached_and_both_patches_dropped"
            ),
            "retention_disabled_before_open": True,
        }
        result_status = CONTACT_GATED_RELEASE_PASS_STATUS
        keep_open_row = q0_open_row
        print(
            "S1_ACTUAL_CONTACT_GATED_Q0_RELEASE "
            + json.dumps(grasp_release_probe_result, sort_keys=True),
            flush=True,
        )

    if args.place_release:
        current_row = lift_row
        free_edge_touchdown_x_m = None
        previous_free_edge_x_m = None
        touchdown_arm_x_m = None
        previous_arm_x_m = None
        low_sweep_reference_free_edge_x_m = None
        maximum_low_sweep_incremental_slip_ratio = 0.0
        minimum_self_contact_separation_during_fold_m = math.inf
        fold_steps = max(2, round(args.fold_phase_seconds / physics_dt_s))
        if suspended_gravity_replay:
            fold_records = source["canonical_replay"]["first_fold"]
            lift_index = next(
                index
                for index, record in enumerate(fold_records)
                if record["name"] == "first_suspend_lift_01"
            )
            release_index = next(
                index
                for index, record in enumerate(fold_records)
                if record.get("attachment_event")
                == "release_both_edge_patches_after_gravity_laydown_gate"
            )
            attached_motion_phases = fold_records[lift_index + 1 : release_index + 1]
        else:
            attached_motion_phases = [
                phase(source, f"first_fold_{sample_index:02d}")
                for sample_index in range(2, 17)
            ]
        for target_phase in attached_motion_phases:
            target_row = phase_model_tensor(
                source,
                target_phase,
                gripper_project_positions_rad=(
                    PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
                ),
                environment_count=environment_count,
                device=sim.device,
            )
            for step in range(1, fold_steps + 1):
                alpha = step / fold_steps
                target = current_row + alpha * (target_row - current_row)
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                enforce_newton_contact_gated_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                    raise RuntimeError(
                        f"cloth produced non-finite nodes during {target_phase['name']}"
                    )
            current_row = target_row
            continuous_suspended_motion = (
                suspended_gravity_replay
                and (
                    target_phase["name"].startswith(
                        "first_suspended_forward_sweep_"
                    )
                    or target_phase["name"].startswith(
                        "first_suspended_forward_descent_"
                    )
                    or target_phase["name"] == "first_free_edge_touchdown"
                    or target_phase["name"].startswith(
                        "first_touchdown_slack_feed_"
                    )
                )
            )
            if continuous_suspended_motion:
                print(
                    "S1_CONTINUOUS_SUSPENDED_FORWARD_MOTION "
                    f"phase={target_phase['name']} settle_skipped=true",
                    flush=True,
                )
            elif suspended_gravity_replay:
                settle_physical_arm_drives(
                    robot,
                    target_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    target_phase["name"],
                    post_step_callback=enforce_newton_contact_gated_retention,
                )
            else:
                settle_physical_arm_drives(
                    robot,
                    target_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    target_phase["name"],
                )
            if (
                suspended_gravity_replay
                and target_phase["name"] == "first_suspend_lift_07"
                and scripted_pre_touchdown_hold_s > 0.0
            ):
                hold_duration_s = (
                    SUSPENDED_TOUCHDOWN_CALIBRATION_DURATION_S
                    if args.calibrate_scripted_touchdown
                    else scripted_pre_touchdown_hold_s
                )
                hold_steps = max(
                    1, round(hold_duration_s / physics_dt_s)
                )
                sample_steps = max(
                    1,
                    round(
                        (1.0 / SUSPENDED_TOUCHDOWN_CALIBRATION_VIDEO_FPS)
                        / physics_dt_s
                    ),
                )
                suspended_trajectory = []
                held_x_m = float(target_phase["targets"][0]["xyz_m"][0])
                for hold_step in range(1, hold_steps + 1):
                    write_scripted_arm_state_and_drive_targets(
                        robot, current_row, zero_velocity, joint_ids
                    )
                    enforce_newton_contact_gated_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if (
                        args.calibrate_scripted_touchdown
                        and hold_step % sample_steps == 0
                    ):
                        suspended_grid = local_nodes(scene, cloth).reshape(
                            environment_count,
                            CLOTH_RESOLUTION[1] + 1,
                            CLOTH_RESOLUTION[0] + 1,
                            3,
                        )
                        free_edge = suspended_grid[:, :, CLOTH_RESOLUTION[0], :]
                        free_edge_x_m = torch.median(free_edge[:, :, 0], dim=1).values
                        free_edge_z_m = torch.median(free_edge[:, :, 2], dim=1).values
                        suspended_trajectory.append(
                            {
                                "time_s": hold_step * physics_dt_s,
                                "per_environment_median_x_m": free_edge_x_m.tolist(),
                                "per_environment_median_z_m": free_edge_z_m.tolist(),
                                "maximum_horizontal_offset_m": float(
                                    torch.max(torch.abs(free_edge_x_m - held_x_m)).item()
                                ),
                            }
                        )
                if args.calibrate_scripted_touchdown:
                    sample_period_s = sample_steps * physics_dt_s
                    descent_sample_count = max(
                        1, round(args.fold_phase_seconds / sample_period_s)
                    )
                    candidates = []
                    for start_index in range(
                        0,
                        len(suspended_trajectory) - descent_sample_count,
                    ):
                        landing_index = start_index + descent_sample_count
                        landing_x_m = float(
                            suspended_trajectory[landing_index][
                                "per_environment_median_x_m"
                            ][0]
                        )
                        if 0 < landing_index < len(suspended_trajectory) - 1:
                            previous_x_m = float(
                                suspended_trajectory[landing_index - 1][
                                    "per_environment_median_x_m"
                                ][0]
                            )
                            next_x_m = float(
                                suspended_trajectory[landing_index + 1][
                                    "per_environment_median_x_m"
                                ][0]
                            )
                            landing_speed_m_s = abs(next_x_m - previous_x_m) / (
                                2.0 * sample_period_s
                            )
                        else:
                            landing_speed_m_s = math.inf
                        landing_offset_m = abs(landing_x_m - held_x_m)
                        candidates.append(
                            {
                                "start_hold_s": suspended_trajectory[start_index][
                                    "time_s"
                                ],
                                "predicted_landing_time_s": suspended_trajectory[
                                    landing_index
                                ]["time_s"],
                                "predicted_landing_x_m": landing_x_m,
                                "predicted_landing_offset_m": landing_offset_m,
                                "predicted_landing_horizontal_speed_m_s": (
                                    landing_speed_m_s
                                ),
                                "score_m": landing_offset_m
                                + 0.05 * landing_speed_m_s,
                            }
                        )
                    candidates.sort(key=lambda item: item["score_m"])
                    calibration_result = {
                        "schema_version": 1,
                        "record_kind": "towel_scripted_touchdown_calibration",
                        "status": "S1_SCRIPTED_TOUCHDOWN_CALIBRATION_PASS",
                        "motion_authorized": False,
                        "automatic_execution_permitted": False,
                        "mid_action_camera_control_used": False,
                        "simulator_privileged_diagnostic_used_offline": True,
                        "held_x_m": held_x_m,
                        "descent_duration_s": args.fold_phase_seconds,
                        "sample_period_s": sample_period_s,
                        "recommended_candidate": candidates[0],
                        "top_candidates": candidates[:10],
                        "trajectory": suspended_trajectory,
                    }
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(
                        json.dumps(calibration_result, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                    print(
                        "S1_SCRIPTED_TOUCHDOWN_CALIBRATION "
                        + json.dumps(
                            calibration_result["recommended_candidate"],
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    return 0
                print(
                    "S1_SCRIPTED_PRE_TOUCHDOWN_HOLD "
                    + json.dumps(
                        {
                            "hold_s": hold_steps * physics_dt_s,
                            "mid_action_camera_control_used": False,
                            "reason": "fixed_towel_grasp_and_occluded_top_view",
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            if suspended_gravity_replay and (
                target_phase["name"] == "first_free_edge_touchdown"
                or target_phase["name"].startswith("first_touchdown_slack_feed_")
                or target_phase["name"].startswith("first_form_l_")
            ):
                phase_grid = local_nodes(scene, cloth).reshape(
                    environment_count,
                    CLOTH_RESOLUTION[1] + 1,
                    CLOTH_RESOLUTION[0] + 1,
                    3,
                )
                free_edge_x_m = torch.median(
                    phase_grid[:, :, CLOTH_RESOLUTION[0], 0], dim=1
                ).values
                free_edge_z_m = torch.median(
                    phase_grid[:, :, CLOTH_RESOLUTION[0], 2], dim=1
                ).values
                if target_phase["name"] == "first_free_edge_touchdown":
                    free_edge_touchdown_x_m = free_edge_x_m.clone()
                    previous_free_edge_x_m = free_edge_x_m.clone()
                    touchdown_arm_x_m = float(target_phase["targets"][0]["xyz_m"][0])
                    previous_arm_x_m = touchdown_arm_x_m
                    print(
                        "S1_FREE_EDGE_TOUCHDOWN_ANCHOR "
                        + json.dumps(
                            {"per_environment_median_x_m": free_edge_x_m.tolist()},
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    touchdown_horizontal_offset_m = torch.abs(
                        free_edge_x_m - touchdown_arm_x_m
                    )
                    maximum_touchdown_horizontal_offset_m = float(
                        torch.max(touchdown_horizontal_offset_m).item()
                    )
                    touchdown_verticality_gate = {
                        "maximum_horizontal_offset_m": (
                            maximum_touchdown_horizontal_offset_m
                        ),
                        "maximum_horizontal_offset_limit_m": (
                            MAXIMUM_TOUCHDOWN_FREE_EDGE_HORIZONTAL_OFFSET_M
                        ),
                        "per_environment_horizontal_offset_m": (
                            touchdown_horizontal_offset_m.tolist()
                        ),
                        "passed": (
                            maximum_touchdown_horizontal_offset_m
                            <= MAXIMUM_TOUCHDOWN_FREE_EDGE_HORIZONTAL_OFFSET_M
                        ),
                    }
                    print(
                        "S1_FREE_EDGE_TOUCHDOWN_VERTICALITY_GATE "
                        + json.dumps(touchdown_verticality_gate, sort_keys=True),
                        flush=True,
                    )
                    if not touchdown_verticality_gate["passed"]:
                        raise RuntimeError(
                            "suspended towel is not vertical enough to start the "
                            "forward lay: horizontal free-edge offset="
                            f"{maximum_touchdown_horizontal_offset_m:.6f} m > "
                            f"{MAXIMUM_TOUCHDOWN_FREE_EDGE_HORIZONTAL_OFFSET_M:.6f} m"
                        )
                else:
                    if (
                        free_edge_touchdown_x_m is None
                        or previous_free_edge_x_m is None
                        or touchdown_arm_x_m is None
                        or previous_arm_x_m is None
                    ):
                        raise RuntimeError("free-edge touchdown anchor was not sampled")
                    arm_x_m = float(target_phase["targets"][0]["xyz_m"][0])
                    free_edge_anchor_drift_m = torch.abs(
                        free_edge_x_m - free_edge_touchdown_x_m
                    )
                    incremental_free_edge_slip_m = torch.abs(
                        free_edge_x_m - previous_free_edge_x_m
                    )
                    cumulative_arm_advance_m = arm_x_m - touchdown_arm_x_m
                    incremental_arm_advance_m = arm_x_m - previous_arm_x_m
                    cumulative_slip_ratio = (
                        free_edge_anchor_drift_m / cumulative_arm_advance_m
                        if cumulative_arm_advance_m > 1.0e-9
                        else torch.zeros_like(free_edge_anchor_drift_m)
                    )
                    incremental_slip_ratio = (
                        incremental_free_edge_slip_m / incremental_arm_advance_m
                        if incremental_arm_advance_m > 1.0e-9
                        else torch.zeros_like(incremental_free_edge_slip_m)
                    )
                    maximum_free_edge_anchor_drift_m = float(
                        torch.max(free_edge_anchor_drift_m).item()
                    )
                    print(
                        "S1_FORWARD_LAY_SLIP_OBSERVATION "
                        + json.dumps(
                            {
                                "phase": target_phase["name"],
                                "cumulative_arm_advance_m": cumulative_arm_advance_m,
                                "incremental_arm_advance_m": incremental_arm_advance_m,
                                "maximum_drift_m": maximum_free_edge_anchor_drift_m,
                                "per_environment_drift_m": (
                                    free_edge_anchor_drift_m.tolist()
                                ),
                                "per_environment_cumulative_slip_ratio": (
                                    cumulative_slip_ratio.tolist()
                                ),
                                "per_environment_incremental_slip_m": (
                                    incremental_free_edge_slip_m.tolist()
                                ),
                                "per_environment_incremental_slip_ratio": (
                                    incremental_slip_ratio.tolist()
                                ),
                                "per_environment_median_x_m": free_edge_x_m.tolist(),
                                "per_environment_median_z_m": free_edge_z_m.tolist(),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    previous_free_edge_x_m = free_edge_x_m.clone()
                    previous_arm_x_m = arm_x_m
                    if target_phase["name"] == "first_touchdown_slack_feed_04":
                        free_edge_table_clearance_m = (
                            free_edge_z_m - table_top_z_m_for_contact_gate
                        )
                        maximum_free_edge_table_clearance_m = float(
                            torch.max(free_edge_table_clearance_m).item()
                        )
                        free_edge_contact_gate = {
                            "maximum_table_clearance_m": (
                                maximum_free_edge_table_clearance_m
                            ),
                            "maximum_table_clearance_limit_m": (
                                MAXIMUM_ANCHORED_FREE_EDGE_TABLE_CLEARANCE_M
                            ),
                            "per_environment_table_clearance_m": (
                                free_edge_table_clearance_m.tolist()
                            ),
                            "passed": (
                                maximum_free_edge_table_clearance_m
                                <= MAXIMUM_ANCHORED_FREE_EDGE_TABLE_CLEARANCE_M
                            ),
                        }
                        print(
                            "S1_FREE_EDGE_TABLE_CONTACT_GATE "
                            + json.dumps(free_edge_contact_gate, sort_keys=True),
                            flush=True,
                        )
                        if (
                            not free_edge_contact_gate["passed"]
                            and not args.diagnostic_allow_airborne_free_edge
                        ):
                            raise RuntimeError(
                                "free edge is still airborne after the fixed "
                                "touchdown feed: clearance="
                                f"{maximum_free_edge_table_clearance_m:.6f} m"
                            )
                        if (
                            not free_edge_contact_gate["passed"]
                            and args.diagnostic_allow_airborne_free_edge
                        ):
                            print(
                                "S1_FREE_EDGE_TABLE_CONTACT_GATE_BYPASSED "
                                "diagnostic_only=true completion_claim_forbidden=true",
                                flush=True,
                            )
                    if target_phase["name"] == "first_form_l_06":
                        low_sweep_reference_free_edge_x_m = free_edge_x_m.clone()
                        maximum_low_sweep_incremental_slip_ratio = 0.0
                    elif target_phase["name"] in {
                        "first_form_l_07",
                        "first_form_l_08",
                        "first_form_l_09",
                    }:
                        if low_sweep_reference_free_edge_x_m is None:
                            raise RuntimeError(
                                "low-sweep free-edge reference was not sampled"
                            )
                        maximum_low_sweep_incremental_slip_ratio = max(
                            maximum_low_sweep_incremental_slip_ratio,
                            float(torch.max(incremental_slip_ratio).item()),
                        )
                        if target_phase["name"] == "first_form_l_09":
                            low_sweep_drift_m = torch.abs(
                                free_edge_x_m - low_sweep_reference_free_edge_x_m
                            )
                            maximum_low_sweep_drift_m = float(
                                torch.max(low_sweep_drift_m).item()
                            )
                            low_sweep_gate = {
                                "maximum_incremental_slip_ratio": (
                                    maximum_low_sweep_incremental_slip_ratio
                                ),
                                "maximum_incremental_slip_ratio_limit": (
                                    MAXIMUM_LOW_SWEEP_INCREMENTAL_SLIP_RATIO
                                ),
                                "maximum_low_sweep_drift_m": (
                                    maximum_low_sweep_drift_m
                                ),
                                "maximum_low_sweep_drift_limit_m": (
                                    MAXIMUM_LOW_SWEEP_FREE_EDGE_DRIFT_M
                                ),
                                "per_environment_low_sweep_drift_m": (
                                    low_sweep_drift_m.tolist()
                                ),
                                "incremental_slip_ratio_diagnostic_passed": (
                                    maximum_low_sweep_incremental_slip_ratio
                                    <= MAXIMUM_LOW_SWEEP_INCREMENTAL_SLIP_RATIO
                                ),
                                "passed": (
                                    maximum_low_sweep_drift_m
                                    <= MAXIMUM_LOW_SWEEP_FREE_EDGE_DRIFT_M
                                ),
                            }
                            print(
                                "S1_LOW_SWEEP_DRAG_GATE "
                                + json.dumps(low_sweep_gate, sort_keys=True),
                                flush=True,
                            )
                            if not low_sweep_gate["passed"]:
                                raise RuntimeError(
                                    "free edge dragged during the final low sweep: "
                                    f"drift={maximum_low_sweep_drift_m:.6f} m, "
                                    "maximum incremental slip ratio="
                                    f"{maximum_low_sweep_incremental_slip_ratio:.6f}"
                                )
            if (
                suspended_gravity_replay
                and target_phase["name"] == "first_touchdown_slack_feed_04"
                and (
                    args.calibrate_post_touchdown_anchor
                    or scripted_post_touchdown_hold_s > 0.0
                )
            ):
                anchor_hold_duration_s = (
                    POST_TOUCHDOWN_CALIBRATION_DURATION_S
                    if args.calibrate_post_touchdown_anchor
                    else scripted_post_touchdown_hold_s
                )
                anchor_hold_steps = max(
                    1, round(anchor_hold_duration_s / physics_dt_s)
                )
                anchor_sample_steps = max(
                    1,
                    round(
                        (1.0 / SUSPENDED_TOUCHDOWN_CALIBRATION_VIDEO_FPS)
                        / physics_dt_s
                    ),
                )
                post_touchdown_trajectory = []
                held_x_m = float(target_phase["targets"][0]["xyz_m"][0])
                for hold_step in range(1, anchor_hold_steps + 1):
                    write_scripted_arm_state_and_drive_targets(
                        robot, current_row, zero_velocity, joint_ids
                    )
                    enforce_newton_contact_gated_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if hold_step % anchor_sample_steps != 0:
                        continue
                    held_grid = local_nodes(scene, cloth).reshape(
                        environment_count,
                        CLOTH_RESOLUTION[1] + 1,
                        CLOTH_RESOLUTION[0] + 1,
                        3,
                    )
                    held_free_edge = held_grid[:, :, CLOTH_RESOLUTION[0], :]
                    held_free_edge_x_m = torch.median(
                        held_free_edge[:, :, 0], dim=1
                    ).values
                    held_free_edge_z_m = torch.median(
                        held_free_edge[:, :, 2], dim=1
                    ).values
                    post_touchdown_trajectory.append(
                        {
                            "time_s": hold_step * physics_dt_s,
                            "per_environment_median_x_m": (
                                held_free_edge_x_m.tolist()
                            ),
                            "per_environment_median_z_m": (
                                held_free_edge_z_m.tolist()
                            ),
                            "maximum_horizontal_offset_m": float(
                                torch.max(
                                    torch.abs(held_free_edge_x_m - held_x_m)
                                ).item()
                            ),
                        }
                    )
                if args.calibrate_post_touchdown_anchor:
                    sample_period_s = anchor_sample_steps * physics_dt_s
                    candidates = []
                    for sample_index in range(
                        1, len(post_touchdown_trajectory) - 1
                    ):
                        previous_x_m = float(
                            post_touchdown_trajectory[sample_index - 1][
                                "per_environment_median_x_m"
                            ][0]
                        )
                        sample_x_m = float(
                            post_touchdown_trajectory[sample_index][
                                "per_environment_median_x_m"
                            ][0]
                        )
                        next_x_m = float(
                            post_touchdown_trajectory[sample_index + 1][
                                "per_environment_median_x_m"
                            ][0]
                        )
                        horizontal_speed_m_s = abs(next_x_m - previous_x_m) / (
                            2.0 * sample_period_s
                        )
                        horizontal_offset_m = abs(sample_x_m - held_x_m)
                        candidates.append(
                            {
                                "post_touchdown_hold_s": (
                                    post_touchdown_trajectory[sample_index]["time_s"]
                                ),
                                "predicted_start_x_m": sample_x_m,
                                "predicted_start_offset_m": horizontal_offset_m,
                                "predicted_start_horizontal_speed_m_s": (
                                    horizontal_speed_m_s
                                ),
                                "score_m": horizontal_offset_m
                                + 0.05 * horizontal_speed_m_s,
                            }
                        )
                    candidates.sort(key=lambda item: item["score_m"])
                    calibration_result = {
                        "schema_version": 1,
                        "record_kind": "towel_post_touchdown_anchor_calibration",
                        "status": "S1_POST_TOUCHDOWN_ANCHOR_CALIBRATION_PASS",
                        "motion_authorized": False,
                        "automatic_execution_permitted": False,
                        "mid_action_camera_control_used": False,
                        "simulator_privileged_diagnostic_used_offline": True,
                        "held_x_m": held_x_m,
                        "sample_period_s": sample_period_s,
                        "recommended_candidate": candidates[0],
                        "top_candidates": candidates[:10],
                        "trajectory": post_touchdown_trajectory,
                    }
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(
                        json.dumps(calibration_result, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                    print(
                        "S1_POST_TOUCHDOWN_ANCHOR_CALIBRATION "
                        + json.dumps(
                            calibration_result["recommended_candidate"],
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    return 0
                final_anchor_grid = local_nodes(scene, cloth).reshape(
                    environment_count,
                    CLOTH_RESOLUTION[1] + 1,
                    CLOTH_RESOLUTION[0] + 1,
                    3,
                )
                previous_free_edge_x_m = torch.median(
                    final_anchor_grid[:, :, CLOTH_RESOLUTION[0], 0], dim=1
                ).values.clone()
                print(
                    "S1_SCRIPTED_POST_TOUCHDOWN_ANCHOR_HOLD "
                    + json.dumps(
                        {
                            "hold_s": anchor_hold_steps * physics_dt_s,
                            "mid_action_camera_control_used": False,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            if args.self_contact:
                minimum_self_contact_separation_during_fold_m = min(
                    minimum_self_contact_separation_during_fold_m,
                    minimum_nonlocal_node_separation_m(local_nodes(scene, cloth)),
                )
            if target_phase["name"] == "first_form_l_09":
                forward_lay_hold_steps = max(
                    1, round(FORWARD_LAY_CONTACT_HOLD_S / physics_dt_s)
                )
                for _ in range(forward_lay_hold_steps):
                    write_scripted_arm_state_and_drive_targets(
                        robot, current_row, zero_velocity, joint_ids
                    )
                    enforce_newton_contact_gated_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                        raise RuntimeError(
                            "cloth produced non-finite nodes during forward-lay "
                            "contact hold"
                        )
                print(
                    "S1_FORWARD_LAY_CONTACT_HELD "
                    f"hold_s={FORWARD_LAY_CONTACT_HOLD_S:.3f} "
                    "observation=none_open_loop_known_length",
                    flush=True,
                )

        correction_phase = next(
            (
                record
                for record in source["canonical_replay"]["first_fold"]
                if record["name"] == "first_fold_correction_01"
            ),
            None,
        )
        release_phase = phase(
            source,
            next(
                record["name"]
                for record in source["canonical_replay"]["first_fold"]
                if record.get("attachment_event")
                == "release_both_edge_patches_after_gravity_laydown_gate"
            )
            if suspended_gravity_replay
            else "first_fold_16",
        )
        if correction_phase is not None:
            correction_row = phase_model_tensor(
                source,
                correction_phase,
                gripper_project_positions_rad=(
                    PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
                ),
                environment_count=environment_count,
                device=sim.device,
            )
            for step in range(1, fold_steps + 1):
                alpha = step / fold_steps
                target = current_row + alpha * (correction_row - current_row)
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                    raise RuntimeError(
                        "cloth produced non-finite nodes during "
                        "first_fold_correction_01"
                    )
            current_row = correction_row
            settle_physical_arm_drives(
                robot,
                correction_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                "first_fold_correction_01",
            )
            release_phase = correction_phase
            if args.self_contact:
                minimum_self_contact_separation_during_fold_m = min(
                    minimum_self_contact_separation_during_fold_m,
                    minimum_nonlocal_node_separation_m(local_nodes(scene, cloth)),
                )

        laydown_gripper_w = robot.data.body_pos_w.torch[:, gripper_body_ids]
        laydown_gripper_xyzw = robot.data.body_quat_w.torch[:, gripper_body_ids]
        laydown_registered_tcp_w = gripper_tcp_positions_w(
            laydown_gripper_w, laydown_gripper_xyzw
        )
        print(
            "S1_LAYDOWN_REGISTERED_TCP "
            + json.dumps(
                {
                    "env_0_local_m": (
                        laydown_registered_tcp_w[0]
                        - scene.env_origins[0].view(1, 3)
                    ).tolist(),
                    "planning_targets_m": [
                        target["xyz_m"] for target in release_phase["targets"]
                    ],
                },
                sort_keys=True,
            ),
            flush=True,
        )
        pinned_laydown_hold_steps = max(
            1, round(PINNED_LAYDOWN_HOLD_S / physics_dt_s)
        )
        for _ in range(pinned_laydown_hold_steps):
            write_scripted_arm_state_and_drive_targets(
                robot, current_row, zero_velocity, joint_ids
            )
            enforce_newton_contact_gated_retention()
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError(
                    "cloth produced non-finite nodes during pinned laydown hold"
                )
        nodes_at_laydown = local_nodes(scene, cloth).clone()
        if scripted_attachment_used:
            disable_runtime_attachments(attachment_records)
            simulation_app.update()
            if not runtime_attachments_are_disabled(attachment_records):
                raise RuntimeError("one or more vertex attachments remained enabled")
        if newton_state_retention_used:
            deactivate_newton_contact_gated_retention(
                "first_fold_q0_opening_started"
            )

        achieved_grippers_before_open = robot.data.joint_pos.torch[
            :, joint_ids
        ][:, GRIPPER_JOINT_INDICES].clone()
        open_row = phase_model_tensor(
            source,
            release_phase,
            gripper_project_positions_rad={"left": 0.0, "right": 0.0},
            environment_count=environment_count,
            device=sim.device,
        )
        # Q0 is the measured 16.7 mm geometry anchor, not the desired release
        # aperture.  Open halfway from Q0 toward each side's operational
        # maximum so the 3 mm numerical cloth clears reliably after the
        # retention target is disabled.
        open_row[:, GRIPPER_JOINT_INDICES[0]] = (
            SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
        )
        open_row[:, GRIPPER_JOINT_INDICES[1]] = (
            SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
        )
        jaw_open_steps = max(2, round(JAW_OPEN_DURATION_S / physics_dt_s))
        for step in range(1, jaw_open_steps + 1):
            alpha = step / jaw_open_steps
            target = current_row + alpha * (open_row - current_row)
            write_scripted_arm_state_and_drive_targets(
                robot,
                target,
                zero_velocity,
                joint_ids,
                lock_gripper_state=True,
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
        settle_physical_arm_drives(
            robot,
            open_row,
            zero_velocity,
            joint_ids,
            scene,
            sim,
            cloth,
            physics_dt_s,
            args.arm_target_settle_timeout_s,
            "first_release",
            lock_gripper_state=True,
        )
        post_open_hold_steps = max(
            1, round(POST_OPEN_RELEASE_HOLD_S / physics_dt_s)
        )
        for _ in range(post_open_hold_steps):
            write_scripted_arm_state_and_drive_targets(
                robot,
                open_row,
                zero_velocity,
                joint_ids,
                lock_gripper_state=True,
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError(
                    "cloth produced non-finite nodes during post-open release hold"
                )
        achieved_open_grippers = robot.data.joint_pos.torch[:, joint_ids][
            :, GRIPPER_JOINT_INDICES
        ]
        target_open_grippers = open_row[:, GRIPPER_JOINT_INDICES]
        maximum_gripper_opening_residual_rad = float(
            torch.max(
                torch.abs(achieved_open_grippers - target_open_grippers)
            ).item()
        )
        requested_release_travel_rad = (
            target_open_grippers - achieved_grippers_before_open
        )
        achieved_release_travel_rad = (
            achieved_open_grippers - achieved_grippers_before_open
        )
        release_travel_fraction = achieved_release_travel_rad / torch.clamp(
            requested_release_travel_rad, min=1.0e-6
        )
        minimum_release_travel_fraction = float(
            torch.min(release_travel_fraction).item()
        )
        print(
            "S1_GRIPPER_RELEASE_OPEN "
            + json.dumps(
                {
                    "achieved_env_0_rad": achieved_open_grippers[0].tolist(),
                    "hold_s": POST_OPEN_RELEASE_HOLD_S,
                    "maximum_residual_rad": (
                        maximum_gripper_opening_residual_rad
                    ),
                    "minimum_release_travel_fraction": (
                        minimum_release_travel_fraction
                    ),
                    "minimum_release_travel_fraction_limit": (
                        MINIMUM_GRIPPER_RELEASE_TRAVEL_FRACTION
                    ),
                    "target_env_0_rad": target_open_grippers[0].tolist(),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        if (
            maximum_gripper_opening_residual_rad
            > MAXIMUM_GRIPPER_OPENING_RESIDUAL_RAD
        ):
            raise RuntimeError(
                "gripper opening tracking diverged before retreat: residual="
                f"{maximum_gripper_opening_residual_rad:.6f} rad > "
                f"{MAXIMUM_GRIPPER_OPENING_RESIDUAL_RAD:.6f} rad"
            )
        if minimum_release_travel_fraction < MINIMUM_GRIPPER_RELEASE_TRAVEL_FRACTION:
            raise RuntimeError(
                "gripper did not open far enough before retreat: minimum release "
                f"travel fraction={minimum_release_travel_fraction:.6f}"
            )
        nodes_after_release = local_nodes(scene, cloth).clone()

        gripper_before_retreat_w = robot.data.body_pos_w.torch[
            :, gripper_body_ids
        ].clone()
        if suspended_gravity_replay:
            fold_records = source["canonical_replay"]["first_fold"]
            release_index = next(
                index
                for index, record in enumerate(fold_records)
                if record.get("attachment_event")
                == "release_both_edge_patches_after_gravity_laydown_gate"
            )
            released_motion_phases = fold_records[release_index + 1 :]
        else:
            released_motion_phases = [phase(source, "first_retreat")]
        retreat_row = open_row
        retreat_phase_name = "first_release"
        for retreat_phase in released_motion_phases:
            target_retreat_row = phase_model_tensor(
                source,
                retreat_phase,
                gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                environment_count=environment_count,
                device=sim.device,
            )
            retreat_steps = max(
                2,
                round(
                    (
                        args.fold_phase_seconds
                        if suspended_gravity_replay
                        else args.retreat_seconds
                    )
                    / physics_dt_s
                ),
            )
            for step in range(1, retreat_steps + 1):
                alpha = step / retreat_steps
                target = retreat_row + alpha * (target_retreat_row - retreat_row)
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    target,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                    raise RuntimeError(
                        "cloth produced non-finite nodes during release retreat "
                        f"{retreat_phase['name']}"
                    )
            retreat_row = target_retreat_row
            retreat_phase_name = retreat_phase["name"]
        settle_physical_arm_drives(
            robot,
            retreat_row,
            zero_velocity,
            joint_ids,
            scene,
            sim,
            cloth,
            physics_dt_s,
            args.arm_target_settle_timeout_s,
            retreat_phase_name,
            lock_gripper_state=True,
        )
        settled_run_after_release = 0
        settled_step_after_release = None
        release_shape_frame_samples = []
        previous_release_shape_nodes = local_nodes(scene, cloth).clone()
        release_shape_video_sample_steps = max(
            1, round((1.0 / RELEASE_SHAPE_VIDEO_FPS) / physics_dt_s)
        )
        maximum_release_settle_steps = math.ceil(
            args.settle_timeout_s / physics_dt_s
        )
        for step in range(1, maximum_release_settle_steps + 1):
            write_scripted_arm_state_and_drive_targets(
                robot,
                retreat_row,
                zero_velocity,
                joint_ids,
                lock_gripper_state=True,
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            release_speed = float(
                torch.max(
                    torch.linalg.vector_norm(cloth.data.nodal_vel_w.torch, dim=-1)
                ).item()
            )
            if step % release_shape_video_sample_steps == 0:
                current_release_shape_nodes = local_nodes(scene, cloth).clone()
                release_shape_displacement = torch.linalg.vector_norm(
                    current_release_shape_nodes - previous_release_shape_nodes,
                    dim=-1,
                )
                maximum_environment_p95_displacement_m = float(
                    torch.max(
                        torch.quantile(release_shape_displacement, 0.95, dim=1)
                    ).item()
                )
                maximum_environment_median_displacement_m = float(
                    torch.max(
                        torch.median(release_shape_displacement, dim=1).values
                    ).item()
                )
                release_shape_frame_samples.append(
                    {
                        "time_s": step * physics_dt_s,
                        "maximum_environment_p95_displacement_m": (
                            maximum_environment_p95_displacement_m
                        ),
                        "maximum_environment_median_displacement_m": (
                            maximum_environment_median_displacement_m
                        ),
                    }
                )
                settled_run_after_release = (
                    settled_run_after_release + 1
                    if maximum_environment_p95_displacement_m
                    <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                    else 0
                )
                previous_release_shape_nodes = current_release_shape_nodes
                if (
                    settled_run_after_release
                    >= RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                ):
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
                        "final_shape_frame_sample": (
                            release_shape_frame_samples[-1]
                            if release_shape_frame_samples
                            else None
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            if args.keep_open:
                print(
                    "S1_VERTEX_PATCH_FAILED_GATE_GUI_KEEP_OPEN "
                    "close the Isaac Sim window when visual inspection is done",
                    flush=True,
                )
                while simulation_app.is_running():
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        keep_open_row,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
            raise RuntimeError(
                "cloth shape did not settle after release; final maximum "
                f"speed diagnostic={release_speed:.6f} m/s"
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
        release_patch_following_flags = [
            bool(
                float(lift.item()) > MAXIMUM_RELEASE_PATCH_LIFT_M
                and float(distance.item())
                < MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
            )
            for lift, distance in zip(
                release_patch_lifts, release_patch_to_jaw_distances
            )
        ]
        # A released edge can rise as the folded towel relaxes; height change
        # alone does not prove that it remained stuck to the retreating jaw.
        # Likewise, proximity alone can occur while the jaw is withdrawing
        # sideways at table height.  Treat it as a failed release only when
        # both observations agree that the same patch followed the jaw.
        release_separation_gate = {
            "maximum_release_patch_lift_m": maximum_release_patch_lift_m,
            "maximum_release_patch_lift_limit_m": MAXIMUM_RELEASE_PATCH_LIFT_M,
            "minimum_release_patch_to_jaw_distance_m": (
                minimum_release_patch_to_jaw_distance_m
            ),
            "minimum_release_patch_to_jaw_distance_limit_m": (
                MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
            ),
            "per_patch_release_lift_m": [
                float(value.item()) for value in release_patch_lifts
            ],
            "per_patch_to_jaw_distance_m": [
                float(value.item()) for value in release_patch_to_jaw_distances
            ],
            "per_patch_followed_jaw": release_patch_following_flags,
            "passed": not any(release_patch_following_flags),
        }
        print(
            "S1_RELEASE_SEPARATION_GATE "
            + json.dumps(release_separation_gate, sort_keys=True),
            flush=True,
        )
        if not release_separation_gate["passed"]:
            raise RuntimeError(
                "released patch rose with and remained close to the retreating "
                "jaw: lift="
                f"{maximum_release_patch_lift_m:.6f} m, distance="
                f"{minimum_release_patch_to_jaw_distance_m:.6f} m"
            )

        if post_release_correction_replay is not None:
            correction_records = post_release_correction_replay["phases"]
            correction_contact_index = next(
                index
                for index, record in enumerate(correction_records)
                if record["name"] == f"{correction_probe_id}_contact"
            )
            correction_target_index = next(
                index
                for index, record in enumerate(correction_records)
                if record["name"] == f"{correction_probe_id}_target"
            )
            correction_nodes_before_approach = local_nodes(scene, cloth).clone()
            correction_row = retreat_row
            correction_phase_steps = max(
                2, round(args.fold_phase_seconds / physics_dt_s)
            )

            for correction_phase_record in correction_records[
                : correction_contact_index + 1
            ]:
                target_correction_row = phase_model_tensor(
                    source,
                    correction_phase_record,
                    gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                    environment_count=environment_count,
                    device=sim.device,
                )
                for step in range(1, correction_phase_steps + 1):
                    alpha = step / correction_phase_steps
                    target = correction_row + alpha * (
                        target_correction_row - correction_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot, target, zero_velocity, joint_ids
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                        raise RuntimeError(
                            "cloth produced non-finite nodes during correction approach "
                            f"{correction_phase_record['name']}"
                        )
                correction_row = target_correction_row
                settle_physical_arm_drives(
                    robot,
                    correction_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    correction_phase_record["name"],
                )

            correction_contact_record = correction_records[
                correction_contact_index
            ]
            correction_open_contact_row = correction_row
            correction_pinch_row = phase_model_tensor(
                source,
                correction_contact_record,
                gripper_project_positions_rad=(
                    PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
                ),
                environment_count=environment_count,
                device=sim.device,
            )
            correction_close_steps = max(
                2, round(PINCH_CLOSE_DURATION_S / physics_dt_s)
            )
            for step in range(1, correction_close_steps + 1):
                alpha = step / correction_close_steps
                target = correction_open_contact_row + alpha * (
                    correction_pinch_row - correction_open_contact_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            for _ in range(max(2, round(PINCH_HOLD_DURATION_S / physics_dt_s))):
                write_scripted_arm_state_and_drive_targets(
                    robot, correction_pinch_row, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            settle_physical_arm_drives(
                robot,
                correction_pinch_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                f"{correction_probe_id}_contact_closed",
            )

            correction_contact_snapshot = newton_soft_contact_snapshot()
            correction_bilateral = correction_contact_snapshot.get(
                "bilateral_same_particle_contacts", {}
            )
            if any(not correction_bilateral.get(side) for side in ("left", "right")):
                correction_pad_centers_w = body_local_points_to_world(
                    robot.data.body_pos_w.torch[:, jaw_body_ids],
                    robot.data.body_quat_w.torch[:, jaw_body_ids],
                    (
                        FIXED_JAW_PAD_CENTER_PARENT_M,
                        MOVING_JAW_PAD_CENTER_PARENT_M,
                        FIXED_JAW_PAD_CENTER_PARENT_M,
                        MOVING_JAW_PAD_CENTER_PARENT_M,
                    ),
                )
                correction_nodes_w = cloth.data.nodal_pos_w.torch.clone()
                correction_pad_to_node = torch.cdist(
                    correction_pad_centers_w, correction_nodes_w
                )
                correction_nearest_distance, correction_nearest_index = torch.min(
                    correction_pad_to_node, dim=2
                )
                correction_contact_geometry = {
                    "pad_centers_env_0_w_m": {
                        name: [
                            float(value)
                            for value in correction_pad_centers_w[0, index].tolist()
                        ]
                        for index, name in enumerate(
                            (
                                "left_fixed",
                                "left_moving",
                                "right_fixed",
                                "right_moving",
                            )
                        )
                    },
                    "nearest_node_by_pad_env_0": {
                        name: {
                            "index": int(correction_nearest_index[0, index].item()),
                            "distance_m": float(
                                correction_nearest_distance[0, index].item()
                            ),
                            "position_w_m": [
                                float(value)
                                for value in correction_nodes_w[
                                    0,
                                    int(correction_nearest_index[0, index].item()),
                                ].tolist()
                            ],
                        }
                        for index, name in enumerate(
                            (
                                "left_fixed",
                                "left_moving",
                                "right_fixed",
                                "right_moving",
                            )
                        )
                    },
                }
                print(
                    "S1_POST_RELEASE_CORRECTION_CONTACT_GEOMETRY "
                    + json.dumps(correction_contact_geometry, sort_keys=True),
                    flush=True,
                )
                raise RuntimeError(
                    "observed-boundary correction actual-contact gate failed: "
                    "each gripper must contact the same towel particle on both jaws; "
                    f"snapshot={correction_contact_snapshot}"
                )
            correction_upper_edge_columns = {0, 1}
            correction_non_upper_edge_particles = {
                side: [
                    int(index)
                    for index in correction_bilateral[side]
                    if int(index) % (CLOTH_RESOLUTION[0] + 1)
                    not in correction_upper_edge_columns
                ]
                for side in ("left", "right")
            }
            if any(correction_non_upper_edge_particles.values()):
                raise RuntimeError(
                    "observed-boundary correction pinched a non-upper-edge "
                    "cloth layer; refusing to retain both layers: "
                    f"particles={correction_non_upper_edge_particles}"
                )
            actual_bilateral_particles_by_side = {
                side: [int(index) for index in correction_bilateral[side]]
                for side in ("left", "right")
            }
            selected_indices = [
                [
                    actual_bilateral_particles_by_side["left"],
                    actual_bilateral_particles_by_side["right"],
                ]
            ]
            nodes_before_w = cloth.data.nodal_pos_w.torch.clone()
            gripper_before_w = robot.data.body_pos_w.torch[
                :, gripper_body_ids
            ].clone()
            gripper_orientation_before_xyzw = robot.data.body_quat_w.torch[
                :, gripper_body_ids
            ].clone()
            contact_gated_local_positions.clear()
            activate_newton_contact_gated_retention()
            enforce_newton_contact_gated_retention()
            print(
                "S1_POST_RELEASE_CORRECTION_ACTUAL_CONTACTS "
                + json.dumps(correction_contact_snapshot, sort_keys=True),
                flush=True,
            )

            correction_row = correction_pinch_row
            for correction_phase_record in correction_records[
                correction_contact_index + 1 : correction_target_index + 1
            ]:
                target_correction_row = phase_model_tensor(
                    source,
                    correction_phase_record,
                    gripper_project_positions_rad=(
                        PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
                    ),
                    environment_count=environment_count,
                    device=sim.device,
                )
                for step in range(1, correction_phase_steps + 1):
                    alpha = step / correction_phase_steps
                    target = correction_row + alpha * (
                        target_correction_row - correction_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot, target, zero_velocity, joint_ids
                    )
                    enforce_newton_contact_gated_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                        raise RuntimeError(
                            "cloth produced non-finite nodes during correction drag "
                            f"{correction_phase_record['name']}"
                        )
                correction_row = target_correction_row
                settle_physical_arm_drives(
                    robot,
                    correction_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    correction_phase_record["name"],
                    post_step_callback=enforce_newton_contact_gated_retention,
                )

            deactivate_newton_contact_gated_retention(
                "post_release_correction_q0_opening_started"
            )
            correction_open_row = phase_model_tensor(
                source,
                correction_records[correction_target_index],
                gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                environment_count=environment_count,
                device=sim.device,
            )
            correction_nodes_before_release = local_nodes(scene, cloth).clone()
            for step in range(1, jaw_open_steps + 1):
                alpha = step / jaw_open_steps
                target = correction_row + alpha * (
                    correction_open_row - correction_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            settle_physical_arm_drives(
                robot,
                correction_open_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                f"{correction_probe_id}_release",
            )
            for _ in range(post_open_hold_steps):
                write_scripted_arm_state_and_drive_targets(
                    robot, correction_open_row, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            correction_post_open_contacts = newton_soft_contact_snapshot()

            correction_row = correction_open_row
            for correction_phase_record in correction_records[
                correction_target_index + 1 :
            ]:
                target_correction_row = phase_model_tensor(
                    source,
                    correction_phase_record,
                    gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                    environment_count=environment_count,
                    device=sim.device,
                )
                for step in range(1, correction_phase_steps + 1):
                    alpha = step / correction_phase_steps
                    target = correction_row + alpha * (
                        target_correction_row - correction_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot, target, zero_velocity, joint_ids
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                        raise RuntimeError(
                            "cloth produced non-finite nodes during correction retreat "
                            f"{correction_phase_record['name']}"
                        )
                correction_row = target_correction_row
            settle_physical_arm_drives(
                robot,
                correction_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                correction_records[-1]["name"],
            )
            keep_open_row = correction_row

            correction_shape_samples = []
            correction_settled_run = 0
            correction_settled_step = None
            correction_previous_nodes = local_nodes(scene, cloth).clone()
            for step in range(1, maximum_release_settle_steps + 1):
                write_scripted_arm_state_and_drive_targets(
                    robot, correction_row, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                if step % release_shape_video_sample_steps == 0:
                    correction_current_nodes = local_nodes(scene, cloth).clone()
                    correction_displacement = torch.linalg.vector_norm(
                        correction_current_nodes - correction_previous_nodes,
                        dim=-1,
                    )
                    correction_p95_displacement_m = float(
                        torch.max(
                            torch.quantile(correction_displacement, 0.95, dim=1)
                        ).item()
                    )
                    correction_shape_samples.append(
                        {
                            "time_s": step * physics_dt_s,
                            "maximum_environment_p95_displacement_m": (
                                correction_p95_displacement_m
                            ),
                        }
                    )
                    correction_settled_run = (
                        correction_settled_run + 1
                        if correction_p95_displacement_m
                        <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                        else 0
                    )
                    correction_previous_nodes = correction_current_nodes
                    if (
                        correction_settled_run
                        >= RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                    ):
                        correction_settled_step = step
                        break
            if correction_settled_step is None:
                raise RuntimeError(
                    "cloth shape did not settle after observed-boundary correction"
                )
            nodes_final = local_nodes(scene, cloth).clone()
            nodes_final_w = cloth.data.nodal_pos_w.torch.clone()
            correction_bilateral_after_open = correction_post_open_contacts.get(
                "bilateral_same_particle_contacts", {}
            )
            post_release_correction_result = {
                "replay_path": str(
                    args.post_release_correction_replay.resolve()
                ),
                "observation": post_release_correction_replay.get("observation"),
                "actual_contact_particles": actual_bilateral_particles_by_side,
                "fresh_actual_contact_gate_passed": True,
                "first_grasp_particle_identity_reused": False,
                "retention_mechanism": (
                    "isaaclab_newton_nodal_kinematic_target"
                ),
                "post_open_bilateral_contacts": (
                    correction_bilateral_after_open
                ),
                "settle_time_s": correction_settled_step * physics_dt_s,
                "shape_settle_frame_samples": correction_shape_samples,
                "maximum_cloth_displacement_during_open_approach_m": float(
                    torch.max(
                        torch.linalg.vector_norm(
                            correction_nodes_before_release
                            - correction_nodes_before_approach,
                            dim=-1,
                        )
                    ).item()
                ),
            }
            print(
                "S1_POST_RELEASE_OBSERVED_BOUNDARY_CORRECTION "
                + json.dumps(post_release_correction_result, sort_keys=True),
                flush=True,
            )

        table_size = source["worktable_geometry"]["size_xyz_m"]
        table_pose = source["worktable_geometry"]["pose_xyz_m"]
        table_top_z_m = float(table_pose[2]) + 0.5 * float(table_size[2])
        table_center_xy_w = scene.env_origins[:, None, :2] + torch.tensor(
            table_pose[:2], dtype=torch.float32, device=sim.device
        ).view(1, 1, 2)
        table_half_size_xy_m = torch.tensor(
            table_size[:2], dtype=torch.float32, device=sim.device
        ).view(1, 1, 2) * 0.5
        nodes_relative_to_table_xy_m = nodes_final_w[..., :2] - table_center_xy_w
        nodes_over_table_footprint = torch.all(
            torch.abs(nodes_relative_to_table_xy_m)
            <= table_half_size_xy_m + 1.0e-6,
            dim=-1,
        )
        if not torch.all(torch.any(nodes_over_table_footprint, dim=1)):
            raise RuntimeError("one or more environments have no cloth over the table")
        table_top_z_w = scene.env_origins[:, None, 2] + table_top_z_m
        final_table_clearance_m = nodes_final_w[..., 2] - table_top_z_w
        minimum_final_clearance_m = float(
            torch.min(final_table_clearance_m[nodes_over_table_footprint]).item()
        )
        maximum_final_height_m = float(
            torch.max(final_table_clearance_m).item()
        )
        below_table_outside_footprint = (
            ~nodes_over_table_footprint
        ) & (final_table_clearance_m < 0.0)
        per_environment_nodes_over_table_footprint = torch.sum(
            nodes_over_table_footprint, dim=1
        ).tolist()
        per_environment_nodes_below_table_outside_footprint = torch.sum(
            below_table_outside_footprint, dim=1
        ).tolist()
        if minimum_final_clearance_m < -MAXIMUM_FINAL_TABLE_PENETRATION_M:
            raise RuntimeError(
                f"released cloth penetrated table by {-minimum_final_clearance_m:.6f} m"
            )
        grid_shape = (
            environment_count,
            CLOTH_RESOLUTION[1] + 1,
            CLOTH_RESOLUTION[0] + 1,
            3,
        )
        final_grid = nodes_final.reshape(grid_shape)
        first_half = final_grid[:, :, : (CLOTH_RESOLUTION[0] + 1) // 2]
        mirrored_second_half = torch.flip(
            final_grid[:, :, (CLOTH_RESOLUTION[0] + 1) // 2 :], dims=(2,)
        )
        paired_vertex_xy_error_m = torch.linalg.vector_norm(
            first_half[..., :2] - mirrored_second_half[..., :2], dim=-1
        )
        per_environment_p95_paired_vertex_xy_error_m = torch.quantile(
            paired_vertex_xy_error_m.reshape(environment_count, -1),
            0.95,
            dim=1,
        )
        maximum_p95_paired_vertex_xy_error_m = float(
            torch.max(per_environment_p95_paired_vertex_xy_error_m).item()
        )
        per_environment_first_fold_footprint_width_m = (
            torch.max(nodes_final[..., 0], dim=1).values
            - torch.min(nodes_final[..., 0], dim=1).values
        )
        maximum_first_fold_footprint_width_m = float(
            torch.max(per_environment_first_fold_footprint_width_m).item()
        )
        topology_median_xyz_m = torch.median(final_grid, dim=1).values
        topology_median_x_m = topology_median_xyz_m[..., 0]
        raw_underfold_observations = []
        raw_underfold_failures = []
        for environment_index in range(environment_count):
            profile = topology_median_x_m[environment_index]
            main_fold_column = int(torch.argmax(profile).item())
            lower_segment = profile[main_fold_column:]
            terminal_curl_local_column = int(torch.argmin(lower_segment).item())
            terminal_curl_column = main_fold_column + terminal_curl_local_column
            terminal_curl_amplitude_m = float(
                profile[-1] - profile[terminal_curl_column]
            )
            terminal_curl_fraction = (
                (profile.shape[0] - 1 - terminal_curl_column)
                / (profile.shape[0] - 1)
            )
            exposed_lower_edge_m = float(profile[0] - profile[-1])
            profile_xz = topology_median_xyz_m[environment_index, :, (0, 2)]
            segment_lengths_m = torch.linalg.vector_norm(
                profile_xz[1:] - profile_xz[:-1], dim=-1
            )
            first_layer_length_m = float(
                torch.sum(segment_lengths_m[:main_fold_column]).item()
            )
            second_layer_length_m = float(
                torch.sum(segment_lengths_m[main_fold_column:]).item()
            )
            profile_length_m = first_layer_length_m + second_layer_length_m
            maximum_layer_fraction = (
                max(first_layer_length_m, second_layer_length_m) / profile_length_m
                if profile_length_m > 0.0
                else math.inf
            )
            observation = {
                "environment_index": environment_index,
                "main_fold_column": main_fold_column,
                "main_fold_column_limits": [
                    MINIMUM_RAW_MAIN_FOLD_COLUMN,
                    MAXIMUM_RAW_MAIN_FOLD_COLUMN,
                ],
                "exposed_lower_edge_m": exposed_lower_edge_m,
                "exposed_lower_edge_role": "signed_diagnostic_only",
                "profile_layer_lengths_m": [
                    first_layer_length_m,
                    second_layer_length_m,
                ],
                "profile_length_m": profile_length_m,
                "profile_length_limits_m": [
                    MINIMUM_NOMINAL_PROFILE_LENGTH_M,
                    MAXIMUM_NOMINAL_PROFILE_LENGTH_M,
                ],
                "maximum_layer_fraction": maximum_layer_fraction,
                "maximum_layer_fraction_limit": MAXIMUM_NOMINAL_LAYER_FRACTION,
                "terminal_curl_start_column": terminal_curl_column,
                "terminal_curl_amplitude_m": terminal_curl_amplitude_m,
                "terminal_curl_amplitude_limit_m": (
                    MAXIMUM_RAW_TERMINAL_CURL_AMPLITUDE_M
                ),
                "terminal_curl_fraction": terminal_curl_fraction,
                "terminal_curl_fraction_limit": (
                    MAXIMUM_RAW_TERMINAL_CURL_FRACTION
                ),
            }
            raw_underfold_observations.append(observation)
            if not (
                MINIMUM_RAW_MAIN_FOLD_COLUMN
                <= main_fold_column
                <= MAXIMUM_RAW_MAIN_FOLD_COLUMN
            ):
                raw_underfold_failures.append(
                    f"env_{environment_index}_main_fold_column={main_fold_column}"
                )
            if not (
                MINIMUM_NOMINAL_PROFILE_LENGTH_M
                <= profile_length_m
                <= MAXIMUM_NOMINAL_PROFILE_LENGTH_M
            ):
                raw_underfold_failures.append(
                    f"env_{environment_index}_profile_length_m="
                    f"{profile_length_m:.6f}"
                )
            if maximum_layer_fraction > MAXIMUM_NOMINAL_LAYER_FRACTION:
                raw_underfold_failures.append(
                    f"env_{environment_index}_maximum_layer_fraction="
                    f"{maximum_layer_fraction:.6f}>"
                    f"{MAXIMUM_NOMINAL_LAYER_FRACTION:.6f}"
                )
            if terminal_curl_amplitude_m > MAXIMUM_RAW_TERMINAL_CURL_AMPLITUDE_M:
                raw_underfold_failures.append(
                    f"env_{environment_index}_terminal_curl_amplitude_m="
                    f"{terminal_curl_amplitude_m:.6f}"
                )
            if terminal_curl_fraction > MAXIMUM_RAW_TERMINAL_CURL_FRACTION:
                raw_underfold_failures.append(
                    f"env_{environment_index}_terminal_curl_fraction="
                    f"{terminal_curl_fraction:.6f}"
                )
        print(
            "S1_FIRST_FOLD_ALIGNMENT "
            + json.dumps(
                {
                    "env_0_median_x_by_topology_column_m": [
                        float(value)
                        for value in torch.median(
                            final_grid[0, :, :, 0], dim=0
                        ).values.tolist()
                    ],
                    "env_0_median_z_by_topology_column_m": [
                        float(value)
                        for value in torch.median(
                            final_grid[0, :, :, 2], dim=0
                        ).values.tolist()
                    ],
                    "maximum_final_height_m": maximum_final_height_m,
                    "per_environment_footprint_width_m": (
                        per_environment_first_fold_footprint_width_m.tolist()
                    ),
                    "per_environment_p95_paired_vertex_xy_error_m": (
                        per_environment_p95_paired_vertex_xy_error_m.tolist()
                    ),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        strict_alignment_gate_failures = []
        if maximum_final_height_m > MAXIMUM_FINAL_CLOTH_HEIGHT_M:
            strict_alignment_gate_failures.append(
                "maximum_height_m="
                f"{maximum_final_height_m:.6f}>"
                f"{MAXIMUM_FINAL_CLOTH_HEIGHT_M:.6f}"
            )
        if (
            maximum_first_fold_footprint_width_m
            > MAXIMUM_FIRST_FOLD_FOOTPRINT_WIDTH_M
        ):
            strict_alignment_gate_failures.append(
                "footprint_width_m="
                f"{maximum_first_fold_footprint_width_m:.6f}>"
                f"{MAXIMUM_FIRST_FOLD_FOOTPRINT_WIDTH_M:.6f}"
            )
        if (
            maximum_p95_paired_vertex_xy_error_m
            > MAXIMUM_FIRST_FOLD_PAIRED_VERTEX_P95_XY_ERROR_M
        ):
            strict_alignment_gate_failures.append(
                "paired_vertex_p95_xy_error_m="
                f"{maximum_p95_paired_vertex_xy_error_m:.6f}>"
                f"{MAXIMUM_FIRST_FOLD_PAIRED_VERTEX_P95_XY_ERROR_M:.6f}"
            )
        if maximum_final_height_m > MAXIMUM_FINAL_CLOTH_HEIGHT_M:
            raw_underfold_failures.append(
                "maximum_height_m="
                f"{maximum_final_height_m:.6f}>"
                f"{MAXIMUM_FINAL_CLOTH_HEIGHT_M:.6f}"
            )
        if maximum_first_fold_footprint_width_m < (
            MINIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M
        ):
            raw_underfold_failures.append(
                "nominal_footprint_width_m="
                f"{maximum_first_fold_footprint_width_m:.6f}<"
                f"{MINIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M:.6f}"
            )
        if maximum_first_fold_footprint_width_m > (
            MAXIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M
        ):
            raw_underfold_failures.append(
                "nominal_footprint_width_m="
                f"{maximum_first_fold_footprint_width_m:.6f}>"
                f"{MAXIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M:.6f}"
            )
        correction_was_executed = post_release_correction_replay is not None
        shape_gate_failures = (
            strict_alignment_gate_failures
            if correction_was_executed
            else raw_underfold_failures
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
            "shape_gate_passed": not shape_gate_failures,
            "shape_gate_failures": shape_gate_failures,
            "shape_gate_stage": (
                "post_correction_strict_alignment"
                if correction_was_executed
                else "nominal_target_50_acceptance_55_45_envelope"
            ),
            "raw_underfold_gate_passed": not raw_underfold_failures,
            "raw_underfold_gate_failures": raw_underfold_failures,
            "raw_underfold_observations": raw_underfold_observations,
            "nominal_half_fold_footprint_width_limits_m": [
                MINIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M,
                MAXIMUM_NOMINAL_HALF_FOLD_FOOTPRINT_WIDTH_M,
            ],
            "strict_alignment_gate_passed": not strict_alignment_gate_failures,
            "strict_alignment_gate_failures": strict_alignment_gate_failures,
            "source_phases": (
                [
                    record["name"]
                    for record in source["canonical_replay"]["first_fold"]
                ]
                if suspended_gravity_replay
                else [
                    *[f"first_fold_{index:02d}" for index in range(1, 17)],
                    *(
                        ["first_fold_correction_01"]
                        if correction_phase is not None
                        else []
                    ),
                ]
            ),
            "release_event": (
                "disable_both_vertex_patch_attachments_then_open_jaws_after_first_fold_16"
                if legacy_attachment_used
                else (
                    "disable_actual_contact_gated_no_slip_then_open_q0_after_"
                    "first_gravity_overcenter_03"
                    if suspended_gravity_replay
                    else "disable_contact_gated_no_slip_retention_then_open_jaws_after_first_fold_16"
                )
                if contact_gated_retention_used
                else "open_both_physical_jaws_after_first_fold_16"
            ),
            "jaw_opened_after_attachment_disable": (
                scripted_attachment_used or newton_state_retention_used
            ),
            "suspended_gravity_replay": suspended_gravity_replay,
            "kinematic_replay_path": source.get("kinematic_replay_path"),
            "post_release_correction_replay_path": (
                str(args.post_release_correction_replay.resolve())
                if args.post_release_correction_replay is not None
                else None
            ),
            "jaw_opened_to_release_frictional_grasp": (
                args.grasp_mode == "frictional"
            ),
            "post_open_release_hold_s": POST_OPEN_RELEASE_HOLD_S,
            "pinned_laydown_hold_s": PINNED_LAYDOWN_HOLD_S,
            "forward_lay_contact_hold_s": FORWARD_LAY_CONTACT_HOLD_S,
            "mid_fold_top_camera_length_observation_used": False,
            "maximum_gripper_opening_residual_rad": (
                maximum_gripper_opening_residual_rad
            ),
            "maximum_gripper_opening_residual_limit_rad": (
                MAXIMUM_GRIPPER_OPENING_RESIDUAL_RAD
            ),
            "retreat_phase": retreat_phase_name,
            "fold_phase_duration_s": args.fold_phase_seconds,
            "retreat_duration_s": args.retreat_seconds,
            "arm_target_settle_timeout_s": args.arm_target_settle_timeout_s,
            "settled_step_after_release": settled_step_after_release,
            "shape_settle_time_s": settled_step_after_release * physics_dt_s,
            "shape_settle_observable": (
                "maximum_across_environments_of_node_displacement_p95_per_video_frame"
            ),
            "shape_settle_video_fps": RELEASE_SHAPE_VIDEO_FPS,
            "shape_settle_displacement_threshold_m": (
                RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
            ),
            "shape_settle_consecutive_video_frames": (
                RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
            ),
            "shape_settle_frame_samples": release_shape_frame_samples,
            "final_maximum_node_speed_m_s_diagnostic_only": release_speed,
            "maximum_node_speed_is_settle_gate": False,
            "maximum_release_patch_lift_m": maximum_release_patch_lift_m,
            "maximum_release_patch_lift_limit_m": MAXIMUM_RELEASE_PATCH_LIFT_M,
            "minimum_release_patch_to_jaw_distance_m": (
                minimum_release_patch_to_jaw_distance_m
            ),
            "minimum_release_patch_to_jaw_distance_limit_m": (
                MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
            ),
            "minimum_final_table_clearance_m": minimum_final_clearance_m,
            "table_penetration_checked_only_within_xy_footprint": True,
            "per_environment_nodes_over_table_footprint": (
                per_environment_nodes_over_table_footprint
            ),
            "per_environment_nodes_below_table_outside_footprint": (
                per_environment_nodes_below_table_outside_footprint
            ),
            "maximum_final_table_penetration_limit_m": (
                MAXIMUM_FINAL_TABLE_PENETRATION_M
            ),
            "maximum_final_cloth_height_m": maximum_final_height_m,
            "maximum_final_cloth_height_limit_m": MAXIMUM_FINAL_CLOTH_HEIGHT_M,
            "maximum_first_fold_footprint_width_m": (
                maximum_first_fold_footprint_width_m
            ),
            "maximum_first_fold_footprint_width_limit_m": (
                MAXIMUM_FIRST_FOLD_FOOTPRINT_WIDTH_M
            ),
            "maximum_p95_paired_vertex_xy_error_m": (
                maximum_p95_paired_vertex_xy_error_m
            ),
            "maximum_p95_paired_vertex_xy_error_limit_m": (
                MAXIMUM_FIRST_FOLD_PAIRED_VERTEX_P95_XY_ERROR_M
            ),
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
            "self_contact_enabled_only_after_closed_jaw_gate": (
                args.self_contact and args.physics_backend == "physx"
            ),
            "self_contact_and_retention_authored_while_physics_paused": (
                args.self_contact and args.physics_backend == "physx"
            ),
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
            PLACE_RELEASE_SHAPE_DIAGNOSTIC_FAIL_STATUS
            if shape_gate_failures
            else (
                NOMINAL_HALF_FOLD_ACCEPTED_STATUS
                if args.newton_curvature_softening
                else NOMINAL_HALF_FOLD_MEASURED_MATERIAL_STATUS
            )
            if not correction_was_executed and suspended_gravity_replay
            else SELF_CONTACT_PLACE_RELEASE_PASS_STATUS
            if args.self_contact
            else PLACE_RELEASE_PASS_STATUS
        )
    elif contact_gated_retention_used and not args.grasp_release_probe:
        result_status = CONTACT_GATED_RETENTION_LIFT_PASS_STATUS
    elif args.grasp_mode == "frictional":
        result_status = FRICTIONAL_LIFT_PASS_STATUS

    second_contact_diagnostic = None
    if args.second_contact_diagnostic:
        nodes_before_second_departure = local_nodes(scene, cloth).clone()
        diagnostic_records = [
            *[
                record
                for record in source["canonical_replay"]["first_fold"]
                if record["name"].startswith("first_reobserve_clear")
            ],
            *[
                record
                for record in source["canonical_replay"]["second_fold"]
                if record["name"].startswith("second_departure")
                or record["name"] == "second_contact"
            ],
        ]
        if not diagnostic_records or diagnostic_records[-1]["name"] != "second_contact":
            raise RuntimeError("canonical second-contact diagnostic sequence is incomplete")
        current_row = keep_open_row
        diagnostic_phase_steps = max(2, round(0.30 / physics_dt_s))
        final_diagnostic_target_row = None
        for phase_record in diagnostic_records:
            target_row = phase_model_tensor(
                source,
                phase_record,
                gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                environment_count=environment_count,
                device=sim.device,
            )
            for step in range(1, diagnostic_phase_steps + 1):
                alpha = step / diagnostic_phase_steps
                target = current_row + alpha * (target_row - current_row)
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            current_row = target_row
            final_diagnostic_target_row = target_row
        if final_diagnostic_target_row is None:
            raise RuntimeError("second-contact diagnostic produced no target")
        for _ in range(max(2, round(0.50 / physics_dt_s))):
            write_scripted_arm_state_and_drive_targets(
                robot, final_diagnostic_target_row, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
        second_gripper_positions_w = robot.data.body_pos_w.torch[
            :, gripper_body_ids
        ].clone()
        second_gripper_orientations_xyzw = robot.data.body_quat_w.torch[
            :, gripper_body_ids
        ].clone()
        second_tcp_w = gripper_tcp_positions_w(
            second_gripper_positions_w, second_gripper_orientations_xyzw
        )
        second_approach_w = gripper_approach_axes_w(
            second_gripper_orientations_xyzw
        )
        second_right_tilt_rad = torch.acos(
            torch.clamp(-second_approach_w[:, 1, 2], -1.0, 1.0)
        )
        maximum_second_right_tilt_rad = float(
            torch.max(second_right_tilt_rad).item()
        )
        second_departure_cloth_displacement_m = float(
            torch.max(
                torch.linalg.vector_norm(
                    local_nodes(scene, cloth) - nodes_before_second_departure,
                    dim=-1,
                )
            ).item()
        )
        second_contact_diagnostic = {
            "source_phase_count": len(diagnostic_records),
            "right_tcp_local_m_env_0": [
                float(value)
                for value in (
                    second_tcp_w[0, 1] - scene.env_origins[0]
                ).tolist()
            ],
            "right_approach_axis_w_env_0": [
                float(value) for value in second_approach_w[0, 1].tolist()
            ],
            "maximum_right_approach_tilt_deg": math.degrees(
                maximum_second_right_tilt_rad
            ),
            "maximum_cloth_displacement_during_clear_departure_m": (
                second_departure_cloth_displacement_m
            ),
            "jaws_open": True,
            "attachment_created": False,
            "maximum_joint_target_residual_rad": float(
                torch.max(
                    torch.abs(
                        robot.data.joint_pos.torch[:, joint_ids]
                        - final_diagnostic_target_row
                    )
                ).item()
            ),
            "achieved_joint_positions_env_0_rad": [
                float(value)
                for value in robot.data.joint_pos.torch[0, joint_ids].tolist()
            ],
            "target_joint_positions_env_0_rad": [
                float(value) for value in final_diagnostic_target_row[0].tolist()
            ],
        }
        print(
            "S1_SECOND_CONTACT_DIAGNOSTIC "
            + json.dumps(second_contact_diagnostic, sort_keys=True),
            flush=True,
        )
        keep_open_row = current_row

    curvature_softening_diagnostic = None
    if args.newton_curvature_softening:
        softened_edges_np = curvature_softening_runtime["softened_edges"].numpy()
        ever_softened_edges_np = curvature_softening_runtime[
            "ever_softened_edges"
        ].numpy()
        peak_angles_np = curvature_softening_runtime["peak_absolute_angles"].numpy()
        curvature_softening_diagnostic = {
            "enabled": True,
            "small_bend_edge_stiffness_n_m": NEWTON_EDGE_STIFFNESS_N_M,
            "activation_angle_deg": args.newton_softening_activation_angle_deg,
            "full_softening_angle_deg": args.newton_full_softening_angle_deg,
            "softened_edge_stiffness_n_m": args.newton_softened_edge_stiffness,
            "softened_edge_count": int((softened_edges_np != 0).sum()),
            "ever_softened_edge_count": int(
                (ever_softened_edges_np != 0).sum()
            ),
            "softening_is_reversible": True,
            "softening_transition": "smoothstep",
            "model_edge_count": int(curvature_softening_runtime["model_edge_count"]),
            "maximum_observed_hinge_angle_deg": math.degrees(
                float(peak_angles_np.max())
            ),
            "high_curvature_material_calibrated": False,
            "evidence": "operator reports real cotton towel forms much less arch",
        }

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
        "material_candidate": {
            "path": str(material_candidate.path),
            "sha256": material_candidate.sha256,
            "status": material_candidate.status,
            "measured_mass_thickness_and_table_friction": True,
            "generic_in_plane_youngs_modulus": True,
            "cantilever_surface_bend_stiffness_calibrated": True,
            "edge_release_bend_damping_calibrated": (
                args.physics_backend == "physx"
            ),
            "newton_meter_calibration_status": (
                material_candidate.newton_calibration_status
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_curvature_softening": curvature_softening_diagnostic,
        },
        "gripper_candidate": {
            "path": str(gripper_candidate.path),
            "sha256": hashlib.sha256(gripper_candidate.path.read_bytes()).hexdigest(),
            "status": gripper_candidate.status,
            "q0_gap_mm": gripper_candidate.q0_gap_mm,
            "q0_gap_uncertainty_mm": gripper_candidate.q0_gap_uncertainty_mm,
            "model_q_at_physical_q0_rad": (
                gripper_candidate.model_q_at_physical_q0_rad
            ),
            "project_positive_direction": "closes",
            "model_positive_direction": "opens",
            "physical_robot_motion_authorized": False,
        },
        "cloth": {
            "size_xy_m": list(CLOTH_SIZE_XY_M),
            "resolution": list(CLOTH_RESOLUTION),
            "node_count": int(nodes_after.shape[1]),
            "mass_kg": CLOTH_MASS_KG,
            "density_kg_m3": CLOTH_DENSITY_KG_M3,
            "static_friction": CLOTH_STATIC_FRICTION,
            "dynamic_friction": CLOTH_DYNAMIC_FRICTION,
            "self_collision_enabled": args.self_contact,
            "self_collision_enabled_at_spawn": (
                args.self_contact
                and args.physics_backend == "newton-coupled-vbd"
            ),
            "self_collision_enabled_after_closed_jaw_gate": (
                args.self_contact and args.physics_backend == "physx"
            ),
            "self_collision_authored_paths": staged_self_collision_paths,
            "self_collision_filter_distance_m": (
                SELF_COLLISION_FILTER_DISTANCE_M if args.self_contact else None
            ),
            "surface_thickness_m": CLOTH_SURFACE_THICKNESS_M,
            "initial_table_clearance_m": CLOTH_INITIAL_CLEARANCE_M,
            "contact_offset_m": CLOTH_CONTACT_OFFSET_M,
            "rest_offset_m": CLOTH_REST_OFFSET_M,
            "youngs_modulus_pa": CLOTH_YOUNGS_MODULUS_PA,
            "poissons_ratio": CLOTH_POISSONS_RATIO,
            "elasticity_damping": CLOTH_ELASTICITY_DAMPING,
            "surface_bend_stiffness_pa": CLOTH_SURFACE_BEND_STIFFNESS_PA,
            "bend_damping_s_inv": CLOTH_BEND_DAMPING_S_INV,
            "newton_triangle_stiffness_pa": (
                NEWTON_TRIANGLE_STIFFNESS_PA
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_triangle_area_stiffness_pa": (
                NEWTON_TRIANGLE_AREA_STIFFNESS_PA
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_triangle_damping_pa_s": (
                NEWTON_TRIANGLE_DAMPING_PA_S
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_edge_stiffness_n_m": (
                NEWTON_EDGE_STIFFNESS_N_M
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_edge_damping_n_m_s": (
                NEWTON_EDGE_DAMPING_N_M_S
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "linear_damping_s_inv": CLOTH_LINEAR_DAMPING_S_INV,
            "settling_damping_s_inv": CLOTH_SETTLING_DAMPING_S_INV,
            "settling_threshold_m_s": CLOTH_SETTLING_THRESHOLD_M_S,
            "physics_dt_s": physics_dt_s,
            "solver_position_iteration_count": 24,
            "collision_pair_update_frequency": 4,
            "collision_iteration_multiplier": 2.0,
            "enable_external_forces_every_iteration": args.self_contact,
            "enable_enhanced_determinism": args.self_contact,
            "speculative_ccd_enabled": False,
            "maximum_linear_velocity_m_s": None,
            "material_physical_fidelity_validated": False,
        },
        "attachment": {
            "type": (
                "OmniPhysicsVtxXformAttachment"
                if scripted_attachment_used
                else "IsaacLabNewtonNodalKinematicTargetActualContactConstraint"
                if newton_state_retention_used
                else None
            ),
            "target": (
                "registered_r0g_gripper_frames_under_articulation_links"
                if scripted_attachment_used or newton_state_retention_used
                else None
            ),
            "direct_articulation_link_attachment": (
                scripted_attachment_used or newton_state_retention_used
            ),
            "scripted_attachment_used": scripted_attachment_used,
            "newton_state_constraint_used": newton_state_retention_used,
            "newton_nodal_kinematic_target_api_used": (
                newton_state_retention_used
            ),
            "legacy_floating_attachment_used": legacy_attachment_used,
            "contact_gated_no_slip_retention_used": contact_gated_retention_used,
            "physical_frictional_grasp_validated": (
                args.grasp_mode == "frictional"
            ),
            "retention_basis": (
                "actual_same_particle_bilateral_contact_then_operator_"
                "measured_no_slip_surrogate"
                if contact_gated_retention_used
                else None
            ),
            "proximity_fallback_used": False if contact_gated_retention_used else None,
            "actual_contact_particles_by_side": (
                actual_bilateral_particles_by_side
                if contact_gated_retention_used
                else None
            ),
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
            "pinch_project_gripper_joint_positions_rad": (
                PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "pinch_model_gripper_joint_positions_rad": (
                PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "release_model_gripper_joint_position_rad": (
                RELEASE_MODEL_GRIPPER_JOINT_POSITION_RAD
            ),
            "simulation_medium_release_model_gripper_joint_positions_rad": (
                SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "achieved_gripper_model_rad_by_environment": (
                achieved_gripper_model_rad.tolist()
            ),
            "closing_contact_residual_rad_by_environment": (
                closing_contact_residual_rad.tolist()
            ),
            "pinch_close_duration_s": PINCH_CLOSE_DURATION_S,
            "pinch_hold_duration_s": PINCH_HOLD_DURATION_S,
            "jaw_open_duration_s": JAW_OPEN_DURATION_S,
            "arm_state_overwritten_after_scene_reset": False,
            "gripper_state_constrained_during_release": args.place_release,
            "gripper_release_constraint_reason": (
                "prevent_newton_bounded_revolute_joint_wrap"
                if args.place_release
                else None
            ),
            "pinch_gap_center_tcp_x_m": PINCH_GAP_CENTER_TCP_X_M,
            "explicit_jaw_collision_proxies": {
                "fixed_pad_size_m": list(JAW_PAD_SIZE_M),
                "moving_pad_size_m": list(JAW_PAD_SIZE_M),
                "thin_axis_parent_by_side": JAW_PAD_NORMALS_PARENT,
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
            "contact_diagnostic": jaw_pad_diagnostic,
        },
        "lift": {
            "source_phase": "first_contact",
            "target_phase": "first_fold_01",
            "duration_s": args.lift_seconds,
            "minimum_maximum_node_lift_m": minimum_maximum_node_lift_m,
            "minimum_selected_patch_lift_m": minimum_selected_patch_lift_m,
            "minimum_required_lift_m": (
                MINIMUM_CONTACT_GATED_LIFT_M
                if contact_gated_retention_used
                else MINIMUM_LIFT_M
            ),
            "gripper_displacement_env_0_m": {
                side: [
                    float(value)
                    for value in (gripper_after_w[0, index] - gripper_before_w[0, index]).tolist()
                ]
                for index, side in enumerate(("left", "right"))
            },
        },
        "grasp_release_probe": grasp_release_probe_result,
        "place_release": place_release_result,
        "post_release_correction": post_release_correction_result,
        "second_contact_diagnostic": second_contact_diagnostic,
        "final_cloth_shape_local_m_env_0": (
            nodes_final[0].tolist() if args.place_release else None
        ),
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
            "dual_gripper_vertex_patches_created": scripted_attachment_used,
            "dual_gripper_actual_contact_constraints_created": (
                newton_state_retention_used
            ),
            "low_lift_executed": True,
            "direct_gripper_link_coupling_checked": True,
            "jaw_aligned_attachment_patch_checked": True,
            "gripper_cloth_collision_enabled": True,
            "physical_frictional_grasp_checked": args.grasp_mode == "frictional",
            "vertical_jaw_pinch_checked": vertical_grasp_used,
            "contact_gated_no_slip_retention_checked": (
                contact_gated_retention_used
            ),
            "q0_mid_open_release_checked": args.grasp_release_probe,
            "place_and_release_checked": args.place_release,
            "robot_cloth_collision_checked": False,
            "self_collision_checked": args.self_contact,
            "self_collision_staged_after_closed_jaw_gate": args.self_contact,
            "full_dynamic_cloth_shape_determinism_checked": False,
        },
        "completion_claim": {
            "vertex_patch_attachment_lift_smoke_passed": legacy_attachment_used,
            "vertex_patch_place_release_smoke_passed": (
                args.place_release and legacy_attachment_used
            ),
            "frictional_jaw_lift_smoke_passed": args.grasp_mode == "frictional",
            "frictional_jaw_place_release_smoke_passed": (
                args.place_release and args.grasp_mode == "frictional"
            ),
            "contact_gated_retention_lift_smoke_passed": (
                contact_gated_retention_used
            ),
            "contact_gated_q0_release_smoke_passed": args.grasp_release_probe,
            "contact_gated_retention_place_release_smoke_passed": (
                args.place_release
                and contact_gated_retention_used
                and bool(
                    place_release_result
                    and place_release_result.get("shape_gate_passed")
                )
            ),
            "self_contact_smoke_passed": args.self_contact,
            "s1_completed": False,
            "blocking_reason": (
                "physical_frictional_grasp_full_shape_determinism_and_"
                "in_plane_tensile_response_not_validated"
                if args.self_contact
                else "physical_frictional_grasp_self_collision_full_shape_"
                "determinism_and_in_plane_tensile_response_not_validated"
                if args.place_release
                else "physical_frictional_grasp_place_release_self_collision_"
                "full_shape_determinism_and_in_plane_tensile_response_not_validated"
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
        # Restore the review viewport explicitly after Newton/Fabric has
        # finished its final transform sync.  Keep the stage's authored light
        # and the original workcell camera used throughout this validator.
        lighting_action = omni.kit.actions.core.get_action_registry().get_action(
            "omni.kit.viewport.menubar.lighting", "set_lighting_mode_stage"
        )
        if lighting_action is not None:
            lighting_action.execute()
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
        sim.set_camera_view(
            eye=(first_origin[0] + 0.72, first_origin[1] + 0.48, 0.48),
            target=(first_origin[0] + 0.32, first_origin[1] - 0.12, 0.02),
        )
        for _ in range(3):
            simulation_app.update()
        print("S1_VERTEX_PATCH_GUI_KEEP_OPEN close the Isaac Sim window when done", flush=True)
        while simulation_app.is_running():
            simulation_app.update()
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
