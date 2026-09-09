#!/usr/bin/env python3
"""Validate a vertical dual-jaw towel pinch, retention, fold, and release."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import hashlib
import json
import math
import numpy as np
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
from tools.lib.towel_four_layer_contact import (
    FourLayerContactError,
    RegisteredFaceFrame,
    select_continuous_four_layer_pinch,
    select_continuous_single_sheet_pinch,
)
from tools.lib.towel_second_fold_correction import (
    SECOND_FOLD_TARGET_TOLERANCE_M,
    classify_opposing_jaw_two_layer_contact,
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
parser.add_argument(
    "--cloth-resolution",
    type=int,
    default=31,
    help=(
        "square cloth element resolution; 31 is the material-calibrated fold "
        "resolution, while a finer value may be used only to qualify local "
        "jaw contact geometry"
    ),
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
    "--newton-curvature-softening-stage",
    choices=("global", "s2-post-laydown"),
    default="global",
    help=(
        "apply high-curvature softening throughout, or enable it only after "
        "the accepted S2 bundle has reached the table"
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
    "--newton-fold-hysteresis",
    action="store_true",
    help=(
        "after the S2 laydown gate, store sharp achieved hinge angles as "
        "an experimental unresolved-textile hysteresis model"
    ),
)
parser.add_argument(
    "--newton-fold-hysteresis-angle-deg",
    type=float,
    default=120.0,
    help="minimum achieved absolute hinge angle captured after S2 laydown",
)
parser.add_argument(
    "--jaw-pad-face-size-mm",
    type=float,
    default=6.0,
    help="square jaw contact-face proxy size used by the strict pinch gate",
)
parser.add_argument(
    "--actual-jaw-mesh-contact",
    action="store_true",
    help="retain both imported jaw STL colliders and add rubber only to the fixed face",
)
parser.add_argument(
    "--retain-contact-evidence-only",
    action="store_true",
    help=(
        "for coarse Newton cloth, retain only the two particles that actually "
        "contacted opposing jaw faces instead of rigidly retaining their full cell"
    ),
)
parser.add_argument(
    "--surface-distributed-contact-retention",
    action="store_true",
    help=(
        "after strict actual jaw contact, distribute no-slip retention over the "
        "three vertices of the continuous cloth contact triangle"
    ),
)
parser.add_argument(
    "--progressive-contact-release",
    action="store_true",
    help=(
        "release the surface-retention triangle from its least-supported vertex "
        "to its contact-dominant vertex while the jaws open"
    ),
)
parser.add_argument(
    "--newton-rubber-friction",
    type=float,
    help="Newton-only fixed-pad friction; official cloth examples use 100 for no-slip grip",
)
parser.add_argument(
    "--newton-coupling-mode",
    choices=("two_way", "one_way"),
    default="two_way",
    help=(
        "Newton rigid/deformable coupling; one_way lets cloth react to the robot "
        "without numerically pushing the arm away from its commanded trajectory"
    ),
)
parser.add_argument(
    "--newton-vbd-iterations",
    type=int,
    default=10,
    help="VBD constraint iterations per substep",
)
parser.add_argument(
    "--newton-cantilever-calibration",
    type=Path,
    help=(
        "passing resolution-specific Newton cantilever artifact; must be paired "
        "with --newton-edge-release-calibration"
    ),
)
parser.add_argument(
    "--newton-edge-release-calibration",
    type=Path,
    help=(
        "passing resolution-specific Newton edge-release artifact; must be paired "
        "with --newton-cantilever-calibration"
    ),
)
parser.add_argument(
    "--newton-contact-damping",
    type=float,
    default=1.0e-2,
    help="Newton body-particle contact damping coefficient",
)
parser.add_argument(
    "--newton-deep-table-support",
    action="store_true",
    help=(
        "replace the 20 mm visual-table collision with a hidden deep cuboid "
        "that has the same top surface, preventing closest-face inversion"
    ),
)
parser.add_argument(
    "--newton-analytic-table-plane",
    action="store_true",
    help=(
        "replace the finite table-box collision with an invisible analytic "
        "plane at the measured tabletop; the plane is filtered against the "
        "robot and remains a one-sided support for the towel"
    ),
)
parser.add_argument(
    "--physx-post-laydown-bend-stiffness",
    type=float,
    help=(
        "PhysX-only high-curvature approximation: after the S2 laydown gate, "
        "replace the uniform surface bend stiffness before Q0 release"
    ),
)
parser.add_argument(
    "--physx-dynamic-friction-workaround",
    type=float,
    help=(
        "PhysX-only static-friction workaround authored before simulation; "
        "the measured towel/table coefficient remains unchanged in the "
        "material candidate"
    ),
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
    "--left-one-way-contact-limited-model-rad",
    type=float,
    help=(
        "Newton one-way contact diagnostic only: keep the measured left "
        "close command in the report, but stop the simulated moving jaw at "
        "this achieved model-space angle after cloth contact.  One-way "
        "coupling cannot otherwise transmit the cloth reaction back to the jaw."
    ),
)
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
    "--trace-s1-corner-dynamics",
    action="store_true",
    help=(
        "record the four cloth corners after each S1 motion, jaw-opening, and "
        "retreat stage so the first table-boundary departure can be located"
    ),
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
    "--second-fold-replay",
    type=Path,
    help=(
        "passing S2-only full-FK diagnostic supplying the left-arm "
        "left-to-right second-fold phases"
    ),
)
parser.add_argument(
    "--execute-second-fold",
    action="store_true",
    help=(
        "after the S2 contact diagnostic, close the left jaw on both S1 "
        "layers, execute the conventional left-to-right arc, release, and "
        "record the settled raw second-fold shape"
    ),
)
parser.add_argument(
    "--second-fold-contact-only",
    action="store_true",
    help=(
        "restore the accepted S1 checkpoint directly, execute only the S2 "
        "open approach and measured four-layer close, record the continuous-"
        "surface contact gate, and stop before transport"
    ),
)
parser.add_argument(
    "--second-fold-grasp-mode",
    choices=(
        "contact-gated-retention",
        "contact-gated-retention-filtered",
        "frictional",
        "physx-attachment",
    ),
    default="contact-gated-retention",
    help=(
        "S2-only grasp transport model: retain one actual bilateral contact "
        "per S1 layer, rely on jaw contact friction, or use a PhysX cloth-to-rigid "
        "attachment released by jaw opening"
    ),
)
parser.add_argument(
    "--second-fold-release-mode",
    choices=(
        "direct-in-place",
        "right-stabilized",
        "right-surface-press",
        "right-edge-handoff",
    ),
    default="direct-in-place",
    help=(
        "open the left jaw at the gated laydown, use the rejected bilateral "
        "right-arm interior stabilizer diagnostic, hold the laid bundle with "
        "one real fixed-pad surface contact, or transfer the upper two-layer "
        "free edge to a clean opposing-jaw right pinch before left release"
    ),
)
parser.add_argument(
    "--second-fold-gripper-config",
    "--second-layer-gripper-config",
    dest="second_fold_gripper_config",
    type=Path,
    default=ROOT / "config/so101_gripper_s2_four_layer.candidate.json",
    help="simulation-only contact-limited four-layer S2 pinch candidate",
)
parser.add_argument(
    "--post-release-correction-replay",
    type=Path,
    help=(
        "after the first shape settles, execute a passing observed-boundary "
        "correction replay with a fresh actual-contact grasp"
    ),
)
parser.add_argument(
    "--second-fold-correction-replay",
    type=Path,
    help=(
        "after the raw S2 shape settles and both arms clear, execute one "
        "passing right-arm upper-bundle edge correction and reobserve"
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
    "--contact-kinematic-replay",
    type=Path,
    help=(
        "passing suspended-gravity replay supplying only first_contact; "
        "the primary replay supplies the already accepted fold suffix"
    ),
)
parser.add_argument(
    "--validated-contact-checkpoint",
    type=Path,
    help=(
        "previous passing S1 actual-contact lift artifact; if the identical "
        "closed-jaw pose lands on a Newton contact-buffer boundary, reuse only "
        "its proven contact particles before executing the requested fold"
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
IS_NEWTON_BACKEND = args.physics_backend == "newton-coupled-vbd"
if not args.manifest.is_file():
    parser.error(f"manifest does not exist: {args.manifest}")
if not args.material_config.is_file():
    parser.error(f"material config does not exist: {args.material_config}")
if not args.gripper_config.is_file():
    parser.error(f"gripper config does not exist: {args.gripper_config}")
if args.kinematic_replay is not None and not args.kinematic_replay.is_file():
    parser.error(f"kinematic replay does not exist: {args.kinematic_replay}")
if (
    args.contact_kinematic_replay is not None
    and not args.contact_kinematic_replay.is_file()
):
    parser.error(
        "contact kinematic replay does not exist: "
        f"{args.contact_kinematic_replay}"
    )
if args.contact_kinematic_replay is not None and args.kinematic_replay is None:
    parser.error("--contact-kinematic-replay requires --kinematic-replay")
if (
    args.validated_contact_checkpoint is not None
    and not args.validated_contact_checkpoint.is_file()
):
    parser.error(
        "validated contact checkpoint does not exist: "
        f"{args.validated_contact_checkpoint}"
    )
if args.validated_contact_checkpoint is not None and not (
    args.grasp_mode == "contact-gated-retention"
    and args.physics_backend == "newton-coupled-vbd"
    and args.kinematic_replay is not None
):
    parser.error(
        "--validated-contact-checkpoint requires Newton contact-gated "
        "execution with --kinematic-replay"
    )
if args.second_fold_replay is not None and not args.second_fold_replay.is_file():
    parser.error(f"second-fold replay does not exist: {args.second_fold_replay}")
if (
    args.execute_second_fold or args.second_fold_contact_only
) and not args.second_fold_gripper_config.is_file():
    parser.error(
        "second-layer gripper config does not exist: "
        f"{args.second_fold_gripper_config}"
    )
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
    args.second_fold_correction_replay is not None
    and not args.second_fold_correction_replay.is_file()
):
    parser.error(
        "second-fold correction replay does not exist: "
        f"{args.second_fold_correction_replay}"
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
if args.cloth_resolution < 31 or args.cloth_resolution > 127:
    parser.error("--cloth-resolution must be in [31, 127]")
if args.cloth_resolution % 2 == 0:
    parser.error("--cloth-resolution must be odd so the cloth has an even node grid")
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
if args.newton_curvature_softening_stage == "s2-post-laydown" and not (
    args.newton_curvature_softening and args.execute_second_fold
):
    parser.error(
        "--newton-curvature-softening-stage s2-post-laydown requires "
        "--newton-curvature-softening --execute-second-fold"
    )
if args.newton_fold_hysteresis and not (
    args.physics_backend == "newton-coupled-vbd" and args.execute_second_fold
):
    parser.error(
        "--newton-fold-hysteresis requires Newton and --execute-second-fold"
    )
if not 90.0 <= args.newton_fold_hysteresis_angle_deg < 180.0:
    parser.error("--newton-fold-hysteresis-angle-deg must be in [90, 180)")
if args.newton_vbd_iterations < 1:
    parser.error("--newton-vbd-iterations must be positive")
if (args.newton_cantilever_calibration is None) != (
    args.newton_edge_release_calibration is None
):
    parser.error(
        "--newton-cantilever-calibration and "
        "--newton-edge-release-calibration must be supplied together"
    )
if args.newton_cantilever_calibration is not None and not IS_NEWTON_BACKEND:
    parser.error("resolution-specific Newton calibration requires the Newton backend")
if args.surface_distributed_contact_retention and args.retain_contact_evidence_only:
    parser.error(
        "select surface-distributed retention or contact-evidence-only retention, not both"
    )
if args.progressive_contact_release and not args.surface_distributed_contact_retention:
    parser.error(
        "--progressive-contact-release requires "
        "--surface-distributed-contact-retention"
    )
if args.surface_distributed_contact_retention and not (
    IS_NEWTON_BACKEND and args.grasp_mode == "contact-gated-retention"
):
    parser.error(
        "surface-distributed retention requires Newton contact-gated retention"
    )
if args.progressive_contact_release and not args.place_release:
    parser.error("--progressive-contact-release requires --place-release")
if not math.isfinite(args.newton_contact_damping) or args.newton_contact_damping < 0.0:
    parser.error("--newton-contact-damping must be finite and non-negative")
if args.newton_deep_table_support and not IS_NEWTON_BACKEND:
    parser.error("--newton-deep-table-support requires Newton")
if args.newton_analytic_table_plane and not IS_NEWTON_BACKEND:
    parser.error("--newton-analytic-table-plane requires Newton")
if args.newton_analytic_table_plane and args.newton_deep_table_support:
    parser.error("select only one Newton table-support replacement")
if args.physx_post_laydown_bend_stiffness is not None and not (
    args.physics_backend == "physx"
    and args.execute_second_fold
    and math.isfinite(args.physx_post_laydown_bend_stiffness)
    and args.physx_post_laydown_bend_stiffness > 0.0
):
    parser.error(
        "--physx-post-laydown-bend-stiffness requires PhysX S2 and a positive value"
    )
if args.physx_dynamic_friction_workaround is not None and not (
    args.physics_backend == "physx"
    and args.execute_second_fold
    and math.isfinite(args.physx_dynamic_friction_workaround)
    and args.physx_dynamic_friction_workaround > 0.0
):
    parser.error(
        "--physx-dynamic-friction-workaround requires PhysX S2 and a positive value"
    )
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
if args.left_one_way_contact_limited_model_rad is not None and not (
    args.grasp_mode in ("frictional", "contact-gated-retention")
    and args.physics_backend == "newton-coupled-vbd"
    and args.newton_coupling_mode == "one_way"
    and math.isfinite(args.left_one_way_contact_limited_model_rad)
):
    parser.error(
        "--left-one-way-contact-limited-model-rad requires finite Newton "
        "one-way frictional or contact-gated execution"
    )
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
if args.second_fold_replay is not None and not (
    args.place_release
    and args.self_contact
    and args.kinematic_replay is not None
):
    parser.error(
        "--second-fold-replay requires --place-release --self-contact "
        "--kinematic-replay"
    )
if args.second_contact_diagnostic and args.second_fold_replay is None:
    parser.error("--second-contact-diagnostic requires --second-fold-replay")
newton_second_fold_execution = (
    args.grasp_mode == "contact-gated-retention"
    and IS_NEWTON_BACKEND
    and args.second_fold_grasp_mode != "physx-attachment"
)
physx_attachment_second_fold_execution = (
    args.grasp_mode == "contact-gated-retention"
    and args.physics_backend == "physx"
    and args.second_fold_grasp_mode == "physx-attachment"
)
if args.execute_second_fold and not (
    args.second_contact_diagnostic
    and (newton_second_fold_execution or physx_attachment_second_fold_execution)
    and args.environment_count in (None, 1)
):
    parser.error(
        "--execute-second-fold requires --second-contact-diagnostic, "
        "one environment, and either Newton contact-gated retention or the "
        "PhysX vertical-grasp/attachment diagnostic combination"
    )
if args.second_fold_contact_only and not (
    args.second_contact_diagnostic
    and newton_second_fold_execution
    and args.environment_count in (None, 1)
):
    parser.error(
        "--second-fold-contact-only requires --second-contact-diagnostic, "
        "one environment, and Newton contact-gated retention"
    )
if args.second_fold_contact_only and args.execute_second_fold:
    parser.error("select contact-only or full second-fold execution, not both")
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
if args.second_fold_correction_replay is not None and not (
    args.execute_second_fold
    and args.second_fold_replay is not None
    and args.grasp_mode == "contact-gated-retention"
    and args.physics_backend == "newton-coupled-vbd"
    and args.environment_count in (None, 1)
):
    parser.error(
        "--second-fold-correction-replay requires one-environment Newton "
        "contact-gated --execute-second-fold with --second-fold-replay"
    )
if args.grasp_release_probe and args.place_release:
    parser.error("--grasp-release-probe and --place-release are separate gates")
if args.grasp_release_probe and args.grasp_mode != "contact-gated-retention":
    parser.error("--grasp-release-probe requires --grasp-mode contact-gated-retention")
if args.grasp_release_probe and not IS_NEWTON_BACKEND:
    parser.error("--grasp-release-probe requires --physics-backend newton-coupled-vbd")

material_candidate = load_material_candidate(args.material_config)
gripper_candidate = load_gripper_geometry_candidate(args.gripper_config)


def load_resolution_specific_newton_calibration() -> dict[str, object] | None:
    if args.newton_cantilever_calibration is None:
        return None

    paths = {
        "cantilever": args.newton_cantilever_calibration,
        "edge-release": args.newton_edge_release_calibration,
    }
    documents: dict[str, dict[str, object]] = {}
    for experiment, path in paths.items():
        assert path is not None
        document = json.loads(path.read_text(encoding="utf-8"))
        if (
            not isinstance(document, dict)
            or document.get("schema_version") != 1
            or document.get("record_kind") != "towel_newton_material_calibration"
            or document.get("status") != "R2_NEWTON_MATERIAL_CALIBRATION_MATCH"
            or document.get("experiment") != experiment
            or document.get("motion_authorized") is not False
            or document.get("execution_api_used") is not False
            or document.get("observation", {}).get("matched") is not True
        ):
            raise ValueError(f"invalid passing Newton {experiment} calibration: {path}")
        if document.get("cloth", {}).get("resolution") != [
            args.cloth_resolution,
            args.cloth_resolution,
        ]:
            raise ValueError(f"Newton {experiment} calibration resolution differs")
        solver = document.get("solver", {})
        if (
            int(solver.get("substeps", 0)) != 10
            or int(solver.get("iterations", 0)) != args.newton_vbd_iterations
            or not math.isclose(float(solver.get("fps", 0.0)), 240.0)
        ):
            raise ValueError(f"Newton {experiment} calibration solver differs")
        documents[experiment] = document

    cantilever_material = documents["cantilever"]["material"]
    edge_release_material = documents["edge-release"]["material"]
    if cantilever_material != edge_release_material:
        raise ValueError("Newton resolution-specific calibration materials differ")
    assert isinstance(cantilever_material, dict)
    expected = {
        "triangle_stiffness_newton_units": material_candidate.newton_triangle_stiffness_pa,
        "triangle_damping_newton_units": material_candidate.newton_triangle_damping_pa_s,
        "contact_stiffness_newton_units": args.newton_contact_stiffness,
    }
    for name, expected_value in expected.items():
        if not math.isclose(
            float(cantilever_material.get(name, math.nan)),
            expected_value,
            rel_tol=1.0e-9,
            abs_tol=1.0e-12,
        ):
            raise ValueError(f"Newton resolution-specific calibration {name} differs")

    return {
        "resolution": [args.cloth_resolution, args.cloth_resolution],
        "edge_stiffness_n_m": float(
            cantilever_material["edge_stiffness_newton_units"]
        ),
        "edge_damping_n_m_s": float(
            cantilever_material["edge_damping_newton_units"]
        ),
        "solver_iterations": args.newton_vbd_iterations,
        "cantilever": {
            "path": str(paths["cantilever"].resolve()),
            "sha256": hashlib.sha256(paths["cantilever"].read_bytes()).hexdigest(),
            "final_chord_angle_deg": documents["cantilever"]["observation"][
                "final_chord_angle_deg"
            ],
        },
        "edge_release": {
            "path": str(paths["edge-release"].resolve()),
            "sha256": hashlib.sha256(paths["edge-release"].read_bytes()).hexdigest(),
            "simulated_window_s": documents["edge-release"]["observation"][
                "simulated_window_s"
            ],
        },
    }


resolution_specific_newton_calibration = (
    load_resolution_specific_newton_calibration()
)


def load_validated_contact_checkpoint(path: Path) -> dict[str, object]:
    """Load contact particles only from a previously passing physical gate."""
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("validated contact checkpoint must be a mapping")
    attachment = document.get("attachment", {})
    checks = document.get("simulation_checks", {})
    particles = attachment.get("actual_contact_particles_by_side", {})
    if (
        not str(document.get("status", "")).startswith(
            "S1_ISAACLAB_CONTACT_GATED_NO_SLIP_RETENTION_LIFT_PASS_"
        )
        or checks.get("dual_gripper_actual_contact_constraints_created") is not True
        or checks.get("low_lift_executed") is not True
        or attachment.get("proximity_fallback_used") is not False
        or attachment.get("legacy_floating_attachment_used") is not False
        or set(particles) != {"left", "right"}
    ):
        raise ValueError("checkpoint is not a passing dual actual-contact S1 lift")
    grid_side = args.cloth_resolution + 1
    validated: dict[str, list[int]] = {}
    for side in ("left", "right"):
        indices = [int(index) for index in particles[side]]
        if len(indices) != 2 or len(set(indices)) != 2:
            raise ValueError(f"checkpoint {side} contact is not one pinch pair")
        if any(index < 0 or index >= grid_side * grid_side for index in indices):
            raise ValueError(f"checkpoint {side} contact index is outside cloth")
        first_row, first_col = divmod(indices[0], grid_side)
        second_row, second_col = divmod(indices[1], grid_side)
        if max(abs(first_row - second_row), abs(first_col - second_col)) > 1:
            raise ValueError(f"checkpoint {side} pair is not one-cell local")
        validated[side] = indices
    if list(document.get("cloth", {}).get("resolution", ())) != [
        args.cloth_resolution,
        args.cloth_resolution,
    ]:
        raise ValueError("checkpoint cloth resolution differs")
    expected_material_sha = hashlib.sha256(args.material_config.read_bytes()).hexdigest()
    expected_gripper_sha = hashlib.sha256(args.gripper_config.read_bytes()).hexdigest()
    if document.get("material_candidate", {}).get("sha256") != expected_material_sha:
        raise ValueError("checkpoint material identity differs")
    if document.get("gripper_candidate", {}).get("sha256") != expected_gripper_sha:
        raise ValueError("checkpoint gripper identity differs")
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "urdf_sha256": document.get("urdf_sha256"),
        "particles_by_side": validated,
        "strict_single_sheet_pinch_by_side": attachment.get(
            "strict_single_sheet_pinch_by_side", {}
        ),
        "achieved_gripper_model_rad": document.get("jaw_alignment", {}).get(
            "achieved_gripper_model_rad_by_environment", []
        ),
        "pad_centers_w_m": document.get("jaw_alignment", {})
        .get("contact_diagnostic", {})
        .get("pad_centers_env_0_w_m", {}),
    }


validated_contact_checkpoint = (
    load_validated_contact_checkpoint(args.validated_contact_checkpoint)
    if args.validated_contact_checkpoint is not None
    else None
)


def load_second_fold_gripper_candidate(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("second-fold gripper candidate must be a mapping")
    base = document.get("base_gripper_geometry", {})
    controls = document.get("control_contract", {})
    if (
        document.get("schema_version") != 1
        or document.get("record_kind") != "so101_gripper_s2_four_layer_candidate"
        or document.get("simulation_only") is not True
        or document.get("motion_authorized") is not False
        or controls.get("command_is_force_claim") is not False
        or controls.get("retention_only_after_four_layer_contact_gate") is not True
        or controls.get("achieved_position_is_contact_limited") is not True
    ):
        raise ValueError("second-fold gripper candidate identity is invalid")
    if Path(str(base.get("path"))).as_posix() != "config/so101_gripper_geometry.candidate.json":
        raise ValueError("second-fold gripper candidate has an unexpected base path")
    if str(base.get("sha256")) != hashlib.sha256(
        gripper_candidate.path.read_bytes()
    ).hexdigest():
        raise ValueError("second-fold gripper candidate base SHA is stale")
    project = document.get("four_layer_project_contact_target_rad", {})
    model = document.get("four_layer_model_contact_target_rad", {})
    for side in ("left", "right"):
        project_value = float(project[side])
        model_value = float(model[side])
        if not math.isfinite(project_value) or not math.isfinite(model_value):
            raise ValueError(f"invalid {side} four-layer gripper target")
        if not math.isclose(
            gripper_candidate.project_to_model(project_value),
            model_value,
            abs_tol=1.0e-6,
        ):
            raise ValueError(f"{side} four-layer project/model targets disagree")
    return document


second_fold_gripper_candidate = (
    load_second_fold_gripper_candidate(args.second_fold_gripper_config)
    if args.execute_second_fold or args.second_fold_contact_only
    else None
)

launcher = AppLauncher(args)
simulation_app = launcher.app

import torch
import warp as wp
from newton import ShapeFlags

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
from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonManager,
)
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


@wp.kernel
def capture_newton_high_curvature_rest_angles(
    positions: wp.array[wp.vec3],
    edge_indices: wp.array2d[wp.int32],
    minimum_absolute_angle: float,
    rest_angles: wp.array[wp.float32],
    captured_edges: wp.array[wp.int32],
):
    """Capture sharp achieved hinges as an experimental post-fold rest shape."""
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
    if wp.abs(angle) >= minimum_absolute_angle:
        rest_angles[tid] = angle
        captured_edges[tid] = 1


if args.disable_cubric_visual_sync:
    def _skip_cubric_visual_sync(cls: type[NewtonManager]) -> None:
        cls._cubric = None

    NewtonManager._setup_cubric_bindings = classmethod(_skip_cubric_visual_sync)


PASS_STATUS = "S1_ISAACLAB_VERTEX_PATCH_LIFT_PASS_MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
FRICTIONAL_LIFT_PASS_STATUS = (
    "S1_ISAACLAB_FRICTIONAL_JAW_LIFT_PASS_MATERIAL_CALIBRATED_NOT_FULLY_VALIDATED"
)
HIGH_RESOLUTION_CONTACT_QUALIFICATION_PASS_STATUS = (
    "S1_ISAACLAB_HIGH_RESOLUTION_FRICTIONAL_JAW_CONTACT_QUALIFICATION_PASS_"
    "MATERIAL_RESOLUTION_EXTRAPOLATED"
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
SECOND_FOLD_RAW_EXECUTED_STATUS = (
    "R2_S2_LEFT_TO_RIGHT_TWO_LAYER_RAW_FOLD_EXECUTED_"
    "CAMERA_CORRECTION_NOT_YET_APPLIED"
)
SECOND_FOLD_CORRECTED_STATUS = (
    "R2_S2_CHECKPOINT_ISOLATED_SINGLE_CAMERA_CORRECTION_EXECUTED_"
    "END_TO_END_NONDETERMINISM_NOT_PASSED"
)
PHYSICS_DT_S = 1.0 / 120.0
SELF_CONTACT_PHYSICS_DT_S = 1.0 / 240.0
ENVIRONMENT_SPACING_M = 1.0
CLOTH_SIZE_XY_M = (0.300, 0.300)
CLOTH_RESOLUTION = (args.cloth_resolution, args.cloth_resolution)
CLOTH_NODE_COUNT = (CLOTH_RESOLUTION[0] + 1) * (CLOTH_RESOLUTION[1] + 1)
MATERIAL_CALIBRATED_CLOTH_RESOLUTION = (
    tuple(resolution_specific_newton_calibration["resolution"])
    if resolution_specific_newton_calibration is not None
    else (31, 31)
)
CLOTH_RESOLUTION_MATCHES_MATERIAL_CALIBRATION = (
    CLOTH_RESOLUTION == MATERIAL_CALIBRATED_CLOTH_RESOLUTION
)
CLOTH_MASS_KG = material_candidate.mass_kg
CLOTH_DENSITY_KG_M3 = material_candidate.density_kg_m3
CLOTH_STATIC_FRICTION = material_candidate.static_friction
CLOTH_DYNAMIC_FRICTION = (
    args.physx_dynamic_friction_workaround
    if args.physx_dynamic_friction_workaround is not None
    else material_candidate.dynamic_friction
)
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
NEWTON_EDGE_STIFFNESS_N_M = (
    float(resolution_specific_newton_calibration["edge_stiffness_n_m"])
    if resolution_specific_newton_calibration is not None
    else material_candidate.newton_edge_stiffness_n_m
)
NEWTON_EDGE_DAMPING_N_M_S = (
    float(resolution_specific_newton_calibration["edge_damping_n_m_s"])
    if resolution_specific_newton_calibration is not None
    else material_candidate.newton_edge_damping_n_m_s
)
CLOTH_INITIAL_CLEARANCE_M = 0.5 * CLOTH_SURFACE_THICKNESS_M
NEWTON_CLOTH_AREAL_DENSITY_KG_M2 = CLOTH_MASS_KG / (
    CLOTH_SIZE_XY_M[0] * CLOTH_SIZE_XY_M[1]
)
PATCH_MASK_RADIUS_M = 0.016
MINIMUM_PATCH_POINT_COUNT = 4
MINIMUM_ACTUAL_CONTACT_POINT_COUNT = 1
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
# Keep the rubber on the same registered side of the fixed-jaw surface used by
# the physical-contact lift qualification.  The earlier +X placement moved the
# left pad 2.1 mm through the registered plane and removed its opposing moving-
# jaw contact.  Half of the measured 2.2 mm thickness puts the pad centre at
# -9.0 mm from the parent origin.
FIXED_JAW_PAD_CENTER_PARENT_M = (-0.0090, -0.000218121, -0.0981274)
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
REQUESTED_PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.grasp_project_rad[side][1]
    for side in ("left", "right")
}
REQUESTED_PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.grasp_model_rad(side, 1)
    for side in ("left", "right")
}
PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD = dict(
    REQUESTED_PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
)
if args.left_one_way_contact_limited_model_rad is not None:
    PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"] = (
        args.left_one_way_contact_limited_model_rad
    )
# phase_model_tensor consumes project-space gripper values.  Convert the
# effective achieved geometry back only for replay execution; the separately
# retained REQUESTED dictionaries remain the measured actuator commands.
PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.model_to_project(model_value)
    for side, model_value in PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD.items()
}
RELEASE_MODEL_GRIPPER_JOINT_POSITION_RAD = gripper_candidate.release_model_rad
SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD = {
    side: gripper_candidate.simulation_release_model_rad(side)
    for side in ("left", "right")
}
PINCH_GAP_CENTER_TCP_X_M = {"left": -0.0107, "right": -0.0084}
MAXIMUM_JAW_TARGET_PATCH_CENTER_XY_DISTANCE_M = 0.008
MAXIMUM_ATTACHMENT_POINT_TCP_DISTANCE_M = 0.030
# A two-layer pinch must involve two locally stacked particles inside one
# registered face neighborhood.  This is intentionally smaller than one
# 9.7 mm cloth cell: particle contact radii can bridge the discretization,
# while unrelated layers or an arched sheet cannot.
MAXIMUM_TWO_LAYER_CONTACT_PAIR_DISTANCE_M = 0.0085
# Newton resolves cloth contact with particles around the calibrated 31x31
# surface mesh.  Its 9.7 mm node pitch is larger than the 6 mm jaw face, so two
# opposing *actual solver contacts* can legitimately be adjacent nodes even
# though their centres do not both lie inside the face rectangle.  The strict
# part of this gate is therefore the registered fixed-pad/moving-STL contact
# labels plus one-cell topology; the metric bounds only reject nonlocal wraps.
CLOTH_NODE_SPACING_M = CLOTH_SIZE_XY_M[0] / CLOTH_RESOLUTION[0]
MAXIMUM_SINGLE_SHEET_PINCH_PAIR_DISTANCE_M = (
    math.sqrt(2.0) * CLOTH_NODE_SPACING_M + CLOTH_CONTACT_OFFSET_M
)
MAXIMUM_SINGLE_SHEET_PINCH_GRID_CHEBYSHEV_DISTANCE = 1
MAXIMUM_PINCH_PAIR_MIDPOINT_TO_GAP_CENTER_M = (
    CLOTH_NODE_SPACING_M + CLOTH_CONTACT_OFFSET_M
)
# Particle contact has a finite radius, so a valid squeezed sheet may sit a
# little beyond a registered face center.  It must not, however, wrap around
# the outside/tip of a jaw and count as an interior pinch.
MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M = 2.0 * CLOTH_CONTACT_OFFSET_M
MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M = (
    math.sqrt(2.0) * CLOTH_NODE_SPACING_M + CLOTH_CONTACT_OFFSET_M
)
MAXIMUM_PINCH_INDUCED_CLOTH_DISPLACEMENT_M = 0.005
MAXIMUM_GRIPPER_CLOSING_RESIDUAL_RAD = 0.01
MAXIMUM_ARM_TARGET_RESIDUAL_RAD = 0.03
PINCH_CLOSE_DURATION_S = 0.25
PINCH_HOLD_DURATION_S = 0.125
PINCH_TARGET_SETTLE_TIMEOUT_S = 0.50
JAW_OPEN_DURATION_S = 0.25
POST_OPEN_RELEASE_HOLD_S = 0.50
SECOND_FOLD_PINNED_LAYDOWN_HOLD_S = 0.50
SECOND_STABILIZER_MINIMUM_PHASE_DURATION_S = 0.025
SECOND_STABILIZER_MAXIMUM_COMMAND_SPEED_RAD_S = 6.0
SECOND_FOLD_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M = 0.010
# The single-arm laydown keeps the fixed pad normal about 19 degrees from the
# table. Across the finite contact patch this can leave the highest retained
# vertex near 22.5 mm even while the lowest vertex is correctly landed at the
# measured four-layer height (about 12--14 mm). This only rejects an airborne
# bundle before release; the post-release landing and shape gates decide the
# actual result.
SECOND_FOLD_MAXIMUM_LAYDOWN_PATCH_EXTENT_CLEARANCE_M = 0.024
# Each actual-contact S2 jaw carries two layers onto two supported layers. The
# measured 3 mm effective thickness therefore puts a valid four-layer landing
# near 12 mm. This pre-release gate only rejects a truly airborne bundle; the
# unchanged post-release landing and shape gates remain authoritative.
SECOND_FOLD_ACTUAL_CONTACT_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M = 0.014
# Differently oriented bimanual jaw faces use the same bounded pre-release
# extent allowance. Their minimum-clearance gate still requires touchdown.
SECOND_FOLD_BIMANUAL_MAXIMUM_LAYDOWN_PATCH_EXTENT_CLEARANCE_M = 0.024
# A hard PhysX attachment preserves the cloth vertex inside the closed jaw,
# whose measured frame is above the surrounding supported cloth.  This gate
# only rejects an airborne release; the common post-release height and settle
# gates decide whether the towel actually lands.  Lowering the TCP to satisfy
# the Newton patch threshold would put the jaw collision mesh into the table.
SECOND_FOLD_PHYSX_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M = 0.020
SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION = 0.50
SECOND_FOLD_SETTLE_TIMEOUT_S = 4.0
SECOND_FOLD_MINIMUM_FOOTPRINT_SPAN_M = 0.120
SECOND_FOLD_MAXIMUM_FOOTPRINT_SPAN_M = 0.190
SECOND_FOLD_MAXIMUM_HEIGHT_M = 0.050
SECOND_FOLD_MAXIMUM_TABLE_PENETRATION_M = 0.002
SECOND_FOLD_POST_LAYDOWN_SOFTENING_RAMP_S = 1.0
NEWTON_DEEP_TABLE_SUPPORT_DEPTH_M = 0.40
SECOND_FOLD_MAXIMUM_RELEASE_PATCH_LIFT_M = 0.015
SECOND_FOLD_MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M = 0.015
SECOND_FOLD_CORRECTION_OBSERVATION_TOLERANCE_M = 0.005
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
# These limits were established on the material-calibrated 31-element mesh.
# Keep the same physical/topological fractions when a finer mesh is used for
# real jaw-contact qualification; using the raw 13..17 indices on a 63-element
# mesh incorrectly rejects its centre fold (column 31/32/33).
RAW_MAIN_FOLD_COLUMN_FRACTION_LIMITS = (13.0 / 31.0, 17.0 / 31.0)
MINIMUM_RAW_MAIN_FOLD_COLUMN = math.ceil(
    CLOTH_RESOLUTION[0] * RAW_MAIN_FOLD_COLUMN_FRACTION_LIMITS[0]
)
MAXIMUM_RAW_MAIN_FOLD_COLUMN = math.floor(
    CLOTH_RESOLUTION[0] * RAW_MAIN_FOLD_COLUMN_FRACTION_LIMITS[1]
)
# At the calibrated low touchdown the free edge has already met the table, so
# this no longer represents a purely suspended verticality angle.  Permit at
# most one quarter of the measured 285 mm hanging length to lie sideways; a
# larger displacement indicates a collapsed panel rather than useful slack.
MAXIMUM_TOUCHDOWN_FREE_EDGE_HORIZONTAL_OFFSET_M = 0.075
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
    if args.contact_kinematic_replay is not None:
        contact_replay = json.loads(
            args.contact_kinematic_replay.read_text(encoding="utf-8")
        )
        if (
            not isinstance(contact_replay, dict)
            or (
                contact_replay.get("record_kind"),
                contact_replay.get("status"),
            )
            not in {
                (
                    "towel_suspended_gravity_full_fk_diagnostic",
                    "TOWEL_SUSPENDED_GRAVITY_FULL_FK_DIAGNOSTIC_PASS",
                ),
                (
                    "towel_suspended_gravity_contact_fk_diagnostic",
                    "TOWEL_SUSPENDED_GRAVITY_CONTACT_FK_DIAGNOSTIC_PASS",
                ),
            }
            or not suspended_gravity_replay
        ):
            raise ValueError(
                "contact replay is not a passing suspended-gravity diagnostic"
            )
        contact_bounds = contact_replay.get("towel_placement", {}).get(
            "bounds_xyxy_m", ()
        )
        if list(contact_bounds) != list(bounds):
            raise ValueError("contact and primary replay towel bounds differ")
        contact_records = {
            record.get("name"): record
            for record in contact_replay.get("selected_candidate", {}).get(
                "first_fold", ()
            )
        }
        if "first_contact" not in contact_records:
            raise ValueError("contact replay is missing first_contact")
        contact_index = next(
            index
            for index, record in enumerate(first_fold)
            if record.get("name") == "first_contact"
        )
        first_fold[contact_index] = copy.deepcopy(
            contact_records["first_contact"]
        )
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
    source["contact_kinematic_replay_path"] = (
        str(args.contact_kinematic_replay.resolve())
        if args.contact_kinematic_replay is not None
        else None
    )
    source["right_arm_kinematic_replay_path"] = (
        str(args.right_arm_kinematic_replay.resolve())
        if args.right_arm_kinematic_replay is not None
        else None
    )
    source["right_arm_kinematic_replay_through"] = (
        args.right_arm_kinematic_replay_through
    )
    if args.second_fold_replay is not None:
        second_replay = json.loads(
            args.second_fold_replay.read_text(encoding="utf-8")
        )
        single_left_second_fold_plan = (
            isinstance(second_replay, dict)
            and second_replay.get("record_kind")
            == "towel_second_fold_bimanual_task_pose_plan_only"
            and second_replay.get("status")
            == "TOWEL_SECOND_FOLD_SINGLE_LEFT_TO_RIGHT_PLAN_ONLY_PASS"
        )
        bimanual_second_fold_replay = (
            isinstance(second_replay, dict)
            and second_replay.get("record_kind")
            == "towel_second_fold_bimanual_full_fk_diagnostic"
            and second_replay.get("status")
            == "TOWEL_SECOND_FOLD_BIMANUAL_FULL_FK_DIAGNOSTIC_PASS"
        )
        if (
            not isinstance(second_replay, dict)
            or not (
                single_left_second_fold_plan
                or
                bimanual_second_fold_replay
                or (
                    second_replay.get("record_kind")
                    == "towel_second_fold_full_fk_diagnostic"
                    and second_replay.get("status")
                    == "TOWEL_SECOND_FOLD_LEFT_TO_RIGHT_FULL_FK_DIAGNOSTIC_PASS"
                )
            )
            or second_replay.get("motion_authorized") is not False
        ):
            raise ValueError("second-fold replay is not a passing motion-locked S2 diagnostic")
        selected_second = second_replay.get("selected_candidate", {})
        selected_active_arm = (
            selected_second.get("second_active_arm")
            if single_left_second_fold_plan
            else selected_second.get("active_arm")
        )
        selected_direction = (
            selected_second.get("second_direction")
            if single_left_second_fold_plan
            else selected_second.get("direction")
        )
        if (
            not isinstance(selected_second, dict)
            or (
                selected_second.get("active_arms") != ["left", "right"]
                if bimanual_second_fold_replay
                else selected_active_arm != "left"
            )
            or selected_direction != "left_to_right"
        ):
            raise ValueError("second-fold replay does not select the reviewed left-to-right topology")
        second_fold = copy.deepcopy(selected_second.get("second_fold", ()))
        if single_left_second_fold_plan:
            for record in second_fold:
                moveit_target = record.get("moveit", {}).get(
                    "target_positions_rad"
                )
                if not isinstance(moveit_target, list) or len(moveit_target) != 12:
                    raise ValueError(
                        "single-left second-fold replay is missing a 12-joint "
                        "strict MoveIt target"
                    )
                record["joint_positions_rad"] = copy.deepcopy(moveit_target)
        second_names = {record.get("name") for record in second_fold}
        required_second = (
            {
                "second_bimanual_contact",
                "second_bimanual_fold_01",
                "second_bimanual_retreat",
                "second_bimanual_reobserve_clear",
            }
            if bimanual_second_fold_replay
            else {
                "second_contact",
                "second_fold_01",
                "second_retreat",
                "second_reobserve_clear",
            }
        )
        if not required_second.issubset(second_names):
            raise ValueError("second-fold replay is missing required phases")
        transfer_records = [
            record
            for record in second_fold
            if str(record.get("name", "")).startswith(
                "second_bimanual_fold_"
                if bimanual_second_fold_replay
                else "second_fold_"
            )
        ]
        expected_transfer_names = [
            (
                f"second_bimanual_fold_{index:02d}"
                if bimanual_second_fold_replay
                else f"second_fold_{index:02d}"
            )
            for index in range(1, len(transfer_records) + 1)
        ]
        if (
            [record.get("name") for record in transfer_records]
            != expected_transfer_names
            or transfer_records[-1].get("attachment_event")
            != (
                "release_two_four_layer_u_pinches_after_dual_laydown_gate"
                if bimanual_second_fold_replay
                else "release_midpoint_bundle_after_laydown_gate"
            )
        ):
            raise ValueError("second-fold transfer phases are not contiguous")
        source["canonical_replay"]["second_fold"] = second_fold
        second_stabilizer = copy.deepcopy(
            selected_second.get("second_fold_stabilizer", ())
        )
        expected_stabilizer_names = [
            f"second_stabilizer_departure_{index:02d}_right"
            for index in range(1, 41)
        ] + [
            "second_stabilizer_contact",
            "second_stabilizer_retreat",
            "second_stabilizer_reobserve_clear",
        ]
        if (not bimanual_second_fold_replay and not single_left_second_fold_plan and (
            not isinstance(second_stabilizer, list)
            or [record.get("name") for record in second_stabilizer]
            != expected_stabilizer_names
            or second_stabilizer[-3].get("attachment_event")
            != "attach_right_stabilizer_after_actual_contact_gate"
            or second_stabilizer[-2].get("attachment_event")
            != "release_right_stabilizer_after_left_clear_gate"
        )):
            raise ValueError(
                "second-fold replay is missing the reviewed right-arm stabilizer"
            )
        source["canonical_replay"]["second_stabilizer"] = second_stabilizer
        second_handoff = copy.deepcopy(
            selected_second.get("second_fold_edge_handoff", ())
        )
        expected_handoff_names = [
            f"second_handoff_departure_{index:02d}_right"
            for index in range(1, 41)
        ] + [
            "second_handoff_contact",
            "second_handoff_hold",
            "second_handoff_release",
            "second_handoff_retreat",
            "second_handoff_reobserve_clear",
        ]
        handoff_valid = (
            isinstance(second_handoff, list)
            and [record.get("name") for record in second_handoff]
            == expected_handoff_names
            and second_handoff[-5].get("attachment_event")
            == "attach_right_upper_edge_after_opposing_layer_contact_gate"
            and second_handoff[-3].get("attachment_event")
            == "release_right_upper_edge_after_left_clear_gate"
        )
        if args.second_fold_release_mode == "right-edge-handoff" and not handoff_valid:
            raise ValueError(
                "second-fold replay is missing the reviewed right edge handoff"
            )
        if handoff_valid:
            source["canonical_replay"]["second_handoff"] = second_handoff
        source["second_fold_replay_path"] = str(
            args.second_fold_replay.resolve()
        )
        source["second_fold_active_arm"] = (
            "bimanual" if bimanual_second_fold_replay else "left"
        )
        source["second_fold_bimanual"] = bimanual_second_fold_replay
        source["second_fold_direction"] = "left_to_right"
        replay_sources = second_replay.get("sources", {})
        s1_result_key = (
            "accepted_s1_result" if single_left_second_fold_plan else "s1_result"
        )
        s1_summary_key = (
            "accepted_s1_summary" if single_left_second_fold_plan else "s1_summary"
        )
        s1_result_source = (
            replay_sources.get(s1_result_key, {})
            if isinstance(replay_sources, dict)
            else {}
        )
        s1_summary_source = (
            replay_sources.get(s1_summary_key, {})
            if isinstance(replay_sources, dict)
            else {}
        )
        s1_result_path = Path(str(s1_result_source.get("path", "")))
        s1_summary_path = Path(str(s1_summary_source.get("path", "")))
        for label, path, expected_digest in (
            ("S1 result", s1_result_path, s1_result_source.get("sha256")),
            ("S1 summary", s1_summary_path, s1_summary_source.get("sha256")),
        ):
            if (
                not path.is_file()
                or hashlib.sha256(path.read_bytes()).hexdigest()
                != str(expected_digest)
            ):
                raise ValueError(f"second-fold {label} source hash is stale")
        accepted_s1 = json.loads(s1_result_path.read_text(encoding="utf-8"))
        accepted_summary = json.loads(s1_summary_path.read_text(encoding="utf-8"))
        accepted_s1_digest = hashlib.sha256(s1_result_path.read_bytes()).hexdigest()
        accepted_digests = {
            str(value)
            for key, value in accepted_summary.get("sources", {}).items()
            if str(key).endswith("result_sha256")
        }
        accepted_nodes = accepted_s1.get("final_cloth_shape_local_m_env_0")
        if (
            not str(accepted_s1.get("status", "")).startswith(
                "S1_ISAACLAB_NOMINAL_HALF_FOLD_ACCEPTED_"
            )
            or accepted_s1.get("motion_authorized") is not False
            or accepted_summary.get("status")
            != "R2_S1_FIRST_FOLD_ACCEPTED_WITHIN_55_45"
            or accepted_s1_digest not in accepted_digests
            or not isinstance(accepted_nodes, list)
            or len(accepted_nodes) != (CLOTH_RESOLUTION[0] + 1) ** 2
            or any(
                not isinstance(node, list)
                or len(node) != 3
                or not all(math.isfinite(float(value)) for value in node)
                for node in accepted_nodes
            )
        ):
            raise ValueError("second-fold S1 checkpoint is not accepted and finite")
        source["second_fold_start_state_local_m"] = copy.deepcopy(accepted_nodes)
        source["second_fold_start_state_path"] = str(s1_result_path.resolve())
        source["second_fold_start_state_sha256"] = accepted_s1_digest
    center_x = 0.5 * (float(bounds[0]) + float(bounds[1]))
    center_y = 0.5 * (float(bounds[2]) + float(bounds[3]))
    for pose in source["rigid_proxy_pose_xyz_yaw_rad"]:
        pose[0] = center_x
        pose[1] = center_y
    return document, source


def phase(source: dict[str, object], name: str) -> dict[str, object]:
    for fold_name in (
        "first_fold",
        "second_fold",
        "second_stabilizer",
        "second_handoff",
    ):
        for record in source["canonical_replay"].get(fold_name, ()):
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
        if args.newton_deep_table_support:
            table_support = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/NewtonDeepTableSupport",
                spawn=sim_utils.CuboidCfg(
                    size=(
                        table_size[0],
                        table_size[1],
                        NEWTON_DEEP_TABLE_SUPPORT_DEPTH_M,
                    ),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        disable_gravity=True, kinematic_enabled=True
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=100.0),
                    collision_props=sim_utils.CollisionPropertiesCfg(
                        collision_enabled=True,
                        contact_offset=0.002,
                        rest_offset=0.0,
                    ),
                    physics_material=sim_utils.RigidBodyMaterialCfg(
                        static_friction=CLOTH_STATIC_FRICTION,
                        dynamic_friction=CLOTH_DYNAMIC_FRICTION,
                        restitution=0.0,
                    ),
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.30, 0.30, 0.30)
                    ),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(
                        table_pose[0],
                        table_pose[1],
                        table_top_z_m
                        - 0.5 * NEWTON_DEEP_TABLE_SUPPORT_DEPTH_M,
                    )
                ),
            )
        cloth = DeformableObjectCfg(
            prim_path="{ENV_REGEX_NS}/TowelCloth",
            spawn=sim_utils.MeshRectangleCfg(
                size=CLOTH_SIZE_XY_M,
                resolution=CLOTH_RESOLUTION,
                deformable_props=(
                    NewtonDeformableBodyPropertiesCfg()
                    if IS_NEWTON_BACKEND
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
                    if IS_NEWTON_BACKEND
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
                        if IS_NEWTON_BACKEND
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
        if IS_NEWTON_BACKEND
        and args.newton_rubber_friction is not None
        else gripper_candidate.rubber_static_friction
    )
    dynamic_friction = (
        static_friction
        if IS_NEWTON_BACKEND
        and args.newton_rubber_friction is not None
        else gripper_candidate.rubber_dynamic_friction
    )
    physics_material.CreateStaticFrictionAttr(static_friction)
    physics_material.CreateDynamicFrictionAttr(dynamic_friction)
    physics_material.CreateRestitutionAttr(gripper_candidate.rubber_restitution)
    return material


def _author_moving_jaw_material(stage: Usd.Stage, path: str) -> UsdShade.Material:
    """Author a generic plastic contact material for the unpadded moving jaw."""
    material = UsdShade.Material.Define(stage, path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.5)
    physics_material.CreateDynamicFrictionAttr(0.4)
    physics_material.CreateRestitutionAttr(0.0)
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
        if args.newton_deep_table_support:
            visual_table_mesh = stage.GetPrimAtPath(
                f"/World/envs/env_{environment_index}/Table/geometry/mesh"
            )
            support_root = stage.GetPrimAtPath(
                f"/World/envs/env_{environment_index}/NewtonDeepTableSupport"
            )
            if not visual_table_mesh.IsValid() or not support_root.IsValid():
                raise RuntimeError("deep Newton table support prims are incomplete")
            UsdPhysics.CollisionAPI.Apply(
                visual_table_mesh
            ).CreateCollisionEnabledAttr().Set(False)
            UsdGeom.Imageable(support_root).MakeInvisible()
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
            moving_jaw_material = _author_moving_jaw_material(
                stage, f"{robot_prefix}/{side}_TowelMovingJawMaterial"
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
                _author_jaw_pad(
                    stage,
                    f"{moving_body_path}/TowelMovingJawCollider",
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                    JAW_PAD_SIZE_M,
                    JAW_PAD_NORMALS_PARENT[side]["moving"],
                    moving_jaw_material,
                )


def enable_cloth_self_collision_after_pinch(environment_count: int) -> list[str]:
    """Enable deformable self-collision only after the closed-jaw gates pass."""
    if IS_NEWTON_BACKEND:
        # Newton compiles particle self-contact into the selected solver at
        # model finalization and does not expose a PhysX deformable USD owner.
        return [f"{args.physics_backend}:particle_self_contact"]
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
    if IS_NEWTON_BACKEND:
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
    """Return penetrating body-particle contacts with imported shape labels.

    Newton's soft-contact buffer also contains speculative proximity candidates
    up to ``margin + particle_radius``.  The VBD solver applies contact force
    only when ``particle_radius - signed_surface_distance > 0``; using every
    buffer entry as a grasp therefore creates false bilateral contacts.
    """
    if not IS_NEWTON_BACKEND:
        return None
    contacts = NewtonManager.get_contacts()
    model = NewtonManager.get_model()
    if contacts is None or model is None:
        return {"available": False, "reason": "missing_model_or_contact_buffer"}
    count_values = contacts.soft_contact_count.numpy().reshape(-1)
    candidate_count = int(count_values[0]) if count_values.size else 0
    shape_indices_all = contacts.soft_contact_shape.numpy().reshape(-1)[:candidate_count]
    particle_indices_all = contacts.soft_contact_particle.numpy().reshape(-1)[:candidate_count]
    body_positions_all = contacts.soft_contact_body_pos.numpy().reshape(-1, 3)[
        :candidate_count
    ]
    normals_all = contacts.soft_contact_normal.numpy().reshape(-1, 3)[:candidate_count]
    state = NewtonManager.get_state()
    particle_positions = state.particle_q.numpy()
    particle_radii = model.particle_radius.numpy().reshape(-1)
    shape_bodies = model.shape_body.numpy().reshape(-1)
    body_transforms = state.body_q.numpy().reshape(-1, 7)

    active_indices: list[int] = []
    active_penetrations_m: list[float] = []
    for contact_index, (shape_index, particle_index) in enumerate(
        zip(shape_indices_all, particle_indices_all, strict=True)
    ):
        body_index = int(shape_bodies[int(shape_index)])
        surface_position = body_positions_all[contact_index]
        if body_index >= 0:
            transform = body_transforms[body_index]
            quaternion_xyz = transform[3:6]
            local_position = surface_position
            rotated = local_position + 2.0 * np.cross(
                quaternion_xyz,
                np.cross(quaternion_xyz, local_position)
                + transform[6] * local_position,
            )
            surface_position = transform[:3] + rotated
        signed_surface_distance = float(
            np.dot(
                normals_all[contact_index],
                particle_positions[int(particle_index)] - surface_position,
            )
        )
        penetration_m = float(
            particle_radii[int(particle_index)] - signed_surface_distance
        )
        if penetration_m > 0.0:
            active_indices.append(contact_index)
            active_penetrations_m.append(penetration_m)

    shape_indices = shape_indices_all[active_indices]
    particle_indices = particle_indices_all[active_indices]
    labels = [
        model.shape_label[int(index)]
        if 0 <= int(index) < len(model.shape_label)
        else f"<shape:{int(index)}>"
        for index in shape_indices
    ]
    label_counts: dict[str, int] = {}
    particles_by_label: dict[str, list[int]] = {}
    maximum_penetration_by_label: dict[str, float] = {}
    for label, particle_index, penetration_m in zip(
        labels, particle_indices, active_penetrations_m, strict=True
    ):
        label_counts[label] = label_counts.get(label, 0) + 1
        particles_by_label.setdefault(label, []).append(int(particle_index))
        maximum_penetration_by_label[label] = max(
            maximum_penetration_by_label.get(label, 0.0), penetration_m
        )
    all_jaw_labels = {
        label: sorted(set(particles))
        for label, particles in particles_by_label.items()
        if "TowelFixedJawCollider" in label
        or "TowelMovingJawCollider" in label
        or "gripper_link" in label
        or "moving_jaw_link" in label
    }
    # The fixed side is the measured 2.2 mm rubber pad.  The opposing SO-101
    # jaw keeps its original curved STL collision; the small authored tangent
    # patch only makes that local face resolvable by the calibrated 31 x 31
    # cloth mesh.  The strict selector below still limits accepted contacts to
    # the local gap neighborhood and one-cell sheet topology.
    jaw_face_labels = {
        label: particles
        for label, particles in all_jaw_labels.items()
        if "TowelFixedJawCollider" in label
        or "TowelMovingJawCollider" in label
        or (
            "moving_jaw_link/" in label
            and "moving_jaw_so101" in label
        )
    }
    bilateral_particles_by_side: dict[str, list[int]] = {}
    for side in ("left", "right"):
        fixed_particles: set[int] = set()
        moving_particles: set[int] = set()
        for label, particles in jaw_face_labels.items():
            if f"/{side}_" not in label:
                continue
            if "TowelFixedJawCollider" in label:
                fixed_particles.update(particles)
            elif (
                "TowelMovingJawCollider" in label
                or (
                    "moving_jaw_link/" in label
                    and "moving_jaw_so101" in label
                )
            ):
                moving_particles.update(particles)
        bilateral_particles_by_side[side] = sorted(
            fixed_particles & moving_particles
        )
    return {
        "available": True,
        "soft_contact_candidate_count": candidate_count,
        "soft_contact_count": len(active_indices),
        "shape_contact_counts": label_counts,
        "shape_maximum_penetration_m": maximum_penetration_by_label,
        "jaw_particles_by_shape": jaw_face_labels,
        "imported_jaw_mesh_particles_by_shape": {
            label: particles
            for label, particles in all_jaw_labels.items()
            if label not in jaw_face_labels
        },
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


def newton_jaw_face_contact_particles(
    snapshot: dict[str, object], side: str
) -> tuple[list[int], list[int]]:
    """Return fixed-rubber-pad and moving-jaw cloth contacts for one gripper."""
    jaw_particles = snapshot.get("jaw_particles_by_shape", {})
    if not isinstance(jaw_particles, dict):
        raise RuntimeError("Newton jaw contact snapshot is malformed")
    fixed_particles: set[int] = set()
    moving_particles: set[int] = set()
    for label, indices in jaw_particles.items():
        label_text = str(label)
        if f"/{side}_" not in label_text:
            continue
        if "TowelFixedJawCollider" in label_text:
            fixed_particles.update(int(index) for index in indices)
        elif (
            "TowelMovingJawCollider" in label_text
            or (
                "moving_jaw_link/" in label_text
                and "moving_jaw_so101" in label_text
            )
        ):
            moving_particles.update(int(index) for index in indices)
    return sorted(fixed_particles), sorted(moving_particles)


def select_local_single_sheet_pinch_pair(
    snapshot: dict[str, object],
    side: str,
    nodes_w: torch.Tensor,
    fixed_face_center_w: torch.Tensor,
    moving_face_center_w: torch.Tensor,
    *,
    allowed_particles: set[int] | None = None,
) -> tuple[list[int], dict[str, object]]:
    """Select two nearby, distinct particles contacting opposing jaw faces.

    A top-down pinch puckers one physical sheet locally.  The fixed and moving
    faces therefore contact two nearby mesh locations; requiring one identical
    particle on both faces confuses a squeezed particle-radius overlap with a
    physical pinch.
    """
    fixed, moving = newton_jaw_face_contact_particles(snapshot, side)
    if allowed_particles is not None:
        fixed = [index for index in fixed if index in allowed_particles]
        moving = [index for index in moving if index in allowed_particles]
    if not fixed or not moving:
        raise RuntimeError(
            f"{side} strict pinch gate needs contact on both registered jaw faces; "
            f"fixed={fixed}, moving={moving}"
        )
    gap_delta_w = moving_face_center_w - fixed_face_center_w
    gap_length_m = float(torch.linalg.vector_norm(gap_delta_w).item())
    if gap_length_m <= 1.0e-6:
        raise RuntimeError(f"{side} registered jaw face centers are coincident")
    gap_axis_w = gap_delta_w / gap_length_m
    gap_center_w = 0.5 * (fixed_face_center_w + moving_face_center_w)
    maximum_axial_offset_m = (
        0.5 * gap_length_m + MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M
    )
    grid_side = CLOTH_RESOLUTION[0] + 1
    candidates: list[tuple[float, float, int, float, int, int]] = []
    nearest_topological_pair: (
        tuple[float, float, int, float, int, int] | None
    ) = None
    nearest_spatial_pair: tuple[float, float, int, float, int, int] | None = None
    for fixed_index in fixed:
        fixed_row, fixed_column = divmod(fixed_index, grid_side)
        for moving_index in moving:
            if fixed_index == moving_index:
                continue
            moving_row, moving_column = divmod(moving_index, grid_side)
            grid_distance = max(
                abs(fixed_row - moving_row),
                abs(fixed_column - moving_column),
            )
            if grid_distance > MAXIMUM_SINGLE_SHEET_PINCH_GRID_CHEBYSHEV_DISTANCE:
                continue
            spatial_distance = float(
                torch.linalg.vector_norm(
                    nodes_w[fixed_index] - nodes_w[moving_index]
                ).item()
            )
            midpoint = 0.5 * (nodes_w[fixed_index] + nodes_w[moving_index])
            midpoint_distance = float(
                torch.linalg.vector_norm(midpoint - gap_center_w).item()
            )
            fixed_face_center_distance_m = float(
                torch.linalg.vector_norm(
                    nodes_w[fixed_index] - fixed_face_center_w
                ).item()
            )
            moving_face_center_distance_m = float(
                torch.linalg.vector_norm(
                    nodes_w[moving_index] - moving_face_center_w
                ).item()
            )
            signed_axial_offset_m = float(
                torch.dot(midpoint - gap_center_w, gap_axis_w).item()
            )
            record = (
                midpoint_distance,
                spatial_distance,
                grid_distance,
                signed_axial_offset_m,
                fixed_index,
                moving_index,
            )
            if nearest_topological_pair is None or record < nearest_topological_pair:
                nearest_topological_pair = record
            if nearest_spatial_pair is None or (
                spatial_distance,
                midpoint_distance,
                grid_distance,
                abs(signed_axial_offset_m),
                fixed_index,
                moving_index,
            ) < (
                nearest_spatial_pair[1],
                nearest_spatial_pair[0],
                nearest_spatial_pair[2],
                abs(nearest_spatial_pair[3]),
                nearest_spatial_pair[4],
                nearest_spatial_pair[5],
            ):
                nearest_spatial_pair = record
            if spatial_distance > MAXIMUM_SINGLE_SHEET_PINCH_PAIR_DISTANCE_M:
                continue
            if midpoint_distance > MAXIMUM_PINCH_PAIR_MIDPOINT_TO_GAP_CENTER_M:
                continue
            if abs(signed_axial_offset_m) > maximum_axial_offset_m:
                continue
            if (
                fixed_face_center_distance_m
                > MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                or moving_face_center_distance_m
                > MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
            ):
                continue
            candidates.append(record)
    if not candidates:
        def pair_record(
            value: tuple[float, float, int, float, int, int] | None,
        ) -> dict[str, object] | None:
            if value is None:
                return None
            (
                midpoint_distance,
                spatial_distance,
                grid_distance,
                signed_axial_offset_m,
                fixed_index,
                moving_index,
            ) = value
            midpoint_offset = (
                0.5 * (nodes_w[fixed_index] + nodes_w[moving_index])
                - gap_center_w
            )
            return {
                "particles": [fixed_index, moving_index],
                "midpoint_to_gap_center_m": midpoint_distance,
                "midpoint_offset_from_gap_center_m": [
                    float(component) for component in midpoint_offset
                ],
                "spatial_distance_m": spatial_distance,
                "grid_chebyshev_distance": grid_distance,
                "signed_axial_offset_m": signed_axial_offset_m,
                "fixed_particle_to_fixed_face_center_m": float(
                    torch.linalg.vector_norm(
                        nodes_w[fixed_index] - fixed_face_center_w
                    ).item()
                ),
                "moving_particle_to_moving_face_center_m": float(
                    torch.linalg.vector_norm(
                        nodes_w[moving_index] - moving_face_center_w
                    ).item()
                ),
            }
        raise RuntimeError(
            f"{side} registered faces touched cloth, but not one local U-pinch; "
            + json.dumps(
                {
                    "same_particle_contacts": sorted(set(fixed) & set(moving)),
                    "nearest_topological_pair": pair_record(nearest_topological_pair),
                    "nearest_spatial_pair": pair_record(nearest_spatial_pair),
                    "maximum_pair_distance_m": (
                        MAXIMUM_SINGLE_SHEET_PINCH_PAIR_DISTANCE_M
                    ),
                    "maximum_midpoint_to_gap_center_m": (
                        MAXIMUM_PINCH_PAIR_MIDPOINT_TO_GAP_CENTER_M
                    ),
                    "registered_face_center_gap_m": gap_length_m,
                    "maximum_absolute_axial_offset_m": maximum_axial_offset_m,
                    "maximum_particle_to_assigned_face_center_m": (
                        MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                    ),
                },
                sort_keys=True,
            )
        )
    (
        midpoint_distance,
        spatial_distance,
        grid_distance,
        signed_axial_offset_m,
        fixed_index,
        moving_index,
    ) = min(candidates)
    midpoint_offset = (
        0.5 * (nodes_w[fixed_index] + nodes_w[moving_index]) - gap_center_w
    )
    return [fixed_index, moving_index], {
        "fixed_face_particles": fixed,
        "moving_face_particles": moving,
        "selected_distinct_particles": [fixed_index, moving_index],
        "selected_spatial_distance_m": spatial_distance,
        "selected_grid_chebyshev_distance": grid_distance,
        "selected_midpoint_to_gap_center_m": midpoint_distance,
        "selected_midpoint_offset_from_gap_center_m": [
            float(component) for component in midpoint_offset
        ],
        "selected_signed_axial_offset_m": signed_axial_offset_m,
        "registered_face_center_gap_m": gap_length_m,
        "maximum_absolute_axial_offset_m": maximum_axial_offset_m,
        "selected_fixed_particle_to_fixed_face_center_m": float(
            torch.linalg.vector_norm(
                nodes_w[fixed_index] - fixed_face_center_w
            ).item()
        ),
        "selected_moving_particle_to_moving_face_center_m": float(
            torch.linalg.vector_norm(
                nodes_w[moving_index] - moving_face_center_w
            ).item()
        ),
        "maximum_particle_to_assigned_face_center_m": (
            MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
        ),
        "gate": (
            "fixed_rubber_pad_and_local_actual_moving_jaw_stl_inside_"
            "registered_face_corridor"
        ),
    }


def finite_element_support_for_contact_pair(
    nodes_w: torch.Tensor,
    pair: list[int],
    gap_center_w: torch.Tensor,
) -> list[int]:
    """Return the one cloth quad represented by a proven one-cell contact pair."""
    grid_side = CLOTH_RESOLUTION[0] + 1
    coordinates = [divmod(int(index), grid_side) for index in pair]
    candidate_rows = range(
        max(row for row, _ in coordinates) - 1,
        min(row for row, _ in coordinates) + 1,
    )
    candidate_columns = range(
        max(column for _, column in coordinates) - 1,
        min(column for _, column in coordinates) + 1,
    )
    candidates: list[tuple[float, list[int]]] = []
    for row in candidate_rows:
        for column in candidate_columns:
            if not (0 <= row < grid_side - 1 and 0 <= column < grid_side - 1):
                continue
            support = [
                row * grid_side + column,
                row * grid_side + column + 1,
                (row + 1) * grid_side + column,
                (row + 1) * grid_side + column + 1,
            ]
            if not set(pair).issubset(support):
                continue
            centroid = torch.mean(nodes_w[support], dim=0)
            candidates.append(
                (
                    float(torch.linalg.vector_norm(centroid - gap_center_w).item()),
                    support,
                )
            )
    if not candidates:
        raise RuntimeError(f"contact pair {pair} does not belong to one cloth element")
    return min(candidates, key=lambda item: (item[0], item[1]))[1]


def select_local_four_layer_pinch(
    snapshot: dict[str, object],
    side: str,
    nodes_w: torch.Tensor,
    fixed_face_center_w: torch.Tensor,
    moving_face_center_w: torch.Tensor,
) -> tuple[list[int], dict[str, object]]:
    """Select a real four-layer pinch of the already folded S1 bundle.

    The S1 result contains two overlapping topology halves.  A top-down S2
    U-pinch bends both halves, so each registered jaw face must contact one
    local particle from *each* half.  This produces four distinct retained
    particles per gripper: fixed/moving for the first S1 half and
    fixed/moving for the second half.
    """
    grid_side = CLOTH_RESOLUTION[0] + 1
    half_column = grid_side // 2
    layer_diagnostics: dict[str, dict[str, object]] = {}
    selected_particles: list[int] = []
    for layer_name, first_column, last_column in (
        ("s1_first_half", 0, half_column),
        ("s1_second_half", half_column, grid_side),
    ):
        allowed = {
            index
            for index in range(int(nodes_w.shape[0]))
            if first_column <= index % grid_side < last_column
        }
        try:
            selected, diagnostic = select_local_single_sheet_pinch_pair(
                snapshot,
                side,
                nodes_w,
                fixed_face_center_w,
                moving_face_center_w,
                allowed_particles=allowed,
            )
        except RuntimeError as error:
            raise RuntimeError(
                f"{side} four-layer pinch is missing a local opposing-face "
                f"pair for {layer_name}: {error}"
            ) from error
        layer_diagnostics[layer_name] = diagnostic
        selected_particles.extend(selected)
    if len(selected_particles) != 4 or len(set(selected_particles)) != 4:
        raise RuntimeError(
            f"{side} four-layer pinch did not select four distinct particles: "
            f"{selected_particles}"
        )
    return selected_particles, {
        "gate": "four_distinct_particles_on_opposing_finite_registered_faces",
        "selected_particles_fixed_moving_per_s1_half": selected_particles,
        "selected_particle_count": len(selected_particles),
        "layers": layer_diagnostics,
    }


def isolate_newton_towel_contact_to_registered_jaw_faces() -> dict[str, object] | None:
    """Keep only the fixed rubber pad and actual moving jaw for cloth contact.

    Newton exposes particle-collision flags per shape.  Clearing only that bit
    on imported jaw meshes preserves rigid collision checks and makes the two
    registered rubber face plus the curved moving-jaw STL the sole source of
    cloth gripping contact.  The flat moving proxy remains diagnostic-only.
    """
    if not IS_NEWTON_BACKEND:
        return None
    model = NewtonManager.get_model()
    values = model.shape_flags.numpy()
    disabled: list[str] = []
    retained: list[str] = []
    diagnostic_only: list[str] = []
    for index, label in enumerate(model.shape_label):
        if "/Robot/" not in label or not any(
            token in label
            for token in (
                "gripper_link",
                "moving_jaw_link",
                "TowelFixedJawCollider",
                "TowelMovingJawCollider",
            )
        ):
            continue
        if "TowelFixedJawCollider" in label:
            # These conservative face proxies exist only to expose a precise
            # cloth-contact surface.  Letting them collide with rigid shapes
            # makes them fight the original STL on their own articulation.
            values[index] = int(values[index]) & ~int(ShapeFlags.COLLIDE_SHAPES)
            values[index] = int(values[index]) | int(ShapeFlags.COLLIDE_PARTICLES)
            retained.append(label)
            continue
        if "TowelMovingJawCollider" in label:
            values[index] = int(values[index]) & ~int(ShapeFlags.COLLIDE_SHAPES)
            values[index] = int(values[index]) | int(ShapeFlags.COLLIDE_PARTICLES)
            retained.append(label)
            continue
        if "moving_jaw_link/" in label and "moving_jaw_so101" in label:
            values[index] = int(values[index]) | int(ShapeFlags.COLLIDE_PARTICLES)
            retained.append(label)
            continue
        values[index] = int(values[index]) & ~int(ShapeFlags.COLLIDE_PARTICLES)
        disabled.append(label)
    if len([label for label in retained if "TowelFixedJawCollider" in label]) != 2:
        raise RuntimeError(
            "expected two registered fixed rubber-pad colliders, found "
            f"{sorted(retained)}"
        )
    if args.actual_jaw_mesh_contact and not disabled:
        raise RuntimeError("no imported jaw STL particle collisions were isolated")
    model.shape_flags.assign(values)
    return {
        "registered_face_labels": sorted(retained),
        "fixed_registered_faces_particle_collision_only": True,
        "moving_registered_tangent_patches_particle_collision": sorted(
            label for label in retained if "TowelMovingJawCollider" in label
        ),
        "moving_flat_proxies_diagnostic_only": sorted(diagnostic_only),
        "actual_curved_moving_jaw_stl_particle_collision": True,
        "imported_mesh_particle_collision_disabled": sorted(disabled),
        "rigid_collision_preserved": True,
    }


def set_explicit_newton_fixed_pad_friction(
    friction: float,
) -> dict[str, float] | None:
    """Set the finalized fixed-pad friction without disabling its collision."""
    if not IS_NEWTON_BACKEND:
        return None
    if not math.isfinite(friction) or friction < 0.0:
        raise ValueError("fixed-pad friction must be finite and non-negative")
    model = NewtonManager.get_model()
    values = model.shape_material_mu.numpy()
    applied: dict[str, float] = {}
    for index, label in enumerate(model.shape_label):
        if "/Robot/" in label and "TowelFixedJawCollider" in label:
            values[index] = friction
            applied[label] = float(values[index])
        elif (
            "/Robot/" in label
            and "moving_jaw_link/" in label
            and "moving_jaw_so101" in label
            and "TowelMovingJawCollider" not in label
        ):
            values[index] = 0.4
            applied[label] = float(values[index])
    if len(
        [label for label in applied if "TowelFixedJawCollider" in label]
    ) != 2:
        raise RuntimeError(
            f"expected two finalized fixed rubber pad shapes, found {sorted(applied)}"
        )
    model.shape_material_mu.assign(values)
    return applied


def apply_explicit_newton_fixed_pad_friction() -> dict[str, float] | None:
    """Apply the closed-pinch numerical friction after finalization."""
    if args.newton_rubber_friction is None:
        return None
    return set_explicit_newton_fixed_pad_friction(args.newton_rubber_friction)


def set_left_jaw_particle_collision(
    enabled: bool,
    saved_flags: dict[int, int] | None = None,
) -> tuple[dict[int, int], list[str]]:
    """Toggle only the left jaw-to-cloth contacts in the finalized Newton model."""
    model = NewtonManager.get_model()
    values = model.shape_flags.numpy()
    matched: list[str] = []
    if saved_flags is None:
        saved_flags = {}
    for index, label in enumerate(model.shape_label):
        if "/Robot/" not in label or "/left_" not in label:
            continue
        if not (
            "TowelFixedJawCollider" in label
            or "left_moving_jaw_link" in label
        ):
            continue
        matched.append(label)
        if index not in saved_flags:
            saved_flags[index] = int(values[index])
        if enabled:
            values[index] = saved_flags[index]
        else:
            values[index] = int(values[index]) & ~int(ShapeFlags.COLLIDE_PARTICLES)
    if len(matched) < 2:
        raise RuntimeError(
            "expected finalized left fixed and moving jaw particle colliders"
        )
    model.shape_flags.assign(values)
    return saved_flags, matched


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


def body_local_directions_to_world(
    body_orientations_xyzw: torch.Tensor,
    local_directions: tuple[tuple[float, float, float], ...],
) -> torch.Tensor:
    """Rotate one registered local direction for each rigid body into world space."""
    if body_orientations_xyzw.shape[1] != len(local_directions):
        raise ValueError("body count and local direction count differ")
    result = torch.empty_like(body_orientations_xyzw[..., :3])
    for environment_index in range(body_orientations_xyzw.shape[0]):
        for body_index, local_direction in enumerate(local_directions):
            orientation = body_orientations_xyzw[
                environment_index, body_index
            ].tolist()
            rotation = Gf.Rotation(
                Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
            )
            direction = rotation.TransformDir(Gf.Vec3d(*local_direction))
            result[environment_index, body_index] = torch.tensor(
                [direction[index] for index in range(3)],
                dtype=result.dtype,
                device=result.device,
            )
    return result


def jaw_pad_axes_parent(
    thin_axis_parent: tuple[float, float, float],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    """Return the authored cube's thin/u/v axes in its parent body frame."""
    rotation = Gf.Rotation(
        Gf.Vec3d(1.0, 0.0, 0.0), Gf.Vec3d(*thin_axis_parent)
    )
    axes = tuple(
        rotation.TransformDir(axis)
        for axis in (
            Gf.Vec3d(1.0, 0.0, 0.0),
            Gf.Vec3d(0.0, 1.0, 0.0),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
    )
    return tuple(
        tuple(float(value) for value in axis) for axis in axes
    )


def registered_jaw_pad_axes_parent_by_slot() -> tuple[
    tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    ...,
]:
    """Return fixed/moving authored face axes in left/right body-slot order."""
    return tuple(
        jaw_pad_axes_parent(JAW_PAD_NORMALS_PARENT[side][face])
        for side, face in (
            ("left", "fixed"),
            ("left", "moving"),
            ("right", "fixed"),
            ("right", "moving"),
        )
    )


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


def author_single_runtime_attachment(
    *,
    environment_index: int,
    side: str,
    attachment_name: str,
    frame_name: str,
    nodes_w: torch.Tensor,
    gripper_position_w: torch.Tensor,
    gripper_orientation_xyzw: torch.Tensor,
    selected_indices: list[int],
) -> str:
    """Author one hard PhysX cloth-to-gripper attachment at current positions."""
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported gripper side: {side}")
    if not selected_indices:
        raise ValueError("a runtime attachment requires at least one cloth vertex")
    stage = omni.usd.get_context().get_stage()
    gripper_path = rigid_gripper_paths(environment_index)[side]
    position = gripper_position_w.tolist()
    orientation_xyzw = gripper_orientation_xyzw.tolist()
    inverse_rotation = Gf.Rotation(
        Gf.Quatd(orientation_xyzw[3], Gf.Vec3d(*orientation_xyzw[:3]))
    ).GetInverse()
    frame_rotation = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)
    frame_local_positions = []
    for node_index in selected_indices:
        node = nodes_w[node_index].tolist()
        body_local = inverse_rotation.TransformDir(
            Gf.Vec3d(*node) - Gf.Vec3d(*position)
        )
        frame_local_positions.append(
            frame_rotation.GetInverse().TransformDir(
                body_local - Gf.Vec3d(*GRIPPER_FRAME_TRANSLATION_M)
            )
        )
    frame_path = Sdf.Path(gripper_path).AppendChild(frame_name)
    frame = UsdGeom.Xform.Define(stage, frame_path)
    frame.AddTranslateOp().Set(Gf.Vec3d(*GRIPPER_FRAME_TRANSLATION_M))
    frame.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 1.0, 0.0)))
    attachment_path = Sdf.Path(
        f"/World/envs/env_{environment_index}/Attachments/{attachment_name}"
    )
    prim = stage.DefinePrim(attachment_path, "OmniPhysicsVtxXformAttachment")
    prim.GetAttribute("omniphysics:attachmentEnabled").Set(True)
    prim.GetRelationship("omniphysics:src0").SetTargets(
        [Sdf.Path(f"/World/envs/env_{environment_index}/TowelCloth/sim_mesh")]
    )
    prim.GetRelationship("omniphysics:src1").SetTargets([frame_path])
    prim.GetAttribute("omniphysics:vtxIndicesSrc0").Set(selected_indices)
    prim.GetAttribute("omniphysics:localPositionsSrc1").Set(
        [Gf.Vec3f(value) for value in frame_local_positions]
    )
    return str(attachment_path)


def create_runtime_attachment_record(
    *, attachment_path: str, environment_index: int, side: str
) -> dict[str, object]:
    """Validate a cooked runtime attachment and return its release record."""
    stage = omni.usd.get_context().get_stage()
    scope_prim = stage.GetPrimAtPath(attachment_path)
    low_level_prims = [
        candidate
        for candidate in Usd.PrimRange(scope_prim)
        if candidate.GetTypeName() == "OmniPhysicsVtxXformAttachment"
    ]
    if len(low_level_prims) != 1:
        raise RuntimeError(
            f"expected one cooked vertex attachment under {attachment_path}, "
            f"found {len(low_level_prims)}"
        )
    prim = low_level_prims[0]
    source1_targets = prim.GetRelationship("omniphysics:src1").GetTargets()
    if len(source1_targets) != 1:
        raise RuntimeError(f"attachment has invalid src1: {prim.GetPath()}")
    target_path = source1_targets[0]
    indices = [
        int(value)
        for value in prim.GetAttribute("omniphysics:vtxIndicesSrc0").Get()
    ]
    local_positions = prim.GetAttribute("omniphysics:localPositionsSrc1").Get()
    if len(local_positions) != len(indices):
        raise RuntimeError(f"cooked patch arrays differ: {prim.GetPath()}")
    if prim.GetAttribute("omniphysics:attachmentEnabled").Get() is not True:
        raise RuntimeError(f"cooked attachment is not enabled: {prim.GetPath()}")
    target_prim = stage.GetPrimAtPath(target_path)
    rigid_ancestor = target_prim
    while rigid_ancestor and not rigid_ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_ancestor = rigid_ancestor.GetParent()
    if not rigid_ancestor or not rigid_ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"attachment target has no rigid ancestor: {target_path}")
    return {
        "environment_index": environment_index,
        "side": side,
        "attachment_path": str(prim.GetPath()),
        "gripper_path": str(rigid_ancestor.GetPath()),
        "attachment_frame_path": str(target_path),
        "selected_patch_point_count": len(indices),
        "vertex_indices": indices,
        "point_count": len(local_positions),
    }


def local_nodes(scene: InteractiveScene, cloth: object) -> torch.Tensor:
    return cloth.data.nodal_pos_w.torch - scene.env_origins[:, None, :]


def authoritative_newton_nodes_w(
    environment_count: int, device: str
) -> torch.Tensor:
    """Read the current coupled-VBD particle state after a direct constraint write."""
    state = NewtonManager.get_state()
    values = state.particle_q.numpy().copy()
    if values.shape != (environment_count * CLOTH_NODE_COUNT, 3):
        raise RuntimeError(f"unexpected Newton particle state shape: {values.shape}")
    return torch.as_tensor(values, dtype=torch.float32, device=device).reshape(
        environment_count, CLOTH_NODE_COUNT, 3
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
    require_target_reached: bool = True,
) -> float:
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
    if require_target_reached:
        require_arm_target_reached(robot, target, joint_ids, phase_name)
    print(
        f"S1_ARM_TARGET_SETTLED phase={phase_name} "
        f"hold_s={settled_step * physics_dt_s:.6f} residual_rad={residual:.6f}",
        flush=True,
    )
    return residual


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


def run_second_fold_contact_only(
    *,
    scene: InteractiveScene,
    sim: SimulationContext,
    robot: object,
    cloth: object,
    joint_ids: list[int],
    source: dict[str, object],
    environment_count: int,
    physics_dt_s: float,
    table_top_z_m: float,
    analytic_plane_filter_runtime: dict[str, object],
) -> int:
    """Qualify the S2 four-layer pinch directly from the accepted S1 state."""
    if environment_count != 1 or second_fold_gripper_candidate is None:
        raise RuntimeError("S2 contact-only requires one environment and a grip config")
    checkpoint_local = torch.tensor(
        source["second_fold_start_state_local_m"],
        dtype=cloth.data.nodal_pos_w.torch.dtype,
        device=sim.device,
    )
    checkpoint_state_w = torch.cat(
        (
            checkpoint_local.unsqueeze(0) + scene.env_origins[:, None, :],
            torch.zeros_like(checkpoint_local).unsqueeze(0),
        ),
        dim=-1,
    )
    cloth.write_nodal_state_to_sim_index(checkpoint_state_w)

    clear_record = source["canonical_replay"]["first_fold"][-1]
    if not str(clear_record.get("name", "")).endswith("reobserve_clear"):
        raise RuntimeError("accepted S1 replay does not end at the clear pose")
    clear_row = phase_model_tensor(
        source,
        clear_record,
        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
        environment_count=environment_count,
        device=sim.device,
    )
    zero_velocity = torch.zeros_like(clear_row)
    write_scripted_arm_state_and_drive_targets(
        robot,
        clear_row,
        zero_velocity,
        joint_ids,
        initialize_arm_state=True,
    )
    scene.write_data_to_sim()
    sim.step()
    scene.update(physics_dt_s)

    checkpoint_error_m = float(
        torch.max(
            torch.linalg.vector_norm(
                local_nodes(scene, cloth) - checkpoint_local.unsqueeze(0), dim=-1
            )
        ).item()
    )
    if checkpoint_error_m > 0.002:
        raise RuntimeError(
            "accepted S1 direct checkpoint drifted during initialization: "
            f"maximum_error={checkpoint_error_m:.6f} m"
        )
    if args.newton_analytic_table_plane:
        model = NewtonManager.get_model()
        shape_flags = model.shape_flags.numpy()
        plane_indices = [
            index
            for index, label in enumerate(model.shape_label)
            if "NewtonAnalyticTablePlane" in str(label)
        ]
        table_indices = [
            index
            for index, label in enumerate(model.shape_label)
            if "/Table/" in str(label)
        ]
        if not plane_indices or not table_indices:
            raise RuntimeError("S2 direct checkpoint could not resolve table supports")
        for index in table_indices:
            shape_flags[index] = int(shape_flags[index]) & ~int(
                ShapeFlags.COLLIDE_PARTICLES
            )
        for index in plane_indices:
            shape_flags[index] = int(shape_flags[index]) | int(
                ShapeFlags.COLLIDE_PARTICLES
            )
        model.shape_flags.assign(shape_flags)
        analytic_plane_filter_runtime.update(
            {
                "particle_collision_enabled_after_s1_restore": True,
                "disabled_finite_table_shape_indices": table_indices,
            }
        )

    second_active_arm = str(source.get("second_fold_active_arm", "left"))
    if second_active_arm not in {"left", "bimanual"}:
        raise RuntimeError("S2 contact-only requires the reviewed arm topology")
    bimanual = second_active_arm == "bimanual"
    departure_prefix = (
        "second_bimanual_departure" if bimanual else "second_departure"
    )
    precontact_prefix = (
        "second_bimanual_precontact" if bimanual else "second_precontact"
    )
    contact_name = "second_bimanual_contact" if bimanual else "second_contact"
    active_arms = ("left", "right") if bimanual else ("left",)
    approach_records = [
        record
        for record in source["canonical_replay"]["second_fold"]
        if record["name"].startswith(departure_prefix)
        or record["name"].startswith(precontact_prefix)
        or record["name"] == contact_name
    ]
    if not approach_records or approach_records[-1]["name"] != contact_name:
        raise RuntimeError("S2 contact-only approach sequence is incomplete")
    # Contact qualification must reproduce the same continuous approach used
    # by full S2 execution.  Teleporting the open grippers to the terminal
    # pose can place jaw collision geometry through the supported bundle and
    # manufacture a different, locally lifted cloth shape before closure.
    # That was especially severe for the diagonally registered right jaw.
    departure_records = [
        record
        for record in approach_records
        if record["name"].startswith(departure_prefix)
    ]
    contact_descent_records = [
        record
        for record in approach_records
        if record["name"].startswith(precontact_prefix)
        or record["name"] == contact_name
    ]
    if not departure_records or not contact_descent_records:
        raise RuntimeError("S2 contact-only staging/descent sequence is incomplete")
    # The last departure pose is FK/collision-qualified and remains 50 mm
    # above the towel.  Initializing the robot there cannot alter the contact
    # event, while replaying the preceding 80 collision-free air waypoints at
    # 63x63 cloth resolution adds minutes without additional evidence.
    staging_row = phase_model_tensor(
        source,
        departure_records[-1],
        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
        environment_count=environment_count,
        device=sim.device,
    )
    write_scripted_arm_state_and_drive_targets(
        robot,
        staging_row,
        zero_velocity,
        joint_ids,
        initialize_arm_state=True,
    )
    scene.write_data_to_sim()
    sim.step()
    scene.update(physics_dt_s)
    staging_cloth_displacement_m = float(
        torch.max(
            torch.linalg.vector_norm(
                local_nodes(scene, cloth) - checkpoint_local.unsqueeze(0), dim=-1
            )
        ).item()
    )
    if staging_cloth_displacement_m > 0.002:
        raise RuntimeError(
            "collision-free S2 staging pose perturbed the accepted S1 cloth: "
            f"maximum_error={staging_cloth_displacement_m:.6f} m"
        )
    current_row = staging_row
    approach_phase_steps = max(2, round(0.15 / physics_dt_s))
    nodes_before_approach = local_nodes(scene, cloth).clone()
    maximum_approach_cloth_displacement_m = 0.0
    for approach_record in contact_descent_records:
        target_row = phase_model_tensor(
            source,
            approach_record,
            gripper_project_positions_rad={"left": 0.0, "right": 0.0},
            environment_count=environment_count,
            device=sim.device,
        )
        for step in range(1, approach_phase_steps + 1):
            alpha = step / approach_phase_steps
            target = current_row + alpha * (target_row - current_row)
            write_scripted_arm_state_and_drive_targets(
                robot, target, zero_velocity, joint_ids
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                raise RuntimeError(
                    "cloth produced non-finite nodes during S2 contact approach "
                    f"{approach_record['name']}"
                )
            maximum_approach_cloth_displacement_m = max(
                maximum_approach_cloth_displacement_m,
                float(
                    torch.max(
                        torch.linalg.vector_norm(
                            local_nodes(scene, cloth) - nodes_before_approach,
                            dim=-1,
                        )
                    ).item()
                ),
            )
        current_row = target_row
    for _ in range(max(2, round(0.50 / physics_dt_s))):
        write_scripted_arm_state_and_drive_targets(
            robot, current_row, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)

    project_targets = {
        side: float(
            second_fold_gripper_candidate["four_layer_project_contact_target_rad"][
                side
            ]
        )
        for side in ("left", "right")
    }
    model_targets = {
        side: float(
            second_fold_gripper_candidate["four_layer_model_contact_target_rad"][side]
        )
        for side in ("left", "right")
    }
    pinch_row = current_row.clone()
    for arm_index, side in enumerate(("left", "right")):
        if side in active_arms:
            pinch_row[:, GRIPPER_JOINT_INDICES[arm_index]] = model_targets[side]
    for step in range(1, max(2, round(PINCH_CLOSE_DURATION_S / physics_dt_s)) + 1):
        alpha = step / max(2, round(PINCH_CLOSE_DURATION_S / physics_dt_s))
        target = current_row + alpha * (pinch_row - current_row)
        write_scripted_arm_state_and_drive_targets(
            robot, target, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)
    for _ in range(max(2, round(PINCH_HOLD_DURATION_S / physics_dt_s))):
        write_scripted_arm_state_and_drive_targets(
            robot, pinch_row, zero_velocity, joint_ids
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(physics_dt_s)

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
        raise RuntimeError(f"unexpected S2 jaw body map: {jaw_body_names}")
    jaw_positions = robot.data.body_pos_w.torch[:, jaw_body_ids]
    jaw_orientations = robot.data.body_quat_w.torch[:, jaw_body_ids]
    centers = body_local_points_to_world(
        jaw_positions,
        jaw_orientations,
        (
            FIXED_JAW_PAD_CENTER_PARENT_M,
            MOVING_JAW_PAD_CENTER_PARENT_M,
            FIXED_JAW_PAD_CENTER_PARENT_M,
            MOVING_JAW_PAD_CENTER_PARENT_M,
        ),
    )
    registered_axes_parent_by_slot = registered_jaw_pad_axes_parent_by_slot()
    tangent_u = body_local_directions_to_world(
        jaw_orientations,
        tuple(axes[1] for axes in registered_axes_parent_by_slot),
    )
    tangent_v = body_local_directions_to_world(
        jaw_orientations,
        tuple(axes[2] for axes in registered_axes_parent_by_slot),
    )
    nodes_w = cloth.data.nodal_pos_w.torch[0]
    snapshot = newton_soft_contact_snapshot()
    if snapshot is None:
        raise RuntimeError("Newton contact snapshot is unavailable")
    contacts_by_arm: dict[str, object] = {}
    continuous_surface_diagnostics_by_arm: dict[str, object] = {}
    continuous_surface_errors_by_arm: dict[str, str] = {}
    support_vertices_by_arm: dict[str, list[int]] = {}
    solver_particles_by_arm: dict[str, object] = {}
    gate_errors_by_arm: dict[str, str] = {}
    frame_directions_by_arm: dict[str, object] = {}
    print(
        "S2_FOUR_LAYER_CONTACT_FRAME_DIAGNOSTIC "
        + json.dumps(
            {
                "registered_face_centers_w_m": {
                    name: [float(value) for value in centers[0, index]]
                    for index, name in enumerate(
                        ("left_fixed", "left_moving", "right_fixed", "right_moving")
                    )
                },
                "cloth_bounds_w_m": {
                    "minimum": torch.min(nodes_w, dim=0).values.tolist(),
                    "maximum": torch.max(nodes_w, dim=0).values.tolist(),
                },
                "table_top_z_m": table_top_z_m,
            },
            sort_keys=True,
        ),
        flush=True,
    )
    for arm_index, side in enumerate(("left", "right")):
        if side not in active_arms:
            continue
        fixed_slot = 2 * arm_index
        moving_slot = fixed_slot + 1
        gap_axis = centers[0, moving_slot] - centers[0, fixed_slot]
        gap_axis = gap_axis / torch.linalg.vector_norm(gap_axis)
        frame_directions_by_arm[side] = {
            "fixed_inward_normal": [float(value) for value in gap_axis],
            "fixed_tangent_u": [
                float(value) for value in tangent_u[0, fixed_slot]
            ],
            "fixed_tangent_v": [
                float(value) for value in tangent_v[0, fixed_slot]
            ],
            "moving_inward_normal": [float(-value) for value in gap_axis],
            "moving_tangent_u": [
                float(value) for value in tangent_u[0, moving_slot]
            ],
            "moving_tangent_v": [
                float(value) for value in tangent_v[0, moving_slot]
            ],
        }
        fixed_particles, moving_particles = newton_jaw_face_contact_particles(
            snapshot, side
        )
        solver_particles_by_arm[side] = {
            "fixed": fixed_particles,
            "moving": moving_particles,
        }
        try:
            selected_particles, physical_pinch = select_local_four_layer_pinch(
                snapshot,
                side,
                nodes_w,
                centers[0, fixed_slot],
                centers[0, moving_slot],
            )
        except RuntimeError as error:
            gate_errors_by_arm[side] = str(error)
        else:
            contacts_by_arm[side] = physical_pinch
            support_vertices_by_arm[side] = selected_particles

        # The moving SO-101 jaw is curved, so this finite tangent-plane test
        # is useful only as a sub-particle diagnostic. The authoritative gate
        # above uses Newton contacts on the actual moving-jaw STL and the
        # fixed 2.2 mm rubber-pad collider for both S1 topology halves.
        try:
            surface_pinch = select_continuous_four_layer_pinch(
                nodes_w.detach().cpu().numpy(),
                grid_side=CLOTH_RESOLUTION[0] + 1,
                fixed_face=RegisteredFaceFrame(
                    center_m=tuple(float(value) for value in centers[0, fixed_slot]),
                    inward_normal=tuple(float(value) for value in gap_axis),
                    tangent_u=tuple(float(value) for value in tangent_u[0, fixed_slot]),
                    tangent_v=tuple(float(value) for value in tangent_v[0, fixed_slot]),
                ),
                moving_face=RegisteredFaceFrame(
                    center_m=tuple(float(value) for value in centers[0, moving_slot]),
                    inward_normal=tuple(float(-value) for value in gap_axis),
                    tangent_u=tuple(float(value) for value in tangent_u[0, moving_slot]),
                    tangent_v=tuple(float(value) for value in tangent_v[0, moving_slot]),
                ),
                face_size_m=args.jaw_pad_face_size_mm * 0.001,
                inward_depth_m=CLOTH_CONTACT_OFFSET_M,
                backside_tolerance_m=MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M,
            )
        except FourLayerContactError as error:
            continuous_surface_errors_by_arm[side] = str(error)
        else:
            continuous_surface_diagnostics_by_arm[side] = surface_pinch.to_dict()

    if gate_errors_by_arm:
        failure = {
            "schema_version": 1,
            "record_kind": "towel_s2_four_layer_actual_contact_result",
            "status": "R2_S2_FOUR_LAYER_ACTUAL_CONTACT_FAIL",
            "motion_authorized": False,
            "motion_commands": 0,
            "accepted_s1_checkpoint": {
                "path": source["second_fold_start_state_path"],
                "sha256": source["second_fold_start_state_sha256"],
                "fresh_s1_prelude_executed": False,
            },
            "gate_errors_by_arm": gate_errors_by_arm,
            "registered_face_centers_w_m": {
                name: [float(value) for value in centers[0, index]]
                for index, name in enumerate(
                    ("left_fixed", "left_moving", "right_fixed", "right_moving")
                )
            },
            "registered_face_directions_w": frame_directions_by_arm,
            "cloth_bounds_w_m": {
                "minimum": torch.min(nodes_w, dim=0).values.tolist(),
                "maximum": torch.max(nodes_w, dim=0).values.tolist(),
            },
            "maximum_cloth_displacement_during_continuous_approach_m": (
                maximum_approach_cloth_displacement_m
            ),
            "maximum_cloth_displacement_at_collision_free_staging_m": (
                staging_cloth_displacement_m
            ),
            "solver_particle_contacts_by_arm": solver_particles_by_arm,
            "continuous_surface_diagnostics_by_arm": (
                continuous_surface_diagnostics_by_arm
            ),
            "continuous_surface_errors_by_arm": continuous_surface_errors_by_arm,
            # Failure-only diagnostic used to solve the jaw-pose correction
            # offline.  It prevents repeated Isaac runs with guessed offsets.
            "cloth_shape_w_m": nodes_w.detach().cpu().tolist(),
            "contacts_that_passed_by_arm": contacts_by_arm,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(failure, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        raise RuntimeError(
            "S2 four-layer actual-contact gate failed: "
            + json.dumps(gate_errors_by_arm, sort_keys=True)
        )

    achieved_model_rad = {
        side: float(
            robot.data.joint_pos.torch[
                0, joint_ids[GRIPPER_JOINT_INDICES[arm_index]]
            ].item()
        )
        for arm_index, side in enumerate(("left", "right"))
        if side in active_arms
    }
    result = {
        "schema_version": 1,
        "record_kind": "towel_s2_four_layer_actual_contact_result",
        "status": "R2_S2_FOUR_LAYER_ACTUAL_CONTACT_PASS",
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "cloth_resolution": list(CLOTH_RESOLUTION),
        "material_resolution_matches_calibration": (
            CLOTH_RESOLUTION_MATCHES_MATERIAL_CALIBRATION
        ),
        "accepted_s1_checkpoint": {
            "path": source["second_fold_start_state_path"],
            "sha256": source["second_fold_start_state_sha256"],
            "maximum_initialization_error_m": checkpoint_error_m,
            "fresh_s1_prelude_executed": False,
        },
        "maximum_cloth_displacement_during_continuous_approach_m": (
            maximum_approach_cloth_displacement_m
        ),
        "maximum_cloth_displacement_at_collision_free_staging_m": (
            staging_cloth_displacement_m
        ),
        "gripper": {
            "candidate_path": str(args.second_fold_gripper_config.resolve()),
            "commanded_project_rad_by_arm": project_targets,
            "commanded_model_rad_by_arm": model_targets,
            "achieved_model_rad_by_arm": achieved_model_rad,
            "command_is_force_claim": False,
            "fixed_jaw_rubber_pad_thickness_m": (
                gripper_candidate.fixed_jaw_rubber_pad_thickness_m
            ),
        },
        "contact_gate": {
            "kind": "actual_fixed_rubber_and_curved_moving_stl_four_layer_contacts",
            "physical_particle_contacts_per_arm": 4,
            "contacts_by_arm": contacts_by_arm,
            "finite_element_support_vertices_by_arm": support_vertices_by_arm,
            "solver_particle_contacts_by_arm": solver_particles_by_arm,
            "solver_contact_required_on_both_physical_jaw_colliders_and_s1_halves": True,
            "continuous_tangent_plane_diagnostics_by_arm": (
                continuous_surface_diagnostics_by_arm
            ),
            "continuous_tangent_plane_errors_by_arm": (
                continuous_surface_errors_by_arm
            ),
            "face_size_m": args.jaw_pad_face_size_mm * 0.001,
            "contact_inward_depth_m": CLOTH_CONTACT_OFFSET_M,
            "backside_tolerance_m": MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M,
        },
        "table_top_z_m": table_top_z_m,
        "newton_analytic_table_plane": analytic_plane_filter_runtime,
        "second_fold_transport_executed": False,
        "second_fold_active_arm": second_active_arm,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        "R2_S2_FOUR_LAYER_ACTUAL_CONTACT_PASS "
        + json.dumps(
            {
                "support_vertices_by_arm": support_vertices_by_arm,
                "achieved_model_rad_by_arm": achieved_model_rad,
                "output": str(args.output),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


def run() -> int:
    manifest, source = load_manifest(args.manifest)
    if args.urdf_override is not None:
        source = copy.deepcopy(source)
        source["urdf_path"] = args.urdf_override.resolve()
        source["urdf_sha256"] = hashlib.sha256(
            args.urdf_override.read_bytes()
        ).hexdigest()
    if (
        validated_contact_checkpoint is not None
        and validated_contact_checkpoint["urdf_sha256"] != source["urdf_sha256"]
    ):
        raise ValueError("validated contact checkpoint URDF identity differs")
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
    second_fold_correction_replay = None
    second_fold_correction_records: list[dict[str, object]] = []
    second_fold_correction_checkpoint_local = None
    second_fold_correction_checkpoint_source: dict[str, object] | None = None
    if args.second_fold_correction_replay is not None:
        second_fold_correction_replay = json.loads(
            args.second_fold_correction_replay.read_text(encoding="utf-8")
        )
        if (
            not isinstance(second_fold_correction_replay, dict)
            or second_fold_correction_replay.get("record_kind")
            != "towel_second_fold_correction_full_fk_diagnostic"
            or second_fold_correction_replay.get("status")
            != "TOWEL_SECOND_FOLD_CORRECTION_CONDITIONAL_STRICT_MOVEIT_PASS"
            or second_fold_correction_replay.get("motion_authorized") is not False
            or second_fold_correction_replay.get("strict_moveit_validated") is not True
            or second_fold_correction_replay.get("strict_moveit_validation", {}).get(
                "unapproved_contact_count"
            )
            != 0
        ):
            raise ValueError(
                "second-fold correction is not a passing motion-locked strict MoveIt plan"
            )
        correction_urdf = second_fold_correction_replay.get("sources", {}).get(
            "urdf", {}
        )
        if correction_urdf.get("sha256") != source["urdf_sha256"]:
            raise ValueError(
                "second-fold correction and Isaac manifest use different URDFs"
            )
        second_fold_correction_records = copy.deepcopy(
            second_fold_correction_replay.get("phases", [])
        )
        second_correction_names = [
            record.get("name") for record in second_fold_correction_records
        ]
        expected_alignment = [
            f"second_correction_pad_yaw_align_{index:02d}"
            for index in range(1, 9)
        ] + [
            f"second_correction_pad_tilt_align_{index:02d}"
            for index in range(1, 6)
        ] + ["second_correction_pad_align"]
        expected_tail = [
            "second_correction_contact",
            "second_correction_lift",
            "second_correction_translate",
            "second_correction_laydown",
            "second_correction_retreat",
            "second_correction_reobserve_clear",
        ]
        correction_contact_mode = second_fold_correction_replay.get(
            "correction_plan", {}
        ).get("contact_mode")
        correction_is_open_jaw_edge_push = (
            correction_contact_mode
            == "exposed_overhang_open_jaw_fixed_pad_edge_push"
        )
        attachment_contract_valid = False
        if len(second_fold_correction_records) == 60:
            attachment_contract_valid = (
                second_fold_correction_records[54].get("attachment_event") is None
                and second_fold_correction_records[57].get("attachment_event") is None
                if correction_is_open_jaw_edge_push
                else second_fold_correction_records[54].get("attachment_event")
                == "attach_observed_upper_edge_after_contact_gate"
                and second_fold_correction_records[57].get("attachment_event")
                == "release_upper_edge_after_laydown_gate"
            )
        if (
            len(second_fold_correction_records) != 60
            or len(set(second_correction_names)) != len(second_correction_names)
            or second_correction_names[:40]
            != [
                f"second_correction_departure_{index:02d}_right"
                for index in range(1, 41)
            ]
            or second_correction_names[40:54] != expected_alignment
            or second_correction_names[54:] != expected_tail
            or not attachment_contract_valid
        ):
            raise ValueError("second-fold correction phase sequence is incomplete")
        second_correction_plan = second_fold_correction_replay.get(
            "correction_plan", {}
        )
        if (
            not isinstance(second_correction_plan, dict)
            or second_correction_plan.get("arm") != "right"
            or second_correction_plan.get("required") is not True
            or second_correction_plan.get("reobserve_after_step") is not True
        ):
            raise ValueError("second-fold correction plan is not a bounded right-arm step")
        raw_source = second_fold_correction_replay.get("sources", {}).get(
            "raw_result", {}
        )
        if not isinstance(raw_source, dict):
            raise ValueError("second-fold correction raw source is invalid")
        raw_source_path = Path(str(raw_source.get("path", "")))
        if (
            not raw_source_path.is_file()
            or hashlib.sha256(raw_source_path.read_bytes()).hexdigest()
            != str(raw_source.get("sha256"))
        ):
            raise ValueError("second-fold correction raw checkpoint source is stale")
        raw_source_document = json.loads(raw_source_path.read_text(encoding="utf-8"))
        raw_source_second_fold = raw_source_document.get("second_fold", {})
        raw_source_nodes = raw_source_document.get(
            "final_cloth_shape_local_m_env_0"
        )
        if (
            raw_source_document.get("record_kind")
            != "towel_isaac_s1_vertex_patch_place_release_result"
            or raw_source_document.get("motion_authorized") is not False
            or not isinstance(raw_source_second_fold, dict)
            or raw_source_second_fold.get("status")
            != SECOND_FOLD_RAW_EXECUTED_STATUS
            or not isinstance(raw_source_nodes, list)
            or len(raw_source_nodes) != (CLOTH_RESOLUTION[0] + 1) ** 2
        ):
            raise ValueError("second-fold correction source is not a raw S2 checkpoint")
        second_fold_correction_checkpoint_local = raw_source_nodes
        second_fold_correction_checkpoint_source = {
            "path": str(raw_source_path.resolve()),
            "sha256": str(raw_source.get("sha256")),
            "node_count": len(raw_source_nodes),
        }
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
    physx_attachment_pipeline_used = (
        args.grasp_mode == "contact-gated-retention"
        and args.physics_backend == "physx"
        and args.second_fold_grasp_mode == "physx-attachment"
    )
    legacy_attachment_used = args.grasp_mode == "legacy-attachment"
    contact_gated_retention_used = (
        args.grasp_mode == "contact-gated-retention"
        and IS_NEWTON_BACKEND
    )
    scripted_attachment_used = (
        legacy_attachment_used or physx_attachment_pipeline_used
    )
    newton_state_retention_used = contact_gated_retention_used
    vertical_grasp_used = not legacy_attachment_used
    print(
        f"S1_VERTEX_PATCH_START environments={environment_count} "
        "gripper=q0_aligned_drive_not_state_overwrite "
        f"grasp_mode={args.grasp_mode} motion_commands=0",
        flush=True,
    )
    if IS_NEWTON_BACKEND:
        @configclass
        class TowelNewtonCfg(NewtonCfg):
            model_cfg: NewtonModelCfg | None = None

        solver_cfg = CoupledMJWarpVBDSolverCfg(
            rigid_solver_cfg=MJWarpSolverCfg(
                njmax=128,
                nconmax=256,
                ls_iterations=20,
                cone="pyramidal",
                integrator="implicitfast",
                ccd_iterations=100,
            ),
            soft_solver_cfg=VBDSolverCfg(
                iterations=args.newton_vbd_iterations,
                integrate_with_external_rigid_solver=True,
                particle_enable_self_contact=args.self_contact,
                particle_self_contact_radius=CLOTH_CONTACT_OFFSET_M,
                particle_self_contact_margin=2.0 * CLOTH_CONTACT_OFFSET_M,
                # Four folded layers can overflow the defaults (32/64),
                # silently dropping cloth self-contact candidates during
                # release.
                particle_vertex_contact_buffer_size=(
                    128
                    if args.execute_second_fold or args.second_fold_contact_only
                    else 32
                ),
                particle_edge_contact_buffer_size=(
                    256
                    if args.execute_second_fold or args.second_fold_contact_only
                    else 64
                ),
                particle_collision_detection_interval=(
                    1 if args.self_contact else -1
                ),
                particle_topological_contact_filter_threshold=(
                    SELF_CONTACT_TOPOLOGY_NEIGHBORHOOD
                ),
            ),
            coupling_mode=args.newton_coupling_mode,
        )
        model_cfg = NewtonModelCfg(
            soft_contact_ke=args.newton_contact_stiffness,
            soft_contact_kd=args.newton_contact_damping,
            soft_contact_mu=CLOTH_STATIC_FRICTION,
            shape_material_ke=args.newton_contact_stiffness,
            shape_material_kd=args.newton_contact_damping,
            # Preserve per-shape friction: measured towel/table candidate,
            # generic plastic moving jaw, and fixed-jaw rubber material.
            shape_material_mu=None,
        )
        physics_cfg = TowelNewtonCfg(
            solver_cfg=solver_cfg,
            model_cfg=model_cfg,
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
        curvature_softening_runtime["enabled"] = (
            args.newton_curvature_softening_stage == "global"
        )
        curvature_softening_runtime["stage"] = (
            args.newton_curvature_softening_stage
        )
        curvature_softening_runtime["ramp_fraction"] = (
            1.0 if args.newton_curvature_softening_stage == "global" else 0.0
        )
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
                if not curvature_softening_runtime["enabled"]:
                    return
                ramp_fraction = min(
                    1.0,
                    float(curvature_softening_runtime["ramp_fraction"])
                    + physics_dt_s
                    / SECOND_FOLD_POST_LAYDOWN_SOFTENING_RAMP_S,
                )
                curvature_softening_runtime["ramp_fraction"] = ramp_fraction
                effective_softened_stiffness = (
                    NEWTON_EDGE_STIFFNESS_N_M
                    + ramp_fraction
                    * (
                        args.newton_softened_edge_stiffness
                        - NEWTON_EDGE_STIFFNESS_N_M
                    )
                )
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
                        effective_softened_stiffness,
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
    analytic_plane_filter_runtime: dict[str, object] = {}
    if args.newton_analytic_table_plane:
        def filter_robot_from_analytic_table_plane(_event: object) -> None:
            builder = NewtonManager._builder
            plane_index = builder.add_shape_plane(
                plane=(0.0, 0.0, 1.0, -table_top_z_m_for_contact_gate),
                width=0.0,
                length=0.0,
                body=-1,
                cfg=builder.ShapeConfig(
                    ke=args.newton_contact_stiffness,
                    kd=args.newton_contact_damping,
                    mu=CLOTH_STATIC_FRICTION,
                    has_shape_collision=False,
                    has_particle_collision=False,
                    is_visible=False,
                ),
                label="/World/NewtonAnalyticTablePlane",
            )
            analytic_plane_filter_runtime.update(
                {
                    "plane_shape_indices": [plane_index],
                    "rigid_shape_collision_disabled": True,
                    "particle_collision_enabled_at_spawn": False,
                    "support_switch_stage": "accepted_s1_checkpoint_restored",
                    "authored_after_imported_scene_shapes": True,
                }
            )

        NewtonManager.register_callback(
            filter_robot_from_analytic_table_plane,
            PhysicsEvent.MODEL_INIT,
            name="towel_analytic_table_plane_robot_filter",
        )
    simulation_app.update()
    sim.reset()
    strict_jaw_face_collision_scope = (
        isolate_newton_towel_contact_to_registered_jaw_faces()
    )
    if strict_jaw_face_collision_scope is not None:
        print(
            "STRICT_JAW_FACE_PARTICLE_COLLISION_SCOPE "
            + json.dumps(strict_jaw_face_collision_scope, sort_keys=True),
            flush=True,
        )
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
    if args.second_fold_contact_only:
        return run_second_fold_contact_only(
            scene=scene,
            sim=sim,
            robot=robot,
            cloth=cloth,
            joint_ids=joint_ids,
            source=source,
            environment_count=environment_count,
            physics_dt_s=physics_dt_s,
            table_top_z_m=table_top_z_m_for_contact_gate,
            analytic_plane_filter_runtime=analytic_plane_filter_runtime,
        )
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
    free_node_mask = torch.ones(CLOTH_NODE_COUNT, dtype=torch.bool, device=sim.device)

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
    # Jaw convergence and arm convergence are separate observables.  The
    # planned descent deliberately continues past first contact, so the arm is
    # expected to stop short under contact load.  The finite-face pinch gate
    # below validates that stop and adopts the achieved arm pose for lift.
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
    closing_contact_residual_rad = (
        achieved_gripper_model_rad - target_gripper_model_rad
    )
    newton_contact_snapshot = newton_soft_contact_snapshot()
    actual_bilateral_particles_by_side: dict[str, list[int]] = {}
    retained_finite_element_support_by_side: dict[str, list[int]] = {}
    strict_single_sheet_pinch_by_side: dict[str, dict[str, object]] = {}
    validated_contact_checkpoint_used = False
    if newton_contact_snapshot is not None:
        print(
            "S1_NEWTON_ACTUAL_CONTACTS "
            + json.dumps(newton_contact_snapshot, sort_keys=True),
            flush=True,
        )
    if contact_gated_retention_used:
        if not IS_NEWTON_BACKEND:
            raise RuntimeError(
                "contact-gated retention requires Newton's inspectable actual-contact buffer"
            )
        if environment_count != 1:
            raise RuntimeError(
                "strict actual-contact particle indexing is currently validated only "
                "with --environment-count 1"
            )
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
    jaw_pad_axes_parent_by_slot = registered_jaw_pad_axes_parent_by_slot()
    jaw_face_thin_axes_before_w = body_local_directions_to_world(
        robot.data.body_quat_w.torch[:, jaw_body_ids],
        tuple(axes[0] for axes in jaw_pad_axes_parent_by_slot),
    )
    jaw_face_tangent_u_before_w = body_local_directions_to_world(
        robot.data.body_quat_w.torch[:, jaw_body_ids],
        tuple(axes[1] for axes in jaw_pad_axes_parent_by_slot),
    )
    jaw_face_tangent_v_before_w = body_local_directions_to_world(
        robot.data.body_quat_w.torch[:, jaw_body_ids],
        tuple(axes[2] for axes in jaw_pad_axes_parent_by_slot),
    )
    # The thin-axis sign only describes the authored cube rotation.  For the
    # contact prism, each finite face must point into the physical gap toward
    # its opposing face, independent of that sign.
    jaw_face_inward_normals_before_w = torch.empty_like(
        jaw_face_thin_axes_before_w
    )
    for side_index in range(2):
        fixed_index = 2 * side_index
        moving_index = fixed_index + 1
        gap_axis = (
            jaw_pad_centers_before_w[:, moving_index]
            - jaw_pad_centers_before_w[:, fixed_index]
        )
        gap_axis = gap_axis / torch.linalg.vector_norm(
            gap_axis, dim=-1, keepdim=True
        )
        jaw_face_inward_normals_before_w[:, fixed_index] = gap_axis
        jaw_face_inward_normals_before_w[:, moving_index] = -gap_axis
    print(
        "S1_REGISTERED_JAW_FACE_CENTERS "
        + json.dumps(
            {
                name: [float(value) for value in jaw_pad_centers_before_w[0, index]]
                for index, name in enumerate(
                    ("left_fixed", "left_moving", "right_fixed", "right_moving")
                )
            },
            sort_keys=True,
        ),
        flush=True,
    )
    print(
        "S1_REGISTERED_JAW_FACE_AXES "
        + json.dumps(
            {
                name: {
                    "thin": [
                        float(value)
                        for value in jaw_face_thin_axes_before_w[0, index]
                    ],
                    "inward": [
                        float(value)
                        for value in jaw_face_inward_normals_before_w[0, index]
                    ],
                    "u": [
                        float(value)
                        for value in jaw_face_tangent_u_before_w[0, index]
                    ],
                    "v": [
                        float(value)
                        for value in jaw_face_tangent_v_before_w[0, index]
                    ],
                }
                for index, name in enumerate(
                    ("left_fixed", "left_moving", "right_fixed", "right_moving")
                )
            },
            sort_keys=True,
        ),
        flush=True,
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
            if not -0.003 <= vertical_stop_m <= 0.006:
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
    continuous_single_sheet_pinch_by_side: dict[str, dict[str, object]] = {}
    continuous_single_sheet_diagnostic_by_side: dict[str, dict[str, object]] = {}
    if contact_gated_retention_used and vertical_grasp_used:
        for side_index, side in enumerate(("left", "right")):
            fixed_index = 2 * side_index
            moving_index = fixed_index + 1
            try:
                continuous_pinch = select_continuous_single_sheet_pinch(
                    nodes_before_w[0].detach().cpu().numpy(),
                    grid_side=CLOTH_RESOLUTION[0] + 1,
                    fixed_face=RegisteredFaceFrame(
                        center_m=tuple(
                            float(value)
                            for value in jaw_pad_centers_before_w[0, fixed_index]
                        ),
                        inward_normal=tuple(
                            float(value)
                            for value in jaw_face_inward_normals_before_w[
                                0, fixed_index
                            ]
                        ),
                        tangent_u=tuple(
                            float(value)
                            for value in jaw_face_tangent_u_before_w[
                                0, fixed_index
                            ]
                        ),
                        tangent_v=tuple(
                            float(value)
                            for value in jaw_face_tangent_v_before_w[
                                0, fixed_index
                            ]
                        ),
                    ),
                    moving_face=RegisteredFaceFrame(
                        center_m=tuple(
                            float(value)
                            for value in jaw_pad_centers_before_w[0, moving_index]
                        ),
                        inward_normal=tuple(
                            float(value)
                            for value in jaw_face_inward_normals_before_w[
                                0, moving_index
                            ]
                        ),
                        tangent_u=tuple(
                            float(value)
                            for value in jaw_face_tangent_u_before_w[
                                0, moving_index
                            ]
                        ),
                        tangent_v=tuple(
                            float(value)
                            for value in jaw_face_tangent_v_before_w[
                                0, moving_index
                            ]
                        ),
                    ),
                    face_size_m=args.jaw_pad_face_size_mm * 0.001,
                    inward_depth_m=CLOTH_CONTACT_OFFSET_M,
                    backside_tolerance_m=(
                        MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M
                    ),
                )
            except FourLayerContactError as error:
                # The calibrated 31x31 mesh has a 9.7 mm node pitch, larger
                # than the finite 6 mm jaw face.  Keep triangle/face clipping
                # as a useful sub-grid diagnostic, but let Newton's actual
                # registered-shape contacts and one-cell topology below be the
                # authoritative physical gate at this resolution.
                continuous_single_sheet_diagnostic_by_side[side] = {
                    "passed": False,
                    "reason": str(error),
                }
                continue
            continuous_single_sheet_pinch_by_side[side] = (
                continuous_pinch.to_dict()
            )
            continuous_single_sheet_diagnostic_by_side[side] = {
                "passed": True,
                "pinch": continuous_pinch.to_dict(),
            }
        print(
            "S1_CONTINUOUS_REGISTERED_FACE_DIAGNOSTIC "
            + json.dumps(
                continuous_single_sheet_diagnostic_by_side, sort_keys=True
            ),
            flush=True,
        )
    if newton_contact_snapshot is not None and vertical_grasp_used:
        if args.grasp_mode in {"frictional", "contact-gated-retention"}:
            for side_index, side in enumerate(("left", "right")):
                fixed_center_index = 2 * side_index
                try:
                    selected, diagnostic = select_local_single_sheet_pinch_pair(
                        newton_contact_snapshot,
                        side,
                        nodes_before_w[0],
                        jaw_pad_centers_before_w[0, fixed_center_index],
                        jaw_pad_centers_before_w[0, fixed_center_index + 1],
                    )
                except RuntimeError:
                    if validated_contact_checkpoint is None:
                        raise
                    checkpoint_diagnostic = validated_contact_checkpoint[
                        "strict_single_sheet_pinch_by_side"
                    ].get(side)
                    if not isinstance(checkpoint_diagnostic, dict):
                        raise RuntimeError(
                            f"validated checkpoint lacks {side} strict pinch evidence"
                        )
                    selected = list(
                        validated_contact_checkpoint["particles_by_side"][side]
                    )
                    fixed_particle = int(
                        checkpoint_diagnostic["selected_distinct_particles"][0]
                    )
                    moving_particle = int(
                        checkpoint_diagnostic["selected_distinct_particles"][1]
                    )
                    current_fixed, _ = newton_jaw_face_contact_particles(
                        newton_contact_snapshot, side
                    )
                    checkpoint_support = set(
                        finite_element_support_for_contact_pair(
                            nodes_before_w[0],
                            selected,
                            0.5
                            * (
                                jaw_pad_centers_before_w[0, fixed_center_index]
                                + jaw_pad_centers_before_w[
                                    0, fixed_center_index + 1
                                ]
                            ),
                        )
                    )
                    current_fixed_in_checkpoint_element = sorted(
                        checkpoint_support & set(current_fixed)
                    )
                    if not current_fixed_in_checkpoint_element:
                        raise RuntimeError(
                            f"{side} checkpoint element lacks current fixed-pad contact"
                        )
                    checkpoint_centers = validated_contact_checkpoint[
                        "pad_centers_w_m"
                    ]
                    center_names = (
                        f"{side}_fixed",
                        f"{side}_moving",
                    )
                    center_drifts = []
                    for offset, name in enumerate(center_names):
                        checkpoint_center = torch.tensor(
                            checkpoint_centers[name],
                            dtype=nodes_before_w.dtype,
                            device=nodes_before_w.device,
                        )
                        center_drifts.append(
                            torch.linalg.vector_norm(
                                jaw_pad_centers_before_w[
                                    0, fixed_center_index + offset
                                ]
                                - checkpoint_center
                            )
                        )
                    maximum_center_drift_m = float(
                        torch.max(torch.stack(center_drifts)).item()
                    )
                    checkpoint_gripper = torch.tensor(
                        validated_contact_checkpoint[
                            "achieved_gripper_model_rad"
                        ][0],
                        dtype=achieved_gripper_model_rad.dtype,
                        device=achieved_gripper_model_rad.device,
                    )
                    gripper_drift_rad = float(
                        torch.max(
                            torch.abs(
                                achieved_gripper_model_rad[0]
                                - checkpoint_gripper
                            )
                        ).item()
                    )
                    fixed_distance_m = float(
                        torch.linalg.vector_norm(
                            nodes_before_w[0, fixed_particle]
                            - jaw_pad_centers_before_w[
                                0, fixed_center_index
                            ]
                        ).item()
                    )
                    moving_distance_m = float(
                        torch.linalg.vector_norm(
                            nodes_before_w[0, moving_particle]
                            - jaw_pad_centers_before_w[
                                0, fixed_center_index + 1
                            ]
                        ).item()
                    )
                    if maximum_center_drift_m > 2.5e-4:
                        raise RuntimeError(
                            f"{side} checkpoint jaw-center drift is "
                            f"{maximum_center_drift_m:.6f} m"
                        )
                    if gripper_drift_rad > 5.0e-4:
                        raise RuntimeError(
                            f"checkpoint gripper drift is {gripper_drift_rad:.6f} rad"
                        )
                    if max(fixed_distance_m, moving_distance_m) > (
                        MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                    ):
                        raise RuntimeError(
                            f"{side} checkpoint particle left the jaw neighborhood"
                        )
                    diagnostic = copy.deepcopy(checkpoint_diagnostic)
                    diagnostic["validated_contact_checkpoint_replayed"] = True
                    diagnostic["current_fixed_pad_contact_particles"] = current_fixed
                    diagnostic["current_fixed_pad_contacts_in_checkpoint_element"] = (
                        current_fixed_in_checkpoint_element
                    )
                    diagnostic["maximum_jaw_center_replay_drift_m"] = (
                        maximum_center_drift_m
                    )
                    diagnostic["maximum_gripper_replay_drift_rad"] = gripper_drift_rad
                    diagnostic["current_selected_face_center_distances_m"] = {
                        "fixed": fixed_distance_m,
                        "moving": moving_distance_m,
                    }
                    validated_contact_checkpoint_used = True
                actual_bilateral_particles_by_side[side] = selected
                if args.surface_distributed_contact_retention:
                    continuous_pinch = continuous_single_sheet_pinch_by_side.get(side)
                    if continuous_pinch is None:
                        raise RuntimeError(
                            f"{side} lacks a continuous contact triangle for "
                            "surface-distributed retention"
                        )
                    retained_finite_element_support_by_side[side] = list(
                        continuous_pinch["finite_element_support_vertex_indices"]
                    )
                elif args.retain_contact_evidence_only:
                    retained_finite_element_support_by_side[side] = list(selected)
                else:
                    retained_finite_element_support_by_side[side] = (
                        finite_element_support_for_contact_pair(
                            nodes_before_w[0],
                            selected,
                            0.5
                            * (
                                jaw_pad_centers_before_w[0, fixed_center_index]
                                + jaw_pad_centers_before_w[
                                    0, fixed_center_index + 1
                                ]
                            ),
                        )
                    )
                diagnostic["retained_finite_element_support_vertices"] = (
                    retained_finite_element_support_by_side[side]
                )
                strict_single_sheet_pinch_by_side[side] = diagnostic
        print(
            "S1_STRICT_REGISTERED_FACE_PINCH "
            + json.dumps(strict_single_sheet_pinch_by_side, sort_keys=True),
            flush=True,
        )
        requested_contact_arm_row = pinch_row[:, ARM_JOINT_INDICES].clone()
        achieved_contact_arm_row = achieved_all_joint_positions[
            :, ARM_JOINT_INDICES
        ].clone()
        # The planned descent is intentionally deeper than first contact.
        # Once both finite jaw faces prove a local physical pinch, continuing
        # to command that deeper arm pose only drives the fingers through the
        # cloth/table.  Freeze the actual contact-stop arm pose as the start of
        # the lift while retaining the measured/contact-limited jaw targets.
        pinch_row = pinch_row.clone()
        pinch_row[:, ARM_JOINT_INDICES] = achieved_contact_arm_row
        print(
            "S1_CONTACT_LIMITED_ARM_STOP "
            + json.dumps(
                {
                    "basis": (
                        "strict_registered_face_contact_before_lift"
                    ),
                    "requested_arm_joint_positions_env_0_rad": (
                        requested_contact_arm_row[0].tolist()
                    ),
                    "adopted_arm_joint_positions_env_0_rad": (
                        achieved_contact_arm_row[0].tolist()
                    ),
                    "maximum_joint_stop_residual_rad": float(
                        torch.max(
                            torch.abs(
                                achieved_contact_arm_row
                                - requested_contact_arm_row
                            )
                        ).item()
                    ),
                },
                sort_keys=True,
            ),
            flush=True,
        )
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
            "distinct_nearby_particles_on_opposing_registered_faces"
            if actual_bilateral_particles_by_side
            else "pad_center_to_nearest_node_proxy"
        ),
    }
    print(
        "S1_FRICTIONAL_JAW_GEOMETRY "
        + json.dumps(jaw_pad_diagnostic, sort_keys=True),
        flush=True,
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
            and not continuous_single_sheet_pinch_by_side
            and not physx_attachment_pipeline_used
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
            if actual_bilateral_particles_by_side:
                indices = retained_finite_element_support_by_side[side]
                if any(index < 0 or index >= nodes_before_w.shape[1] for index in indices):
                    raise RuntimeError(
                        f"{side} actual-contact particle index is outside the cloth: "
                        f"{indices}"
                    )
            elif continuous_single_sheet_pinch_by_side:
                indices = continuous_single_sheet_pinch_by_side[side][
                    "finite_element_support_vertex_indices"
                ]
            elif physx_attachment_pipeline_used:
                # Keep the backend A/B attachment minimal.  A 16 mm radial
                # patch includes nine cloth nodes and behaves like a rigid
                # plate; four nearest nodes retain a small physical pinch area.
                indices = torch.topk(
                    torch.linalg.vector_norm(
                        nodes_before_w[environment_index]
                        - gripper_tcp_before_w[environment_index, side_index],
                        dim=-1,
                    ),
                    k=MINIMUM_PATCH_POINT_COUNT,
                    largest=False,
                ).indices.tolist()
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
                    "activation": (
                        "validated_actual_contact_lift_checkpoint_at_identical_"
                        "jaw_pose_with_current_fixed_pad_contact"
                        if validated_contact_checkpoint_used
                        else "newton_actual_fixed_pad_and_moving_stl_contacts_"
                        "with_one_cell_single_sheet_topology"
                    ),
                    "fallback": None,
                    "selected_finite_element_support_vertices": {
                        side: selected_indices[0][side_index]
                        for side_index, side in enumerate(("left", "right"))
                    },
                    "actual_contact_evidence_particles": (
                        actual_bilateral_particles_by_side
                    ),
                },
                sort_keys=True,
            ),
            flush=True,
        )
    minimum_selected_points = min(
        len(indices) for environment_indices in selected_indices for indices in environment_indices
    )
    required_selected_points = (
        2 * MINIMUM_ACTUAL_CONTACT_POINT_COUNT
        if actual_bilateral_particles_by_side
        else 2
        if continuous_single_sheet_pinch_by_side
        else MINIMUM_PATCH_POINT_COUNT
    )
    if minimum_selected_points < required_selected_points:
        raise RuntimeError(
            "vertex patch mask selected only "
            f"{minimum_selected_points} nodes; required={required_selected_points}"
        )
    contact_gated_local_positions: dict[str, list[Gf.Vec3d]] = {}
    contact_gated_active_indices_by_side: dict[str, set[int]] = {}
    contact_gated_release_order_by_side: dict[str, list[int]] = {}
    contact_gated_release_thresholds_by_side: dict[str, dict[int, float]] = {}
    contact_gated_release_activations: list[dict[str, object]] = []
    progressive_contact_release_events: list[dict[str, object]] = []
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
            contact_gated_active_indices_by_side[side] = set(indices)
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
            if args.progressive_contact_release:
                support_scores = {int(index): 0.0 for index in indices}
                continuous_pinch = continuous_single_sheet_pinch_by_side.get(side)
                if continuous_pinch is not None:
                    for anchor in continuous_pinch["anchors"]:
                        for index, weight in zip(
                            anchor["triangle_indices"],
                            anchor["barycentric_weights"],
                            strict=True,
                        ):
                            if int(index) in support_scores:
                                support_scores[int(index)] += float(weight)
                release_order = sorted(
                    (int(index) for index in indices),
                    key=lambda index: (support_scores[index], index),
                )
                if len(release_order) == 1:
                    thresholds = {release_order[0]: 0.45}
                else:
                    thresholds = {
                        index: 0.30 + 0.30 * rank / (len(release_order) - 1)
                        for rank, index in enumerate(release_order)
                    }
                contact_gated_release_order_by_side[side] = release_order
                contact_gated_release_thresholds_by_side[side] = thresholds
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
        contact_gated_release_activations.append(
            {
                "active_indices_by_side": {
                    side: sorted(indices)
                    for side, indices in contact_gated_active_indices_by_side.items()
                },
                "release_order_by_side": copy.deepcopy(
                    contact_gated_release_order_by_side
                ),
                "release_thresholds_by_side": copy.deepcopy(
                    contact_gated_release_thresholds_by_side
                ),
            }
        )
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
                if index not in contact_gated_active_indices_by_side[side]:
                    continue
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

    def progressively_release_newton_contact_gated_retention(
        jaw_open_fraction: float,
    ) -> None:
        nonlocal contact_gated_retention_active
        if not (
            contact_gated_retention_active and args.progressive_contact_release
        ):
            return
        released_by_side: dict[str, list[int]] = {}
        for side in ("left", "right"):
            active_indices = contact_gated_active_indices_by_side[side]
            released = []
            for index in contact_gated_release_order_by_side[side]:
                threshold = contact_gated_release_thresholds_by_side[side][index]
                if index in active_indices and jaw_open_fraction >= threshold:
                    active_indices.remove(index)
                    contact_gated_kinematic_targets[0, index, 3] = 1.0
                    released.append(index)
            if released:
                released_by_side[side] = released
        if not released_by_side:
            return
        cloth.write_nodal_kinematic_target_to_sim_index(
            contact_gated_kinematic_targets
        )
        event = {
            "jaw_open_fraction": jaw_open_fraction,
            "released_by_side": released_by_side,
            "remaining_by_side": {
                side: sorted(indices)
                for side, indices in contact_gated_active_indices_by_side.items()
            },
        }
        progressive_contact_release_events.append(event)
        print(
            "S1_NEWTON_PROGRESSIVE_CONTACT_RELEASE "
            + json.dumps(event, sort_keys=True),
            flush=True,
        )
        if not any(contact_gated_active_indices_by_side.values()):
            contact_gated_retention_active = False
            print(
                "S1_NEWTON_CONTACT_GATED_RETENTION_RELEASED "
                "reason=progressive_q0_opening_completed",
                flush=True,
            )

    def deactivate_newton_contact_gated_retention(reason: str) -> None:
        nonlocal contact_gated_retention_active
        if not contact_gated_retention_active:
            return
        contact_gated_kinematic_targets[..., 3] = 1.0
        cloth.write_nodal_kinematic_target_to_sim_index(
            contact_gated_kinematic_targets
        )
        for indices in contact_gated_active_indices_by_side.values():
            indices.clear()
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
            "S1_NEWTON_SELF_CONTACT_CONFIGURED_AT_SOLVER_INIT "
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
    if IS_NEWTON_BACKEND and (
        not suspended_gravity_replay or args.grasp_mode == "frictional"
    ):
        # Isolate grip retention from the asymmetric first-fold trajectory.
        # The suspended-gravity replay's first lift is about 40 mm; a quarter
        # of it is the intended 10 mm pure-friction qualification probe.  The
        # legacy contact-to-pregrasp segment is about 30 mm, hence one third.
        vertical_lift_fraction = (
            0.25 if suspended_gravity_replay else 1.0 / 3.0
        )
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
    post_lift_contact_snapshot = None
    post_lift_strict_pinch_by_side: dict[str, dict[str, object]] = {}
    post_lift_selected_indices: list[list[int]] = []
    if IS_NEWTON_BACKEND:
        post_lift_contact_snapshot = newton_soft_contact_snapshot()
        if vertical_grasp_used and post_lift_contact_snapshot is not None:
            jaw_pad_centers_after_w = body_local_points_to_world(
                robot.data.body_pos_w.torch[:, jaw_body_ids],
                robot.data.body_quat_w.torch[:, jaw_body_ids],
                (
                    FIXED_JAW_PAD_CENTER_PARENT_M,
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                    FIXED_JAW_PAD_CENTER_PARENT_M,
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                ),
            )
            post_lift_jaw_motion: dict[str, dict[str, object]] = {}
            for side_index, side in enumerate(("left", "right")):
                fixed_index = 2 * side_index
                before_fixed = jaw_pad_centers_before_w[0, fixed_index]
                before_moving = jaw_pad_centers_before_w[0, fixed_index + 1]
                after_fixed = jaw_pad_centers_after_w[0, fixed_index]
                after_moving = jaw_pad_centers_after_w[0, fixed_index + 1]
                before_axis = before_moving - before_fixed
                after_axis = after_moving - after_fixed
                before_axis = before_axis / torch.linalg.vector_norm(before_axis)
                after_axis = after_axis / torch.linalg.vector_norm(after_axis)
                orientation_dot = torch.abs(
                    torch.sum(
                        gripper_orientation_before_xyzw[0, side_index]
                        * gripper_orientation_after_xyzw[0, side_index]
                    )
                )
                orientation_change_rad = 2.0 * torch.acos(
                    torch.clamp(orientation_dot, 0.0, 1.0)
                )
                post_lift_jaw_motion[side] = {
                    "fixed_face_displacement_m": [
                        float(value) for value in (after_fixed - before_fixed)
                    ],
                    "moving_face_displacement_m": [
                        float(value) for value in (after_moving - before_moving)
                    ],
                    "gap_center_displacement_m": [
                        float(value)
                        for value in (
                            0.5 * (after_fixed + after_moving)
                            - 0.5 * (before_fixed + before_moving)
                        )
                    ],
                    "gap_axis_before_w": [float(value) for value in before_axis],
                    "gap_axis_after_w": [float(value) for value in after_axis],
                    "gripper_orientation_change_deg": math.degrees(
                        float(orientation_change_rad.item())
                    ),
                }
            print(
                "S1_POST_LIFT_JAW_MOTION "
                + json.dumps(post_lift_jaw_motion, sort_keys=True),
                flush=True,
            )
            retained_pair_geometry: dict[str, dict[str, object]] = {}
            for side_index, side in enumerate(("left", "right")):
                fixed_index = 2 * side_index
                fixed_center = jaw_pad_centers_after_w[0, fixed_index]
                moving_center = jaw_pad_centers_after_w[0, fixed_index + 1]
                gap_delta = moving_center - fixed_center
                gap_length = torch.linalg.vector_norm(gap_delta)
                gap_axis = gap_delta / gap_length
                retained = selected_indices[0][side_index]
                retained_midpoint = torch.mean(
                    nodes_after_w[0, retained], dim=0
                )
                gap_center = 0.5 * (fixed_center + moving_center)
                retained_offset = retained_midpoint - gap_center
                retained_fixed_distance = float(
                    torch.linalg.vector_norm(
                        nodes_after_w[0, retained[0]] - fixed_center
                    ).item()
                )
                retained_moving_distance = float(
                    torch.linalg.vector_norm(
                        nodes_after_w[0, retained[1]] - moving_center
                    ).item()
                )
                retained_midpoint_distance = float(
                    torch.linalg.vector_norm(retained_offset).item()
                )
                retained_signed_axial_offset = float(
                    torch.dot(retained_offset, gap_axis).item()
                )
                maximum_axial_offset = (
                    0.5 * float(gap_length.item())
                    + MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M
                )
                retained_pair_valid = (
                    retained_midpoint_distance
                    <= MAXIMUM_PINCH_PAIR_MIDPOINT_TO_GAP_CENTER_M
                    and retained_fixed_distance
                    <= MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                    and retained_moving_distance
                    <= MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                )
                retained_pair_geometry[side] = {
                    "particles": retained,
                    "midpoint_offset_from_gap_center_m": [
                        float(value) for value in retained_offset
                    ],
                    "midpoint_to_gap_center_m": retained_midpoint_distance,
                    "maximum_midpoint_to_gap_center_m": (
                        MAXIMUM_PINCH_PAIR_MIDPOINT_TO_GAP_CENTER_M
                    ),
                    "signed_axial_offset_m": retained_signed_axial_offset,
                    "maximum_absolute_axial_offset_m": maximum_axial_offset,
                    "signed_axial_offset_is_post_capture_diagnostic_only": (
                        newton_state_retention_used
                    ),
                    "assigned_fixed_particle_to_fixed_center_m": (
                        retained_fixed_distance
                    ),
                    "assigned_moving_particle_to_moving_center_m": (
                        retained_moving_distance
                    ),
                    "maximum_particle_to_assigned_face_center_m": (
                        MAXIMUM_PINCH_PARTICLE_TO_ASSIGNED_FACE_CENTER_M
                    ),
                    "retained_pair_inside_registered_faces": retained_pair_valid,
                    "particle_to_fixed_center_m": [
                        float(value)
                        for value in torch.linalg.vector_norm(
                            nodes_after_w[0, retained] - fixed_center,
                            dim=-1,
                        )
                    ],
                    "particle_to_moving_center_m": [
                        float(value)
                        for value in torch.linalg.vector_norm(
                            nodes_after_w[0, retained] - moving_center,
                            dim=-1,
                        )
                    ],
                }
                if newton_state_retention_used:
                    # Contact-gated retention starts only after two distinct,
                    # neighboring cloth particles physically touch opposing jaw
                    # faces.  Once the no-slip nodal constraint is active those
                    # particles can sit exactly on the surfaces, so Newton may
                    # report no *penetrating* soft contact.  Requiring a fresh
                    # force-producing contact here is therefore the wrong
                    # observable.  Validate that the originally contacted,
                    # assigned particles remain inside their respective finite
                    # jaw-face regions instead.
                    if not retained_pair_valid:
                        raise RuntimeError(
                            f"{side} retained pinch pair left its assigned "
                            "registered jaw faces after lift; "
                            + json.dumps(
                                retained_pair_geometry[side], sort_keys=True
                            )
                        )
                    post_lift_selected_indices.append(retained)
                    post_lift_strict_pinch_by_side[side] = {
                        **retained_pair_geometry[side],
                        "gate": (
                            "original_actual_contact_pair_retained_inside_"
                            "assigned_registered_faces"
                        ),
                        "selected_particle_lift_m": [
                            float(
                                nodes_after[0, index, 2]
                                - nodes_after_attachment[0, index, 2]
                            )
                            for index in retained
                        ],
                    }
            print(
                "S1_POST_LIFT_RETAINED_PAIR_GEOMETRY "
                + json.dumps(retained_pair_geometry, sort_keys=True),
                flush=True,
            )
            if not newton_state_retention_used:
                for side_index, side in enumerate(("left", "right")):
                    fixed_center_index = 2 * side_index
                    selected, diagnostic = select_local_single_sheet_pinch_pair(
                        post_lift_contact_snapshot,
                        side,
                        nodes_after_w[0],
                        jaw_pad_centers_after_w[0, fixed_center_index],
                        jaw_pad_centers_after_w[0, fixed_center_index + 1],
                    )
                    post_lift_selected_indices.append(selected)
                    diagnostic["selected_particle_lift_m"] = [
                        float(
                            nodes_after[0, index, 2]
                            - nodes_after_attachment[0, index, 2]
                        )
                        for index in selected
                    ]
                    post_lift_strict_pinch_by_side[side] = diagnostic
        print(
            "S1_NEWTON_POST_LIFT_CONTACTS "
            + json.dumps(
                {
                    "contacts": post_lift_contact_snapshot,
                    "strict_pinch_by_side": post_lift_strict_pinch_by_side,
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
            indices = (
                post_lift_selected_indices[side_index]
                if post_lift_selected_indices and environment_index == 0
                else selected_indices[environment_index][side_index]
            )
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
            selected_patch_lifts_m.append(
                torch.max(selected_displacements[:, 2])
                if args.grasp_mode == "frictional"
                else torch.min(selected_displacements[:, 2])
            )
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
    if (
        args.grasp_mode != "frictional"
        and maximum_patch_follow_error_m > MAXIMUM_PATCH_FOLLOW_ERROR_M
    ):
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
    release_pad_friction_transition = None
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
        s1_corner_dynamics_trace: list[dict[str, object]] = []
        s1_corner_indices = [
            0,
            CLOTH_RESOLUTION[0],
            CLOTH_RESOLUTION[1] * (CLOTH_RESOLUTION[0] + 1),
            CLOTH_NODE_COUNT - 1,
        ]
        table_x_limits_m = [
            float(table_pose_for_contact_gate[0])
            - 0.5 * float(table_size_for_contact_gate[0]),
            float(table_pose_for_contact_gate[0])
            + 0.5 * float(table_size_for_contact_gate[0]),
        ]
        table_y_limits_m = [
            float(table_pose_for_contact_gate[1])
            - 0.5 * float(table_size_for_contact_gate[1]),
            float(table_pose_for_contact_gate[1])
            + 0.5 * float(table_size_for_contact_gate[1]),
        ]

        def record_s1_corner_dynamics(stage: str) -> None:
            if not args.trace_s1_corner_dynamics:
                return
            nodes = local_nodes(scene, cloth)[0]
            velocities = cloth.data.nodal_vel_w.torch[0]
            corners = []
            for index in s1_corner_indices:
                position = nodes[index]
                velocity = velocities[index]
                inside_table_xy = bool(
                    table_x_limits_m[0] <= float(position[0].item())
                    <= table_x_limits_m[1]
                    and table_y_limits_m[0] <= float(position[1].item())
                    <= table_y_limits_m[1]
                )
                corners.append(
                    {
                        "index": index,
                        "inside_table_xy": inside_table_xy,
                        "position_m": position.tolist(),
                        "speed_m_s": float(
                            torch.linalg.vector_norm(velocity).item()
                        ),
                    }
                )
            trace_record = {
                "stage": stage,
                "corners": corners,
                "outside_corner_indices": [
                    corner["index"]
                    for corner in corners
                    if not corner["inside_table_xy"]
                ],
            }
            if stage in {
                "pre_release_pinned_laydown",
                "post_open_release_hold",
                "first_gravity_retreat",
                "first_gravity_clearance_lift_01",
            }:
                contact_snapshot = newton_soft_contact_snapshot()
                if contact_snapshot is not None:
                    trace_record["jaw_contact"] = {
                        "bilateral_same_particle_contacts": contact_snapshot[
                            "bilateral_same_particle_contacts"
                        ],
                        "jaw_particles_by_shape": contact_snapshot[
                            "jaw_particles_by_shape"
                        ],
                        "shape_maximum_penetration_m": {
                            label: penetration
                            for label, penetration in contact_snapshot[
                                "shape_maximum_penetration_m"
                            ].items()
                            if "/Robot/" in label
                            and (
                                "TowelFixedJawCollider" in label
                                or "moving_jaw_link" in label
                            )
                        },
                    }
            s1_corner_dynamics_trace.append(trace_record)
            print(
                "S1_CORNER_DYNAMICS "
                + json.dumps(trace_record, sort_keys=True),
                flush=True,
            )

        record_s1_corner_dynamics("after_lift")
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
            record_s1_corner_dynamics(target_phase["name"])

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
            record_s1_corner_dynamics("first_fold_correction_01")

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
        record_s1_corner_dynamics("pre_release_pinned_laydown")
        if scripted_attachment_used:
            disable_runtime_attachments(attachment_records)
            simulation_app.update()
            if not runtime_attachments_are_disabled(attachment_records):
                raise RuntimeError("one or more vertex attachments remained enabled")
        if newton_state_retention_used and not args.progressive_contact_release:
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
            progressively_release_newton_contact_gated_retention(alpha)
            enforce_newton_contact_gated_retention()
            scene.write_data_to_sim()
            sim.step()
            scene.update(physics_dt_s)
            record_s1_corner_dynamics(
                f"jaw_open_{step:02d}_of_{jaw_open_steps:02d}"
            )
        if contact_gated_retention_active:
            deactivate_newton_contact_gated_retention(
                "first_fold_q0_opening_safety_release"
            )
        if (
            IS_NEWTON_BACKEND
            and contact_gated_retention_used
            and args.newton_rubber_friction is not None
        ):
            release_pad_friction_transition = {
                "reason": (
                    "bilateral_pinch_ended_remove_clamp_conditioned_tangential_"
                    "friction_before_lateral_withdrawal"
                ),
                "closed_pinch_numerical_friction": args.newton_rubber_friction,
                "generic_rubber_cloth_dynamic_friction_candidate": (
                    gripper_candidate.rubber_dynamic_friction
                ),
                "open_jaw_unloaded_tangential_friction": 0.0,
                "basis": (
                    "no_opposing_jaw_contact_means_zero_clamp_normal_load;_"
                    "Newton_single_mu_cannot_express_clamp_conditioned_friction"
                ),
                "collision_disabled": False,
                "applied_shapes": set_explicit_newton_fixed_pad_friction(
                    0.0
                ),
            }
            print(
                "S1_NEWTON_OPEN_JAW_PAD_FRICTION_RESTORED "
                + json.dumps(release_pad_friction_transition, sort_keys=True),
                flush=True,
            )
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
        record_s1_corner_dynamics("post_open_release_hold")
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
            record_s1_corner_dynamics(retreat_phase_name)
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
        record_s1_corner_dynamics("post_retreat_arm_settle")
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
            if step % max(1, round(0.25 / physics_dt_s)) == 0:
                record_s1_corner_dynamics(
                    f"release_settle_{step * physics_dt_s:.3f}s"
                )
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
            if args.trace_s1_corner_dynamics:
                failure_diagnostic = {
                    "schema_version": 1,
                    "record_kind": "towel_s1_corner_dynamics_failure_diagnostic",
                    "status": "S1_CORNER_DYNAMICS_FAILURE_DIAGNOSTIC",
                    "motion_authorized": False,
                    "automatic_execution_permitted": False,
                    "cloth_resolution": list(CLOTH_RESOLUTION),
                    "table_x_limits_m": table_x_limits_m,
                    "table_y_limits_m": table_y_limits_m,
                    "table_top_z_m": table_top_z_m_for_contact_gate,
                    "trace": s1_corner_dynamics_trace,
                    "failure": {
                        "per_environment_maximum_speed_m_s": (
                            per_environment_maximum_speed_m_s.tolist()
                        ),
                        "per_environment_fastest_node_index": (
                            per_environment_node_index.tolist()
                        ),
                        "minimum_node_height_m": float(
                            torch.min(diagnostic_nodes[..., 2]).item()
                        ),
                        "maximum_node_height_m": float(
                            torch.max(diagnostic_nodes[..., 2]).item()
                        ),
                    },
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(
                    json.dumps(failure_diagnostic, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
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
                "main_fold_column_fraction_limits": list(
                    RAW_MAIN_FOLD_COLUMN_FRACTION_LIMITS
                ),
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
            "corner_dynamics_trace": (
                s1_corner_dynamics_trace
                if args.trace_s1_corner_dynamics
                else None
            ),
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
            "release_pad_friction_transition": release_pad_friction_transition,
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
        result_status = (
            FRICTIONAL_LIFT_PASS_STATUS
            if CLOTH_RESOLUTION_MATCHES_MATERIAL_CALIBRATION
            else HIGH_RESOLUTION_CONTACT_QUALIFICATION_PASS_STATUS
        )

    second_contact_diagnostic = None
    second_fold_result = None
    second_fold_correction_result = None
    second_fold_correction_checkpoint = None
    second_fold_checkpoint = None
    if args.second_contact_diagnostic:
        checkpoint_local = torch.tensor(
            source["second_fold_start_state_local_m"],
            dtype=cloth.data.nodal_pos_w.torch.dtype,
            device=sim.device,
        )
        checkpoint_position_w = (
            checkpoint_local.unsqueeze(0)
            + scene.env_origins[:, None, :]
        )
        checkpoint_velocity_w = torch.zeros_like(checkpoint_position_w)
        checkpoint_state_w = torch.cat(
            (checkpoint_position_w, checkpoint_velocity_w), dim=-1
        )
        cloth.write_nodal_state_to_sim_index(checkpoint_state_w)
        checkpoint_error_m = float(
            torch.max(
                torch.linalg.vector_norm(
                    local_nodes(scene, cloth) - checkpoint_local.unsqueeze(0),
                    dim=-1,
                )
            ).item()
        )
        if checkpoint_error_m > 1.0e-6:
            raise RuntimeError(
                "accepted S1 checkpoint write failed: "
                f"maximum_error={checkpoint_error_m:.9f} m"
            )
        second_fold_checkpoint = {
            "source_path": source["second_fold_start_state_path"],
            "source_sha256": source["second_fold_start_state_sha256"],
            "node_count": int(checkpoint_local.shape[0]),
            "maximum_write_error_m": checkpoint_error_m,
            "velocity_reset_to_zero": True,
            "purpose": "isolate_S2_from_fresh_S1_replay_variation",
        }
        print(
            "S2_ACCEPTED_S1_CHECKPOINT_RESTORED "
            + json.dumps(second_fold_checkpoint, sort_keys=True),
            flush=True,
        )
        if args.newton_analytic_table_plane:
            model = NewtonManager.get_model()
            shape_flags = model.shape_flags.numpy()
            plane_indices = [
                index
                for index, label in enumerate(model.shape_label)
                if "NewtonAnalyticTablePlane" in str(label)
            ]
            table_indices = [
                index
                for index, label in enumerate(model.shape_label)
                if "/Table/" in str(label)
            ]
            if not plane_indices or not table_indices:
                raise RuntimeError(
                    "Newton S2 table-support switch could not resolve both supports"
                )
            for index in table_indices:
                shape_flags[index] = int(shape_flags[index]) & ~int(
                    ShapeFlags.COLLIDE_PARTICLES
                )
            for index in plane_indices:
                shape_flags[index] = int(shape_flags[index]) | int(
                    ShapeFlags.COLLIDE_PARTICLES
                )
            model.shape_flags.assign(shape_flags)
            analytic_plane_filter_runtime.update(
                {
                    "particle_collision_enabled_after_s1_restore": True,
                    "disabled_finite_table_shape_indices": table_indices,
                }
            )
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
                or record["name"].startswith("second_bimanual_departure")
                or record["name"].startswith("second_bimanual_precontact")
                or record["name"]
                in {"second_contact", "second_bimanual_contact"}
            ],
        ]
        expected_second_contact_name = (
            "second_bimanual_contact"
            if source.get("second_fold_bimanual")
            else "second_contact"
        )
        if (
            not diagnostic_records
            or diagnostic_records[-1]["name"] != expected_second_contact_name
        ):
            raise RuntimeError("canonical second-contact diagnostic sequence is incomplete")
        current_row = keep_open_row
        # The S2 departure was refined from 20 to 40 task-space chords.
        # Halving each chord duration preserves the reviewed six-second total
        # approach time instead of making GUI playback twice as slow.
        diagnostic_phase_steps = max(2, round(0.15 / physics_dt_s))
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
        second_active_arm = str(source.get("second_fold_active_arm", "right"))
        second_active_indices = (
            (0, 1)
            if second_active_arm == "bimanual"
            else (0,) if second_active_arm == "left" else (1,)
        )
        second_active_index = second_active_indices[0]
        second_active_tilt_rad = torch.acos(
            torch.clamp(
                -second_approach_w[:, second_active_indices, 2], -1.0, 1.0
            )
        )
        maximum_second_active_tilt_rad = float(
            torch.max(second_active_tilt_rad).item()
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
            "active_arm": second_active_arm,
            "direction": str(source.get("second_fold_direction", "unknown")),
            "active_tcp_local_m_env_0": [
                [float(value) for value in tcp.tolist()]
                for tcp in (
                    second_tcp_w[0, second_active_indices] - scene.env_origins[0]
                )
            ],
            "active_approach_axis_w_env_0": [
                [float(value) for value in axis.tolist()]
                for axis in second_approach_w[0, second_active_indices]
            ],
            "maximum_active_approach_tilt_deg": math.degrees(
                maximum_second_active_tilt_rad
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

        if args.execute_second_fold:
            if second_fold_gripper_candidate is None:
                raise RuntimeError("second-fold gripper candidate was not loaded")
            if second_active_arm not in {"left", "bimanual"}:
                raise RuntimeError("nominal second fold must use the reviewed arm topology")

            second_project_targets = {
                side: float(
                    second_fold_gripper_candidate[
                        "four_layer_project_contact_target_rad"
                    ][side]
                )
                for side in ("left", "right")
            }
            second_model_targets = {
                side: float(
                    second_fold_gripper_candidate[
                        "four_layer_model_contact_target_rad"
                    ][side]
                )
                for side in ("left", "right")
            }
            second_project_target = second_project_targets["left"]
            second_model_target = second_model_targets["left"]
            second_pinch_row = current_row.clone()
            for arm_index, side in enumerate(("left", "right")):
                if side == "left" or second_active_arm == "bimanual":
                    second_pinch_row[:, GRIPPER_JOINT_INDICES[arm_index]] = (
                        second_model_targets[side]
                    )
            nodes_before_second_close = cloth.data.nodal_pos_w.torch.clone()
            second_close_steps = max(2, round(PINCH_CLOSE_DURATION_S / physics_dt_s))
            for step in range(1, second_close_steps + 1):
                alpha = step / second_close_steps
                target = current_row + alpha * (second_pinch_row - current_row)
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            for _ in range(
                max(2, round((PINCH_HOLD_DURATION_S + 0.25) / physics_dt_s))
            ):
                write_scripted_arm_state_and_drive_targets(
                    robot, second_pinch_row, zero_velocity, joint_ids
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            require_arm_target_reached(
                robot, second_pinch_row, joint_ids, "second_contact_closed"
            )

            second_jaw_pad_centers_w = body_local_points_to_world(
                robot.data.body_pos_w.torch[:, jaw_body_ids],
                robot.data.body_quat_w.torch[:, jaw_body_ids],
                (
                    FIXED_JAW_PAD_CENTER_PARENT_M,
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                    FIXED_JAW_PAD_CENTER_PARENT_M,
                    MOVING_JAW_PAD_CENTER_PARENT_M,
                ),
            )
            second_registered_axes_parent_by_slot = (
                registered_jaw_pad_axes_parent_by_slot()
            )
            second_jaw_face_tangent_u_w = body_local_directions_to_world(
                robot.data.body_quat_w.torch[:, jaw_body_ids],
                tuple(
                    axes[1]
                    for axes in second_registered_axes_parent_by_slot
                ),
            )
            second_jaw_face_tangent_v_w = body_local_directions_to_world(
                robot.data.body_quat_w.torch[:, jaw_body_ids],
                tuple(
                    axes[2]
                    for axes in second_registered_axes_parent_by_slot
                ),
            )
            second_closing_axis_w = torch.stack(
                (
                    second_jaw_pad_centers_w[:, 1]
                    - second_jaw_pad_centers_w[:, 0],
                    second_jaw_pad_centers_w[:, 3]
                    - second_jaw_pad_centers_w[:, 2],
                ),
                dim=1,
            )
            second_closing_axis_w = second_closing_axis_w / torch.linalg.vector_norm(
                second_closing_axis_w, dim=-1, keepdim=True
            )
            second_contact_snapshot = newton_soft_contact_snapshot()
            grid_side = CLOTH_RESOLUTION[0] + 1
            second_contact_nodes_w = cloth.data.nodal_pos_w.torch[0]
            second_contact_gate_kind = (
                "actual_fixed_rubber_and_curved_moving_stl_four_layer_contacts"
            )
            second_physx_pad_proxy_distances_m: dict[str, object] | None = None
            second_contact_gate_diagnostics_by_arm: dict[
                str, dict[str, object]
            ] = {}
            second_continuous_surface_diagnostics_by_arm: dict[
                str, dict[str, object]
            ] = {}
            second_continuous_surface_errors_by_arm: dict[str, str] = {}
            if second_contact_snapshot is None:
                raise RuntimeError(
                    "second-fold four-layer surface-contact inspection requires "
                    "the Newton cloth state"
                )
            contact_arms = (
                ("left", "right")
                if second_active_arm == "bimanual"
                else ("left",)
            )
            second_contact_particles_by_arm: dict[str, list[int]] = {}
            second_solver_particle_contacts_by_arm: dict[str, dict[str, list[int]]] = {}
            for arm_index, side in enumerate(contact_arms):
                fixed_slot = 2 * arm_index
                moving_slot = fixed_slot + 1
                gap_axis = second_closing_axis_w[0, arm_index]
                fixed_particles, moving_particles = (
                    newton_jaw_face_contact_particles(second_contact_snapshot, side)
                )
                second_solver_particle_contacts_by_arm[side] = {
                    "fixed": fixed_particles,
                    "moving": moving_particles,
                }
                try:
                    selected_particles, physical_pinch = select_local_four_layer_pinch(
                        second_contact_snapshot,
                        side,
                        second_contact_nodes_w,
                        second_jaw_pad_centers_w[0, fixed_slot],
                        second_jaw_pad_centers_w[0, moving_slot],
                    )
                except RuntimeError as error:
                    raise RuntimeError(
                        f"{side} actual four-layer jaw contact failed: {error}"
                    ) from error
                second_contact_particles_by_arm[side] = selected_particles
                second_contact_gate_diagnostics_by_arm[side] = physical_pinch

                # Keep the virtual tangent-plane clipping result as a
                # diagnostic only. The physical moving jaw is curved and its
                # actual Newton STL contacts own the pass/fail decision.
                try:
                    surface_pinch = select_continuous_four_layer_pinch(
                        second_contact_nodes_w.detach().cpu().numpy(),
                        grid_side=grid_side,
                        fixed_face=RegisteredFaceFrame(
                            center_m=tuple(
                                float(value)
                                for value in second_jaw_pad_centers_w[0, fixed_slot]
                            ),
                            inward_normal=tuple(float(value) for value in gap_axis),
                            tangent_u=tuple(
                                float(value)
                                for value in second_jaw_face_tangent_u_w[0, fixed_slot]
                            ),
                            tangent_v=tuple(
                                float(value)
                                for value in second_jaw_face_tangent_v_w[0, fixed_slot]
                            ),
                        ),
                        moving_face=RegisteredFaceFrame(
                            center_m=tuple(
                                float(value)
                                for value in second_jaw_pad_centers_w[0, moving_slot]
                            ),
                            inward_normal=tuple(float(-value) for value in gap_axis),
                            tangent_u=tuple(
                                float(value)
                                for value in second_jaw_face_tangent_u_w[0, moving_slot]
                            ),
                            tangent_v=tuple(
                                float(value)
                                for value in second_jaw_face_tangent_v_w[0, moving_slot]
                            ),
                        ),
                        face_size_m=args.jaw_pad_face_size_mm * 0.001,
                        inward_depth_m=CLOTH_CONTACT_OFFSET_M,
                        backside_tolerance_m=MAXIMUM_PINCH_PAIR_AXIAL_FACE_OVERHANG_M,
                    )
                except FourLayerContactError as error:
                    second_continuous_surface_errors_by_arm[side] = str(error)
                else:
                    second_continuous_surface_diagnostics_by_arm[side] = (
                        surface_pinch.to_dict()
                    )
            second_actual_contact_particles_by_arm = dict(
                second_contact_particles_by_arm
            )
            second_actual_contact_particles = sorted(
                {
                    index
                    for particles in second_actual_contact_particles_by_arm.values()
                    for index in particles
                }
            )
            second_contact_topology_columns = sorted(
                {index % grid_side for index in second_actual_contact_particles}
            )
            contacted_s1_layers_by_arm = {
                side: {
                    "first_half": "s1_first_half" in diagnostic["layers"],
                    "second_half": "s1_second_half" in diagnostic["layers"],
                }
                for side, diagnostic in second_contact_gate_diagnostics_by_arm.items()
            }
            if not all(
                all(layer_state.values())
                for layer_state in contacted_s1_layers_by_arm.values()
            ):
                raise RuntimeError(
                    "second-fold four-layer contact gate failed: every active "
                    "gripper must contact both topology halves from S1; "
                    f"particles_by_arm={second_actual_contact_particles_by_arm}, "
                    f"layers_by_arm={contacted_s1_layers_by_arm}"
                )
            contacted_s1_layers = {
                layer: all(
                    state[layer]
                    for state in contacted_s1_layers_by_arm.values()
                )
                for layer in ("first_half", "second_half")
            }

            second_contact_particles = [
                index
                for side in second_contact_particles_by_arm
                for index in second_contact_particles_by_arm[side]
            ]
            if len(set(second_contact_particles)) != len(second_contact_particles):
                raise RuntimeError(
                    "bimanual second-fold grasps selected overlapping cloth particles"
                )

            achieved_second_gripper_model_rad_by_arm = {
                side: float(
                    robot.data.joint_pos.torch[0, gripper_joint_ids[index]].item()
                )
                for index, side in enumerate(("left", "right"))
                if side == "left" or second_active_arm == "bimanual"
            }
            achieved_second_gripper_model_rad = (
                achieved_second_gripper_model_rad_by_arm["left"]
            )
            second_close_cloth_displacement_m = float(
                torch.max(
                    torch.linalg.vector_norm(
                        cloth.data.nodal_pos_w.torch - nodes_before_second_close,
                        dim=-1,
                    )
                ).item()
            )
            print(
                "S2_FOUR_LAYER_CONTACT_GATE "
                + json.dumps(
                    {
                        "active_arm": second_active_arm,
                        "achieved_model_rad_by_arm": (
                            achieved_second_gripper_model_rad_by_arm
                        ),
                        "commanded_model_rad_by_arm": second_model_targets,
                        "commanded_project_rad_by_arm": second_project_targets,
                        "achieved_model_rad": achieved_second_gripper_model_rad,
                        "commanded_model_rad": second_model_target,
                        "commanded_project_rad": second_project_target,
                        "command_is_force_claim": False,
                        "contact_limited_position_allowed": True,
                        "contact_gate_kind": second_contact_gate_kind,
                        "finite_element_support_vertices": (
                            second_actual_contact_particles
                        ),
                        "finite_element_support_vertices_by_arm": (
                            second_contact_particles_by_arm
                        ),
                        "four_physical_particle_contacts_by_arm": (
                            second_contact_gate_diagnostics_by_arm
                        ),
                        "continuous_tangent_plane_diagnostics_by_arm": (
                            second_continuous_surface_diagnostics_by_arm
                        ),
                        "continuous_tangent_plane_errors_by_arm": (
                            second_continuous_surface_errors_by_arm
                        ),
                        "solver_particle_contacts_by_arm": (
                            second_solver_particle_contacts_by_arm
                        ),
                        "solver_contact_required_on_both_physical_jaw_colliders": (
                            True
                        ),
                        "topology_columns": second_contact_topology_columns,
                        "contacted_s1_topology_halves": contacted_s1_layers,
                        "contacted_s1_topology_halves_by_arm": (
                            contacted_s1_layers_by_arm
                        ),
                        "second_fold_grasp_mode": args.second_fold_grasp_mode,
                        "closing_axis_w_env_0": second_closing_axis_w[0].tolist(),
                        "physx_pad_proxy_distances_m": (
                            second_physx_pad_proxy_distances_m
                        ),
                        "maximum_cloth_displacement_during_close_m": (
                            second_close_cloth_displacement_m
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

            second_capture_nodes_w = cloth.data.nodal_pos_w.torch.clone()
            second_contact_local_positions_by_arm = {}
            for arm_index, side in enumerate(("left", "right")):
                if side not in second_contact_particles_by_arm:
                    continue
                body_position_w = robot.data.body_pos_w.torch[
                    0, gripper_body_ids[arm_index]
                ].clone()
                body_orientation_xyzw = robot.data.body_quat_w.torch[
                    0, gripper_body_ids[arm_index]
                ].clone()
                inverse_rotation = Gf.Rotation(
                    Gf.Quatd(
                        float(body_orientation_xyzw[3]),
                        Gf.Vec3d(
                            *[float(value) for value in body_orientation_xyzw[:3]]
                        ),
                    )
                ).GetInverse()
                body_position = Gf.Vec3d(
                    *[float(value) for value in body_position_w]
                )
                second_contact_local_positions_by_arm[side] = [
                    inverse_rotation.TransformDir(
                        Gf.Vec3d(
                            *[
                                float(value)
                                for value in second_capture_nodes_w[0, index]
                            ]
                        )
                        - body_position
                    )
                    for index in second_contact_particles_by_arm[side]
                ]
            second_fixed_body_position_w = robot.data.body_pos_w.torch[
                0, gripper_body_ids[0]
            ].clone()
            second_fixed_body_orientation_xyzw = robot.data.body_quat_w.torch[
                0, gripper_body_ids[0]
            ].clone()
            second_contact_local_positions = (
                second_contact_local_positions_by_arm["left"]
            )
            second_kinematic_targets = torch.empty(
                (environment_count, second_capture_nodes_w.shape[1], 4),
                dtype=second_capture_nodes_w.dtype,
                device=second_capture_nodes_w.device,
            )
            second_kinematic_targets[..., :3] = second_capture_nodes_w
            second_kinematic_targets[..., 3] = 1.0
            second_filtered_jaw_flags: dict[int, int] = {}
            second_filtered_jaw_labels: list[str] = []
            if (
                args.second_fold_grasp_mode
                == "contact-gated-retention-filtered"
            ):
                (
                    second_filtered_jaw_flags,
                    second_filtered_jaw_labels,
                ) = set_left_jaw_particle_collision(False)
                print(
                    "S2_RETAINED_TRANSPORT_JAW_PARTICLE_COLLISION_FILTERED "
                    + json.dumps(second_filtered_jaw_labels),
                    flush=True,
                )
            second_physx_attachment_records: list[dict[str, object]] = []
            second_physx_attachment_active = False
            if args.second_fold_grasp_mode == "physx-attachment":
                sim.pause()
                second_attachment_path = author_single_runtime_attachment(
                    environment_index=0,
                    side="left",
                    attachment_name="second_fold_left_gripper_patch",
                    frame_name="TowelSecondFoldAttachmentFrame",
                    nodes_w=second_capture_nodes_w[0],
                    gripper_position_w=second_fixed_body_position_w,
                    gripper_orientation_xyzw=second_fixed_body_orientation_xyzw,
                    selected_indices=second_contact_particles,
                )
                simulation_app.update()
                sim.play()
                second_physx_attachment_records = [
                    create_runtime_attachment_record(
                        attachment_path=second_attachment_path,
                        environment_index=0,
                        side="left",
                    )
                ]
                if second_physx_attachment_records[0]["vertex_indices"] != (
                    second_contact_particles
                ):
                    raise RuntimeError(
                        "second-fold PhysX attachment indices changed while authoring"
                    )
                second_physx_attachment_active = True
                print(
                    "S2_PHYSX_ATTACHMENT_ACTIVATED "
                    + json.dumps(second_physx_attachment_records, sort_keys=True),
                    flush=True,
                )
            second_retention_active = args.second_fold_grasp_mode in {
                "contact-gated-retention",
                "contact-gated-retention-filtered",
            }
            right_stabilizer_retention_active = (
                second_retention_active and second_active_arm == "bimanual"
            )
            right_stabilizer_contact_particles: list[int] = list(
                second_contact_particles_by_arm.get("right", [])
            )
            right_stabilizer_local_positions: list[Gf.Vec3d] = list(
                second_contact_local_positions_by_arm.get("right", [])
            )

            def enforce_second_fold_retention() -> None:
                if not (
                    second_retention_active or right_stabilizer_retention_active
                ):
                    return
                for active, body_slot, particles, local_positions in (
                    (
                        second_retention_active,
                        0,
                        second_contact_particles_by_arm["left"],
                        second_contact_local_positions,
                    ),
                    (
                        right_stabilizer_retention_active,
                        1,
                        right_stabilizer_contact_particles,
                        right_stabilizer_local_positions,
                    ),
                ):
                    if not active:
                        continue
                    body_position = Gf.Vec3d(
                        *[
                            float(value)
                            for value in robot.data.body_pos_w.torch[
                                0, gripper_body_ids[body_slot]
                            ]
                        ]
                    )
                    orientation = robot.data.body_quat_w.torch[
                        0, gripper_body_ids[body_slot]
                    ].tolist()
                    rotation = Gf.Rotation(
                        Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
                    )
                    for index, local_position in zip(
                        particles, local_positions, strict=True
                    ):
                        target_position = body_position + rotation.TransformDir(
                            local_position
                        )
                        second_kinematic_targets[0, index, :3] = torch.tensor(
                            [target_position[axis] for axis in range(3)],
                            dtype=second_kinematic_targets.dtype,
                            device=second_kinematic_targets.device,
                        )
                        second_kinematic_targets[0, index, 3] = 0.0
                cloth.write_nodal_kinematic_target_to_sim_index(
                    second_kinematic_targets
                )

            enforce_second_fold_retention()
            second_fold_phase_records = [
                record
                for record in source["canonical_replay"]["second_fold"]
                if record["name"].startswith(
                    "second_bimanual_fold_"
                    if second_active_arm == "bimanual"
                    else "second_fold_"
                )
            ]
            if [record["name"] for record in second_fold_phase_records] != [
                (
                    f"second_bimanual_fold_{index:02d}"
                    if second_active_arm == "bimanual"
                    else f"second_fold_{index:02d}"
                )
                for index in range(1, len(second_fold_phase_records) + 1)
            ]:
                raise RuntimeError("second-fold transfer phase sequence is incomplete")
            second_phase_steps = max(2, round(args.fold_phase_seconds / physics_dt_s))
            second_current_row = second_pinch_row
            for second_phase in second_fold_phase_records:
                second_target_row = phase_model_tensor(
                    source,
                    second_phase,
                    gripper_project_positions_rad={
                        "left": second_project_target,
                        "right": (
                            second_project_targets["right"]
                            if second_active_arm == "bimanual"
                            else 0.0
                        ),
                    },
                    environment_count=environment_count,
                    device=sim.device,
                )
                for step in range(1, second_phase_steps + 1):
                    alpha = step / second_phase_steps
                    target = second_current_row + alpha * (
                        second_target_row - second_current_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot, target, zero_velocity, joint_ids
                    )
                    enforce_second_fold_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    if not torch.all(torch.isfinite(cloth.data.nodal_pos_w.torch)):
                        raise RuntimeError(
                            "cloth produced non-finite nodes during "
                            f"{second_phase['name']}"
                        )
                second_current_row = second_target_row
                settle_physical_arm_drives(
                    robot,
                    second_target_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    second_phase["name"],
                    post_step_callback=enforce_second_fold_retention,
                )

            for _ in range(
                max(1, round(SECOND_FOLD_PINNED_LAYDOWN_HOLD_S / physics_dt_s))
            ):
                write_scripted_arm_state_and_drive_targets(
                    robot, second_current_row, zero_velocity, joint_ids
                )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            second_nodes_at_laydown = cloth.data.nodal_pos_w.torch.clone()
            second_patch_at_laydown = second_nodes_at_laydown[
                0, second_contact_particles
            ].clone()
            second_laydown_patch_clearances_m = (
                second_patch_at_laydown[:, 2] - table_top_z_m_for_contact_gate
            )
            second_laydown_patch_minimum_table_clearance_m = float(
                torch.min(second_laydown_patch_clearances_m).item()
            )
            second_laydown_patch_median_table_clearance_m = float(
                torch.median(second_laydown_patch_clearances_m).item()
            )
            second_laydown_patch_table_clearance_m = float(
                torch.max(second_laydown_patch_clearances_m).item()
            )
            second_laydown_patch_table_clearance_limit_m = (
                SECOND_FOLD_PHYSX_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M
                if args.second_fold_grasp_mode == "physx-attachment"
                else SECOND_FOLD_ACTUAL_CONTACT_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M
                if args.second_fold_grasp_mode.startswith("contact-gated-retention")
                else SECOND_FOLD_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M
            )
            second_laydown_patch_extent_clearance_limit_m = (
                SECOND_FOLD_BIMANUAL_MAXIMUM_LAYDOWN_PATCH_EXTENT_CLEARANCE_M
                if second_active_arm == "bimanual"
                else SECOND_FOLD_MAXIMUM_LAYDOWN_PATCH_EXTENT_CLEARANCE_M
            )
            if (
                second_laydown_patch_minimum_table_clearance_m
                > second_laydown_patch_table_clearance_limit_m
                or second_laydown_patch_table_clearance_m
                > second_laydown_patch_extent_clearance_limit_m
            ):
                raise RuntimeError(
                    "second-fold laydown gate failed before release: "
                    "patch_clearance_min_m="
                    f"{second_laydown_patch_minimum_table_clearance_m:.6f}, "
                    "median_m="
                    f"{second_laydown_patch_median_table_clearance_m:.6f}, "
                    f"maximum_m={second_laydown_patch_table_clearance_m:.6f}, "
                    "minimum_limit_m="
                    f"{second_laydown_patch_table_clearance_limit_m:.6f}, "
                    "maximum_limit_m="
                    f"{second_laydown_patch_extent_clearance_limit_m:.6f}"
                )
            print(
                "S2_LAYDOWN_TABLE_CONTACT_GATE "
                + json.dumps(
                    {
                        "minimum_patch_table_clearance_m": (
                            second_laydown_patch_minimum_table_clearance_m
                        ),
                        "median_patch_table_clearance_m": (
                            second_laydown_patch_median_table_clearance_m
                        ),
                        "maximum_patch_table_clearance_m": (
                            second_laydown_patch_table_clearance_m
                        ),
                        "minimum_patch_table_clearance_limit_m": (
                            second_laydown_patch_table_clearance_limit_m
                        ),
                        "maximum_patch_table_clearance_limit_m": (
                            second_laydown_patch_extent_clearance_limit_m
                        ),
                        "passed": True,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            second_laydown_minimum = torch.min(
                second_nodes_at_laydown, dim=1
            ).values
            second_laydown_maximum = torch.max(
                second_nodes_at_laydown, dim=1
            ).values
            second_laydown_spans = (
                second_laydown_maximum - second_laydown_minimum
            )
            second_laydown_grid = second_nodes_at_laydown.reshape(
                environment_count, grid_side, grid_side, 3
            )
            second_laydown_edge_residual_m = torch.median(
                second_laydown_grid[:, -1, :, 1], dim=1
            ).values - torch.median(
                second_laydown_grid[:, 0, :, 1], dim=1
            ).values
            print(
                "S2_PINNED_LAYDOWN_SHAPE "
                + json.dumps(
                    {
                        "per_environment_footprint_span_xy_m": (
                            second_laydown_spans[..., :2].tolist()
                        ),
                        "simulation_oracle_moving_minus_stationary_edge_y_m": (
                            second_laydown_edge_residual_m.tolist()
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            second_physx_post_laydown_material_diagnostic = None
            if args.physx_post_laydown_bend_stiffness is not None:
                stage = omni.usd.get_context().get_stage()
                updated_materials = []
                for prim in stage.Traverse():
                    path = str(prim.GetPath())
                    attribute = prim.GetAttribute(
                        "omniphysics:surfaceBendStiffness"
                    )
                    if (
                        "/TowelCloth" in path
                        and attribute.IsValid()
                        and attribute.HasAuthoredValueOpinion()
                    ):
                        previous_value = float(attribute.Get())
                        attribute.Set(args.physx_post_laydown_bend_stiffness)
                        updated_materials.append(
                            {
                                "path": path,
                                "previous_pa": previous_value,
                                "updated_pa": (
                                    args.physx_post_laydown_bend_stiffness
                                ),
                            }
                        )
                if len(updated_materials) != environment_count:
                    raise RuntimeError(
                        "expected one PhysX towel material per environment, found "
                        f"{len(updated_materials)}"
                    )
                simulation_app.update()
                second_physx_post_laydown_material_diagnostic = {
                    "updated_materials": updated_materials,
                    "positions_or_velocities_overwritten": False,
                    "calibration_status": (
                        "high_curvature_operator_evidence_not_bench_calibrated"
                    ),
                }
                print(
                    "S2_PHYSX_POST_LAYDOWN_BEND_STIFFNESS_UPDATED "
                    + json.dumps(
                        second_physx_post_laydown_material_diagnostic,
                        sort_keys=True,
                    ),
                    flush=True,
                )

            second_fold_hysteresis_diagnostic = None
            if args.newton_fold_hysteresis:
                newton_model = NewtonManager.get_model()
                captured_hysteresis_edges = wp.zeros(
                    newton_model.edge_count,
                    dtype=wp.int32,
                    device=args.device,
                )
                wp.launch(
                    capture_newton_high_curvature_rest_angles,
                    dim=newton_model.edge_count,
                    inputs=[
                        NewtonManager.get_state().particle_q,
                        newton_model.edge_indices,
                        math.radians(args.newton_fold_hysteresis_angle_deg),
                        newton_model.edge_rest_angle,
                        captured_hysteresis_edges,
                    ],
                    device=args.device,
                )
                captured_hysteresis_edge_count = int(
                    captured_hysteresis_edges.numpy().sum()
                )
                if captured_hysteresis_edge_count <= 0:
                    raise RuntimeError(
                        "S2 fold hysteresis captured no high-curvature hinges"
                    )
                second_fold_hysteresis_diagnostic = {
                    "enabled": True,
                    "activation_angle_deg": (
                        args.newton_fold_hysteresis_angle_deg
                    ),
                    "captured_edge_count": captured_hysteresis_edge_count,
                    "model_edge_count": int(newton_model.edge_count),
                    "positions_or_velocities_overwritten": False,
                    "calibration_status": (
                        "operator_evidence_only_not_material_bench_calibrated"
                    ),
                }
                print(
                    "S2_HIGH_CURVATURE_FOLD_HYSTERESIS_CAPTURED "
                    + json.dumps(
                        second_fold_hysteresis_diagnostic,
                        sort_keys=True,
                    ),
                    flush=True,
                )

            second_post_laydown_softening_displacement_m = 0.0
            if (
                args.newton_curvature_softening
                and args.newton_curvature_softening_stage == "s2-post-laydown"
            ):
                nodes_before_softening_ramp = cloth.data.nodal_pos_w.torch.clone()
                curvature_softening_runtime["enabled"] = True
                print(
                    "S2_POST_LAYDOWN_HIGH_CURVATURE_SOFTENING_ENABLED "
                    + json.dumps(
                        {
                            "activation_angle_deg": (
                                args.newton_softening_activation_angle_deg
                            ),
                            "full_softening_angle_deg": (
                                args.newton_full_softening_angle_deg
                            ),
                            "softened_edge_stiffness_n_m": (
                                args.newton_softened_edge_stiffness
                            ),
                            "small_bend_stiffness_n_m": (
                                NEWTON_EDGE_STIFFNESS_N_M
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                for _ in range(
                    max(
                        1,
                        round(
                            SECOND_FOLD_POST_LAYDOWN_SOFTENING_RAMP_S
                            / physics_dt_s
                        ),
                    )
                ):
                    write_scripted_arm_state_and_drive_targets(
                        robot, second_current_row, zero_velocity, joint_ids
                    )
                    enforce_second_fold_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                second_post_laydown_softening_displacement_m = float(
                    torch.max(
                        torch.linalg.vector_norm(
                            cloth.data.nodal_pos_w.torch
                            - nodes_before_softening_ramp,
                            dim=-1,
                        )
                    ).item()
                )
                print(
                    "S2_POST_LAYDOWN_HIGH_CURVATURE_SOFTENING_RAMP_COMPLETE "
                    + json.dumps(
                        {
                            "duration_s": (
                                SECOND_FOLD_POST_LAYDOWN_SOFTENING_RAMP_S
                            ),
                            "maximum_cloth_displacement_m": (
                                second_post_laydown_softening_displacement_m
                            ),
                            "ramp_fraction": float(
                                curvature_softening_runtime["ramp_fraction"]
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

            # The nominal release opens the left jaw in place and then retreats
            # vertically.  The bilateral right-arm handoff remains only as a
            # rejected diagnostic.  The surface-press variant instead stops at
            # the first real fixed-pad contact and never closes or attaches the
            # right jaw; it physically suppresses spring-back while the left
            # jaw opens without pretending to grasp the folded bundle.
            right_surface_press_enabled = (
                args.second_fold_release_mode == "right-surface-press"
            )
            right_edge_handoff_enabled = (
                args.second_fold_release_mode == "right-edge-handoff"
            )
            right_stabilizer_enabled = (
                args.second_fold_release_mode
                in {
                    "right-stabilized",
                    "right-surface-press",
                    "right-edge-handoff",
                }
            )
            right_support_replay_name = (
                "second_handoff"
                if right_edge_handoff_enabled
                else "second_stabilizer"
            )
            right_support_phase_prefix = (
                "second_handoff" if right_edge_handoff_enabled else "second_stabilizer"
            )
            right_stabilizer_approach_records = [
                record
                for record in source["canonical_replay"][right_support_replay_name]
                if record["name"].startswith(
                    f"{right_support_phase_prefix}_departure_"
                )
            ] if right_stabilizer_enabled else []
            expected_right_stabilizer_approach_names = [
                f"{right_support_phase_prefix}_departure_{index:02d}_right"
                for index in range(1, 41)
            ] if right_stabilizer_enabled else []
            if [record["name"] for record in right_stabilizer_approach_records] != (
                expected_right_stabilizer_approach_names
            ):
                raise RuntimeError("right S2 stabilizer approach sequence is incomplete")
            nodes_before_right_stabilizer_approach = (
                cloth.data.nodal_pos_w.torch.clone()
            )
            right_stabilizer_current_row = second_current_row
            right_stabilizer_approach_command_duration_s = 0.0
            for right_stabilizer_phase in right_stabilizer_approach_records:
                right_stabilizer_target_row = phase_model_tensor(
                    source,
                    right_stabilizer_phase,
                    gripper_project_positions_rad={
                        "left": second_project_target,
                        "right": 0.0,
                    },
                    environment_count=environment_count,
                    device=sim.device,
                )
                # The stabilizer replay was solved from the canonical clear
                # pose, so preserve every left-arm joint at the S2 laydown.
                right_stabilizer_target_row[:, :6] = second_current_row[:, :6]
                right_stabilizer_joint_chord_rad = float(
                    torch.max(
                        torch.abs(
                            right_stabilizer_target_row[:, 6:11]
                            - right_stabilizer_current_row[:, 6:11]
                        )
                    ).item()
                )
                right_stabilizer_phase_duration_s = max(
                    SECOND_STABILIZER_MINIMUM_PHASE_DURATION_S,
                    right_stabilizer_joint_chord_rad
                    / SECOND_STABILIZER_MAXIMUM_COMMAND_SPEED_RAD_S,
                )
                right_stabilizer_phase_steps = max(
                    2, round(right_stabilizer_phase_duration_s / physics_dt_s)
                )
                right_stabilizer_approach_command_duration_s += (
                    right_stabilizer_phase_steps * physics_dt_s
                )
                for step in range(1, right_stabilizer_phase_steps + 1):
                    alpha = step / right_stabilizer_phase_steps
                    target = right_stabilizer_current_row + alpha * (
                        right_stabilizer_target_row - right_stabilizer_current_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot, target, zero_velocity, joint_ids
                    )
                    enforce_second_fold_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                right_stabilizer_current_row = right_stabilizer_target_row

            settle_physical_arm_drives(
                robot,
                right_stabilizer_current_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                f"{right_support_phase_prefix}_pregrasp_open",
                post_step_callback=enforce_second_fold_retention,
            )
            right_stabilizer_pregrasp_cloth_displacement_m = float(
                torch.max(
                    torch.linalg.vector_norm(
                        cloth.data.nodal_pos_w.torch
                        - nodes_before_right_stabilizer_approach,
                        dim=-1,
                    )
                ).item()
            )
            nodes_before_right_stabilizer_descent = (
                cloth.data.nodal_pos_w.torch.clone()
            )
            if right_stabilizer_enabled:
                right_stabilizer_contact_record = phase(
                    source, f"{right_support_phase_prefix}_contact"
                )
                right_stabilizer_requested_contact_row = phase_model_tensor(
                    source,
                    right_stabilizer_contact_record,
                    gripper_project_positions_rad={
                        "left": second_project_target,
                        "right": 0.0,
                    },
                    environment_count=environment_count,
                    device=sim.device,
                )
                right_stabilizer_requested_contact_row[:, :6] = second_current_row[
                    :, :6
                ]
            else:
                right_stabilizer_requested_contact_row = (
                    right_stabilizer_current_row.clone()
                )
            right_stabilizer_descent_stop_step = (
                None if right_stabilizer_enabled else 0
            )
            right_stabilizer_descent_contact_snapshot = None
            right_stabilizer_contact_descent_steps = max(
                2, round(0.15 / physics_dt_s)
            )
            for step in (
                range(1, right_stabilizer_contact_descent_steps + 1)
                if right_stabilizer_enabled
                else ()
            ):
                alpha = step / right_stabilizer_contact_descent_steps
                target = right_stabilizer_current_row + alpha * (
                    right_stabilizer_requested_contact_row
                    - right_stabilizer_current_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                contact_snapshot = newton_soft_contact_snapshot()
                bilateral = (
                    contact_snapshot.get("bilateral_same_particle_contacts", {})
                    if contact_snapshot is not None
                    else {}
                )
                right_fixed_pad_particles = sorted(
                    {
                        int(index)
                        for label, indices in (
                            contact_snapshot.get("jaw_particles_by_shape", {}).items()
                            if contact_snapshot is not None
                            else ()
                        )
                        if "/right_" in str(label)
                        and "TowelFixedJawCollider" in str(label)
                        for index in indices
                    }
                )
                contact_reached = (
                    bool(right_fixed_pad_particles)
                    if right_surface_press_enabled
                    else bool(bilateral.get("right"))
                )
                if contact_reached:
                    right_stabilizer_descent_stop_step = step
                    right_stabilizer_descent_contact_snapshot = contact_snapshot
                    right_stabilizer_current_row = target.clone()
                    break
            if right_stabilizer_descent_stop_step is None:
                right_stabilizer_current_row = (
                    right_stabilizer_requested_contact_row
                )
            settle_physical_arm_drives(
                robot,
                right_stabilizer_current_row,
                zero_velocity,
                joint_ids,
                scene,
                sim,
                cloth,
                physics_dt_s,
                args.arm_target_settle_timeout_s,
                f"{right_support_phase_prefix}_contact_open",
                post_step_callback=enforce_second_fold_retention,
            )
            right_stabilizer_approach_cloth_displacement_m = float(
                torch.max(
                    torch.linalg.vector_norm(
                        cloth.data.nodal_pos_w.torch
                        - nodes_before_right_stabilizer_approach,
                        dim=-1,
                    )
                ).item()
            )
            right_stabilizer_descent_cloth_displacement_m = float(
                torch.max(
                    torch.linalg.vector_norm(
                        cloth.data.nodal_pos_w.torch
                        - nodes_before_right_stabilizer_descent,
                        dim=-1,
                    )
                ).item()
            )
            print(
                "S2_RIGHT_STABILIZER_APPROACH_GATE "
                + json.dumps(
                    {
                        "maximum_cloth_displacement_m": (
                            right_stabilizer_approach_cloth_displacement_m
                        ),
                        "pregrasp_cloth_displacement_m": (
                            right_stabilizer_pregrasp_cloth_displacement_m
                        ),
                        "contact_descent_cloth_displacement_m": (
                            right_stabilizer_descent_cloth_displacement_m
                        ),
                        "contact_descent_stop_fraction": (
                            right_stabilizer_descent_stop_step
                            / right_stabilizer_contact_descent_steps
                            if right_stabilizer_descent_stop_step is not None
                            else 1.0
                        ),
                        "left_retention_active": second_retention_active,
                        "commanded_route_duration_s": (
                            right_stabilizer_approach_command_duration_s
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

            if right_edge_handoff_enabled:
                right_stabilizer_project_target = float(
                    second_fold_gripper_candidate[
                        "four_layer_project_contact_target_rad"
                    ]["right"]
                )
                right_stabilizer_model_target = float(
                    second_fold_gripper_candidate[
                        "four_layer_model_contact_target_rad"
                    ]["right"]
                )
            else:
                right_stabilizer_project_target = float(
                    gripper_candidate.grasp_project_rad["right"][4]
                )
                right_stabilizer_model_target = float(
                    gripper_candidate.grasp_model_rad("right", 4)
                )
            right_stabilizer_closed_row = right_stabilizer_current_row.clone()
            right_stabilizer_closed_row[:, GRIPPER_JOINT_INDICES[1]] = (
                right_stabilizer_model_target
            )
            nodes_before_right_stabilizer_close = (
                cloth.data.nodal_pos_w.torch.clone()
            )
            right_stabilizer_contact_snapshot = newton_soft_contact_snapshot()
            existing_right_bilateral = (
                right_stabilizer_contact_snapshot.get(
                    "bilateral_same_particle_contacts", {}
                ).get("right", [])
                if right_stabilizer_enabled
                and right_stabilizer_contact_snapshot is not None
                else []
            )
            # S2 folds topology rows 32..63 over rows 0..31.  The contact
            # classifier protects the stationary lower bundle, so its split
            # must be the fold midline rather than an arbitrary four-row
            # strip at the original free edge.  Edge locality is already
            # enforced geometrically by the planned jaw pose and finite pad.
            right_handoff_upper_row_minimum = grid_side // 2
            right_handoff_contact_state = None
            if right_edge_handoff_enabled:
                (
                    right_handoff_fixed_particles,
                    right_handoff_moving_particles,
                ) = newton_jaw_face_contact_particles(
                    right_stabilizer_contact_snapshot, "right"
                )
                right_handoff_contact_state = (
                    classify_opposing_jaw_two_layer_contact(
                        right_handoff_fixed_particles,
                        right_handoff_moving_particles,
                        grid_side=grid_side,
                        upper_row_minimum=right_handoff_upper_row_minimum,
                    )
                )
            right_stabilizer_contact_stop_step = (
                0
                if right_surface_press_enabled
                or (
                    right_edge_handoff_enabled
                    and right_handoff_contact_state.clean_two_layer_pinch
                )
                or (not right_edge_handoff_enabled and existing_right_bilateral)
                or not right_stabilizer_enabled
                else None
            )
            right_stabilizer_requested_closed_row = (
                right_stabilizer_closed_row.clone()
            )
            if right_stabilizer_contact_stop_step == 0:
                right_stabilizer_closed_row = right_stabilizer_current_row.clone()
            for step in (
                range(1, second_close_steps + 1)
                if right_stabilizer_contact_stop_step is None
                else ()
            ):
                alpha = step / second_close_steps
                target = right_stabilizer_current_row + alpha * (
                    right_stabilizer_requested_closed_row
                    - right_stabilizer_current_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot, target, zero_velocity, joint_ids
                )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                contact_snapshot = newton_soft_contact_snapshot()
                bilateral = (
                    contact_snapshot.get("bilateral_same_particle_contacts", {})
                    if contact_snapshot is not None
                    else {}
                )
                contact_reached = bool(bilateral.get("right"))
                if right_edge_handoff_enabled:
                    (
                        right_handoff_fixed_particles,
                        right_handoff_moving_particles,
                    ) = newton_jaw_face_contact_particles(
                        contact_snapshot, "right"
                    )
                    right_handoff_contact_state = (
                        classify_opposing_jaw_two_layer_contact(
                            right_handoff_fixed_particles,
                            right_handoff_moving_particles,
                            grid_side=grid_side,
                            upper_row_minimum=(
                                right_handoff_upper_row_minimum
                            ),
                        )
                    )
                    if right_handoff_contact_state.lower_bundle_particles:
                        raise RuntimeError(
                            "S2 edge handoff touched the stationary lower bundle "
                            "before a clean two-layer pinch: "
                            f"closing_fraction={alpha:.6f}, "
                            f"contact_state={right_handoff_contact_state}"
                        )
                    contact_reached = (
                        right_handoff_contact_state.clean_two_layer_pinch
                    )
                if contact_reached:
                    right_stabilizer_contact_snapshot = contact_snapshot
                    right_stabilizer_contact_stop_step = step
                    right_stabilizer_closed_row = target.clone()
                    break
            if right_stabilizer_contact_stop_step is None:
                right_stabilizer_closed_row = (
                    right_stabilizer_requested_closed_row
                )
            for _ in range(max(2, round(PINCH_HOLD_DURATION_S / physics_dt_s))):
                write_scripted_arm_state_and_drive_targets(
                    robot, right_stabilizer_closed_row, zero_velocity, joint_ids
                )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            require_arm_target_reached(
                robot,
                right_stabilizer_closed_row,
                joint_ids,
                "second_stabilizer_contact_closed",
            )
            right_stabilizer_contact_snapshot = newton_soft_contact_snapshot()
            if right_stabilizer_enabled and right_stabilizer_contact_snapshot is None:
                raise RuntimeError(
                    "right S2 stabilizer requires Newton bilateral contact data"
                )
            right_stabilizer_bilateral = (
                right_stabilizer_contact_snapshot.get(
                    "bilateral_same_particle_contacts", {}
                )
                if right_stabilizer_enabled
                else {}
            )
            if right_edge_handoff_enabled:
                (
                    right_handoff_fixed_particles,
                    right_handoff_moving_particles,
                ) = newton_jaw_face_contact_particles(
                    right_stabilizer_contact_snapshot, "right"
                )
                right_handoff_contact_state = (
                    classify_opposing_jaw_two_layer_contact(
                        right_handoff_fixed_particles,
                        right_handoff_moving_particles,
                        grid_side=grid_side,
                        upper_row_minimum=right_handoff_upper_row_minimum,
                    )
                )
                if not right_handoff_contact_state.clean_two_layer_pinch:
                    raise RuntimeError(
                        "S2 edge handoff actual-contact gate did not preserve "
                        "a clean opposing-jaw pinch across both upper layers: "
                        f"contact_state={right_handoff_contact_state}"
                    )
                right_stabilizer_actual_contact_particles = list(
                    right_handoff_contact_state.upper_bundle_particles
                )
            elif right_surface_press_enabled:
                right_stabilizer_actual_contact_particles = sorted(
                    {
                        int(index)
                        for label, indices in right_stabilizer_contact_snapshot.get(
                            "jaw_particles_by_shape", {}
                        ).items()
                        if "/right_" in str(label)
                        and "TowelFixedJawCollider" in str(label)
                        for index in indices
                    }
                )
            else:
                right_stabilizer_actual_contact_particles = sorted(
                    {
                        int(index)
                        for index in right_stabilizer_bilateral.get("right", [])
                    }
                )
            if right_stabilizer_enabled and not right_stabilizer_actual_contact_particles:
                raise RuntimeError(
                    "right S2 support contact gate failed: "
                    + (
                        "the fixed rubber pad does not contact the towel"
                        if right_surface_press_enabled
                        else "the fixed and moving right jaws do not share a towel particle"
                    )
                )
            right_stabilizer_particle = None
            if right_stabilizer_enabled:
                right_stabilizer_gripper_position_w = (
                    robot.data.body_pos_w.torch[0, gripper_body_ids[1]].clone()
                )
                right_stabilizer_gripper_orientation_xyzw = (
                    robot.data.body_quat_w.torch[0, gripper_body_ids[1]].clone()
                )
                right_stabilizer_tcp_w = gripper_tcp_positions_w(
                    right_stabilizer_gripper_position_w.reshape(1, 1, 3),
                    right_stabilizer_gripper_orientation_xyzw.reshape(1, 1, 4),
                )[0, 0]
                if right_edge_handoff_enabled:
                    for first_column, last_column in (
                        (0, grid_side // 2),
                        (grid_side // 2, grid_side),
                    ):
                        layer_particles = [
                            index
                            for index in right_stabilizer_actual_contact_particles
                            if first_column <= index % grid_side < last_column
                        ]
                        right_stabilizer_contact_particles.append(
                            min(
                                layer_particles,
                                key=lambda index: float(
                                    torch.linalg.vector_norm(
                                        cloth.data.nodal_pos_w.torch[0, index]
                                        - right_stabilizer_tcp_w
                                    ).item()
                                ),
                            )
                        )
                    right_stabilizer_particle = right_stabilizer_contact_particles[0]
                else:
                    right_stabilizer_particle = min(
                        right_stabilizer_actual_contact_particles,
                        key=lambda index: float(
                            torch.linalg.vector_norm(
                                cloth.data.nodal_pos_w.torch[0, index]
                                - right_stabilizer_tcp_w
                            ).item()
                        ),
                    )
                    if right_stabilizer_particle in second_contact_particles:
                        raise RuntimeError(
                            "right S2 stabilizer contacted only a left-retained edge particle"
                        )
                    right_stabilizer_contact_particles.append(
                        right_stabilizer_particle
                    )
                right_stabilizer_inverse_rotation = Gf.Rotation(
                    Gf.Quatd(
                        float(right_stabilizer_gripper_orientation_xyzw[3]),
                        Gf.Vec3d(
                            *[
                                float(value)
                                for value in right_stabilizer_gripper_orientation_xyzw[:3]
                            ]
                        ),
                    )
                ).GetInverse()
                right_stabilizer_body_position = Gf.Vec3d(
                    *[float(value) for value in right_stabilizer_gripper_position_w]
                )
                if not right_surface_press_enabled:
                    right_stabilizer_local_positions.extend(
                        right_stabilizer_inverse_rotation.TransformDir(
                            Gf.Vec3d(
                                *[
                                    float(value)
                                    for value in cloth.data.nodal_pos_w.torch[
                                        0, contact_particle
                                    ]
                                ]
                            )
                            - right_stabilizer_body_position
                        )
                        for contact_particle in right_stabilizer_contact_particles
                    )
                    right_stabilizer_retention_active = True
            enforce_second_fold_retention()
            achieved_right_stabilizer_model_rad = float(
                robot.data.joint_pos.torch[0, gripper_joint_ids[1]].item()
            )
            right_stabilizer_close_cloth_displacement_m = float(
                torch.max(
                    torch.linalg.vector_norm(
                        cloth.data.nodal_pos_w.torch
                        - nodes_before_right_stabilizer_close,
                        dim=-1,
                    )
                ).item()
            )
            print(
                "S2_RIGHT_STABILIZER_CONTACT_GATE "
                + json.dumps(
                    {
                        "actual_bilateral_particles": (
                            right_stabilizer_actual_contact_particles
                            if not right_surface_press_enabled
                            else []
                        ),
                        "actual_fixed_pad_contact_particles": (
                            right_stabilizer_actual_contact_particles
                            if right_surface_press_enabled
                            else []
                        ),
                        "contact_particle": right_stabilizer_particle,
                        "retained_particle": (
                            right_stabilizer_particle
                            if right_stabilizer_retention_active
                            else None
                        ),
                        "right_contact_mode": (
                            "fixed_rubber_pad_surface_press"
                            if right_surface_press_enabled
                            else "bilateral_pinch"
                        ),
                        "retention_created": right_stabilizer_retention_active,
                        "commanded_project_rad": (
                            right_stabilizer_project_target
                        ),
                        "commanded_model_rad": right_stabilizer_model_target,
                        "contact_stop_model_rad": float(
                            right_stabilizer_closed_row[
                                0, GRIPPER_JOINT_INDICES[1]
                            ].item()
                        ),
                        "contact_stop_closing_fraction": (
                            right_stabilizer_contact_stop_step / second_close_steps
                            if right_stabilizer_contact_stop_step is not None
                            else 1.0
                        ),
                        "achieved_model_rad": (
                            achieved_right_stabilizer_model_rad
                        ),
                        "command_is_force_claim": False,
                        "maximum_cloth_displacement_during_approach_m": (
                            right_stabilizer_approach_cloth_displacement_m
                        ),
                        "maximum_cloth_displacement_during_close_m": (
                            right_stabilizer_close_cloth_displacement_m
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

            second_current_row = right_stabilizer_closed_row
            second_open_row = second_current_row.clone()
            second_open_row[:, GRIPPER_JOINT_INDICES[0]] = (
                SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
            )
            if second_active_arm == "bimanual":
                second_open_row[:, GRIPPER_JOINT_INDICES[1]] = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
                )
            second_open_steps = max(2, round(JAW_OPEN_DURATION_S / physics_dt_s))
            second_retention_released_during_open = not (
                second_retention_active
                or right_stabilizer_retention_active
                or second_physx_attachment_active
            )
            for step in range(1, second_open_steps + 1):
                alpha = step / second_open_steps
                target = second_current_row + alpha * (
                    second_open_row - second_current_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    target,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                if (
                    second_retention_active
                    and alpha < SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                ):
                    enforce_second_fold_retention()
                elif second_retention_active:
                    second_kinematic_targets[
                        0, second_contact_particles, 3
                    ] = 1.0
                    cloth.write_nodal_kinematic_target_to_sim_index(
                        second_kinematic_targets
                    )
                    second_retention_active = False
                    if second_active_arm == "bimanual":
                        right_stabilizer_retention_active = False
                    second_retention_released_during_open = True
                    if second_filtered_jaw_flags:
                        set_left_jaw_particle_collision(
                            True, saved_flags=second_filtered_jaw_flags
                        )
                        print(
                            "S2_RELEASE_JAW_PARTICLE_COLLISION_RESTORED",
                            flush=True,
                        )
                    print(
                        "S2_RETENTION_RELEASED_AFTER_MEDIUM_OPEN "
                        + json.dumps(
                            {
                                "opening_travel_fraction": alpha,
                                "minimum_opening_travel_fraction": (
                                    SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                                ),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                elif (
                    second_physx_attachment_active
                    and alpha >= SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                ):
                    disable_runtime_attachments(second_physx_attachment_records)
                    simulation_app.update()
                    if not runtime_attachments_are_disabled(
                        second_physx_attachment_records
                    ):
                        raise RuntimeError(
                            "second-fold PhysX attachment remained enabled after release"
                        )
                    second_physx_attachment_active = False
                    second_retention_released_during_open = True
                    print(
                        "S2_PHYSX_ATTACHMENT_RELEASED_AFTER_MEDIUM_OPEN "
                        + json.dumps(
                            {
                                "opening_travel_fraction": alpha,
                                "minimum_opening_travel_fraction": (
                                    SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                                ),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            if not second_retention_released_during_open:
                raise RuntimeError("second-fold retention was not released during jaw opening")
            for _ in range(max(1, round(POST_OPEN_RELEASE_HOLD_S / physics_dt_s))):
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    second_open_row,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                enforce_second_fold_retention()
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)

            second_release_patch_w = cloth.data.nodal_pos_w.torch[
                0, second_contact_particles
            ].clone()
            second_release_patch_lift_m = float(
                torch.max(
                    second_release_patch_w[:, 2]
                    - second_patch_at_laydown[:, 2]
                ).item()
            )
            second_retreat_records = [
                record
                for record in source["canonical_replay"]["second_fold"]
                if record["name"]
                in (
                    {"second_bimanual_retreat"}
                    if second_active_arm == "bimanual"
                    else {"second_retreat", "second_reobserve_clear"}
                )
            ]
            expected_second_retreat_names = (
                ["second_bimanual_retreat"]
                if second_active_arm == "bimanual"
                else ["second_retreat", "second_reobserve_clear"]
            )
            if [record["name"] for record in second_retreat_records] != (
                expected_second_retreat_names
            ):
                raise RuntimeError("second-fold release retreat sequence is incomplete")
            second_current_row = second_open_row
            for second_retreat_phase in second_retreat_records:
                second_target_row = phase_model_tensor(
                    source,
                    second_retreat_phase,
                    gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                    environment_count=environment_count,
                    device=sim.device,
                )
                second_target_row[:, GRIPPER_JOINT_INDICES[0]] = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
                )
                if second_active_arm == "bimanual":
                    second_target_row[:, GRIPPER_JOINT_INDICES[1]] = (
                        SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD[
                            "right"
                        ]
                    )
                else:
                    second_target_row[:, 6:12] = (
                        right_stabilizer_closed_row[:, 6:12]
                    )
                second_retreat_steps = max(
                    2, round(args.retreat_seconds / physics_dt_s)
                )
                for step in range(1, second_retreat_steps + 1):
                    alpha = step / second_retreat_steps
                    target = second_current_row + alpha * (
                        second_target_row - second_current_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        target,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    enforce_second_fold_retention()
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                second_current_row = second_target_row
                settle_physical_arm_drives(
                    robot,
                    second_target_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    second_retreat_phase["name"],
                    lock_gripper_state=True,
                    post_step_callback=enforce_second_fold_retention,
                )

            second_patch_to_gripper_distance_m_by_arm = {}
            for arm_index, side in enumerate(second_contact_particles_by_arm):
                arm_patch_after_retreat_w = cloth.data.nodal_pos_w.torch[
                    0, second_contact_particles_by_arm[side]
                ]
                arm_gripper_after_retreat_w = robot.data.body_pos_w.torch[
                    0, gripper_body_ids[arm_index]
                ]
                second_patch_to_gripper_distance_m_by_arm[side] = float(
                    torch.min(
                        torch.linalg.vector_norm(
                            arm_patch_after_retreat_w
                            - arm_gripper_after_retreat_w,
                            dim=-1,
                        )
                    ).item()
                )
            second_patch_to_gripper_distance_m = min(
                second_patch_to_gripper_distance_m_by_arm.values()
            )
            stuck_release_arms = [
                side
                for side, distance_m in (
                    second_patch_to_gripper_distance_m_by_arm.items()
                )
                if distance_m
                < SECOND_FOLD_MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
            ]
            if (
                second_release_patch_lift_m
                > SECOND_FOLD_MAXIMUM_RELEASE_PATCH_LIFT_M
                and stuck_release_arms
            ):
                raise RuntimeError(
                    "second-fold release gate failed: "
                    f"patch_lift={second_release_patch_lift_m:.6f} m, "
                    "patch_to_gripper_by_arm="
                    f"{second_patch_to_gripper_distance_m_by_arm}, "
                    f"stuck_arms={stuck_release_arms}"
                )

            right_stabilizer_patch_at_release_w = (
                cloth.data.nodal_pos_w.torch[
                    0, right_stabilizer_contact_particles
                ].clone()
                if right_stabilizer_enabled
                else None
            )
            right_stabilizer_open_row = second_current_row.clone()
            if right_stabilizer_enabled and not right_surface_press_enabled:
                right_stabilizer_open_row[:, GRIPPER_JOINT_INDICES[1]] = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
                )
            right_stabilizer_released_during_open = (
                not right_stabilizer_enabled or right_surface_press_enabled
            )
            for step in (
                range(1, second_open_steps + 1)
                if right_stabilizer_enabled and not right_surface_press_enabled
                else ()
            ):
                alpha = step / second_open_steps
                target = second_current_row + alpha * (
                    right_stabilizer_open_row - second_current_row
                )
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    target,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                if (
                    right_stabilizer_retention_active
                    and alpha < SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                ):
                    enforce_second_fold_retention()
                elif right_stabilizer_retention_active:
                    second_kinematic_targets[
                        0, right_stabilizer_contact_particles, 3
                    ] = 1.0
                    cloth.write_nodal_kinematic_target_to_sim_index(
                        second_kinematic_targets
                    )
                    right_stabilizer_retention_active = False
                    right_stabilizer_released_during_open = True
                    print(
                        "S2_RIGHT_STABILIZER_RELEASED_AFTER_LEFT_CLEAR "
                        + json.dumps(
                            {
                                "opening_travel_fraction": alpha,
                                "minimum_opening_travel_fraction": (
                                    SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                                ),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
            if not right_stabilizer_released_during_open:
                raise RuntimeError("right S2 stabilizer was not released during jaw opening")
            for _ in range(max(1, round(POST_OPEN_RELEASE_HOLD_S / physics_dt_s))):
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    right_stabilizer_open_row,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)

            right_stabilizer_retreat_records = [
                record
                for record in source["canonical_replay"][right_support_replay_name]
                if record["name"]
                in {
                    f"{right_support_phase_prefix}_retreat",
                    f"{right_support_phase_prefix}_reobserve_clear",
                }
            ] if right_stabilizer_enabled else []
            if [record["name"] for record in right_stabilizer_retreat_records] != [
                *(
                    (
                        f"{right_support_phase_prefix}_retreat",
                        f"{right_support_phase_prefix}_reobserve_clear",
                    )
                    if right_stabilizer_enabled
                    else ()
                )
            ]:
                raise RuntimeError("right S2 stabilizer retreat sequence is incomplete")
            second_current_row = right_stabilizer_open_row
            for right_stabilizer_retreat_phase in right_stabilizer_retreat_records:
                right_stabilizer_target_row = phase_model_tensor(
                    source,
                    right_stabilizer_retreat_phase,
                    gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                    environment_count=environment_count,
                    device=sim.device,
                )
                right_stabilizer_target_row[:, :6] = second_current_row[:, :6]
                right_stabilizer_target_row[:, GRIPPER_JOINT_INDICES[0]] = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
                )
                right_stabilizer_target_row[:, GRIPPER_JOINT_INDICES[1]] = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
                )
                right_stabilizer_retreat_steps = max(
                    2, round(args.retreat_seconds / physics_dt_s)
                )
                for step in range(1, right_stabilizer_retreat_steps + 1):
                    alpha = step / right_stabilizer_retreat_steps
                    target = second_current_row + alpha * (
                        right_stabilizer_target_row - second_current_row
                    )
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
                second_current_row = right_stabilizer_target_row
                settle_physical_arm_drives(
                    robot,
                    right_stabilizer_target_row,
                    zero_velocity,
                    joint_ids,
                    scene,
                    sim,
                    cloth,
                    physics_dt_s,
                    args.arm_target_settle_timeout_s,
                    right_stabilizer_retreat_phase["name"],
                    lock_gripper_state=True,
                )

            right_stabilizer_release_patch_lift_m = None
            right_stabilizer_patch_to_gripper_distance_m = None
            if right_stabilizer_enabled:
                right_stabilizer_patch_after_retreat_w = cloth.data.nodal_pos_w.torch[
                    0, right_stabilizer_contact_particles
                ]
                right_stabilizer_gripper_after_retreat_w = robot.data.body_pos_w.torch[
                    0, gripper_body_ids[1]
                ]
                right_stabilizer_release_patch_lift_m = float(
                    torch.max(
                        right_stabilizer_patch_after_retreat_w[:, 2]
                        - right_stabilizer_patch_at_release_w[:, 2]
                    ).item()
                )
                right_stabilizer_patch_to_gripper_distance_m = float(
                    torch.min(
                        torch.linalg.vector_norm(
                            right_stabilizer_patch_after_retreat_w
                            - right_stabilizer_gripper_after_retreat_w,
                            dim=-1,
                        )
                    ).item()
                )
                if (
                    right_stabilizer_release_patch_lift_m
                    > SECOND_FOLD_MAXIMUM_RELEASE_PATCH_LIFT_M
                    and right_stabilizer_patch_to_gripper_distance_m
                    < SECOND_FOLD_MINIMUM_RELEASE_PATCH_TO_JAW_DISTANCE_M
                ):
                    raise RuntimeError(
                        "right S2 stabilizer release gate failed: "
                        f"patch_lift={right_stabilizer_release_patch_lift_m:.6f} m, "
                        "patch_to_gripper="
                        f"{right_stabilizer_patch_to_gripper_distance_m:.6f} m"
                    )

            if second_fold_correction_checkpoint_local is not None:
                correction_checkpoint_local = torch.tensor(
                    second_fold_correction_checkpoint_local,
                    dtype=cloth.data.nodal_pos_w.torch.dtype,
                    device=sim.device,
                )
                correction_checkpoint_position_w = (
                    correction_checkpoint_local.unsqueeze(0)
                    + scene.env_origins[:, None, :]
                )
                correction_checkpoint_velocity_w = torch.zeros_like(
                    correction_checkpoint_position_w
                )
                correction_checkpoint_state_w = torch.cat(
                    (
                        correction_checkpoint_position_w,
                        correction_checkpoint_velocity_w,
                    ),
                    dim=-1,
                )
                cloth.write_nodal_state_to_sim_index(
                    correction_checkpoint_state_w
                )
                correction_checkpoint_error_m = float(
                    torch.max(
                        torch.linalg.vector_norm(
                            local_nodes(scene, cloth)
                            - correction_checkpoint_local.unsqueeze(0),
                            dim=-1,
                        )
                    ).item()
                )
                if correction_checkpoint_error_m > 1.0e-6:
                    raise RuntimeError(
                        "raw S2 correction checkpoint write failed: "
                        f"maximum_error={correction_checkpoint_error_m:.9f} m"
                    )
                second_fold_correction_checkpoint = {
                    **second_fold_correction_checkpoint_source,
                    "maximum_write_error_m": correction_checkpoint_error_m,
                    "velocity_reset_to_zero": True,
                    "purpose": "isolate_camera_correction_from_raw_S2_nondeterminism",
                    "end_to_end_result_claimed": False,
                }
                print(
                    "S2_CORRECTION_RAW_CHECKPOINT_RESTORED "
                    + json.dumps(
                        second_fold_correction_checkpoint, sort_keys=True
                    ),
                    flush=True,
                )

            second_settled_run = 0
            second_settled_step = None
            second_settle_samples: list[dict[str, float]] = []
            second_settle_sample_steps = max(
                1, round((1.0 / RELEASE_SHAPE_VIDEO_FPS) / physics_dt_s)
            )
            second_previous_sample_nodes = local_nodes(scene, cloth).clone()
            second_previous_edge_residual_m = None
            for settle_step in range(
                1, math.ceil(SECOND_FOLD_SETTLE_TIMEOUT_S / physics_dt_s) + 1
            ):
                write_scripted_arm_state_and_drive_targets(
                    robot,
                    second_current_row,
                    zero_velocity,
                    joint_ids,
                    lock_gripper_state=True,
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(physics_dt_s)
                second_speed = float(
                    torch.max(
                        torch.linalg.vector_norm(
                            cloth.data.nodal_vel_w.torch, dim=-1
                        )
                    ).item()
                )
                if settle_step % second_settle_sample_steps == 0:
                    second_current_sample_nodes = local_nodes(scene, cloth).clone()
                    second_sample_displacement = torch.linalg.vector_norm(
                        second_current_sample_nodes - second_previous_sample_nodes,
                        dim=-1,
                    )
                    second_p95_sample_displacement_m = float(
                        torch.max(
                            torch.quantile(
                                second_sample_displacement, 0.95, dim=1
                            )
                        ).item()
                    )
                    second_sample_grid = second_current_sample_nodes.reshape(
                        environment_count, grid_side, grid_side, 3
                    )
                    second_edge_residual_sample_m = float(
                        (
                            torch.median(second_sample_grid[0, -1, :, 1])
                            - torch.median(second_sample_grid[0, 0, :, 1])
                        ).item()
                    )
                    second_edge_residual_change_m = (
                        abs(
                            second_edge_residual_sample_m
                            - second_previous_edge_residual_m
                        )
                        if second_previous_edge_residual_m is not None
                        else None
                    )
                    second_settle_samples.append(
                        {
                            "time_s": settle_step * physics_dt_s,
                            "p95_shape_displacement_m": (
                                second_p95_sample_displacement_m
                            ),
                            "edge_residual_m": second_edge_residual_sample_m,
                            "edge_residual_change_m": (
                                second_edge_residual_change_m
                            ),
                            "maximum_node_speed_m_s": second_speed,
                        }
                    )
                    sample_stable = (
                        second_p95_sample_displacement_m
                        <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                        and second_edge_residual_change_m is not None
                        and second_edge_residual_change_m
                        <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                    )
                    second_settled_run = (
                        second_settled_run + 1 if sample_stable else 0
                    )
                    second_previous_sample_nodes = second_current_sample_nodes
                    second_previous_edge_residual_m = (
                        second_edge_residual_sample_m
                    )
                    if (
                        second_settled_run
                        >= RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                    ):
                        second_settled_step = settle_step
                        break
            second_settle_gate_passed = second_settled_step is not None

            if second_fold_correction_replay is not None:
                if not second_settle_gate_passed:
                    raise RuntimeError(
                        "second-fold correction requires a camera-stable raw S2 "
                        f"shape; last_sample={second_settle_samples[-1]}"
                    )
                correction_grid_before = local_nodes(scene, cloth).reshape(
                    environment_count, grid_side, grid_side, 3
                )
                correction_moving_y_before = float(
                    torch.median(correction_grid_before[0, -1, :, 1]).item()
                )
                correction_stationary_y_before = float(
                    torch.median(correction_grid_before[0, 0, :, 1]).item()
                )
                correction_residual_before = (
                    correction_moving_y_before - correction_stationary_y_before
                )
                planned_observation = second_fold_correction_replay.get(
                    "geometry_oracle_observation", {}
                )
                planned_residual = float(
                    planned_observation["moving_free_edge_y_m"]
                ) - float(planned_observation["stationary_right_edge_y_m"])
                if (
                    abs(correction_residual_before - planned_residual)
                    > SECOND_FOLD_CORRECTION_OBSERVATION_TOLERANCE_M
                    or correction_residual_before * planned_residual <= 0.0
                ):
                    raise RuntimeError(
                        "fresh S2 edge observation no longer matches the correction "
                        f"plan: current={correction_residual_before:.6f} m, "
                        f"planned={planned_residual:.6f} m"
                    )
                print(
                    "S2_CORRECTION_FRESH_OBSERVATION_GATE "
                    + json.dumps(
                        {
                            "moving_free_edge_y_m": correction_moving_y_before,
                            "stationary_right_edge_y_m": correction_stationary_y_before,
                            "signed_residual_m": correction_residual_before,
                            "planned_signed_residual_m": planned_residual,
                            "maximum_plan_difference_m": (
                                SECOND_FOLD_CORRECTION_OBSERVATION_TOLERANCE_M
                            ),
                            "settled": True,
                            "arms_clear": True,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

                correction_contact_index = next(
                    index
                    for index, record in enumerate(second_fold_correction_records)
                    if record["name"] == "second_correction_contact"
                )
                correction_laydown_index = next(
                    index
                    for index, record in enumerate(second_fold_correction_records)
                    if record["name"] == "second_correction_laydown"
                )
                correction_current_row = second_current_row
                correction_nodes_before_approach = (
                    cloth.data.nodal_pos_w.torch.clone()
                )

                def execute_second_correction_phase(
                    record: dict[str, object],
                    *,
                    gripper_model_rad: float,
                    retention_callback: object | None = None,
                ) -> torch.Tensor:
                    nonlocal correction_current_row
                    target_row = phase_model_tensor(
                        source,
                        record,
                        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                        environment_count=environment_count,
                        device=sim.device,
                    )
                    target_row[:, :6] = correction_current_row[:, :6]
                    target_row[:, GRIPPER_JOINT_INDICES[0]] = (
                        SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
                    )
                    target_row[:, GRIPPER_JOINT_INDICES[1]] = gripper_model_rad
                    right_joint_chord_rad = float(
                        torch.max(
                            torch.abs(
                                target_row[:, 6:11]
                                - correction_current_row[:, 6:11]
                            )
                        ).item()
                    )
                    duration_s = max(
                        SECOND_STABILIZER_MINIMUM_PHASE_DURATION_S,
                        right_joint_chord_rad
                        / SECOND_STABILIZER_MAXIMUM_COMMAND_SPEED_RAD_S,
                    )
                    phase_steps = max(2, round(duration_s / physics_dt_s))
                    for phase_step in range(1, phase_steps + 1):
                        alpha = phase_step / phase_steps
                        target = correction_current_row + alpha * (
                            target_row - correction_current_row
                        )
                        write_scripted_arm_state_and_drive_targets(
                            robot,
                            target,
                            zero_velocity,
                            joint_ids,
                            lock_gripper_state=True,
                        )
                        if retention_callback is not None:
                            retention_callback()
                        scene.write_data_to_sim()
                        sim.step()
                        scene.update(physics_dt_s)
                        if not torch.all(
                            torch.isfinite(cloth.data.nodal_pos_w.torch)
                        ):
                            raise RuntimeError(
                                "cloth produced non-finite nodes during S2 correction "
                                f"{record['name']}"
                            )
                    correction_current_row = target_row
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
                        str(record["name"]),
                        post_step_callback=retention_callback,
                        lock_gripper_state=True,
                    )
                    return target_row

                correction_open_model_rad = (
                    SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["right"]
                )
                for correction_record in second_fold_correction_records[
                    : correction_contact_index + 1
                ]:
                    execute_second_correction_phase(
                        correction_record,
                        gripper_model_rad=correction_open_model_rad,
                    )

                correction_project_target = float(
                    second_fold_gripper_candidate[
                        "four_layer_project_contact_target_rad"
                    ]["right"]
                )
                correction_model_target = float(
                    second_fold_gripper_candidate[
                        "four_layer_model_contact_target_rad"
                    ]["right"]
                )
                correction_open_contact_row = correction_current_row.clone()
                if correction_is_open_jaw_edge_push:
                    # The complete push is the old "translate" tail target.
                    # Interpolate to it with the jaw held fully open so contact
                    # is physical and no cloth particle is kinematically retained.
                    correction_requested_pinch_row = phase_model_tensor(
                        source,
                        second_fold_correction_records[
                            correction_contact_index + 2
                        ],
                        gripper_project_positions_rad={"left": 0.0, "right": 0.0},
                        environment_count=environment_count,
                        device=sim.device,
                    )
                    correction_requested_pinch_row[:, :6] = (
                        correction_current_row[:, :6]
                    )
                    correction_requested_pinch_row[
                        :, GRIPPER_JOINT_INDICES[0]
                    ] = SIMULATION_RELEASE_MODEL_GRIPPER_JOINT_POSITIONS_RAD["left"]
                    correction_requested_pinch_row[
                        :, GRIPPER_JOINT_INDICES[1]
                    ] = correction_open_model_rad
                    correction_project_target = None
                    correction_model_target = correction_open_model_rad
                else:
                    correction_requested_pinch_row = correction_current_row.clone()
                    correction_requested_pinch_row[:, GRIPPER_JOINT_INDICES[1]] = (
                        correction_model_target
                    )
                correction_nodes_before_close = cloth.data.nodal_pos_w.torch.clone()
                correction_close_steps = max(
                    2, round(PINCH_CLOSE_DURATION_S / physics_dt_s)
                )
                # The complete moved S2 half is the upper bundle.  Restricting
                # this to the last four topology rows falsely labels a valid
                # inward contact on a curled/arched upper edge as stationary
                # lower-cloth contact (for example row 55 of a 64x64 cloth).
                correction_upper_row_minimum = grid_side // 2
                correction_contact_stop_step = None
                correction_contact_snapshot = None
                correction_bilateral_particles: list[int] = []
                correction_fixed_face_particles: list[int] = []
                correction_moving_face_particles: list[int] = []
                correction_upper_particles: list[int] = []
                correction_lower_particles: list[int] = []
                correction_opposing_assignment = None
                correction_contacted_s1_layers = {
                    "first_half": False,
                    "second_half": False,
                }
                correction_last_contact_state = None
                for correction_close_step in range(1, correction_close_steps + 1):
                    alpha = correction_close_step / correction_close_steps
                    target = correction_open_contact_row + alpha * (
                        correction_requested_pinch_row - correction_open_contact_row
                    )
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
                    step_snapshot = newton_soft_contact_snapshot()
                    step_bilateral = step_snapshot.get(
                        "bilateral_same_particle_contacts", {}
                    ).get("right", [])
                    step_bilateral_particles = sorted(
                        {int(index) for index in step_bilateral}
                    )
                    (
                        step_fixed_face_particles,
                        step_moving_face_particles,
                    ) = newton_jaw_face_contact_particles(step_snapshot, "right")
                    if (
                        correction_is_open_jaw_edge_push
                        and step_moving_face_particles
                    ):
                        raise RuntimeError(
                            "S2 fixed-pad edge push touched cloth with the long "
                            "moving jaw: "
                            f"motion_fraction={alpha:.6f}, "
                            f"particles={step_moving_face_particles}"
                        )
                    step_contact_state = classify_opposing_jaw_two_layer_contact(
                        step_fixed_face_particles,
                        step_moving_face_particles,
                        grid_side=grid_side,
                        upper_row_minimum=correction_upper_row_minimum,
                    )
                    correction_last_contact_state = step_contact_state
                    step_upper_particles = list(
                        step_contact_state.upper_bundle_particles
                    )
                    step_lower_particles = list(
                        step_contact_state.lower_bundle_particles
                    )
                    step_contacted_s1_layers = {
                        "first_half": (
                            step_contact_state.fixed_contacts_first_half
                            or step_contact_state.moving_contacts_first_half
                        ),
                        "second_half": (
                            step_contact_state.fixed_contacts_second_half
                            or step_contact_state.moving_contacts_second_half
                        ),
                    }
                    if step_lower_particles:
                        raise RuntimeError(
                            "S2 correction touched the stationary lower bundle: "
                            f"closing_fraction={alpha:.6f}, "
                            f"lower={step_lower_particles}, "
                            f"upper={step_upper_particles}"
                        )
                    if correction_is_open_jaw_edge_push:
                        correction_current_row = target.clone()
                        if step_upper_particles:
                            # Retain the latest physical pusher contact, but do
                            # not stop the commanded bounded push early.
                            correction_contact_stop_step = correction_close_step
                            correction_contact_snapshot = step_snapshot
                            correction_bilateral_particles = step_bilateral_particles
                            correction_fixed_face_particles = (
                                step_fixed_face_particles
                            )
                            correction_moving_face_particles = (
                                step_moving_face_particles
                            )
                            correction_upper_particles = step_upper_particles
                            correction_lower_particles = step_lower_particles
                            correction_opposing_assignment = "open_jaw_edge_push"
                            correction_contacted_s1_layers = (
                                step_contacted_s1_layers
                            )
                    elif step_contact_state.clean_two_layer_pinch:
                        correction_contact_stop_step = correction_close_step
                        correction_current_row = target.clone()
                        correction_contact_snapshot = step_snapshot
                        correction_bilateral_particles = step_bilateral_particles
                        correction_fixed_face_particles = (
                            step_fixed_face_particles
                        )
                        correction_moving_face_particles = (
                            step_moving_face_particles
                        )
                        correction_upper_particles = step_upper_particles
                        correction_lower_particles = step_lower_particles
                        correction_opposing_assignment = (
                            step_contact_state.opposing_assignment
                        )
                        correction_contacted_s1_layers = (
                            step_contacted_s1_layers
                        )
                        break
                if correction_contact_stop_step is None:
                    raise RuntimeError(
                        "S2 correction did not obtain the required upper-edge contact: "
                        f"last_contact_state={correction_last_contact_state}"
                    )
                for _ in range(max(2, round(PINCH_HOLD_DURATION_S / physics_dt_s))):
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        correction_current_row,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                require_arm_target_reached(
                    robot,
                    correction_current_row,
                    joint_ids,
                    "second_correction_contact_closed",
                )

                correction_contact_snapshot = newton_soft_contact_snapshot()
                correction_bilateral = correction_contact_snapshot.get(
                    "bilateral_same_particle_contacts", {}
                ).get("right", [])
                correction_bilateral_particles = sorted(
                    {int(index) for index in correction_bilateral}
                )
                (
                    correction_fixed_face_particles,
                    correction_moving_face_particles,
                ) = newton_jaw_face_contact_particles(
                    correction_contact_snapshot, "right"
                )
                correction_contact_state = classify_opposing_jaw_two_layer_contact(
                    correction_fixed_face_particles,
                    correction_moving_face_particles,
                    grid_side=grid_side,
                    upper_row_minimum=correction_upper_row_minimum,
                )
                correction_upper_particles = list(
                    correction_contact_state.upper_bundle_particles
                )
                correction_lower_particles = list(
                    correction_contact_state.lower_bundle_particles
                )
                correction_opposing_assignment = (
                    "open_jaw_fixed_pad_edge_push"
                    if correction_is_open_jaw_edge_push
                    else correction_contact_state.opposing_assignment
                )
                if (
                    correction_is_open_jaw_edge_push
                    and correction_moving_face_particles
                ):
                    raise RuntimeError(
                        "S2 fixed-pad edge push ended with moving-jaw cloth contact: "
                        f"particles={correction_moving_face_particles}"
                    )
                if correction_lower_particles:
                    raise RuntimeError(
                        "S2 correction clean contact did not survive the hold; "
                        "stationary lower-bundle particles appeared: "
                        f"lower={correction_lower_particles}, "
                        f"fixed_upper={correction_contact_state.fixed_upper_particles}, "
                        f"moving_upper={correction_contact_state.moving_upper_particles}"
                    )
                correction_contacted_s1_layers = {
                    "first_half": (
                        correction_contact_state.fixed_contacts_first_half
                        or correction_contact_state.moving_contacts_first_half
                    ),
                    "second_half": (
                        correction_contact_state.fixed_contacts_second_half
                        or correction_contact_state.moving_contacts_second_half
                    ),
                }
                if (
                    correction_is_open_jaw_edge_push
                    and not correction_fixed_face_particles
                ):
                    raise RuntimeError(
                        "S2 open-jaw correction lost fixed-pad upper-edge contact "
                        "at the end of the bounded push"
                    )
                if (
                    not correction_is_open_jaw_edge_push
                    and not correction_contact_state.clean_two_layer_pinch
                ):
                    raise RuntimeError(
                        "S2 correction actual-contact gate did not preserve an "
                        "opposing-jaw pinch across both upper S1 layers: "
                        f"contact_state={correction_contact_state}"
                    )

                correction_gripper_position_w = robot.data.body_pos_w.torch[
                    0, gripper_body_ids[1]
                ].clone()
                correction_gripper_orientation_xyzw = robot.data.body_quat_w.torch[
                    0, gripper_body_ids[1]
                ].clone()
                correction_tcp_w = gripper_tcp_positions_w(
                    correction_gripper_position_w.reshape(1, 1, 3),
                    correction_gripper_orientation_xyzw.reshape(1, 1, 4),
                )[0, 0]
                correction_contact_particles: list[int] = []
                if correction_is_open_jaw_edge_push:
                    correction_contact_particles.append(
                        min(
                            correction_upper_particles,
                            key=lambda index: float(
                                torch.linalg.vector_norm(
                                    cloth.data.nodal_pos_w.torch[0, index]
                                    - correction_tcp_w
                                ).item()
                            ),
                        )
                    )
                else:
                    for first_column, last_column in (
                        (0, grid_side // 2),
                        (grid_side // 2, grid_side),
                    ):
                        layer_particles = [
                            index
                            for index in correction_upper_particles
                            if first_column <= index % grid_side < last_column
                        ]
                        correction_contact_particles.append(
                            min(
                                layer_particles,
                                key=lambda index: float(
                                    torch.linalg.vector_norm(
                                        cloth.data.nodal_pos_w.torch[0, index]
                                        - correction_tcp_w
                                    ).item()
                                ),
                            )
                        )
                correction_inverse_rotation = Gf.Rotation(
                    Gf.Quatd(
                        float(correction_gripper_orientation_xyzw[3]),
                        Gf.Vec3d(
                            *[
                                float(value)
                                for value in correction_gripper_orientation_xyzw[:3]
                            ]
                        ),
                    )
                ).GetInverse()
                correction_body_position = Gf.Vec3d(
                    *[float(value) for value in correction_gripper_position_w]
                )
                correction_local_positions = [
                    correction_inverse_rotation.TransformDir(
                        Gf.Vec3d(
                            *[
                                float(value)
                                for value in cloth.data.nodal_pos_w.torch[0, index]
                            ]
                        )
                        - correction_body_position
                    )
                    for index in correction_contact_particles
                ]
                correction_kinematic_targets = torch.empty(
                    (environment_count, grid_side * grid_side, 4),
                    dtype=cloth.data.nodal_pos_w.torch.dtype,
                    device=sim.device,
                )
                correction_kinematic_targets[..., :3] = (
                    cloth.data.nodal_pos_w.torch
                )
                correction_kinematic_targets[..., 3] = 1.0
                correction_retention_active = not correction_is_open_jaw_edge_push

                def enforce_second_fold_correction_retention() -> None:
                    if not correction_retention_active:
                        return
                    body_position = Gf.Vec3d(
                        *[
                            float(value)
                            for value in robot.data.body_pos_w.torch[
                                0, gripper_body_ids[1]
                            ]
                        ]
                    )
                    orientation = robot.data.body_quat_w.torch[
                        0, gripper_body_ids[1]
                    ].tolist()
                    rotation = Gf.Rotation(
                        Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3]))
                    )
                    for index, local_position in zip(
                        correction_contact_particles,
                        correction_local_positions,
                        strict=True,
                    ):
                        target_position = body_position + rotation.TransformDir(
                            local_position
                        )
                        correction_kinematic_targets[0, index, :3] = torch.tensor(
                            [target_position[axis] for axis in range(3)],
                            dtype=correction_kinematic_targets.dtype,
                            device=correction_kinematic_targets.device,
                        )
                        correction_kinematic_targets[0, index, 3] = 0.0
                    cloth.write_nodal_kinematic_target_to_sim_index(
                        correction_kinematic_targets
                    )

                enforce_second_fold_correction_retention()
                correction_close_displacement_m = float(
                    torch.max(
                        torch.linalg.vector_norm(
                            cloth.data.nodal_pos_w.torch - correction_nodes_before_close,
                            dim=-1,
                        )
                    ).item()
                )
                print(
                    "S2_CORRECTION_ACTUAL_CONTACT_GATE "
                    + json.dumps(
                        {
                            "bilateral_particles": correction_bilateral_particles,
                            "fixed_rubber_face_particles": (
                                correction_fixed_face_particles
                            ),
                            "moving_jaw_face_particles": (
                                correction_moving_face_particles
                            ),
                            "upper_bundle_particles": correction_upper_particles,
                            "retained_particles_one_per_s1_layer": (
                                correction_contact_particles
                            ),
                            "contacted_s1_topology_halves": (
                                correction_contacted_s1_layers
                            ),
                            "opposing_face_assignment": (
                                correction_opposing_assignment
                            ),
                            "commanded_project_rad": correction_project_target,
                            "commanded_model_rad": correction_model_target,
                            "contact_stop_closing_fraction": (
                                correction_contact_stop_step
                                / correction_close_steps
                            ),
                            "contact_stop_model_rad": float(
                                correction_current_row[
                                    0, GRIPPER_JOINT_INDICES[1]
                                ].item()
                            ),
                            "maximum_cloth_displacement_during_close_m": (
                                correction_close_displacement_m
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

                correction_nodes_before_transport = (
                    cloth.data.nodal_pos_w.torch.clone()
                )
                correction_transport_records = (
                    []
                    if correction_is_open_jaw_edge_push
                    else second_fold_correction_records[
                        correction_contact_index + 1 : correction_laydown_index + 1
                    ]
                )
                for correction_record in correction_transport_records:
                    execute_second_correction_phase(
                        correction_record,
                        gripper_model_rad=correction_model_target,
                        retention_callback=enforce_second_fold_correction_retention,
                    )
                correction_patch_at_laydown_w = cloth.data.nodal_pos_w.torch[
                    0, correction_contact_particles
                ].clone()
                correction_laydown_clearance_m = float(
                    torch.max(
                        correction_patch_at_laydown_w[:, 2]
                        - table_top_z_m_for_contact_gate
                    ).item()
                )
                if (
                    not correction_is_open_jaw_edge_push
                    and
                    correction_laydown_clearance_m
                    > SECOND_FOLD_MAXIMUM_LAYDOWN_PATCH_TABLE_CLEARANCE_M
                ):
                    raise RuntimeError(
                        "S2 correction laydown remained airborne: "
                        f"clearance={correction_laydown_clearance_m:.6f} m"
                    )

                correction_open_row = correction_current_row.clone()
                correction_open_row[:, GRIPPER_JOINT_INDICES[1]] = (
                    correction_open_model_rad
                )
                correction_released_during_open = correction_is_open_jaw_edge_push
                for correction_open_step in range(1, second_open_steps + 1):
                    alpha = correction_open_step / second_open_steps
                    target = correction_current_row + alpha * (
                        correction_open_row - correction_current_row
                    )
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        target,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    if (
                        correction_retention_active
                        and alpha < SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                    ):
                        enforce_second_fold_correction_retention()
                    elif correction_retention_active:
                        correction_kinematic_targets[
                            0, correction_contact_particles, 3
                        ] = 1.0
                        cloth.write_nodal_kinematic_target_to_sim_index(
                            correction_kinematic_targets
                        )
                        correction_retention_active = False
                        correction_released_during_open = True
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                if not correction_released_during_open:
                    raise RuntimeError("S2 correction retention did not release")
                correction_current_row = correction_open_row
                for _ in range(max(1, round(POST_OPEN_RELEASE_HOLD_S / physics_dt_s))):
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        correction_current_row,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)

                for correction_record in second_fold_correction_records[
                    correction_laydown_index + 1 :
                ]:
                    execute_second_correction_phase(
                        correction_record,
                        gripper_model_rad=correction_open_model_rad,
                    )
                second_current_row = correction_current_row

                correction_settled_run = 0
                correction_settled_step = None
                correction_settle_samples: list[dict[str, object]] = []
                correction_previous_sample_nodes = local_nodes(scene, cloth).clone()
                correction_previous_edge_residual_m = None
                for correction_settle_step in range(
                    1, math.ceil(SECOND_FOLD_SETTLE_TIMEOUT_S / physics_dt_s) + 1
                ):
                    write_scripted_arm_state_and_drive_targets(
                        robot,
                        second_current_row,
                        zero_velocity,
                        joint_ids,
                        lock_gripper_state=True,
                    )
                    scene.write_data_to_sim()
                    sim.step()
                    scene.update(physics_dt_s)
                    correction_speed = float(
                        torch.max(
                            torch.linalg.vector_norm(
                                cloth.data.nodal_vel_w.torch, dim=-1
                            )
                        ).item()
                    )
                    if correction_settle_step % second_settle_sample_steps == 0:
                        correction_current_sample_nodes = local_nodes(
                            scene, cloth
                        ).clone()
                        correction_sample_displacement = torch.linalg.vector_norm(
                            correction_current_sample_nodes
                            - correction_previous_sample_nodes,
                            dim=-1,
                        )
                        correction_p95_sample_displacement_m = float(
                            torch.max(
                                torch.quantile(
                                    correction_sample_displacement, 0.95, dim=1
                                )
                            ).item()
                        )
                        correction_sample_grid = (
                            correction_current_sample_nodes.reshape(
                                environment_count, grid_side, grid_side, 3
                            )
                        )
                        correction_edge_residual_sample_m = float(
                            (
                                torch.median(
                                    correction_sample_grid[0, -1, :, 1]
                                )
                                - torch.median(
                                    correction_sample_grid[0, 0, :, 1]
                                )
                            ).item()
                        )
                        correction_edge_residual_change_m = (
                            abs(
                                correction_edge_residual_sample_m
                                - correction_previous_edge_residual_m
                            )
                            if correction_previous_edge_residual_m is not None
                            else None
                        )
                        correction_settle_samples.append(
                            {
                                "time_s": correction_settle_step * physics_dt_s,
                                "p95_shape_displacement_m": (
                                    correction_p95_sample_displacement_m
                                ),
                                "edge_residual_m": (
                                    correction_edge_residual_sample_m
                                ),
                                "edge_residual_change_m": (
                                    correction_edge_residual_change_m
                                ),
                                "maximum_node_speed_m_s": correction_speed,
                            }
                        )
                        sample_stable = (
                            correction_p95_sample_displacement_m
                            <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                            and correction_edge_residual_change_m is not None
                            and correction_edge_residual_change_m
                            <= RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                        )
                        correction_settled_run = (
                            correction_settled_run + 1 if sample_stable else 0
                        )
                        correction_previous_sample_nodes = (
                            correction_current_sample_nodes
                        )
                        correction_previous_edge_residual_m = (
                            correction_edge_residual_sample_m
                        )
                        if (
                            correction_settled_run
                            >= RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                        ):
                            correction_settled_step = correction_settle_step
                            break
                if correction_settled_step is None:
                    raise RuntimeError("cloth did not settle after S2 correction")
                correction_grid_after = local_nodes(scene, cloth).reshape(
                    environment_count, grid_side, grid_side, 3
                )
                correction_residual_after = float(
                    (
                        torch.median(correction_grid_after[0, -1, :, 1])
                        - torch.median(correction_grid_after[0, 0, :, 1])
                    ).item()
                )
                correction_within_tolerance = (
                    abs(correction_residual_after) <= SECOND_FOLD_TARGET_TOLERANCE_M
                )
                correction_step_limited = bool(
                    second_fold_correction_replay["correction_plan"].get(
                        "step_limited"
                    )
                )
                correction_monotonic_improvement = (
                    abs(correction_residual_after) < abs(correction_residual_before)
                    and (
                        correction_residual_after * correction_residual_before > 0.0
                        or correction_within_tolerance
                    )
                )
                if (
                    not correction_within_tolerance
                    and (
                        not correction_step_limited
                        or not correction_monotonic_improvement
                    )
                ):
                    raise RuntimeError(
                        "S2 correction did not safely reach or monotonically approach "
                        "the camera edge tolerance: "
                        f"residual={correction_residual_after:.6f} m, "
                        f"limit={SECOND_FOLD_TARGET_TOLERANCE_M:.6f} m"
                    )
                second_fold_correction_result = {
                    "replay_path": str(
                        args.second_fold_correction_replay.resolve()
                    ),
                    "fresh_observation_signed_edge_residual_m": (
                        correction_residual_before
                    ),
                    "planned_correction_delta_y_m": float(
                        second_fold_correction_replay["correction_plan"][
                            "correction_delta_y_m"
                        ]
                    ),
                    "contact_mode": correction_contact_mode,
                    "open_jaw_edge_push": correction_is_open_jaw_edge_push,
                    "actual_contact_particles": correction_upper_particles,
                    "fixed_rubber_face_contact_particles": (
                        correction_fixed_face_particles
                    ),
                    "moving_jaw_face_contact_particles": (
                        correction_moving_face_particles
                    ),
                    "opposing_face_assignment": correction_opposing_assignment,
                    "contact_stop_motion_fraction": (
                        correction_contact_stop_step / correction_close_steps
                    ),
                    "retained_particles_one_per_s1_layer": (
                        correction_contact_particles
                    ),
                    "lower_bundle_jaw_contact_particles": (
                        correction_lower_particles
                    ),
                    "laydown_patch_maximum_table_clearance_m": (
                        correction_laydown_clearance_m
                    ),
                    "maximum_cloth_displacement_during_approach_m": float(
                        torch.max(
                            torch.linalg.vector_norm(
                                correction_nodes_before_transport
                                - correction_nodes_before_approach,
                                dim=-1,
                            )
                        ).item()
                    ),
                    "settle_time_s": correction_settled_step * physics_dt_s,
                    "settle_gate": {
                        "observable": "p95_shape_and_median_edge_change_per_video_frame",
                        "video_fps": RELEASE_SHAPE_VIDEO_FPS,
                        "displacement_threshold_m": (
                            RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                        ),
                        "consecutive_frames": (
                            RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                        ),
                        "samples": correction_settle_samples,
                    },
                    "final_signed_edge_residual_m": correction_residual_after,
                    "target_tolerance_m": SECOND_FOLD_TARGET_TOLERANCE_M,
                    "within_target_tolerance": correction_within_tolerance,
                    "bounded_step_monotonic_improvement": (
                        correction_monotonic_improvement
                    ),
                    "another_observe_correct_step_required": (
                        not correction_within_tolerance
                    ),
                    "reobservation_required": True,
                    "motion_authorized": False,
                    "checkpoint_isolated": True,
                    "end_to_end_result_claimed": False,
                    "raw_checkpoint": second_fold_correction_checkpoint,
                }
                print(
                    "S2_CORRECTION_RESULT "
                    + json.dumps(second_fold_correction_result, sort_keys=True),
                    flush=True,
                )

            nodes_final = local_nodes(scene, cloth).clone()
            second_final_node_speeds = torch.linalg.vector_norm(
                cloth.data.nodal_vel_w.torch, dim=-1
            ).reshape(-1)
            second_maximum_final_speed_m_s = float(
                torch.max(second_final_node_speeds).item()
            )
            second_p95_final_speed_m_s = float(
                torch.quantile(second_final_node_speeds, 0.95).item()
            )
            second_minimum = torch.min(nodes_final, dim=1).values
            second_maximum = torch.max(nodes_final, dim=1).values
            second_spans = second_maximum - second_minimum
            second_grid = nodes_final.reshape(
                environment_count, grid_side, grid_side, 3
            )
            second_row_pair_errors = torch.linalg.vector_norm(
                second_grid[:, : grid_side // 2, :, :2]
                - torch.flip(
                    second_grid[:, grid_side // 2 :, :, :2], dims=(1,)
                ),
                dim=-1,
            ).reshape(environment_count, -1)
            second_p95_pair_error_m = torch.quantile(
                second_row_pair_errors, 0.95, dim=1
            )
            moving_edge_y_m = torch.median(second_grid[:, -1, :, 1], dim=1).values
            stationary_edge_y_m = torch.median(
                second_grid[:, 0, :, 1], dim=1
            ).values
            second_oracle_edge_residual_m = moving_edge_y_m - stationary_edge_y_m
            second_shape_gate_failures = []
            if not second_settle_gate_passed:
                second_shape_gate_failures.append(
                    "settle_timeout_p95_shape_or_edge_change_not_stable"
                )
            for environment_index in range(environment_count):
                x_span = float(second_spans[environment_index, 0].item())
                y_span = float(second_spans[environment_index, 1].item())
                if not (
                    SECOND_FOLD_MINIMUM_FOOTPRINT_SPAN_M
                    <= x_span
                    <= SECOND_FOLD_MAXIMUM_FOOTPRINT_SPAN_M
                ):
                    second_shape_gate_failures.append(
                        f"env_{environment_index}_x_span_m={x_span:.6f}"
                    )
                if not (
                    SECOND_FOLD_MINIMUM_FOOTPRINT_SPAN_M
                    <= y_span
                    <= SECOND_FOLD_MAXIMUM_FOOTPRINT_SPAN_M
                ):
                    second_shape_gate_failures.append(
                        f"env_{environment_index}_y_span_m={y_span:.6f}"
                    )
            second_maximum_height_m = float(
                torch.max(nodes_final[..., 2] - table_top_z_m_for_contact_gate).item()
            )
            if second_maximum_height_m > SECOND_FOLD_MAXIMUM_HEIGHT_M:
                second_shape_gate_failures.append(
                    f"maximum_height_m={second_maximum_height_m:.6f}"
                )
            second_minimum_table_clearance_m = float(
                torch.min(
                    nodes_final[..., 2] - table_top_z_m_for_contact_gate
                ).item()
            )
            second_nodes_below_table_limit = int(
                torch.sum(
                    nodes_final[..., 2] - table_top_z_m_for_contact_gate
                    < -SECOND_FOLD_MAXIMUM_TABLE_PENETRATION_M
                ).item()
            )
            if (
                second_minimum_table_clearance_m
                < -SECOND_FOLD_MAXIMUM_TABLE_PENETRATION_M
            ):
                second_shape_gate_failures.append(
                    "minimum_table_clearance_m="
                    f"{second_minimum_table_clearance_m:.6f}"
                )
            second_fold_result = {
                "status": (
                    SECOND_FOLD_CORRECTED_STATUS
                    if second_fold_correction_result is not None
                    else SECOND_FOLD_RAW_EXECUTED_STATUS
                ),
                "active_arm": second_active_arm,
                "direction": "left_to_right",
                "motion_style": (
                    "bimanual_four_layer_u_pinch_and_fold_arc"
                    if second_active_arm == "bimanual"
                    else "single_arm_four_layer_u_pinch_and_flip"
                ),
                "right_arm_role": (
                    "robot_far_edge_primary_grasp_and_simultaneous_release"
                    if second_active_arm == "bimanual"
                    else "fixed_rubber_pad_surface_press_during_left_release"
                    if right_surface_press_enabled
                    else
                    "actual_contact_gated_bundle_stabilizer_during_left_release"
                    if right_stabilizer_enabled
                    else "clear_observer_then_post_release_camera_correction"
                ),
                "release_mode": args.second_fold_release_mode,
                "newton_coupling_mode": args.newton_coupling_mode,
                "grasp_transport_mode": args.second_fold_grasp_mode,
                "contact_gate_kind": second_contact_gate_kind,
                "physx_pad_proxy_distances_m": (
                    second_physx_pad_proxy_distances_m
                ),
                "physx_attachment_records": second_physx_attachment_records,
                "jaw_particle_collision_filtered_during_retained_transport": bool(
                    second_filtered_jaw_flags
                ),
                "retention_release_opening_travel_fraction": (
                    SECOND_FOLD_RETENTION_RELEASE_OPEN_FRACTION
                ),
                "four_layer_gripper_candidate_path": str(
                    args.second_fold_gripper_config.resolve()
                ),
                "four_layer_contact_gate_passed": True,
                "finite_element_support_vertices": (
                    second_actual_contact_particles
                ),
                "solver_particle_contacts_by_arm": (
                    second_solver_particle_contacts_by_arm
                ),
                "retained_contact_element_vertices": second_contact_particles,
                "retained_contact_element_vertices_by_arm": (
                    second_contact_particles_by_arm
                ),
                "four_physical_particle_contacts_by_arm": (
                    second_contact_gate_diagnostics_by_arm
                ),
                "continuous_tangent_plane_diagnostics_by_arm": (
                    second_continuous_surface_diagnostics_by_arm
                ),
                "continuous_tangent_plane_errors_by_arm": (
                    second_continuous_surface_errors_by_arm
                ),
                "contacted_s1_topology_halves": contacted_s1_layers,
                "contacted_s1_topology_halves_by_arm": (
                    contacted_s1_layers_by_arm
                ),
                "commanded_project_rad": second_project_target,
                "commanded_model_rad": second_model_target,
                "achieved_model_rad": achieved_second_gripper_model_rad,
                "commanded_project_rad_by_arm": second_project_targets,
                "commanded_model_rad_by_arm": second_model_targets,
                "achieved_model_rad_by_arm": (
                    achieved_second_gripper_model_rad_by_arm
                ),
                "command_is_force_claim": False,
                "closing_axis_w_env_0": second_closing_axis_w[0].tolist(),
                "maximum_cloth_displacement_during_close_m": (
                    second_close_cloth_displacement_m
                ),
                "laydown_patch_minimum_table_clearance_m": (
                    second_laydown_patch_minimum_table_clearance_m
                ),
                "laydown_patch_median_table_clearance_m": (
                    second_laydown_patch_median_table_clearance_m
                ),
                "laydown_patch_maximum_table_clearance_m": (
                    second_laydown_patch_table_clearance_m
                ),
                "pinned_laydown_footprint_span_xy_m": (
                    second_laydown_spans[..., :2].tolist()
                ),
                "pinned_laydown_oracle_edge_residual_m": (
                    second_laydown_edge_residual_m.tolist()
                ),
                "release_patch_lift_m": second_release_patch_lift_m,
                "release_patch_to_gripper_distance_m": (
                    second_patch_to_gripper_distance_m
                ),
                "release_patch_to_gripper_distance_m_by_arm": (
                    second_patch_to_gripper_distance_m_by_arm
                ),
                "right_stabilizer_actual_bilateral_particles": (
                    right_stabilizer_actual_contact_particles
                    if right_stabilizer_enabled and not right_surface_press_enabled
                    else []
                ),
                "right_surface_press_actual_fixed_pad_particles": (
                    right_stabilizer_actual_contact_particles
                    if right_surface_press_enabled
                    else []
                ),
                "right_stabilizer_retained_particle": (
                    right_stabilizer_particle
                    if right_stabilizer_enabled and not right_surface_press_enabled
                    else None
                ),
                "right_stabilizer_commanded_project_rad": (
                    right_stabilizer_project_target
                    if right_stabilizer_enabled and not right_surface_press_enabled
                    else None
                ),
                "right_stabilizer_commanded_model_rad": (
                    right_stabilizer_model_target
                    if right_stabilizer_enabled and not right_surface_press_enabled
                    else None
                ),
                "right_stabilizer_contact_stop_model_rad": float(
                    right_stabilizer_closed_row[
                        0, GRIPPER_JOINT_INDICES[1]
                    ].item()
                ) if right_stabilizer_enabled else None,
                "right_stabilizer_contact_stop_closing_fraction": (
                    right_stabilizer_contact_stop_step / second_close_steps
                    if right_stabilizer_contact_stop_step is not None
                    else 1.0
                ),
                "right_stabilizer_achieved_model_rad": (
                    achieved_right_stabilizer_model_rad
                    if right_stabilizer_enabled
                    else None
                ),
                "right_stabilizer_approach_cloth_displacement_m": (
                    right_stabilizer_approach_cloth_displacement_m
                ),
                "right_stabilizer_approach_command_duration_s": (
                    right_stabilizer_approach_command_duration_s
                ),
                "right_stabilizer_pregrasp_cloth_displacement_m": (
                    right_stabilizer_pregrasp_cloth_displacement_m
                ),
                "right_stabilizer_descent_cloth_displacement_m": (
                    right_stabilizer_descent_cloth_displacement_m
                ),
                "right_stabilizer_descent_stop_fraction": (
                    right_stabilizer_descent_stop_step
                    / right_stabilizer_contact_descent_steps
                    if right_stabilizer_descent_stop_step is not None
                    else 1.0
                ),
                "right_stabilizer_close_cloth_displacement_m": (
                    right_stabilizer_close_cloth_displacement_m
                ),
                "right_stabilizer_release_patch_lift_m": (
                    right_stabilizer_release_patch_lift_m
                ),
                "right_stabilizer_release_patch_to_gripper_distance_m": (
                    right_stabilizer_patch_to_gripper_distance_m
                ),
                "settle_gate_passed": second_settle_gate_passed,
                "settle_gate": {
                    "observable": "p95_shape_and_median_edge_change_per_video_frame",
                    "video_fps": RELEASE_SHAPE_VIDEO_FPS,
                    "displacement_threshold_m": (
                        RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M
                    ),
                    "consecutive_frames": (
                        RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES
                    ),
                    "samples": second_settle_samples,
                },
                "settle_time_s": (
                    second_settled_step * physics_dt_s
                    if second_settled_step is not None
                    else SECOND_FOLD_SETTLE_TIMEOUT_S
                ),
                "maximum_final_speed_m_s": second_maximum_final_speed_m_s,
                "p95_final_speed_m_s": second_p95_final_speed_m_s,
                "per_environment_footprint_span_xy_m": (
                    second_spans[..., :2].tolist()
                ),
                "per_environment_p95_paired_row_xy_error_m": (
                    second_p95_pair_error_m.tolist()
                ),
                "simulation_oracle_moving_minus_stationary_edge_y_m": (
                    second_oracle_edge_residual_m.tolist()
                ),
                "maximum_final_height_m": second_maximum_height_m,
                "minimum_final_table_clearance_m": (
                    second_minimum_table_clearance_m
                ),
                "maximum_table_penetration_limit_m": (
                    SECOND_FOLD_MAXIMUM_TABLE_PENETRATION_M
                ),
                "nodes_below_table_penetration_limit": (
                    second_nodes_below_table_limit
                ),
                "post_laydown_softening_ramp_displacement_m": (
                    second_post_laydown_softening_displacement_m
                ),
                "high_curvature_fold_hysteresis": (
                    second_fold_hysteresis_diagnostic
                ),
                "physx_post_laydown_material": (
                    second_physx_post_laydown_material_diagnostic
                ),
                "coarse_shape_gate_passed": not second_shape_gate_failures,
                "coarse_shape_gate_failures": second_shape_gate_failures,
                "camera_correction_applied": (
                    second_fold_correction_result is not None
                ),
                "camera_correction": second_fold_correction_result,
                "accepted_s1_checkpoint": second_fold_checkpoint,
            }
            print(
                "S2_RAW_FOLD_RESULT "
                + json.dumps(second_fold_result, sort_keys=True),
                flush=True,
            )
            result_status = (
                SECOND_FOLD_CORRECTED_STATUS
                if second_fold_correction_result is not None
                else SECOND_FOLD_RAW_EXECUTED_STATUS
            )
            keep_open_row = second_current_row

    curvature_softening_diagnostic = None
    if args.newton_curvature_softening:
        softened_edges_np = curvature_softening_runtime["softened_edges"].numpy()
        ever_softened_edges_np = curvature_softening_runtime[
            "ever_softened_edges"
        ].numpy()
        peak_angles_np = curvature_softening_runtime["peak_absolute_angles"].numpy()
        curvature_softening_diagnostic = {
            "enabled": True,
            "stage": args.newton_curvature_softening_stage,
            "post_laydown_ramp_s": (
                SECOND_FOLD_POST_LAYDOWN_SOFTENING_RAMP_S
                if args.newton_curvature_softening_stage == "s2-post-laydown"
                else 0.0
            ),
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
        "physics_backend": args.physics_backend,
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
            "physx_dynamic_friction_workaround": (
                args.physx_dynamic_friction_workaround
            ),
            "measured_dynamic_friction": material_candidate.dynamic_friction,
            "edge_release_bend_damping_calibrated": (
                args.physics_backend == "physx"
            ),
            "newton_meter_calibration_status": (
                "CANTILEVER_AND_EDGE_RELEASE_MATCH"
                if resolution_specific_newton_calibration is not None
                else material_candidate.newton_calibration_status
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_resolution_specific_calibration": (
                resolution_specific_newton_calibration
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
            "measured_one_layer_close_command_project_rad": (
                REQUESTED_PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "measured_one_layer_close_command_model_target_rad": (
                REQUESTED_PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "simulation_contact_limited_achieved_model_rad": (
                PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "one_way_left_contact_limit_override_used": (
                args.left_one_way_contact_limited_model_rad is not None
            ),
            "one_way_contact_limit_force_validated": False,
        },
        "cloth": {
            "size_xy_m": list(CLOTH_SIZE_XY_M),
            "resolution": list(CLOTH_RESOLUTION),
            "material_calibrated_resolution": list(
                MATERIAL_CALIBRATED_CLOTH_RESOLUTION
            ),
            "resolution_matches_material_calibration": (
                CLOTH_RESOLUTION_MATCHES_MATERIAL_CALIBRATION
            ),
            "validation_scope": (
                "full_fold_material_and_contact"
                if CLOTH_RESOLUTION_MATCHES_MATERIAL_CALIBRATION
                else "local_jaw_contact_geometry_only_material_extrapolated"
            ),
            "node_count": int(nodes_after.shape[1]),
            "mass_kg": CLOTH_MASS_KG,
            "density_kg_m3": CLOTH_DENSITY_KG_M3,
            "static_friction": CLOTH_STATIC_FRICTION,
            "dynamic_friction": CLOTH_DYNAMIC_FRICTION,
            "self_collision_enabled": args.self_contact,
            "self_collision_enabled_at_spawn": (
                args.self_contact and IS_NEWTON_BACKEND
            ),
            "self_collision_enabled_after_closed_jaw_gate": (
                args.self_contact
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
            "newton_vbd_iterations_per_substep": (
                args.newton_vbd_iterations
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_contact_damping": (
                args.newton_contact_damping
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_deep_table_support": (
                {
                    "enabled": args.newton_deep_table_support,
                    "depth_m": (
                        NEWTON_DEEP_TABLE_SUPPORT_DEPTH_M
                        if args.newton_deep_table_support
                        else None
                    ),
                    "top_surface_matches_measured_table": True,
                    "visual_table_unchanged": True,
                }
                if IS_NEWTON_BACKEND
                else None
            ),
            "newton_analytic_table_plane": (
                {
                    "enabled": args.newton_analytic_table_plane,
                    "top_surface_matches_measured_table": True,
                    "robot_collision_filtered": True,
                    "visual_table_unchanged": True,
                    **analytic_plane_filter_runtime,
                }
                if IS_NEWTON_BACKEND
                else None
            ),
            "newton_vertex_contact_buffer_size": (
                128
                if args.physics_backend == "newton-coupled-vbd"
                and args.execute_second_fold
                else 32
                if args.physics_backend == "newton-coupled-vbd"
                else None
            ),
            "newton_edge_contact_buffer_size": (
                256
                if args.physics_backend == "newton-coupled-vbd"
                and args.execute_second_fold
                else 64
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
                "validated_actual_contact_lift_checkpoint_replayed_at_"
                "identical_jaw_pose_then_operator_measured_no_slip_retention"
                if validated_contact_checkpoint_used
                else "actual_distinct_registered_face_contacts_then_operator_"
                "measured_no_slip_retention"
                if contact_gated_retention_used
                else None
            ),
            "s1_checkpoint_prelude_surrogate_used": False,
            "validated_contact_checkpoint": (
                {
                    "path": validated_contact_checkpoint["path"],
                    "sha256": validated_contact_checkpoint["sha256"],
                    "used_for_contact_buffer_boundary_replay": True,
                }
                if validated_contact_checkpoint_used
                else None
            ),
            "strict_single_sheet_pinch_by_side": (
                strict_single_sheet_pinch_by_side
                if strict_single_sheet_pinch_by_side
                else None
            ),
            "proximity_fallback_used": False if contact_gated_retention_used else None,
            "actual_contact_particles_by_side": (
                actual_bilateral_particles_by_side
                if contact_gated_retention_used
                else None
            ),
            "retained_finite_element_support_by_side": (
                retained_finite_element_support_by_side
                if contact_gated_retention_used
                else None
            ),
            "retention_support_mode": (
                "actual_opposing_face_contact_particles_only"
                if contact_gated_retention_used
                and args.retain_contact_evidence_only
                else "continuous_contact_triangle_vertices"
                if contact_gated_retention_used
                and args.surface_distributed_contact_retention
                else "entire_contact_finite_element"
                if contact_gated_retention_used
                else None
            ),
            "progressive_contact_release_used": (
                args.progressive_contact_release
                if contact_gated_retention_used
                else None
            ),
            "contact_release_activations": (
                contact_gated_release_activations
                if contact_gated_retention_used
                else None
            ),
            "progressive_contact_release_events": (
                progressive_contact_release_events
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
            "requested_measured_pinch_project_gripper_joint_positions_rad": (
                REQUESTED_PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "pinch_model_gripper_joint_positions_rad": (
                PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
            ),
            "requested_measured_pinch_model_gripper_joint_positions_rad": (
                REQUESTED_PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD
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
        "second_fold": second_fold_result,
        "second_fold_correction": second_fold_correction_result,
        "second_fold_correction_checkpoint": second_fold_correction_checkpoint,
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
