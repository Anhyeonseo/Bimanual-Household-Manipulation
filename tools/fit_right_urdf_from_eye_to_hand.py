#!/usr/bin/env python3
"""Fit a simulation-only right-arm URDF preview to eye-to-hand captures.

This utility deliberately uses every capture, including the nominal validation
set, because its output is only a visual diagnostic for comparing the inferred
q0 geometry in Isaac Sim.  The generated URDF must never be promoted directly
to MoveIt or hardware motion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from solve_top_base_visual_registration import (  # noqa: E402
    axis_angle_matrix,
    parse_vector,
    rpy_matrix,
    transform,
    urdf_fk,
    yaml_matrix,
)
from solve_top_eye_to_hand import (  # noqa: E402
    ARM_JOINT_NAMES_BY_SIDE,
    PoseObservation,
    capture_observation,
    invert_transform,
    make_transform,
    matrix_document,
    parse_target,
    residual_summary,
    solve_eye_to_hand,
)


DEFAULT_SINGLE_ARM_XACRO = (
    ROOT / "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"
)
DEFAULT_PREVIEW_XACRO = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
DEFAULT_CAMERA_INFO = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml"
)
FIT_JOINTS = (
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
)
Q0_LINKS = (
    "right_shoulder_link",
    "right_upper_arm_link",
    "right_lower_arm_link",
    "right_wrist_link",
    "right_gripper_link",
    "right_gripper_frame_link",
)
TRANSLATION_RESIDUAL_SCALE_M = 0.003
ROTATION_RESIDUAL_SCALE_RAD = math.radians(1.0)
ORIGIN_TRANSLATION_PRIOR_M = 0.005
ORIGIN_ROTATION_PRIOR_RAD = math.radians(3.0)


@dataclass(frozen=True)
class JointModel:
    name: str
    joint_type: str
    origin: np.ndarray
    axis: np.ndarray


def load_yaml(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return document


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xacro_executable() -> str:
    discovered = shutil.which("xacro")
    if discovered is not None:
        return discovered
    jazzy = Path("/opt/ros/jazzy/bin/xacro")
    if jazzy.is_file():
        return str(jazzy)
    raise FileNotFoundError("xacro was not found")


def expand_xacro(path: Path, *mappings: str) -> str:
    return subprocess.run(
        [xacro_executable(), str(path), *mappings],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def element_origin(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    xyz = parse_vector(
        None if origin is None else origin.attrib.get("xyz"),
        "0 0 0",
    )
    rpy = parse_vector(
        None if origin is None else origin.attrib.get("rpy"),
        "0 0 0",
    )
    return transform(rpy_matrix(rpy), xyz)


def joint_chain(
    urdf_xml: str,
    base_link: str,
    target_link: str,
) -> list[JointModel]:
    root = ET.fromstring(urdf_xml)
    joints_by_child = {
        str(joint.find("child").attrib["link"]): joint
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    chain: list[ET.Element] = []
    link = target_link
    while link != base_link:
        if link not in joints_by_child:
            raise ValueError(f"no URDF chain from {base_link} to {target_link}")
        joint = joints_by_child[link]
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None:
            raise ValueError("joint has no parent")
        link = str(parent.attrib["link"])
    chain.reverse()
    result = []
    for joint in chain:
        axis_element = joint.find("axis")
        axis = parse_vector(
            None if axis_element is None else axis_element.attrib.get("xyz"),
            "1 0 0",
        )
        result.append(
            JointModel(
                name=str(joint.attrib["name"]),
                joint_type=str(joint.attrib.get("type", "fixed")),
                origin=element_origin(joint),
                axis=axis,
            )
        )
    return result


def correction_transform(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if values.shape != (6,):
        raise ValueError("joint-origin correction must contain six values")
    return make_transform(
        Rotation.from_rotvec(values[3:]).as_matrix(),
        values[:3],
    )


def corrected_fk(
    chain: list[JointModel],
    joint_positions: dict[str, float],
    corrections: dict[str, np.ndarray],
) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    for joint in chain:
        origin = joint.origin
        if joint.name in corrections:
            origin = origin @ correction_transform(corrections[joint.name])
        result = result @ origin
        if joint.joint_type in ("revolute", "continuous"):
            result = result @ transform(
                axis_angle_matrix(
                    joint.axis,
                    float(joint_positions.get(joint.name, 0.0)),
                ),
                np.zeros(3),
            )
        elif joint.joint_type != "fixed":
            raise ValueError(f"unsupported joint type: {joint.joint_type}")
    return result


def unpack_fit(
    values: np.ndarray,
    fit_joints: tuple[str, ...] = FIT_JOINTS,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    base_to_camera = make_transform(
        Rotation.from_rotvec(values[0:3]).as_matrix(),
        values[3:6],
    )
    gripper_to_target = make_transform(
        Rotation.from_rotvec(values[6:9]).as_matrix(),
        values[9:12],
    )
    corrections = {
        name: np.asarray(values[12 + 6 * index : 18 + 6 * index])
        for index, name in enumerate(fit_joints)
    }
    return base_to_camera, gripper_to_target, corrections


def adjusted_observations(
    captures: list[dict],
    camera_observations: list[PoseObservation],
    chain: list[JointModel],
    corrections: dict[str, np.ndarray],
    joint_names: tuple[str, ...],
) -> list[PoseObservation]:
    result = []
    for capture, observation in zip(
        captures,
        camera_observations,
        strict=True,
    ):
        positions = np.asarray(capture["measured_arm_rad"], dtype=np.float64)
        fk = corrected_fk(
            chain,
            dict(zip(joint_names, positions, strict=True)),
            corrections,
        )
        result.append(
            PoseObservation(
                capture_id=observation.capture_id,
                base_to_gripper=fk,
                camera_to_target=observation.camera_to_target,
                pnp_rms_px=observation.pnp_rms_px,
                image_border_px=observation.image_border_px,
                detected_marker_ids=observation.detected_marker_ids,
            )
        )
    return result


def fit_geometry(
    captures: list[dict],
    observations: list[PoseObservation],
    chain: list[JointModel],
    regularization: float,
    max_nfev: int,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray], object]:
    if regularization < 0.0 or not math.isfinite(regularization):
        raise ValueError("regularization must be finite and nonnegative")
    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    initial_base_to_camera, initial_gripper_to_target = solve_eye_to_hand(
        observations
    )
    initial = np.concatenate(
        (
            Rotation.from_matrix(
                initial_base_to_camera[:3, :3]
            ).as_rotvec(),
            initial_base_to_camera[:3, 3],
            Rotation.from_matrix(
                initial_gripper_to_target[:3, :3]
            ).as_rotvec(),
            initial_gripper_to_target[:3, 3],
            np.zeros(6 * len(FIT_JOINTS)),
        )
    )
    measured = [
        np.asarray(capture["measured_arm_rad"], dtype=np.float64)
        for capture in captures
    ]

    def residual(values: np.ndarray) -> np.ndarray:
        base_to_camera, gripper_to_target, corrections = unpack_fit(values)
        values_out: list[float] = []
        for positions, observation in zip(
            measured,
            observations,
            strict=True,
        ):
            base_to_gripper = corrected_fk(
                chain,
                dict(zip(joint_names, positions, strict=True)),
                corrections,
            )
            error = invert_transform(
                base_to_gripper @ gripper_to_target
            ) @ (base_to_camera @ observation.camera_to_target)
            values_out.extend(
                (error[:3, 3] / TRANSLATION_RESIDUAL_SCALE_M).tolist()
            )
            values_out.extend(
                (
                    Rotation.from_matrix(error[:3, :3]).as_rotvec()
                    / ROTATION_RESIDUAL_SCALE_RAD
                ).tolist()
            )
        if regularization > 0.0:
            weight = math.sqrt(regularization)
            for correction in corrections.values():
                values_out.extend(
                    (
                        weight
                        * correction[:3]
                        / ORIGIN_TRANSLATION_PRIOR_M
                    ).tolist()
                )
                values_out.extend(
                    (
                        weight
                        * correction[3:]
                        / ORIGIN_ROTATION_PRIOR_RAD
                    ).tolist()
                )
        return np.asarray(values_out, dtype=np.float64)

    fit = least_squares(
        residual,
        initial,
        max_nfev=max_nfev,
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    base_to_camera, gripper_to_target, corrections = unpack_fit(fit.x)
    return base_to_camera, gripper_to_target, corrections, fit


def format_vector(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def matrix_xyz_rpy(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray(matrix[:3, 3], dtype=np.float64),
        Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz"),
    )


def apply_corrections_to_preview(
    preview_xml: str,
    chain: list[JointModel],
    corrections: dict[str, np.ndarray],
) -> tuple[ET.Element, list[dict]]:
    root = ET.fromstring(preview_xml)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    chain_by_name = {joint.name: joint for joint in chain}
    records = []
    for name in FIT_JOINTS:
        if name not in joints or name not in chain_by_name:
            raise ValueError(f"preview or fitting chain is missing {name}")
        preview_nominal = element_origin(joints[name])
        nominal = chain_by_name[name].origin
        if not np.allclose(preview_nominal, nominal, atol=1e-9):
            raise RuntimeError(f"single-arm and preview origins differ: {name}")
        candidate = nominal @ correction_transform(corrections[name])
        nominal_xyz, nominal_rpy = matrix_xyz_rpy(nominal)
        candidate_xyz, candidate_rpy = matrix_xyz_rpy(candidate)
        origin = joints[name].find("origin")
        if origin is None:
            origin = ET.SubElement(joints[name], "origin")
        origin.attrib["xyz"] = format_vector(candidate_xyz)
        origin.attrib["rpy"] = format_vector(candidate_rpy)
        correction = corrections[name]
        records.append(
            {
                "joint": name,
                "nominal_xyz_m": [float(value) for value in nominal_xyz],
                "nominal_rpy_rad": [float(value) for value in nominal_rpy],
                "candidate_xyz_m": [float(value) for value in candidate_xyz],
                "candidate_rpy_rad": [float(value) for value in candidate_rpy],
                "local_delta_xyz_mm": [
                    float(value * 1000.0) for value in correction[:3]
                ],
                "local_delta_rotvec_deg": [
                    float(math.degrees(value)) for value in correction[3:]
                ],
            }
        )
    root.attrib["name"] = "so101_dual_right_data_fit_preview"
    root.insert(
        0,
        ET.Comment(
            " VISUAL PREVIEW ONLY; validation captures were used in the fit; "
            "never use for MoveIt or hardware motion "
        ),
    )
    return root, records


def q0_link_differences(
    nominal_xml: str,
    candidate_xml: str,
) -> list[dict]:
    zero = {name: 0.0 for name in ARM_JOINT_NAMES_BY_SIDE["right"]}
    result = []
    for link in Q0_LINKS:
        nominal = urdf_fk(
            nominal_xml,
            "right_base_link",
            link,
            zero,
        )
        candidate = urdf_fk(
            candidate_xml,
            "right_base_link",
            link,
            zero,
        )
        relative = invert_transform(nominal) @ candidate
        result.append(
            {
                "link": link,
                "position_delta_mm": float(
                    np.linalg.norm(candidate[:3, 3] - nominal[:3, 3])
                    * 1000.0
                ),
                "orientation_delta_deg": float(
                    math.degrees(
                        Rotation.from_matrix(relative[:3, :3]).magnitude()
                    )
                ),
                "nominal_xyz_m": [
                    float(value) for value in nominal[:3, 3]
                ],
                "candidate_xyz_m": [
                    float(value) for value in candidate[:3, 3]
                ],
            }
        )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a simulation-only right-arm URDF preview to all eye-to-hand "
            "captures for q0 visual inspection."
        )
    )
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--camera-info", type=Path, default=DEFAULT_CAMERA_INFO)
    parser.add_argument(
        "--single-arm-xacro",
        type=Path,
        default=DEFAULT_SINGLE_ARM_XACRO,
    )
    parser.add_argument(
        "--preview-xacro",
        type=Path,
        default=DEFAULT_PREVIEW_XACRO,
    )
    parser.add_argument(
        "--regularization",
        type=float,
        default=0.1,
        help="joint-origin prior weight; default keeps the preview interpretable",
    )
    parser.add_argument("--max-nfev", type=int, default=1000)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session_path = args.session.resolve()
    session = load_yaml(session_path)
    if session.get("arm") != "right":
        raise ValueError("the URDF data-fit preview requires a right-arm session")
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    training = list(session.get("training_captures", []))
    validation = list(session.get("validation_captures", []))
    captures = training + validation
    if len(training) < 8 or len(validation) < 2:
        raise ValueError("at least eight training and two validation captures are required")

    single_xml = expand_xacro(args.single_arm_xacro, "arm_slot:=right")
    preview_xml = expand_xacro(args.preview_xacro)
    chain = joint_chain(
        single_xml,
        "right_base_link",
        "right_gripper_frame_link",
    )
    camera_info = load_yaml(args.camera_info)
    specification = parse_target(session)
    camera_matrix = yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = yaml_matrix(
        camera_info,
        "distortion_coefficients",
        1,
        5,
    ).reshape(-1)
    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    observations = [
        capture_observation(
            capture,
            session_path.parent,
            single_xml,
            "right_base_link",
            "right_gripper_frame_link",
            camera_matrix,
            distortion,
            specification,
            joint_names,
        )
        for capture in captures
    ]

    nominal_base_to_camera, nominal_gripper_to_target = solve_eye_to_hand(
        observations
    )
    nominal_training = residual_summary(
        observations[: len(training)],
        nominal_base_to_camera,
        nominal_gripper_to_target,
    )
    nominal_validation = residual_summary(
        observations[len(training) :],
        nominal_base_to_camera,
        nominal_gripper_to_target,
    )
    base_to_camera, gripper_to_target, corrections, fit = fit_geometry(
        captures,
        observations,
        chain,
        args.regularization,
        args.max_nfev,
    )
    adjusted = adjusted_observations(
        captures,
        observations,
        chain,
        corrections,
        joint_names,
    )
    fitted_training = residual_summary(
        adjusted[: len(training)],
        base_to_camera,
        gripper_to_target,
    )
    fitted_validation = residual_summary(
        adjusted[len(training) :],
        base_to_camera,
        gripper_to_target,
    )

    candidate_root, origin_records = apply_corrections_to_preview(
        preview_xml,
        chain,
        corrections,
    )
    ET.indent(candidate_root, space="  ")
    candidate_xml = ET.tostring(
        candidate_root,
        encoding="unicode",
        xml_declaration=True,
    )
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(candidate_xml, encoding="utf-8")
    q0_differences = q0_link_differences(preview_xml, candidate_xml)

    fit_report = output.with_suffix(".fit.yaml")
    report = {
        "schema_version": 1,
        "status": "VISUAL_PREVIEW_ONLY_NOT_CALIBRATION_APPROVED",
        "simulation_only": True,
        "motion_authorized": False,
        "validation_used_in_fit": True,
        "session": str(session_path),
        "session_sha256": sha256_file(session_path),
        "fit_scope": "all_training_and_validation_captures",
        "fit_joint_origins": list(FIT_JOINTS),
        "regularization": {
            "weight": float(args.regularization),
            "translation_prior_mm": ORIGIN_TRANSLATION_PRIOR_M * 1000.0,
            "rotation_prior_deg": math.degrees(ORIGIN_ROTATION_PRIOR_RAD),
        },
        "optimizer": {
            "success": bool(fit.success),
            "message": str(fit.message),
            "nfev": int(fit.nfev),
            "cost": float(fit.cost),
        },
        "nominal_all_capture_fit": {
            "training": nominal_training,
            "validation": nominal_validation,
        },
        "data_fit_all_capture_fit": {
            "training": fitted_training,
            "validation": fitted_validation,
        },
        "base_to_camera": matrix_document(base_to_camera),
        "gripper_to_target": matrix_document(gripper_to_target),
        "joint_origin_changes": origin_records,
        "q0_link_differences": q0_differences,
        "required_next_gate": (
            "operator visually compares q0 in Isaac Sim; this preview cannot "
            "replace independent right-arm kinematic metrology"
        ),
    }
    fit_report.write_text(
        yaml.safe_dump(report, sort_keys=False),
        encoding="utf-8",
    )

    manifest = output.with_suffix(".manifest.json")
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "simulation_only": True,
                "motion_authorized": False,
                "validation_used_in_fit": True,
                "urdf": str(output),
                "urdf_sha256": sha256_file(output),
                "fit_report": str(fit_report),
                "fit_report_sha256": sha256_file(fit_report),
                "source_preview_xacro": str(args.preview_xacro.resolve()),
                "isaac_import": {
                    "input_file": str(output),
                    "ros_package_list": [
                        {
                            "package_name": "so101_description",
                            "package_path": str(
                                ROOT / "ros2_ws/src/so101_description"
                            ),
                        }
                    ],
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"RIGHT_DATA_FIT_PREVIEW_URDF={output}")
    print(f"FIT_REPORT={fit_report}")
    print(f"MANIFEST={manifest}")
    print(
        "FIT_TRAIN_RMS_MM="
        f"{fitted_training['translation_rms_mm']:.3f} "
        "FIT_VALIDATION_RMS_MM="
        f"{fitted_validation['translation_rms_mm']:.3f}"
    )
    print("ISAAC_ROS_PACKAGE_NAME=so101_description")
    print(
        "ISAAC_ROS_PACKAGE_PATH="
        f"{ROOT / 'ros2_ws/src/so101_description'}"
    )
    print(
        "SIMULATION_ONLY=true MOTION_AUTHORIZED=false "
        "VALIDATION_USED_IN_FIT=true"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
