from __future__ import annotations

import subprocess
import sys

import numpy as np
import pytest
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics

ENTRYPOINT = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
ARM_JOINT_SUFFIXES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
    "gripper_joint",
)


def _expand(**mappings: object) -> ET.Element:
    command = ["xacro", str(ENTRYPOINT)]
    command.extend(f"{key}:={value}" for key, value in mappings.items())
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def _joints(root: ET.Element) -> dict[str, ET.Element]:
    return {joint.attrib["name"]: joint for joint in root.findall("joint")}


def test_preview_has_two_prefixed_arms_and_no_control_plugin() -> None:
    root = _expand()
    joints = _joints(root)
    for prefix in ("left_", "right_"):
        assert {prefix + suffix for suffix in ARM_JOINT_SUFFIXES} <= joints.keys()
    assert root.find("ros2_control") is None


def test_default_right_mount_closes_both_ten_mm_camera_base_joints() -> None:
    root = _expand()
    joints = _joints(root)
    left_origin = joints["left_mount_arm_base_joint"].find("origin")
    right_origin = joints["right_mount_arm_base_joint"].find("origin")
    assert left_origin is not None
    assert right_origin is not None
    assert left_origin.attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}
    assert right_origin.attrib == {
        "xyz": "0 -0.232064146 0",
        "rpy": "0 0 0",
    }


def test_right_mount_candidate_is_explicitly_overridable() -> None:
    root = _expand(
        right_mount_xyz="0.012 -0.348 0.003",
        right_mount_rpy="0.001 -0.002 0.004",
    )
    origin = _joints(root)["right_mount_arm_base_joint"].find("origin")
    assert origin is not None
    assert origin.attrib["xyz"] == "0.012 -0.348 0.003"
    assert origin.attrib["rpy"] == "0.001 -0.002 0.004"


def test_both_arms_reuse_identical_joint_kinematics_without_mirroring() -> None:
    root = _expand()
    joints = _joints(root)
    for suffix in ARM_JOINT_SUFFIXES:
        left = joints["left_" + suffix]
        right = joints["right_" + suffix]
        assert left.attrib["type"] == right.attrib["type"]
        for field in ("origin", "axis"):
            left_field = left.find(field)
            right_field = right.find(field)
            assert (left_field is None) == (right_field is None)
            if left_field is not None and right_field is not None:
                assert left_field.attrib == right_field.attrib
    assert joints["left_gripper_joint"].find("limit").attrib == (
        joints["right_gripper_joint"].find("limit").attrib
    )


def test_preview_uses_operator_approved_full_task_arm_limits() -> None:
    joints = _joints(_expand())
    expected = {
        "left_base_joint": (-1.633689, 1.523243),
        "left_shoulder_joint": (-0.228563, 3.281185),
        "left_elbow_joint": (-0.681087, 2.702874),
        "left_wrist_flex_joint": (-0.515418, 2.880816),
        "left_wrist_roll_joint": (-2.241146, 1.211845),
        "right_base_joint": (-1.441942, 1.454214),
        "right_shoulder_joint": (-0.289922, 3.282719),
        "right_elbow_joint": (-0.728641, 2.686000),
        "right_wrist_flex_joint": (-0.598252, 2.563282),
        "right_wrist_roll_joint": (-1.992641, 1.414330),
    }
    for name, interval in expected.items():
        limit = joints[name].find("limit")
        assert limit is not None
        assert (float(limit.attrib["lower"]), float(limit.attrib["upper"])) == interval


def test_stl_geometry_is_present_on_both_arms() -> None:
    root = _expand()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    for prefix in ("left_", "right_"):
        meshes = links[prefix + "base_link"].findall("visual/geometry/mesh")
        assert len(meshes) == 4
        assert all(mesh.attrib["filename"].endswith(".stl") for mesh in meshes)


def test_validated_overhead_workcell_is_present_by_default() -> None:
    root = _expand()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = _joints(root)
    assert {
        "left_mount_arm_base_link",
        "right_mount_arm_base_link",
        "top_cam_mount_bottom_link",
        "top_cam_mount_top_link",
        "top_camera_link",
        "top_camera_optical_frame",
    } <= links
    assert joints["left_mount_arm_base_joint"].attrib["type"] == "fixed"
    assert joints["right_mount_arm_base_joint"].attrib["type"] == "fixed"
    assert joints["top_cam_mount_bottom_joint"].attrib["type"] == "fixed"
    assert joints["top_cam_mount_top_joint"].attrib["type"] == "fixed"


def test_arms_are_bolted_to_their_matching_base_plates() -> None:
    joints = _joints(_expand())
    assert joints["left_arm_mount_joint"].find("parent").attrib["link"] == (
        "left_mount_arm_base_link"
    )
    assert joints["right_arm_mount_joint"].find("parent").attrib["link"] == (
        "right_mount_arm_base_link"
    )
    assert joints["top_cam_mount_bottom_joint"].find("parent").attrib[
        "link"
    ] == "left_mount_arm_base_link"


def test_validated_left_wrist_camera_frames_are_present_by_default() -> None:
    root = _expand()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = _joints(root)
    assert "left_wrist_camera_mount_center_link" in links
    assert "left_wrist_camera_link" in links
    assert "left_wrist_camera_optical_frame" in links
    mount_origin = joints["left_wrist_camera_mount_joint"].find("origin")
    optical_origin = joints["left_wrist_camera_optical_joint"].find("origin")
    assert mount_origin is not None
    assert optical_origin is not None
    assert mount_origin.attrib["xyz"] == "0.01437087 -0.00675864 0.01616744"
    assert optical_origin.attrib["rpy"] == "-0.02242196 0.03092788 3.02135163"


def test_validated_right_wrist_camera_frames_are_present_by_default() -> None:
    root = _expand()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = _joints(root)
    assert "right_wrist_camera_mount_center_link" in links
    right_gripper_meshes = links["right_gripper_link"].findall(
        "visual/geometry/mesh"
    )
    filenames = {mesh.attrib["filename"] for mesh in right_gripper_meshes}
    assert any(
        filename.endswith("wrist_cam_mount_32x32_uvc_module_so101.stl")
        for filename in filenames
    )
    assert not any(
        filename.endswith("wrist_roll_follower_so101_v1.stl")
        for filename in filenames
    )
    assert "right_wrist_camera_link" in links
    assert "right_wrist_camera_optical_frame" in links
    mount_origin = joints["right_wrist_camera_mount_joint"].find("origin")
    optical_origin = joints["right_wrist_camera_optical_joint"].find("origin")
    assert mount_origin.attrib["xyz"] == (
        "0.000752553355 0.012058633035 -0.008833814916"
    )
    assert optical_origin.attrib["rpy"] == (
        "0.018617155252 0.007963771556 3.122102338518"
    )


def test_right_optical_frame_is_absent_when_mount_is_disabled() -> None:
    root = _expand(right_use_wrist_camera_mount=False)
    links = {link.attrib["name"] for link in root.findall("link")}
    assert "right_wrist_camera_mount_center_link" not in links
    assert "right_wrist_camera_link" not in links
    assert "right_wrist_camera_optical_frame" not in links


def test_right_data_fit_candidate_constrains_equal_height_horizontal_bases() -> None:
    candidate = (
        ROOT
        / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
    )
    root = ET.parse(candidate).getroot()
    assert root.attrib["name"] == "so101_dual_preview"
    mount = _joints(root)["right_mount_arm_base_joint"].find("origin")
    assert mount is not None
    assert mount.attrib == {
        "rpy": "0 0 0",
        "xyz": "0 -0.232064146 0",
    }
    joints = _joints(root)
    assert joints["right_base_joint"].find("origin").attrib != (
        joints["left_base_joint"].find("origin").attrib
    )

    kinematics = GraspYawKinematics(candidate, prefix="right_")
    local = kinematics.point_in_base_frame(
        np.array([0.420, -0.170, 0.0063]),
        root_link="workcell_base_link",
    )
    assert local == pytest.approx(
        [0.420, 0.062064146, 0.0063], abs=1e-8
    )


def test_all_dual_moveit_launches_share_the_opt_in_calibrated_urdf() -> None:
    launch_dir = ROOT / "ros2_ws/src/so101_moveit_config/launch"
    for name in (
        "dual_rsp.launch.py",
        "dual_static_virtual_joint_tfs.launch.py",
        "dual_move_group.launch.py",
        "dual_moveit_rviz.launch.py",
    ):
        source = (launch_dir / name).read_text(encoding="utf-8")
        assert 'os.environ.get("SO101_DUAL_URDF_PATH", default_urdf)' in source
