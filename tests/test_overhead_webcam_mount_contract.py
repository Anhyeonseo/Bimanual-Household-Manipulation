from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "ros2_ws/src/so101_description"
ENTRYPOINT = DESCRIPTION / "urdf/so101_left.urdf.xacro"

EXPECTED_MESH_SHA256 = {
    "overhead_webcam_arm_base.stl": "169adfd40bcca689334efd1188c9b42cc03c914dc0afeaa98cb5431013610833",
    "overhead_webcam_cam_mount_bottom.stl": "b3545b6cae437210e17b7dcfee2e12e00dc7a59ece9264f4b13ab9fd8ceb8088",
    "overhead_webcam_cam_mount_top.stl": "177fbfae49cabba47b0b51811421b656d580dd1ab4f47f2248350f5184c75488",
    "overhead_webcam_cam_mount_top_hinge_removed.stl": "55319a9b26f9cdb7217c94000f7aec716d63b0a433150cd7672baa7b85a006cf",
}

TOP_LINKS = {
    "top_arm_base_link",
    "top_cam_mount_bottom_link",
    "top_cam_mount_top_link",
    "top_camera_link",
    "top_camera_optical_frame",
}
TOP_JOINTS = {
    "top_arm_base_joint",
    "top_cam_mount_bottom_joint",
    "top_cam_mount_top_joint",
    "top_camera_mount_joint",
    "top_camera_optical_joint",
}


def _expand(**mappings: object) -> ET.Element:
    command = ["xacro", str(ENTRYPOINT)]
    command.extend(f"{key}:={str(value).lower() if isinstance(value, bool) else value}" for key, value in mappings.items())
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return ET.fromstring(completed.stdout)


def test_overhead_meshes_are_exact_pinned_copies() -> None:
    for filename, expected in EXPECTED_MESH_SHA256.items():
        actual = hashlib.sha256((DESCRIPTION / "meshes" / filename).read_bytes()).hexdigest()
        assert actual == expected


def test_overhead_workcell_is_disabled_by_default() -> None:
    root = _expand()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"] for joint in root.findall("joint")}
    assert links.isdisjoint(TOP_LINKS)
    assert joints.isdisjoint(TOP_JOINTS)


def test_physical_overhead_configuration_has_only_fixed_joints() -> None:
    root = _expand(use_overhead_webcam_mount=True)
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint.attrib["type"] for joint in root.findall("joint")}
    assert TOP_LINKS <= links
    assert TOP_JOINTS <= joints.keys()
    assert {joints[name] for name in TOP_JOINTS} == {"fixed"}
    assert not any("hinge" in name for name in joints)


def test_physical_top_uses_hinge_removed_mesh_and_exact_insertion_depth() -> None:
    root = _expand(use_overhead_webcam_mount=True)
    top = next(link for link in root.findall("link") if link.attrib["name"] == "top_cam_mount_top_link")
    mesh = top.find("visual/geometry/mesh")
    visual_origin = top.find("visual/origin")
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    top_joint_origin = joints["top_cam_mount_top_joint"].find("origin")
    assert mesh is not None
    assert visual_origin is not None
    assert top_joint_origin is not None
    assert mesh.attrib["filename"].endswith("overhead_webcam_cam_mount_top_hinge_removed.stl")
    assert mesh.attrib["scale"] == "0.001 0.001 0.001"
    assert visual_origin.attrib["xyz"] == "0 0 0"
    assert top_joint_origin.attrib["xyz"] == "0.0187 0.2231 0.0365125"
    collision_box = top.find("collision/geometry/box")
    assert collision_box is not None
    assert collision_box.attrib["size"] == "0.0254 0.2344404 0.036625"


def test_floor_and_right_side_groove_alignment_is_explicit() -> None:
    root = _expand(use_overhead_webcam_mount=True)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    bottom_origin = joints["top_cam_mount_bottom_joint"].find("origin")
    assert bottom_origin is not None
    assert bottom_origin.attrib["xyz"] == "0.0137353073 -0.1525445687 -0.005"
    assert bottom_origin.attrib["rpy"] == "1.5707963267948966 0 3.141592653589793"

    arm = next(link for link in root.findall("link") if link.attrib["name"] == "top_arm_base_link")
    arm_origin = arm.find("visual/origin")
    assert arm_origin is not None
    assert arm_origin.attrib["xyz"] == "0.0702562210 -0.0534304657 0"
    assert arm_origin.attrib["rpy"] == "1.5707963267948966 0 0"


def test_no_direction_switch_or_hinge_geometry_remains_in_expanded_urdf() -> None:
    completed = subprocess.run(
        ["xacro", str(ENTRYPOINT), "use_overhead_webcam_mount:=true"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "overhead_top_reversed" not in completed.stdout
    assert "cam_mount_top.stl" not in completed.stdout
    assert "hinge" not in {joint.attrib["name"] for joint in ET.fromstring(completed.stdout).findall("joint")}


def test_camera_extrinsic_remains_configurable_and_fixed() -> None:
    root = _expand(
        use_overhead_webcam_mount=True,
        top_camera_xyz="0.01 0.30 0.02",
        top_camera_optical_rpy="3.14 0.01 -3.12",
    )
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount = joints["top_camera_mount_joint"]
    optical = joints["top_camera_optical_joint"]
    assert mount.attrib["type"] == "fixed"
    assert optical.attrib["type"] == "fixed"
    assert mount.find("origin").attrib["xyz"] == "0.01 0.30 0.02"
    assert optical.find("origin").attrib["rpy"] == "3.14 0.01 -3.12"
