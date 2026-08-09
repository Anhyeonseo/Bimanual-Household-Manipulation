from __future__ import annotations

import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "ros2_ws/src/so101_description"
ENTRYPOINT = DESCRIPTION / "urdf/so101_left.urdf.xacro"

WRIST_CAMERA_LINKS = {
    "left_wrist_camera_link",
    "left_wrist_camera_optical_frame",
}
WRIST_CAMERA_JOINTS = {
    "left_wrist_camera_mount_joint",
    "left_wrist_camera_optical_joint",
}


def _expand(**mappings: object) -> ET.Element:
    command = ["xacro", str(ENTRYPOINT)]
    command.extend(f"{key}:={str(value).lower() if isinstance(value, bool) else value}" for key, value in mappings.items())
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return ET.fromstring(completed.stdout)


def test_wrist_camera_optical_frame_present_by_default() -> None:
    root = _expand()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint.attrib["type"] for joint in root.findall("joint")}
    assert WRIST_CAMERA_LINKS <= links
    assert WRIST_CAMERA_JOINTS <= joints.keys()
    assert {joints[name] for name in WRIST_CAMERA_JOINTS} == {"fixed"}


def test_wrist_camera_absent_when_mount_disabled() -> None:
    root = _expand(use_wrist_camera_mount=False)
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"] for joint in root.findall("joint")}
    assert links.isdisjoint(WRIST_CAMERA_LINKS)
    assert joints.isdisjoint(WRIST_CAMERA_JOINTS)


def test_wrist_camera_optical_frame_parented_off_mount_center() -> None:
    root = _expand()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount_joint = joints["left_wrist_camera_mount_joint"]
    optical_joint = joints["left_wrist_camera_optical_joint"]
    assert mount_joint.find("parent").attrib["link"] == "left_wrist_camera_mount_center_link"
    assert mount_joint.find("child").attrib["link"] == "left_wrist_camera_link"
    assert optical_joint.find("parent").attrib["link"] == "left_wrist_camera_link"
    assert optical_joint.find("child").attrib["link"] == "left_wrist_camera_optical_frame"


def test_wrist_camera_extrinsic_remains_configurable_and_fixed() -> None:
    root = _expand(
        wrist_camera_xyz="0.01 -0.02 0.03",
        wrist_camera_optical_rpy="1.57 0 -1.57",
    )
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount = joints["left_wrist_camera_mount_joint"]
    optical = joints["left_wrist_camera_optical_joint"]
    assert mount.attrib["type"] == "fixed"
    assert optical.attrib["type"] == "fixed"
    assert mount.find("origin").attrib["xyz"] == "0.01 -0.02 0.03"
    assert optical.find("origin").attrib["rpy"] == "1.57 0 -1.57"


def test_wrist_camera_defaults_are_the_w3_eye_in_hand_solve() -> None:
    # Solved 2026-08-09 by cv2.calibrateHandEye against a fixed planar ArUco
    # target (see docs/checklists/WRIST_CAMERA_EYE_IN_HAND.md W3). Re-derive
    # this pin if the camera, lens, or mount moves.
    root = _expand()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount_origin = joints["left_wrist_camera_mount_joint"].find("origin")
    optical_origin = joints["left_wrist_camera_optical_joint"].find("origin")
    assert mount_origin.attrib["xyz"] == "0.01437087 -0.00675864 0.01616744"
    assert optical_origin.attrib["rpy"] == "-0.02242196 0.03092788 3.02135163"


def test_existing_mount_center_contract_pin_is_unaffected() -> None:
    root = _expand()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount_center = joints["left_wrist_camera_mount_center_joint"]
    assert mount_center.find("origin").attrib["xyz"] == "0.00250008 -0.07292374 0.00595299"
    assert mount_center.find("origin").attrib["rpy"] == "-2.70526034 0 0"
