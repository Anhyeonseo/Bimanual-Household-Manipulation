import hashlib
import re
import struct
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "ros2_ws/src/so101_description"
MACRO_PATH = DESCRIPTION / "urdf/so101_arm_macro.xacro"
LEFT_PATH = DESCRIPTION / "urdf/so101_left.urdf.xacro"
ORIGINAL_MESH = DESCRIPTION / "meshes/wrist_roll_follower_so101_v1.stl"
ISAAC_INSTANCES = (
    ROOT / "isaac_sim/assets/so101_new_calib/payloads/instances.usda"
)
ISAAC_GEOMETRY = (
    ROOT
    / "isaac_sim/assets/so101_new_calib/payloads"
    / "wrist_camera_mount_geometry.usd"
)
CAMERA_MOUNT_MESH = (
    DESCRIPTION / "meshes/wrist_cam_mount_32x32_uvc_module_so101.stl"
)
CAMERA_MOUNT_SHA256 = (
    "b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4"
)
XACRO_NS = "http://www.ros.org/wiki/xacro"


def binary_stl_bounds(path: Path):
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise ValueError(f"not a binary STL: {path}")

    vertices = []
    offset = 84
    for _ in range(triangle_count):
        facet = struct.unpack_from("<12fH", data, offset)
        offset += 50
        vertices.extend((facet[3:6], facet[6:9], facet[9:12]))
    return tuple(
        (min(vertex[axis] for vertex in vertices), max(vertex[axis] for vertex in vertices))
        for axis in range(3)
    )


def ascii_stl_bounds(path: Path):
    vertices = [
        tuple(float(value) for value in match.groups())
        for match in re.finditer(
            r"^\s*vertex\s+([^ ]+)\s+([^ ]+)\s+([^ \r\n]+)",
            path.read_text(encoding="ascii"),
            re.MULTILINE,
        )
    ]
    if not vertices:
        raise ValueError(f"not an ASCII STL: {path}")
    return tuple(
        (min(vertex[axis] for vertex in vertices), max(vertex[axis] for vertex in vertices))
        for axis in range(3)
    )


class WristCameraMountContractTest(unittest.TestCase):
    def test_asset_is_the_verified_official_so101_stl(self) -> None:
        digest = hashlib.sha256(CAMERA_MOUNT_MESH.read_bytes()).hexdigest()
        self.assertEqual(digest, CAMERA_MOUNT_SHA256)

    def test_replacement_retains_original_wrist_roll_cad_origin(self) -> None:
        original = binary_stl_bounds(ORIGINAL_MESH)
        replacement_mm = ascii_stl_bounds(CAMERA_MOUNT_MESH)
        replacement = tuple(
            (lower * 0.001, upper * 0.001)
            for lower, upper in replacement_mm
        )

        # The replacement grows only where the camera support is added. These
        # shared extrema prove that it retains the original mounting frame.
        self.assertAlmostEqual(replacement[0][0], original[0][0], places=7)
        self.assertAlmostEqual(replacement[0][1], original[0][1], places=7)
        self.assertAlmostEqual(replacement[1][0], original[1][0], places=7)
        self.assertAlmostEqual(replacement[2][1], original[2][1], places=7)

    def test_macro_scales_both_camera_mount_mesh_uses_from_mm(self) -> None:
        root = ET.parse(MACRO_PATH).getroot()
        meshes = [
            element
            for element in root.iter("mesh")
            if element.attrib.get("filename", "").endswith(
                "wrist_cam_mount_32x32_uvc_module_so101.stl"
            )
        ]
        self.assertEqual(len(meshes), 2)
        self.assertTrue(
            all(mesh.attrib.get("scale") == "0.001 0.001 0.001" for mesh in meshes)
        )

    def test_left_arm_enables_mount_but_macro_default_remains_generic(self) -> None:
        macro_root = ET.parse(MACRO_PATH).getroot()
        macro = macro_root.find(f"{{{XACRO_NS}}}macro")
        self.assertIsNotNone(macro)
        self.assertIn("use_wrist_camera_mount:=false", macro.attrib["params"])

        left_root = ET.parse(LEFT_PATH).getroot()
        arg = next(
            element
            for element in left_root.findall(f"{{{XACRO_NS}}}arg")
            if element.attrib.get("name") == "use_wrist_camera_mount"
        )
        self.assertEqual(arg.attrib.get("default"), "true")
        arm = left_root.find(f"{{{XACRO_NS}}}so101_arm")
        self.assertEqual(
            arm.attrib.get("use_wrist_camera_mount"),
            "$(arg use_wrist_camera_mount)",
        )

    def test_camera_mount_center_is_a_fixed_cad_frame(self) -> None:
        root = ET.parse(MACRO_PATH).getroot()
        joint = next(
            element
            for element in root.iter("joint")
            if element.attrib.get("name")
            == "${prefix}wrist_camera_mount_center_joint"
        )
        self.assertEqual(joint.attrib.get("type"), "fixed")
        self.assertEqual(
            joint.find("parent").attrib.get("link"),
            "${prefix}gripper_link",
        )
        self.assertEqual(
            joint.find("child").attrib.get("link"),
            "${prefix}wrist_camera_mount_center_link",
        )
        origin = joint.find("origin")
        self.assertEqual(
            origin.attrib.get("xyz"),
            "0.00250008 -0.07292374 0.00595299",
        )
        self.assertEqual(origin.attrib.get("rpy"), "-2.70526034 0 0")

    def test_registration_marker_is_on_printed_plate_outer_surface(self) -> None:
        root = ET.parse(MACRO_PATH).getroot()
        joint = next(
            element
            for element in root.iter("joint")
            if element.attrib.get("name")
            == "${prefix}wrist_registration_marker_joint"
        )
        self.assertEqual(joint.attrib.get("type"), "fixed")
        self.assertEqual(
            joint.find("parent").attrib.get("link"),
            "${prefix}wrist_camera_mount_center_link",
        )
        self.assertEqual(
            joint.find("child").attrib.get("link"),
            "${prefix}wrist_registration_marker_link",
        )
        origin = joint.find("origin")
        self.assertEqual(origin.attrib.get("xyz"), "0 0 -0.002")
        self.assertEqual(origin.attrib.get("rpy"), "0 0 0")

    def test_isaac_visual_and_collision_reference_generated_geometry(self) -> None:
        text = ISAAC_INSTANCES.read_text()
        camera_reference = (
            "@./wrist_camera_mount_geometry.usd@"
            "</Geometries/wrist_roll_follower_so101_v1>"
        )
        original_reference = (
            "@./geometries.usd@"
            "</Geometries/wrist_roll_follower_so101_v1>"
        )
        self.assertTrue(ISAAC_GEOMETRY.is_file())
        self.assertEqual(text.count(camera_reference), 2)
        self.assertEqual(text.count(original_reference), 0)


if __name__ == "__main__":
    unittest.main()
