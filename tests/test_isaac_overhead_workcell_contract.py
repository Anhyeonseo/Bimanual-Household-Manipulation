from __future__ import annotations

import ast
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "tools/setup/isaac/generate_isaac_overhead_workcell.py"
ISAAC_ROOT = ROOT / "isaac_sim/assets/so101_new_calib/so101_new_calib.usda"
MESH_DIR = ROOT / "ros2_ws/src/so101_description/meshes"

EXPECTED_MESH_SHA256 = {
    "overhead_webcam_arm_base.stl": "169adfd40bcca689334efd1188c9b42cc03c914dc0afeaa98cb5431013610833",
    "overhead_webcam_cam_mount_bottom.stl": "b3545b6cae437210e17b7dcfee2e12e00dc7a59ece9264f4b13ab9fd8ceb8088",
    "overhead_webcam_cam_mount_top_hinge_removed.stl": "55319a9b26f9cdb7217c94000f7aec716d63b0a433150cd7672baa7b85a006cf",
}


def _assignments() -> dict[str, object]:
    tree = ast.parse(GENERATOR.read_text(encoding="utf-8"))
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        try:
            values[target.id] = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            continue
    return values


def test_isaac_root_composes_only_the_separate_workcell_layer() -> None:
    text = ISAAC_ROOT.read_text(encoding="utf-8")
    assert "subLayers = [" in text
    assert "@./payloads/overhead_workcell.usd@" in text


def test_isaac_generator_pins_the_confirmed_meshes() -> None:
    for filename, expected in EXPECTED_MESH_SHA256.items():
        actual = hashlib.sha256((MESH_DIR / filename).read_bytes()).hexdigest()
        assert actual == expected
    text = GENERATOR.read_text(encoding="utf-8")
    for filename, digest in EXPECTED_MESH_SHA256.items():
        assert filename in text
        assert digest in text


def test_isaac_local_transforms_match_the_confirmed_xacro() -> None:
    assignments = _assignments()
    assert assignments["ARM_BASE_TRANSLATION"] == (0.0702562210, -0.0534304657, 0.0)
    assert assignments["BOTTOM_TRANSLATION"] == (0.0137353073, -0.1525445687, -0.005)
    assert assignments["TOP_TRANSLATION"] == (0.0187, 0.2231, 0.0365125)
    assert assignments["CAMERA_TRANSLATION"] == (0.0, 0.2344404, 0.0)


def test_isaac_layer_is_fail_closed_for_physics_and_camera() -> None:
    text = GENERATOR.read_text(encoding="utf-8")
    assert '"visualOnly", True' in text
    assert '"collisionAuthorized", False' in text
    assert '"cameraExtrinsicCalibrated", False' in text
    assert "UsdPhysics" not in text
    assert "UsdGeom.Camera" not in text
