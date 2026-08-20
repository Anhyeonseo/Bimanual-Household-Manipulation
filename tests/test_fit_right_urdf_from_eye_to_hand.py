from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "fit_right_urdf_from_eye_to_hand",
    ROOT / "tools/setup/camera_calibration/fit_right_urdf_from_eye_to_hand.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_zero_origin_correction_is_identity() -> None:
    assert np.allclose(MODULE.correction_transform(np.zeros(6)), np.eye(4))


def test_preview_correction_changes_only_right_joint_origins() -> None:
    joints = []
    chain = []
    for index, name in enumerate(MODULE.FIT_JOINTS):
        joints.append(
            f'<joint name="{name}" type="revolute">'
            f'<origin xyz="{index} 0 0" rpy="0 0 0"/>'
            "</joint>"
        )
        chain.append(
            MODULE.JointModel(
                name=name,
                joint_type="revolute",
                origin=MODULE.transform(
                    np.eye(3),
                    np.asarray([float(index), 0.0, 0.0]),
                ),
                axis=np.asarray([0.0, 0.0, 1.0]),
            )
        )
    joints.append(
        '<joint name="left_shoulder_joint" type="revolute">'
        '<origin xyz="9 8 7" rpy="0 0 0"/>'
        "</joint>"
    )
    xml = '<robot name="preview">' + "".join(joints) + "</robot>"
    corrections = {
        name: np.zeros(6, dtype=np.float64) for name in MODULE.FIT_JOINTS
    }
    corrections["right_shoulder_joint"][0] = 0.006

    root, records = MODULE.apply_corrections_to_preview(
        xml,
        chain,
        corrections,
    )

    updated = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    right_xyz = np.fromstring(
        updated["right_shoulder_joint"].find("origin").attrib["xyz"],
        sep=" ",
    )
    assert np.allclose(right_xyz, [1.006, 0.0, 0.0])
    assert updated["left_shoulder_joint"].find("origin").attrib["xyz"] == (
        "9 8 7"
    )
    assert len(records) == len(MODULE.FIT_JOINTS)
    assert root.attrib["name"] == "so101_dual_right_data_fit_preview"


def test_generated_preview_is_fail_closed_by_source_contract() -> None:
    source = (
        ROOT / "tools/setup/camera_calibration/fit_right_urdf_from_eye_to_hand.py"
    ).read_text(encoding="utf-8")
    assert '"simulation_only": True' in source
    assert '"motion_authorized": False' in source
    assert '"validation_used_in_fit": True' in source
    assert "VISUAL_PREVIEW_ONLY_NOT_CALIBRATION_APPROVED" in source
