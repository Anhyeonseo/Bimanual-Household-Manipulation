from __future__ import annotations

import importlib.util
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generate_isaac_bimanual_preview_urdf",
    ROOT / "tools/setup/isaac/generate_isaac_bimanual_preview_urdf.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_joint_zero_offsets_change_only_right_joint_origins() -> None:
    joints = []
    for name in MODULE.RIGHT_ARM_JOINTS:
        joints.append(
            f'<joint name="{name}" type="revolute">'
            '<origin xyz="0 0 0" rpy="0 0 0"/>'
            '<axis xyz="0 1 0"/>'
            "</joint>"
        )
    joints.append(
        '<joint name="left_shoulder_joint" type="revolute">'
        '<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>'
        '<axis xyz="0 1 0"/>'
        "</joint>"
    )
    xml = '<robot name="preview">' + "".join(joints) + "</robot>"
    offsets = {name: 0.0 for name in MODULE.RIGHT_ARM_JOINTS}
    offsets["right_shoulder_joint"] = 0.05

    output = MODULE._apply_right_joint_zero_offsets(xml, offsets)
    root = ET.fromstring(output)
    updated = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    shoulder_rpy = np.fromstring(
        updated["right_shoulder_joint"].find("origin").attrib["rpy"],
        sep=" ",
    )

    assert np.allclose(
        Rotation.from_euler("xyz", shoulder_rpy).as_matrix(),
        Rotation.from_rotvec([0.0, 0.05, 0.0]).as_matrix(),
    )
    assert updated["left_shoulder_joint"].find("origin").attrib["rpy"] == (
        "0.1 0.2 0.3"
    )
    # Keep the SRDF robot identity so the same shadow model can be used by
    # MoveIt in allow_trajectory_execution=false plan-only mode.
    assert root.attrib["name"] == "preview"


def test_registration_candidate_must_be_validated_and_fail_closed() -> None:
    document = {
        "status": "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        "motion_authorized": False,
        "arm": "right",
        "method": MODULE.REGISTERED_METHOD,
        "right_kinematic_registration": {
            "training_only_fit": True,
            "validation_used_in_fit": False,
            "workcell_to_right_base": {
                "rows": 4,
                "cols": 4,
                "data": np.eye(4).tolist(),
            },
            "joint_zero_offsets_rad": {
                name: 0.0 for name in MODULE.RIGHT_ARM_JOINTS
            },
        },
    }
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "candidate.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        _, mount, offsets = MODULE._load_registration_candidate(path)
        assert np.allclose(mount, np.eye(4))
        assert tuple(offsets) == MODULE.RIGHT_ARM_JOINTS

        document["motion_authorized"] = True
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        with pytest.raises(RuntimeError, match="not independently validated"):
            MODULE._load_registration_candidate(path)
