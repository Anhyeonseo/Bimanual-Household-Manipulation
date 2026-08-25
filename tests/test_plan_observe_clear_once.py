import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools/run"
SPEC = importlib.util.spec_from_file_location(
    "plan_observe_clear_once", TOOLS / "plan_observe_clear_once.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_inputs(directory: Path):
    joints = tuple(MODULE.CANONICAL_JOINTS)
    contract = directory / "contract.yaml"
    shadow = directory / "shadow.yaml"
    urdf = directory / "robot.urdf"
    manifest = directory / "robot.manifest.json"
    contract.write_text(yaml.safe_dump({
        "motion_authorized": False,
        "workcell_observation_candidate": {
            "motion_authorized": False,
            "observe_clear": {
                "status": "VISUAL_CANDIDATE",
                "visual_towel_occlusion": False,
                "motion_reproducibility_validated": False,
                "joint_names": list(joints),
                "joint_positions_rad": [0.0] * 12,
            },
        },
    }))
    shadow.write_text(yaml.safe_dump({
        "status": "RIGHT_REGISTRATION_WORKCELL_SHADOW_VALIDATED",
        "motion_authorized": False,
        "robot_target_available": False,
        "tabletop_object_validation_performed": False,
        "sources": {
            "candidate": {"sha256": "a" * 64},
            "worktable": {},
        },
    }))
    worktable = directory / "worktable.yaml"
    worktable.write_text(yaml.safe_dump({
        "status": "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        "base_registration": {"transform_validated": True},
        "board": {
            "calibrated_span_m": [0.377, 0.371],
            "origin_in_left_base_link_xy_m": [0.14, -0.307],
            "table_z_in_left_base_link_m": -0.005,
        },
    }))
    shadow_document = yaml.safe_load(shadow.read_text())
    shadow_document["sources"]["worktable"] = {
        "path": str(worktable),
        "sha256": MODULE.sha256_file(worktable),
    }
    shadow.write_text(yaml.safe_dump(shadow_document))
    urdf.write_text("<robot name='so101_dual_preview'/>")
    manifest.write_text(json.dumps({
        "simulation_only": True,
        "motion_authorized": False,
        "urdf": str(urdf),
        "urdf_sha256": MODULE.sha256_file(urdf),
        "right_registration_candidate": {
            "sha256": "a" * 64,
            "runtime_promotion_authorized": False,
        },
    }))
    return contract, shadow, manifest, urdf


def test_load_inputs_requires_matching_registered_plan_only_model(tmp_path, monkeypatch):
    contract, shadow, manifest, urdf = write_inputs(tmp_path)
    monkeypatch.setenv("SO101_DUAL_URDF_PATH", str(urdf))
    _, _, _, worktable, target = MODULE.load_inputs(contract, shadow, manifest)
    assert target == (0.0,) * 12
    scene = MODULE.validated_table_scene(worktable)
    collision = scene.world.collision_objects[0]
    assert collision.id == "validated_worktable_region"
    assert collision.header.frame_id == "left_base_link"
    assert collision.primitives[0].dimensions[0] == pytest.approx(0.377)
    document = json.loads(manifest.read_text())
    document["right_registration_candidate"]["sha256"] = "b" * 64
    manifest.write_text(json.dumps(document))
    with pytest.raises(RuntimeError, match="does not match"):
        MODULE.load_inputs(contract, shadow, manifest)


def test_load_inputs_accepts_motion_locked_supervised_clear(tmp_path, monkeypatch):
    contract, shadow, manifest, urdf = write_inputs(tmp_path)
    document = yaml.safe_load(contract.read_text())
    clear = document["workcell_observation_candidate"]["observe_clear"]
    clear.update({
        "status": "SUPERVISED_ROUNDTRIP_VALIDATED",
        "motion_reproducibility_validated": True,
        "all_four_towel_corners_visible": True,
        "coordinated_stop_verified": True,
    })
    contract.write_text(yaml.safe_dump(document))
    monkeypatch.setenv("SO101_DUAL_URDF_PATH", str(urdf))
    MODULE.load_inputs(contract, shadow, manifest)


def test_both_arms_request_excludes_grippers_but_preserves_full_start():
    request = MODULE.planning.both_arms_joint_request((0.0,) * 12, (0.1,) * 12)
    motion = request.motion_plan_request
    assert motion.group_name == "both_arms"
    assert tuple(motion.start_state.joint_state.name) == MODULE.CANONICAL_JOINTS
    names = tuple(item.joint_name for item in motion.goal_constraints[0].joint_constraints)
    assert names == MODULE.planning.BOTH_ARM_JOINTS
    assert "left_gripper_joint" not in names
    assert "right_gripper_joint" not in names


def test_collision_exceptions_are_local_reversible_and_bounded():
    from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

    matrix = AllowedCollisionMatrix()
    matrix.entry_names = ["left_shoulder_link"]
    row = AllowedCollisionEntry()
    row.enabled = [True]
    matrix.entry_values = [row]
    allowed = MODULE.collision_matrix_with_exceptions(matrix, True)
    restored = MODULE.collision_matrix_with_exceptions(allowed, False)
    indices = {name: index for index, name in enumerate(restored.entry_names)}
    for pair in MODULE.OBSERVE_CLEAR_CONTACT_EXCEPTIONS:
        first, second = tuple(pair)
        assert allowed.entry_values[indices[first]].enabled[indices[second]] is True
        assert restored.entry_values[indices[first]].enabled[indices[second]] is False


def test_collision_exceptions_are_restored_on_planning_failure():
    source = (TOOLS / "plan_observe_clear_once.py").read_text()
    assert "exceptions_enabled = False" in source
    assert "if exceptions_enabled and strict_matrix is not None" in source
    assert "emergency_restore" in source


def test_dense_path_samples_every_executed_linear_segment():
    class Point:
        def __init__(self, positions):
            self.positions = positions

    start = [0.0] * 12
    names = list(MODULE.planning.BOTH_ARM_JOINTS)
    samples = MODULE.dense_arm_path(start, names, [Point([0.025] * 10)], 0.01)
    assert len(samples) == 4
    assert samples[0] == (0.0,) * 10
    assert samples[-1] == (0.025,) * 10


def test_plan_contract_is_explicitly_motionless():
    source = (TOOLS / "plan_observe_clear_once.py").read_text()
    assert '"motion_authorized": False' in source
    assert '"automatic_execution_permitted": False' in source
    assert '"motion_commands": 0' in source


def test_staged_right_clearance_orders_base_shoulder_elbow_wrist():
    start = tuple(float(index) for index in range(12))
    target = tuple(float(index) + 0.5 for index in range(12))
    stages = MODULE.staged_right_clearance_targets(start, target)
    assert len(stages) == 7
    assert stages[0][6] == target[6]
    assert stages[0][7] == start[7]
    assert stages[1][7] == MODULE.RIGHT_CLEARANCE_SHOULDER_RAD
    assert stages[2][8] == MODULE.RIGHT_CLEARANCE_ELBOW_RAD
    assert stages[3][7] == target[7]
    assert stages[4][8] == target[8]
    assert stages[5][9] == target[9]
    assert stages[-1] == target
