import importlib.util
import math
import sys
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/camera_calibration"
SPEC = importlib.util.spec_from_file_location(
    "validate_right_registration_shadow",
    TOOLS / "validate_right_registration_shadow.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def valid_documents():
    candidate = {
        "status": "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        "arm": "right",
        "method": "workcell_anchored_right_joint_zero_bundle_adjustment",
        "motion_authorized": False,
        "robot_target_available": False,
        "right_kinematic_registration": {
            "training_only_fit": True,
            "validation_used_in_fit": False,
        },
    }
    session = {
        "status": "CAPTURE_SET_VALIDATED_READY_TO_SOLVE",
        "motion_authorized": False,
        "arm": "right",
        "validation_captures": [{"id": "v1"}, {"id": "v2"}],
    }
    workcell = {
        "status": "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        "motion_authorized": False,
        "base_registration": {"transform_validated": True},
    }
    return candidate, session, workcell


def test_requires_held_out_fail_closed_sources():
    candidate, session, workcell = valid_documents()
    MODULE.validate_documents(candidate, session, workcell)
    candidate["right_kinematic_registration"]["validation_used_in_fit"] = True
    with pytest.raises(RuntimeError, match="held-out"):
        MODULE.validate_documents(candidate, session, workcell)


def test_wrapped_angle_handles_pi_boundary():
    error = MODULE.wrapped_angle(math.radians(-179) - math.radians(179))
    assert math.degrees(error) == pytest.approx(2.0)


def test_shadow_status_never_authorizes_motion_or_tabletop_targets():
    source = (TOOLS / "validate_right_registration_shadow.py").read_text()
    assert '"motion_authorized": False' in source
    assert '"robot_target_available": False' in source
    assert '"tabletop_object_validation_performed": False' in source
