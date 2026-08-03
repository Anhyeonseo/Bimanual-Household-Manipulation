import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))
MODULE_PATH = ROOT / "tools" / "plan_buffered_action_commissioning.py"
SPEC = importlib.util.spec_from_file_location(
    "plan_buffered_action_commissioning",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"


def test_plan_is_non_executable_resampled_and_returns_to_anchor():
    document = MODULE.build_plan(
        CALIBRATION,
        CONTRACT,
        (2070, 2253, 1785, 1981, 2064, 2002),
        {
            "left_base_joint": 0.015,
            "left_shoulder_joint": 0.015,
            "left_wrist_roll_joint": 0.030,
        },
    )

    assert document["status"] == MODULE.STATUS
    assert document["execution_api_used"] is False
    assert document["motion_authorized"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["resampling"]["period_ms"] == 20
    assert document["resampling"]["duration_ms"] == 1200
    assert document["resampling"]["sample_count"] == 61
    assert document["queue_contract"]["admission_batch_sizes"] == [
        9,
        7,
        6,
        6,
        6,
        6,
        6,
        6,
        6,
        3,
    ]
    assert document["queue_contract"]["simulation_terminal"] == {
        "state": "input_complete",
        "accepted_samples": 61,
        "applied_samples": 48,
        "queued_samples": 13,
        "safe_stop_required": False,
        "success_without_firmware_terminal": False,
    }
    assert document["apex"]["delta_raw"] == [10, 10, 0, 0, 20, 0]
    assert document["round_trip"]["maximum_return_error_rad"] == 0.0


def test_tool_has_no_ros_serial_or_execution_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "rclpy" not in source
    assert "serial_port" not in source
    assert "BufferedTransportDriver" not in source
    assert "ActionClient" not in source
