import argparse
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools.pi_runtime_resource_baseline import (
    POLICY_STATUS_NAME,
    classify_bridge_error,
    cpu_percent,
    load_phase_budget,
    parse_process_spec,
    percentile,
    policy_counter_delta_entry,
    read_process_rss_mb,
    read_throttled_flags,
    ros_uint8,
    summarize,
    validate_policy_shadow,
    write_report,
)


ROOT = Path(__file__).parents[1]


class Value:
    def __init__(self, key, value):
        self.key = key
        self.value = value


class PiRuntimeResourceBaselineTests(unittest.TestCase):
    def test_repository_dual_private_budget(self):
        cameras, policy_hz = load_phase_budget(
            ROOT / "config" / "camera_schedule.json", "DUAL_PRIVATE"
        )
        self.assertEqual(cameras, {"top": 6.0, "wrist_a": 5.0, "wrist_b": 5.0})
        self.assertEqual(policy_hz, 0.0)

    def test_policy_assist_budget_requires_policy(self):
        cameras, policy_hz = load_phase_budget(
            ROOT / "config" / "camera_schedule.json", "POLICY_ASSIST"
        )
        self.assertEqual(cameras["top"], 6.0)
        self.assertEqual(policy_hz, 10.0)

    def test_runtime_baseline_runs_three_cameras_and_policy(self):
        cameras, policy_hz = load_phase_budget(
            ROOT / "config" / "camera_schedule.json", "RUNTIME_BASELINE"
        )
        self.assertEqual(cameras, {"top": 6.0, "wrist_a": 5.0, "wrist_b": 5.0})
        self.assertEqual(policy_hz, 10.0)

    def test_percentile_and_summary(self):
        self.assertEqual(percentile([1.0, 2.0, 3.0], 0.5), 2.0)
        self.assertEqual(summarize([])["count"], 0)
        self.assertEqual(summarize([1.0, 3.0])["average"], 2.0)

    def test_cpu_percent(self):
        self.assertAlmostEqual(cpu_percent((100, 40), (200, 60)), 80.0)
        self.assertEqual(cpu_percent((100, 40), (100, 40)), 0.0)

    def test_bridge_error_classification(self):
        self.assertEqual(
            classify_bridge_error(
                "single_arm_bridge",
                "transient heartbeat delay (1/3): timeout",
            ),
            "heartbeat",
        )
        self.assertEqual(
            classify_bridge_error(
                "/single_arm_bridge",
                "feedback error: GET_STATE position read failed",
            ),
            "feedback",
        )
        self.assertIsNone(classify_bridge_error("camera_manager", "feedback error"))

    def test_ros_uint8_accepts_integer_and_one_byte_value(self):
        self.assertEqual(ros_uint8(2), 2)
        self.assertEqual(ros_uint8(b"\x00"), 0)
        self.assertEqual(ros_uint8(bytearray(b"\xff")), 255)
        with self.assertRaises(ValueError):
            ros_uint8(b"")

    def test_process_spec(self):
        self.assertEqual(parse_process_spec("camera=123"), ("camera", 123))
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_process_spec("camera")

    def test_process_rss(self):
        with tempfile.TemporaryDirectory() as directory:
            status = Path(directory) / "42" / "status"
            status.parent.mkdir()
            status.write_text("Name:\ttest\nVmRSS:\t2048 kB\n", encoding="utf-8")
            self.assertEqual(read_process_rss_mb(42, Path(directory)), 2.0)

    def test_throttled_flags(self):
        runner = mock.Mock(
            return_value=mock.Mock(returncode=0, stdout="throttled=0x50005\n")
        )
        self.assertEqual(read_throttled_flags(runner), 0x50005)

    def test_policy_shadow_passes(self):
        values = {
            "mode": "SHADOW",
            "model_sha256": "a" * 64,
            "observation_contract_sha256": "b" * 64,
            "inference_count": "100",
            "inference_p50_ms": "10",
            "inference_p95_ms": "20",
            "inference_max_ms": "30",
            "deadline_misses": "0",
            "stale_observations": "0",
            "rejected_outputs": "0",
            "command_publications": "0",
        }
        report, failures = validate_policy_shadow(
            (0, "SHADOW", values), 10.0, 10.0, 80.0
        )
        self.assertEqual(failures, [])
        self.assertEqual(report["measured_hz"], 10.0)

    def test_policy_command_publication_fails(self):
        values = {
            "mode": "SHADOW",
            "model_sha256": "a" * 64,
            "observation_contract_sha256": "b" * 64,
            "inference_count": "100",
            "inference_p50_ms": "10",
            "inference_p95_ms": "20",
            "inference_max_ms": "30",
            "deadline_misses": "0",
            "stale_observations": "0",
            "rejected_outputs": "0",
            "command_publications": "1",
        }
        _, failures = validate_policy_shadow((0, "SHADOW", values), 10.0, 10.0, 80.0)
        self.assertTrue(any("command_publications" in item for item in failures))

    def test_policy_counters_use_measurement_window_delta(self):
        baseline = {
            "inference_count": "100",
            "deadline_misses": "2",
            "stale_observations": "4",
            "rejected_outputs": "1",
            "command_publications": "0",
        }
        final = {
            **baseline,
            "inference_count": "200",
            "deadline_misses": "2",
            "stale_observations": "5",
            "rejected_outputs": "1",
            "command_publications": "0",
        }
        adjusted = policy_counter_delta_entry((0, "SHADOW", final), baseline)
        self.assertIsNotNone(adjusted)
        self.assertEqual(adjusted[2]["inference_count"], "100")
        self.assertEqual(adjusted[2]["stale_observations"], "1")
        self.assertEqual(adjusted[2]["deadline_misses"], "0")

    def test_policy_hash_requires_hex(self):
        values = {
            "mode": "SHADOW",
            "model_sha256": "z" * 64,
            "observation_contract_sha256": "b" * 64,
            "inference_count": "100",
            "inference_p50_ms": "10",
            "inference_p95_ms": "20",
            "inference_max_ms": "30",
            "deadline_misses": "0",
            "stale_observations": "0",
            "rejected_outputs": "0",
            "command_publications": "0",
        }
        _, failures = validate_policy_shadow((0, "SHADOW", values), 10.0, 10.0, 80.0)
        self.assertTrue(any("model_sha256" in item for item in failures))

    def test_policy_contract_matches_runtime_topic(self):
        contract = json.loads(
            (ROOT / "config" / "policy_shadow_diagnostics_contract.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(contract["status_name"], POLICY_STATUS_NAME)
        self.assertEqual(contract["acceptance"]["command_publications"], 0)
        self.assertIn("end-minus-warmup", contract["counter_semantics"])

    def test_report_is_atomic_json(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result.json"
            write_report(output, {"passed": True})
            self.assertEqual(json.loads(output.read_text(encoding="utf-8")), {"passed": True})
            self.assertFalse(output.with_suffix(".json.tmp").exists())

    def test_tool_contains_no_robot_command_endpoints(self):
        source = (ROOT / "tools" / "pi_runtime_resource_baseline.py").read_text(
            encoding="utf-8"
        )
        for forbidden in (
            "follow_joint_trajectory",
            "parallel_gripper",
            "clear_fault",
            "setpoint",
            "motion_enable",
        ):
            self.assertNotIn(forbidden, source.lower())
        self.assertIn('create_publisher(String, "/camera_phase"', source)


if __name__ == "__main__":
    unittest.main()
