#!/usr/bin/env python3
"""Measure the Pi camera/policy runtime without publishing robot commands."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import subprocess
import time
from typing import Any, Callable


CAMERA_NAMES = ("top", "wrist_a", "wrist_b")
POLICY_STATUS_NAME = "policy_runtime/shadow"
POLICY_COUNTER_KEYS = (
    "inference_count",
    "deadline_misses",
    "stale_observations",
    "rejected_outputs",
    "command_publications",
)
DEFAULT_SCHEDULE = Path(__file__).parents[1] / "config" / "camera_schedule.json"


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = fraction * (len(ordered) - 1)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "count": 0,
            "average": 0.0,
            "min": 0.0,
            "p50": 0.0,
            "p95": 0.0,
            "max": 0.0,
        }
    return {
        "count": len(values),
        "average": statistics.fmean(values),
        "min": min(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def load_phase_budget(path: Path, phase: str) -> tuple[dict[str, float], float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("camera schedule schema_version must be 1")
    phases = data.get("phases")
    if not isinstance(phases, dict) or phase not in phases:
        raise ValueError(f"camera phase is not defined: {phase}")
    selected = phases[phase]
    targets: dict[str, float] = {}
    for camera in CAMERA_NAMES:
        budget = selected.get(camera)
        if not isinstance(budget, dict):
            raise ValueError(f"missing camera budget: {phase}.{camera}")
        rate = budget.get("decode_hz")
        if not isinstance(rate, (int, float)) or isinstance(rate, bool) or rate < 0:
            raise ValueError(f"invalid decode_hz: {phase}.{camera}")
        targets[camera] = float(rate)
    policy_hz = selected.get("policy_hz")
    if not isinstance(policy_hz, (int, float)) or isinstance(policy_hz, bool) or policy_hz < 0:
        raise ValueError(f"invalid policy_hz: {phase}")
    return targets, float(policy_hz)


def parse_process_spec(value: str) -> tuple[str, int]:
    try:
        name, raw_pid = value.rsplit("=", maxsplit=1)
        pid = int(raw_pid)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("process must use NAME=PID") from exc
    if not name or pid <= 0:
        raise argparse.ArgumentTypeError("process must use a non-empty NAME and positive PID")
    return name, pid


def read_cpu_times(path: Path = Path("/proc/stat")) -> tuple[int, int]:
    fields = path.read_text(encoding="utf-8").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def cpu_percent(before: tuple[int, int], after: tuple[int, int]) -> float:
    total = after[0] - before[0]
    idle = after[1] - before[1]
    return 0.0 if total <= 0 else 100.0 * (1.0 - idle / total)


def read_memory_mb(path: Path = Path("/proc/meminfo")) -> tuple[float, float]:
    fields: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, value = line.split(":", maxsplit=1)
        fields[key] = int(value.strip().split()[0])
    total_mb = fields["MemTotal"] / 1024.0
    available_mb = fields["MemAvailable"] / 1024.0
    return total_mb - available_mb, available_mb


def read_swap_counters(path: Path = Path("/proc/vmstat")) -> tuple[int, int]:
    fields: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, value = line.split()
        if key in {"pswpin", "pswpout"}:
            fields[key] = int(value)
    return fields.get("pswpin", 0), fields.get("pswpout", 0)


def read_temperature_c(path: Path = Path("/sys/class/thermal/thermal_zone0/temp")) -> float:
    return int(path.read_text(encoding="utf-8").strip()) / 1000.0


def read_process_rss_mb(pid: int, proc_root: Path = Path("/proc")) -> float:
    for line in (proc_root / str(pid) / "status").read_text(encoding="utf-8").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024.0
    raise RuntimeError(f"VmRSS is missing for PID {pid}")


def read_throttled_flags(
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> int | None:
    try:
        result = runner(
            ["vcgencmd", "get_throttled"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    output = result.stdout.strip()
    if result.returncode != 0 or "=" not in output:
        return None
    try:
        return int(output.split("=", maxsplit=1)[1], 16)
    except ValueError:
        return None


def status_values(status: Any) -> dict[str, str]:
    return {item.key: item.value for item in status.values}


def diagnostic_map(message: Any | None) -> dict[str, tuple[int, str, dict[str, str]]]:
    if message is None:
        return {}
    return {
        status.name: (int(status.level), status.message, status_values(status))
        for status in message.status
    }


def _as_int(values: dict[str, str], key: str, failures: list[str]) -> int:
    try:
        parsed = int(values[key])
    except (KeyError, ValueError):
        failures.append(f"policy diagnostic {key} is missing or invalid")
        return 0
    if parsed < 0:
        failures.append(f"policy diagnostic {key} must be nonnegative")
        return 0
    return parsed


def _as_float(values: dict[str, str], key: str, failures: list[str]) -> float:
    try:
        parsed = float(values[key])
    except (KeyError, ValueError):
        failures.append(f"policy diagnostic {key} is missing or invalid")
        return 0.0
    if parsed < 0:
        failures.append(f"policy diagnostic {key} must be nonnegative")
        return 0.0
    return parsed


def is_sha256(value: str) -> bool:
    return len(value) == 64 and all(
        character in "0123456789abcdefABCDEF" for character in value
    )


def policy_counter_delta_entry(
    final_entry: tuple[int, str, dict[str, str]] | None,
    baseline_values: dict[str, str] | None,
) -> tuple[int, str, dict[str, str]] | None:
    """Replace monotonic policy counters with this measurement window's deltas."""
    if final_entry is None or baseline_values is None:
        return None
    level, message, final_values = final_entry
    adjusted = dict(final_values)
    for key in POLICY_COUNTER_KEYS:
        try:
            adjusted[key] = str(int(final_values[key]) - int(baseline_values[key]))
        except (KeyError, ValueError):
            adjusted[key] = "INVALID"
    return level, message, adjusted


def classify_bridge_error(logger_name: str, message: str) -> str | None:
    if not logger_name.endswith("single_arm_bridge"):
        return None
    lowered = message.lower()
    for category, markers in (
        ("heartbeat", ("transient heartbeat delay", "heartbeat error:")),
        ("feedback", ("transient feedback delay", "feedback error:")),
        ("safety_latch", ("safety latch error:",)),
    ):
        if any(marker in lowered for marker in markers):
            return category
    return None


def validate_policy_shadow(
    entry: tuple[int, str, dict[str, str]] | None,
    elapsed_s: float,
    target_hz: float,
    maximum_p95_ms: float,
) -> tuple[dict[str, Any], list[str]]:
    failures: list[str] = []
    if entry is None:
        return {}, [f"{POLICY_STATUS_NAME} diagnostics are missing"]
    level, message, values = entry
    if level != 0:
        failures.append(f"policy diagnostic level={level} message={message}")
    if values.get("mode") != "SHADOW":
        failures.append("policy mode must be SHADOW")
    inference_count = _as_int(values, "inference_count", failures)
    inference_p95_ms = _as_float(values, "inference_p95_ms", failures)
    deadline_misses = _as_int(values, "deadline_misses", failures)
    command_publications = _as_int(values, "command_publications", failures)
    model_sha256 = values.get("model_sha256", "")
    observation_sha256 = values.get("observation_contract_sha256", "")
    if not is_sha256(model_sha256):
        failures.append("policy model_sha256 must contain 64 hex characters")
    if not is_sha256(observation_sha256):
        failures.append("policy observation_contract_sha256 must contain 64 hex characters")
    measured_hz = inference_count / elapsed_s if elapsed_s > 0 else 0.0
    if target_hz > 0 and measured_hz < target_hz * 0.90:
        failures.append(f"policy inference rate={measured_hz:.3f}Hz below target")
    if inference_p95_ms > maximum_p95_ms:
        failures.append(f"policy inference p95={inference_p95_ms:.3f}ms")
    if deadline_misses != 0:
        failures.append(f"policy deadline_misses={deadline_misses}")
    if command_publications != 0:
        failures.append(f"policy command_publications={command_publications}")
    return {
        "status_message": message,
        "mode": values.get("mode", ""),
        "model_sha256": model_sha256,
        "observation_contract_sha256": observation_sha256,
        "inference_count": inference_count,
        "measured_hz": measured_hz,
        "inference_p50_ms": _as_float(values, "inference_p50_ms", failures),
        "inference_p95_ms": inference_p95_ms,
        "inference_max_ms": _as_float(values, "inference_max_ms", failures),
        "deadline_misses": deadline_misses,
        "stale_observations": _as_int(values, "stale_observations", failures),
        "rejected_outputs": _as_int(values, "rejected_outputs", failures),
        "command_publications": command_publications,
    }, failures


@dataclass
class Measurements:
    image_counts: dict[str, int]
    image_bytes: dict[str, int]
    image_age_ms: dict[str, list[float]]
    checksum: int = 0
    joint_count: int = 0
    joint_intervals_s: list[float] = field(default_factory=list)
    last_joint_at: float | None = None
    bridge_error_events: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def create(cls) -> "Measurements":
        return cls(
            image_counts={name: 0 for name in CAMERA_NAMES},
            image_bytes={name: 0 for name in CAMERA_NAMES},
            image_age_ms={name: [] for name in CAMERA_NAMES},
        )

    def reset(self) -> None:
        for name in CAMERA_NAMES:
            self.image_counts[name] = 0
            self.image_bytes[name] = 0
            self.image_age_ms[name].clear()
        self.checksum = 0
        self.joint_count = 0
        self.joint_intervals_s.clear()
        self.last_joint_at = None
        self.bridge_error_events.clear()


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--warmup", type=float, default=10.0)
    parser.add_argument("--phase", default="RUNTIME_BASELINE")
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-missing-joint-states", action="store_true")
    parser.add_argument("--require-policy", action="store_true")
    parser.add_argument("--policy-topic", default="/policy_runtime/diagnostics")
    parser.add_argument("--max-policy-p95-ms", type=float, default=80.0)
    parser.add_argument("--require-throttling-status", action="store_true")
    parser.add_argument("--process", action="append", default=[], type=parse_process_spec)
    args = parser.parse_args()
    if args.duration < 30.0 or args.warmup < 3.0:
        parser.error("duration must be >=30s and warmup must be >=3s")
    if args.max_policy_p95_ms <= 0:
        parser.error("max-policy-p95-ms must be positive")
    process_names = [name for name, _ in args.process]
    if len(process_names) != len(set(process_names)):
        parser.error("process NAME values must be unique")

    camera_targets, policy_target_hz = load_phase_budget(args.schedule, args.phase)
    if args.require_policy and policy_target_hz <= 0:
        parser.error("selected phase has policy_hz=0")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("pi_runtime_resource_baseline")
    measurements = Measurements.create()
    latest_camera_diagnostics: list[Any | None] = [None]
    latest_policy_diagnostics: list[Any | None] = [None]

    def image_callback(name: str):
        def callback(message: Image) -> None:
            measurements.image_counts[name] += 1
            measurements.image_bytes[name] += len(message.data)
            measurements.checksum = (
                measurements.checksum + sum(message.data[::4096])
            ) & 0xFFFFFFFF
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age_ms = max(0.0, (node.get_clock().now().nanoseconds - stamp_ns) / 1e6)
            measurements.image_age_ms[name].append(age_ms)

        return callback

    def joint_callback(_: JointState) -> None:
        sampled_at = time.monotonic()
        if measurements.last_joint_at is not None:
            measurements.joint_intervals_s.append(sampled_at - measurements.last_joint_at)
        measurements.last_joint_at = sampled_at
        measurements.joint_count += 1

    def rosout_callback(message: Log) -> None:
        category = classify_bridge_error(message.name, message.msg)
        if category is None:
            return
        measurements.bridge_error_events.append(
            {
                "category": category,
                "level": int(message.level),
                "message": message.msg,
            }
        )

    subscriptions: list[Any] = []
    for name, target_hz in camera_targets.items():
        if target_hz > 0:
            subscriptions.append(
                node.create_subscription(
                    Image,
                    f"/camera/{name}/image_raw",
                    image_callback(name),
                    qos_profile_sensor_data,
                )
            )
    subscriptions.append(node.create_subscription(JointState, "/joint_states", joint_callback, 10))
    subscriptions.append(node.create_subscription(Log, "/rosout", rosout_callback, 100))
    subscriptions.append(
        node.create_subscription(
            DiagnosticArray,
            "/camera_diagnostics",
            lambda message: latest_camera_diagnostics.__setitem__(0, message),
            10,
        )
    )
    subscriptions.append(
        node.create_subscription(
            DiagnosticArray,
            args.policy_topic,
            lambda message: latest_policy_diagnostics.__setitem__(0, message),
            10,
        )
    )
    phase_publisher = node.create_publisher(String, "/camera_phase", 10)

    def spin_for(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    failures: list[str] = []
    cpu_samples: list[float] = []
    used_memory_samples: list[float] = []
    available_memory_samples: list[float] = []
    temperature_samples: list[float] = []
    throttled_samples: list[int] = []
    process_rss_samples: dict[str, list[float]] = {name: [] for name, _ in args.process}
    report: dict[str, Any] = {
        "schema_version": 1,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "safety": {
            "robot_command_topics_created": 0,
            "camera_phase_publication_only": True,
            "motion_authorized": False,
        },
        "phase": args.phase,
        "duration_s": args.duration,
        "warmup_s": args.warmup,
        "policy_required": args.require_policy,
    }

    try:
        spin_for(1.0)
        phase_publisher.publish(String(data=args.phase))
        phase_deadline = time.monotonic() + 8.0
        while time.monotonic() < phase_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            diagnostics = diagnostic_map(latest_camera_diagnostics[0])
            scheduler = diagnostics.get("camera_manager/scheduler")
            if scheduler and scheduler[2].get("active_phase") == args.phase:
                break
        else:
            raise RuntimeError(f"{args.phase} phase acknowledgement timeout")

        spin_for(args.warmup)
        if not args.allow_missing_joint_states and measurements.joint_count == 0:
            raise RuntimeError("no /joint_states received during warmup")
        camera_start = diagnostic_map(latest_camera_diagnostics[0])
        reconnect_start = {
            name: int(camera_start.get(f"camera_manager/{name}", (0, "", {}))[2].get("reconnect_count", "0"))
            for name in CAMERA_NAMES
        }
        policy_start_entry = diagnostic_map(latest_policy_diagnostics[0]).get(
            POLICY_STATUS_NAME
        )
        if args.require_policy and policy_start_entry is None:
            raise RuntimeError(
                f"{POLICY_STATUS_NAME} diagnostics are missing during warmup"
            )
        policy_start_values = policy_start_entry[2] if policy_start_entry else None
        measurements.reset()

        swap_before = read_swap_counters()
        cpu_before = read_cpu_times()
        next_resource_sample = time.monotonic() + 1.0
        started_at = time.monotonic()
        finished_at = started_at + args.duration
        print(
            f"PI_RUNTIME_BASELINE_STARTED PHASE={args.phase} "
            f"DURATION={args.duration:.0f}s POLICY_REQUIRED={int(args.require_policy)}",
            flush=True,
        )

        while time.monotonic() < finished_at:
            rclpy.spin_once(node, timeout_sec=0.05)
            sampled_at = time.monotonic()
            if sampled_at >= next_resource_sample:
                cpu_after = read_cpu_times()
                cpu_samples.append(cpu_percent(cpu_before, cpu_after))
                cpu_before = cpu_after
                used_mb, available_mb = read_memory_mb()
                used_memory_samples.append(used_mb)
                available_memory_samples.append(available_mb)
                temperature_samples.append(read_temperature_c())
                throttled = read_throttled_flags()
                if throttled is not None:
                    throttled_samples.append(throttled)
                for name, pid in args.process:
                    process_rss_samples[name].append(read_process_rss_mb(pid))
                next_resource_sample += 1.0

        elapsed = time.monotonic() - started_at
        camera_diagnostics = diagnostic_map(latest_camera_diagnostics[0])
        camera_report: dict[str, Any] = {}
        for name, target_hz in camera_targets.items():
            if target_hz <= 0:
                continue
            measured_hz = measurements.image_counts[name] / elapsed
            bandwidth_mbps = measurements.image_bytes[name] * 8.0 / elapsed / 1e6
            entry = camera_diagnostics.get(f"camera_manager/{name}")
            if entry is None:
                failures.append(f"{name} diagnostics missing")
                level, message, values = 2, "MISSING", {}
            else:
                level, message, values = entry
            decode_failures = int(values.get("decode_failures", "-1"))
            age_p95 = float(values.get("decode_frame_age_p95_ms", "inf"))
            decode_p95 = float(values.get("decode_time_p95_ms", "inf"))
            reconnect_delta = int(values.get("reconnect_count", "0")) - reconnect_start[name]
            camera_report[name] = {
                "target_hz": target_hz,
                "measured_hz": measured_hz,
                "dds_mbps": bandwidth_mbps,
                "subscriber_frame_age_ms": summarize(measurements.image_age_ms[name]),
                "diagnostic_level": level,
                "diagnostic_message": message,
                "decode_failures": decode_failures,
                "decode_frame_age_p95_ms": age_p95,
                "decode_time_p95_ms": decode_p95,
                "reconnect_delta": reconnect_delta,
            }
            if measured_hz < target_hz * 0.90 or measured_hz > target_hz * 1.10:
                failures.append(f"{name} image rate={measured_hz:.3f}Hz")
            if level != 0 or message != "STREAMING":
                failures.append(f"{name} diagnostic={message} level={level}")
            if decode_failures != 0:
                failures.append(f"{name} decode_failures={decode_failures}")
            if age_p95 > 200.0:
                failures.append(f"{name} decode frame age p95={age_p95:.3f}ms")
            if decode_p95 > 50.0:
                failures.append(f"{name} decode p95={decode_p95:.3f}ms")
            if reconnect_delta != 0:
                failures.append(f"{name} reconnect_delta={reconnect_delta}")
        report["cameras"] = camera_report

        joint_hz = measurements.joint_count / elapsed
        joint_summary = summarize(measurements.joint_intervals_s)
        report["joint_states"] = {
            "required": not args.allow_missing_joint_states,
            "count": measurements.joint_count,
            "measured_hz": joint_hz,
            "interval_s": joint_summary,
        }
        if not args.allow_missing_joint_states:
            if not 4.8 <= joint_hz <= 5.2:
                failures.append(f"joint_states rate={joint_hz:.3f}Hz")
            if float(joint_summary["max"]) > 0.5:
                failures.append(f"joint_states max gap={joint_summary['max']:.3f}s")
        bridge_error_counts = {
            category: sum(
                event["category"] == category
                for event in measurements.bridge_error_events
            )
            for category in ("heartbeat", "feedback", "safety_latch")
        }
        report["bridge_stability"] = {
            "observation_source": "/rosout plus /joint_states continuity",
            "error_counts": bridge_error_counts,
            "events": measurements.bridge_error_events,
        }
        if measurements.bridge_error_events:
            failures.append(
                f"single_arm_bridge transport log events={len(measurements.bridge_error_events)}"
            )

        swap_after = read_swap_counters()
        throttled_or = 0
        for value in throttled_samples:
            throttled_or |= value
        report["resources"] = {
            "cpu_percent": summarize(cpu_samples),
            "memory_used_mb": summarize(used_memory_samples),
            "memory_available_mb": summarize(available_memory_samples),
            "temperature_c": summarize(temperature_samples),
            "swap_in_delta": swap_after[0] - swap_before[0],
            "swap_out_delta": swap_after[1] - swap_before[1],
            "throttled_status_available": bool(throttled_samples),
            "throttled_flags_or": throttled_or,
            "process_rss_mb": {
                name: summarize(values) for name, values in process_rss_samples.items()
            },
            "checksum": measurements.checksum,
        }
        if float(summarize(cpu_samples)["average"]) > 70.0:
            failures.append("average CPU exceeds 70%")
        if float(summarize(cpu_samples)["max"]) >= 90.0:
            failures.append("1s maximum CPU is at least 90%")
        if float(summarize(used_memory_samples)["max"]) > 3000.0:
            failures.append("used memory exceeds 3000MB")
        if float(summarize(available_memory_samples)["min"]) < 700.0:
            failures.append("minimum available memory is below 700MB")
        if float(summarize(temperature_samples)["max"]) >= 80.0:
            failures.append("temperature is at least 80C")
        if swap_after[0] != swap_before[0] or swap_after[1] != swap_before[1]:
            failures.append("swap activity detected")
        if args.require_throttling_status and not throttled_samples:
            failures.append("vcgencmd throttling status is unavailable")
        if throttled_or != 0:
            failures.append(f"throttling flags=0x{throttled_or:08X}")

        policy_entry = diagnostic_map(latest_policy_diagnostics[0]).get(
            POLICY_STATUS_NAME
        )
        if args.require_policy:
            policy_entry = policy_counter_delta_entry(policy_entry, policy_start_values)
            policy_report, policy_failures = validate_policy_shadow(
                policy_entry, elapsed, policy_target_hz, args.max_policy_p95_ms
            )
            report["policy"] = policy_report
            failures.extend(policy_failures)
        else:
            report["policy"] = {
                "required": False,
                "diagnostics_seen": policy_entry is not None,
            }
    except Exception as error:
        failures.append(str(error))
    finally:
        phase_publisher.publish(String(data="STANDBY"))
        spin_for(1.0)
        node.destroy_node()
        rclpy.shutdown()

    report["finished_at"] = datetime.now(timezone.utc).isoformat()
    report["passed"] = not failures
    report["failures"] = failures
    write_report(args.output, report)
    print(f"PI_RUNTIME_BASELINE_ARTIFACT={args.output}")
    if failures:
        print("PI_RUNTIME_RESOURCE_BASELINE_FAIL")
        for failure in failures:
            print(f"FAIL_REASON={failure}")
        return 1
    print("PI_RUNTIME_RESOURCE_BASELINE_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
