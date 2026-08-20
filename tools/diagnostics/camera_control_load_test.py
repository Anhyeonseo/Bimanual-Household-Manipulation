#!/usr/bin/env python3
"""Measure camera decode/DDS load while checking STM32 bridge feedback stability."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import statistics
import time
from typing import Any


CAMERA_TARGET_HZ = {"top": 6.0, "wrist_a": 5.0, "wrist_b": 5.0}


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = fraction * (len(ordered) - 1)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def cpu_percent(before: tuple[int, int], after: tuple[int, int]) -> float:
    total = after[0] - before[0]
    idle = after[1] - before[1]
    return 0.0 if total <= 0 else 100.0 * (1.0 - idle / total)


def read_memory_mb() -> tuple[float, float]:
    fields: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, value = line.split(":", maxsplit=1)
        fields[key] = int(value.strip().split()[0])
    total_mb = fields["MemTotal"] / 1024.0
    available_mb = fields["MemAvailable"] / 1024.0
    return total_mb - available_mb, available_mb


def read_temperature_c() -> float:
    return int(
        Path("/sys/class/thermal/thermal_zone0/temp").read_text(
            encoding="utf-8"
        ).strip()
    ) / 1000.0


def read_swap_counters() -> tuple[int, int]:
    fields: dict[str, int] = {}
    for line in Path("/proc/vmstat").read_text(encoding="utf-8").splitlines():
        key, value = line.split()
        if key in {"pswpin", "pswpout"}:
            fields[key] = int(value)
    return fields.get("pswpin", 0), fields.get("pswpout", 0)


@dataclass
class Measurements:
    image_counts: dict[str, int] = field(
        default_factory=lambda: {name: 0 for name in CAMERA_TARGET_HZ}
    )
    image_bytes: dict[str, int] = field(
        default_factory=lambda: {name: 0 for name in CAMERA_TARGET_HZ}
    )
    checksum: int = 0
    joint_count: int = 0
    joint_intervals: list[float] = field(default_factory=list)
    last_joint_at: float | None = None

    def reset(self) -> None:
        for name in CAMERA_TARGET_HZ:
            self.image_counts[name] = 0
            self.image_bytes[name] = 0
        self.checksum = 0
        self.joint_count = 0
        self.joint_intervals.clear()
        self.last_joint_at = None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--warmup", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration < 30.0 or args.warmup < 3.0:
        parser.error("duration must be >=30s and warmup must be >=3s")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("camera_control_load_test")
    measurements = Measurements()
    latest_diagnostics: list[Any] = []

    def image_callback(name: str):
        def callback(message: Image) -> None:
            measurements.image_counts[name] += 1
            measurements.image_bytes[name] += len(message.data)
            # DDS already copied the full image. Touch one byte per 4KiB so the
            # consumer also exercises memory access without doing inference.
            measurements.checksum = (
                measurements.checksum + sum(message.data[::4096])
            ) & 0xFFFFFFFF

        return callback

    def joint_callback(_: JointState) -> None:
        now = time.monotonic()
        if measurements.last_joint_at is not None:
            measurements.joint_intervals.append(now - measurements.last_joint_at)
        measurements.last_joint_at = now
        measurements.joint_count += 1

    subscriptions = [
        node.create_subscription(
            Image,
            f"/camera/{name}/image_raw",
            image_callback(name),
            qos_profile_sensor_data,
        )
        for name in CAMERA_TARGET_HZ
    ]
    subscriptions.append(
        node.create_subscription(JointState, "/joint_states", joint_callback, 10)
    )
    subscriptions.append(
        node.create_subscription(
            DiagnosticArray,
            "/camera_diagnostics",
            lambda message: latest_diagnostics.append(message),
            10,
        )
    )
    phase_publisher = node.create_publisher(String, "/camera_phase", 10)

    def spin_for(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def diagnostic_values() -> dict[str, tuple[str, dict[str, str]]]:
        if not latest_diagnostics:
            return {}
        result = {}
        for status in latest_diagnostics[-1].status:
            result[status.name] = (
                status.message,
                {item.key: item.value for item in status.values},
            )
        return result

    failures: list[str] = []
    cpu_samples: list[float] = []
    used_memory_samples: list[float] = []
    available_memory_samples: list[float] = []
    temperature_samples: list[float] = []
    try:
        spin_for(1.5)
        phase_publisher.publish(String(data="DUAL_PRIVATE"))
        phase_deadline = time.monotonic() + 8.0
        while time.monotonic() < phase_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            diagnostics = diagnostic_values()
            scheduler = diagnostics.get("camera_manager/scheduler")
            if scheduler and scheduler[1].get("active_phase") == "DUAL_PRIVATE":
                break
        else:
            raise RuntimeError("DUAL_PRIVATE phase acknowledgement timeout")

        spin_for(args.warmup)
        if measurements.joint_count == 0:
            raise RuntimeError("no /joint_states received during warmup")
        measurements.reset()

        swap_before = read_swap_counters()
        cpu_before = read_cpu_times()
        next_resource_sample = time.monotonic() + 1.0
        started_at = time.monotonic()
        finished_at = started_at + args.duration
        print(
            f"LOAD_TEST_STARTED PHASE=DUAL_PRIVATE DURATION={args.duration:.0f}s",
            flush=True,
        )

        while time.monotonic() < finished_at:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_resource_sample:
                cpu_after = read_cpu_times()
                cpu_samples.append(cpu_percent(cpu_before, cpu_after))
                cpu_before = cpu_after
                used_mb, available_mb = read_memory_mb()
                used_memory_samples.append(used_mb)
                available_memory_samples.append(available_mb)
                temperature_samples.append(read_temperature_c())
                next_resource_sample += 1.0

        elapsed = time.monotonic() - started_at
        diagnostics = diagnostic_values()
        swap_after = read_swap_counters()

        for name, target_hz in CAMERA_TARGET_HZ.items():
            measured_hz = measurements.image_counts[name] / elapsed
            bandwidth_mbps = measurements.image_bytes[name] * 8.0 / elapsed / 1e6
            print(
                f"IMAGE name={name} target_hz={target_hz:.2f} "
                f"measured_hz={measured_hz:.2f} dds_mbps={bandwidth_mbps:.2f}"
            )
            if measured_hz < target_hz * 0.90 or measured_hz > target_hz * 1.10:
                failures.append(
                    f"{name} image rate {measured_hz:.2f}Hz outside target tolerance"
                )

            camera_diagnostic = diagnostics.get(f"camera_manager/{name}")
            if camera_diagnostic is None:
                failures.append(f"{name} diagnostics missing")
                continue
            state, values = camera_diagnostic
            decode_failures = int(values.get("decode_failures", "-1"))
            age_p95 = float(values.get("decode_frame_age_p95_ms", "inf"))
            decode_p95 = float(values.get("decode_time_p95_ms", "inf"))
            print(
                f"CAMERA_DIAG name={name} state={state} failures={decode_failures} "
                f"age_p95_ms={age_p95:.2f} decode_p95_ms={decode_p95:.2f}"
            )
            if state != "STREAMING":
                failures.append(f"{name} state is {state}")
            if decode_failures != 0:
                failures.append(f"{name} decode failures={decode_failures}")
            if age_p95 > 200.0:
                failures.append(f"{name} frame age p95={age_p95:.2f}ms")
            if decode_p95 > 50.0:
                failures.append(f"{name} decode p95={decode_p95:.2f}ms")

        joint_hz = measurements.joint_count / elapsed
        joint_p95 = percentile(measurements.joint_intervals, 0.95)
        joint_max = max(measurements.joint_intervals, default=0.0)
        print(
            f"JOINT_STATES count={measurements.joint_count} rate_hz={joint_hz:.3f} "
            f"interval_p95_ms={joint_p95 * 1000.0:.2f} "
            f"interval_max_ms={joint_max * 1000.0:.2f}"
        )
        if not 4.8 <= joint_hz <= 5.2:
            failures.append(f"joint_states rate={joint_hz:.3f}Hz")
        if joint_max > 0.5:
            failures.append(f"joint_states max gap={joint_max:.3f}s")

        cpu_average = statistics.fmean(cpu_samples)
        cpu_p95 = percentile(cpu_samples, 0.95)
        cpu_max = max(cpu_samples)
        memory_used_max = max(used_memory_samples)
        memory_available_min = min(available_memory_samples)
        temperature_max = max(temperature_samples)
        swap_in_delta = swap_after[0] - swap_before[0]
        swap_out_delta = swap_after[1] - swap_before[1]
        print(
            f"RESOURCES cpu_avg={cpu_average:.2f}% cpu_p95={cpu_p95:.2f}% "
            f"cpu_max_1s={cpu_max:.2f}% memory_used_max_mb={memory_used_max:.1f} "
            f"memory_available_min_mb={memory_available_min:.1f} "
            f"temp_max_c={temperature_max:.1f} swap_in={swap_in_delta} "
            f"swap_out={swap_out_delta} checksum={measurements.checksum}"
        )
        if cpu_average > 70.0:
            failures.append(f"average CPU={cpu_average:.2f}%")
        if cpu_max >= 90.0:
            failures.append(f"1s maximum CPU={cpu_max:.2f}%")
        if memory_used_max > 3000.0:
            failures.append(f"used memory={memory_used_max:.1f}MB")
        if memory_available_min < 700.0:
            failures.append(f"available memory={memory_available_min:.1f}MB")
        if temperature_max >= 80.0:
            failures.append(f"temperature={temperature_max:.1f}C")
        if swap_in_delta != 0 or swap_out_delta != 0:
            failures.append(
                f"swap activity in={swap_in_delta} out={swap_out_delta}"
            )
    except Exception as error:
        failures.append(str(error))
    finally:
        phase_publisher.publish(String(data="STANDBY"))
        spin_for(1.2)
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        print("CAMERA_CONTROL_LOAD_FAIL")
        for failure in failures:
            print(f"FAIL_REASON={failure}")
        return 1
    print("CAMERA_CONTROL_LOAD_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
