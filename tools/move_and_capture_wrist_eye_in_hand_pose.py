#!/usr/bin/env python3
"""Move to one explicit joint-space pose and capture a wrist eye-in-hand
sample there. One pose = one confirmed physical move.

This is an orchestrator, not a new motion path: it chains the existing
Motion-14 pipeline exactly as execute_grasp_convergence_once.py's
plan_and_execute_leg() does --

    capture_buffered_anchor_raw.py         torque-held anchor (read-only)
    ros_moveit_plan_pregrasp_segments.py   explicit joint-space segments
    plan_buffered_segment_leg.py           20 ms buffered leg + contract check
    execute_buffered_segment_leg_once.py   Action, single send, no retries

then, once the arm has settled at the target, runs
capture_wrist_eye_in_hand_sample.py to record the image/joint_state pair.

**All physical motion still goes through the same single-send, no-retry
executor.** This tool adds no new motion path; it only removes the manual
copy-pasting of --start, anchor-raw, and sha256 values between steps for a
repeated procedure (>=10 poses per W2 capture session).

**Does not open the serial port.** Every step here talks to the bridge over
ROS, exactly like the tools it wraps.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

CONFIRMATION = "EXECUTE_WRIST_EYE_IN_HAND_POSE_MOVE_ONCE"
SEGMENT_LEG_CONFIRMATION = "EXECUTE_MOTION14_FRESH_SEGMENT_LEG_ONCE"
ARM_JOINT_NAMES = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
JOINT_STATE_TIMEOUT_S = 10.0


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def run(command: list[str], label: str) -> str:
    """Call a sub-tool and show its output verbatim. Any failure stops here."""
    print(f"\n----- {label} -----")
    print("$ " + " ".join(command))
    completed = subprocess.run(
        command, capture_output=True, text=True, cwd=str(ROOT)
    )
    output = completed.stdout + completed.stderr
    print(output.rstrip())
    if completed.returncode != 0:
        raise RuntimeError(f"{label} failed with exit {completed.returncode}")
    return output


def read_joint_state(arm_names: tuple[str, ...]) -> tuple[float, ...]:
    """Read /joint_states once. Never opens the serial port."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("wrist_eye_in_hand_pose_joint_state_reader")
    latest: dict[str, float] = {}
    try:
        def on_message(message: JointState) -> None:
            for name, position in zip(message.name, message.position):
                latest[name] = float(position)

        node.create_subscription(JointState, "/joint_states", on_message, 10)
        deadline = time.monotonic() + JOINT_STATE_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(name in latest for name in arm_names):
                return tuple(latest[name] for name in arm_names)
        raise TimeoutError(
            "/joint_states did not carry every arm joint; is the bridge up?"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def capture_anchor_raw(
    workdir: Path,
    capture_id: str,
    bridge_calibration: Path,
    samples: int,
    timeout_s: float,
    maximum_spread_raw: int,
) -> tuple[int, ...]:
    import json

    anchor_path = workdir / f"{capture_id}_anchor.json"
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "capture_buffered_anchor_raw.py"),
            "--calibration",
            str(bridge_calibration),
            "--samples",
            str(samples),
            "--timeout-s",
            str(timeout_s),
            "--maximum-spread-raw",
            str(maximum_spread_raw),
            "--output",
            str(anchor_path),
        ],
        f"{capture_id}: torque-held anchor capture",
    )
    document = json.loads(anchor_path.read_text(encoding="utf-8"))
    return tuple(int(value) for value in document["anchor_raw"])


def plan_and_execute_leg(
    workdir: Path,
    capture_id: str,
    calibration: Path,
    start_rad: tuple[float, ...],
    target_rad: tuple[float, ...],
    anchor_raw: tuple[int, ...],
) -> str:
    """One pass through the Motion-14 pipeline. Identical body to
    execute_grasp_convergence_once.py's plan_and_execute_leg()."""
    segments = workdir / f"{capture_id}_segments.json"
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"),
            "--plan-only",
            "--calibration",
            str(calibration),
            "--start",
            ",".join(f"{value:.12f}" for value in start_rad),
            "--target-joints",
            ",".join(f"{value:.12f}" for value in target_rad),
            "--output",
            str(segments),
        ],
        f"{capture_id}: collision-checked joint segments",
    )

    leg = workdir / f"{capture_id}_leg.json"
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "plan_buffered_segment_leg.py"),
            "--plan-only",
            "--segments",
            str(segments),
            "--segments-sha256",
            sha256_file(segments),
            "--anchor-raw",
            *[str(value) for value in anchor_raw],
            "--output",
            str(leg),
        ],
        f"{capture_id}: buffered leg",
    )

    return run(
        [
            sys.executable,
            str(ROOT / "tools" / "execute_buffered_segment_leg_once.py"),
            str(leg),
            "--expected-sha256",
            sha256_file(leg),
            "--confirmation",
            SEGMENT_LEG_CONFIRMATION,
        ],
        f"{capture_id}: execute (physical motion)",
    )


def parse_joint_vector(value: str) -> tuple[float, ...]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 5:
        raise argparse.ArgumentTypeError(
            f"expected 5 comma-separated radians, got {len(parts)}"
        )
    return tuple(float(part) for part in parts)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--target-joints", required=True, type=parse_joint_vector)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config" / "single_arm_calibration.json",
    )
    parser.add_argument(
        "--bridge-calibration",
        type=Path,
        default=PACKAGE / "config" / "single_arm_calibration.json",
    )
    parser.add_argument("--anchor-samples", type=int, default=15)
    parser.add_argument("--anchor-timeout-s", type=float, default=20.0)
    parser.add_argument("--anchor-maximum-spread-raw", type=int, default=4)
    parser.add_argument("--image-topic", default="/camera/wrist_a/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--frames", type=int, default=20)
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error(
            "exact confirmation is required; this tool moves the arm"
        )
    return arguments


def main() -> int:
    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    arm_names = tuple(
        name.replace("left_", f"{arguments.arm}_", 1) for name in ARM_JOINT_NAMES
    )

    start_rad = read_joint_state(arm_names)
    print(f"START_RAD={[round(v, 6) for v in start_rad]}")

    anchor_raw = capture_anchor_raw(
        arguments.workdir,
        arguments.capture_id,
        arguments.bridge_calibration,
        arguments.anchor_samples,
        arguments.anchor_timeout_s,
        arguments.anchor_maximum_spread_raw,
    )
    print(f"ANCHOR_RAW={anchor_raw}")

    plan_and_execute_leg(
        arguments.workdir,
        arguments.capture_id,
        arguments.calibration,
        start_rad,
        arguments.target_joints,
        anchor_raw,
    )

    capture_directory = arguments.workdir / arguments.capture_id
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "capture_wrist_eye_in_hand_sample.py"),
            "--capture-id",
            arguments.capture_id,
            "--output-directory",
            str(capture_directory),
            "--image-topic",
            arguments.image_topic,
            "--joint-topic",
            arguments.joint_topic,
            "--frames",
            str(arguments.frames),
        ],
        f"{arguments.capture_id}: wrist eye-in-hand sample capture",
    )

    print(f"\nWRIST_EYE_IN_HAND_POSE_CAPTURE_PASS id={arguments.capture_id} "
          f"output={capture_directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
