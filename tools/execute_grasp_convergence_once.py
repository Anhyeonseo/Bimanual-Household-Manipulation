#!/usr/bin/env python3
"""명령한 자세에 도달할 때까지 경계된 보정을 반복하는 실행기. 1회 승인.

**이 도구는 여러 번 움직인다.** 그것이 목적이다. 지금까지의 one-shot 도구는
"한 번 보내고 결과를 본다" 였는데, 수렴은 정의상 반복이다. 대신 반복이
무한하지 않다는 것을 구조로 보장한다.

  - 최대 반복 횟수가 정해져 있고 (`grasp_convergence.MAXIMUM_ITERATIONS`)
  - 한 회 보정이 이동해도 되는 거리에 상한이 있고 (`MAXIMUM_CORRECTION_M`)
  - 넘겨명령은 크기 상한이 있으며 단 1회만 쓰이고
  - 수렴하지 못하면 잔차와 함께 멈춘다. 조용히 계속하지 않는다

그래서 운영자 승인은 **수렴 실행 1회**에 대한 것이고, 그 안의 각 leg 는
위 경계가 보증한다.

**모든 물리 이동은 Motion-14 파이프라인을 그대로 지난다.** 이 도구는
조율만 한다. 새 이동 경로를 만들지 않는다.

    ros_moveit_plan_pregrasp_segments.py   관절 공간 segment (MoveIt 계획)
    plan_buffered_segment_leg.py           20 ms buffered leg + 계약 검증
    execute_buffered_segment_leg_once.py   Action 1회 전송, 재시도 없음

**MoveIt 의 충돌 검사에 기대지 않는다.** 2026-08-06 확인 결과 planning scene
에 충돌 객체도 octomap 도 없다. 즉 자기충돌 외에는 아무것도 막지 못한다.
테이블 근처에서 수렴할 때는 `--minimum-tcp-z-m` 으로 바닥을 명시할 것.

**시리얼을 열지 않는다.** bridge 가 포트를 소유하고 있고, 그것을 뺏으려고
bridge 를 죽이면 torque 가 풀려 팔이 처진다. 2026-08-06 에 그렇게 해서 관절이
계약을 벗어났고 복구에 세 번의 수동 주기가 들었다. 전부 ROS 로 간다.

**잔차는 FK 로 mm 로 잰다.** 관절 오차를 `raw x 반경` 으로 환산하면 어느
관절이 틀렸는지에 따라 답이 달라진다. `grasp_yaw_kinematics` 의 FK 는 MoveIt
`/compute_fk` 와 `1 µm` 이내로 일치함이 회귀로 고정돼 있다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

CONFIRMATION = "EXECUTE_C2_GRASP_CONVERGENCE_ONCE"
STATUS = "GRASP_CONVERGENCE_ONCE"
SEGMENT_LEG_CONFIRMATION = "EXECUTE_MOTION14_FRESH_SEGMENT_LEG_ONCE"
ARM_URDF_XACRO = ROOT / "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"
JOINT_STATE_TIMEOUT_S = 10.0
TERMINAL_DIAGNOSTICS_PATTERN = re.compile(r"^TERMINAL_DIAGNOSTICS=(?P<text>.*)$")
POST_SETTLE_MAX_PATTERN = re.compile(r"post_settle_max_error_raw=(\d+)")


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def run(command: list[str], label: str) -> str:
    """하위 도구를 부르고 출력을 그대로 보여준다. 실패는 즉시 멈춘다."""
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


def build_urdf(workdir: Path) -> Path:
    path = workdir / "so101_left.urdf"
    if not path.exists():
        expanded = subprocess.run(
            ["xacro", str(ARM_URDF_XACRO)],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        path.write_text(expanded, encoding="utf-8")
    return path


def radians_to_raw(calibration, positions_rad: tuple[float, ...]) -> tuple[int, ...]:
    """`buffered_action_execution._verify_post_settle` 와 같은 환산식."""
    return tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * math.pi)
        )
        for joint, position in zip(calibration.joints, positions_rad, strict=True)
    )


def parse_measurement(output: str) -> tuple[tuple[int, ...], int] | None:
    """실행기 출력에서 관절별 post-settle 실측을 꺼낸다."""
    from execute_buffered_action_plan_once import parse_post_settle_vectors

    for line in output.splitlines():
        match = TERMINAL_DIAGNOSTICS_PATTERN.match(line.strip())
        if match is None:
            continue
        vectors = parse_post_settle_vectors(match.group("text"))
        if vectors is None:
            return None
        maximum = POST_SETTLE_MAX_PATTERN.search(output)
        return vectors["measured"], (
            int(maximum.group(1)) if maximum else max(vectors["error"])
        )
    return None


def read_joint_state(arm_names: tuple[str, ...]) -> tuple[float, ...]:
    """`/joint_states` 를 한 번 읽는다. 시리얼을 열지 않는다."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("grasp_convergence_joint_state_reader")
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


def plan_and_execute_leg(
    workdir: Path,
    iteration: int,
    calibration_path: Path,
    start_rad: tuple[float, ...],
    target_rad: tuple[float, ...],
    anchor_raw: tuple[int, ...],
) -> str:
    """Motion-14 파이프라인을 그대로 한 번 지난다."""
    segments = workdir / f"iter{iteration}_segments.json"
    run(
        [
            sys.executable,
            str(ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"),
            "--plan-only",
            "--calibration",
            str(calibration_path),
            "--start",
            ",".join(f"{value:.12f}" for value in start_rad),
            "--target-joints",
            ",".join(f"{value:.12f}" for value in target_rad),
            "--output",
            str(segments),
        ],
        f"iteration {iteration}: collision-checked joint segments",
    )

    leg = workdir / f"iter{iteration}_leg.json"
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
        f"iteration {iteration}: buffered leg",
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
        f"iteration {iteration}: execute (physical motion)",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-plan", type=Path, required=True)
    parser.add_argument("--target-name", default="grasp")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument("--task-tolerance-m", type=float, default=None)
    parser.add_argument("--max-iterations", type=int, default=None)
    parser.add_argument(
        "--minimum-tcp-z-m",
        type=float,
        default=None,
        help=(
            "넘겨명령이 이 높이 아래로 내려가면 거부한다. 테이블 근처에서 "
            "수렴할 때 넣을 것. planning scene 이 비어 있어 하위 계획기는 "
            "테이블을 막지 못한다."
        ),
    )
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
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error(
            "exact convergence confirmation is required; this tool moves the "
            "arm more than once"
        )
    return arguments


def main() -> int:
    import grasp_convergence as GC
    from grasp_yaw_kinematics import GraspYawKinematics, arm_joint_names
    from ros_moveit_plan_pregrasp_segments import load_target
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)

    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")
    kinematics = GraspYawKinematics(
        build_urdf(arguments.workdir), prefix=f"{arguments.arm}_"
    )

    def forward_kinematics(named: dict[str, float]) -> tuple[float, float, float]:
        return tuple(float(value) for value in kinematics.tcp_position(named))

    nominal = load_target(arguments.source_plan, arguments.target_name)
    overrides = {}
    if arguments.task_tolerance_m is not None:
        overrides["task_tolerance_m"] = arguments.task_tolerance_m
    if arguments.max_iterations is not None:
        overrides["maximum_iterations"] = arguments.max_iterations
    if arguments.minimum_tcp_z_m is not None:
        overrides["minimum_tcp_z_m"] = arguments.minimum_tcp_z_m
    policy = GC.ConvergencePolicy(arm=arguments.arm, **overrides)
    limits = {
        name: calibration.ros_radian_limits[name] for name in arm_names
    }

    print(f"ARM={policy.arm}")
    print(f"SOURCE_PLAN={arguments.source_plan}")
    print(f"SOURCE_PLAN_SHA256={sha256_file(arguments.source_plan)}")
    print(f"NOMINAL_RAD={[round(v, 9) for v in nominal]}")
    print(f"NOMINAL_TCP_M={[round(v, 6) for v in forward_kinematics(dict(zip(arm_names, nominal)))]}")
    print(f"TASK_TOLERANCE_MM={policy.task_tolerance_m * 1000.0:.3f}")
    print(f"MAXIMUM_ITERATIONS={policy.maximum_iterations}")
    print(f"MAXIMUM_CORRECTION_MM={policy.maximum_correction_m * 1000.0:.3f}")
    print(f"MAXIMUM_OVERSHOOT_MM={policy.maximum_overshoot_m * 1000.0:.3f}")
    print(
        "MINIMUM_TCP_Z_MM="
        + ("none" if policy.minimum_tcp_z_m is None
           else f"{policy.minimum_tcp_z_m * 1000.0:.3f}")
    )

    state = GC.begin(policy, arm_names, nominal)
    decisions: list = []
    legs: list[dict] = []
    all_names = (*arm_names, f"{arguments.arm}_gripper_joint")
    observed = read_joint_state(all_names)
    measured, gripper_rad = observed[:5], observed[5]

    while True:
        state, decision = GC.evaluate(
            state, measured, forward_kinematics, limits
        )
        decisions.append(decision)
        print(
            f"\n===== iteration {decision.iteration}: {decision.action.upper()} "
            f"residual={decision.error_mm():.3f} mm ====="
        )
        print(f"REASON={decision.reason}")
        print(
            "RESIDUAL_VECTOR_MM="
            + ",".join(f"{v * 1000.0:.3f}" for v in decision.error_vector_m)
        )
        if not decision.requires_motion:
            break

        anchor_raw = radians_to_raw(calibration, (*measured, gripper_rad))
        output = plan_and_execute_leg(
            arguments.workdir,
            decision.iteration,
            arguments.calibration,
            measured,
            decision.next_commanded_rad,
            anchor_raw,
        )
        parsed = parse_measurement(output)
        if parsed is None:
            raise RuntimeError(
                "the executor terminal carried no per-joint measurement; "
                "the bridge is older than the C1 change"
            )
        measured_raw, settle_max = parsed
        observed = calibration.raw_feedback_to_radians(measured_raw)
        measured, gripper_rad = tuple(observed[:5]), observed[5]
        legs.append(
            {
                "iteration": decision.iteration,
                "action": decision.action,
                "commanded_rad": list(decision.next_commanded_rad),
                "measured_raw": list(measured_raw),
                "post_settle_max_error_raw": settle_max,
            }
        )

    summary = GC.summarize(policy, tuple(decisions))
    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'PASS' if summary['converged'] else 'FAIL'}",
        "arm": policy.arm,
        "source_plan": str(arguments.source_plan),
        "source_plan_sha256": sha256_file(arguments.source_plan),
        "target_name": arguments.target_name,
        "nominal_rad": list(nominal),
        "nominal_tcp_m": list(
            forward_kinematics(dict(zip(arm_names, nominal)))
        ),
        "convergence": summary,
        "legs": legs,
        "physical_motion_count": len(legs),
        "automatic_retry_count": 0,
        "serial_port_opened": False,
        "motion_authorized": True,
        "operator_confirmation": CONFIRMATION,
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    print()
    print(f"CONVERGED={int(summary['converged'])}")
    print(f"ITERATIONS={summary['iterations']}")
    print(f"PHYSICAL_MOTION_COUNT={len(legs)}")
    print(f"RESIDUAL_MM_BY_ITERATION={summary['residual_mm_by_iteration']}")
    print(f"FINAL_RESIDUAL_MM={summary['final_residual_mm']}")
    print(f"OVERSHOOT_USED={int(summary['overshoot_used'])}")
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(f"{document['status']}")
    return 0 if summary["converged"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
