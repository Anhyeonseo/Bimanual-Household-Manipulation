#!/usr/bin/env python3
"""캔 파지 기하를 URDF FK 로만 측정한다. ROS·MoveIt·serial 을 쓰지 않는다.

`docs/PLAN_CAN_TO_BIN.md` 의 §2 수치가 주장이 아니라 재현 가능한 측정이 되도록
하는 도구다. 이 도구가 내는 값이 그 문서와 다르면 문서를 고치기 전에 보고한다.

측정하는 것:

1. 접근축이 수직에서 얼마나 기울어 있는가 — `vertical_from_above` 가 가능한가
2. wrist_roll 이 접근축과 TCP 에 각각 무엇을 하는가
3. d(finger_yaw)/d(wrist_roll) 이 1 인가
4. 캔 무방향 장축 yaw 마다 한계 안 wrist_roll 분기가 몇 개인가

회귀 기준점은 2026-08-16 session03 에서 **실제로 실행된** 오른팔 pick_grasp
자세다. 그 계획 파일이 기록한 finger yaw 와 이 도구의 FK 가 일치해야 한다.
일치하지 않으면 URDF 나 한계 파일이 그때와 다른 것이며 즉시 실패한다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from grasp_yaw_kinematics import (  # noqa: E402
    GraspYawKinematics,
    wrap_half_turn,
)

STATUS = "CAN_GRASP_GEOMETRY_PLAN_ONLY_PASS"
SCHEMA_VERSION = 1
DOWN = np.array([0.0, 0.0, -1.0])
WORKCELL_FRAME = "workcell_base_link"
ARM_JOINT_SHORT_NAMES = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
DUAL_URDF_ENVIRONMENT = "SO101_DUAL_URDF_PATH"
DEFAULT_DUAL_URDF = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
)

# 실행된 오른팔 pick_grasp 자세. 계획 파일이 기록한 finger yaw 를 함께 둔다.
REFERENCE_PLAN = (
    "artifacts/top_pick_place/2026-08-16/"
    "pen_interarm_continuous_session03/right_plan.json"
)
REFERENCE_SIDE = "right"
REFERENCE_JOINTS_RAD = (
    -0.29366012646773704,
    2.6135725928114755,
    1.2219831294243892,
    1.264178694179015,
    0.0,
)
REFERENCE_FINGER_YAW_RAD = -1.278421260830095
REFERENCE_FINGER_YAW_TOLERANCE_RAD = 1.0e-6

# 상단 카메라 보정 영역: origin (0.34, -0.28), span (0.18, 0.28).
CALIBRATED_X_M = (0.34, 0.40, 0.46, 0.52)
CALIBRATED_Y_M = (-0.28, -0.14, 0.00)
TABLE_GRASP_Z_M = 0.0053  # 실행된 펜 pick_grasp 의 workcell z
POSITION_TOLERANCE_M = 0.002

CAN_LENGTH_M = 0.13244
CAN_DIAMETER_M = 0.053


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/can_to_bin/can_grasp_geometry_plan_only.json",
    )
    parser.add_argument("--roll-samples", type=int, default=721)
    parser.add_argument("--yaw-bin-deg", type=float, default=1.0)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="기존 artifact 를 덮어쓴다",
    )
    return parser.parse_args()


def dual_urdf_path() -> Path:
    path = Path(os.environ.get(DUAL_URDF_ENVIRONMENT, DEFAULT_DUAL_URDF))
    if not path.is_file():
        raise RuntimeError(f"dual robot description does not exist: {path}")
    return path


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_kinematics(path: Path, side: str) -> GraspYawKinematics:
    if path.suffix == ".xacro":
        import xacro

        xml = xacro.process_file(str(path)).toxml()
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".urdf", delete=False
        )
        handle.write(xml)
        handle.flush()
        return GraspYawKinematics(Path(handle.name), prefix=f"{side}_")
    return GraspYawKinematics(path, prefix=f"{side}_")


def load_arm_bounds(side: str) -> tuple[np.ndarray, np.ndarray]:
    document = json.loads(OPERATIONAL_LIMITS.read_text(encoding="utf-8"))
    if (
        document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True
    ):
        raise RuntimeError("bimanual operational limits are not approved")
    arm = document["arms"][side]
    lower = np.array(
        [arm[name]["minimum_urad"] / 1.0e6 for name in ARM_JOINT_SHORT_NAMES]
    )
    upper = np.array(
        [arm[name]["maximum_urad"] / 1.0e6 for name in ARM_JOINT_SHORT_NAMES]
    )
    return lower, upper


def tilt_from_vertical_deg(kinematics, positions: dict[str, float]) -> float:
    approach = kinematics.gripper_rotation(positions) @ DOWN
    cosine = float(np.dot(approach, DOWN))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def measure_reference_pose(kinematics, side: str, lower, upper, samples: int):
    """실행된 자세에서 접근축·gain·TCP orbit·분기 수를 잰다."""
    names = kinematics.arm_joints
    joints = np.array(REFERENCE_JOINTS_RAD)
    positions = dict(zip(names, joints, strict=True))

    finger_yaw = float(kinematics.finger_yaw(positions))
    drift = abs(wrap_half_turn(finger_yaw - REFERENCE_FINGER_YAW_RAD))
    if drift > REFERENCE_FINGER_YAW_TOLERANCE_RAD:
        raise RuntimeError(
            "reference pose finger yaw does not reproduce the recorded plan: "
            f"model={finger_yaw:.9f} plan={REFERENCE_FINGER_YAW_RAD:.9f} "
            f"drift={drift:.3e} rad"
        )

    approach = kinematics.gripper_rotation(positions) @ DOWN
    tilt_deg = tilt_from_vertical_deg(kinematics, positions)

    roll_lower, roll_upper = float(lower[4]), float(upper[4])
    step = 1.0e-5
    forward = dict(positions)
    forward[names[4]] = joints[4] + step
    backward = dict(positions)
    backward[names[4]] = joints[4] - step
    gain = wrap_half_turn(
        kinematics.finger_yaw(forward) - kinematics.finger_yaw(backward)
    ) / (2.0 * step)

    reference_tcp = kinematics.tcp_position(positions)
    orbit_mm = 0.0
    approach_deviation_deg = 0.0
    yaw_minimum = math.inf
    yaw_maximum = -math.inf
    for index in range(samples):
        roll = roll_lower + (roll_upper - roll_lower) * index / (samples - 1)
        probe = dict(positions)
        probe[names[4]] = roll
        orbit_mm = max(
            orbit_mm,
            float(np.linalg.norm(kinematics.tcp_position(probe) - reference_tcp))
            * 1000.0,
        )
        approach_probe = kinematics.gripper_rotation(probe) @ DOWN
        # 0 근처에서 acos 는 조건수가 나쁘다. 단위벡터 차이의 노름으로 잰다.
        chord = float(np.linalg.norm(approach_probe - approach))
        deviation_deg = math.degrees(2.0 * math.asin(min(1.0, chord / 2.0)))
        approach_deviation_deg = max(approach_deviation_deg, deviation_deg)
        if deviation_deg > 1.0e-3:
            raise RuntimeError(
                "wrist roll changed the approach axis by "
                f"{deviation_deg:.6f} deg"
            )
        yaw = math.degrees(wrap_half_turn(kinematics.finger_yaw(probe)))
        yaw_minimum = min(yaw_minimum, yaw)
        yaw_maximum = max(yaw_maximum, yaw)

    return {
        "side": side,
        "source_plan": REFERENCE_PLAN,
        "joint_positions_rad": [float(value) for value in joints],
        "recorded_plan_finger_yaw_rad": REFERENCE_FINGER_YAW_RAD,
        "model_finger_yaw_rad": finger_yaw,
        "finger_yaw_drift_rad": drift,
        "approach_axis_base": [float(value) for value in approach],
        "approach_tilt_from_vertical_deg": tilt_deg,
        "wrist_roll_limits_deg": [
            math.degrees(roll_lower),
            math.degrees(roll_upper),
        ],
        "wrist_roll_span_deg": math.degrees(roll_upper - roll_lower),
        "finger_yaw_per_wrist_roll_gain": float(gain),
        "wrist_roll_changes_approach_axis_deg": approach_deviation_deg,
        "wrist_roll_tcp_orbit_max_mm": orbit_mm,
        "finger_yaw_reachable_deg": [yaw_minimum, yaw_maximum],
    }


def measure_roll_branches(kinematics, positions, lower, upper, bin_deg, samples):
    """캔 yaw 구간마다 한계 안 분기 수와 필요한 회전량을 센다."""
    roll_lower, roll_upper = float(lower[4]), float(upper[4])
    bins = int(round(180.0 / bin_deg))
    counts = {"zero": 0, "one": 0, "two_or_more": 0}
    travels_deg: list[float] = []
    unreachable_yaw_deg: list[float] = []
    two_branch_yaw_deg: list[float] = []

    for index in range(bins):
        can_yaw = math.radians(-90.0 + index * bin_deg)
        target = wrap_half_turn(can_yaw + math.pi / 2.0)
        solution = kinematics.solve_wrist_roll_branches(
            positions, target, roll_lower, roll_upper, samples=samples
        )
        count = int(solution["branch_count"])
        if count == 0:
            counts["zero"] += 1
            unreachable_yaw_deg.append(math.degrees(can_yaw))
            continue
        counts["one" if count == 1 else "two_or_more"] += 1
        if count >= 2:
            two_branch_yaw_deg.append(math.degrees(can_yaw))
        selected = solution["selected"]
        travels_deg.append(
            abs(math.degrees(float(selected["rotation_from_current_rad"])))
        )

    return {
        "can_yaw_bin_deg": bin_deg,
        "can_yaw_bins": bins,
        "branch_count_histogram": counts,
        "reachable_bins": bins - counts["zero"],
        "unreachable_can_yaw_deg": unreachable_yaw_deg,
        "two_branch_can_yaw_deg": two_branch_yaw_deg,
        "selected_rotation_deg": {
            "maximum": max(travels_deg) if travels_deg else None,
            "mean": (sum(travels_deg) / len(travels_deg)) if travels_deg else None,
        },
    }


def measure_workspace_tilt(kinematics, side, lower, upper):
    """보정 영역 각 지점의 도달성과 접근축 기울기를 잰다."""
    from scipy.optimize import least_squares

    names = kinematics.arm_joints
    rows = []
    for x in CALIBRATED_X_M:
        for y in CALIBRATED_Y_M:
            target = kinematics.point_in_base_frame(
                np.array([x, y, TABLE_GRASP_Z_M]), root_link=WORKCELL_FRAME
            )

            def residual(values):
                probe = dict(zip(names, list(values) + [0.0], strict=True))
                return (kinematics.tcp_position(probe) - target) * 1000.0

            best = None
            for seed in (
                np.array([0.0, 2.6, 1.2, 1.3]),
                np.array([-0.3, 2.2, 1.0, 1.6]),
                np.array([0.3, 2.9, 1.5, 0.9]),
            ):
                clipped = np.clip(seed, lower[:4] + 1.0e-6, upper[:4] - 1.0e-6)
                result = least_squares(
                    residual,
                    clipped,
                    bounds=(lower[:4], upper[:4]),
                    xtol=1.0e-12,
                    ftol=1.0e-12,
                )
                probe = dict(zip(names, list(result.x) + [0.0], strict=True))
                error = float(
                    np.linalg.norm(kinematics.tcp_position(probe) - target)
                )
                if best is None or error < best[0]:
                    best = (error, tilt_from_vertical_deg(kinematics, probe))
            reachable = best[0] <= POSITION_TOLERANCE_M
            rows.append(
                {
                    "workcell_xy_m": [x, y],
                    "reachable": reachable,
                    "position_error_m": best[0],
                    "approach_tilt_from_vertical_deg": best[1] if reachable else None,
                }
            )
    tilts = [
        row["approach_tilt_from_vertical_deg"] for row in rows if row["reachable"]
    ]
    return {
        "side": side,
        "grasp_z_m": TABLE_GRASP_Z_M,
        "position_tolerance_m": POSITION_TOLERANCE_M,
        "samples": rows,
        "reachable_count": len(tilts),
        "sample_count": len(rows),
        "approach_tilt_deg": {
            "minimum": min(tilts) if tilts else None,
            "maximum": max(tilts) if tilts else None,
        },
    }


def jaw_width_for_crossing_error(theta_rad: float) -> float:
    """닫힘선이 캔 장축 수직에서 theta 만큼 벗어났을 때 필요한 조 개방 폭 [m]."""
    return CAN_DIAMETER_M * abs(math.cos(theta_rad)) + CAN_LENGTH_M * abs(
        math.sin(theta_rad)
    )


def main() -> int:
    args = parse_args()
    if args.output.exists() and not args.overwrite:
        raise SystemExit(f"refusing to overwrite existing artifact: {args.output}")

    urdf = dual_urdf_path()
    kinematics = load_kinematics(urdf, REFERENCE_SIDE)
    lower, upper = load_arm_bounds(REFERENCE_SIDE)

    reference = measure_reference_pose(
        kinematics, REFERENCE_SIDE, lower, upper, args.roll_samples
    )
    positions = dict(
        zip(kinematics.arm_joints, REFERENCE_JOINTS_RAD, strict=True)
    )
    branches = measure_roll_branches(
        kinematics, positions, lower, upper, args.yaw_bin_deg, args.roll_samples
    )

    workspace = []
    for side in ("left", "right"):
        side_kinematics = load_kinematics(urdf, side)
        side_lower, side_upper = load_arm_bounds(side)
        workspace.append(
            measure_workspace_tilt(
                side_kinematics, side, side_lower, side_upper
            )
        )

    document = {
        "schema_version": SCHEMA_VERSION,
        "status": STATUS,
        "motion_authorized": False,
        "motion_commands": 0,
        "execution_api_used": False,
        "collision_checked": False,
        "robot_description": {
            "environment": DUAL_URDF_ENVIRONMENT,
            "path": str(urdf),
            "sha256": sha256_file(urdf),
        },
        "operational_limits": {
            "path": str(OPERATIONAL_LIMITS),
            "sha256": sha256_file(OPERATIONAL_LIMITS),
        },
        "reference_pose": reference,
        "wrist_roll_branches": branches,
        "workspace_approach_tilt": workspace,
        "can_model": {
            "length_m": CAN_LENGTH_M,
            "diameter_m": CAN_DIAMETER_M,
            "source": "M1_operator_measurement_2026-08-15",
            "required_jaw_width_m": {
                f"crossing_error_{degrees:g}_deg": jaw_width_for_crossing_error(
                    math.radians(degrees)
                )
                for degrees in (0.0, 5.0, 10.0, 20.0, 35.9)
            },
            "jaw_width_measured": False,
            "note": "jaw gap in millimetres has never been measured; M3 gate",
        },
        "can_axis_model": {
            "definition": "horizontal unit vector perpendicular to the finger axis",
            "rejects": "gripper jaw hinge axis is not the can long axis",
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    digest = sha256_file(args.output)

    print(f"{STATUS} motion_commands=0 execution_api_used=false")
    print(f"artifact={args.output}")
    print(f"sha256={digest}")
    print(
        "reference approach tilt from vertical="
        f"{reference['approach_tilt_from_vertical_deg']:.2f} deg"
    )
    print(
        "d(finger_yaw)/d(wrist_roll)="
        f"{reference['finger_yaw_per_wrist_roll_gain']:.4f}"
    )
    print(
        "wrist roll TCP orbit max="
        f"{reference['wrist_roll_tcp_orbit_max_mm']:.2f} mm"
    )
    histogram = branches["branch_count_histogram"]
    print(
        f"can yaw bins reachable={branches['reachable_bins']}"
        f"/{branches['can_yaw_bins']} "
        f"two_branch={histogram['two_or_more']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
