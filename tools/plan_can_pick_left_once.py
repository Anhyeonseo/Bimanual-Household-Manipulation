#!/usr/bin/env python3
"""왼팔 캔 파지 계획을 한 번 만든다. plan-only 다.

**범위는 pick 까지다.** place 와 핸드오버는 없다. 왼팔은 제안된 쓰레기통
지점에 88.27 mm 부족해 닿지 않으므로 그 부분은 별도 단계로 남긴다.

**펜 계획기와 다른 점.**

펜은 `wrist_roll` 을 q0 인 0 에 고정하고 나머지 4축으로 TCP xyz 만 맞췄다.
물체 yaw 는 진단값이었고 실행 조건이 아니었다. 캔은 그럴 수 없다. 실행된 펜
계획이 기록한 `crossing_residual_rad = 0.627` = 35.9 도를 캔에 그대로 두면
조가 벌려야 하는 폭이 53 mm 가 아니라 121 mm 가 된다.

그래서 이 계획기는 세 가지를 더 한다.

1. **roll 을 푼다.** `finger_yaw(roll) = target` 을 수치로 푼다. 해석식
   `roll += Δyaw` 는 gain 1 을 가정하는데, 이 팔은 접근축이 수직에서
   21~73 도 기울어 있어 실제 gain 이 0.43~0.61 이다.
2. **한계 안 분기 중 최단 회전을 고른다.** roll 가동 범위가 180 도보다 넓어
   어떤 캔 방향에서는 해가 두 개다. 한계 검사가 분기 선택보다 먼저다 —
   대부분의 방향에서는 수학적으로 더 가까운 분기가 한계 밖이다.
3. **하강 중 roll 을 고정한다.** TCP 가 roll 축에서 7.9 mm 편심돼 있어
   roll 을 돌리면 TCP 가 최대 13.3 mm 움직인다. grasp 에서 푼 roll 을
   pregrasp 와 lift 에 그대로 물려 하강을 연직으로 유지한다.

이 도구는 hardware 를 만지지 않는다. `motion_commands=0`,
`execution_api_used=false` 를 산출물에 기록한다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import plan_top_camera_pick_place_once as pen  # noqa: E402
from can_pick_application import (  # noqa: E402
    CanPickContractError,
    _solve_position_with_fixed_roll,
    load_calibrated_region,
    load_can_pick_policy,
    required_jaw_width_mm,
    solve_can_pick_endpoint,
)

STATUS = "CAN_PICK_LEFT_PLAN_ONLY_PASS"
SCHEMA_VERSION = 1
SIDE = "left"
Q0 = (0.0,) * 5
DOWN = np.array([0.0, 0.0, -1.0])
CAN_TARGET_TOPIC = "/perception/top/can_obb/object_pose_board"
DEFAULT_CONTRACT = ROOT / "config/can_pick_contract.candidate.json"
DEFAULT_SHADOW_CONFIG = (
    ROOT / "ros2_ws/src/so101_top_perception/config/top_shadow_target.yaml"
)

# 펜이 실기로 확정한 왼팔 화면축 보정. 같은 카메라·같은 보정판이므로 캔에도
# 같은 계통 오차가 실린다. 값을 새로 만들지 않고 그대로 상속한다.
LEFT_SCREEN_X_CORRECTION_M = pen.LEFT_SCREEN_X_CORRECTION_M
LEFT_SCREEN_X_CORRECTION_REASON = pen.LEFT_SCREEN_X_CORRECTION_REASON

# 접근 높이. 펜 값을 그대로 쓰지 않는다 — 캔은 지름 53 mm 로 서 있어
# pregrasp 를 캔 위로 충분히 띄워야 조가 캔을 치지 않는다.
PICK_PREGRASP_OFFSET_M = 0.120
PICK_LIFT_OFFSET_M = 0.060


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan-only",
        action="store_true",
        required=True,
        help="이 도구는 계획만 만든다. 명시적으로 표시해야 실행된다.",
    )
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument(
        "--shadow-config", type=Path, default=DEFAULT_SHADOW_CONFIG
    )
    parser.add_argument("--target-samples", type=int, default=5)
    parser.add_argument("--timeout-s", type=float, default=25.0)
    parser.add_argument(
        "--grasp-offset-m",
        type=float,
        default=-0.001,
        help="검출된 캔 z 에 더할 파지 높이 보정. 펜의 값을 상속하지 않는다.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.target_samples <= 40:
        parser.error("target samples must be within 1..40")
    if not -0.05 <= args.grasp_offset_m <= 0.05:
        parser.error("grasp offset must be within +-50 mm")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing plan: {args.output}")
    return args


def wait_can_target(node, messages, config, count: int, timeout_s: float):
    """캔 target 을 여러 프레임에 걸쳐 잠근다.

    펜의 `wait_target` 은 픽셀 x 로 팔을 고른다. 이 도구는 왼팔 전용이므로
    그 분기를 쓰지 않되, **픽셀 routing 이 무엇이라고 말했는지는 기록한다.**
    실제 제약은 픽셀 규칙이 아니라 도달성이며 그건 뒤에서 따로 검사한다.
    """
    deadline = time.monotonic() + timeout_s
    samples = []
    center_x_samples: list[float] = []
    center_y_samples: list[float] = []
    yaw_samples: list[float] = []
    widths: list[int] = []
    heights: list[int] = []
    stamps: set[tuple[int, int]] = set()
    rejection = "no observation"
    while len(samples) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not messages:
            continue
        message = messages[-1]
        stamp = (
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
        )
        if stamp in stamps:
            continue
        stamps.add(stamp)
        try:
            samples.append(pen.target_sample(node, config, message, SIDE))
            center_x_samples.append(float(message.center_x_px))
            center_y_samples.append(float(message.center_y_px))
            yaw_samples.append(float(message.yaw_rad))
            widths.append(int(message.image_width_px))
            heights.append(int(message.image_height_px))
            print(
                f"CAN_PICK_TARGET_SAMPLE count={len(samples)}/{count} "
                f"pixel_x={float(message.center_x_px):.1f}/"
                f"{int(message.image_width_px)} "
                f"yaw_deg={math.degrees(float(message.yaw_rad)):+.2f}"
            )
        except Exception as error:  # noqa: BLE001 - 거부 사유를 그대로 보고한다
            rejection = f"{type(error).__name__}: {error}"
    if len(samples) < count:
        raise RuntimeError(
            f"only {len(samples)}/{count} valid can samples; {rejection}"
        )
    if len(set(widths)) != 1 or len(set(heights)) != 1:
        raise RuntimeError("camera image size changed during the target lock")
    locked = pen.lock_target(samples)
    return (
        locked,
        float(pen.median(center_x_samples)),
        float(pen.median(center_y_samples)),
        widths[0],
        heights[0],
        yaw_samples,
    )


def can_yaw_spread_rad(yaw_samples) -> float:
    """무방향 장축이므로 단순 max-min 이 아니라 modulo pi 로 퍼짐을 잰다."""
    from lying_can_upright_application import undirected_axis_error

    reference = yaw_samples[0]
    return max(
        undirected_axis_error(value, reference) for value in yaw_samples
    )


def can_steps_from_phases(phases, policy) -> list[dict]:
    """캔 그리퍼 값으로 단계를 조립한다.

    펜의 `steps_from_phases` 를 쓰면 안 된다. 그 함수는 펜의 `GRIPPER_OPEN_RAD`
    와 `GRIPPER_CLOSE_RAD` 를 **자기가 직접 끼워 넣는다.** 펜의 개방값
    raw 2048 은 개방 범위의 거의 닫힌 끝이라 53 mm 캔에는 조가 아예 안 벌어진다.

    순서는 펜과 같다. 개방은 접근 전에 끝내고, 파지는 grasp 에 도착한 뒤
    lift 로 떠나기 전에 한다.
    """
    steps: list[dict] = [
        {
            "kind": "gripper",
            "phase": "pick_open",
            "target_position_rad": policy.jaw.open_command_rad,
            "target_gap_mm": policy.jaw.open_gap_mm,
        }
    ]
    for phase in phases:
        if phase["name"] == "pick_grasp_to_lift":
            steps.append(
                {
                    "kind": "gripper",
                    "phase": "pick_close",
                    "target_position_rad": policy.jaw.grasp_command_rad,
                    "target_gap_mm": policy.jaw.grasp_gap_mm,
                }
            )
        for segment in phase["segments"]:
            steps.append(
                {
                    "kind": "arm",
                    "phase": phase["name"],
                    "target_positions_rad": segment["target_positions_rad"],
                    "maximum_joint_delta_rad": segment[
                        "maximum_joint_delta_rad"
                    ],
                }
            )
    kinds = [step["kind"] for step in steps]
    if kinds.count("gripper") != 2:
        raise CanPickContractError(
            f"can pick needs exactly one open and one close step: {kinds}"
        )
    for index, step in enumerate(steps, start=1):
        step["index"] = index
    return steps


def endpoint_document(
    name: str,
    target_xyz,
    joints,
    kinematics,
    joint_names,
    lower,
    upper,
    achieved_tcp,
    residual_m: float,
    extra: dict | None = None,
) -> dict:
    positions = dict(zip(joint_names, joints, strict=True))
    approach = kinematics.gripper_rotation(positions) @ DOWN
    tilt = math.acos(max(-1.0, min(1.0, float(np.dot(approach, DOWN)))))
    document = {
        "name": name,
        "target_m": [float(value) for value in target_xyz],
        "final_joint_positions_rad": [float(value) for value in joints],
        "achieved_tcp_m": [float(value) for value in achieved_tcp],
        "plan_residual_norm_m": float(residual_m),
        "wrist_roll_rad": float(joints[4]),
        "approach_axis_base": [float(value) for value in approach],
        "approach_tilt_from_vertical_rad": tilt,
        "approach_tilt_from_vertical_deg": math.degrees(tilt),
        "joint_limit_margin_rad": float(
            min(
                np.min(np.asarray(joints) - lower),
                np.min(upper - np.asarray(joints)),
            )
        ),
        "achieved_finger_yaw_rad": float(kinematics.finger_yaw(positions)),
    }
    if extra:
        document.update(extra)
    return document


def main() -> int:
    args = parse_args()
    policy, contract_provenance = load_can_pick_policy(args.contract)

    # 실측 안 된 그리퍼로는 계획을 만들지 않는다. 여기서 걸려야 잘못된 개방
    # 명령이 실기까지 내려가지 않는다.
    needed_open_mm = policy.require_open_gap_covers_tolerance()

    region = load_calibrated_region(pen.TOP_HOMOGRAPHY_PATH)
    config = pen.load_shadow_config(args.shadow_config)
    if config.output_frame != "left_base_link":
        raise RuntimeError(
            "Top shadow transform must currently terminate at left_base_link"
        )

    rclpy.init()
    node = Node("can_pick_left_planner")
    messages: list = []
    node.create_subscription(
        pen.TopObjectPose, CAN_TARGET_TOPIC, messages.append, 10
    )
    plan_client = node.create_client(pen.GetMotionPlan, pen.PLAN_SERVICE)
    fk_client = node.create_client(pen.GetPositionFK, pen.FK_SERVICE)
    try:
        for name, client in (
            (pen.PLAN_SERVICE, plan_client),
            (pen.FK_SERVICE, fk_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        (
            locked,
            center_x_px,
            center_y_px,
            image_width_px,
            image_height_px,
            yaw_samples,
        ) = wait_can_target(
            node, messages, config, args.target_samples, args.timeout_s
        )
        observed_x, observed_y, z, yaw = (
            locked.x_m,
            locked.y_m,
            locked.z_m,
            locked.yaw_rad,
        )
        yaw_spread = can_yaw_spread_rad(yaw_samples)
        if yaw_spread > policy.crossing_tolerance_rad:
            raise RuntimeError(
                f"locked can yaw spread {math.degrees(yaw_spread):.2f} deg "
                "already exceeds the crossing tolerance "
                f"{math.degrees(policy.crossing_tolerance_rad):.2f} deg; "
                "the detection is not stable enough to grasp on"
            )

        unit_x, unit_y = pen.screen_positive_x_unit_workcell(
            pen.TOP_HOMOGRAPHY_PATH, center_x_px, center_y_px
        )
        delta_x = unit_x * LEFT_SCREEN_X_CORRECTION_M
        delta_y = unit_y * LEFT_SCREEN_X_CORRECTION_M
        x = observed_x + delta_x
        y = observed_y + delta_y
        region.require_inside(x, y)
        print(
            "CAN_PICK_LEFT_LATERAL_CORRECTION_PASS "
            f"screen_right_mm={LEFT_SCREEN_X_CORRECTION_M * 1000.0:.3f} "
            f"corrected_target=({x:.6f},{y:.6f}) "
            f"can_yaw_deg={math.degrees(yaw):+.2f} "
            f"yaw_spread_deg={math.degrees(yaw_spread):.3f}"
        )

        kinematics = pen.load_yaw_kinematics(SIDE)
        lower, upper = pen.load_arm_joint_bounds(SIDE)
        _, joint_names, _ = pen.arm_contract(SIDE)

        # 1) grasp 에서 roll 을 푼다. 최단 한계 안 분기를 고른다.
        grasp_xyz = (x, y, z + args.grasp_offset_m)
        grasp = solve_can_pick_endpoint(
            kinematics,
            joint_names,
            grasp_xyz,
            yaw,
            Q0,
            lower,
            upper,
            policy,
        )
        locked_roll = float(grasp["joint_positions_rad"][4])
        print(
            "CAN_PICK_LEFT_ROLL_BRANCH_PASS "
            f"branch={grasp['wrist_roll_branch_index']}"
            f"/{grasp['wrist_roll_branch_count']} "
            f"roll_deg={math.degrees(locked_roll):+.3f} "
            f"rotation_from_q0_deg="
            f"{math.degrees(grasp['wrist_roll_rotation_from_current_rad']):+.3f} "
            f"crossing_error_deg="
            f"{math.degrees(grasp['crossing_error_rad']):.3f} "
            f"tilt_deg="
            f"{math.degrees(grasp['approach_tilt_from_vertical_rad']):.2f}"
        )

        # 2) pregrasp 와 lift 는 같은 roll 을 고정한 채 위치만 다시 푼다.
        #    roll 을 바꾸면 TCP 가 최대 13.3 mm 움직이므로 하강이 연직이
        #    아니게 된다.
        endpoints: dict[str, dict] = {}
        seed_four = np.asarray(grasp["joint_positions_rad"][:4], dtype=float)
        specs = (
            ("pick_pregrasp", (x, y, z + PICK_PREGRASP_OFFSET_M)),
            ("pick_grasp", grasp_xyz),
            ("pick_lift", (x, y, z + PICK_LIFT_OFFSET_M)),
        )
        for name, target_xyz in specs:
            if name == "pick_grasp":
                joints = tuple(grasp["joint_positions_rad"])
                extra = {
                    "can_axis_yaw_rad": grasp["can_axis_yaw_rad"],
                    "finger_target_yaw_rad": grasp["finger_target_yaw_rad"],
                    "crossing_error_rad": grasp["crossing_error_rad"],
                    "crossing_tolerance_rad": grasp["crossing_tolerance_rad"],
                    "wrist_roll_policy": grasp["wrist_roll_policy"],
                    "wrist_roll_branch_index": grasp["wrist_roll_branch_index"],
                    "wrist_roll_branch_count": grasp["wrist_roll_branch_count"],
                    "wrist_roll_candidates_rad": grasp[
                        "wrist_roll_candidates_rad"
                    ],
                    "wrist_roll_rotation_from_q0_rad": grasp[
                        "wrist_roll_rotation_from_current_rad"
                    ],
                    "rejected_branches": grasp["rejected_branches"],
                }
            else:
                target_base = kinematics.point_in_base_frame(
                    np.asarray(target_xyz, dtype=float),
                    root_link=pen.WORKCELL_FRAME,
                )
                values, error_m = _solve_position_with_fixed_roll(
                    kinematics,
                    joint_names,
                    target_base,
                    locked_roll,
                    seed_four,
                    lower,
                    upper,
                )
                if error_m > policy.position_tolerance_m:
                    raise CanPickContractError(
                        f"{name} is not reachable with the locked wrist roll: "
                        f"residual {error_m * 1000.0:.3f} mm exceeds "
                        f"{policy.position_tolerance_m * 1000.0:.3f} mm"
                    )
                joints = tuple(list(values) + [locked_roll])
                extra = {
                    "wrist_roll_policy": "locked_to_pick_grasp_solution",
                    "wrist_roll_locked": True,
                }

            achieved = pen.measure_tcp(
                fk_client, node, SIDE, joint_names, joints
            )
            residual = math.dist(achieved, target_xyz)
            if residual > policy.position_tolerance_m:
                raise CanPickContractError(
                    f"{name} MoveIt FK disagrees with the solver: residual "
                    f"{residual * 1000.0:.3f} mm exceeds "
                    f"{policy.position_tolerance_m * 1000.0:.3f} mm"
                )
            endpoints[name] = endpoint_document(
                name,
                target_xyz,
                joints,
                kinematics,
                joint_names,
                lower,
                upper,
                achieved,
                residual,
                extra,
            )

        # 3) 하강이 실제로 연직인지, roll 이 정말 안 바뀌는지 확인한다.
        rolls = {
            name: endpoints[name]["wrist_roll_rad"] for name in endpoints
        }
        if max(rolls.values()) - min(rolls.values()) > 1.0e-9:
            raise CanPickContractError(f"wrist roll moved between endpoints: {rolls}")
        descent_lateral_m = math.dist(
            endpoints["pick_pregrasp"]["achieved_tcp_m"][:2],
            endpoints["pick_grasp"]["achieved_tcp_m"][:2],
        )
        if descent_lateral_m > policy.position_tolerance_m:
            raise CanPickContractError(
                f"descent is not vertical: {descent_lateral_m * 1000.0:.3f} mm "
                "of lateral travel between pregrasp and grasp"
            )

        # 4) MoveIt 으로 충돌 인지 phase 계획을 만든다.
        positions = {
            name: tuple(item["final_joint_positions_rad"])
            for name, item in endpoints.items()
        }
        phase_specs = (
            ("q0_to_pick_pregrasp", Q0, positions["pick_pregrasp"]),
            (
                "pick_pregrasp_to_grasp",
                positions["pick_pregrasp"],
                positions["pick_grasp"],
            ),
            ("pick_grasp_to_lift", positions["pick_grasp"], positions["pick_lift"]),
            ("pick_lift_to_q0", positions["pick_lift"], Q0),
        )
        phases = [
            pen.plan_phase(node, plan_client, SIDE, name, start, target)
            for name, start, target in phase_specs
        ]
        # 5) 캔 그리퍼 값으로 단계를 조립한다. 펜 조립기를 쓰지 않는다.
        ordered_steps = can_steps_from_phases(phases, policy)

        document = {
            "schema_version": SCHEMA_VERSION,
            "record_kind": "can_pick_left_plan_only",
            "status": STATUS,
            "generated_at_unix_s": time.time(),
            "execution_api_used": False,
            "motion_authorized": False,
            "motion_commands": 0,
            "automatic_execution_permitted": False,
            "scope": "left_arm_can_pick_only_no_place_no_handover",
            "selected_arm": SIDE,
            "planning_frame": pen.WORKCELL_FRAME,
            "joint_names": list(joint_names),
            "q0_rad": list(Q0),
            "contract": contract_provenance,
            "robot_description": {
                "path": str(pen.dual_urdf_path()),
                "sha256": pen.sha256_file(pen.dual_urdf_path()),
                "environment": pen.DUAL_URDF_ENVIRONMENT,
            },
            "operational_limits": {
                "path": str(pen.OPERATIONAL_LIMITS),
                "sha256": pen.sha256_file(pen.OPERATIONAL_LIMITS),
            },
            "calibrated_region": {
                "path": region.source_path,
                "sha256": region.source_sha256,
                "origin_xy_m": list(region.origin_xy_m),
                "span_xy_m": list(region.span_xy_m),
                "table_z_m": region.table_z_m,
            },
            "target_lock": {
                "topic": CAN_TARGET_TOPIC,
                "sample_count": locked.sample_count,
                "observed_xy_m": [observed_x, observed_y],
                "corrected_xy_m": [x, y],
                "z_m": z,
                "can_axis_yaw_rad": yaw,
                "can_axis_yaw_deg": math.degrees(yaw),
                "can_axis_yaw_spread_rad": yaw_spread,
                "yaw_semantics": "undirected_long_axis_modulo_pi",
                "center_x_px": center_x_px,
                "center_y_px": center_y_px,
                "image_width_px": image_width_px,
                "image_height_px": image_height_px,
                "pixel_routing_note": (
                    "routing rule recorded only; the binding constraint is "
                    "left-arm reachability, checked by the endpoint solver"
                ),
            },
            "lateral_adjustment": {
                "applied": True,
                "selected_arm": SIDE,
                "screen_axis": "positive_image_x",
                "command_correction_m": LEFT_SCREEN_X_CORRECTION_M,
                "direction_unit_workcell_xy": [unit_x, unit_y],
                "delta_workcell_xy_m": [delta_x, delta_y],
                "observed_target_xy_m": [observed_x, observed_y],
                "corrected_target_xy_m": [x, y],
                "reason": LEFT_SCREEN_X_CORRECTION_REASON,
                "homography": {
                    "path": str(pen.TOP_HOMOGRAPHY_PATH),
                    "sha256": pen.sha256_file(pen.TOP_HOMOGRAPHY_PATH),
                },
            },
            "pick_offsets_m": {
                "pregrasp": PICK_PREGRASP_OFFSET_M,
                "grasp": args.grasp_offset_m,
                "lift": PICK_LIFT_OFFSET_M,
                "reason": (
                    "pen offsets are not inherited; a 53 mm can needs more "
                    "pregrasp clearance than a 15 mm pen"
                ),
            },
            "gripper_contract": {
                "preopen_required": True,
                "open_phase": "before_approach",
                "open_gap_mm": policy.jaw.open_gap_mm,
                "open_command_rad": policy.jaw.open_command_rad,
                "grasp_gap_mm": policy.jaw.grasp_gap_mm,
                "grasp_command_rad": policy.jaw.grasp_command_rad,
                "contact_threshold_raw": policy.jaw.contact_threshold_raw,
                "release_tolerance_raw": policy.jaw.release_tolerance_raw,
                "provenance": policy.jaw.provenance,
                "minimum_open_gap_for_tolerance_mm": needed_open_mm,
                "required_jaw_width_at_achieved_error_mm": required_jaw_width_mm(
                    grasp["crossing_error_rad"],
                    policy.jaw.can_length_mm,
                    policy.jaw.can_diameter_mm,
                ),
            },
            "acceptance_limits": {
                "crossing_tolerance_rad": policy.crossing_tolerance_rad,
                "position_tolerance_m": policy.position_tolerance_m,
                "maximum_approach_tilt_rad": policy.maximum_approach_tilt_rad,
            },
            "descent_check": {
                "lateral_travel_m": descent_lateral_m,
                "wrist_roll_span_rad": max(rolls.values()) - min(rolls.values()),
                "vertical_only": True,
            },
            "endpoints": endpoints,
            "phases": phases,
            "steps": ordered_steps,
            "arm_segment_count": sum(
                step.get("kind") == "arm" for step in ordered_steps
            ),
            "command_step_count": len(ordered_steps),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = sha256(args.output.read_bytes()).hexdigest()
        print(
            f"{STATUS} arm={SIDE} "
            f"target=({x:.6f},{y:.6f},{z:.6f}) "
            f"can_yaw_deg={math.degrees(yaw):+.2f} "
            f"roll_deg={math.degrees(locked_roll):+.3f} "
            f"crossing_error_deg="
            f"{math.degrees(grasp['crossing_error_rad']):.3f} "
            f"steps={len(ordered_steps)} motion_commands=0 "
            f"execution_api_used=false "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
