#!/usr/bin/env python3
"""캔 한 개를 방향에 맞춰 집는 순수 판정·해법 모듈.

ROS publisher/service client, serial transport, motion executor를 넣지 않는다.
`lying_can_upright_application.py`와 같은 격리 수준을 유지한다.

**펜 계획기와 다른 점 하나.** 펜은 `wrist_roll`을 q0인 0에 고정하고 나머지
4축으로 TCP xyz만 맞췄다. 캔은 굴러가므로 손가락 닫힘선이 캔 장축을 반드시
가로질러야 하고, 그 손잡이가 `wrist_roll`이다.

그런데 두 가지가 순진한 구현을 막는다.

1. **`finger_yaw`와 `wrist_roll`은 1:1이 아니다.** `finger_yaw`는 손가락 축을
   수평면에 투영한 방위각이라, 회전축(=접근축)이 연직일 때만 gain이 1이 된다.
   이 팔은 작업대 높이에서 접근축이 수직에서 51~77도 기울어 있어 gain이
   0.43~0.61이다. 그래서 `roll += Δyaw` 식은 쓸 수 없고 수치로 풀어야 한다.
   (`GraspYawKinematics.solve_wrist_roll_branches`)

2. **`wrist_roll`을 돌리면 TCP가 최대 13.3 mm 움직인다.** TCP가 roll 축에서
   7.9 mm 편심돼 있다. 그래서 xyz를 먼저 풀고 roll을 얹으면 파지점이 어긋난다.
   roll을 고정한 상태로 4축을 다시 풀어야 한다.

이 둘이 겹쳐서 **결합 문제**가 된다. roll을 바꾸면 4축 해가 바뀌고, 4축 해가
바뀌면 gain이 바뀌어 같은 목표 yaw에 필요한 roll이 또 바뀐다. 번갈아 푸는
방식(위치 → roll → 위치 → …)은 실제로 **진동해서 수렴하지 않는다.** 그래서
`solve_can_pick_endpoint`는 위치 3개와 교차각 1개를 한 residual 벡터에 넣어
5축을 동시에 풀고, 분기 열거는 "어느 분기에서 출발할지"를 고르는 데만 쓴다.
어떤 분기도 수락 한계를 통과하지 못하면 조용히 근사하지 않고 거부한다.

수치 근거: `docs/PLAN_CAN_TO_BIN.md` §2,
`tools/probe_can_grasp_geometry_plan_only.py`.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from lying_can_upright_application import (
    LyingCanContractError,
    undirected_axis_error,
    wrap_undirected_axis,
)


class CanPickContractError(RuntimeError):
    """파지 전제가 안전하게 성립하지 않는다."""


WORKCELL_FRAME = "workcell_base_link"
DOWN = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class CalibratedRegion:
    """상단 보정판이 덮는 workcell 영역.

    **상수로 박아두지 않는다.** 2026-08-16 재보정에서 span이
    `[0.18, 0.28]`에서 `[0.290, 0.393]`으로 바뀌었다. 값을 복사해 두면
    보정과 코드가 조용히 갈라진다. 항상 homography YAML에서 읽는다.
    """

    origin_xy_m: tuple[float, float]
    span_xy_m: tuple[float, float]
    table_z_m: float
    source_path: str
    source_sha256: str

    @property
    def x_bounds_m(self) -> tuple[float, float]:
        return (
            self.origin_xy_m[0],
            self.origin_xy_m[0] + self.span_xy_m[0],
        )

    @property
    def y_bounds_m(self) -> tuple[float, float]:
        return (
            self.origin_xy_m[1],
            self.origin_xy_m[1] + self.span_xy_m[1],
        )

    def require_inside(self, x_m: float, y_m: float) -> None:
        if not all(math.isfinite(value) for value in (x_m, y_m)):
            raise CanPickContractError("target coordinates must be finite")
        low_x, high_x = self.x_bounds_m
        low_y, high_y = self.y_bounds_m
        if not low_x <= x_m <= high_x:
            raise CanPickContractError(
                f"target x={x_m:.4f} m is outside the calibrated region "
                f"[{low_x:.4f}, {high_x:.4f}]"
            )
        if not low_y <= y_m <= high_y:
            raise CanPickContractError(
                f"target y={y_m:.4f} m is outside the calibrated region "
                f"[{low_y:.4f}, {high_y:.4f}]"
            )

    def board_to_workcell_xy_m(
        self, board_x_m: float, board_y_m: float
    ) -> tuple[float, float]:
        """board 좌표를 workcell(=left_base_link)로 옮긴다.

        homography YAML이 board 축을 `left_base_link` 축에 평행하다고
        선언하므로 평행이동뿐이다. 회전을 가정하지 않는다.
        """
        return (
            self.origin_xy_m[0] + float(board_x_m),
            self.origin_xy_m[1] + float(board_y_m),
        )


def load_calibrated_region(homography_path) -> CalibratedRegion:
    """homography YAML에서 보정 영역과 작업대 높이를 읽는다."""
    import hashlib

    import yaml

    path = Path(homography_path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    board = document.get("board")
    if not isinstance(board, dict):
        raise CanPickContractError("homography has no board block")
    if board.get("positive_x") != "parallel to left_base_link +X" or board.get(
        "positive_y"
    ) != "parallel to left_base_link +Y":
        raise CanPickContractError(
            "board axes are not declared parallel to left_base_link; the "
            "translation-only board->workcell mapping is not valid"
        )
    span = board.get("calibrated_span_m")
    origin = board.get("origin_in_left_base_link_xy_m")
    table_z = board.get("table_z_in_left_base_link_m")
    for name, value in (("calibrated_span_m", span), ("origin", origin)):
        if (
            not isinstance(value, list)
            or len(value) != 2
            or not all(isinstance(item, (int, float)) for item in value)
        ):
            raise CanPickContractError(f"homography {name} is malformed")
    if not isinstance(table_z, (int, float)):
        raise CanPickContractError("homography table_z is malformed")
    if not all(float(item) > 0.0 for item in span):
        raise CanPickContractError("calibrated span must be positive")
    return CalibratedRegion(
        origin_xy_m=(float(origin[0]), float(origin[1])),
        span_xy_m=(float(span[0]), float(span[1])),
        table_z_m=float(table_z),
        source_path=str(path),
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )


@dataclass(frozen=True)
class CanJawContract:
    """실측으로만 채우는 그리퍼 계약.

    `open_gap_mm`은 접근 전 개방 폭이고 `grasp_gap_mm`은 캔을 문 상태의 폭이다.
    캔이 얇은 알루미늄이면 지름보다 좁게 물려 눌릴 수 있으므로 두 값을 따로
    둔다. **둘 중 하나라도 None이면 계획을 거부한다.**
    """

    open_gap_mm: float | None
    grasp_gap_mm: float | None
    open_command_rad: float | None
    grasp_command_rad: float | None
    contact_threshold_raw: int | None
    release_tolerance_raw: int | None
    can_diameter_mm: float
    provenance: str
    can_length_mm: float = 132.44

    def require_commissioned(self) -> None:
        missing = [
            name
            for name in (
                "open_gap_mm",
                "grasp_gap_mm",
                "open_command_rad",
                "grasp_command_rad",
                "contact_threshold_raw",
                "release_tolerance_raw",
            )
            if getattr(self, name) is None
        ]
        if missing:
            raise CanPickContractError(
                "gripper contract is not commissioned; missing "
                + ", ".join(missing)
            )
        if self.open_gap_mm <= self.can_diameter_mm:
            raise CanPickContractError(
                f"open gap {self.open_gap_mm:.1f} mm does not clear a "
                f"{self.can_diameter_mm:.1f} mm can; the jaws would push it"
            )


@dataclass(frozen=True)
class CanPickPolicy:
    """파지 계획의 수락 한계."""

    # 닫힘선이 캔 장축 수직에서 벗어나도 되는 한계.
    crossing_tolerance_rad: float
    # TCP 위치 잔차 한계. 펜 계획기의 PLAN_RESIDUAL_BOUND_M와 같은 역할.
    position_tolerance_m: float
    # 접근축이 수직에서 기울어도 되는 한계. M3에서 finger 간섭으로 정한다.
    maximum_approach_tilt_rad: float | None
    jaw: CanJawContract

    def __post_init__(self) -> None:
        for name in ("crossing_tolerance_rad", "position_tolerance_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

    def require_open_gap_covers_tolerance(self) -> float:
        """개방 폭이 허용 교차각까지 실제로 감당하는지 본다.

        **개방 폭을 캔 지름으로 정하면 안 된다.** 닫힘선이 θ 만큼 어긋나면
        조가 벌려야 하는 폭은 `required_jaw_width_mm(θ)` 이고, 이 값은 θ 에
        대해 급격히 커진다. 53 mm 캔에서 61 mm 로 열면 허용 오차가 약 3.4°
        뿐인데 인식 yaw 오차만 p95 2.36° 다. 즉 개방 폭이 교차각 계약을
        조용히 무효로 만든다.

        여기서 두 값을 묶어두면 계획 단계에서 걸린다.
        """
        self.jaw.require_commissioned()
        needed_mm = required_jaw_width_mm(
            self.crossing_tolerance_rad,
            self.jaw.can_length_mm,
            self.jaw.can_diameter_mm,
        )
        if self.jaw.open_gap_mm < needed_mm:
            raise CanPickContractError(
                f"open gap {self.jaw.open_gap_mm:.1f} mm cannot hold the can at "
                f"the {math.degrees(self.crossing_tolerance_rad):.2f} deg "
                f"crossing tolerance, which needs {needed_mm:.1f} mm; widen the "
                "opening or tighten the tolerance"
            )
        return needed_mm


def load_can_pick_policy(contract_path) -> tuple[CanPickPolicy, dict]:
    """계약 JSON 에서 수락 한계와 그리퍼 실측값을 읽는다.

    실측되지 않은 값은 JSON 에 `null` 로 남아 있고, 그대로 dataclass 에 들어가
    `require_commissioned()` 에서 걸린다. **로더가 기본값을 채우지 않는다** —
    채우면 안 잰 값이 잰 값처럼 계획에 실린다.
    """
    import hashlib
    import json

    path = Path(contract_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1:
        raise CanPickContractError("can pick contract schema is not 1")
    if document.get("record_kind") != "can_pick_contract":
        raise CanPickContractError("contract record kind is wrong")
    can = document["can"]
    jaw_document = document["jaw"]
    limits = document["acceptance_limits"]
    jaw = CanJawContract(
        open_gap_mm=jaw_document["open_gap_mm"],
        grasp_gap_mm=jaw_document["grasp_gap_mm"],
        open_command_rad=jaw_document["open_command_rad"],
        grasp_command_rad=jaw_document["grasp_command_rad"],
        contact_threshold_raw=jaw_document["contact_threshold_raw"],
        release_tolerance_raw=jaw_document["release_tolerance_raw"],
        can_diameter_mm=float(can["diameter_mm"]),
        provenance=str(jaw_document["provenance"]),
        can_length_mm=float(can["length_mm"]),
    )
    tilt = limits["maximum_approach_tilt_deg"]
    policy = CanPickPolicy(
        crossing_tolerance_rad=math.radians(
            float(limits["crossing_tolerance_deg"])
        ),
        position_tolerance_m=float(limits["position_tolerance_m"]),
        maximum_approach_tilt_rad=(
            None if tilt is None else math.radians(float(tilt))
        ),
        jaw=jaw,
    )
    provenance = {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "status": document.get("status"),
        "jaw_provenance": jaw.provenance,
    }
    return policy, provenance


def finger_target_yaw(can_axis_yaw_rad: float) -> float:
    """캔 장축을 90도로 가로지르는 손가락 닫힘선 방위각.

    두 선 모두 무방향이므로 `(-pi/2, pi/2]`로 감는다.
    """
    return wrap_undirected_axis(
        wrap_undirected_axis(can_axis_yaw_rad) + math.pi / 2.0
    )


def required_jaw_width_mm(
    crossing_error_rad: float,
    can_length_mm: float,
    can_diameter_mm: float,
) -> float:
    """닫힘선이 수직에서 벗어났을 때 조가 벌려야 하는 폭.

    캔을 원통으로 보면 닫힘 방향 투영 폭은 아래와 같다. 교차각이 어긋날수록
    급격히 커지므로 이 식이 `crossing_tolerance_rad`의 물리적 근거다.
    """
    theta = abs(float(crossing_error_rad))
    return can_diameter_mm * abs(math.cos(theta)) + can_length_mm * abs(
        math.sin(theta)
    )


def _solve_position_with_fixed_roll(
    kinematics,
    joint_names: Sequence[str],
    target_base_xyz: np.ndarray,
    roll_rad: float,
    seed_four: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, float]:
    """roll을 고정하고 나머지 4축으로 TCP xyz를 맞춘다."""

    def residuals(values):
        positions = dict(
            zip(joint_names, list(values) + [roll_rad], strict=True)
        )
        return (kinematics.tcp_position(positions) - target_base_xyz) * 1000.0

    best_values = None
    best_error = math.inf
    seeds = (
        seed_four,
        np.array([0.0, 2.6, 1.2, 1.3]),
        np.array([-0.3, 2.2, 1.0, 1.6]),
        np.array([0.3, 2.9, 1.5, 0.9]),
    )
    for seed in seeds:
        clipped = np.clip(
            np.asarray(seed, dtype=float),
            lower[:4] + 1.0e-9,
            upper[:4] - 1.0e-9,
        )
        result = least_squares(
            residuals,
            clipped,
            bounds=(lower[:4], upper[:4]),
            xtol=1.0e-12,
            ftol=1.0e-12,
            gtol=1.0e-12,
            max_nfev=1200,
        )
        positions = dict(
            zip(joint_names, list(result.x) + [roll_rad], strict=True)
        )
        error = float(
            np.linalg.norm(
                kinematics.tcp_position(positions) - target_base_xyz
            )
        )
        if error < best_error:
            best_error = error
            best_values = np.asarray(result.x, dtype=float)
    return best_values, best_error


def _solve_position_and_crossing(
    kinematics,
    joint_names: Sequence[str],
    target_base_xyz: np.ndarray,
    target_yaw_rad: float,
    seed_five: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    crossing_weight_mm_per_rad: float = 200.0,
) -> tuple[np.ndarray, float, float]:
    """5축으로 TCP xyz와 손가락 교차각을 **동시에** 맞춘다.

    번갈아 푸는 방식(위치 → roll → 위치 → …)은 진동한다. roll 을 바꾸면 4축
    해가 바뀌고, 그러면 같은 목표 yaw 에 필요한 roll 이 또 옮겨가기 때문이다.
    그래서 두 조건을 하나의 residual 벡터에 넣어 한 번에 푼다. 어느 분기에서
    출발할지는 호출자가 seed 로 정한다 — 그게 "최단 회전" 선택이 들어오는 자리다.
    """

    def residuals(values):
        positions = dict(zip(joint_names, values, strict=True))
        position_mm = (
            kinematics.tcp_position(positions) - target_base_xyz
        ) * 1000.0
        crossing = wrap_undirected_axis(
            kinematics.finger_yaw(positions) - target_yaw_rad
        )
        return np.concatenate(
            [position_mm, [crossing_weight_mm_per_rad * crossing]]
        )

    clipped = np.clip(
        np.asarray(seed_five, dtype=float), lower + 1.0e-9, upper - 1.0e-9
    )
    result = least_squares(
        residuals,
        clipped,
        bounds=(lower, upper),
        xtol=1.0e-12,
        ftol=1.0e-12,
        gtol=1.0e-12,
        max_nfev=2000,
    )
    solved = np.asarray(result.x, dtype=float)
    positions = dict(zip(joint_names, solved, strict=True))
    position_error_m = float(
        np.linalg.norm(kinematics.tcp_position(positions) - target_base_xyz)
    )
    crossing_error_rad = undirected_axis_error(
        kinematics.finger_yaw(positions), target_yaw_rad
    )
    return solved, position_error_m, crossing_error_rad


def solve_can_pick_endpoint(
    kinematics,
    joint_names: Sequence[str],
    target_workcell_xyz: Sequence[float],
    can_axis_yaw_rad: float,
    current_joints_rad: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    policy: CanPickPolicy,
) -> dict[str, object]:
    """캔 방향에 맞춘 한 endpoint의 5축 해를 구한다.

    1. 현재 roll 에서 위치만 맞춰 기준 자세를 얻는다.
    2. 그 자세에서 목표 yaw 를 만드는 **한계 안 roll 분기를 전부** 열거한다.
    3. 현재 roll 에서 **가까운 분기부터** 차례로 seed 로 넣고 위치·교차각을
       동시에 푼다.
    4. 모든 수락 한계를 통과하는 **첫** 해를 채택한다. 가까운 순으로 봤으므로
       그게 최단 회전 해다.

    어떤 분기도 통과하지 못하면 근사하지 않고 거부한다.
    """
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    current = np.asarray(current_joints_rad, dtype=float)
    if lower.shape != (5,) or upper.shape != (5,) or current.shape != (5,):
        raise ValueError("limits and current joints must each have 5 values")
    if not np.all(lower < upper):
        raise ValueError("each joint limit must satisfy lower < upper")

    target_base = kinematics.point_in_base_frame(
        np.asarray(target_workcell_xyz, dtype=float),
        root_link=WORKCELL_FRAME,
    )
    target_yaw = finger_target_yaw(can_axis_yaw_rad)
    current_roll = float(current[4])

    # 1) 기준 자세. 위치가 애초에 안 되면 방향을 볼 필요가 없다.
    seed_four, seed_position_error_m = _solve_position_with_fixed_roll(
        kinematics,
        joint_names,
        target_base,
        current_roll,
        np.asarray(current[:4], dtype=float),
        lower,
        upper,
    )
    if seed_position_error_m > policy.position_tolerance_m:
        raise CanPickContractError(
            "target is not reachable at this height: TCP position residual "
            f"{seed_position_error_m * 1000.0:.3f} mm exceeds "
            f"{policy.position_tolerance_m * 1000.0:.3f} mm"
        )

    # 2) 한계 안 roll 분기 열거. 한계 필터가 분기 선택보다 먼저다.
    seed_pose = dict(
        zip(joint_names, list(seed_four) + [current_roll], strict=True)
    )
    enumerated = kinematics.solve_wrist_roll_branches(
        seed_pose, target_yaw, float(lower[4]), float(upper[4])
    )
    if enumerated["branch_count"] == 0:
        raise CanPickContractError(
            "no wrist roll within the operational limits puts the finger line "
            f"across the can axis (target finger yaw {target_yaw:+.4f} rad)"
        )
    candidate_rolls = [
        float(branch["wrist_roll_rad"])
        for branch in sorted(
            enumerated["branches"],
            key=lambda item: abs(item["wrist_roll_rad"] - current_roll),
        )
    ]

    # 3~4) 가까운 분기부터 동시 해를 시도한다.
    rejections: list[str] = []
    for index, candidate in enumerate(candidate_rolls):
        solved, position_error_m, crossing_error_rad = (
            _solve_position_and_crossing(
                kinematics,
                joint_names,
                target_base,
                target_yaw,
                np.concatenate((seed_four, [candidate])),
                lower,
                upper,
            )
        )
        positions = dict(zip(joint_names, solved, strict=True))
        approach = kinematics.gripper_rotation(positions) @ DOWN
        tilt_rad = math.acos(
            max(-1.0, min(1.0, float(np.dot(approach, DOWN))))
        )
        label = f"branch {index} roll={math.degrees(candidate):+.2f} deg"

        if position_error_m > policy.position_tolerance_m:
            rejections.append(
                f"{label}: position residual "
                f"{position_error_m * 1000.0:.3f} mm exceeds "
                f"{policy.position_tolerance_m * 1000.0:.3f} mm"
            )
            continue
        if crossing_error_rad > policy.crossing_tolerance_rad:
            rejections.append(
                f"{label}: crossing error "
                f"{math.degrees(crossing_error_rad):.3f} deg exceeds "
                f"{math.degrees(policy.crossing_tolerance_rad):.3f} deg"
            )
            continue
        if np.any(solved < lower - 1.0e-9) or np.any(solved > upper + 1.0e-9):
            rejections.append(f"{label}: solved joints violate the limits")
            continue
        if (
            policy.maximum_approach_tilt_rad is not None
            and tilt_rad > policy.maximum_approach_tilt_rad
        ):
            rejections.append(
                f"{label}: approach tilt {math.degrees(tilt_rad):.2f} deg "
                "exceeds the commissioned limit "
                f"{math.degrees(policy.maximum_approach_tilt_rad):.2f} deg"
            )
            continue

        return {
            "joint_positions_rad": [float(value) for value in solved],
            "position_residual_m": position_error_m,
            "can_axis_yaw_rad": wrap_undirected_axis(can_axis_yaw_rad),
            "finger_target_yaw_rad": target_yaw,
            "achieved_finger_yaw_rad": float(
                kinematics.finger_yaw(positions)
            ),
            "crossing_error_rad": crossing_error_rad,
            "crossing_tolerance_rad": policy.crossing_tolerance_rad,
            "wrist_roll_rad": float(solved[4]),
            "wrist_roll_rotation_from_current_rad": float(
                solved[4] - current_roll
            ),
            "wrist_roll_policy": (
                "nearest_in_limit_branch_then_joint_position_crossing_solve"
            ),
            "wrist_roll_branch_index": index,
            "wrist_roll_branch_count": int(enumerated["branch_count"]),
            "wrist_roll_candidates_rad": candidate_rolls,
            "approach_axis_base": [float(value) for value in approach],
            "approach_tilt_from_vertical_rad": tilt_rad,
            "joint_limit_margin_rad": float(
                min(np.min(solved - lower), np.min(upper - solved))
            ),
            "rejected_branches": rejections,
        }

    raise CanPickContractError(
        "no in-limit wrist roll branch satisfies the grasp contract: "
        + "; ".join(rejections)
    )
