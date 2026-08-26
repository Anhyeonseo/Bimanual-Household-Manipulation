#!/usr/bin/env python3
"""손가락이 어느 방향으로 벌어지는지 URDF로 계산하고 손목 각을 역산한다.

현재 5축 MoveIt 위치 IK는 yaw를 직접 풀지 않는다. 따라서 손가락 방향을
별도로 풀지 않으면 캔 장축과 닫힘선의 교차 조건을 보장할 수 없다. 위에서
접근하는 자세에서는 `WRIST_ROLL`이 그 방향을 조절하지만, 허용 관절 범위
안에서 가능한 분기를 직접 계산해야 한다.

**부호를 손으로 유도하지 않는다.** `wrist_roll` 의 origin rpy 가 비자명하고
`gripper_frame` 은 Ry(pi) 로 뒤집혀 있다. URDF 를 그대로 합성해 계산한다.

**그리퍼는 180도 대칭이다.** 손가락 축은 방향이 아니라 선이므로, 필요한
회전은 `(-90, +90]` 로 감아서 구한다. 그래야 가동 범위 안에 들어오는 해를
놓치지 않는다.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


# 팔 이름은 접두사로 들어온다. 양팔이 되면 오른팔이 같은 코드를 그대로
# 쓰고, 지금 하드코딩해두면 그때 갈라진 복사본이 생긴다.
DEFAULT_PREFIX = "left_"

ARM_JOINT_SUFFIXES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


def arm_joint_names(prefix: str = DEFAULT_PREFIX) -> tuple[str, ...]:
    return tuple(f"{prefix}{suffix}" for suffix in ARM_JOINT_SUFFIXES)


def wrist_roll_joint(prefix: str = DEFAULT_PREFIX) -> str:
    return f"{prefix}wrist_roll_joint"


ARM_JOINTS = arm_joint_names()
BASE_LINK = f"{DEFAULT_PREFIX}base_link"
GRIPPER_LINK = f"{DEFAULT_PREFIX}gripper_link"
# 계획기가 목표를 다는 링크. TCP 오차는 여기서 재야 계획과 같은 것을 본다.
TCP_LINK = f"{DEFAULT_PREFIX}gripper_frame_link"
JAW_JOINT = f"{DEFAULT_PREFIX}gripper_joint"


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy 규약: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues."""
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ]
    )


class GraspYawKinematics:
    """base -> gripper_link FK 와 손가락 축 계산."""

    def __init__(
        self, urdf_path: Path, prefix: str = DEFAULT_PREFIX
    ) -> None:
        from urdf_parser_py.urdf import URDF

        self.prefix = prefix
        self.arm_joints = arm_joint_names(prefix)
        self._wrist_roll_joint = wrist_roll_joint(prefix)
        jaw_name = f"{prefix}gripper_joint"
        # ``urdf_parser_py`` forwards unicode strings to lxml.  lxml rejects a
        # unicode document that still contains an XML encoding declaration,
        # which is exactly what the registered preview generator emits.  Feed
        # bytes instead so both xacro-generated temporary URDFs and persisted
        # registered previews follow the XML declaration correctly.
        self._robot = URDF.from_xml_string(urdf_path.read_bytes())
        self._by_child = {joint.child: joint for joint in self._robot.joints}
        self._chain = self._build_chain(
            f"{prefix}base_link", f"{prefix}gripper_link"
        )
        self._tcp_chain = self._build_chain(
            f"{prefix}base_link", f"{prefix}gripper_frame_link"
        )
        jaw = next(
            joint for joint in self._robot.joints if joint.name == jaw_name
        )
        # 턱은 이 축을 중심으로 돈다. 손가락이 벌어지는 변위는 그 축과
        # 수직이며, 위에서 내려찍는 자세에서는 접근축과도 수직이다.
        self._jaw_axis_in_gripper = np.array(jaw.axis, dtype=float)
        jaw_rpy = np.array(jaw.origin.rpy, dtype=float)
        self._jaw_axis_in_gripper = (
            _rpy_matrix(*jaw_rpy) @ self._jaw_axis_in_gripper
        )

    def _build_chain(self, base: str, tip: str) -> list:
        chain, link = [], tip
        while link != base:
            joint = self._by_child.get(link)
            if joint is None:
                raise ValueError(f"{tip} does not connect to {base}")
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    def _compose(
        self, chain: list, positions: dict[str, float]
    ) -> tuple[np.ndarray, np.ndarray]:
        """체인을 합성해 (회전, 위치) 를 돌려준다."""
        rotation = np.eye(3)
        translation = np.zeros(3)
        for joint in chain:
            if joint.origin is not None:
                rpy = (
                    np.array(joint.origin.rpy, dtype=float)
                    if joint.origin.rpy is not None
                    else np.zeros(3)
                )
                xyz = (
                    np.array(joint.origin.xyz, dtype=float)
                    if joint.origin.xyz is not None
                    else np.zeros(3)
                )
            else:
                rpy, xyz = np.zeros(3), np.zeros(3)
            translation = translation + rotation @ xyz
            rotation = rotation @ _rpy_matrix(*rpy)
            if joint.type in ("revolute", "continuous"):
                angle = positions.get(joint.name, 0.0)
                rotation = rotation @ _axis_matrix(
                    np.array(joint.axis, dtype=float), angle
                )
        return rotation, translation

    def gripper_rotation(self, positions: dict[str, float]) -> np.ndarray:
        """base 기준 gripper_link 회전행렬."""
        rotation, _ = self._compose(self._chain, positions)
        return rotation

    def tcp_position(self, positions: dict[str, float]) -> np.ndarray:
        """base 기준 TCP(`left_gripper_frame_link`) 위치 [m].

        계획기가 목표를 다는 바로 그 링크다. 여기서 재야 "명령한 자세" 와
        "도달한 자세" 를 같은 자로 비교할 수 있다.
        """
        _, translation = self._compose(self._tcp_chain, positions)
        return translation

    def _root_from_base(
        self, root_link: str = "workcell_base_link"
    ) -> tuple[np.ndarray, np.ndarray]:
        chain = self._build_chain(root_link, f"{self.prefix}base_link")
        return self._compose(chain, {})

    def tcp_pose_in_root(
        self,
        positions: dict[str, float],
        root_link: str = "workcell_base_link",
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(root_R_tcp, root_xyz_tcp)`` from the registered URDF.

        Fold planning must evaluate both arms in one workcell frame.  Returning
        the complete transform also prevents a position-only IK result from
        being mislabeled as a grasp-pose solution.
        """
        root_from_base_rotation, root_from_base_translation = (
            self._root_from_base(root_link)
        )
        base_from_tcp_rotation, base_from_tcp_translation = self._compose(
            self._tcp_chain, positions
        )
        return (
            root_from_base_rotation @ base_from_tcp_rotation,
            root_from_base_translation
            + root_from_base_rotation @ base_from_tcp_translation,
        )

    def approach_axis_in_root(
        self,
        positions: dict[str, float],
        root_link: str = "workcell_base_link",
    ) -> np.ndarray:
        """Return the physical tip approach axis in the root frame.

        ``gripper_frame_joint`` contains ``Ry(pi)``: TCP ``+Z`` therefore
        equals gripper-link ``-Z`` and points from the wrist toward the jaws.
        Using TCP ``-Z`` here would silently reverse the contact approach.
        """
        rotation, _ = self.tcp_pose_in_root(positions, root_link)
        return rotation @ np.array([0.0, 0.0, 1.0])

    def finger_axis_in_root(
        self,
        positions: dict[str, float],
        root_link: str = "workcell_base_link",
    ) -> np.ndarray:
        """Return the jaw-opening line in the shared root frame."""
        root_from_base_rotation, _ = self._root_from_base(root_link)
        return root_from_base_rotation @ self.finger_axis(positions)

    def point_in_base_frame(
        self,
        point_in_root: np.ndarray,
        root_link: str = "workcell_base_link",
    ) -> np.ndarray:
        """Transform a root-frame XYZ point into this arm's base frame."""
        point = np.asarray(point_in_root, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("point_in_root must be one finite XYZ vector")
        root_from_base_rotation, root_from_base_translation = (
            self._root_from_base(root_link)
        )
        return root_from_base_rotation.T @ (
            point - root_from_base_translation
        )

    def tcp_error_m(
        self,
        commanded: dict[str, float],
        measured: dict[str, float],
    ) -> np.ndarray:
        """명령 자세와 측정 자세의 TCP 변위 [m]. 근사식이 아니라 FK 차이다.

        관절 오차를 `raw x 반경` 으로 환산하면 어느 관절이 틀렸는지에 따라
        답이 달라진다. FK 로 두 자세의 TCP 를 각각 구해 빼면 그 모호함이 없고,
        팔이 바뀌어도 같은 방식이 그대로 쓰인다.
        """
        return self.tcp_position(measured) - self.tcp_position(commanded)

    def finger_axis(self, positions: dict[str, float]) -> np.ndarray:
        """base 기준 손가락이 벌어지는 방향(단위벡터).

        턱 회전축과 접근축(도구 -Z)에 모두 수직인 방향이다.
        """
        rotation = self.gripper_rotation(positions)
        jaw_axis = rotation @ self._jaw_axis_in_gripper
        approach = rotation @ np.array([0.0, 0.0, -1.0])
        finger = np.cross(jaw_axis, approach)
        norm = np.linalg.norm(finger)
        if norm < 1.0e-9:
            raise ValueError("jaw axis is parallel to the approach axis")
        return finger / norm

    def finger_yaw(self, positions: dict[str, float]) -> float:
        """손가락 축을 수평면에 투영한 방위각. 180도 대칭이라 (-pi/2, pi/2]."""
        axis = self.finger_axis(positions)
        return wrap_half_turn(math.atan2(axis[1], axis[0]))

    def solve_wrist_roll(
        self,
        positions: dict[str, float],
        target_yaw_rad: float,
        lower_rad: float,
        upper_rad: float,
    ) -> dict[str, object]:
        """손가락을 target_yaw 에 맞추는 wrist_roll 을 구한다.

        회전은 (-90, +90] 로 감는다. 그리퍼가 180도 대칭이므로 그 안에서
        항상 해가 존재하며, 가동 범위 밖이면 그 사실을 그대로 보고한다.
        """
        current = dict(positions)
        current_roll = current.get(self._wrist_roll_joint, 0.0)
        present = self.finger_yaw(current)
        delta = wrap_half_turn(wrap_half_turn(target_yaw_rad) - present)
        solved = current_roll + delta

        # 수치로 확인한다. 유도한 관계가 맞는지 계산으로 되돌려 본다.
        check = dict(current)
        check[self._wrist_roll_joint] = solved
        achieved = self.finger_yaw(check)
        residual = abs(wrap_half_turn(achieved - wrap_half_turn(target_yaw_rad)))

        return {
            "present_finger_yaw_rad": present,
            "target_yaw_rad": wrap_half_turn(target_yaw_rad),
            "required_delta_rad": delta,
            "solved_wrist_roll_rad": solved,
            "achieved_finger_yaw_rad": achieved,
            "residual_rad": residual,
            "within_limits": lower_rad <= solved <= upper_rad,
            "limit_lower_rad": lower_rad,
            "limit_upper_rad": upper_rad,
        }

    def solve_wrist_roll_branches(
        self,
        positions: dict[str, float],
        target_yaw_rad: float,
        lower_rad: float,
        upper_rad: float,
        samples: int = 721,
    ) -> dict[str, object]:
        """가동 범위 안의 wrist_roll 해를 **전부** 찾아 가까운 순으로 돌려준다.

        `solve_wrist_roll` 은 `roll_new = roll_now + Δfinger_yaw`, 즉 gain 1 을
        가정한다. 그 가정은 회전축(=접근축)이 연직일 때만 맞다. 이 팔은 작업대
        높이에서 접근축이 수직에서 51~77도 기울어 있어 실제 gain 은 0.43~0.61
        이다. 그래서 해석식 대신 `finger_yaw(roll) = target` 을 수치로 푼다.

        분기가 둘인 이유. 손가락 축은 선이라 180도마다 같은 자세가 되는데,
        이 팔의 roll 가동 범위는 195~198도로 180도보다 넓다. 따라서 어떤 캔
        방향에서는 한계 안에 해가 두 개 있고, 그때 가까운 쪽을 고르지 않으면
        쓸데없이 반바퀴를 돈다. 반대로 대부분의 방향에서는 해가 하나뿐이고
        수학적으로 더 가까운 분기는 한계 밖이다. **그래서 한계 검사가 분기
        선택보다 먼저다.**
        """
        if samples < 3:
            raise ValueError("samples must be at least 3")
        if not lower_rad < upper_rad:
            raise ValueError("wrist roll limits must satisfy lower < upper")

        current = dict(positions)
        current_roll = current.get(self._wrist_roll_joint, 0.0)
        target = wrap_half_turn(target_yaw_rad)

        def error(roll: float) -> float:
            probe = dict(current)
            probe[self._wrist_roll_joint] = roll
            return wrap_half_turn(self.finger_yaw(probe) - target)

        grid = [
            lower_rad + (upper_rad - lower_rad) * index / (samples - 1)
            for index in range(samples)
        ]
        values = [error(roll) for roll in grid]

        roots: list[float] = []
        for index in range(samples - 1):
            low_value, high_value = values[index], values[index + 1]
            if low_value == 0.0:
                roots.append(grid[index])
                continue
            if low_value * high_value >= 0.0:
                continue
            # (-pi/2, pi/2] 로 감긴 오차는 근이 아닌 곳에서도 부호가 뒤집힌다.
            # 진짜 근은 두 표본이 붙어 있고, wrap 도약은 pi 에 가깝다.
            if abs(low_value - high_value) > math.pi / 2.0:
                continue
            lower_bound, upper_bound = grid[index], grid[index + 1]
            lower_value = low_value
            for _ in range(80):
                middle = 0.5 * (lower_bound + upper_bound)
                middle_value = error(middle)
                if lower_value * middle_value <= 0.0:
                    upper_bound = middle
                else:
                    lower_bound, lower_value = middle, middle_value
            roots.append(0.5 * (lower_bound + upper_bound))
        if values[-1] == 0.0:
            roots.append(grid[-1])

        unique: list[float] = []
        for root in roots:
            if all(abs(root - kept) > 1.0e-6 for kept in unique):
                unique.append(root)
        unique.sort(key=lambda root: abs(root - current_roll))

        branches = []
        for root in unique:
            probe = dict(current)
            probe[self._wrist_roll_joint] = root
            branches.append(
                {
                    "wrist_roll_rad": root,
                    "rotation_from_current_rad": root - current_roll,
                    "achieved_finger_yaw_rad": self.finger_yaw(probe),
                    "residual_rad": abs(error(root)),
                    "limit_margin_rad": min(
                        root - lower_rad, upper_rad - root
                    ),
                }
            )

        return {
            "present_finger_yaw_rad": self.finger_yaw(current),
            "present_wrist_roll_rad": current_roll,
            "target_yaw_rad": target,
            "branches": branches,
            "branch_count": len(branches),
            "selected": branches[0] if branches else None,
            "limit_lower_rad": lower_rad,
            "limit_upper_rad": upper_rad,
            "samples": samples,
        }


def wrap_half_turn(angle_rad: float) -> float:
    """(-pi/2, pi/2] 로 감는다. 그리퍼 손가락 축은 선이지 화살표가 아니다."""
    wrapped = (angle_rad + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.pi / 2.0 if wrapped <= -math.pi / 2.0 else wrapped
