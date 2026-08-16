#!/usr/bin/env python3
"""손가락이 어느 방향으로 벌어지는지 URDF 로 계산하고 손목 각을 역산한다.

**왜 필요한가.**

`ros_moveit_plan_grasp.py` 는 yaw 를 받아서 버린다.

    # The current five-DOF MoveIt configuration deliberately uses
    # position_only_ik. Preserve yaw as candidate metadata, but do not pretend
    # the active IK plugin validated an orientation it does not solve.
    del yaw_rad, tilt_tolerance_rad

정직한 주석이고 그 판단도 맞다 — 5축 IK 플러그인은 방향을 풀지 않는다.
그런데 그 결과 손가락 방향이 IK 가 우연히 낸 값이 된다. 2026-08-06 A4.5 에서
인식이 낸 펜 yaw `-20.5도` 가 계획에 반영되지 않아 그리퍼가 펜과 나란히
닫혔고, 손가락이 펜을 가로지르지 못해 파지에 실패했다.

**팔이 못 하는 것이 아니다.** 위에서 내려찍는 자세에서 `WRIST_ROLL` 이 곧
손가락 방향 손잡이다. 그때 필요한 회전은 `-1.6도` 였고 가동 범위는
`-15.3..+15.0도` 였다. 아무도 시키지 않았을 뿐이다.

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
        self._robot = URDF.from_xml_file(str(urdf_path))
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

    def point_in_base_frame(
        self,
        point_in_root: np.ndarray,
        root_link: str = "workcell_base_link",
    ) -> np.ndarray:
        """Transform a root-frame XYZ point into this arm's base frame."""
        point = np.asarray(point_in_root, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("point_in_root must be one finite XYZ vector")
        chain = self._build_chain(root_link, f"{self.prefix}base_link")
        root_from_base_rotation, root_from_base_translation = self._compose(
            chain, {}
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


def wrap_half_turn(angle_rad: float) -> float:
    """(-pi/2, pi/2] 로 감는다. 그리퍼 손가락 축은 선이지 화살표가 아니다."""
    wrapped = (angle_rad + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.pi / 2.0 if wrapped <= -math.pi / 2.0 else wrapped
