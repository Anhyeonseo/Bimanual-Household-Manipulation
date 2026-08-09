"""Calibration loading and ROS-radian conversion."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .protocol import crc32c


FIRMWARE_CLEAR_STOP_MARGIN_RAW = 40


@dataclass(frozen=True, slots=True)
class JointCalibration:
    servo_id: int
    name: str
    zero_raw: int
    minimum_raw: int
    maximum_raw: int
    direction: int
    p_gain: int
    d_gain: int


@dataclass(frozen=True, slots=True)
class ArmCalibration:
    arm_slot: str
    joints: tuple[JointCalibration, ...]
    calibration_hash: int

    @property
    def ros_joint_names(self) -> list[str]:
        return [f"{self.arm_slot}_{joint.name.lower()}_joint" for joint in self.joints]

    @property
    def ros_radian_limits(self) -> dict[str, tuple[float, float]]:
        scale = 2.0 * math.pi / 4096.0
        result: dict[str, tuple[float, float]] = {}
        for name, joint in zip(self.ros_joint_names, self.joints, strict=True):
            endpoint_a = (
                (joint.minimum_raw - joint.zero_raw) * joint.direction * scale
            )
            endpoint_b = (
                (joint.maximum_raw - joint.zero_raw) * joint.direction * scale
            )
            result[name] = (
                min(endpoint_a, endpoint_b),
                max(endpoint_a, endpoint_b),
            )
        return result

    def raw_feedback_to_radians(self, raw_positions: tuple[int, ...]) -> list[float]:
        if len(raw_positions) != len(self.joints):
            raise ValueError("feedback joint count mismatch")
        return [
            (raw - joint.zero_raw) * joint.direction * (2.0 * math.pi / 4096.0)
            for joint, raw in zip(self.joints, raw_positions, strict=True)
        ]

    def validate_feedback_recovery_envelope(
        self,
        positions: tuple[float, ...],
    ) -> dict[str, int]:
        """Validate feedback against the firmware clear-stop recovery envelope.

        Returns each joint outside the strict range and its raw overrun while
        requiring every position to remain inside the recovery envelope.
        """

        if len(positions) != len(self.joints):
            raise ValueError("feedback joint count mismatch")

        outside_strict: dict[str, int] = {}
        for name, joint, position in zip(
            self.ros_joint_names,
            self.joints,
            positions,
            strict=True,
        ):
            if not math.isfinite(position):
                raise ValueError(f"{joint.name}: non-finite feedback position")
            raw = round(
                joint.zero_raw
                + joint.direction * position * 4096.0 / (2.0 * math.pi)
            )
            recovery_minimum = (
                joint.minimum_raw - FIRMWARE_CLEAR_STOP_MARGIN_RAW
            )
            recovery_maximum = (
                joint.maximum_raw + FIRMWARE_CLEAR_STOP_MARGIN_RAW
            )
            if not recovery_minimum <= raw <= recovery_maximum:
                raise ValueError(
                    f"{joint.name}: feedback raw {raw} outside recovery range "
                    f"{recovery_minimum}..{recovery_maximum}"
                )
            if raw < joint.minimum_raw:
                outside_strict[name] = joint.minimum_raw - raw
            elif raw > joint.maximum_raw:
                outside_strict[name] = raw - joint.maximum_raw
        return outside_strict

    def radians_to_urad(self, positions: list[float]) -> list[int]:
        if len(positions) != len(self.joints):
            raise ValueError("command joint count mismatch")
        result: list[int] = []
        for joint, position in zip(self.joints, positions, strict=True):
            if not math.isfinite(position):
                raise ValueError(f"{joint.name}: non-finite position")
            raw = round(
                joint.zero_raw
                + joint.direction * position * 4096.0 / (2.0 * math.pi)
            )
            if not joint.minimum_raw <= raw <= joint.maximum_raw:
                raise ValueError(
                    f"{joint.name}: target raw {raw} outside "
                    f"{joint.minimum_raw}..{joint.maximum_raw}"
                )
            result.append(round(position * 1_000_000.0))
        return result


def load_calibration(path: str | Path) -> ArmCalibration:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    joints = tuple(
        JointCalibration(
            servo_id=item["id"],
            name=item["name"],
            zero_raw=item["zero_raw"],
            minimum_raw=item["minimum_raw"],
            maximum_raw=item["maximum_raw"],
            direction=item["positive_raw_direction"],
            p_gain=item["p_gain"],
            d_gain=item["d_gain"],
        )
        for item in document["joints"]
    )
    if len(joints) != 6 or [joint.servo_id for joint in joints] != list(range(1, 7)):
        raise ValueError("calibration must contain ordered servo IDs 1..6")

    serialized = bytearray()
    for joint in joints:
        serialized.extend(
            bytes((joint.servo_id,))
            + joint.zero_raw.to_bytes(2, "little")
            + joint.minimum_raw.to_bytes(2, "little")
            + joint.maximum_raw.to_bytes(2, "little")
            + (joint.direction & 0xFF).to_bytes(1, "little")
            + joint.p_gain.to_bytes(1, "little")
            + joint.d_gain.to_bytes(1, "little")
        )
    return ArmCalibration(document["arm_slot"], joints, crc32c(bytes(serialized)))
