"""Pure validation and conversion for the R4 read-only 12-axis snapshot."""

from __future__ import annotations

from dataclasses import dataclass

from .calibration import ArmCalibration
from .protocol import RightArmDiscovery


@dataclass(frozen=True, slots=True)
class BimanualFeedback:
    names: tuple[str, ...]
    positions: tuple[float, ...]


def validate_bimanual_calibrations(
    left: ArmCalibration,
    right: ArmCalibration,
) -> None:
    """Reject ambiguous arm identity before exposing a combined topic."""

    if left.arm_slot != "left":
        raise ValueError("primary calibration arm_slot must be left")
    if right.arm_slot != "right":
        raise ValueError("right calibration arm_slot must be right")
    if left.calibration_hash != right.calibration_hash:
        raise ValueError(
            "left/right calibration hashes differ: "
            f"left=0x{left.calibration_hash:08X} "
            f"right=0x{right.calibration_hash:08X}"
        )
    names = tuple(left.ros_joint_names + right.ros_joint_names)
    if len(names) != 12 or len(set(names)) != 12:
        raise ValueError("bimanual joint names must contain 12 unique axes")


def compose_bimanual_feedback(
    left: ArmCalibration,
    right: ArmCalibration,
    left_raw: tuple[int, ...],
    right_snapshot: RightArmDiscovery,
) -> BimanualFeedback:
    """Build one validated, explicitly asynchronous bimanual observation."""

    validate_bimanual_calibrations(left, right)
    if len(left_raw) != 6:
        raise ValueError("left-arm feedback must contain six positions")
    if right_snapshot.joint_count != 6:
        raise ValueError("right-arm discovery joint count is not six")
    if right_snapshot.status_code != 0:
        raise ValueError(
            f"right-arm discovery status is {right_snapshot.status_code}"
        )
    if right_snapshot.present_mask != 0x3F:
        raise ValueError(
            "right-arm discovery is incomplete: "
            f"present_mask=0x{right_snapshot.present_mask:02X}"
        )
    if right_snapshot.failure_count != 0 or any(right_snapshot.read_statuses):
        raise ValueError("right-arm discovery contains failed reads")

    left_positions = left.raw_feedback_to_radians(left_raw)
    right_positions = right.raw_feedback_to_radians(
        right_snapshot.positions_raw
    )
    return BimanualFeedback(
        names=tuple(left.ros_joint_names + right.ros_joint_names),
        positions=tuple(left_positions + right_positions),
    )
