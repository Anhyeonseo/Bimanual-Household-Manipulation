"""Shared ownership guard for arm and gripper motion goals."""

from __future__ import annotations

import threading


VALID_OWNERS = frozenset(
    {
        "arm",
        "gripper",
        "diagnostics",
        "bimanual_read_only_feedback",
        "right_arm_discovery",
        "right_arm_configuration",
        "right_arm_configure_once",
        "right_arm_jog",
        "right_arm_torque_enable",
    }
)


class MotionGoalArbiter:
    """Allow one motion goal or diagnostic snapshot to own the servo bus."""

    def __init__(self) -> None:
        self._owner: str | None = None
        self._lock = threading.RLock()

    @property
    def owner(self) -> str | None:
        with self._lock:
            return self._owner

    def try_reserve(self, owner: str) -> bool:
        if owner not in VALID_OWNERS:
            raise ValueError(f"invalid motion owner: {owner}")
        with self._lock:
            if self._owner is not None:
                return False
            self._owner = owner
            return True

    def release(self, owner: str) -> bool:
        if owner not in VALID_OWNERS:
            raise ValueError(f"invalid motion owner: {owner}")
        with self._lock:
            if self._owner != owner:
                return False
            self._owner = None
            return True
