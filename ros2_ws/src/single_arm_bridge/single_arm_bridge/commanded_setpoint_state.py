"""Thread-safe ownership of the last successfully commanded six-axis target."""

from __future__ import annotations

import math
import threading
from typing import Sequence


class CommandedSetpointState:
    """Preserve inactive-axis targets without confusing feedback with intent."""

    def __init__(self, joint_count: int = 6) -> None:
        if joint_count <= 0:
            raise ValueError("joint_count must be positive")
        self._joint_count = joint_count
        self._positions: tuple[float, ...] | None = None
        self._lock = threading.RLock()

    def _validated(self, positions: Sequence[float]) -> tuple[float, ...]:
        values = tuple(float(value) for value in positions)
        if len(values) != self._joint_count:
            raise ValueError(
                f"setpoint count must be {self._joint_count}, got {len(values)}"
            )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("setpoint contains a non-finite value")
        return values

    def commit(self, positions: Sequence[float]) -> None:
        """Commit a target only after firmware reports successful completion."""
        values = self._validated(positions)
        with self._lock:
            self._positions = values

    def snapshot(self) -> tuple[float, ...] | None:
        with self._lock:
            return self._positions

    def reset(self) -> None:
        """Discard intent after disable, fault, cancel, or connection loss."""
        with self._lock:
            self._positions = None
