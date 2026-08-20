"""Pure geometry helpers for a cylindrical desk object."""

from __future__ import annotations

import math


class CanGeometryError(RuntimeError):
    """A can geometry input is invalid."""


def wrap_undirected_axis(angle_rad: float) -> float:
    """Normalize an undirected line angle to ``(-pi/2, pi/2]``."""
    if not math.isfinite(angle_rad):
        raise CanGeometryError("axis yaw must be finite")
    wrapped = (angle_rad + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.pi / 2.0 if wrapped <= -math.pi / 2.0 else wrapped


def undirected_axis_error(a_rad: float, b_rad: float) -> float:
    """Return the smallest absolute error between two undirected axes."""
    return abs(wrap_undirected_axis(a_rad - b_rad))
