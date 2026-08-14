"""Fail-closed 12-bit encoder unwrapping and command modulo conversion."""

from __future__ import annotations

from dataclasses import dataclass
import math


RAW_MODULUS = 4096
HALF_TURN_RAW = RAW_MODULUS // 2
TURN_RAD = 2.0 * math.pi


class JointUnwrapError(ValueError):
    pass


def signed_modular_delta(current_raw: int, previous_raw: int) -> int:
    for name, value in (
        ("current_raw", current_raw),
        ("previous_raw", previous_raw),
    ):
        if not 0 <= value < RAW_MODULUS:
            raise JointUnwrapError(f"{name} must be within 0..4095")
    difference = current_raw - previous_raw
    if abs(difference) == HALF_TURN_RAW:
        raise JointUnwrapError("half-turn feedback delta is ambiguous")
    if difference > HALF_TURN_RAW:
        difference -= RAW_MODULUS
    elif difference < -HALF_TURN_RAW:
        difference += RAW_MODULUS
    return difference


def nearest_unwrapped_raw(
    observed_raw: int,
    reference_unwrapped_raw: int,
    maximum_reference_delta_raw: int,
) -> int:
    if not 0 <= observed_raw < RAW_MODULUS:
        raise JointUnwrapError("observed_raw must be within 0..4095")
    if not 1 <= maximum_reference_delta_raw < HALF_TURN_RAW:
        raise JointUnwrapError(
            "maximum reference delta must be within 1..2047"
        )
    approximate_turn = (reference_unwrapped_raw - observed_raw) // RAW_MODULUS
    candidates = tuple(
        observed_raw + RAW_MODULUS * (approximate_turn + offset)
        for offset in (-1, 0, 1)
    )
    distances = tuple(
        abs(candidate - reference_unwrapped_raw) for candidate in candidates
    )
    best_distance = min(distances)
    if distances.count(best_distance) != 1:
        raise JointUnwrapError("reference branch is ambiguous")
    if best_distance > maximum_reference_delta_raw:
        raise JointUnwrapError(
            "observed raw is too far from the verified branch reference"
        )
    return candidates[distances.index(best_distance)]


@dataclass(slots=True)
class JointUnwrapper:
    previous_raw: int | None = None
    unwrapped_raw: int | None = None

    @property
    def bound(self) -> bool:
        return self.previous_raw is not None and self.unwrapped_raw is not None

    def reset(self) -> None:
        self.previous_raw = None
        self.unwrapped_raw = None

    def bind(
        self,
        observed_raw: int,
        reference_unwrapped_raw: int,
        maximum_reference_delta_raw: int,
    ) -> int:
        value = nearest_unwrapped_raw(
            observed_raw,
            reference_unwrapped_raw,
            maximum_reference_delta_raw,
        )
        self.previous_raw = observed_raw
        self.unwrapped_raw = value
        return value

    def update(self, observed_raw: int) -> int:
        if not self.bound:
            raise JointUnwrapError("joint branch is not bound")
        assert self.previous_raw is not None
        assert self.unwrapped_raw is not None
        delta = signed_modular_delta(observed_raw, self.previous_raw)
        self.previous_raw = observed_raw
        self.unwrapped_raw += delta
        return self.unwrapped_raw


def unwrapped_raw_to_radians(
    unwrapped_raw: int,
    *,
    zero_raw: int,
    positive_raw_direction: int,
) -> float:
    if not 0 <= zero_raw < RAW_MODULUS:
        raise JointUnwrapError("zero_raw must be within 0..4095")
    if positive_raw_direction not in (-1, 1):
        raise JointUnwrapError("positive_raw_direction must be -1 or 1")
    return (
        (unwrapped_raw - zero_raw)
        * positive_raw_direction
        * TURN_RAD
        / RAW_MODULUS
    )


def radians_to_unwrapped_and_modulo_raw(
    position_rad: float,
    *,
    zero_raw: int,
    positive_raw_direction: int,
    minimum_unwrapped_raw: int,
    maximum_unwrapped_raw: int,
) -> tuple[int, int]:
    if not math.isfinite(position_rad):
        raise JointUnwrapError("position must be finite")
    if not 0 <= zero_raw < RAW_MODULUS:
        raise JointUnwrapError("zero_raw must be within 0..4095")
    if positive_raw_direction not in (-1, 1):
        raise JointUnwrapError("positive_raw_direction must be -1 or 1")
    if minimum_unwrapped_raw > maximum_unwrapped_raw:
        raise JointUnwrapError("unwrapped limits are reversed")
    unwrapped_raw = round(
        zero_raw
        + positive_raw_direction * position_rad * RAW_MODULUS / TURN_RAD
    )
    if not minimum_unwrapped_raw <= unwrapped_raw <= maximum_unwrapped_raw:
        raise JointUnwrapError("target is outside unwrapped joint limits")
    return unwrapped_raw, unwrapped_raw % RAW_MODULUS
