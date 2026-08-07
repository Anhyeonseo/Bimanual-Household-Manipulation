"""Pure validation for the STM32 arm and gripper Action adapter."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math


MIN_DURATION_NS = 300_000_000
MAX_DURATION_NS = 2_000_000_000


class GoalValidationError(ValueError):
    """Raised before an invalid goal can reach the serial transport."""


@dataclass(frozen=True, slots=True)
class TrajectoryPointData:
    positions: tuple[float, ...]
    time_from_start_ns: int
    velocities: tuple[float, ...] = ()
    accelerations: tuple[float, ...] = ()
    effort: tuple[float, ...] = ()


@dataclass(frozen=True, slots=True)
class ValidatedTrajectory:
    ordered_positions: tuple[float, ...]
    duration_ms: int


@dataclass(frozen=True, slots=True)
class ValidatedBufferedTrajectory:
    start_positions: tuple[float, ...]
    ordered_points: tuple[tuple[float, ...], ...]
    time_from_start_ns: tuple[int, ...]
    segment_velocities_rad_s: tuple[tuple[float, ...], ...]
    duration_ms: int


@dataclass(frozen=True, slots=True)
class GripperCommandData:
    positions: tuple[float, ...]
    joint_names: tuple[str, ...] = ()
    velocities: tuple[float, ...] = ()
    efforts: tuple[float, ...] = ()


def _validate_expected_contract(
    expected_joint_names: Sequence[str],
    limits: Mapping[str, tuple[float, float]],
) -> tuple[str, ...]:
    expected = tuple(expected_joint_names)
    if not expected:
        raise GoalValidationError("expected joint contract is empty")
    if len(set(expected)) != len(expected):
        raise GoalValidationError("expected joint contract contains duplicates")
    if set(limits) != set(expected):
        raise GoalValidationError("safe limits do not match expected joints")
    for name in expected:
        lower, upper = limits[name]
        if (
            not math.isfinite(lower)
            or not math.isfinite(upper)
            or lower > upper
        ):
            raise GoalValidationError(f"{name} has invalid safe limits")
    return expected


def _validate_goal_joint_names(
    joint_names: Sequence[str],
    expected: tuple[str, ...],
) -> tuple[str, ...]:
    names = tuple(joint_names)
    if len(set(names)) != len(names):
        raise GoalValidationError("goal contains duplicate joint names")
    if len(names) != len(expected) or set(names) != set(expected):
        raise GoalValidationError("goal joint names do not match expected joints")
    return names


def _validate_strictly_increasing_times(
    points: Sequence[TrajectoryPointData],
) -> None:
    previous = -1
    for point in points:
        value = point.time_from_start_ns
        if isinstance(value, bool) or not isinstance(value, int):
            raise GoalValidationError("time_from_start must be integer nanoseconds")
        if value <= previous:
            raise GoalValidationError("trajectory times must be strictly increasing")
        previous = value


# 관절 한계 비교의 부동소수점 여유.
#
# 한계는 raw count 에서 유도된다(`(zero_raw - minimum_raw) * 2pi/4096` 등).
# 계획기가 한계 자세를 정확히 겨누면 그 값이 rad 로 왕복하면서 마지막 비트가
# 어긋나 한계 밖으로 밀린다. 2026-08-06 A4 스윕에서 MoveIt 이 WRIST_FLEX 를
# raw 1194(=한계)에 정확히 놓았고, 초과량 4.441e-16 rad 로 계획이 거부됐다.
# 물리적으로는 0.000000000 raw 다 — 존재하지 않는 위반이었다.
#
# 이 여유는 그 표현 오차만 흡수한다. 관측된 오차의 200만 배 위이고
# 한 raw count 의 150만 분의 1 아래다. 명령 해상도가 raw 이므로 이 밴드
# 안의 차이는 애초에 서보에 전달될 수 없다.
JOINT_LIMIT_EPSILON_RAD = 1.0e-9


def _validate_position(name: str, value: float, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GoalValidationError(f"{name} position is not a finite number")
    position = float(value)
    if not math.isfinite(position):
        raise GoalValidationError(f"{name} position is not finite")
    if not (
        lower - JOINT_LIMIT_EPSILON_RAD
        <= position
        <= upper + JOINT_LIMIT_EPSILON_RAD
    ):
        raise GoalValidationError(
            f"{name} position {position} is outside safe range {lower}..{upper}"
        )
    # 여유는 비교에만 쓰고 값은 한계 안으로 되돌린다. 여유를 통과한 값이
    # 그대로 하류로 흘러 raw 변환에서 다시 거부되면 안 된다.
    return min(max(position, lower), upper)


def validate_single_point_trajectory(
    joint_names: Sequence[str],
    points: Sequence[TrajectoryPointData],
    expected_joint_names: Sequence[str],
    limits: Mapping[str, tuple[float, float]],
) -> ValidatedTrajectory:
    """Validate and reorder a single arm point without touching hardware."""

    expected = _validate_expected_contract(expected_joint_names, limits)
    names = _validate_goal_joint_names(joint_names, expected)
    if not points:
        raise GoalValidationError("trajectory has no points")
    _validate_strictly_increasing_times(points)
    if len(points) != 1:
        raise GoalValidationError("STM32 milestone requires exactly one point")

    point = points[0]
    if len(point.positions) != len(names):
        raise GoalValidationError("trajectory point position count is invalid")
    if point.velocities or point.accelerations or point.effort:
        raise GoalValidationError(
            "velocity, acceleration, and effort fields are not supported"
        )
    if not MIN_DURATION_NS <= point.time_from_start_ns <= MAX_DURATION_NS:
        raise GoalValidationError("duration must be within 300..2000 ms")

    by_name = dict(zip(names, point.positions, strict=True))
    ordered = tuple(
        _validate_position(name, by_name[name], *limits[name])
        for name in expected
    )
    duration_ms = (point.time_from_start_ns + 999_999) // 1_000_000
    return ValidatedTrajectory(ordered, duration_ms)


def _validate_positive_limit_map(
    expected: tuple[str, ...],
    limits: Mapping[str, float],
    field: str,
) -> tuple[float, ...]:
    if set(limits) != set(expected):
        raise GoalValidationError(f"{field} do not match expected joints")
    values: list[float] = []
    for name in expected:
        if isinstance(limits[name], bool) or not isinstance(
            limits[name],
            (int, float),
        ):
            raise GoalValidationError(f"{name} has invalid {field}")
        value = float(limits[name])
        if not math.isfinite(value) or value <= 0.0:
            raise GoalValidationError(f"{name} has invalid {field}")
        values.append(value)
    return tuple(values)


def validate_buffered_trajectory(
    joint_names: Sequence[str],
    points: Sequence[TrajectoryPointData],
    expected_joint_names: Sequence[str],
    position_limits: Mapping[str, tuple[float, float]],
    start_positions: Sequence[float],
    velocity_limits_rad_s: Mapping[str, float],
    acceleration_limits_rad_s2: Mapping[str, float],
    *,
    start_tolerance_rad: float | Mapping[str, float],
) -> ValidatedBufferedTrajectory:
    """Validate a position-only multi-point path without enabling execution."""

    expected = _validate_expected_contract(expected_joint_names, position_limits)
    names = _validate_goal_joint_names(joint_names, expected)
    if len(points) < 2:
        raise GoalValidationError("buffered trajectory requires at least two points")
    _validate_strictly_increasing_times(points)
    if points[-1].time_from_start_ns <= 0:
        raise GoalValidationError("buffered trajectory duration must be positive")
    if isinstance(start_tolerance_rad, Mapping):
        if set(start_tolerance_rad) != set(expected):
            raise GoalValidationError(
                "start tolerances do not match expected joints"
            )
        values = tuple(start_tolerance_rad[name] for name in expected)
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < 0.0
            for value in values
        ):
            raise GoalValidationError(
                "start tolerances must be finite and non-negative"
            )
        start_tolerances = tuple(float(value) for value in values)
    else:
        if (
            isinstance(start_tolerance_rad, bool)
            or not isinstance(start_tolerance_rad, (int, float))
            or not math.isfinite(float(start_tolerance_rad))
            or start_tolerance_rad < 0.0
        ):
            raise GoalValidationError(
                "start tolerance must be finite and non-negative"
            )
        start_tolerances = tuple(
            float(start_tolerance_rad) for _ in expected
        )

    start = tuple(start_positions)
    if len(start) != len(expected):
        raise GoalValidationError("fresh start position count is invalid")
    start = tuple(
        _validate_position(name, value, *position_limits[name])
        for name, value in zip(expected, start, strict=True)
    )
    velocity_limits = _validate_positive_limit_map(
        expected,
        velocity_limits_rad_s,
        "velocity limits",
    )
    acceleration_limits = _validate_positive_limit_map(
        expected,
        acceleration_limits_rad_s2,
        "acceleration limits",
    )

    ordered_points: list[tuple[float, ...]] = []
    times: list[int] = []
    for point in points:
        if len(point.positions) != len(names):
            raise GoalValidationError("trajectory point position count is invalid")
        if point.effort:
            raise GoalValidationError(
                "buffered trajectory does not accept effort fields"
            )
        for field, values, limits_for_field in (
            ("velocity", point.velocities, velocity_limits),
            ("acceleration", point.accelerations, acceleration_limits),
        ):
            if not values:
                continue
            if len(values) != len(names):
                raise GoalValidationError(
                    f"trajectory point {field} count is invalid"
                )
            by_name = dict(zip(names, values, strict=True))
            for name, limit in zip(
                expected,
                limits_for_field,
                strict=True,
            ):
                value = by_name[name]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    or abs(float(value)) > limit + 1.0e-12
                ):
                    raise GoalValidationError(
                        f"{name} declared {field} exceeds {limit}"
                    )
        by_name = dict(zip(names, point.positions, strict=True))
        ordered_points.append(
            tuple(
                _validate_position(name, by_name[name], *position_limits[name])
                for name in expected
            )
        )
        times.append(point.time_from_start_ns)

    if times[0] == 0:
        outside_start_tolerance = [
            name
            for name, value, actual, tolerance in zip(
                expected,
                ordered_points[0],
                start,
                start_tolerances,
                strict=True,
            )
            if abs(value - actual) > tolerance
        ]
        if outside_start_tolerance:
            raise GoalValidationError(
                "zero-time trajectory point exceeds fresh start tolerance: "
                + ",".join(outside_start_tolerance)
            )

    previous_positions = start
    previous_time_ns = 0
    previous_velocity: tuple[float, ...] | None = None
    segment_velocities: list[tuple[float, ...]] = []
    for positions, time_ns in zip(ordered_points, times, strict=True):
        if time_ns == previous_time_ns:
            previous_positions = positions
            continue
        duration_s = (time_ns - previous_time_ns) / 1_000_000_000.0
        velocities = tuple(
            (position - previous) / duration_s
            for position, previous in zip(
                positions,
                previous_positions,
                strict=True,
            )
        )
        for name, velocity, limit in zip(
            expected,
            velocities,
            velocity_limits,
            strict=True,
        ):
            if abs(velocity) > limit + 1.0e-12:
                raise GoalValidationError(
                    f"{name} segment velocity exceeds {limit} rad/s"
                )
        reference_velocity = previous_velocity or tuple(0.0 for _ in expected)
        for name, velocity, previous, limit in zip(
            expected,
            velocities,
            reference_velocity,
            acceleration_limits,
            strict=True,
        ):
            acceleration = abs(velocity - previous) / duration_s
            if acceleration > limit + 1.0e-12:
                raise GoalValidationError(
                    f"{name} segment acceleration exceeds {limit} rad/s^2"
                )
        segment_velocities.append(velocities)
        previous_positions = positions
        previous_time_ns = time_ns
        previous_velocity = velocities

    rounded_duration_ms = (
        (times[-1] + 19_999_999) // 20_000_000
    ) * 20
    if rounded_duration_ms < 300:
        raise GoalValidationError(
            "buffered trajectory duration must provide at least 300 ms"
        )

    return ValidatedBufferedTrajectory(
        start_positions=start,
        ordered_points=tuple(ordered_points),
        time_from_start_ns=tuple(times),
        segment_velocities_rad_s=tuple(segment_velocities),
        duration_ms=rounded_duration_ms,
    )


def interpolate_buffered_trajectory(
    trajectory: ValidatedBufferedTrajectory,
    elapsed_ns: int,
) -> tuple[float, ...]:
    """Linearly interpolate one already validated trajectory for mock tests."""

    if isinstance(elapsed_ns, bool) or not isinstance(elapsed_ns, int):
        raise GoalValidationError("elapsed time must be integer nanoseconds")
    if elapsed_ns < 0:
        raise GoalValidationError("elapsed time must be non-negative")
    if elapsed_ns == 0:
        return trajectory.start_positions
    if elapsed_ns >= trajectory.time_from_start_ns[-1]:
        return trajectory.ordered_points[-1]

    previous_time = 0
    previous_positions = trajectory.start_positions
    for time_ns, positions in zip(
        trajectory.time_from_start_ns,
        trajectory.ordered_points,
        strict=True,
    ):
        if elapsed_ns == time_ns:
            return positions
        if elapsed_ns < time_ns:
            span = time_ns - previous_time
            ratio = (elapsed_ns - previous_time) / span
            return tuple(
                start + ratio * (end - start)
                for start, end in zip(
                    previous_positions,
                    positions,
                    strict=True,
                )
            )
        previous_time = time_ns
        previous_positions = positions
    return trajectory.ordered_points[-1]


def validate_gripper_command(
    command: GripperCommandData,
    expected_joint_name: str,
    safe_limit: tuple[float, float],
) -> float:
    """Validate a hardware gripper command and return its project-radian target."""

    lower, upper = safe_limit
    if (
        not math.isfinite(lower)
        or not math.isfinite(upper)
        or lower > upper
    ):
        raise GoalValidationError("gripper has invalid safe limits")
    if command.velocities or command.efforts:
        raise GoalValidationError("gripper velocity and effort are not supported")

    names = tuple(command.joint_names)
    if names:
        if len(set(names)) != len(names):
            raise GoalValidationError("gripper command contains duplicate joint names")
        if names != (expected_joint_name,):
            raise GoalValidationError("gripper command joint name is invalid")
    if len(command.positions) != 1:
        raise GoalValidationError("gripper command requires exactly one position")
    return _validate_position(
        expected_joint_name,
        command.positions[0],
        lower,
        upper,
    )
