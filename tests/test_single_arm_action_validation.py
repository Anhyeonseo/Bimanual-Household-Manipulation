import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.action_validation import (  # noqa: E402
    GoalValidationError,
    GripperCommandData,
    TrajectoryPointData,
    validate_gripper_command,
    validate_single_point_trajectory,
)
from single_arm_bridge import action_validation  # noqa: E402
from single_arm_bridge.calibration import load_calibration  # noqa: E402


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


class SingleArmActionValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.calibration = load_calibration(CALIBRATION_PATH)
        cls.all_joints = tuple(cls.calibration.ros_joint_names)
        cls.arm_joints = cls.all_joints[:5]
        cls.gripper_joint = cls.all_joints[5]
        cls.arm_limits = {
            name: cls.calibration.ros_radian_limits[name]
            for name in cls.arm_joints
        }
        cls.gripper_limit = cls.calibration.ros_radian_limits[
            cls.gripper_joint
        ]

    @staticmethod
    def point(
        positions,
        time_from_start_ns=1_000_000_000,
        velocities=(),
        accelerations=(),
        effort=(),
    ) -> TrajectoryPointData:
        return TrajectoryPointData(
            tuple(positions),
            time_from_start_ns,
            tuple(velocities),
            tuple(accelerations),
            tuple(effort),
        )

    def validate_arm(self, names, points):
        return validate_single_point_trajectory(
            names,
            points,
            self.arm_joints,
            self.arm_limits,
        )

    def test_calibration_exposes_verified_project_limits(self) -> None:
        # 2026-08-06: q0(raw 2048)가 BASE 하한이자 WRIST_FLEX 상한이라
        # 복귀할 때마다 계약을 벗어났다. 양쪽에 60 raw 여유를 넣었다.
        self.assertAlmostEqual(
            self.arm_limits["left_base_joint"][0],
            -60 * 2.0 * math.pi / 4096.0,
        )
        self.assertAlmostEqual(
            self.arm_limits["left_base_joint"][1],
            562 * 2.0 * math.pi / 4096.0,
        )
        self.assertAlmostEqual(self.gripper_limit[0], 0.0)
        self.assertAlmostEqual(
            self.gripper_limit[1],
            182 * 2.0 * math.pi / 4096.0,
        )

    def test_feedback_recovery_envelope_matches_firmware_margin(self) -> None:
        recoverable_raw = (2070, 1983, 2041, 2131, 2080, 1965)
        recoverable = tuple(
            self.calibration.raw_feedback_to_radians(recoverable_raw)
        )
        self.assertEqual(
            self.calibration.validate_feedback_recovery_envelope(recoverable),
            {
                "left_shoulder_joint": 5,
                "left_wrist_flex_joint": 23,
            },
        )

        outside_raw = list(recoverable_raw)
        outside_raw[1] = 1988 - 41
        outside = tuple(
            self.calibration.raw_feedback_to_radians(tuple(outside_raw))
        )
        with self.assertRaisesRegex(ValueError, "outside recovery range"):
            self.calibration.validate_feedback_recovery_envelope(outside)

    def test_valid_arm_goal_is_reordered_to_project_contract(self) -> None:
        targets = {
            name: 0.01 * (index + 1)
            for index, name in enumerate(self.arm_joints)
        }
        names = tuple(reversed(self.arm_joints))
        positions = tuple(targets[name] for name in names)
        validated = self.validate_arm(names, [self.point(positions)])
        self.assertEqual(
            validated.ordered_positions,
            tuple(targets[name] for name in self.arm_joints),
        )
        self.assertEqual(validated.duration_ms, 1000)

    def test_duration_rounds_up_without_shortening_request(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        validated = self.validate_arm(
            self.arm_joints,
            [self.point(positions, time_from_start_ns=300_000_001)],
        )
        self.assertEqual(validated.duration_ms, 301)

    def test_arm_goal_rejects_non_finite_positions(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                positions = [0.0] * len(self.arm_joints)
                positions[0] = value
                with self.assertRaisesRegex(
                    GoalValidationError,
                    "not finite",
                ):
                    self.validate_arm(
                        self.arm_joints,
                        [self.point(positions)],
                    )

    def test_arm_goal_rejects_missing_unknown_and_duplicate_names(self) -> None:
        valid_positions = [0.0] * len(self.arm_joints)
        invalid_names = {
            "missing": self.arm_joints[:-1],
            "unknown": ("unknown_joint",) + self.arm_joints[1:],
            "duplicate": (self.arm_joints[0],) + self.arm_joints[:-1],
        }
        for case, names in invalid_names.items():
            with self.subTest(case=case):
                with self.assertRaises(GoalValidationError):
                    self.validate_arm(names, [self.point(valid_positions)])

    def test_arm_goal_rejects_empty_and_incomplete_trajectory(self) -> None:
        with self.assertRaisesRegex(GoalValidationError, "no points"):
            self.validate_arm(self.arm_joints, [])
        with self.assertRaisesRegex(GoalValidationError, "position count"):
            self.validate_arm(
                self.arm_joints,
                [self.point([0.0] * (len(self.arm_joints) - 1))],
            )

    def test_arm_goal_rejects_multiple_points(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        with self.assertRaisesRegex(GoalValidationError, "exactly one point"):
            self.validate_arm(
                self.arm_joints,
                [
                    self.point(positions, 500_000_000),
                    self.point(positions, 1_000_000_000),
                ],
            )

    def test_arm_goal_rejects_non_monotonic_timestamps(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        with self.assertRaisesRegex(GoalValidationError, "strictly increasing"):
            self.validate_arm(
                self.arm_joints,
                [
                    self.point(positions, 1_000_000_000),
                    self.point(positions, 500_000_000),
                ],
            )

    def test_arm_goal_rejects_duration_outside_protocol_range(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        for duration in (299_999_999, 2_000_000_001):
            with self.subTest(duration=duration):
                with self.assertRaisesRegex(GoalValidationError, "300..2000"):
                    self.validate_arm(
                        self.arm_joints,
                        [self.point(positions, duration)],
                    )

    def test_arm_goal_rejects_values_outside_safe_range_without_clamp(self) -> None:
        for joint_index, name in enumerate(self.arm_joints):
            lower, upper = self.arm_limits[name]
            for value in (lower - 1.0e-6, upper + 1.0e-6):
                with self.subTest(name=name, value=value):
                    positions = [0.0] * len(self.arm_joints)
                    positions[joint_index] = value
                    with self.assertRaisesRegex(
                        GoalValidationError,
                        "outside safe range",
                    ):
                        self.validate_arm(
                            self.arm_joints,
                            [self.point(positions)],
                        )

    def test_arm_goal_rejects_unsupported_dynamic_fields(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        with self.assertRaisesRegex(GoalValidationError, "not supported"):
            self.validate_arm(
                self.arm_joints,
                [self.point(positions, velocities=[0.0] * len(positions))],
            )

    def test_gripper_accepts_empty_or_exact_joint_name(self) -> None:
        for names in ((), (self.gripper_joint,)):
            with self.subTest(names=names):
                value = validate_gripper_command(
                    GripperCommandData((0.1,), names),
                    self.gripper_joint,
                    self.gripper_limit,
                )
                self.assertEqual(value, 0.1)

    def test_gripper_rejects_simulation_open_in_hardware_mode(self) -> None:
        with self.assertRaisesRegex(GoalValidationError, "outside safe range"):
            validate_gripper_command(
                GripperCommandData((1.91986,), (self.gripper_joint,)),
                self.gripper_joint,
                self.gripper_limit,
            )

    def test_gripper_rejects_invalid_name_count_and_number(self) -> None:
        invalid_commands = (
            GripperCommandData((), ()),
            GripperCommandData((0.0, 0.1), ()),
            GripperCommandData((0.1,), ("unknown_joint",)),
            GripperCommandData((math.nan,), (self.gripper_joint,)),
        )
        for command in invalid_commands:
            with self.subTest(command=command):
                with self.assertRaises(GoalValidationError):
                    validate_gripper_command(
                        command,
                        self.gripper_joint,
                        self.gripper_limit,
                    )

    def test_gripper_rejects_unsupported_velocity_and_effort(self) -> None:
        for command in (
            GripperCommandData((0.1,), velocities=(0.1,)),
            GripperCommandData((0.1,), efforts=(1.0,)),
        ):
            with self.subTest(command=command):
                with self.assertRaisesRegex(GoalValidationError, "not supported"):
                    validate_gripper_command(
                        command,
                        self.gripper_joint,
                        self.gripper_limit,
                    )


if __name__ == "__main__":
    unittest.main()


class JointLimitEpsilonTests(unittest.TestCase):
    """한계 자세를 정확히 겨눈 계획이 표현 오차로 거부되면 안 된다.

    2026-08-06 A4 스윕: MoveIt 이 WRIST_FLEX 를 raw 1194(=한계)에 정확히
    놓았는데, rad 왕복의 마지막 비트가 값을 한계 밖으로 밀어 계획이
    거부됐다. 초과량 4.441e-16 rad = 0.000000000 raw 로 물리적으로는
    존재하지 않는 위반이었다.
    """

    def setUp(self) -> None:
        self.calibration = load_calibration(CALIBRATION_PATH)
        self.limits = {
            name: self.calibration.ros_radian_limits[name]
            for name in self.calibration.ros_joint_names[:5]
        }

    def test_a_target_exactly_at_the_limit_is_accepted(self) -> None:
        name = "left_wrist_flex_joint"
        _, upper = self.limits[name]
        # 계획기가 한계를 정확히 겨눈 뒤 왕복하며 마지막 비트가 어긋난 값.
        nudged = math.nextafter(upper, math.inf)
        self.assertGreater(nudged, upper)
        accepted = action_validation._validate_position(
            name, nudged, *self.limits[name]
        )
        # 값 자체는 한계 안으로 되돌아와야 한다. 그래야 하류 raw 변환이
        # 같은 값을 다시 거부하지 않는다.
        self.assertLessEqual(accepted, upper)

    def test_the_epsilon_is_far_below_one_raw_count(self) -> None:
        raw_step = 2.0 * math.pi / 4096.0
        self.assertLess(
            action_validation.JOINT_LIMIT_EPSILON_RAD, raw_step / 1000.0
        )

    def test_a_real_violation_is_still_refused(self) -> None:
        name = "left_wrist_flex_joint"
        _, upper = self.limits[name]
        raw_step = 2.0 * math.pi / 4096.0
        with self.assertRaises(GoalValidationError):
            action_validation._validate_position(
                name, upper + raw_step, *self.limits[name]
            )
