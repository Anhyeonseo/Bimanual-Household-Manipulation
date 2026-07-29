from dataclasses import dataclass
from math import radians


@dataclass(frozen=True)
class JointSpec:
    project_name: str
    isaac_name: str
    sign: float
    offset: float
    lower: float
    upper: float

    def project_to_isaac_position(self, value: float) -> float:
        return (value - self.offset) / self.sign

    def isaac_to_project_position(self, value: float) -> float:
        return self.sign * value + self.offset

    def project_to_isaac_rate(self, value: float) -> float:
        return value / self.sign

    def isaac_to_project_rate(self, value: float) -> float:
        return self.sign * value

    def contains(self, value: float) -> bool:
        return self.lower <= value <= self.upper


JOINT_SPECS = (
    JointSpec(
        "left_base_joint",
        "shoulder_pan",
        -1.0,
        0.0,
        -1.91986,
        1.91986,
    ),
    JointSpec(
        "left_shoulder_joint",
        "shoulder_lift",
        -1.0,
        0.0,
        -3.3161263267949,
        0.174533673205103,
    ),
    JointSpec(
        "left_elbow_joint",
        "elbow_flex",
        -1.0,
        0.0,
        -0.730068911403119,
        2.64993108859688,
    ),
    JointSpec(
        "left_wrist_flex_joint",
        "wrist_flex",
        -1.0,
        0.0,
        -0.654495680103261,
        2.66162431989674,
    ),
    JointSpec(
        "left_wrist_roll_joint",
        "wrist_roll",
        -1.0,
        0.0,
        -1.2704136732051,
        4.3146463267949,
    ),
    JointSpec(
        "left_gripper_joint",
        "gripper",
        1.0,
        radians(10.0),
        0.0,
        1.91986,
    ),
)

BY_PROJECT_NAME = {spec.project_name: spec for spec in JOINT_SPECS}
BY_ISAAC_NAME = {spec.isaac_name: spec for spec in JOINT_SPECS}

PROJECT_JOINTS = tuple(spec.project_name for spec in JOINT_SPECS)
ISAAC_JOINTS = tuple(spec.isaac_name for spec in JOINT_SPECS)
ARM_JOINTS = PROJECT_JOINTS[:5]
GRIPPER_JOINT = PROJECT_JOINTS[5]
