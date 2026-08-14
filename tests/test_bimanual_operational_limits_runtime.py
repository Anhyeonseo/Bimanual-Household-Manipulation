"""One runtime table owns the complete operator-verified bimanual envelope."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "config/bimanual_operational_limits.json"
INSTALLED_MANIFEST = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/config/bimanual_operational_limits.json"
)
LIMIT_SOURCE = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Src/bimanual_operational_limits.c"
)
INCLUDE = ROOT / "firmware/stm32_g474_single_arm/Core/Inc"
ACTUATOR_INCLUDE = ROOT / "firmware/stm32_actuator/include"
CMAKE = (ROOT / "firmware/stm32_g474_single_arm/CMakeLists.txt").read_text()
CONFIG = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text()
IDENTITY = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
).read_text()
BRIDGE = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bridge_node.py"
).read_text()

EXPECTED_SHA256 = (
    "436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263"
)


def test_manifest_is_canonical_and_contains_all_reviewed_ranges() -> None:
    canonical = MANIFEST.read_bytes()
    assert canonical == INSTALLED_MANIFEST.read_bytes()
    assert hashlib.sha256(canonical).hexdigest() == EXPECTED_SHA256
    document = json.loads(canonical)
    assert document["operator_approved"] is True
    assert document["firmware_limit_authorized"] is True
    assert document["general_trajectory_output_available"] is False
    assert document["arms"]["left"]["base"]["minimum_unwrapped_raw"] == 983
    assert document["arms"]["left"]["gripper"]["maximum_unwrapped_raw"] == 3257
    assert document["arms"]["right"]["shoulder"] == {
        "coordinate": "unwrapped_raw",
        "minimum_unwrapped_raw": 1859,
        "maximum_unwrapped_raw": 4188,
        "minimum_urad": -289922,
        "maximum_urad": 3282719,
    }
    assert document["arms"]["right"]["elbow"]["maximum_unwrapped_raw"] == 2523
    assert document["arms"]["right"]["gripper"]["minimum_unwrapped_raw"] == 1907
    assert document["arms"]["right"]["gripper"]["maximum_unwrapped_raw"] == 3299


def test_manifest_matches_all_twelve_compiled_limits() -> None:
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    source = LIMIT_SOURCE.read_text(encoding="utf-8")
    table = source.split("operational_limits", 1)[1].split(
        r"/\* Archived table", 1
    )[0]
    compiled = [
        (int(direction), int(lower), int(upper), int(minimum), int(maximum))
        for direction, lower, upper, minimum, maximum in re.findall(
            r"\{2048U,\s*(-?1),\s*\{\s*(\d+),\s*(\d+)\},"
            r"\s*\{\s*(-?\d+),\s*(-?\d+)\}\}",
            table,
        )
    ]
    expected = []
    directions = (1, 1, -1, -1, 1, -1)
    names = (
        "base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"
    )
    for arm in ("left", "right"):
        for direction, name in zip(directions, names, strict=True):
            limit = document["arms"][arm][name]
            expected.append(
                (
                    direction,
                    limit["minimum_unwrapped_raw"],
                    limit["maximum_unwrapped_raw"],
                    limit["minimum_urad"],
                    limit["maximum_urad"],
                )
            )
    assert compiled == expected


def test_compiled_limits_are_inclusive_wrap_aware_and_fail_closed(
    tmp_path: Path,
) -> None:
    harness = tmp_path / "bimanual_limits.c"
    harness.write_text(
        r'''
#include "bimanual_operational_limits.h"
#include <stdint.h>

int main(void)
{
    uint16_t target = 0U;
    int32_t unwrapped = 0;
    actuator_v2_joint_limit_t limits[12];
    int32_t goals[12] = {0};
    uint16_t left_raw[6] = {0};
    uint16_t right_raw[6] = {0};
    uint8_t failed_joint = 0U;

    if (!BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_LEFT, 0U, 983)) return 1;
    if (BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_LEFT, 0U, 982)) return 2;
    if (!BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_RIGHT, 2U, 2523)) return 3;
    if (BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_RIGHT, 2U, 2524)) return 4;
    if (!BimanualOperationalLimits_UnwrapModuloRaw(
            BIMANUAL_ARM_RIGHT, 1U, 50U, &unwrapped)) return 5;
    if (unwrapped != 4146) return 6;
    if (!BimanualOperationalLimits_StepModuloRaw(
            BIMANUAL_ARM_RIGHT, 1U, 4090U, 20, &target)) return 7;
    if (target != 14U) return 8;
    if (BimanualOperationalLimits_StepModuloRaw(
            BIMANUAL_ARM_RIGHT, 1U, 92U, 1, &target)) return 9;
    if (!BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_LEFT, 5U, 3257)) return 10;
    if (!BimanualOperationalLimits_ContainsUnwrappedRaw(
            BIMANUAL_ARM_RIGHT, 5U, 1907)) return 11;
    if (BimanualOperationalLimits_Get(BIMANUAL_ARM_COUNT, 0U) != 0) return 12;

    BimanualOperationalLimits_LoadExecutorLimits(limits);
    if (limits[0].minimum_urad != -1633689) return 13;
    if (limits[7].maximum_urad != 3282719) return 14;
    if (limits[11].minimum_urad != -1919010) return 15;
    goals[1] = 3281185;
    goals[2] = 2702874;
    if (BimanualOperationalLimits_MapExecutorOutput(
            goals, left_raw, right_raw, &failed_joint) !=
        ACTUATOR_BIMANUAL_GOAL_MAP_OK) return 16;
    if (left_raw[1] != 91U || left_raw[2] != 286U) return 17;
    if (failed_joint != UINT8_MAX) return 18;
    return 0;
}
''',
        encoding="utf-8",
    )
    executable = tmp_path / "bimanual_limits"
    compile_result = subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(INCLUDE),
            "-I",
            str(ACTUATOR_INCLUDE),
            str(LIMIT_SOURCE),
            str(ROOT / "firmware/stm32_actuator/src/bimanual_goal_map.c"),
            str(harness),
            "-o",
            str(executable),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, compile_result.stderr
    run_result = subprocess.run([str(executable)], check=False)
    assert run_result.returncode == 0


def test_candidate_and_bridge_use_one_general_identity() -> None:
    assert "BIMANUAL_OPERATIONAL_LIMITS_CANDIDATE" in CMAKE
    assert "BIMANUAL_DISPATCH_REFACTOR_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024400UL" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024500UL" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x607FFFFFUL" in CMAKE
    assert "HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U" in CMAKE
    assert "HOST_BINARY_CAPABILITIES=0x207FFFFFUL" in CMAKE
    assert "HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD=1U" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "BIMANUAL_OPERATIONAL_LIMITS_FIRMWARE_VERSION = 0x00024400" in IDENTITY
    assert "BIMANUAL_DISPATCH_REFACTOR_FIRMWARE_VERSION = 0x00024500" in IDENTITY
    assert "BIMANUAL_OPERATIONAL_LIMITS_CAPABILITY = 0x20000000" in IDENTITY
    assert "BIMANUAL_DISPATCH_REFACTOR_CAPABILITY = 0x40000000" in IDENTITY
    assert "BIMANUAL_READ_ONLY_DISPATCH_REFACTOR" in BRIDGE
    assert 'self.declare_parameter("require_bimanual_operational_limits", False)' in BRIDGE
    assert '"bimanual_operational_limits_identity"' in BRIDGE
    assert EXPECTED_SHA256 in BRIDGE
    assert "firmware=0x00024400/0x00024500 requires" in BRIDGE
    assert "does not authorize " in BRIDGE


def test_temporary_elbow_recovery_branch_is_gone() -> None:
    combined = "\n".join((CMAKE, CONFIG, IDENTITY, BRIDGE, LIMIT_SOURCE.read_text()))
    for forbidden in (
        "J2_ELBOW_RECOVERY",
        "J2-R",
        "j2r_elbow",
        "right_arm_command_limits",
        "inward-only",
    ):
        assert forbidden not in combined
