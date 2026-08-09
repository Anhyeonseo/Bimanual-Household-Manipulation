"""보정 교체가 저장된 자세의 의미를 바꾸지 않았음을 기계로 증명한다.

2026-08-06 에 `BASE minimum_raw` 와 `WRIST_FLEX maximum_raw` 를 넓히면서
calibration hash 가 `0x8AD27897` 에서 `0xB32257F4` 로 바뀌었다. 그 해시는
collision-checked artifact 22개에 박혀 있고, 도구들은 불일치를 fail-closed 로
거부한다 — 옳은 동작이다.

문제는 해시가 "보정이 바뀌었다" 만 말하고 "어떻게 바뀌었는지" 는 구분하지
못한다는 것이다. 두 종류는 결과가 완전히 다르다.

  사상(zero_raw / positive_raw_direction / raw_units_per_turn)이 바뀌면
  -> 저장된 raw 가 다른 물리 자세를 뜻하게 된다. 기존 경로는 전부 무효다.

  한계(minimum_raw / maximum_raw)만 넓어지면
  -> 자세의 의미는 그대로이고, 더 좁은 옛 한계 안에서 계획된 경로는 새 한계
     안에도 그대로 들어간다. 경로는 유효하다.

이번 교체는 후자다. 그래서 artifact 의 해시 필드를 재발행했다. 이 시험이
그 재발행이 정당했음을 강제한다. 사상이 하나라도 바뀌었거나 한계가 좁아졌다면
실패하며, 그때는 재발행이 아니라 재계획이 답이다.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.joint_calibration import (  # noqa: E402
    calibration_hash,
    load_calibration,
)

AUTHORITATIVE = ROOT / "config" / "single_arm_calibration.json"
PACKAGED = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json"
)

# 사상을 이루는 필드. 하나라도 바뀌면 저장된 자세의 의미가 달라진다.
MAPPING_FIELDS = ("id", "name", "zero_raw", "positive_raw_direction")


@pytest.fixture(scope="module")
def document():
    return json.loads(AUTHORITATIVE.read_text(encoding="utf-8"))


def test_current_hash_is_the_reviewed_value(document) -> None:
    assert calibration_hash(load_calibration(AUTHORITATIVE)) == 0x2D90167E
    assert calibration_hash(load_calibration(PACKAGED)) == 0x2D90167E


def test_supersession_history_is_recorded(document) -> None:
    history = document.get("superseded_calibrations")
    assert isinstance(history, list) and history
    for entry in history:
        assert entry["calibration_hash"].startswith("0x")
        assert entry["superseded_on"]
        assert entry["reason"]
        assert len(entry["joints"]) == len(document["joints"])


def test_no_superseded_hash_equals_the_current_one(document) -> None:
    current = f"0x{calibration_hash(load_calibration(AUTHORITATIVE)):08X}"
    assert current not in {
        entry["calibration_hash"] for entry in document["superseded_calibrations"]
    }


def test_every_supersession_preserved_the_raw_to_radian_mapping(
    document,
) -> None:
    """사상이 바뀌었다면 artifact 해시 재발행은 정당하지 않다."""
    assert document["raw_units_per_turn"] == 4096
    assert document["turn_urad"] == 6283185
    for entry in document["superseded_calibrations"]:
        assert entry["raw_to_radian_mapping_changed"] is False
        for old, new in zip(entry["joints"], document["joints"], strict=True):
            for field in MAPPING_FIELDS:
                assert old[field] == new[field], (
                    f"{entry['calibration_hash']} {new['name']}.{field} "
                    "이 바뀌었다. 저장된 자세의 의미가 달라졌으므로 해시 "
                    "재발행이 아니라 재계획이 필요하다"
                )


def test_every_supersession_only_widened_the_limits(document) -> None:
    """옛 구간이 새 구간의 부분집합이어야 기존 경로가 여전히 유효하다."""
    for entry in document["superseded_calibrations"]:
        assert entry["limits_widened_only"] is True
        for old, new in zip(entry["joints"], document["joints"], strict=True):
            assert new["minimum_raw"] <= old["minimum_raw"], (
                f"{new['name']} 하한이 좁아졌다"
            )
            assert new["maximum_raw"] >= old["maximum_raw"], (
                f"{new['name']} 상한이 좁아졌다"
            )


def test_p_gain_changes_are_declared_rather_than_silent(document) -> None:
    """p_gain 은 해시에 들어가지만 자세의 의미와는 무관하다.

    그래도 조용히 바뀌면 안 된다. 바뀌었다면 그것이 이번 교체의 이유여야 한다.

    history 는 최신이 먼저 오는 순서다. 각 entry 는 "그 다음 상태"(더 최신 쪽,
    즉 배열에서 하나 앞 entry 이거나 없으면 현재 document)와만 비교한다 —
    체인이 두 단계 이상 길어지면 각 entry 가 자신이 대체된 그 순간의 변경만
    책임지고, 그 뒤에 다시 바뀐 값까지 설명할 의무는 없다.
    """
    history = document["superseded_calibrations"]
    for index, entry in enumerate(history):
        next_state = history[index - 1]["joints"] if index > 0 else document["joints"]
        for old, new in zip(entry["joints"], next_state, strict=True):
            if old["p_gain"] != new["p_gain"]:
                assert "p_gain" in entry["reason"], (
                    f"{new['name']} p_gain 이 바뀌었는데 이유에 적혀 있지 않다"
                )


def test_d_gain_changes_are_declared_rather_than_silent(document) -> None:
    """d_gain 은 p_gain 과 같은 이유로 해시에 들어간다(조용히 바뀌면 안 된다).

    d_gain 은 이번 교체에서 처음 관절별 필드가 됐다 — 그 이전 history entry 들의
    joints 스냅샷에는 d_gain 이 없다. 두 쪽 다 d_gain 을 가진 쌍(현재는 최신
    entry 뿐)만 비교한다.
    """
    history = document["superseded_calibrations"]
    for index, entry in enumerate(history):
        next_state = history[index - 1]["joints"] if index > 0 else document["joints"]
        for old, new in zip(entry["joints"], next_state, strict=True):
            if "d_gain" not in old or "d_gain" not in new:
                continue
            if old["d_gain"] != new["d_gain"]:
                assert "d_gain" in entry["reason"], (
                    f"{new['name']} d_gain 이 바뀌었는데 이유에 적혀 있지 않다"
                )


def test_q0_is_inside_every_joint_range_with_margin(document) -> None:
    """이번 교체의 목적 자체를 시험으로 고정한다.

    q0 는 전 축 raw 2048 이다. 어느 축이든 한계선 위에 있으면 관측 오차
    (post-settle 실측 6 raw)가 그대로 계약 위반이 된다.
    """
    for joint in document["joints"]:
        if joint["name"] == "GRIPPER":
            # GRIPPER 는 maximum_raw 가 URDF 상한과 같아 넓힐 수 없다.
            # 다만 운영 중 2048 에 머무르지 않으므로(명령은 1963..2009)
            # 여유가 없어도 문제가 되지 않는다.
            continue
        margin_below = joint["zero_raw"] - joint["minimum_raw"]
        margin_above = joint["maximum_raw"] - joint["zero_raw"]
        assert margin_below >= 60, f"{joint['name']} q0 하한 여유 {margin_below}"
        assert margin_above >= 60, f"{joint['name']} q0 상한 여유 {margin_above}"
