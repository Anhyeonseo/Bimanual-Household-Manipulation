"""H1 상주 세션의 joint-state freshness 계약. ROS/하드웨어 없이 검증한다."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from resident_arm_session import ResidentArmSession  # noqa: E402


def bare_session() -> ResidentArmSession:
    session = ResidentArmSession.__new__(ResidentArmSession)
    session._latest_joint_state = {}
    session._joint_state_stamps_s = {}
    return session


def test_partial_joint_state_updates_have_independent_freshness() -> None:
    session = bare_session()
    session._on_joint_state(SimpleNamespace(name=["a"], position=[1.0]))
    first_stamp = session._joint_state_stamps_s["a"]
    session._on_joint_state(SimpleNamespace(name=["b"], position=[2.0]))

    assert session._latest_joint_state == {"a": 1.0, "b": 2.0}
    assert session._joint_state_stamps_s["a"] == first_stamp
    assert session._joint_state_stamps_s["b"] >= first_stamp


def test_one_new_joint_cannot_make_an_old_vector_fresh() -> None:
    session = bare_session()
    session._latest_joint_state = {"a": 1.0, "b": 2.0}
    session._joint_state_stamps_s = {"a": 10.0, "b": 20.0}
    requested_after = 15.0

    assert not all(
        name in session._latest_joint_state
        and session._joint_state_stamps_s.get(name, 0.0) > requested_after
        for name in ("a", "b")
    )
