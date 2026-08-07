"""여러 pilot 실행의 leg 진단을 이어 붙이는 계층의 계약.

pilot 은 실패하면 그 자리에서 멈춘다(의도적, 자동 재시도 없음). 그래서
`grasp` 처럼 회당 한 번만 나오는 leg 은 호출 1회로는 표본이 부족하다. 이
도구가 여러 evidence 파일을 세션 순서로 이어 붙여 추세를 다시 낸다.

ROS 없이, 직렬 포트 없이 검증한다.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import aggregate_leg_telemetry as AGG  # noqa: E402


def leg(ordinal: int, tag: str, precompute: float, lead: int) -> dict:
    return {
        "ordinal": ordinal,
        "run": ordinal,
        "tag": tag,
        "ok": True,
        "precompute_ms": precompute,
        "first_sample_lead_ms": lead,
        "sample_count": 1301,
    }


def write_evidence(path: Path, legs: list[dict]) -> None:
    path.write_text(
        json.dumps({"legs": legs}, indent=2, sort_keys=True), encoding="utf-8"
    )


def test_legs_are_renumbered_across_files_in_file_order(tmp_path: Path) -> None:
    """원래 파일 안의 ordinal 은 버려진다 — 세션 전체 순번으로 다시 매긴다."""
    first = tmp_path / "run1.json"
    second = tmp_path / "run2.json"
    write_evidence(first, [leg(1, "pregrasp", 100.0, 140), leg(2, "grasp", 110.0, 130)])
    write_evidence(second, [leg(1, "pregrasp", 105.0, 138), leg(2, "grasp", 150.0, 90)])

    legs = AGG.load_legs([first, second])
    assert [item["ordinal"] for item in legs] == [1, 2, 3, 4]
    assert [item["tag"] for item in legs] == [
        "pregrasp", "grasp", "pregrasp", "grasp",
    ]


def test_each_leg_remembers_which_file_it_came_from(tmp_path: Path) -> None:
    first = tmp_path / "run1.json"
    second = tmp_path / "run2.json"
    write_evidence(first, [leg(1, "grasp", 100.0, 140)])
    write_evidence(second, [leg(1, "grasp", 150.0, 90)])

    legs = AGG.load_legs([first, second])
    assert legs[0]["source_file"] == "run1.json"
    assert legs[1]["source_file"] == "run2.json"


def test_an_evidence_file_with_no_legs_is_rejected(tmp_path: Path) -> None:
    empty = tmp_path / "empty.json"
    write_evidence(empty, [])
    other = tmp_path / "other.json"
    write_evidence(other, [leg(1, "grasp", 100.0, 140)])

    with pytest.raises(ValueError, match="leg 진단이 없다"):
        AGG.load_legs([empty, other])


def test_the_combined_trend_reveals_a_pattern_no_single_file_shows(
    tmp_path: Path,
) -> None:
    """단일 파일엔 grasp leg 이 하나씩뿐이라 기울기가 안 난다.

    합치면 4개가 되어 처음으로 기울기가 나온다 — 이게 이 도구의 존재
    이유다.
    """
    files = []
    for index, (precompute, lead) in enumerate(
        [(400.0, 140), (410.0, 135), (500.0, 100), (600.0, 70)], start=1
    ):
        path = tmp_path / f"run{index}.json"
        write_evidence(path, [leg(1, "grasp", precompute, lead)])
        files.append(path)

    legs = AGG.load_legs(files)
    from buffered_leg_telemetry import summarise_leg_telemetry

    summary = summarise_leg_telemetry(legs)
    assert summary["by_tag"]["grasp"]["leg_count"] == 4
    assert summary["by_tag"]["grasp"]["precompute_ms"]["slope_per_leg"] > 0


def test_the_cli_requires_at_least_two_files() -> None:
    source = (ROOT / "tools" / "aggregate_leg_telemetry.py").read_text(
        encoding="utf-8"
    )
    assert "합칠 evidence 파일이 2개 이상 필요하다" in source


def test_the_tool_never_touches_hardware() -> None:
    source = (ROOT / "tools" / "aggregate_leg_telemetry.py").read_text(
        encoding="utf-8"
    )
    for forbidden in ("serial", "rclpy", "subprocess", "ActuatorTransport"):
        assert forbidden not in source


def test_the_output_records_which_files_were_combined(tmp_path: Path) -> None:
    first = tmp_path / "run1.json"
    second = tmp_path / "run2.json"
    write_evidence(first, [leg(1, "grasp", 100.0, 140)])
    write_evidence(second, [leg(1, "grasp", 150.0, 90)])
    output = tmp_path / "aggregate.json"

    sys.argv = [
        "aggregate_leg_telemetry.py",
        str(first), str(second),
        "--output", str(output),
    ]
    assert AGG.main() == 0
    document = json.loads(output.read_text(encoding="utf-8"))
    assert document["leg_count"] == 2
    assert len(document["source_files_sha256"]) == 2
    assert document["status"] == "LEG_TELEMETRY_AGGREGATE_PASS"
