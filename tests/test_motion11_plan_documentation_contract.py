"""
Motion-11 결과 문서가 계획 생성기의 실제 상수와 일치하는지 검증한다.

문서는 한때 `43000 ms / 2151 samples`를 기록했지만 생성기는
`47000 ms / 2351 samples`였다. 실기 계획을 읽고 재현하는 사람이 문서를
신뢰할 수 있어야 하므로, 사람이 고치는 대신 기계가 확인한다.

소스/문서 파싱 시험 선례:
- tests/test_left_arm_q0_contract.py (mapping.py 파싱)
- tests/test_stm32_servo_uart_circular_dma_contract.py (C 소스 파싱)
"""

import importlib.util
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOCUMENT = (
    ROOT
    / "docs"
    / "test-results"
    / "2026-08-04-motion11-buffered-pick-pregrasp-plan-only.md"
)
TEXT = DOCUMENT.read_text(encoding="utf-8")

SPEC = importlib.util.spec_from_file_location(
    "plan_buffered_pick_pregrasp_doc_contract",
    ROOT / "tools" / "plan_buffered_pick_pregrasp.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def bullet_value(label: str) -> str:
    """Return the first backticked value of a `- <label>: \\`value\\`` bullet."""
    match = re.search(
        rf"^- {re.escape(label)}: `([^`]+)`",
        TEXT,
        re.MULTILINE,
    )
    assert match is not None, f"문서에 '- {label}: `...`' 항목이 없다"
    return match.group(1)


def test_document_durations_match_the_plan_generator() -> None:
    assert bullet_value("anchor→q0") == f"{MODULE.ANCHOR_TO_Q0_DURATION_MS} ms"
    assert bullet_value("q0→pregrasp") == (
        f"{MODULE.Q0_TO_PREGRASP_DURATION_MS} ms"
    )
    assert bullet_value("총 시간") == f"{MODULE.TOTAL_DURATION_MS} ms"


def test_document_sample_count_matches_the_resampling_period() -> None:
    period_ms = 20
    expected = MODULE.TOTAL_DURATION_MS // period_ms + 1
    waypoints = bullet_value("waypoint/sample")
    assert waypoints == f"{expected}개"
    assert f"accepted samples: `{expected}`" in TEXT
    # 본문 산문도 같은 초 단위를 써야 한다.
    assert f"{MODULE.TOTAL_DURATION_MS // 1000}초 후보" in TEXT
    assert "43초" not in TEXT
    assert "2151" not in TEXT


def test_document_tracking_contract_matches_the_generator_limits() -> None:
    assert bullet_value("계획 검증용 보수적 rate") == (
        f"{MODULE.CONSERVATIVE_TRACKING_RATE_RAW_S:g} raw/s"
    )
    assert bullet_value("허용 peak error") == (
        f"{MODULE.MAXIMUM_MODELED_PEAK_ERROR_RAW:g} raw"
    )
    assert bullet_value("허용 terminal error") == (
        f"{MODULE.MAXIMUM_MODELED_TERMINAL_ERROR_RAW:g} raw"
    )


def test_document_firmware_identity_matches_the_generator() -> None:
    assert f"`{MODULE.CAPABILITIES}`" in TEXT
    assert f"`{MODULE.EXPECTED_SOURCE_ROUTE_SHA256}`" in TEXT


def test_document_apply_offsets_follow_the_deployed_startup_lead() -> None:
    """마지막 apply offset은 lead + 총 시간이어야 한다."""
    first, last = bullet_value("첫/마지막 apply offset").split(" / ")
    lead_ms = int(first)
    assert last == f"{lead_ms + MODULE.TOTAL_DURATION_MS} ms"
