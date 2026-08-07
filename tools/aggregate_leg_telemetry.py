#!/usr/bin/env python3
"""여러 pilot 실행의 leg 진단을 세션 순서로 이어 붙여 추세를 다시 낸다.

**왜 필요한가.** pilot 은 한 회차라도 실패하면 그 자리에서 멈춘다(의도적 —
자동 재시도 없음). 그래서 `grasp` leg 처럼 회당 한 번만 나오는 leg 은
호출 1회로는 표본이 1~2개뿐이라 추세를 볼 수 없다. 이 도구는 순서대로 다시
호출한 pilot 의 evidence JSON 여러 개를 **파일이 만들어진 순서 그대로**
이어 붙여, 세션 전체를 관통하는 하나의 leg 순번으로 다시 계산한다.

물리 이동을 하지 않는다. 이미 남은 evidence JSON 만 읽는다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from buffered_leg_telemetry import (  # noqa: E402
    format_leg_trend,
    summarise_leg_telemetry,
)


STATUS = "LEG_TELEMETRY_AGGREGATE_PASS"


def load_legs(paths: list[Path]) -> list[dict]:
    """파일 순서 = 세션 순서. leg 의 원래 ordinal 은 파일 안에서만 유효하므로
    버리고, 이어 붙인 전체 순번으로 다시 매긴다."""
    combined: list[dict] = []
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        legs = document.get("legs")
        if not legs:
            raise ValueError(f"{path} 에 leg 진단이 없다 — legs 필드가 비었다")
        for leg in legs:
            entry = dict(leg)
            entry["source_file"] = path.name
            entry["ordinal"] = len(combined) + 1
            combined.append(entry)
    return combined


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if len(arguments.evidence) < 2:
        parser.error("합칠 evidence 파일이 2개 이상 필요하다")
    return arguments


def main() -> int:
    arguments = parse_args()
    legs = load_legs(arguments.evidence)
    summary = summarise_leg_telemetry(legs)
    document = {
        "schema_version": 1,
        "status": STATUS,
        "source_files": [str(path) for path in arguments.evidence],
        "source_files_sha256": [
            sha256(path.read_bytes()).hexdigest() for path in arguments.evidence
        ],
        "leg_count": len(legs),
        "summary": summary,
        "legs": legs,
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    print(f"SOURCE_FILES={len(arguments.evidence)}")
    print(f"TOTAL_LEG_COUNT={len(legs)}")
    for line in format_leg_trend(summary):
        print(line)
    print(f"OUTPUT={arguments.output}")
    print(document["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
