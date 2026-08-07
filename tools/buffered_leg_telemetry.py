#!/usr/bin/env python3
"""실행기가 이미 찍고 있는 startup 진단을 leg 마다 거둬 추세로 만든다.

**새 계측이 아니다. 배선이다.**

`buffered_action_execution.start_goal` 은 leg 마다 `precompute_ms`(전체 sample
재생성 소요)와 `first_sample_lead_ms`(첫 sample 이 heartbeat 보다 얼마나
앞서는가)를 계산해 terminal 문자열에 남긴다. 실행기 3종이 그것을
`TERMINAL_DIAGNOSTICS=` 로 stdout 에 찍는다. 그런데 pilot 은 그 stdout 을
받아서 버렸다 — 그리퍼 출력만 `.txt` 로 남았다.

그래서 "6~7회 반복하면 첫 프레임이 거부된다"는 관찰이 **한 번도 수치로
기록된 적이 없다.** 이 모듈은 그 문자열을 구조로 바꾸고, 회차에 따라
커지는지 아닌지를 계산한다.

**어느 구간이 lead 를 먹는지 갈라 봐야 한다.**

`start_goal` 의 순서는 이렇다.

    precompute → **fresh heartbeat** → reanchor → prime frame 1
    → prime heartbeat(여기서 lead 를 재고 게이트)

`precompute` 는 heartbeat **앞**에서 끝난다. 그러니 아무리 커져도 lead 를
깎을 수 없다 — 2026-08-06 에 precompute 가 두 배(186 → 386 ms)로 늘어난 것을
멈춤의 원인으로 지목했다가 코드 구조로 반증됐다. heartbeat 이 사 온 220 ms 를
실제로 갉아먹는 것은 `reanchor_ms + prime_frame_1_ms` 뿐이고, 이 둘은 여태
**따로 측정된 적이 없었다.** 그래서 `lead_consuming_ms` 를 따로 낸다.

**게이트가 두 겹이라 둘 다 봐야 한다.**

  - 펌웨어 하한 `60 ms` — 이 아래면 `ACTUATOR_QUEUE_STALE_TICK` 로 거부된다
  - host startup 게이트 `80..220 ms` — 그 전에 host 가 먼저 fail-closed 한다

관측되는 증상은 host 게이트 쪽이 먼저 걸리는 형태일 가능성이 높다. 두
여유를 따로 보고한다.

**leg 정체를 회차와 섞지 않는다.**

`precompute_ms` 는 sample 수에 비례한다. pick_pregrasp(긴 leg)와 lift(짧은
leg)를 한 줄에 놓고 기울기를 재면 회차 추세가 아니라 leg 종류의 차이를 재게
된다. 그래서 전체 기울기와 **tag 별 기울기**를 같이 낸다. 결론은 tag 별
쪽에서 읽어야 한다.

순수 파싱·산술만 한다. 직렬 포트도 ROS 도 열지 않는다.
"""

from __future__ import annotations

import re


# 진단 문자열이 성공 경로와 실패 경로에서 다른 이름을 쓴다.
#
#   성공: "... first_sample_lead_ms=132 prime_heartbeat_gates=1 ..."
#   실패: "startup first-sample lead gate failed: lead_ms=57 required=80..160
#          heartbeat_gates=3"
#
# 실패 쪽이 정확히 우리가 찾는 사건이므로 두 이름을 다 받는다.
_INTEGER_FIELDS = {
    "first_sample_lead_ms": "first_sample_lead_ms",
    "lead_ms": "first_sample_lead_ms",
    "prime_heartbeat_gates": "prime_heartbeat_gates",
    "heartbeat_gates": "prime_heartbeat_gates",
    "fresh_tick": "fresh_tick_ms",
    "prime_tick": "prime_tick_ms",
    "prime_frames": "prime_frames",
    "accepted": "accepted_samples",
    "applied": "applied_samples",
    "queued": "queued_samples",
    "maximum_apply_lateness_ms": "maximum_apply_lateness_ms",
    "post_settle_max_error_raw": "post_settle_max_error_raw",
    "PLAN_DURATION_MS": "duration_ms",
    "PLAN_SAMPLE_COUNT": "sample_count",
}
_FLOAT_FIELDS = {
    "precompute_ms": "precompute_ms",
    # **가설이 걸리는 자리는 여기다.** `precompute` 는 fresh heartbeat
    # *앞*에서 끝나므로 lead 를 깎을 수 없다. heartbeat 이 사 온 220 ms 를
    # 실제로 갉아먹는 것은 그 뒤의 두 구간뿐이다.
    "reanchor_ms": "reanchor_ms",
    "prime_frame_1_ms": "prime_frame_1_ms",
    # 속도 손잡이. leg 기록에 같이 실려야 "이 duration 이 어느 가정에서
    # 나왔는가"를 나중에 되짚을 수 있다.
    "PLAN_TRACKING_RATE_RAW_S": "tracking_rate_raw_s",
    "TRACKING_RATE_RAW_S": "tracking_rate_raw_s",
}

_FIELD_PATTERN = re.compile(
    r"(?<![\w.])(" + "|".join(
        sorted({*_INTEGER_FIELDS, *_FLOAT_FIELDS}, key=len, reverse=True)
    ) + r")=(-?\d+(?:\.\d+)?)"
)

# 이 두 값이 실행기의 startup 게이트다. 여유를 계산하려면 여기에서도
# 알아야 하는데, bridge package 를 import 하면 이 모듈이 ROS overlay 에
# 묶인다. 값이 갈라지지 않도록 계약 시험이 두 곳을 대조한다.
HOST_LEAD_GATE_FLOOR_MS = 80
HOST_LEAD_GATE_CEILING_MS = 220
FIRMWARE_LEAD_FLOOR_MS = 60


def parse_leg_telemetry(text: str) -> dict[str, float | int]:
    """실행기 출력 한 덩어리에서 startup 진단 수치를 뽑는다.

    성공 출력과 실패 출력을 가리지 않는다 — 실패 경로도 같은 진단 문자열을
    예외 메시지에 실어 보내기 때문이다.

    같은 이름이 여러 번 나오면 **마지막 것**을 쓴다. 실행기가 요약을 두 번
    찍는 자리가 있고, 뒤엣것이 terminal 에 가깝다.
    """
    record: dict[str, float | int] = {}
    for name, raw in _FIELD_PATTERN.findall(text):
        if name in _FLOAT_FIELDS:
            record[_FLOAT_FIELDS[name]] = float(raw)
        else:
            key = _INTEGER_FIELDS[name]
            # 별칭(lead_ms)은 정식 이름이 이미 있으면 덮지 않는다.
            if name in ("lead_ms", "heartbeat_gates") and key in record:
                continue
            record[key] = int(float(raw))

    # 파생값 하나. fresh heartbeat 이 사 온 220 ms 를 실제로 갉아먹는 구간의
    # 합이다. `precompute_ms` 는 그 heartbeat **앞**에서 끝나므로 아무리
    # 커져도 lead 를 깎지 못한다 — 2026-08-06 에 precompute 가 두 배로
    # 늘어난 것(186 → 386 ms)을 원인으로 지목했다가 코드 구조로 반증됐다.
    consuming = [
        record[field]
        for field in ("reanchor_ms", "prime_frame_1_ms")
        if field in record
    ]
    if consuming:
        record["lead_consuming_ms"] = round(sum(consuming), 3)
    return record


def _linear_slope(xs: list[float], ys: list[float]) -> float | None:
    """최소자승 기울기. 점이 둘 미만이거나 x 가 전부 같으면 None."""
    if len(xs) < 2:
        return None
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator == 0.0:
        return None
    numerator = sum(
        (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys, strict=True)
    )
    return numerator / denominator


def _series(legs: list[dict], field: str) -> dict | None:
    """한 수치의 회차 추세. x 축은 세션 누적 leg 번호다."""
    points = [
        (float(leg["ordinal"]), float(leg[field]))
        for leg in legs
        if isinstance(leg.get(field), (int, float))
    ]
    if not points:
        return None
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    summary = {
        "values": ys,
        "ordinals": [int(x) for x in xs],
        "count": len(ys),
        "minimum": min(ys),
        "maximum": max(ys),
        "first": ys[0],
        "last": ys[-1],
        "drift": ys[-1] - ys[0],
    }
    slope = _linear_slope(xs, ys)
    if slope is not None:
        summary["slope_per_leg"] = slope
    return summary


def summarise_leg_telemetry(legs: list[dict]) -> dict:
    """leg 기록에서 "반복할수록 커지는가"를 계산한다.

    판정을 내리지 않는다. 기울기와 여유를 내놓을 뿐이다 — 가설을 확정하는
    것은 사람의 몫이고, 이 함수가 임계값을 들고 있으면 그 임계값이 다음
    세션에 근거 없이 재사용된다.
    """
    ordered = sorted(legs, key=lambda leg: leg["ordinal"])
    summary: dict = {
        "leg_count": len(ordered),
        "failed_leg_count": sum(1 for leg in ordered if not leg.get("ok", True)),
    }
    for field in (
        "precompute_ms",
        "reanchor_ms",
        "prime_frame_1_ms",
        "lead_consuming_ms",
        "first_sample_lead_ms",
    ):
        series = _series(ordered, field)
        if series is not None:
            summary[field] = series

    leads = [
        leg["first_sample_lead_ms"]
        for leg in ordered
        if isinstance(leg.get("first_sample_lead_ms"), int)
    ]
    if leads:
        summary["lead_gate"] = {
            "host_floor_ms": HOST_LEAD_GATE_FLOOR_MS,
            "host_ceiling_ms": HOST_LEAD_GATE_CEILING_MS,
            "firmware_floor_ms": FIRMWARE_LEAD_FLOOR_MS,
            "minimum_observed_ms": min(leads),
            "minimum_host_margin_ms": min(leads) - HOST_LEAD_GATE_FLOOR_MS,
            "minimum_firmware_margin_ms": min(leads) - FIRMWARE_LEAD_FLOOR_MS,
        }

    # precompute 는 sample 수에 비례한다. 회차 추세를 보려면 그 비례분을
    # 나눠야 leg 길이가 다른 경로끼리 비교된다.
    per_sample = [
        {
            "ordinal": leg["ordinal"],
            "value": leg["precompute_ms"] / leg["sample_count"],
        }
        for leg in ordered
        if isinstance(leg.get("precompute_ms"), (int, float))
        and isinstance(leg.get("sample_count"), int)
        and leg["sample_count"] > 0
    ]
    normalised = _series(
        [{"ordinal": item["ordinal"], "value": item["value"]} for item in per_sample],
        "value",
    )
    if normalised is not None:
        summary["precompute_ms_per_sample"] = normalised

    # 결론은 여기에서 읽는다. leg 정체가 섞이지 않은 유일한 자리다.
    by_tag: dict[str, dict] = {}
    for tag in dict.fromkeys(leg.get("tag", "") for leg in ordered):
        same = [leg for leg in ordered if leg.get("tag", "") == tag]
        if len(same) < 2:
            continue
        entry: dict = {"leg_count": len(same)}
        for field in (
            "precompute_ms",
            "reanchor_ms",
            "lead_consuming_ms",
            "first_sample_lead_ms",
        ):
            series = _series(same, field)
            if series is not None:
                entry[field] = series
        if len(entry) > 1:
            by_tag[tag] = entry
    if by_tag:
        summary["by_tag"] = by_tag
    return summary


def format_leg_trend(summary: dict) -> list[str]:
    """터미널 한 화면 요약. 파일을 열지 않고도 추세가 보이게."""
    lines = [f"LEG_COUNT={summary['leg_count']}"]
    if summary.get("failed_leg_count"):
        lines.append(f"FAILED_LEGS={summary['failed_leg_count']}")
    for field, label in (
        ("precompute_ms", "PRECOMPUTE_MS"),
        ("reanchor_ms", "REANCHOR_MS"),
        ("prime_frame_1_ms", "PRIME_FRAME_1_MS"),
        ("lead_consuming_ms", "LEAD_CONSUMING_MS"),
        ("first_sample_lead_ms", "FIRST_SAMPLE_LEAD_MS"),
    ):
        series = summary.get(field)
        if not series:
            continue
        lines.append(
            f"{label}={series['first']:g}..{series['last']:g}  "
            f"range={series['minimum']:g}..{series['maximum']:g}  "
            f"drift={series['drift']:+g}"
            + (
                f"  slope={series['slope_per_leg']:+.3f}/leg"
                if "slope_per_leg" in series
                else ""
            )
        )
    gate = summary.get("lead_gate")
    if gate:
        lines.append(
            f"LEAD_MARGIN_MS host={gate['minimum_host_margin_ms']:+d} "
            f"firmware={gate['minimum_firmware_margin_ms']:+d}  "
            f"(관측 최소 lead {gate['minimum_observed_ms']} ms)"
        )
    for tag, entry in (summary.get("by_tag") or {}).items():
        parts = []
        for field, label in (
            ("precompute_ms", "precompute"),
            ("reanchor_ms", "reanchor"),
            ("lead_consuming_ms", "consuming"),
            ("first_sample_lead_ms", "lead"),
        ):
            series = entry.get(field)
            if series and "slope_per_leg" in series:
                parts.append(f"{label}={series['slope_per_leg']:+.3f}")
        if parts:
            lines.append(
                f"TAG_SLOPE[{tag}] n={entry['leg_count']} " + " ".join(parts)
            )
    return lines
