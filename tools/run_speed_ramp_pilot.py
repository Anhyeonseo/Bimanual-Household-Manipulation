#!/usr/bin/env python3
"""추종률 가정을 단계적으로 올리며 실측이 어디서 무너지는지 잰다.
**단계마다 물리 이동이 있다.**

**무엇을 재는가.**

이 저장소에서 팔이 느린 이유는 하나다 —
`CONSERVATIVE_TRACKING_RATE_RAW_S = 50.0` (초당 4.4°). `select_duration_ms`
가 "서보가 초당 50 raw 밖에 못 따라온다"고 가정하고 leg 시간을 정한다.
MoveIt 의 velocity scaling 은 이 경로에 아무 영향이 없다 —
`plan_buffered_segment_leg` 가 MoveIt 타이밍을 버리고 여기서 다시 정한다.

그런데 `50` 의 근거가 약하다. 2026-08-04 Motion-11 이 관측한 `60 raw/s` 는
**post-terminal** 추종률 — 궤적이 끝난 뒤 뒤처진 팔이 기어서 따라잡던
속도지 서보의 능력치가 아니다. 그 값을 능력치로 쓰면서 전 저장소가 느려졌다.

**모델 게이트는 이 시험에서 아무것도 지켜주지 않는다.**

peak/terminal 오차를 rate 가정으로 계산하므로, rate 를 올리면 게이트도 같이
느슨해져 self-consistent 하게 통과한다. 실제로 지켜주는 것은 둘뿐이다.

  1. `validate_buffered_trajectory` 의 관절 속도 상한 `0.5 rad/s`(=326 raw/s)
  2. 하드웨어에서 실측되는 post-settle 오차

그래서 이 도구는 **2번을 단계마다 읽고 스스로 멈춘다.** bridge 의 경성
게이트는 `30 raw` 이고 넘으면 fail-closed 되어 bridge 를 다시 올려야 한다.
그 전에 멈추려고 기본 정지선을 `20 raw` 로 둔다 — 벽에 부딪히지 않고 벽의
위치를 알아내는 것이 목적이다.

**한 단계는 이렇게 돈다.**

    q0 → pregrasp(이 단계의 rate 로) → q0

`grasp` 로 내려가지 않는다. 물체도 필요 없다. 재려는 것은 파지가 아니라
**펼치는 방향의 긴 이동에서 서보가 명령을 따라오는가**이고, Motion-11 이
추종 실패로 중단됐던 구간이 정확히 이 구간이다. q0 복귀는 검증된 Motion-12
경로를 기본 rate 로 쓴다 — 재는 구간과 복귀 구간을 섞지 않는다.

**자동 재시도가 없다.** 한 단계라도 실패하면 멈추고 그때까지를 남긴다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from buffered_leg_telemetry import (  # noqa: E402
    format_leg_trend,
    summarise_leg_telemetry,
)
from plan_buffered_pick_place_leg import (  # noqa: E402
    MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S,
)
from plan_buffered_pick_pregrasp import (  # noqa: E402
    CONSERVATIVE_TRACKING_RATE_RAW_S,
)
from run_grasp_repeatability_pilot import (  # noqa: E402
    LegLog,
    move_to,
    return_to_q0,
    sha256_file,
)


CONFIRMATION = "RUN_SPEED_RAMP_PILOT"
STATUS = "SPEED_RAMP_PILOT"

# bridge 의 경성 post-settle 게이트. 넘으면 abort 후 fail-closed 되어
# bridge 를 다시 올려야 한다. 우리는 그 앞에서 멈춘다.
BRIDGE_POST_SETTLE_GATE_RAW = 30
DEFAULT_STOP_RAW = 20

DEFAULT_RATES = (50.0, 70.0, 90.0, 120.0, 150.0)


def parse_rates(text: str) -> tuple[float, ...]:
    """단조 증가하는 rate 목록만 받는다.

    순서가 뒤섞이면 "어느 rate 에서 무너졌는가"를 말할 수 없다 — 앞 단계가
    남긴 열·처짐 상태가 뒤 단계에 실리기 때문이다.
    """
    values = tuple(float(part) for part in text.split(",") if part.strip())
    if not values:
        raise ValueError("at least one rate is required")
    for value in values:
        if not 0.0 < value <= MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S:
            raise ValueError(
                "every rate must be in (0, "
                f"{MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S:g}] raw/s"
            )
    if any(b <= a for a, b in zip(values, values[1:])):
        raise ValueError("rates must increase strictly")
    return values


def summarise(steps: list[dict], stop_raw: int) -> dict:
    """어느 rate 까지 통과했고 얼마나 빨라졌는가.

    판정하지 않는다. 통과한 마지막 rate 와 그때의 여유를 내놓을 뿐이다 —
    운영값으로 무엇을 채택할지는 이 자료를 보고 사람이 정한다.
    """
    passed = [step for step in steps if step["ok"]]
    summary: dict = {
        "step_count": len(steps),
        "passed_step_count": len(passed),
        "post_settle_stop_raw": stop_raw,
        "bridge_post_settle_gate_raw": BRIDGE_POST_SETTLE_GATE_RAW,
        "default_rate_raw_s": CONSERVATIVE_TRACKING_RATE_RAW_S,
    }
    if not passed:
        return summary

    baseline = passed[0]
    summary["highest_passing_rate_raw_s"] = passed[-1]["rate_raw_s"]
    summary["baseline_rate_raw_s"] = baseline["rate_raw_s"]
    observed = [
        step["post_settle_max_error_raw"]
        for step in passed
        if step.get("post_settle_max_error_raw") is not None
    ]
    if observed:
        summary["post_settle_max_error_raw"] = {
            "values": observed,
            "maximum": max(observed),
            "margin_to_stop_raw": stop_raw - max(observed),
            "margin_to_bridge_gate_raw": (
                BRIDGE_POST_SETTLE_GATE_RAW - max(observed)
            ),
        }
    durations = [
        step["duration_ms"]
        for step in passed
        if step.get("duration_ms") is not None
    ]
    if len(durations) >= 2 and durations[-1]:
        summary["duration_ms"] = {
            "values": durations,
            "baseline": durations[0],
            "fastest": durations[-1],
            # 이 숫자가 사용자가 체감할 변화다.
            "speedup_factor": durations[0] / durations[-1],
        }
    lateness = [
        step["maximum_apply_lateness_ms"]
        for step in passed
        if step.get("maximum_apply_lateness_ms") is not None
    ]
    if lateness:
        summary["maximum_apply_lateness_ms"] = {
            "values": lateness,
            "maximum": max(lateness),
        }
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-plan", type=Path, required=True)
    parser.add_argument(
        "--rates",
        default=",".join(f"{value:g}" for value in DEFAULT_RATES),
        help="쉼표로 구분한 추종률(raw/s). 단조 증가해야 한다.",
    )
    parser.add_argument(
        "--post-settle-stop-raw",
        type=int,
        default=DEFAULT_STOP_RAW,
        help=(
            "이 값을 넘으면 다음 단계로 올라가지 않는다. bridge 의 경성 "
            f"게이트 {BRIDGE_POST_SETTLE_GATE_RAW} 아래여야 한다."
        ),
    )
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--arm", default="left")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config" / "single_arm_calibration.json",
    )
    parser.add_argument(
        "--bridge-calibration",
        type=Path,
        default=PACKAGE / "config" / "single_arm_calibration.json",
    )
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact speed ramp confirmation is required")
    if not 0 < arguments.post_settle_stop_raw < BRIDGE_POST_SETTLE_GATE_RAW:
        parser.error(
            "the operator stop must sit strictly below the bridge gate "
            f"({BRIDGE_POST_SETTLE_GATE_RAW} raw); 넘어서면 bridge 가 먼저 "
            "fail-closed 되어 다시 올려야 한다"
        )
    try:
        arguments.rate_values = parse_rates(arguments.rates)
    except ValueError as error:
        parser.error(str(error))
    return arguments


def main() -> int:
    from grasp_yaw_kinematics import arm_joint_names
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    arguments.workdir.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration(arguments.bridge_calibration)
    arm_names = arm_joint_names(f"{arguments.arm}_")

    print(f"ARM={arguments.arm}")
    print(f"SOURCE_PLAN={arguments.source_plan}")
    print(f"SOURCE_PLAN_SHA256={sha256_file(arguments.source_plan)}")
    print(f"RATES_RAW_S={[f'{v:g}' for v in arguments.rate_values]}")
    print(f"DEFAULT_RATE_RAW_S={CONSERVATIVE_TRACKING_RATE_RAW_S:g}")
    print(f"POST_SETTLE_STOP_RAW={arguments.post_settle_stop_raw}")
    print(f"BRIDGE_POST_SETTLE_GATE_RAW={BRIDGE_POST_SETTLE_GATE_RAW}")
    print("CYCLE=q0 -> pregrasp(단계 rate) -> q0(기본 rate)")
    print(
        "QUESTION=어느 추종률까지 서보가 실제로 따라오는가 — "
        "모델이 아니라 post-settle 실측으로"
    )

    steps: list[dict] = []
    leg_log = LegLog()
    stopped: str | None = None
    for index, rate in enumerate(arguments.rate_values, start=1):
        work = arguments.workdir / f"rate{index:02d}_{rate:g}"
        work.mkdir(parents=True, exist_ok=True)
        print(
            f"\n===== 단계 {index}/{len(arguments.rate_values)}  "
            f"{rate:g} raw/s ====="
        )
        try:
            move_to(work, "pregrasp", arguments.source_plan, "pregrasp",
                    arguments.calibration, arm_names, calibration,
                    leg_log, index, rate)
            return_to_q0(work, arm_names, calibration, leg_log, index)
        except Exception as error:
            stopped = f"rate {rate:g} raw/s: {error}"
            print(f"STOPPED {stopped}")
            break

        # 이 단계에서 rate 를 적용받은 leg 은 pregrasp 하나다. q0 복귀는
        # 기본 rate 라 여기 섞으면 안 된다.
        measured = next(
            leg for leg in reversed(leg_log.legs) if leg["tag"] == "pregrasp"
        )
        step = {
            "step": index,
            "rate_raw_s": rate,
            "ok": True,
            "duration_ms": measured.get("duration_ms"),
            "sample_count": measured.get("sample_count"),
            "post_settle_max_error_raw": measured.get(
                "post_settle_max_error_raw"
            ),
            "maximum_apply_lateness_ms": measured.get(
                "maximum_apply_lateness_ms"
            ),
            "precompute_ms": measured.get("precompute_ms"),
            "first_sample_lead_ms": measured.get("first_sample_lead_ms"),
        }
        steps.append(step)
        settle = step["post_settle_max_error_raw"]
        print(
            f"  duration {step['duration_ms']} ms  "
            f"post-settle {settle} raw  "
            f"lateness {step['maximum_apply_lateness_ms']} ms"
        )
        if settle is None:
            stopped = (
                f"rate {rate:g} raw/s: post-settle 값을 읽지 못했다 — "
                "근거 없이 다음 단계로 올릴 수 없다"
            )
            print(f"STOPPED {stopped}")
            break
        if settle > arguments.post_settle_stop_raw:
            stopped = (
                f"rate {rate:g} raw/s: post-settle {settle} raw 가 정지선 "
                f"{arguments.post_settle_stop_raw} 을 넘었다"
            )
            print(f"STOPPED {stopped}")
            break

    summary = summarise(steps, arguments.post_settle_stop_raw)
    startup = summarise_leg_telemetry(leg_log.legs) if leg_log.legs else None
    document = {
        "schema_version": 1,
        "status": f"{STATUS}_{'COMPLETE' if stopped is None else 'STOPPED'}",
        "arm": arguments.arm,
        "source_plan": str(arguments.source_plan),
        "source_plan_sha256": sha256_file(arguments.source_plan),
        "requested_rates_raw_s": list(arguments.rate_values),
        "summary": summary,
        "steps": steps,
        "startup_trend": startup,
        "legs": leg_log.legs,
        "stopped_reason": stopped,
        "automatic_retry_count": 0,
        "serial_port_opened": False,
        "operator_confirmation": CONFIRMATION,
    }
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    if startup is not None:
        print()
        for line in format_leg_trend(startup):
            print(line)

    print()
    print(f"STEPS_PASSED={summary['passed_step_count']}/{summary['step_count']}")
    if "highest_passing_rate_raw_s" in summary:
        print(
            "HIGHEST_PASSING_RATE_RAW_S="
            f"{summary['highest_passing_rate_raw_s']:g}"
        )
    settle = summary.get("post_settle_max_error_raw")
    if settle:
        print(
            f"POST_SETTLE_MAX_RAW={settle['maximum']}  "
            f"정지선까지 {settle['margin_to_stop_raw']}  "
            f"bridge 게이트까지 {settle['margin_to_bridge_gate_raw']}"
        )
    duration = summary.get("duration_ms")
    if duration:
        print(
            f"DURATION_MS={duration['values']}  "
            f"speedup={duration['speedup_factor']:.2f}x"
        )
    print(f"OUTPUT={arguments.output}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    print(document["status"])
    return 0 if stopped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
