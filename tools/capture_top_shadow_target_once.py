#!/usr/bin/env python3
"""Top 인식의 shadow target 을 1회 포착해 파지 계획 입력으로 승격한다.

**이 도구는 의도적으로 경계를 넘는다.** `ShadowObjectTarget` 메시지는 첫 줄에
"Never consume this as a motion goal" 이라고 적혀 있고, 발행자는 언제나
`robot_target_available=false` 로 낸다. 그 잠금은 인식 파이프라인이 스스로
동작을 유발하지 못하게 하기 위한 것이다.

여기서 그것을 파지 좌표로 쓰려면 잠금을 대신할 것이 있어야 한다. 이 도구가
지는 책임은 셋이다.

  1. 발행자가 스스로 권한을 주장하지 않았음을 확인한다. `motion_authorized`
     나 `robot_target_available` 가 참이면 거부한다 — 인식 노드가 그것을
     주장하기 시작했다는 뜻이고, 그건 이 도구의 문제가 아니라 계약 위반이다.
  2. 발행자가 이미 계산한 게이트를 요구한다. freshness, workspace,
     transform 검증, confidence. 보정 조건은 두 모드 중 하나면 된다 —
     펜처럼 긴 물체는 보드 사각형 밖으로 나가는 것이 정상이고 검출기가
     `allow_partial_footprint_observation` 으로 그것을 지원한다.
  3. **여러 표본이 일치할 것을 요구한다.** 한 프레임의 검출은 깜빡일 수
     있다. `capture_buffered_anchor_raw.py` 가 torque 유지를 확인하는 것과
     같은 이유다.

그리고 이 도구는 동작 명령을 보내지 않는다. 좌표를 artifact 로 남길 뿐이고,
실제 하강은 collision 검사를 거친 뒤 사람이 따로 승인한다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import statistics
import time


STATUS = "TOP_SHADOW_TARGET_CAPTURE_PASS"
DEFAULT_TOPIC = "/perception/top/object_shadow_left_base"
# VIS-001 실측: 위치 최대 10 mm, yaw 최대 5도. 표본 간 흔들림은 그보다
# 훨씬 작아야 한다. 크면 검출이 안정되지 않은 것이다.
DEFAULT_MAXIMUM_SPREAD_M = 0.004
DEFAULT_MAXIMUM_SPREAD_YAW_RAD = math.radians(2.0)
DEFAULT_MAXIMUM_FRAME_AGE_S = 0.5
DEFAULT_MINIMUM_CONFIDENCE = 0.5

REQUIRED_TRUE = (
    "shadow_pose_available",
    "transform_validated",
    "inside_workspace",
    "fresh",
)
REQUIRED_FALSE = ("motion_authorized", "robot_target_available")

# 검출기는 보정 조건을 두 가지로 인정한다. `top_perception.yaml` 의
# `allow_partial_footprint_observation: true` 가 후자를 켠다.
#
#   footprint_inside      -> TRACKING_BOARD_ONLY
#                            물체가 보정 보드 사각형 안에 들어온다
#   image_fully_visible   -> TRACKING_CENTER_CALIBRATED_FULLY_VISIBLE
#                            펜처럼 긴 물체가 보드 밖으로 나가지만 중심이
#                            보정 영역에 있고 물체 전체가 화면 안에 있다
#
# 둘 다 OK 상태다. 하나라도 성립하면 좌표를 믿을 수 있다. 둘 다 아니면
# 보정되지 않은 영역의 추정이므로 거부한다.
CALIBRATION_MODES = ("source_footprint_inside", "source_image_fully_visible")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--samples", type=int, default=15)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--maximum-spread-m", type=float, default=DEFAULT_MAXIMUM_SPREAD_M
    )
    parser.add_argument(
        "--maximum-spread-yaw-rad",
        type=float,
        default=DEFAULT_MAXIMUM_SPREAD_YAW_RAD,
    )
    parser.add_argument(
        "--maximum-frame-age-s", type=float, default=DEFAULT_MAXIMUM_FRAME_AGE_S
    )
    parser.add_argument(
        "--minimum-confidence", type=float, default=DEFAULT_MINIMUM_CONFIDENCE
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def evaluate(samples: list[dict], arguments: argparse.Namespace) -> dict:
    """포착한 표본이 파지 좌표로 승격될 자격이 있는지 판정한다."""
    if len(samples) < arguments.samples:
        raise ValueError(
            f"only {len(samples)}/{arguments.samples} shadow samples arrived"
        )

    for index, sample in enumerate(samples, start=1):
        for field in REQUIRED_TRUE:
            if sample[field] is not True:
                raise ValueError(
                    f"sample {index} has {field}=false "
                    f"(status={sample['status']!r})"
                )
        if not any(sample[field] for field in CALIBRATION_MODES):
            raise ValueError(
                f"sample {index} satisfies neither calibration mode: "
                f"footprint_inside=False image_fully_visible=False "
                f"(status={sample['status']!r}). 물체가 보정 영역 밖이거나 "
                "화면에서 잘렸다"
            )
        for field in REQUIRED_FALSE:
            if sample[field] is not False:
                raise ValueError(
                    f"sample {index} has {field}=true; the perception node "
                    "must never claim motion authority"
                )
        if sample["source_frame_age_s"] > arguments.maximum_frame_age_s:
            raise ValueError(
                f"sample {index} frame age {sample['source_frame_age_s']:.3f}s "
                f"exceeds {arguments.maximum_frame_age_s}s"
            )
        if sample["confidence"] < arguments.minimum_confidence:
            raise ValueError(
                f"sample {index} confidence {sample['confidence']:.3f} is "
                f"below {arguments.minimum_confidence}"
            )
        for axis in ("x_m", "y_m", "z_m", "yaw_rad"):
            if not math.isfinite(sample[axis]):
                raise ValueError(f"sample {index} {axis} is not finite")

    xs = [s["x_m"] for s in samples]
    ys = [s["y_m"] for s in samples]
    yaws = [s["yaw_rad"] for s in samples]
    spread_x = max(xs) - min(xs)
    spread_y = max(ys) - min(ys)
    spread_yaw = max(yaws) - min(yaws)
    if max(spread_x, spread_y) > arguments.maximum_spread_m:
        raise ValueError(
            f"shadow position is not settled: spread x={spread_x:.5f} "
            f"y={spread_y:.5f} exceeds {arguments.maximum_spread_m}"
        )
    if spread_yaw > arguments.maximum_spread_yaw_rad:
        raise ValueError(
            f"shadow yaw is not settled: spread {spread_yaw:.5f} exceeds "
            f"{arguments.maximum_spread_yaw_rad}"
        )

    return {
        "schema_version": 1,
        "status": STATUS,
        "topic": arguments.topic,
        "sample_count": len(samples),
        "target": {
            "x_m": statistics.median(xs),
            "y_m": statistics.median(ys),
            "z_m": statistics.median([s["z_m"] for s in samples]),
            "yaw_rad": statistics.median(yaws),
        },
        "spread": {
            "x_m": spread_x,
            "y_m": spread_y,
            "yaw_rad": spread_yaw,
            "maximum_allowed_m": arguments.maximum_spread_m,
            "maximum_allowed_yaw_rad": arguments.maximum_spread_yaw_rad,
        },
        "quality": {
            "maximum_frame_age_s": max(
                s["source_frame_age_s"] for s in samples
            ),
            "minimum_confidence": min(s["confidence"] for s in samples),
            "frame_age_limit_s": arguments.maximum_frame_age_s,
            "confidence_limit": arguments.minimum_confidence,
        },
        "calibration_mode": (
            "board_footprint"
            if all(s["source_footprint_inside"] for s in samples)
            else "center_calibrated_fully_visible"
        ),
        "footprint_inside_count": sum(
            1 for s in samples if s["source_footprint_inside"]
        ),
        "image_fully_visible_count": sum(
            1 for s in samples if s["source_image_fully_visible"]
        ),
        "publisher_claimed_authority": False,
        "promoted_to_grasp_input": True,
        "promotion_gate": "operator approves each descent separately",
        "execution_api_used": False,
        "motion_authorized": False,
    }


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from so101_interfaces.msg import ShadowObjectTarget

    arguments = parse_args()
    samples: list[dict] = []

    rclpy.init()
    node = Node("top_shadow_target_capture_once")
    try:
        def on_message(message) -> None:
            if len(samples) >= arguments.samples:
                return
            samples.append(
                {
                    "x_m": float(message.x_m),
                    "y_m": float(message.y_m),
                    "z_m": float(message.z_m),
                    "yaw_rad": float(message.yaw_rad),
                    "source_frame_age_s": float(message.source_frame_age_s),
                    "confidence": float(message.confidence),
                    "status": str(message.status),
                    **{
                        field: bool(getattr(message, field))
                        for field in REQUIRED_TRUE + REQUIRED_FALSE
                        + CALIBRATION_MODES
                    },
                }
            )

        node.create_subscription(
            ShadowObjectTarget, arguments.topic, on_message, 10
        )
        deadline = time.monotonic() + arguments.timeout_s
        while len(samples) < arguments.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"SAMPLES={len(samples)}/{arguments.samples}")
    if not samples:
        print(f"{arguments.topic} 에서 아무것도 오지 않았다.")
        print("top perception 노드가 실행 중인지 확인할 것.")
        return 1

    document = evaluate(samples, arguments)
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    target = document["target"]
    spread = document["spread"]
    print(f"TARGET_X_M={target['x_m']:.9f}")
    print(f"TARGET_Y_M={target['y_m']:.9f}")
    print(f"TARGET_Z_M={target['z_m']:.9f}")
    print(f"TARGET_YAW_RAD={target['yaw_rad']:.9f}")
    print(
        f"SPREAD_M=x{spread['x_m']:.5f} y{spread['y_m']:.5f} "
        f"yaw{spread['yaw_rad']:.5f}"
    )
    print(f"MAX_FRAME_AGE_S={document['quality']['maximum_frame_age_s']:.3f}")
    print(f"MIN_CONFIDENCE={document['quality']['minimum_confidence']:.3f}")
    print(f"CALIBRATION_MODE={document['calibration_mode']}")
    print(
        f"FOOTPRINT_INSIDE={document['footprint_inside_count']}/"
        f"{document['sample_count']}  "
        f"IMAGE_FULLY_VISIBLE={document['image_fully_visible_count']}/"
        f"{document['sample_count']}"
    )
    print("PUBLISHER_CLAIMED_AUTHORITY=0")
    print("SHADOW_GATE=PASS")
    print(f"OUTPUT={arguments.output}")
    print(
        "PLAN_ARGUMENT=--x "
        f"{target['x_m']:.12f} --y {target['y_m']:.12f} "
        f"--yaw {target['yaw_rad']:.12f}"
    )
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
