#!/usr/bin/env python3
"""그리퍼 jaw gap(mm) ↔ semantic raw 대응표를 감독 하에 실측한다.

캔을 안전하게 파지하려면 승인된 gripper envelope 안에서 실제 jaw gap을
직접 측정해야 한다. 이 도구는 임의의 과거 파지 상수를 재사용하지 않는다.

**무엇을 재는가.** M3. 저장소 어디에도 jaw gap 을 mm 로 잰 기록이 없다.
URDF 의 `gripper_joint` 한계 `0..1.91986 rad` 은 각도이지 mm 가 아니고,
`config/bimanual_operational_limits.json` 의 그리퍼 항목도 "servo-command
bounds, not jaw-gap geometry" 라고 스스로 밝히고 있다.

**개방 방향.** semantic raw가 커지면 열린다는 규약을
**첫 두 점의 실측으로 검증**한다.
부호가 반대였다면 큰 raw 명령이 조를 닫아버리므로, 한 번에 움직이는 양도
제한한다.

안전 계약:

- 그리퍼 축(5/11) 하나만 바꾼 12축 절대 command 를 낸다. 나머지는 측정된
  현재 자세를 그대로 유지한다.
- 한 걸음의 raw 변화량을 `--maximum-step-raw` 로 제한한다.
- 매 걸음 뒤 팔 10축이 `0.02 rad` 넘게 움직였으면 즉시 중단한다.
- 자동 재시도 0. `accepted=false` 면 즉시 중단한다.
- 정상 종료는 torque hold 다. STOP 은 작업자가 팔을 받친 뒤 따로 낸다.

**자세는 topic 이 아니라 status service 에서 읽는다.** READY 에서는 서보
피드백 폴링이 멈춰 `/bimanual_stream_adapter/joint_states` 가 갱신되지 않는다.
이 도구는 걸음마다 작업자가 caliper 를 재는 동안 몇 분씩 멈추므로, transient
topic 에 의존하면 다음 읽기에서 hang 한다. resident 는 각 finite leg 완료 시
측정된 최종 자세를 prepared state 에 넣어두고 status service 로 돌려주므로
그 값을 쓴다. 장시간 계측 중에도 transient topic에 의존하지 않는다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from rclpy.node import Node
from so101_interfaces.srv import BimanualStreamCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES


ROOT = Path(__file__).resolve().parents[3]
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
OWNER = "can_jaw_gap_commissioning"
CONFIRMATION = "COMMISSION_CAN_JAW_GAP_MAP_ONCE"
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
RAW_STEP_RAD = 2.0 * math.pi / 4096.0
SAMPLE_PERIOD_MS = 50
FIRST_POINT_MS = 80
GRIPPER_INDICES = {"left": 5, "right": 11}
ARM_INDICES = tuple(index for index in range(12) if index not in (5, 11))
ARM_MOTION_LIMIT_RAD = 0.02
EXPECTED_FIRMWARE = "0x00024809"


def semantic_raw_to_rad(raw: float) -> float:
    return (2048.0 - float(raw)) * RAW_STEP_RAD


def semantic_rad_to_raw(position_rad: float) -> int:
    if not math.isfinite(position_rad):
        raise ValueError("gripper position must be finite")
    return round(2048.0 - position_rad / RAW_STEP_RAD)


def approved_gripper_raw_bounds(side: str) -> tuple[int, int]:
    """승인된 envelope 에서 그리퍼 command 한계를 읽는다.

    상수로 복사하지 않는다. 이 파일이 펌웨어 표
    (`firmware/.../bimanual_operational_limits.c`) 의 canonical source 다.
    """
    document = json.loads(OPERATIONAL_LIMITS.read_text(encoding="utf-8"))
    if (
        document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True
    ):
        raise RuntimeError("bimanual operational limits are not approved")
    gripper = document["arms"][side]["gripper"]
    if gripper.get("coordinate") != "semantic_raw":
        raise RuntimeError("gripper limits are not in semantic raw")
    return int(gripper["minimum_unwrapped_raw"]), int(
        gripper["maximum_unwrapped_raw"]
    )


def parse_raw_steps(text: str) -> tuple[int, ...]:
    steps = tuple(int(item) for item in text.replace(" ", "").split(",") if item)
    if len(steps) < 2:
        raise ValueError("at least two raw steps are required")
    return steps


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--raw-steps",
        required=True,
        help="쉼표로 구분한 semantic raw 목표. 예: 2048,2200,2350,2500,2650",
    )
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--maximum-step-raw", type=int, default=200)
    parser.add_argument("--duration-ms", type=int, default=1500)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--label", required=True)
    parser.add_argument(
        "--can-diameter-mm",
        type=float,
        default=53.0,
        help="계획 문서에 기록된 실측 지름. 보고용이며 명령에 쓰지 않는다.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/can_to_bin/jaw_gap_map_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error(
            "confirmation mismatch; support both arms and clear the jaws first"
        )
    try:
        args.raw_steps = parse_raw_steps(args.raw_steps)
    except ValueError as error:
        parser.error(str(error))
    minimum, maximum = approved_gripper_raw_bounds(args.side)
    outside = [raw for raw in args.raw_steps if not minimum <= raw <= maximum]
    if outside:
        parser.error(
            f"raw steps {outside} are outside the approved {args.side} gripper "
            f"envelope {minimum}..{maximum}"
        )
    if not 20 <= args.maximum_step_raw <= 400:
        parser.error("maximum step raw must be within 20..400")
    if not 800 <= args.duration_ms <= 2500 or args.duration_ms % 50 != 0:
        parser.error("duration must be a multiple of 50 ms within 800..2500 ms")
    if args.timeout_s <= 0.0:
        parser.error("timeout must be positive")
    if not args.label.strip():
        parser.error("label must not be empty")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing artifact: {args.output}")
    return args


def call(node: Node, client: Any, request: Any, timeout_s: float) -> Any:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}") from error
    return future.result()


def status_document(node: Node, client: Any, timeout_s: float) -> dict[str, Any]:
    response = call(node, client, Trigger.Request(), timeout_s)
    if not response.success:
        raise RuntimeError(f"status service rejected: {response.message}")
    document = json.loads(response.message)
    if not isinstance(document, dict):
        raise RuntimeError("status response is not an object")
    return document


def prepared_positions(
    document: dict[str, Any],
    *,
    label: str,
    expected_epoch: int,
    require_torque_hold: bool,
) -> tuple[float, ...]:
    """resident prepared state 에서 측정된 12축 자세를 꺼낸다."""
    if int(document.get("prepared_epoch", -1)) != expected_epoch:
        raise RuntimeError(
            f"{label} prepared epoch mismatch: "
            f"{document.get('prepared_epoch')} != {expected_epoch}"
        )
    if require_torque_hold and document.get("torque_hold_active") is not True:
        raise RuntimeError(f"{label} status does not prove torque hold")
    values = document.get("prepared_positions_rad")
    if (
        not isinstance(values, list)
        or len(values) != 12
        or not all(math.isfinite(float(value)) for value in values)
    ):
        raise RuntimeError(f"{label} status has no complete prepared anchor")
    return tuple(float(value) for value in values)


def wait_until_ready(
    node: Node, client: Any, expected_epoch: int, timeout_s: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        latest = status_document(node, client, timeout_s)
        if (
            latest.get("state") == "ready"
            and latest.get("owner") == OWNER
            and latest.get("arbiter_epoch") == expected_epoch
        ):
            return latest
        if latest.get("state") not in ("active", "ready"):
            raise RuntimeError(f"resident failed closed: {latest}")
        time.sleep(0.05)
    raise RuntimeError(f"step completion timeout: {latest}")


def interpolate(
    start: tuple[float, ...], target: tuple[float, ...], duration_ms: int
) -> list[tuple[int, tuple[float, ...]]]:
    count = duration_ms // SAMPLE_PERIOD_MS
    output: list[tuple[int, tuple[float, ...]]] = []
    for index in range(1, count + 1):
        fraction = index / count
        output.append(
            (
                FIRST_POINT_MS + (index - 1) * SAMPLE_PERIOD_MS,
                tuple(
                    begin + (end - begin) * fraction
                    for begin, end in zip(start, target, strict=True)
                ),
            )
        )
    output.append((FIRST_POINT_MS + count * SAMPLE_PERIOD_MS, tuple(target)))
    return output


def trajectory_point(
    positions: tuple[float, ...], offset_ms: int
) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = offset_ms // 1000
    point.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return point


def prompt_measured_gap_mm(raw: int) -> float:
    """작업자에게 caliper 값을 받는다.

    빈 입력이나 비정상 값을 조용히 0 으로 만들지 않는다. 이 값이 그대로
    실기 close 명령이 되므로 잘못 들어가면 캔이 아니라 조가 상한다.
    """
    while True:
        text = input(
            f"  raw={raw} 에서 두 jaw 평행면 사이를 mm 로 재서 입력하세요: "
        ).strip()
        try:
            value = float(text)
        except ValueError:
            print("  숫자를 입력하세요.")
            continue
        if not math.isfinite(value) or not 0.0 <= value <= 200.0:
            print("  0..200 mm 안의 값이어야 합니다.")
            continue
        return value


def interpolate_raw_for_gap(
    samples: list[dict[str, Any]], gap_mm: float
) -> dict[str, Any] | None:
    """실측 표에서 목표 gap 에 해당하는 raw 를 선형 보간한다.

    외삽하지 않는다. 실측 구간 밖이면 None 을 돌려주고 계획은 그 값을 거부한다.
    """
    ordered = sorted(samples, key=lambda item: item["measured_gap_mm"])
    for low, high in zip(ordered, ordered[1:], strict=False):
        low_gap = low["measured_gap_mm"]
        high_gap = high["measured_gap_mm"]
        if low_gap <= gap_mm <= high_gap and high_gap > low_gap:
            fraction = (gap_mm - low_gap) / (high_gap - low_gap)
            raw = low["measured_raw"] + fraction * (
                high["measured_raw"] - low["measured_raw"]
            )
            return {
                "gap_mm": gap_mm,
                "raw": raw,
                "rad": semantic_raw_to_rad(raw),
                "bracketed_by_raw": [low["measured_raw"], high["measured_raw"]],
                "extrapolated": False,
            }
    return None


def main() -> int:
    args = parse_args()
    minimum_raw, maximum_raw = approved_gripper_raw_bounds(args.side)
    gripper_index = GRIPPER_INDICES[args.side]

    rclpy.init()
    node = Node("commission_can_jaw_gap_map_once")
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)
    refresh_client = node.create_client(Trigger, REFRESH_ANCHOR_SERVICE)
    motion_commands = 0
    samples: list[dict[str, Any]] = []
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
            (REFRESH_ANCHOR_SERVICE, refresh_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        initial = status_document(node, status_client, args.timeout_s)
        initial_owner = initial.get("owner")
        epoch = int(initial.get("arbiter_epoch", -1))
        if (
            initial.get("state") != "ready"
            or initial_owner not in (None, OWNER)
            or (initial_owner is None and epoch != 0)
            or initial.get("motion_authorized") is not True
            or initial.get("fault_diagnostic") is not None
            or initial.get("firmware_version") != EXPECTED_FIRMWARE
        ):
            raise RuntimeError(f"unexpected initial resident state: {initial}")

        # 첫 자세. owner 가 없으면 torque 가 꺼져 있으므로 hold 를 요구하지
        # 않는다. 이미 이 도구가 owner 면 직전 leg 의 terminal anchor 를 쓴다.
        if initial_owner is None:
            response = call(node, refresh_client, Trigger.Request(), args.timeout_s)
            if not response.success:
                raise RuntimeError(f"resident anchor refresh failed: {response}")
            refreshed = json.loads(response.message)
            source = refreshed if "prepared_positions_rad" in refreshed else initial
            session_start = prepared_positions(
                source,
                label="startup refresh",
                expected_epoch=epoch,
                require_torque_hold=False,
            )
        else:
            session_start = prepared_positions(
                initial,
                label="resumed session",
                expected_epoch=epoch,
                require_torque_hold=True,
            )
        current = session_start
        print(
            f"start raw={semantic_rad_to_raw(session_start[gripper_index])} "
            f"envelope={minimum_raw}..{maximum_raw}"
        )

        for step_index, target_raw in enumerate(args.raw_steps):
            current_raw = semantic_rad_to_raw(current[gripper_index])
            delta = abs(int(target_raw) - current_raw)
            if delta > args.maximum_step_raw:
                raise RuntimeError(
                    f"step {step_index} moves {delta} raw which exceeds the "
                    f"{args.maximum_step_raw} raw per-step limit; add "
                    "intermediate steps"
                )
            target = list(current)
            target[gripper_index] = semantic_raw_to_rad(target_raw)

            request = BimanualStreamCommand.Request()
            request.operation = BimanualStreamCommand.Request.START_FINITE
            request.owner = OWNER
            request.joint_names = list(CANONICAL_JOINT_NAMES)
            request.points = [
                trajectory_point(positions, offset_ms)
                for offset_ms, positions in interpolate(
                    current, tuple(target), args.duration_ms
                )
            ]
            epoch += 1
            motion_commands += 1
            started = call(node, command_client, request, args.timeout_s)
            if (
                not started.accepted
                or started.adapter_state != "active"
                or started.arbiter_epoch != epoch
            ):
                raise RuntimeError(
                    f"step {step_index} rejected: accepted={started.accepted} "
                    f"state={started.adapter_state} "
                    f"epoch={started.arbiter_epoch} "
                    f"diagnostic={started.diagnostic}"
                )
            terminal = wait_until_ready(node, status_client, epoch, args.timeout_s)

            after = prepared_positions(
                terminal,
                label=f"step {step_index}",
                expected_epoch=epoch,
                require_torque_hold=True,
            )
            current = after
            arm_motion = max(
                abs(after[index] - session_start[index]) for index in ARM_INDICES
            )
            if arm_motion > ARM_MOTION_LIMIT_RAD:
                raise RuntimeError(
                    f"uncommanded arm motion {arm_motion:.6f} rad exceeds limit"
                )
            measured_raw = semantic_rad_to_raw(after[gripper_index])
            print(
                f"[{step_index + 1}/{len(args.raw_steps)}] commanded "
                f"raw={target_raw} measured raw={measured_raw} "
                f"residual={abs(measured_raw - target_raw)}"
            )
            gap_mm = prompt_measured_gap_mm(measured_raw)
            samples.append(
                {
                    "step_index": step_index,
                    "commanded_raw": int(target_raw),
                    "commanded_rad": semantic_raw_to_rad(target_raw),
                    "measured_raw": measured_raw,
                    "measured_rad": after[gripper_index],
                    "command_residual_raw": abs(measured_raw - int(target_raw)),
                    "measured_gap_mm": gap_mm,
                    "maximum_uncommanded_arm_motion_rad": arm_motion,
                    "arbiter_epoch": epoch,
                }
            )

            # 부호 검증. raw 가 커지면 열려야 한다. 반대라면 다음 큰 raw 명령이
            # 조를 닫아버리므로 여기서 멈춘다.
            if len(samples) == 2:
                raw_delta = samples[1]["measured_raw"] - samples[0]["measured_raw"]
                gap_delta = (
                    samples[1]["measured_gap_mm"] - samples[0]["measured_gap_mm"]
                )
                if raw_delta * gap_delta <= 0.0:
                    raise RuntimeError(
                        "opening direction violates the semantic raw contract: "
                        f"raw {raw_delta:+d} produced gap {gap_delta:+.1f} mm. "
                        "Stop and re-check the gripper sign convention before "
                        "commanding any wider opening."
                    )

        monotonic = all(
            (later["measured_raw"] - earlier["measured_raw"])
            * (later["measured_gap_mm"] - earlier["measured_gap_mm"])
            > 0.0
            for earlier, later in zip(samples, samples[1:], strict=False)
        )
        grasp = interpolate_raw_for_gap(samples, 44.0)
        clearance = interpolate_raw_for_gap(
            samples, args.can_diameter_mm + 8.0
        )
        document = {
            "schema_version": 1,
            "record_kind": "can_jaw_gap_map_once",
            "status": "CAN_JAW_GAP_MAP_PASS",
            "label": args.label,
            "operator_confirmation": args.confirmation,
            "side": args.side,
            "firmware_version": initial["firmware_version"],
            "owner": OWNER,
            "approved_envelope_raw": [minimum_raw, maximum_raw],
            "maximum_step_raw": args.maximum_step_raw,
            "duration_ms": args.duration_ms,
            "can_diameter_mm": args.can_diameter_mm,
            "samples": samples,
            "monotonic": monotonic,
            "measured_gap_span_mm": [
                min(item["measured_gap_mm"] for item in samples),
                max(item["measured_gap_mm"] for item in samples),
            ],
            "derived_grasp_gap_44mm": grasp,
            "derived_clearance_gap": clearance,
            "session_start_positions_rad": list(session_start),
            "automatic_retry_count": 0,
            "motion_commands": motion_commands,
            "initial_status": initial,
            "terminal_state": "ready_torque_hold",
            "stop_sent": False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "CAN_JAW_GAP_MAP_PASS "
            f"label={args.label} side={args.side} "
            f"points={len(samples)} monotonic={monotonic} "
            f"motion_commands={motion_commands} "
            f"output={args.output} sha256={digest}"
        )
        if grasp is None:
            print(
                "WARNING: 44 mm is outside the measured span; add raw steps "
                "before planning a grasp."
            )
        if clearance is None:
            print(
                f"WARNING: {args.can_diameter_mm + 8.0:.1f} mm clearance gap is "
                "outside the measured span; add wider raw steps."
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
