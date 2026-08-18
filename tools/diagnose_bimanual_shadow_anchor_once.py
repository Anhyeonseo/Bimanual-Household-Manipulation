#!/usr/bin/env python3
"""12축 shadow anchor 실패(status 8)의 원인 관절을 찾는다. 읽기 전용.

**왜 필요한가.**

resident가 startup에서 status 8로 죽으면 메시지는
`joint unwrap or operational-limit binding failed`뿐이고 **어느 관절인지 말해
주지 않는다.** 그런데 펌웨어는 실패 직전에 12축 raw를 응답에 이미 채워
넣는다(`host_v2_shadow_raw[joint] = raw`), 그리고 `parse_shadow_snapshot_v2`는
status와 무관하게 그 raw를 그대로 돌려준다. resident는 status를 보고 곧바로
예외를 던져 그 값을 버린다. 이 도구는 버리지 않고 출력한다.

**판정 근거.** 펌웨어의 `BimanualOperationalLimits_UnwrapModuloRaw`는
`turn ∈ {-1,0,+1}` 후보 중 operational limit 안에 드는 것이 **정확히 하나**일
때만 통과한다. 그래서 각 관절마다 허용되는 "읽힌 raw" 집합이 정해지고, 이
도구는 같은 규칙을 host에서 재현해 관절별로 통과/실패를 표시한다.

**안전.** arm, setpoint, fault clear, calibration 쓰기를 하지 않는다. 다만
`PREPARE_SHADOW`는 펌웨어가 verified torque-disable을 수행하는 요청이므로
**팔이 중력으로 떨어질 수 있다.** 그리고 status가 0이 아니면 펌웨어가 stop을
latch하므로, 실패를 확인한 뒤에는 STM32 RESET이 필요하다.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "I_AM_SUPPORTING_BOTH_ARMS_TORQUE_OFF"
HOST_BAUD = 921_600
RAW_MODULUS = 4096
TURN_URAD = 6_283_185
JOINT_COUNT = 12
ROOT = Path(__file__).resolve().parents[1]
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
CALIBRATION = ROOT / "config/single_arm_calibration.json"
SHORT_NAMES = (
    ("base", "BASE"),
    ("shoulder", "SHOULDER"),
    ("elbow", "ELBOW"),
    ("wrist_flex", "WRIST_FLEX"),
    ("wrist_roll", "WRIST_ROLL"),
    ("gripper", "GRIPPER"),
)
SHADOW_FAILURE_REASONS = {
    1: "stream executor is active",
    2: "left-arm torque-disable command failed",
    3: "right-arm verified torque-disable failed",
    4: "left-arm position capture failed",
    5: "right-arm position capture failed",
    6: "raw-to-angle calibration conversion failed",
    7: "invalid shadow preparation request",
    8: "joint unwrap or operational-limit binding failed",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--confirm-supported-torque-off",
        required=True,
        choices=(CONFIRMATION,),
        help="PREPARE_SHADOW disables torque; both arms must be supported",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/can_to_bin/shadow_anchor_diagnosis.json",
    )
    return parser.parse_args()


_CONTRACTS: dict | None = None


def load_contracts() -> dict | None:
    """한계·보정 파일을 한 번만 읽는다. 없으면 None.

    이 도구는 Pi에서도 돌아야 하는데 Pi 배포본에는 `config/`가 없을 수 있다.
    그때도 status와 raw는 출력해야 하므로 판정만 건너뛴다.
    """
    global _CONTRACTS
    if _CONTRACTS is not None:
        return _CONTRACTS or None
    if not (OPERATIONAL_LIMITS.is_file() and CALIBRATION.is_file()):
        _CONTRACTS = {}
        return None
    limits = json.loads(OPERATIONAL_LIMITS.read_text(encoding="utf-8"))
    calibration = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    _CONTRACTS = {
        "limits": limits,
        "by_name": {joint["name"]: joint for joint in calibration["joints"]},
    }
    return _CONTRACTS


def unwrapped_raw_window(arm: str, short_name: str) -> tuple[float, float]:
    """관절의 operational limit을 unwrapped raw 구간으로 환산한다."""
    contracts = load_contracts()
    if contracts is None:
        raise FileNotFoundError("operational limits/calibration are not deployed")
    limits = contracts["limits"]
    by_name = contracts["by_name"]
    limit = limits["arms"][arm][short_name[0]]
    joint = by_name[short_name[1]]
    zero = float(joint["zero_raw"])
    direction = float(joint["positive_raw_direction"])

    def to_raw(urad: float) -> float:
        return zero + direction * (urad / TURN_URAD) * RAW_MODULUS

    low = to_raw(limit["minimum_urad"])
    high = to_raw(limit["maximum_urad"])
    return (min(low, high), max(low, high))


def binding_verdict(observed_raw: int, low: float, high: float) -> dict:
    """펌웨어의 match_count 규칙을 host에서 그대로 재현한다."""
    matches = [
        observed_raw + turn * RAW_MODULUS
        for turn in (-1, 0, 1)
        if low <= observed_raw + turn * RAW_MODULUS <= high
    ]
    if len(matches) == 1:
        return {"binds": True, "match_count": 1, "unwrapped_raw": matches[0]}
    return {
        "binds": False,
        "match_count": len(matches),
        "unwrapped_raw": None,
        "cause": (
            "no in-limit turn candidate; the joint is parked outside its "
            "operational limits"
            if not matches
            else "two in-limit turn candidates; the binding is ambiguous"
        ),
    }


def main() -> int:
    args = parse_args()
    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    record: dict[str, object] = {
        "schema_version": 1,
        "record_kind": "bimanual_shadow_anchor_diagnosis",
        "motion_commands": 0,
        "arm_enable_calls": 0,
        "fault_clear_calls": 0,
        "device": device,
    }
    try:
        transport = StreamValidationTransportV2(port)
        hello = transport.enter_binary_mode()
        record["hello"] = asdict(hello)
        print(
            f"HELLO firmware=0x{hello.firmware_version:08X} "
            f"protocol={hello.protocol_version} joints={hello.joint_count} "
            f"stop_latched={hello.stop_latched} "
            f"rejected_frames={hello.rejected_frame_count}"
        )
        if hello.stop_latched:
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_BLOCKED_STOP_LATCHED"
            print(
                "\nSTM32 stop is latched. PREPARE_SHADOW cannot run.\n"
                "Support both arms, press the NUCLEO RESET button, then rerun."
            )
            return 2

        snapshot = transport.prepare_shadow()
        status = int(snapshot.status_code)
        record["shadow_status_code"] = status
        record["shadow_status_reason"] = SHADOW_FAILURE_REASONS.get(
            status, "ok" if status == 0 else "unknown firmware shadow failure"
        )
        record["positions_raw"] = list(snapshot.positions_raw)
        record["left_present_mask"] = int(snapshot.left_present_mask)
        record["right_present_mask"] = int(snapshot.right_present_mask)

        print(
            f"\nshadow status={status} "
            f"({record['shadow_status_reason']})"
        )
        print(
            f"present_mask left=0x{snapshot.left_present_mask:02X} "
            f"right=0x{snapshot.right_present_mask:02X} (both must be 0x3F)"
        )
        # status 2..5 는 관절 loop 앞에서 끝난 실패다. 그때 raw 배열과
        # present_mask 는 채워지지 않으므로 관절별 판정을 하면 안 된다.
        # 없는 데이터로 표를 그리면 원인을 엉뚱한 관절로 오인하게 된다.
        if status in (1, 2, 3, 4, 5, 7):
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_BUS_OR_POWER"
            print(
                "\n이 실패는 관절 위치 판정 이전 단계다. servo bus 가 응답하지 "
                "않았다는 뜻이며, raw 배열과 present_mask 는 채워지지 않는다.\n"
                "확인할 것: 양팔 servo 12 V 전원, 버스 배선, 중복 resident 부재.\n"
                "펌웨어가 stop 을 latch 했으므로 다음 시도 전에 STM32 RESET 이 "
                "필요하다."
            )
            return 3

        contracts = load_contracts()
        if contracts is None:
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_RAW_ONLY_NO_CONTRACTS"
            print(
                f"\n{OPERATIONAL_LIMITS} 와 {CALIBRATION} 가 이 기계에 없다. "
                "관절별 판정은 건너뛰고 raw 만 출력한다."
            )
            print(f"\n{'idx':>3} {'arm':>5} {'joint':>11} {'raw':>6}")
            for index in range(JOINT_COUNT):
                arm = "left" if index < 6 else "right"
                short = SHORT_NAMES[index % 6]
                print(
                    f"{index:>3} {arm:>5} {short[0]:>11} "
                    f"{int(snapshot.positions_raw[index]):>6}"
                )
            return 0

        print(
            f"\n{'idx':>3} {'arm':>5} {'joint':>11} {'raw':>6} "
            f"{'allowed raw window':>26} {'verdict':>9}"
        )
        offenders = []
        joints = []
        for index in range(JOINT_COUNT):
            arm = "left" if index < 6 else "right"
            short = SHORT_NAMES[index % 6]
            low, high = unwrapped_raw_window(arm, short)
            observed = int(snapshot.positions_raw[index])
            verdict = binding_verdict(observed, low, high)
            window = f"[{low:.0f}, {high:.0f}]"
            if high > RAW_MODULUS or low < 0:
                window += " wrap"
            mark = "OK" if verdict["binds"] else "**FAIL**"
            print(
                f"{index:>3} {arm:>5} {short[0]:>11} {observed:>6} "
                f"{window:>26} {mark:>9}"
            )
            entry = {
                "index": index,
                "arm": arm,
                "joint": short[0],
                "observed_raw": observed,
                "allowed_window_raw": [low, high],
                **verdict,
            }
            joints.append(entry)
            if not verdict["binds"]:
                offenders.append(entry)
        record["joints"] = joints
        record["offending_joints"] = offenders

        if offenders:
            print("\n원인 관절:")
            for entry in offenders:
                low, high = entry["allowed_window_raw"]
                print(
                    f"  {entry['arm']} {entry['joint']}: "
                    f"raw={entry['observed_raw']} is outside [{low:.0f}, "
                    f"{high:.0f}] -> {entry['cause']}"
                )
            print(
                "\n이 관절을 허용 구간 안으로 손으로 옮긴 뒤 STM32 RESET하고 "
                "resident를 다시 띄운다."
            )
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_FOUND_OFFENDER"
        elif status != 0:
            print(
                "\n모든 관절이 한계 안이지만 펌웨어가 실패했다. unwrap 이후의 "
                "보정 변환 또는 다른 상태 문제다. 위 status 코드를 보고한다."
            )
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_UNEXPLAINED"
        else:
            print("\nshadow anchor는 정상이다. resident를 띄울 수 있다.")
            record["status"] = "SHADOW_ANCHOR_DIAGNOSIS_PASS"
        if status != 0:
            print(
                "주의: status != 0 이므로 펌웨어가 stop을 latch했다. "
                "다음 실행 전에 STM32 RESET이 필요하다."
            )
        return 0
    finally:
        port.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(record, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(f"\nartifact={args.output}\nsha256={digest}")


if __name__ == "__main__":
    raise SystemExit(main())
