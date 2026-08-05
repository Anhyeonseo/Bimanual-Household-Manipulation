#!/usr/bin/env python3
"""
gripper 명령을 자동 재시도 없이 1회 전송하고 결과를 전부 보고한다.

Motion-13 은 Pick/Place 를 buffered leg 3개로 나누고 경계에 gripper 동작을
둔다. 접촉이 load/current 감시가 있는 이 경로에서 일어나게 하려는 것이다
(`tools/plan_buffered_pick_place_leg.py` 서두 참조).

**물체를 문 gripper 가 무엇을 보고하는지는 아직 실측된 적이 없다.**

`ParallelGripperCommandActionAdapter._finish_goal` 은 실행이 SUCCEEDED 일
때만 `reached_goal=True` 를 낸다. 그런데 firmware 의 최종 정착 검사는
`SERVO_FINAL_ERROR_TOLERANCE_RAW = 30` 이고, open(raw 2009) 과
close(raw 1963) 사이 전체 이동량이 46 raw 뿐이다. 물체 두께가 그 46 중
얼마를 막느냐에 따라 정착 오차가 30 을 넘을 수도, 안 넘을 수도 있다.
넘으면 abort 이고 stop latch 까지 갈 수 있다.

Stage 7 의 supervised 실행은 "gripper close and verified object hold" 를
기록했지만 명령값도 action 결과도 남기지 않았다. 그래서 이 도구는
`--expect report` 로 먼저 관측하는 것을 기본으로 하고, 관측된 뒤에야
`contact` 나 `reached` 로 게이트를 건다.

`stalled` 는 현재 adapter 가 항상 False 로 둔다. 판정에 쓰지 않고 보고만 한다.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from execute_buffered_q0_roundtrip_once import wait_joint_state
from plan_buffered_q0_roundtrip import radians_to_raw


ACTION_NAME = "/left_gripper_controller/gripper_cmd"
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 30.0

CONFIRMATIONS = {
    "pick_close": "EXECUTE_MOTION13_GRIPPER_PICK_CLOSE_ONCE",
    "place_release": "EXECUTE_MOTION13_GRIPPER_PLACE_RELEASE_ONCE",
    "probe": "EXECUTE_MOTION13_GRIPPER_PROBE_ONCE",
}

# firmware 의 최종 정착 허용치. 이 도구는 이 값을 강제하지 않고, 관측된
# 잔여 간격을 이 값과 나란히 보고해 판정 근거를 남긴다.
FIRMWARE_SETTLE_TOLERANCE_RAW = 30

# gripper 만 움직여야 한다. 팔 축이 이만큼 넘게 움직였으면 비명령 동작이다.
ARM_MOTION_LIMIT_RAD = 0.02


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws" / "src" / "single_arm_bridge"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", choices=sorted(CONFIRMATIONS), required=True)
    parser.add_argument("--position-rad", type=float, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--duration-ms", type=int, default=1000)
    parser.add_argument(
        "--expect",
        choices=("report", "reached", "contact"),
        default="report",
        help=(
            "report: 관측만 한다. reached: 명령 위치 도달을 요구한다. "
            "contact: 물체에 막혀 도달하지 못했음을 요구한다"
        ),
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=package / "config" / "single_arm_calibration.json",
    )
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATIONS[arguments.label]:
        parser.error(
            f"{arguments.label} requires its own confirmation: "
            f"{CONFIRMATIONS[arguments.label]}"
        )
    if not 300 <= arguments.duration_ms <= 2000:
        parser.error("gripper duration must be within 300..2000 ms")
    return arguments


def main() -> int:
    from control_msgs.action import ParallelGripperCommand
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    import sys

    sys.path.insert(
        0,
        str(Path(__file__).resolve().parents[1] / "ros2_ws/src/single_arm_bridge"),
    )
    from single_arm_bridge.calibration import load_calibration

    arguments = parse_args()
    calibration = load_calibration(arguments.calibration)
    joint_names = tuple(calibration.ros_joint_names)
    gripper_name = joint_names[5]
    lower, upper = calibration.ros_radian_limits[gripper_name]
    if not lower <= arguments.position_rad <= upper:
        raise SystemExit(
            f"commanded gripper position is outside [{lower}, {upper}]"
        )

    def to_raw(position_rad: float) -> int:
        return radians_to_raw(calibration, (0.0,) * 5 + (position_rad,))[5]

    commanded_raw = to_raw(arguments.position_rad)

    rclpy.init()
    node = Node(f"motion13_gripper_{arguments.label}_once")
    action_client = ActionClient(node, ParallelGripperCommand, ACTION_NAME)
    try:
        before = wait_joint_state(node, joint_names)
        print(f"LABEL={arguments.label}")
        print(f"COMMANDED_RAD={arguments.position_rad:.6f}")
        print(f"COMMANDED_RAW={commanded_raw}")
        print(f"BEFORE_GRIPPER_RAD={before[5]:.6f}")
        print(f"BEFORE_GRIPPER_RAW={to_raw(before[5])}")
        if not action_client.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_S
        ):
            raise RuntimeError("ParallelGripperCommand Action is unavailable")

        goal = ParallelGripperCommand.Goal()
        goal.command = JointState()
        goal.command.name = [gripper_name]
        goal.command.position = [float(arguments.position_rad)]
        print("ACTION_SEND_COUNT=1")

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("gripper goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            node, result_future, timeout_sec=ACTION_RESULT_TIMEOUT_S
        )
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("gripper result did not arrive within the budget")

        status = wrapped.status
        result = wrapped.result
        after = wait_joint_state(node, joint_names)
        after_raw = to_raw(after[5])
        residual_raw = abs(after_raw - commanded_raw)
        arm_motion = max(
            abs(a - b) for a, b in zip(before[:5], after[:5], strict=True)
        )

        print(f"ACTION_STATUS={status}")
        print(f"REACHED_GOAL={bool(result.reached_goal)}")
        print(f"STALLED={bool(result.stalled)}  (adapter 는 항상 False 로 둔다)")
        print(f"AFTER_GRIPPER_RAD={after[5]:.6f}")
        print(f"AFTER_GRIPPER_RAW={after_raw}")
        print(f"RESIDUAL_GAP_RAW={residual_raw}")
        print(
            f"FIRMWARE_SETTLE_TOLERANCE_RAW={FIRMWARE_SETTLE_TOLERANCE_RAW}"
        )
        print(f"ARM_MOTION_RAD={arm_motion:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")

        if not math.isfinite(arm_motion) or arm_motion > ARM_MOTION_LIMIT_RAD:
            print("VERDICT=ARM_MOVED_UNCOMMANDED")
            raise SystemExit(
                f"arm axes moved {arm_motion:.6f} rad during a gripper command"
            )

        if residual_raw > FIRMWARE_SETTLE_TOLERANCE_RAW:
            print("CONTACT_EVIDENCE=residual gap exceeds the settle tolerance")
        elif residual_raw > 0:
            print("CONTACT_EVIDENCE=residual gap inside the settle tolerance")
        else:
            print("CONTACT_EVIDENCE=none; the gripper reached the command")

        if arguments.expect == "reached":
            if not result.reached_goal:
                print("VERDICT=FAIL_DID_NOT_REACH")
                raise SystemExit("gripper did not reach the commanded position")
            print("VERDICT=REACHED")
        elif arguments.expect == "contact":
            if result.reached_goal or residual_raw == 0:
                print("VERDICT=FAIL_NO_CONTACT")
                raise SystemExit(
                    "gripper reached its command; nothing is held"
                )
            print("VERDICT=CONTACT")
        else:
            print("VERDICT=REPORTED (게이트 없음; 관측이 목적이다)")

        print(f"MOTION13_GRIPPER_{arguments.label.upper()}_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
