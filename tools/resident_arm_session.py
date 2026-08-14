#!/usr/bin/env python3
"""H1: 상주 ROS 세션.

`move_to()`가 leg 마다 subprocess 3개(ROS 노드 생성 + MoveIt/action 탐색을
매번 새로 하는)를 띄우는 구조가 leg 간격 표준 중앙값 `4137 ms`의 대부분을
차지한다(`docs/PLAN_CONTINUOUS_EXECUTION.md` §1 실측, `precompute_ms`는
`3.7~330 ms`뿐이었다). 이 모듈은 rclpy 컨텍스트, MoveIt 서비스 클라이언트,
buffered action 클라이언트, `/joint_states` 구독을 **세션 내내 하나씩만**
유지해 그 dead time을 없앤다.

기존 subprocess 기반 CLI 도구는 건드리지 않는다. 이미 그 도구들이 분리해
둔 순수 함수(`plan_segment`, `build_plan`, `send_goal_once`,
`wait_joint_state`, `validate_fresh_start`, `build_goal`,
`validate_action_terminal`)를 그대로 재사용하고, rclpy 라이프사이클만
세션 단위로 끌어올린다 — `tools/resident_pick_place.py` 가 그 조립을 한다.
"""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

FOLLOW_JOINT_TRAJECTORY_ACTION = "/left_arm_controller/follow_joint_trajectory"
PLAN_SERVICE = "/plan_kinematic_path"
JOINT_STATE_TOPIC = "/joint_states"


class ResidentSessionError(RuntimeError):
    """세션 초기화나 서버 연결 실패."""


class ResidentArmSession:
    """rclpy 컨텍스트 하나, 노드 하나, 클라이언트 둘을 세션 내내 유지한다.

    `close()`를 반드시 호출한다 (또는 `with` 로 사용한다). 한 팔 전용이다
    — 양팔은 F7의 12관절 단일 queue 설계를 따로 반영해야 한다.
    """

    def __init__(
        self,
        *,
        node_name: str = "resident_pick_place_session",
        follow_joint_trajectory_action: str = FOLLOW_JOINT_TRAJECTORY_ACTION,
        plan_service: str = PLAN_SERVICE,
        service_timeout_s: float = 10.0,
        action_timeout_s: float = 10.0,
    ) -> None:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from moveit_msgs.srv import GetMotionPlan
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState

        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self._closed = False

        self._latest_joint_state: dict[str, float] = {}
        # JointState는 부분 벡터일 수도 있다. 메시지 전체에 timestamp 하나만
        # 두면 새 BASE 한 점이 도착했을 때 다른 다섯 관절의 오래된 캐시까지
        # fresh로 오인한다. 요청된 관절 *각각*이 호출 이후 갱신됐는지 본다.
        self._joint_state_stamps_s: dict[str, float] = {}
        self._joint_state_subscription = self.node.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._on_joint_state,
            10,
        )

        self.moveit_client = self.node.create_client(GetMotionPlan, plan_service)
        if not self.moveit_client.wait_for_service(timeout_sec=service_timeout_s):
            self.close()
            raise ResidentSessionError(
                f"MoveIt plan service unavailable: {plan_service}"
            )

        self.action_client = ActionClient(
            self.node, FollowJointTrajectory, follow_joint_trajectory_action
        )
        if not self.action_client.wait_for_server(timeout_sec=action_timeout_s):
            self.close()
            raise ResidentSessionError(
                "FollowJointTrajectory Action is unavailable: "
                f"{follow_joint_trajectory_action}"
            )

    def _on_joint_state(self, message: Any) -> None:
        stamp = time.monotonic()
        for name, position in zip(message.name, message.position):
            self._latest_joint_state[name] = position
            self._joint_state_stamps_s[name] = stamp

    def spin_once(self, timeout_s: float = 0.05) -> None:
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def wait_joint_state(
        self,
        names: tuple[str, ...],
        timeout_s: float = 5.0,
    ) -> tuple[float, ...]:
        """이번 호출 *이후* 갱신된 값만 돌려준다.

        원본 `wait_joint_state(node, names)`는 그때그때 새 구독을 만들어
        "구독 이후 첫 메시지"를 기다린다 — 항상 신선하다. 여기서는 구독을
        세션 내내 유지하는 대신 그 신선도 계약을 시각 비교로 재현한다.
        오래된 캐시값을 몰래 돌려주지 않는다.
        """
        import rclpy

        start = time.monotonic()
        deadline = start + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if all(
                name in self._latest_joint_state
                and self._joint_state_stamps_s.get(name, 0.0) > start
                for name in names
            ):
                return tuple(self._latest_joint_state[name] for name in names)
            self.spin_once(min(0.05, max(0.0, deadline - time.monotonic())))
        raise TimeoutError("/joint_states did not refresh within timeout")

    def wait_future(self, future: Any, timeout_s: float) -> Any:
        import rclpy

        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("ROS future timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS future completed without a result")
        return result

    def close(self) -> None:
        if self._closed:
            return
        import rclpy

        action_client = getattr(self, "action_client", None)
        if action_client is not None:
            action_client.destroy()
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
        self._closed = True

    def __enter__(self) -> "ResidentArmSession":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


if __name__ == "__main__":
    raise SystemExit(
        "resident_arm_session.py는 라이브러리다. "
        "run_pick_place_once_resident.py 를 실행할 것."
    )
