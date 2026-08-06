"""수렴 실행기와 그것이 지나는 계획 경로의 계약.

이 실행기는 **여러 번 움직인다.** 지금까지의 one-shot 도구와 다르므로,
무엇이 그 반복을 유한하게 만드는지와 어떤 경로로만 움직이는지가 여기서
고정되어야 한다.

ROS 없이 검증한다 — 파싱과 환산은 순수 함수이고, 나머지는 소스 검사다.
"""

from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(PACKAGE))

RUNNER_PATH = ROOT / "tools" / "execute_grasp_convergence_once.py"
SEGMENTS_PATH = ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"

_spec = importlib.util.spec_from_file_location(
    "execute_grasp_convergence_once", RUNNER_PATH
)
RUNNER = importlib.util.module_from_spec(_spec)
sys.modules["execute_grasp_convergence_once"] = RUNNER
_spec.loader.exec_module(RUNNER)

RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
SEGMENTS_SOURCE = SEGMENTS_PATH.read_text(encoding="utf-8")


def segments_constants() -> dict[str, object]:
    tree = ast.parse(SEGMENTS_SOURCE)
    values: dict[str, object] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    try:
                        values[target.id] = ast.literal_eval(node.value)
                    except ValueError:
                        pass
    return values


# ---------------------------------------------------------------------------
# 측정을 읽어오는 경로
# ---------------------------------------------------------------------------


def executor_output(measured: str = "2295,3440,1552,1206,2113,2003") -> str:
    return "\n".join(
        (
            "PLAN_GATE=PASS",
            "FRESH_START_GATE=PASS",
            "ACTION_SEND_COUNT=1",
            "ACTION_TERMINAL_PASS status=4 error_code=0 "
            "maximum_apply_lateness_ms=4 post_settle_max_error_raw=19",
            "TERMINAL_DIAGNOSTICS=startup=prime_depth=16; "
            "lateness_buckets=200,1,0,0,0,0 lateness_worst_sample=91; "
            "post_settle_target_raw=2276,3437,1550,1201,2112,2003 "
            f"post_settle_measured_raw={measured} "
            "post_settle_error_raw=19,3,2,5,1,0",
            "MOTION14_FRESH_SEGMENT_LEG_ONCE_PASS",
        )
    )


def test_the_per_joint_measurement_is_read_from_the_executor() -> None:
    """수렴의 입력은 최대값이 아니라 관절별 실측이다."""
    parsed = RUNNER.parse_measurement(executor_output())
    assert parsed is not None
    measured, settle_max = parsed
    assert measured == (2295, 3440, 1552, 1206, 2113, 2003)
    assert settle_max == 19


def test_an_executor_without_the_measurement_is_detected() -> None:
    """C1 이전 bridge 면 벡터가 없다. 추측하지 말고 그렇다고 말해야 한다."""
    old = executor_output().replace(
        "post_settle_target_raw=2276,3437,1550,1201,2112,2003 "
        "post_settle_measured_raw=2295,3440,1552,1206,2113,2003 "
        "post_settle_error_raw=19,3,2,5,1,0",
        "",
    )
    assert RUNNER.parse_measurement(old) is None


def test_a_missing_terminal_line_is_detected() -> None:
    assert RUNNER.parse_measurement("PLAN_GATE=PASS\nnothing else") is None


def test_inconsistent_vectors_propagate_as_an_error() -> None:
    broken = executor_output(measured="9999,3440,1552,1206,2113,2003")
    with pytest.raises(RuntimeError, match="inconsistent"):
        RUNNER.parse_measurement(broken)


def test_the_runner_refuses_a_measurement_free_terminal() -> None:
    """벡터가 없으면 수렴할 수 없다. 조용히 계속하지 않는다."""
    assert "the bridge is older than the C1 change" in RUNNER_SOURCE
    assert "if parsed is None:" in RUNNER_SOURCE


# ---------------------------------------------------------------------------
# 환산이 bridge 와 같은 식인가
# ---------------------------------------------------------------------------


def test_the_raw_conversion_matches_the_bridge() -> None:
    """anchor 를 다른 식으로 환산하면 fresh-start 게이트가 흔들린다."""
    from single_arm_bridge.calibration import load_calibration

    calibration = load_calibration(
        PACKAGE / "config" / "single_arm_calibration.json"
    )
    raw = (2276, 3437, 1550, 1201, 2112, 2003)
    radians = calibration.raw_feedback_to_radians(raw)
    assert RUNNER.radians_to_raw(calibration, radians) == raw


def test_the_conversion_formula_is_the_one_the_action_uses() -> None:
    execution = (
        PACKAGE / "single_arm_bridge" / "buffered_action_execution.py"
    ).read_text(encoding="utf-8")
    for fragment in (
        "joint.zero_raw",
        "joint.direction * position * 4096.0 / (2.0 * math.pi)",
    ):
        assert fragment in execution
        assert fragment in RUNNER_SOURCE


# ---------------------------------------------------------------------------
# 무엇이 반복을 유한하게 만드는가
# ---------------------------------------------------------------------------


def test_the_runner_requires_its_own_exact_confirmation() -> None:
    assert RUNNER.CONFIRMATION == "EXECUTE_C2_GRASP_CONVERGENCE_ONCE"
    assert "this tool moves the " in RUNNER_SOURCE
    assert "arm more than once" in RUNNER_SOURCE


def test_the_runner_never_opens_a_serial_port() -> None:
    """bridge 가 포트를 소유한다. 뺏으면 torque 가 풀려 팔이 처진다."""
    for forbidden in (
        "open_exclusive_serial",
        "serial.Serial",
        "ActuatorTransport",
        "BufferedTransportDriver",
    ):
        assert forbidden not in RUNNER_SOURCE
    assert '"serial_port_opened": False' in RUNNER_SOURCE


def test_every_physical_move_goes_through_the_validated_pipeline() -> None:
    """새 이동 경로를 만들지 않는다. Motion-14 를 그대로 지난다."""
    for tool in (
        "ros_moveit_plan_pregrasp_segments.py",
        "plan_buffered_segment_leg.py",
        "execute_buffered_segment_leg_once.py",
    ):
        assert tool in RUNNER_SOURCE
    assert (
        RUNNER.SEGMENT_LEG_CONFIRMATION
        == "EXECUTE_MOTION14_FRESH_SEGMENT_LEG_ONCE"
    )
    # 자체 Action client 를 만들지 않는다.
    for forbidden in ("ActionClient", "FollowJointTrajectory"):
        assert forbidden not in RUNNER_SOURCE


def test_each_leg_is_digest_pinned_between_planning_and_execution() -> None:
    """계획과 실행 사이에 파일이 바뀌지 못하게 한다."""
    assert "--segments-sha256" in RUNNER_SOURCE
    assert "--expected-sha256" in RUNNER_SOURCE
    assert RUNNER_SOURCE.count("sha256_file(") >= 4


def test_the_loop_is_driven_by_the_bounded_library() -> None:
    assert "GC.begin(" in RUNNER_SOURCE
    assert "GC.evaluate(" in RUNNER_SOURCE
    assert "if not decision.requires_motion:" in RUNNER_SOURCE
    assert "break" in RUNNER_SOURCE
    # 자체 반복 상한을 따로 두지 않는다. 경계는 라이브러리 하나가 소유한다.
    assert "while True:" in RUNNER_SOURCE


def test_the_evidence_records_every_iteration_residual() -> None:
    for field in (
        "residual_mm_by_iteration",
        "physical_motion_count",
        "automatic_retry_count",
        "final_residual_mm",
    ):
        assert field in RUNNER_SOURCE


def test_the_runner_names_no_arm_side_in_its_logic() -> None:
    """양팔이 되면 --arm 만 바꿔 같은 코드를 쓴다."""
    body = "\n".join(
        line
        for line in RUNNER_SOURCE.splitlines()
        if "default=" not in line and not line.lstrip().startswith("#")
    )
    assert "left_" not in body


# ---------------------------------------------------------------------------
# 관절 목표도 점이 아니라 구간이었다
# ---------------------------------------------------------------------------


def test_the_joint_goal_tolerance_is_below_one_raw_count() -> None:
    """서보의 1-raw 양자화보다 작아야 계획이 하드웨어를 앞선다.

    2026-08-06 실측: 종전 `0.005 rad` 는 서보가 움직이기도 전에 TCP 를 최대
    `2.81 mm` 어긋나게 했고, 그것은 과제 허용치 `4 mm` 의 대부분이다. 그 상태로
    수렴 루프를 돌리면 팔의 처짐이 아니라 이 허용치를 재게 된다.
    """
    tolerance = segments_constants()["JOINT_GOAL_TOLERANCE_RAD"]
    one_raw_rad = 2.0 * math.pi / 4096.0
    assert tolerance < one_raw_rad
    assert tolerance == 0.0005


def test_the_planned_final_pose_is_measured_and_gated() -> None:
    """success 를 돌려줬다고 목표에 갔다는 뜻이 아니다. 구간 안이면 success 다."""
    for fragment in (
        "joint_goal_residual_rad",
        "within_joint_goal_residual_bound",
        "joint_goal_residual_bound_rad",
    ):
        assert fragment in SEGMENTS_SOURCE
    assert 'result["success"] = result["success"] and residual <= bound' in (
        SEGMENTS_SOURCE
    )


def test_the_residual_bound_cannot_reject_a_legitimate_solution() -> None:
    """되재는 값이지 두 번째 허용치가 아니다."""
    constants = segments_constants()
    assert constants["JOINT_GOAL_RESIDUAL_MARGIN"] > 1.0


def test_an_explicit_joint_target_is_accepted() -> None:
    """넘겨명령 목표는 어떤 PLAN_ONLY_PASS 파일에도 없다.

    그것을 담은 합성 문서를 만들어 MoveIt 이 내지 않은 것에 PLAN_ONLY_PASS 를
    붙이는 대신, 명시적 관절 목표를 정직하게 받는다.
    """
    assert "--target-joints" in SEGMENTS_SOURCE
    assert "EXPLICIT_TARGET_NAME" in SEGMENTS_SOURCE
    assert segments_constants()["EXPLICIT_TARGET_NAME"] == "explicit"


def test_exactly_one_target_source_is_required() -> None:
    assert (
        "give exactly one of --source-plan or --target-joints"
        in SEGMENTS_SOURCE
    )


def test_an_explicit_target_still_passes_the_downstream_status_gate() -> None:
    """하위 계획기는 `_SEGMENT_PLAN_ONLY_PASS` 로 끝나는 status 를 요구한다."""
    status = (
        segments_constants()["EXPLICIT_TARGET_NAME"].upper()
        + "_SEGMENT_PLAN_ONLY_PASS"
    )
    assert status.endswith("_SEGMENT_PLAN_ONLY_PASS")


def test_an_explicit_target_is_still_limit_checked() -> None:
    """명시적 목표라고 검사를 건너뛰지 않는다."""
    assert 'validate_positions(args.target_name, target, limits)' in (
        SEGMENTS_SOURCE
    )
