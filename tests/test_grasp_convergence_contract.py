"""수렴 계층의 경계값이 계약과 코드에서 같은 값인지 강제한다.

계약에 적힌 수와 코드가 쓰는 수가 갈라지면 계약은 장식이 된다. 여기서는
`tools/grasp_convergence.py` 와 `tools/ros_moveit_plan_grasp.py` 의 상수를
소스에서 읽어 계약과 대조하고, 계약의 각 게이트가 실제로 무엇을 거부하는지
하나씩 확인한다.

소스 파싱 시험 선례: tests/test_buffered_terminal_format_contract.py,
tests/test_left_arm_q0_contract.py
"""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from single_arm_bridge.buffered_action_execution import (  # noqa: E402
    POST_SETTLE_TOLERANCE_RAW,
)
from single_arm_bridge.buffered_trajectory import (  # noqa: E402
    BufferedTrajectoryContractError,
    validate_buffered_trajectory_contract,
)

CONTRACT_PATH = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
PLANNER_PATH = ROOT / "tools" / "ros_moveit_plan_grasp.py"

_spec = importlib.util.spec_from_file_location(
    "grasp_convergence", ROOT / "tools" / "grasp_convergence.py"
)
GC = importlib.util.module_from_spec(_spec)
sys.modules["grasp_convergence"] = GC
_spec.loader.exec_module(GC)


def module_constants(path: Path) -> dict[str, object]:
    """모듈을 import 하지 않고 최상위 상수만 읽는다.

    `ros_moveit_plan_grasp.py` 는 rclpy 를 import 하므로 ROS 없이 돌 수 있는
    시험으로 두려면 소스를 파싱해야 한다.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    values: dict[str, object] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                try:
                    values[target.id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    return values


@pytest.fixture(scope="module")
def document() -> dict:
    return json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def convergence(document) -> dict:
    return document["grasp_convergence"]


def mutated(document: dict, **overrides) -> dict:
    changed = copy.deepcopy(document)
    changed["grasp_convergence"].update(overrides)
    return changed


# ---------------------------------------------------------------------------
# 계약과 코드가 같은 수를 쓰는가
# ---------------------------------------------------------------------------


def test_the_contract_validates_as_shipped(document) -> None:
    validate_buffered_trajectory_contract(document)


@pytest.mark.parametrize(
    "contract_key,module_name",
    (
        ("task_tolerance_m", "TASK_TOLERANCE_M"),
        ("maximum_iterations", "MAXIMUM_ITERATIONS"),
        ("maximum_correction_m", "MAXIMUM_CORRECTION_M"),
        ("plateau_improvement_m", "PLATEAU_IMPROVEMENT_M"),
        ("maximum_overshoot_m", "MAXIMUM_OVERSHOOT_M"),
        ("divergence_ratio", "DIVERGENCE_RATIO"),
    ),
)
def test_every_bound_matches_the_library(
    convergence, contract_key, module_name
) -> None:
    assert convergence[contract_key] == getattr(GC, module_name)


def test_the_default_policy_uses_exactly_those_bounds() -> None:
    """상수를 정의해두고 기본값을 따로 쓰면 계약이 아무것도 강제하지 못한다."""
    policy = GC.ConvergencePolicy(arm="left")
    assert policy.task_tolerance_m == GC.TASK_TOLERANCE_M
    assert policy.maximum_iterations == GC.MAXIMUM_ITERATIONS
    assert policy.maximum_correction_m == GC.MAXIMUM_CORRECTION_M
    assert policy.plateau_improvement_m == GC.PLATEAU_IMPROVEMENT_M
    assert policy.maximum_overshoot_m == GC.MAXIMUM_OVERSHOOT_M


def test_the_plan_tolerance_matches_the_planner(convergence) -> None:
    constants = module_constants(PLANNER_PATH)
    assert (
        convergence["plan_position_tolerance_m"]
        == constants["DEFAULT_POSITION_TOLERANCE_M"]
    )


def test_the_planner_default_argument_uses_that_constant() -> None:
    """argparse 기본값에 숫자를 다시 적으면 상수가 장식이 된다."""
    source = PLANNER_PATH.read_text(encoding="utf-8")
    assert "default=DEFAULT_POSITION_TOLERANCE_M," in source
    assert "default=0.006" not in source


def test_the_safety_tolerance_matches_the_action(convergence) -> None:
    assert convergence["safety_tolerance_raw"] == POST_SETTLE_TOLERANCE_RAW


def test_the_two_tolerances_have_different_owners(convergence) -> None:
    """하나의 수가 두 질문에 답하고 있었다. 그것이 문제의 절반이었다."""
    assert convergence["safety_tolerance_owner"] != (
        convergence["task_tolerance_owner"]
    )
    assert "Action" in convergence["safety_tolerance_owner"]
    assert "task" in convergence["task_tolerance_owner"]
    assert convergence["safety_tolerance_question"] != (
        convergence["task_tolerance_question"]
    )


# ---------------------------------------------------------------------------
# 각 게이트가 실제로 거부하는가
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "overrides,message",
    (
        ({"tolerances_separated": False}, "must stay separated"),
        ({"safety_tolerance_raw": 6}, "match the Action's constant"),
        ({"safety_tolerance_unchanged": False}, "must not be tightened"),
        ({"task_tolerance_m": 0.0}, "positive and tighter"),
        ({"task_tolerance_m": 0.05}, "positive and tighter"),
        ({"motion_authorized": True}, "motion_authorized=false"),
        ({"budget_closes": False}, "budget closes"),
        ({"plan_position_tolerance_m": 0.02}, "tighter than the value"),
        (
            {"plan_residual_recorded_in_artifact": False},
            "record the residual it was allowed to have",
        ),
        ({"maximum_overshoot_m": 0.010}, "largest measured residual"),
        ({"maximum_overshoot_m": 0.050}, "largest measured residual"),
        ({"overshoot_used_at_most_once": False}, "single use"),
        (
            {"error_measured_against_nominal_goal": False},
            "against the original goal",
        ),
        (
            {"fail_closed_on_non_convergence": False},
            "not retry silently",
        ),
        ({"bridge_action_unchanged": False}, "must not be changed"),
        (
            {"convergence_lives_in_host_library": False},
            "outside the bridge",
        ),
        ({"per_arm_policy": False}, "bimanual-shaped now: per_arm_policy"),
        ({"arm_name_required": False}, "bimanual-shaped now: arm_name_required"),
        (
            {"send_and_evaluate_separated": False},
            "bimanual-shaped now: send_and_evaluate_separated",
        ),
        (
            {"coordinated_stop_input": False},
            "bimanual-shaped now: coordinated_stop_input",
        ),
        (
            {"per_joint_post_settle_recorded": False},
            "bimanual-shaped now: per_joint_post_settle_recorded",
        ),
    ),
)
def test_each_gate_refuses(document, overrides, message) -> None:
    with pytest.raises(BufferedTrajectoryContractError, match=message):
        validate_buffered_trajectory_contract(mutated(document, **overrides))


def test_a_budget_that_reaches_the_failure_point_is_refused(document) -> None:
    """A4 스윕이 위로 8 mm 를 확실한 실패점으로 남겼다.

    계획 잔차와 과제 허용치의 합이 거기 닿으면 파지가 성공할 이유가 없다.
    """
    with pytest.raises(
        BufferedTrajectoryContractError, match="grasp failure point"
    ):
        validate_buffered_trajectory_contract(
            mutated(document, task_tolerance_m=0.0075)
        )


def test_a_plan_residual_that_did_not_improve_is_refused(document) -> None:
    changed = copy.deepcopy(document)
    changed["grasp_convergence"]["plan_residual_measured_m"]["pregrasp"] = 0.007
    with pytest.raises(
        BufferedTrajectoryContractError, match="pregrasp plan residual did not"
    ):
        validate_buffered_trajectory_contract(changed)


def test_the_budget_gate_is_stricter_than_the_improvement_gate(
    document,
) -> None:
    """grasp 잔차가 나빠지면 개선 게이트보다 예산 게이트가 먼저 걸린다.

    종전 grasp 잔차 `0.004599` 는 이미 과제 허용치 `0.004` 와 합쳐 실패점
    `0.008` 을 넘는다. 즉 상자를 조이기 전의 계획으로는 예산이 애초에 닫히지
    않았다. 그것이 2026-08-06 A4.5 가 실패한 조건이다.
    """
    changed = copy.deepcopy(document)
    changed["grasp_convergence"]["plan_residual_measured_m"]["grasp"] = (
        changed["grasp_convergence"]["previous_plan_residual_measured_m"][
            "grasp"
        ]
    )
    with pytest.raises(
        BufferedTrajectoryContractError, match="grasp failure point"
    ):
        validate_buffered_trajectory_contract(changed)


# ---------------------------------------------------------------------------
# 기록된 실측이 실제 관측과 일치하는가
# ---------------------------------------------------------------------------


def test_the_recorded_plan_residuals_are_below_the_tolerance_they_used(
    convergence,
) -> None:
    """상자 안에 들어갔다는 주장이 산술로 성립해야 한다.

    상자의 모서리 거리는 `tolerance x sqrt(3)` 이다.
    """
    tightened = convergence["plan_position_tolerance_m"] * 3**0.5
    previous = convergence["previous_plan_position_tolerance_m"] * 3**0.5
    for pose in ("pregrasp", "grasp"):
        assert convergence["plan_residual_measured_m"][pose] <= tightened
        assert (
            convergence["previous_plan_residual_measured_m"][pose] <= previous
        )


def test_tightening_the_box_removed_more_than_three_millimetres(
    convergence,
) -> None:
    """무엇을 얻었는지 수로 남긴다."""
    gained = (
        convergence["previous_plan_residual_measured_m"]["grasp"]
        - convergence["plan_residual_measured_m"]["grasp"]
    )
    assert gained > 0.003


def test_the_hardware_confirmation_is_still_open(convergence) -> None:
    """과제 허용치는 아직 실기에서 확인되지 않았다. 그렇게 적혀 있어야 한다."""
    assert convergence["task_tolerance_confirmed_on_hardware"] is False
    assert convergence["deployed"] is False
    assert convergence["motion_authorized"] is False


# ---------------------------------------------------------------------------
# C2 실측이 계약에 반영됐는가
# ---------------------------------------------------------------------------


def test_the_ineffective_correction_threshold_matches_the_library(
    convergence,
) -> None:
    assert (
        convergence["ineffective_correction_raw"]
        == GC.INEFFECTIVE_CORRECTION_RAW
    )


def test_the_threshold_is_not_below_the_repository_measurement(
    convergence,
) -> None:
    """저장소가 독립적으로 잰 최소 관측 가능 명령보다 낮으면 안 된다."""
    delta_source = (
        ROOT / "tools" / "execute_buffered_joint_delta_once.py"
    ).read_text(encoding="utf-8")
    assert (
        f"MINIMUM_OBSERVABLE_COMMAND_RAW = "
        f"{convergence['minimum_observable_command_raw']}" in delta_source
    )
    assert (
        convergence["minimum_observable_command_raw"]
        <= convergence["ineffective_correction_raw"]
    )


@pytest.mark.parametrize(
    "overrides,message",
    (
        (
            {"minimum_observable_command_raw": 40},
            "must not sit below the measured minimum observable command",
        ),
        (
            {"overshoot_is_the_only_supra_threshold_command": False},
            "overshoot_is_the_only_supra_threshold_command",
        ),
        (
            {"overshoot_clamped_at_joint_limits": False},
            "overshoot_clamped_at_joint_limits",
        ),
        ({"clamped_joints_reported": False}, "clamped_joints_reported"),
    ),
)
def test_each_c2_gate_refuses(document, overrides, message) -> None:
    with pytest.raises(BufferedTrajectoryContractError, match=message):
        validate_buffered_trajectory_contract(mutated(document, **overrides))


def test_the_hardware_confirmation_remains_open_after_the_first_c2_run(
    convergence,
) -> None:
    """첫 C2 회차는 수렴하지 못했다. 그렇게 적혀 있어야 한다."""
    assert convergence["c2_first_correction_was_ineffective"] is True
    assert convergence["task_tolerance_confirmed_on_hardware"] is False


def test_the_limit_margin_matches_the_library(convergence) -> None:
    assert (
        convergence["joint_limit_margin_rad"] == GC.JOINT_LIMIT_MARGIN_RAD
    )


def test_the_recorded_bridge_epsilon_matches_the_bridge(convergence) -> None:
    from single_arm_bridge.action_validation import JOINT_LIMIT_EPSILON_RAD

    assert (
        convergence["bridge_joint_limit_epsilon_rad"]
        == JOINT_LIMIT_EPSILON_RAD
    )


@pytest.mark.parametrize(
    "overrides,message",
    (
        (
            {"never_command_exactly_on_a_limit": False},
            "never sit exactly on a joint limit",
        ),
        (
            {"joint_limit_margin_rad": 1.0e-9},
            "must exceed the wire quantisation",
        ),
    ),
)
def test_each_limit_margin_gate_refuses(document, overrides, message) -> None:
    with pytest.raises(BufferedTrajectoryContractError, match=message):
        validate_buffered_trajectory_contract(mutated(document, **overrides))


# ---------------------------------------------------------------------------
# 수렴 적용 후 다시 잰 파지 offset
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def offsets(document) -> dict:
    return document["tcp_contact_offsets"]


def test_the_converged_sweep_brackets_the_grasp(offsets) -> None:
    """놓친 회차와 잡은 회차가 모두 있어야 경계가 정해진다."""
    held = [e for e in offsets["converged_sweep"] if e["held"]]
    missed = [e for e in offsets["converged_sweep"] if not e["held"]]
    assert held and missed
    assert max(e["grasp_offset_m"] for e in held) < min(
        e["grasp_offset_m"] for e in missed
    )


def test_removing_sag_required_a_shallower_offset(offsets) -> None:
    """처짐을 흡수하던 값이었으므로 처짐을 줄이면 얕아져야 한다."""
    assert (
        offsets["pick_grasp_offset_m"]
        < offsets["superseded_pick_grasp_offset_m"]
    )
    assert offsets["measured_under_convergence"] is True


def test_the_deployed_offset_is_the_shallowest_that_held(offsets) -> None:
    held = [e for e in offsets["converged_sweep"] if e["held"]]
    assert offsets["pick_grasp_offset_m"] == min(
        e["grasp_offset_m"] for e in held
    )


@pytest.mark.parametrize(
    "mutate,message",
    (
        (
            lambda o: o.update(measured_under_convergence=False),
            "measured with the convergence layer active",
        ),
        (
            lambda o: o.update(pick_grasp_offset_m=0.020),
            "shallower offset, never a deeper",
        ),
        (
            lambda o: o.__setitem__(
                "converged_sweep",
                [e for e in o["converged_sweep"] if not e["held"]],
            ),
            "must bracket the grasp",
        ),
    ),
)
def test_each_converged_sweep_gate_refuses(document, mutate, message) -> None:
    changed = copy.deepcopy(document)
    mutate(changed["tcp_contact_offsets"])
    with pytest.raises(BufferedTrajectoryContractError, match=message):
        validate_buffered_trajectory_contract(changed)


def test_the_grasp_succeeded_outside_the_task_tolerance(convergence) -> None:
    """파지는 잔차 10.17 mm 에서 성립했다. 4 mm 는 필요조건이 아니었다.

    수렴의 값은 잔차를 0 으로 만드는 데 있지 않고 작게 만들고 측정 가능하게
    만드는 데 있다. 남은 잔차는 offset 이 흡수하며, 그것이 성립하려면 잔차가
    회차 간 재현되어야 한다 — 아직 측정되지 않았다.
    """
    assert convergence["c2_grasp_succeeded_at_residual_mm"] > (
        convergence["task_tolerance_m"] * 1000.0
    )
    assert convergence["task_tolerance_is_not_a_grasp_predictor"] is True
    assert convergence["task_tolerance_confirmed_on_hardware"] is False


def test_the_a45_retry_is_recorded(convergence) -> None:
    """A4.5 는 이 계층이 존재하는 이유다. 닫혔으면 그렇게 적혀 있어야 한다."""
    assert convergence["a45_retry_passed"] is True
    assert convergence["a45_retry_residual_gap_raw"] >= 14
    # 파지는 과제 허용치 밖에서 성립했다. 그 사실이 계약에 남아 있어야 한다.
    assert convergence["a45_retry_residual_mm"] > (
        convergence["task_tolerance_m"] * 1000.0
    )


def test_the_shadow_target_was_actually_consumed(document) -> None:
    """`ShadowObjectTarget` 은 "Never consume this as a motion goal" 로 시작한다.

    그 잠금을 넘어 실제 파지에 쓰인 것이 기록되어야 하고, 그래도 발행자는
    여전히 권한을 주장하지 않아야 한다.
    """
    shadow = document["top_shadow_grasp_candidate"]
    assert shadow["consumed_for_a_physical_grasp"] is True
    assert shadow["publisher_must_not_claim_authority"] is True
    assert shadow["motion_authorized"] is False
    assert shadow["operator_approves_each_descent"] is True
