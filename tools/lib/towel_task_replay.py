"""Offline replay of towel observations through the bounded task state machine."""

from __future__ import annotations

from typing import Any, Iterable, Mapping

from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    TaskPhase,
    TowelObservation,
    TowelTaskContractError,
    TowelTaskStateMachine,
    estimate_towel_state,
    validate_towel_contract,
)


def replay_towel_task(
    contract: Mapping[str, Any],
    observation_documents: Iterable[Mapping[str, Any]],
) -> dict[str, Any]:
    validate_towel_contract(contract)
    limits = PerceptionLimits.from_contract(contract)
    machine = TowelTaskStateMachine.from_contract(contract)
    steps = []
    calibration_sha256: str | None = None
    seen_ids: set[str] = set()
    for index, source in enumerate(observation_documents, start=1):
        flags = {}
        for flag_name in ("fault", "workspace_exit"):
            value = source.get(flag_name, False)
            if not isinstance(value, bool):
                raise TowelTaskContractError(
                    f"replay {flag_name} must be boolean"
                )
            flags[flag_name] = value
        observation = TowelObservation.from_dict(source)
        if observation.observation_id in seen_ids:
            raise TowelTaskContractError(
                f"duplicate replay observation_id: {observation.observation_id}"
            )
        seen_ids.add(observation.observation_id)
        if calibration_sha256 is None:
            calibration_sha256 = observation.calibration_sha256
        elif observation.calibration_sha256 != calibration_sha256:
            raise TowelTaskContractError(
                "replay sequence mixes calibration identities"
            )
        estimate = estimate_towel_state(observation, limits)
        decision = machine.decide(
            estimate,
            fault=flags["fault"],
            workspace_exit=flags["workspace_exit"],
        )
        steps.append({
            "index": index,
            "observation_id": observation.observation_id,
            "estimated_state": estimate.state.value,
            "state_reason": estimate.reason,
            "phase": decision.phase.value,
            "primitive": decision.primitive,
            "terminal": decision.terminal,
            "decision_reason": decision.reason,
        })
        if decision.terminal:
            break
    if machine.phase not in (TaskPhase.COMPLETE, TaskPhase.FAILED):
        machine.phase = TaskPhase.FAILED
        steps.append({
            "index": len(steps) + 1,
            "observation_id": None,
            "estimated_state": None,
            "state_reason": "observation sequence exhausted",
            "phase": TaskPhase.FAILED.value,
            "primitive": None,
            "terminal": True,
            "decision_reason": "observation sequence exhausted before terminal state",
        })
    return {
        "schema_version": 1,
        "record_kind": "towel_task_offline_replay",
        "status": (
            "TOWEL_TASK_REPLAY_COMPLETE"
            if machine.phase == TaskPhase.COMPLETE
            else "TOWEL_TASK_REPLAY_FAILED"
        ),
        "motion_authorized": False,
        "motion_commands": 0,
        "execution_api_used": False,
        "calibration_sha256": calibration_sha256,
        "terminal_phase": machine.phase.value,
        "decision_count": machine.decision_count,
        "recovery_attempts": dict(sorted(machine.ledger.attempts.items())),
        "steps": steps,
    }
