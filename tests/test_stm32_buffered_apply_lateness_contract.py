from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")
EXECUTOR = (
    ROOT / "firmware/stm32_actuator/src/buffered_executor.c"
).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_g474_separates_validation_and_execution_lateness_policy() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert (
        "HOST_BUFFERED_VALIDATION_MAXIMUM_APPLY_LATENESS_MS "
        "UINT32_C(0)"
    ) in CONFIG
    assert (
        "HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS "
        "UINT32_C(5)"
    ) in CONFIG

    validation = function_body(
        BINARY,
        "static uint8_t Host_InitBufferedValidationRoute(void)",
    )
    execution = function_body(
        BINARY,
        "static uint8_t Host_InitBufferedExecutionRoute(void)",
    )
    assert "HOST_BUFFERED_VALIDATION_MAXIMUM_APPLY_LATENESS_MS" in validation
    assert "HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS" in execution


def test_executor_applies_only_bounded_late_sample_at_scheduled_anchor() -> None:
    step = function_body(
        EXECUTOR,
        "actuator_buffered_result_t actuator_buffered_executor_step(",
    )
    assert "apply_lateness = current_tick - next.apply_tick" in step
    assert "apply_lateness > executor->maximum_apply_lateness_ticks" in step
    assert "ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK" in step
    assert "next.apply_tick" in step
    assert "last_applied_tick = current_tick" in step
    assert "maximum_apply_lateness_ticks" in step


def test_success_terminal_detail_reports_maximum_apply_lateness() -> None:
    service = function_body(
        BINARY,
        "static void Host_ServiceBufferedExecution(void)",
    )
    assert "diagnostics->maximum_apply_lateness_ticks" in service
    assert "Host_FinalizeBufferedExecution(" in service
    assert "UINT8_MAX" in service
    assert "diagnostics->reason" in service
