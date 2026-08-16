from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools/run_left_right_pen_transfer_once.py").read_text(
    encoding="utf-8"
)


def test_transfer_is_freshly_replanned_between_arms_without_retry() -> None:
    assert "RUN_LEFT_RIGHT_PEN_TRANSFER_ONCE" in SOURCE
    assert "RUN_RIGHT_PLACE_HEIGHT_CHECK_ONCE" in SOURCE
    assert '"--interarm-place"' in SOURCE
    assert 'require_plan_side(left_plan, "left")' in SOURCE
    assert 'require_plan_side(right_plan, "right")' in SOURCE
    assert SOURCE.index('"left_execute"') < SOURCE.index('"right_plan"')
    assert "time.sleep(args.perception_settle_s)" in SOURCE
    assert '"automatic_retry_count": 0' in SOURCE
    assert "stage failed without retry" in SOURCE


def test_transfer_pins_plans_urdf_and_right_lateral_schema() -> None:
    assert "EXPECTED_DUAL_URDF_SHA256" in SOURCE
    assert 'environment["SO101_DUAL_URDF_PATH"]' in SOURCE
    assert 'document.get("schema_version") != 12' in SOURCE
    assert '"--plan-sha256"' in SOURCE
    assert '"--right-place-validation"' in SOURCE
    assert "LEFT_RIGHT_PEN_TRANSFER_ONCE_PASS" in SOURCE
