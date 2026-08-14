from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAPTURE = load(
    "capture_j1w_branch_reference_read_only",
    ROOT / "tools/capture_j1w_branch_reference_read_only.py",
)
VALIDATE = load(
    "validate_protocol_v2_unwrap_shadow_no_output",
    ROOT / "tools/validate_protocol_v2_unwrap_shadow_no_output.py",
)
OBSERVE = load(
    "observe_j1w_unwrapped_wrap_torque_off",
    ROOT / "tools/observe_j1w_unwrapped_wrap_torque_off.py",
)
CAPTURE_SOURCE = (
    ROOT / "tools/capture_j1w_branch_reference_read_only.py"
).read_text()
VALIDATE_SOURCE = (
    ROOT / "tools/validate_protocol_v2_unwrap_shadow_no_output.py"
).read_text()
OBSERVE_SOURCE = (
    ROOT / "tools/observe_j1w_unwrapped_wrap_torque_off.py"
).read_text()


def test_reference_capture_requires_stationary_non_ambiguous_shoulders() -> None:
    sample = (2048,) * 12
    assert CAPTURE.validate_stable_reference([sample] * 10) == sample
    moved = list(sample)
    moved[4] += 5
    with pytest.raises(ValueError, match="moved"):
        CAPTURE.validate_stable_reference([sample, tuple(moved)])
    ambiguous = list(sample)
    ambiguous[1] = 24
    with pytest.raises(ValueError, match="shoulder"):
        CAPTURE.validate_stable_reference([tuple(ambiguous)] * 10)


def test_validator_recomputes_unwrapped_anchor_independently() -> None:
    raw = (24,) + (2048,) * 11
    references = (4120,) + (2048,) * 11
    unwrapped = VALIDATE.expected_unwrapped_snapshot(raw, references, 64)
    assert unwrapped[0] == 4120
    joints = tuple(
        {
            "zero_raw": 2048,
            "positive_raw_direction": 1,
        }
        for _ in range(12)
    )
    anchor = VALIDATE.expected_anchor(unwrapped, joints)
    assert anchor[0] > 3_000_000
    assert anchor[1:] == (0,) * 11


def test_reference_artifact_is_sha_bound(tmp_path: Path) -> None:
    path = tmp_path / "reference.json"
    path.write_text(
        json.dumps(
            {
                "status": "J1W_BRANCH_REFERENCE_CAPTURE_PASS",
                "motion_authorized": False,
                "reference_unwrapped_raw": [2048] * 12,
            }
        ),
        encoding="utf-8",
    )
    digest = VALIDATE.file_sha256(path)
    assert VALIDATE.load_reference(path, digest) == (2048,) * 12
    with pytest.raises(RuntimeError, match="SHA mismatch"):
        VALIDATE.load_reference(path, "0" * 64)


def test_tools_expose_no_motion_or_torque_api() -> None:
    assert '"motion_authorized": False' in CAPTURE_SOURCE
    assert '"motion_authorized": False' in VALIDATE_SOURCE
    assert '"motion_authorized": False' in OBSERVE_SOURCE
    assert '"executor_goal_output_connected": False' in VALIDATE_SOURCE
    assert '"executor_goal_output_connected": False' in OBSERVE_SOURCE
    assert "continuous_update_samples = 3" in VALIDATE_SOURCE
    assert "transport.prepare_shadow()" in OBSERVE_SOURCE
    for source in (CAPTURE_SOURCE, VALIDATE_SOURCE, OBSERVE_SOURCE):
        for forbidden in (
            ".arm_and_enable(",
            ".enable(",
            ".hold(",
            ".safe_stop(",
            ".clear_fault(",
            "send_goal_async",
        ):
            assert forbidden not in source
