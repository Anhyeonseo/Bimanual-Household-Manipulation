#!/usr/bin/env python3
"""Validate phase-based camera and inference budgets for Raspberry Pi."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


CAMERAS = ("top", "wrist_a", "wrist_b")
REQUIRED_PHASES = {
    "STANDBY",
    "SEARCH",
    "APPROACH_RIGHT",
    "VISUAL_ALIGN_RIGHT",
    "TRANSFER_RIGHT",
    "VERIFY_RIGHT",
    "DUAL_PRIVATE",
    "RUNTIME_BASELINE",
    "POLICY_ASSIST",
}


def _nonnegative_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value >= 0


def validate_schedule(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    capture = data.get("capture", {})
    capture_fps = capture.get("fps")
    if not _nonnegative_number(capture_fps) or capture_fps <= 0:
        errors.append("capture.fps must be positive")
        capture_fps = 0
    if capture.get("queue_depth") != 1:
        errors.append("capture.queue_depth must be exactly 1")

    runtime = data.get("vision_runtime", {})
    max_total = runtime.get("max_total_inference_hz")
    if not _nonnegative_number(max_total) or max_total <= 0:
        errors.append("vision_runtime.max_total_inference_hz must be positive")
        max_total = 0
    if runtime.get("max_concurrent_inference") != 1:
        errors.append("vision_runtime.max_concurrent_inference must be exactly 1")
    if runtime.get("intra_op_threads") not in (1, 2):
        errors.append("vision_runtime.intra_op_threads must be 1 or 2")
    if runtime.get("inter_op_threads") != 1:
        errors.append("vision_runtime.inter_op_threads must be 1")

    policy = data.get("policy_runtime", {})
    if policy.get("raw_image_input") is not False:
        errors.append("policy_runtime.raw_image_input must be false")
    if policy.get("intra_op_threads") != 1:
        errors.append("policy_runtime.intra_op_threads must be 1")
    policy_default = policy.get("default_rate_hz")
    if not _nonnegative_number(policy_default) or policy_default <= 0:
        errors.append("policy_runtime.default_rate_hz must be positive")
        policy_default = 0

    phases = data.get("phases")
    if not isinstance(phases, dict):
        return errors + ["phases object is required"]
    missing = sorted(REQUIRED_PHASES - set(phases))
    if missing:
        errors.append(f"required phases are missing: {missing}")

    for phase_name, phase in phases.items():
        path = f"phases.{phase_name}"
        if not isinstance(phase, dict):
            errors.append(f"{path} must be an object")
            continue
        total_inference = 0.0
        for camera in CAMERAS:
            camera_budget = phase.get(camera)
            if not isinstance(camera_budget, dict):
                errors.append(f"{path}.{camera} must be an object")
                continue
            decode_hz = camera_budget.get("decode_hz")
            inference_hz = camera_budget.get("inference_hz")
            if not _nonnegative_number(decode_hz):
                errors.append(f"{path}.{camera}.decode_hz must be nonnegative")
                continue
            if not _nonnegative_number(inference_hz):
                errors.append(f"{path}.{camera}.inference_hz must be nonnegative")
                continue
            if decode_hz > capture_fps:
                errors.append(f"{path}.{camera}.decode_hz exceeds capture.fps")
            if inference_hz > decode_hz:
                errors.append(f"{path}.{camera}.inference_hz exceeds decode_hz")
            total_inference += inference_hz

        if total_inference > max_total:
            errors.append(f"{path}: total inference {total_inference:g}Hz exceeds {max_total:g}Hz")
        policy_hz = phase.get("policy_hz")
        if not _nonnegative_number(policy_hz):
            errors.append(f"{path}.policy_hz must be nonnegative")
        elif policy_hz > policy_default:
            errors.append(f"{path}.policy_hz exceeds policy default rate")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=Path("config/camera_schedule.json"))
    args = parser.parse_args()
    try:
        data = json.loads(args.path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[ERROR] Could not read {args.path}: {exc}")
        return 2
    errors = validate_schedule(data)
    if errors:
        print(f"[FAIL] Camera schedule has {len(errors)} error(s):")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(f"[PASS] Camera schedule validates {len(data['phases'])} phases.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
