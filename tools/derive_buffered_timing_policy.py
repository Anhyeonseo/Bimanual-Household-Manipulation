#!/usr/bin/env python3
"""Derive a reviewed buffered timing policy from offline capture files only."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from single_arm_bridge.buffered_timing import (
    analyze_buffered_timing_capture,
    derive_buffered_timing_policy,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline-only reviewed buffered timing policy derivation."
    )
    parser.add_argument("output", type=Path)
    parser.add_argument("--capture", type=Path, action="append", required=True)
    parser.add_argument("--analysis", type=Path, action="append", required=True)
    parser.add_argument("--rejected-first-lead-ms", type=int, required=True)
    parser.add_argument("--rejected-status-code", type=int, required=True)
    parser.add_argument("--rejected-detail", type=int, required=True)
    arguments = parser.parse_args()

    if len(arguments.capture) != len(arguments.analysis):
        raise SystemExit("capture and analysis counts must match")

    captures: list[dict] = []
    evidence: list[dict[str, str]] = []
    for capture_path, analysis_path in zip(
        arguments.capture,
        arguments.analysis,
        strict=True,
    ):
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        checked_analysis = analyze_buffered_timing_capture(capture)
        stored_analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        if stored_analysis != checked_analysis:
            raise SystemExit(f"analysis does not match capture: {analysis_path}")
        captures.append(capture)
        evidence.append({
            "capture_path": str(capture_path),
            "capture_sha256": sha256(capture_path),
            "analysis_path": str(analysis_path),
            "analysis_sha256": sha256(analysis_path),
        })

    policy = derive_buffered_timing_policy(
        captures,
        rejected_first_lead_ms=arguments.rejected_first_lead_ms,
        rejected_status_code=arguments.rejected_status_code,
        rejected_detail=arguments.rejected_detail,
    )
    policy["evidence"] = evidence
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(policy, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    values = policy["deployment_values"]
    print(f"BUFFERED_TIMING_POLICY={arguments.output}")
    print(f"MINIMUM_LEAD_MS={values['minimum_lead_ms']}")
    print(f"MAXIMUM_LEAD_MS={values['maximum_lead_ms']}")
    print(f"STARTUP_PRIME_DEPTH={values['startup_prime_depth_samples']}")
    print(f"LOW_WATERMARK={values['low_watermark_samples']}")
    print(f"REFILL_TARGET={values['refill_target_samples']}")
    print("OPERATIONAL_VALUES_AUTHORIZED=1")
    print("MOTION_AUTHORIZED=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
