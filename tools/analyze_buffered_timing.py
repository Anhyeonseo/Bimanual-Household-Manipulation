#!/usr/bin/env python3
"""Analyze an offline buffered timing capture; never opens serial or ROS."""

import argparse
import json
from pathlib import Path

from single_arm_bridge.buffered_timing import analyze_buffered_timing_capture


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    analysis = analyze_buffered_timing_capture(
        json.loads(arguments.capture.read_text(encoding="utf-8"))
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(analysis, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"BUFFERED_TIMING_ANALYSIS={arguments.output}")
    print(f"STATUS={analysis['status']}")
    print(f"OPERATIONAL_VALUES_AUTHORIZED={int(analysis['operational_values_authorized'])}")
    print("MOTION_AUTHORIZED=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
