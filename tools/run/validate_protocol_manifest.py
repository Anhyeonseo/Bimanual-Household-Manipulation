#!/usr/bin/env python3
"""Validate the machine-readable Pi–STM32 protocol message manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


MANDATORY_MESSAGES = {
    "HELLO_REQUEST",
    "HELLO_RESPONSE",
    "HEARTBEAT",
    "ARM_REQUEST",
    "ARM_RESPONSE",
    "ENABLE",
    "HOLD",
    "SAFE_STOP",
    "DISABLE",
    "CLEAR_FAULT",
    "SETPOINT_BATCH",
    "SETPOINT_STATUS",
    "GET_STATE",
    "STATE_FEEDBACK",
    "FAULT_REPORT",
    "DIAGNOSTICS",
}
VALID_DIRECTIONS = {"HOST_TO_MCU", "MCU_TO_HOST"}
VALID_CATEGORIES = {"session", "state_control", "motion", "feedback"}


def validate_manifest(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("protocol_version") != 1:
        errors.append("protocol_version must be 1")

    ranges = data.get("ranges")
    if not isinstance(ranges, dict):
        return errors + ["ranges object is required"]

    parsed_ranges: dict[str, tuple[int, int]] = {}
    for name, limits in ranges.items():
        if not isinstance(limits, list) or len(limits) != 2 or not all(isinstance(value, int) for value in limits):
            errors.append(f"ranges.{name} must be [integer, integer]")
            continue
        start, end = limits
        if not 0 <= start <= end <= 255:
            errors.append(f"ranges.{name} must stay inside uint8 and start <= end")
        parsed_ranges[name] = (start, end)

    messages = data.get("messages")
    if not isinstance(messages, list):
        return errors + ["messages list is required"]

    seen_ids: set[int] = set()
    seen_names: set[str] = set()
    for index, message in enumerate(messages):
        path = f"messages[{index}]"
        if not isinstance(message, dict):
            errors.append(f"{path} must be an object")
            continue
        message_id = message.get("id")
        name = message.get("name")
        category = message.get("category")
        direction = message.get("direction")

        if not isinstance(message_id, int) or isinstance(message_id, bool) or not 1 <= message_id <= 255:
            errors.append(f"{path}.id must be an integer in 1..255")
        elif message_id in seen_ids:
            errors.append(f"{path}.id duplicates {message_id}")
        else:
            seen_ids.add(message_id)

        if not isinstance(name, str) or not name or name != name.upper():
            errors.append(f"{path}.name must be a non-empty uppercase string")
        elif name in seen_names:
            errors.append(f"{path}.name duplicates {name}")
        else:
            seen_names.add(name)

        if direction not in VALID_DIRECTIONS:
            errors.append(f"{path}.direction is invalid")
        if category not in VALID_CATEGORIES:
            errors.append(f"{path}.category is invalid")
        if not isinstance(message.get("ack_required"), bool):
            errors.append(f"{path}.ack_required must be boolean")

        category_range = parsed_ranges.get(category)
        if isinstance(message_id, int) and category_range is not None:
            start, end = category_range
            if not start <= message_id <= end:
                errors.append(f"{path}.id {message_id} is outside category range {start}..{end}")

        reserved_range = parsed_ranges.get("reserved")
        if isinstance(message_id, int) and reserved_range is not None:
            if reserved_range[0] <= message_id <= reserved_range[1]:
                errors.append(f"{path}.id uses reserved range")

    missing = sorted(MANDATORY_MESSAGES - seen_names)
    if missing:
        errors.append(f"mandatory messages are missing: {missing}")
    if "ESTOP" in seen_names:
        errors.append("ESTOP serial message is forbidden; use SAFE_STOP and physical E-stop reporting")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=Path("protocol/message_ids.json"))
    args = parser.parse_args()
    try:
        data = json.loads(args.path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[ERROR] Could not read {args.path}: {exc}")
        return 2
    errors = validate_manifest(data)
    if errors:
        print(f"[FAIL] Protocol manifest has {len(errors)} error(s):")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(f"[PASS] Protocol manifest defines {len(data['messages'])} unique messages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
