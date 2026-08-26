#!/usr/bin/env python3
"""Interactively capture auto-numbered Top-camera images on the camera Pi."""

from __future__ import annotations

import argparse
import os
import re
import shlex


DEFAULT_HOST = "pi@192.168.35.237"
DEFAULT_REMOTE_REPOSITORY = "/home/pi/SO101-Bimanual-Manipulation"
DEFAULT_SESSION = "20260826_top_01"

CATEGORIES = {
    "00_empty_table": "empty",
    "01_flat": "flat",
    "02_light_wrinkle": "light_wrinkle",
    "03_heavy_wrinkle": "heavy_wrinkle",
    "04_curled_or_overlapped": "curled_or_overlapped",
    "05_first_fold": "first_fold",
    "06_second_fold": "second_fold",
    "07_robot_occluded": "robot_occluded",
}

SAFE_COMPONENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def safe_component(value: str, label: str) -> str:
    if not SAFE_COMPONENT.fullmatch(value):
        raise ValueError(f"{label} must contain only letters, digits, '.', '_' or '-'")
    return value


def remote_script(repository: str, session: str, category: str) -> str:
    prefix = CATEGORIES[category]
    repository_q = shlex.quote(repository)
    session_q = shlex.quote(session)
    category_q = shlex.quote(category)
    prefix_q = shlex.quote(prefix)
    return f"""set -e
repository={repository_q}
session={session_q}
category={category_q}
prefix={prefix_q}
output_dir="$repository/datasets/towel_yolo_source/$session/$category"

mkdir -p "$output_dir"
cd "$repository"
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=30
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
set -u

printf 'Remote output: %s\\n' "$output_dir"
printf 'Enter=capture, q=quit. Reposition the towel before every capture.\\n'

while true; do
    printf '[%s] Enter/q > ' "$category"
    IFS= read -r answer || break
    if [ "$answer" = q ] || [ "$answer" = Q ]; then
        break
    fi

    index=1
    while true; do
        filename=$(printf '%s_%04d.jpg' "$prefix" "$index")
        output="$output_dir/$filename"
        [ ! -e "$output" ] && break
        index=$((index + 1))
    done

    python3 tools/setup/camera_calibration/capture_top_frame.py \
        --output "$output" \
        --timeout 30 \
        --settle-frames 5
done
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--remote-repository", default=DEFAULT_REMOTE_REPOSITORY)
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument("--category", choices=tuple(CATEGORIES), default="01_flat")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = safe_component(args.session, "session")
    category = safe_component(args.category, "category")
    script = remote_script(args.remote_repository, session, category)
    command = "bash -lc " + shlex.quote(script)
    os.execvp("ssh", ("ssh", "-t", args.host, command))


if __name__ == "__main__":
    main()
