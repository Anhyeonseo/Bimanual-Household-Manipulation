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


def remote_script(
    repository: str,
    session: str,
    category: str,
    split: str,
    frames_per_episode: int = 1,
) -> str:
    prefix = CATEGORIES[category]
    repository_q = shlex.quote(repository)
    session_q = shlex.quote(session)
    category_q = shlex.quote(category)
    prefix_q = shlex.quote(prefix)
    split_q = shlex.quote(split)
    if frames_per_episode <= 0:
        raise ValueError("frames_per_episode must be positive")
    return f"""set -e
repository={repository_q}
session={session_q}
category={category_q}
prefix={prefix_q}
split={split_q}
frames_per_episode={frames_per_episode}
session_root="$repository/datasets/towel_yolo_source/$session"
output_dir="$session_root/$category"
manifest="$session_root/capture_manifest.jsonl"
session_metadata="$session_root/session.json"

mkdir -p "$output_dir"
cd "$repository"
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=30
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
set -u

python3 - "$session_metadata" "$session" "$split" "$frames_per_episode" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
frames_per_episode = int(sys.argv[4])
document = {{
    "schema_version": 1,
    "record_kind": "towel_capture_session",
    "session_id": sys.argv[2],
    "split": sys.argv[3],
    "capture_protocol": (
        "independent_physical_reposition_per_frame_v1"
        if frames_per_episode == 1
        else "independent_physical_reposition_episode_burst_v1"
    ),
    "image_width_px": 1280,
    "image_height_px": 960,
}}
if frames_per_episode != 1:
    document["frames_per_episode"] = frames_per_episode
if path.exists():
    existing = json.loads(path.read_text(encoding="utf-8"))
    if existing != document:
        raise SystemExit(f"session metadata mismatch: {{path}}")
else:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\\n",
        encoding="utf-8",
    )
    temporary.replace(path)
PY

printf 'Remote output: %s\\n' "$output_dir"
if [ "$split" = train ]; then
    printf 'Enter=capture, q=quit. Reposition the towel before every episode.\\n'
else
    printf 'Type moved=capture, q=quit. A physical reposition is required for every held-out episode.\\n'
fi

while true; do
    printf '[%s:%s] capture/q > ' "$split" "$category"
    IFS= read -r answer || break
    if [ "$answer" = q ] || [ "$answer" = Q ]; then
        break
    fi
    if [ "$split" != train ] && [ "$answer" != moved ]; then
        printf 'Held-out capture rejected: type exactly moved after repositioning.\\n'
        continue
    fi

    index=1
    while true; do
        if [ "$frames_per_episode" -eq 1 ]; then
            filename=$(printf '%s_%04d.jpg' "$prefix" "$index")
        else
            filename=$(printf '%s_%04d_01.jpg' "$prefix" "$index")
        fi
        [ ! -e "$output_dir/$filename" ] && break
        index=$((index + 1))
    done

    frame_index=1
    while [ "$frame_index" -le "$frames_per_episode" ]; do
        if [ "$frames_per_episode" -eq 1 ]; then
            filename=$(printf '%s_%04d.jpg' "$prefix" "$index")
        else
            filename=$(printf '%s_%04d_%02d.jpg' "$prefix" "$index" "$frame_index")
        fi
        output="$output_dir/$filename"
        python3 tools/setup/camera_calibration/capture_top_frame.py \
            --output "$output" \
            --timeout 30 \
            --settle-frames 5

        python3 - "$output" "$manifest" "$session" "$split" "$category" "$prefix" "$index" "$frame_index" "$frames_per_episode" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

import cv2

image_path = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
session, split, category, prefix = sys.argv[3:7]
index = int(sys.argv[7])
frame_index = int(sys.argv[8])
frames_per_episode = int(sys.argv[9])
image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
if image is None or image.shape[:2] != (960, 1280):
    raise SystemExit(f"captured image is not a decodable 1280x960 RGB frame: {{image_path}}")
episode_id = f"{{session}}-{{prefix}}-{{index:04d}}"
capture_id = (
    episode_id
    if frames_per_episode == 1
    else f"{{episode_id}}-frame-{{frame_index:02d}}"
)
record = {{
    "schema_version": 1,
    "record_kind": "towel_capture_episode",
    "session_id": session,
    "capture_id": capture_id,
    "episode_id": episode_id,
    "frame_index": frame_index,
    "frames_per_episode": frames_per_episode,
    "split": split,
    "category": category,
    "image_path": f"{{category}}/{{image_path.name}}",
    "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
    "image_width_px": 1280,
    "image_height_px": 960,
    "physical_reposition_confirmed": split != "train",
    "captured_at_utc": datetime.now(timezone.utc).isoformat(),
}}
if manifest_path.exists():
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        if line and json.loads(line).get("capture_id") == capture_id:
            raise SystemExit(f"duplicate capture_id in manifest: {{capture_id}}")
with manifest_path.open("a", encoding="utf-8") as stream:
    stream.write(json.dumps(record, sort_keys=True) + "\\n")
print(f"TOWEL_CAPTURE_EPISODE_RECORDED id={{capture_id}} split={{split}}")
PY
        frame_index=$((frame_index + 1))
    done
done
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--remote-repository", default=DEFAULT_REMOTE_REPOSITORY)
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument(
        "--split",
        choices=("train", "validation", "test"),
        default="train",
        help=(
            "dataset partition; validation/test require an explicit 'moved' "
            "confirmation before each independently repositioned episode"
        ),
    )
    parser.add_argument("--category", choices=tuple(CATEGORIES), default="01_flat")
    parser.add_argument(
        "--frames-per-episode",
        type=int,
        default=1,
        help="capture a settled burst after one physical reposition confirmation",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = safe_component(args.session, "session")
    category = safe_component(args.category, "category")
    if args.split != "train" and session == DEFAULT_SESSION:
        raise ValueError(
            "validation/test captures require a new --session; the legacy "
            "training session cannot be reused"
        )
    if not 1 <= args.frames_per_episode <= 10:
        raise ValueError("frames_per_episode must be within 1..10")
    script = remote_script(
        args.remote_repository,
        session,
        category,
        args.split,
        args.frames_per_episode,
    )
    command = "bash -lc " + shlex.quote(script)
    os.execvp("ssh", ("ssh", "-t", args.host, command))


if __name__ == "__main__":
    main()
