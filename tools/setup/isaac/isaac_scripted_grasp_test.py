#!/usr/bin/env python3
"""Reproduce the confirmed Gate 2 manual grasp sequence without GUI clicking.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
open, TrainingObject at the same fixed position used to confirm Gate 2
(10/10 successful lifts), and Play already running. It drives the six
joints through Q_HOME -> Q_PRE_GRASP -> Q_GRASP -> Q_CLOSE -> Q_LIFT with
linear interpolation over time instead of instantaneous jumps, matching
how the manual GUI test moved the sliders gradually.

Waypoints mirror config/manual_grasp_poses.json (duplicated here rather
than read from disk, because Script Editor execution does not reliably
expose __file__ / the repo path — see the isaac_preview_*.py scripts for
the same convention).

Do not run this a second time before the first run prints
GATE3_SCRIPTED_GRASP_COMPLETE — a second concurrent run adds a second
per-frame subscription driving the same joints and the two will fight.
"""

from __future__ import annotations


JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# deg, matches config/manual_grasp_poses.json (gate2_confirmed, 10/10)
WAYPOINTS = [
    ("Q_HOME", [0, 0, 0, 0, 0, -10]),
    ("Q_PRE_GRASP", [-1.7, 52, -111.99, -0.5, 40, 20.4]),
    ("Q_GRASP", [-1.7, 77, -13.1, -0.5, 72.79, 20.4]),
    ("Q_CLOSE", [-1.7, 77, -13.1, -0.5, 72.79, 8.0]),
    ("Q_LIFT", [-1.7, 60, -13.1, -0.5, 72.79, 8.0]),
]

# seconds to spend moving from WAYPOINTS[i] to WAYPOINTS[i + 1]
SEGMENT_DURATIONS_S = [2.0, 2.0, 1.0, 1.5]
HOLD_AFTER_LIFT_S = 1.0

JOINT_PRIM_PATHS = {name: f"/so101_new_calib/Physics/{name}" for name in JOINT_ORDER}


def _lerp(start_values, end_values, ratio):
    return [
        start + (end - start) * ratio
        for start, end in zip(start_values, end_values)
    ]


def main() -> None:
    import omni.kit.app
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    drives = {}
    for name, path in JOINT_PRIM_PATHS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"joint prim not found: {path}")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            raise RuntimeError(f"no angular DriveAPI on {path}")
        drives[name] = drive

    segment_starts = [0.0]
    for duration in SEGMENT_DURATIONS_S:
        segment_starts.append(segment_starts[-1] + duration)
    total_duration = segment_starts[-1] + HOLD_AFTER_LIFT_S

    state = {"elapsed": 0.0, "subscription": None, "done": False}

    def apply_pose(values):
        for name, value in zip(JOINT_ORDER, values):
            drives[name].GetTargetPositionAttr().Set(float(value))

    def on_update(event):
        if state["done"]:
            return
        dt = event.payload.get("dt", 0.0) if event.payload else 0.0
        state["elapsed"] += dt
        elapsed = state["elapsed"]

        if elapsed >= total_duration:
            apply_pose(WAYPOINTS[-1][1])
            state["done"] = True
            state["subscription"] = None
            print(
                "GATE3_SCRIPTED_GRASP_COMPLETE "
                f"final_pose={WAYPOINTS[-1][0]} elapsed_s={elapsed:.2f}"
            )
            return

        segment_index = len(SEGMENT_DURATIONS_S) - 1
        for i in range(len(SEGMENT_DURATIONS_S)):
            if elapsed < segment_starts[i + 1]:
                segment_index = i
                break

        seg_start = segment_starts[segment_index]
        seg_duration = SEGMENT_DURATIONS_S[segment_index]
        ratio = 0.0 if seg_duration <= 0 else (elapsed - seg_start) / seg_duration
        ratio = max(0.0, min(1.0, ratio))

        start_values = WAYPOINTS[segment_index][1]
        end_values = WAYPOINTS[segment_index + 1][1]
        apply_pose(_lerp(start_values, end_values, ratio))

    apply_pose(WAYPOINTS[0][1])
    stream = omni.kit.app.get_app().get_update_event_stream()
    state["subscription"] = stream.create_subscription_to_pop(on_update)
    print(f"GATE3_SCRIPTED_GRASP_STARTED total_duration_s={total_duration:.2f}")


if __name__ == "__main__":
    main()
