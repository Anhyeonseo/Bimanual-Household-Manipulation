#!/usr/bin/env python3
"""Reset both the robot pose and TrainingObject to the Gate 2/3 baseline.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
open. Use it before each retry of isaac_scripted_grasp_test.py (or manual
testing) — Stop alone does not undo prior Target Position / transform
writes, since those overwrite the authored default value rather than a
transient override, so Stop just resets physics to whatever that value
currently is. This script writes both back explicitly:

- all six joints to Q_HOME
- TrainingObject's transform to the confirmed initial pose (clears
  whatever xform ops are currently authored — translate/scale from
  isaac_create_gate2_training_scene.py, plus any rotate added later by
  hand — and re-authors a clean translate -> rotateXYZ -> scale chain)

Values mirror config/manual_grasp_poses.json.
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
Q_HOME = [0, 0, 0, 0, 0, -10]
JOINT_PRIM_PATHS = {name: f"/so101_new_calib/Physics/{name}" for name in JOINT_ORDER}

OBJECT_PATH = "/so101_new_calib/TrainingScene/TrainingObject"
OBJECT_TRANSLATE_M = (0.19523, 0.02106, 0.00391)
OBJECT_ROTATE_XYZ_DEG = (0.0, -0.0, 72.414)
OBJECT_SCALE_M = (0.08, 0.025, 0.025)


def _reset_joints(stage) -> None:
    from pxr import UsdPhysics

    for name, value in zip(JOINT_ORDER, Q_HOME):
        path = JOINT_PRIM_PATHS[name]
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"joint prim not found: {path}")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            raise RuntimeError(f"no angular DriveAPI on {path}")
        drive.GetTargetPositionAttr().Set(float(value))


def _reset_object(stage) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(OBJECT_PATH)
    if not prim.IsValid():
        raise RuntimeError(f"{OBJECT_PATH} not found")

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*OBJECT_TRANSLATE_M))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(*OBJECT_ROTATE_XYZ_DEG))
    xformable.AddScaleOp().Set(Gf.Vec3f(*OBJECT_SCALE_M))


def main() -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    _reset_joints(stage)
    _reset_object(stage)

    print(
        "GATE2_SCENE_RESET "
        f"joints={Q_HOME} "
        f"object_translate={OBJECT_TRANSLATE_M} "
        f"object_rotate_xyz_deg={OBJECT_ROTATE_XYZ_DEG}"
    )


if __name__ == "__main__":
    main()
