#!/usr/bin/env python3
"""Reset the robot pose, TrainingObject, and Table to the Gate 2/3 baseline.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
open (Table/TrainingObject/PhysicsMaterial already created — run
isaac_create_gate2_training_scene.py + isaac_update_gate2_friction.py
first if they don't exist yet). Use it before each retry of
isaac_scripted_grasp_test.py (or manual testing), and after any Isaac Sim
crash/restart that reverted the live stage to its last-saved-to-disk
state — Stop alone does not undo prior Target Position / transform
writes, since those overwrite the authored default value rather than a
transient override, so Stop just resets physics to whatever that value
currently is, and an unsaved stage reverts to whatever was last written
with Ctrl+S. This script writes all three back explicitly:

- all six joints to Q_HOME
- TrainingObject's transform to the confirmed initial pose
- Table's transform to the confirmed size/position (the table was
  originally created at a placeholder size by
  isaac_create_gate2_training_scene.py, then resized by hand in the GUI —
  that final size only lives here and in config/manual_grasp_poses.json,
  not in the creation script)

Both transforms clear whatever xform ops are currently authored and
re-author a clean translate -> rotateXYZ -> scale chain.

Values mirror config/manual_grasp_poses.json. After running this, save
the stage (Ctrl+S) so the fix actually survives the next Isaac Sim
restart.
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

TABLE_PATH = "/so101_new_calib/TrainingScene/Table"
TABLE_TRANSLATE_M = (0.11715, -0.00018, -0.01859)
TABLE_ROTATE_XYZ_DEG = (0.0, 0.0, 0.0)
TABLE_SCALE_M = (0.62974, 0.67526, 0.02)


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


def _reset_transform(stage, path, translate_m, rotate_xyz_deg, scale_m) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"{path} not found")

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translate_m))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz_deg))
    xformable.AddScaleOp().Set(Gf.Vec3f(*scale_m))


def main() -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    _reset_joints(stage)
    _reset_transform(stage, TABLE_PATH, TABLE_TRANSLATE_M, TABLE_ROTATE_XYZ_DEG, TABLE_SCALE_M)
    _reset_transform(stage, OBJECT_PATH, OBJECT_TRANSLATE_M, OBJECT_ROTATE_XYZ_DEG, OBJECT_SCALE_M)

    print(
        "GATE2_SCENE_RESET "
        f"joints={Q_HOME} "
        f"table_translate={TABLE_TRANSLATE_M} table_scale={TABLE_SCALE_M} "
        f"object_translate={OBJECT_TRANSLATE_M} object_rotate_xyz_deg={OBJECT_ROTATE_XYZ_DEG}"
    )


if __name__ == "__main__":
    main()
