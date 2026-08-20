#!/usr/bin/env python3
"""Raise Gate 2 grip friction and extend it to the gripper fingers.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
already open, after isaac_create_gate2_training_scene.py has already run
once (TrainingScene/PhysicsMaterial must already exist).

It updates the existing PhysicsMaterial's friction values in place (does
not recreate Table/TrainingObject), then binds that same material to the
two gripper finger links. Binding is applied on the link Xform rather than
the instanced collision mesh underneath it (instanceable references don't
show up in a plain Stage.Traverse(), see isaac_dump_rl_asset_stage.py) —
USD physics-material binding is inherited down to descendant collision
prims unless something closer overrides it, so this still reaches the
actual finger collision geometry.
"""

from __future__ import annotations


MATERIAL_PATH = "/so101_new_calib/TrainingScene/PhysicsMaterial"
GRIPPER_LINK_BASE = (
    "/so101_new_calib/Geometry/base_link/shoulder_link/upper_arm_link"
    "/lower_arm_link/wrist_link/gripper_link"
)
FINGER_LINK_PATHS = [
    f"{GRIPPER_LINK_BASE}/moving_jaw_so101_v1_link",
    f"{GRIPPER_LINK_BASE}/gripper_frame_link",
]

NEW_STATIC_FRICTION = 1.0
NEW_DYNAMIC_FRICTION = 0.9


def main() -> None:
    import omni.usd
    from pxr import UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    material_prim = stage.GetPrimAtPath(MATERIAL_PATH)
    if not material_prim.IsValid():
        raise RuntimeError(
            f"{MATERIAL_PATH} not found — run "
            "isaac_create_gate2_training_scene.py first"
        )

    physics_material = UsdPhysics.MaterialAPI(material_prim)
    physics_material.GetStaticFrictionAttr().Set(NEW_STATIC_FRICTION)
    physics_material.GetDynamicFrictionAttr().Set(NEW_DYNAMIC_FRICTION)

    material = UsdShade.Material(material_prim)
    bound_paths = []
    for finger_path in FINGER_LINK_PATHS:
        finger_prim = stage.GetPrimAtPath(finger_path)
        if not finger_prim.IsValid():
            print(f"GATE2_FRICTION_SKIP missing_prim={finger_path}")
            continue
        UsdShade.MaterialBindingAPI.Apply(finger_prim).Bind(
            material, materialPurpose="physics"
        )
        bound_paths.append(finger_path)

    print(
        "GATE2_FRICTION_UPDATED "
        f"static={NEW_STATIC_FRICTION} dynamic={NEW_DYNAMIC_FRICTION} "
        f"bound_to={bound_paths}"
    )


if __name__ == "__main__":
    main()
