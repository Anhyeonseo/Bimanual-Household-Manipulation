#!/usr/bin/env python3
"""Create the Gate 2 training scene: a static table and a graspable object.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
already open, timeline stopped, joints at their q0 default pose. Delete any
placeholder table/object prims first (this script does not clean up prior
attempts).

Placement is anchored to the robot's actual current gripper_link world
position (queried live from the stage, not hand-computed FK), so it tracks
wherever the real robot is instead of an assumed workspace layout. The
table-drop and forward-offset distances below are still first-guess
placeholders: after running this, look at the viewport and nudge the
Table/TrainingObject Translate values by hand before pressing Play, per
Gate 2's own manual-verification process.
"""

from __future__ import annotations


SCENE_SCOPE_PATH = "/so101_new_calib/TrainingScene"
TABLE_PATH = f"{SCENE_SCOPE_PATH}/Table"
OBJECT_PATH = f"{SCENE_SCOPE_PATH}/TrainingObject"
MATERIAL_PATH = f"{SCENE_SCOPE_PATH}/PhysicsMaterial"
GRIPPER_LINK_PATH = (
    "/so101_new_calib/Geometry/base_link/shoulder_link/upper_arm_link"
    "/lower_arm_link/wrist_link/gripper_link"
)

TABLE_SIZE_M = (0.4, 0.4, 0.02)  # x, y, z
OBJECT_SIZE_M = (0.08, 0.025, 0.025)  # 80x25x25mm per Gate 2 spec
OBJECT_MASS_KG = 0.02  # ~20g per Gate 2 spec

# First-guess placeholders — adjust by hand in the viewport after running.
TABLE_DROP_BELOW_GRIPPER_M = 0.15
OBJECT_FORWARD_OFFSET_M = 0.05


def main() -> None:
    import omni.usd
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    gripper_prim = stage.GetPrimAtPath(GRIPPER_LINK_PATH)
    if not gripper_prim.IsValid():
        raise RuntimeError(f"gripper_link not found at {GRIPPER_LINK_PATH}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    gripper_pos = cache.GetLocalToWorldTransform(gripper_prim).ExtractTranslation()

    table_top_z = gripper_pos[2] - TABLE_DROP_BELOW_GRIPPER_M
    table_center = Gf.Vec3d(
        gripper_pos[0], gripper_pos[1], table_top_z - TABLE_SIZE_M[2] / 2.0
    )
    object_center = Gf.Vec3d(
        gripper_pos[0],
        gripper_pos[1] - OBJECT_FORWARD_OFFSET_M,
        table_top_z + OBJECT_SIZE_M[2] / 2.0,
    )

    UsdGeom.Scope.Define(stage, SCENE_SCOPE_PATH)

    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.8)
    physics_material.CreateDynamicFrictionAttr(0.7)
    physics_material.CreateRestitutionAttr(0.0)

    def make_box(path, size_m, center):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)  # unit cube; scale op below sets meters
        prim = cube.GetPrim()
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(center)
        xform.AddScaleOp().Set(Gf.Vec3f(*size_m))
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, materialPurpose="physics"
        )
        return prim

    table_prim = make_box(TABLE_PATH, TABLE_SIZE_M, table_center)
    UsdPhysics.CollisionAPI.Apply(table_prim)

    object_prim = make_box(OBJECT_PATH, OBJECT_SIZE_M, object_center)
    UsdPhysics.CollisionAPI.Apply(object_prim)
    UsdPhysics.RigidBodyAPI.Apply(object_prim)
    UsdPhysics.MassAPI.Apply(object_prim).CreateMassAttr(OBJECT_MASS_KG)

    print(
        "GATE2_SCENE_CREATED "
        f"gripper_world_pos={tuple(gripper_pos)} "
        f"table_center={tuple(table_center)} "
        f"object_center={tuple(object_center)}"
    )


if __name__ == "__main__":
    main()
