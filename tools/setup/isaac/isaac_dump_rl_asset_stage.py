#!/usr/bin/env python3
"""Dump the prim hierarchy of the currently open Isaac Sim stage.

Run this file only in Isaac Sim's Script Editor with so101_rl_asset.usd
already open. It is read-only: it does not modify the stage or the timeline.
It force-loads all payloads before traversing, otherwise collision/visual
mesh prims nested under payload arcs (see isaac_sim/README.md) are absent
from Stage.Traverse() and silently look like they don't exist.

Use it before authoring the Gate 2 training-object scene (table collider,
rigid body, physics material) so prim paths are taken from the real stage
instead of guessed.
"""

from __future__ import annotations


def main() -> None:
    import omni.usd
    from pxr import Sdf, Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open in Isaac Sim")

    stage.Load(Sdf.Path.absoluteRootPath, Usd.LoadWithDescendants)

    physics_schemas = (
        UsdPhysics.RigidBodyAPI,
        UsdPhysics.CollisionAPI,
        UsdPhysics.MassAPI,
        UsdPhysics.MaterialAPI,
    )

    print(f"STAGE_ROOT_LAYER {stage.GetRootLayer().identifier}")
    for prim in stage.Traverse():
        depth = str(prim.GetPath()).count("/") - 1
        applied = [
            schema.__name__.replace("API", "")
            for schema in physics_schemas
            if prim.HasAPI(schema)
        ]
        marker = f" [{', '.join(applied)}]" if applied else ""
        print(f"{'  ' * depth}{prim.GetPath()} ({prim.GetTypeName()}){marker}")


if __name__ == "__main__":
    main()
