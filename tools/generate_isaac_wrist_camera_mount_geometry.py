#!/usr/bin/env python3
"""Generate the Isaac USD geometry for the verified SO-101 wrist camera mount.

Run this script with the Python interpreter bundled with Isaac Sim 6.0.1.
It intentionally changes geometry only; link transforms, joints, mass, and
inertia remain authored by the existing robot USD layers.
"""

from __future__ import annotations

import argparse
import hashlib
import re
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, Vt


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = (
    ROOT
    / "ros2_ws/src/so101_description/meshes"
    / "wrist_cam_mount_32x32_uvc_module_so101.stl"
)
DEFAULT_OUTPUT = (
    ROOT
    / "isaac_sim/assets/so101_new_calib/payloads"
    / "wrist_camera_mount_geometry.usd"
)
EXPECTED_SHA256 = (
    "b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4"
)
GEOMETRY_ROOT = "/Geometries/wrist_roll_follower_so101_v1"
MESH_PATH = f"{GEOMETRY_ROOT}/wrist_roll_follower_so101_v1"
MILLIMETRES_TO_METRES = 0.001

FACET_RE = re.compile(
    r"^\s*facet normal\s+([^ ]+)\s+([^ ]+)\s+([^ \r\n]+)"
)
VERTEX_RE = re.compile(
    r"^\s*vertex\s+([^ ]+)\s+([^ ]+)\s+([^ \r\n]+)"
)


def parse_ascii_stl(path: Path):
    points = []
    point_indices = {}
    face_indices = []
    face_normals = []
    pending_face = []
    current_normal = (0.0, 0.0, 1.0)

    for line in path.read_text(encoding="ascii").splitlines():
        facet_match = FACET_RE.match(line)
        if facet_match:
            current_normal = tuple(float(value) for value in facet_match.groups())
            continue

        vertex_match = VERTEX_RE.match(line)
        if not vertex_match:
            continue

        point_mm = tuple(float(value) for value in vertex_match.groups())
        point = tuple(value * MILLIMETRES_TO_METRES for value in point_mm)
        index = point_indices.get(point)
        if index is None:
            index = len(points)
            point_indices[point] = index
            points.append(point)
        pending_face.append(index)

        if len(pending_face) == 3:
            face_indices.extend(pending_face)
            face_normals.append(current_normal)
            pending_face = []

    if pending_face or not face_normals:
        raise ValueError(f"invalid triangle ASCII STL: {path}")
    return points, face_indices, face_normals


def generate(input_path: Path, output_path: Path) -> None:
    digest = hashlib.sha256(input_path.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        raise ValueError(
            f"unexpected input SHA-256: expected {EXPECTED_SHA256}, got {digest}"
        )

    points, face_indices, face_normals = parse_ascii_stl(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root = UsdGeom.Xform.Define(stage, GEOMETRY_ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, MESH_PATH)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(face_normals)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
    mesh.CreateExtentAttr(
        Vt.Vec3fArray(
            [
                Gf.Vec3f(*(min(point[axis] for point in points) for axis in range(3))),
                Gf.Vec3f(*(max(point[axis] for point in points) for axis in range(3))),
            ]
        )
    )
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    normals = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "normals",
        Sdf.ValueTypeNames.Normal3fArray,
        UsdGeom.Tokens.uniform,
    )
    normals.Set(Vt.Vec3fArray([Gf.Vec3f(*normal) for normal in face_normals]))

    stage.GetRootLayer().Save()
    print(
        "ISAAC_WRIST_CAMERA_GEOMETRY_PASS "
        f"path={output_path} points={len(points)} faces={len(face_normals)} "
        f"sha256={digest}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    generate(args.input.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
