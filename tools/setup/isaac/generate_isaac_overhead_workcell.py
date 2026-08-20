#!/usr/bin/env python3
"""Generate the user-confirmed overhead workcell layer for Isaac Sim 6.0.1.

Run this script with the Python interpreter bundled with Isaac Sim 6.0.1.
The generated layer is deliberately visual-only: it authors no rigid bodies,
collision APIs, articulation membership, or active camera sensor.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import struct
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, Vt


ROOT = Path(__file__).resolve().parents[3]
MESH_DIR = ROOT / "ros2_ws/src/so101_description/meshes"
DEFAULT_OUTPUT = (
    ROOT
    / "isaac_sim/assets/so101_new_calib/payloads"
    / "overhead_workcell.usd"
)

EXPECTED_MESH_SHA256 = {
    "overhead_webcam_arm_base.stl": (
        "169adfd40bcca689334efd1188c9b42cc03c914dc0afeaa98cb5431013610833"
    ),
    "overhead_webcam_cam_mount_bottom.stl": (
        "b3545b6cae437210e17b7dcfee2e12e00dc7a59ece9264f4b13ab9fd8ceb8088"
    ),
    "overhead_webcam_cam_mount_top_hinge_removed.stl": (
        "55319a9b26f9cdb7217c94000f7aec716d63b0a433150cd7672baa7b85a006cf"
    ),
}

WORKCELL_ROOT = "/so101_new_calib/Workcell"
MILLIMETRES_TO_METRES = 0.001

# These local transforms exactly mirror so101_overhead_webcam_mount.xacro.
ARM_BASE_TRANSLATION = (0.0702562210, -0.0534304657, 0.0)
ARM_BASE_RPY = (math.pi / 2.0, 0.0, 0.0)
BOTTOM_TRANSLATION = (0.0137353073, -0.1525445687, -0.005)
BOTTOM_RPY = (math.pi / 2.0, 0.0, math.pi)
TOP_TRANSLATION = (0.0187, 0.2231, 0.0365125)
TOP_RPY = (0.0, 0.0, 0.0)
CAMERA_TRANSLATION = (0.0, 0.2344404, 0.0)
CAMERA_OPTICAL_RPY = (math.pi / 2.0, 0.0, 0.0)


def _parse_binary_stl(path: Path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"invalid binary STL header: {path}")

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(
            f"invalid binary STL size: {path}: expected {expected_size}, got {len(data)}"
        )

    points = []
    point_indices = {}
    face_indices = []
    face_normals = []

    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", data, offset)
        normal = values[0:3]
        vertices = (values[3:6], values[6:9], values[9:12])
        for vertex_mm in vertices:
            point = tuple(value * MILLIMETRES_TO_METRES for value in vertex_mm)
            index = point_indices.get(point)
            if index is None:
                index = len(points)
                point_indices[point] = index
                points.append(point)
            face_indices.append(index)
        face_normals.append(normal)
        offset += 50

    if not face_normals:
        raise ValueError(f"STL contains no triangles: {path}")
    return points, face_indices, face_normals


def _quaternion_from_rpy(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return Gf.Quatd(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _define_xform(stage, path: str, translation, rpy):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        _quaternion_from_rpy(rpy)
    )
    return xform


def _define_mesh(stage, parent_path: str, input_path: Path) -> tuple[int, int]:
    points, face_indices, face_normals = _parse_binary_stl(input_path)
    mesh = UsdGeom.Mesh.Define(stage, f"{parent_path}/mesh")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(face_normals)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
    mesh.CreateExtentAttr(
        Vt.Vec3fArray(
            [
                Gf.Vec3f(
                    *(min(point[axis] for point in points) for axis in range(3))
                ),
                Gf.Vec3f(
                    *(max(point[axis] for point in points) for axis in range(3))
                ),
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
    return len(points), len(face_normals)


def generate(mesh_dir: Path, output_path: Path) -> None:
    for filename, expected_digest in EXPECTED_MESH_SHA256.items():
        input_path = mesh_dir / filename
        digest = hashlib.sha256(input_path.read_bytes()).hexdigest()
        if digest != expected_digest:
            raise ValueError(
                f"unexpected {filename} SHA-256: "
                f"expected {expected_digest}, got {digest}"
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    robot_root = UsdGeom.Xform.Define(stage, "/so101_new_calib")
    stage.SetDefaultPrim(robot_root.GetPrim())
    workcell = UsdGeom.Xform.Define(stage, WORKCELL_ROOT)
    workcell.GetPrim().SetCustomDataByKey(
        "assemblyStatus", "RVIZ_USER_CONFIRMED_2026-07-28"
    )
    workcell.GetPrim().SetCustomDataByKey("visualOnly", True)
    workcell.GetPrim().SetCustomDataByKey("collisionAuthorized", False)
    workcell.GetPrim().SetCustomDataByKey("cameraExtrinsicCalibrated", False)

    arm_path = f"{WORKCELL_ROOT}/arm_base"
    bottom_path = f"{WORKCELL_ROOT}/cam_mount_bottom"
    top_path = f"{bottom_path}/cam_mount_top"
    camera_path = f"{top_path}/top_camera_link"
    optical_path = f"{camera_path}/top_camera_optical_frame"

    _define_xform(stage, arm_path, ARM_BASE_TRANSLATION, ARM_BASE_RPY)
    _define_xform(stage, bottom_path, BOTTOM_TRANSLATION, BOTTOM_RPY)
    _define_xform(stage, top_path, TOP_TRANSLATION, TOP_RPY)
    _define_xform(stage, camera_path, CAMERA_TRANSLATION, (0.0, 0.0, 0.0))
    _define_xform(stage, optical_path, (0.0, 0.0, 0.0), CAMERA_OPTICAL_RPY)

    counts = {}
    counts["arm_base"] = _define_mesh(
        stage, arm_path, mesh_dir / "overhead_webcam_arm_base.stl"
    )
    counts["cam_mount_bottom"] = _define_mesh(
        stage, bottom_path, mesh_dir / "overhead_webcam_cam_mount_bottom.stl"
    )
    counts["cam_mount_top"] = _define_mesh(
        stage,
        top_path,
        mesh_dir / "overhead_webcam_cam_mount_top_hinge_removed.stl",
    )

    stage.GetRootLayer().Save()
    summary = " ".join(
        f"{name}_points={point_count} {name}_faces={face_count}"
        for name, (point_count, face_count) in counts.items()
    )
    print(f"ISAAC_OVERHEAD_WORKCELL_PASS path={output_path} {summary}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh-dir", type=Path, default=MESH_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    generate(args.mesh_dir.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
