#!/usr/bin/env python3
"""Generate the physical overhead top rail with its broken hinge removed.

The source STL is the unmodified TheRobotStudio webcam top mount. The physical
rig retains both printed end structures, but the small center tip protruding
beyond raw y=234.4404 mm is physically broken and absent. This deterministic
half-space clip removes only that tip. URDF uses a conservative box collision.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct

MIN_Y_MM = 0.0
MAX_Y_MM = 234.4404
RECORD = struct.Struct("<12fH")


def read_triangles(path: Path) -> list[list[tuple[float, float, float]]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"not a binary STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * RECORD.size:
        raise ValueError(f"unexpected binary STL size: {path}")
    triangles = []
    offset = 84
    for _ in range(count):
        values = RECORD.unpack_from(data, offset)
        triangles.append(
            [
                (values[3], values[4], values[5]),
                (values[6], values[7], values[8]),
                (values[9], values[10], values[11]),
            ]
        )
        offset += RECORD.size
    return triangles


def intersection(a: tuple[float, float, float], b: tuple[float, float, float], plane: float):
    amount = (plane - a[1]) / (b[1] - a[1])
    return tuple(a[i] + amount * (b[i] - a[i]) for i in range(3))


def clip_polygon(polygon, plane: float, keep_above: bool):
    clipped = []
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        current_inside = current[1] >= plane if keep_above else current[1] <= plane
        previous_inside = previous[1] >= plane if keep_above else previous[1] <= plane
        if current_inside:
            if not previous_inside:
                clipped.append(intersection(previous, current, plane))
            clipped.append(current)
        elif previous_inside:
            clipped.append(intersection(previous, current, plane))
    return clipped


def clip_triangle(triangle):
    polygon = clip_polygon(triangle, MIN_Y_MM, keep_above=True)
    polygon = clip_polygon(polygon, MAX_Y_MM, keep_above=False)
    if len(polygon) < 3:
        return []
    return [[polygon[0], polygon[i], polygon[i + 1]] for i in range(1, len(polygon) - 1)]


def normal(triangle):
    a, b, c = triangle
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    value = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(component * component for component in value))
    if length <= 1e-12:
        return None
    return tuple(component / length for component in value)


def write_triangles(path: Path, triangles) -> None:
    records = []
    for triangle in triangles:
        face_normal = normal(triangle)
        if face_normal is None:
            continue
        records.append(
            RECORD.pack(
                *face_normal,
                *triangle[0],
                *triangle[1],
                *triangle[2],
                0,
            )
        )
    header = b"SO101 overhead top; center tip y>234.4404mm removed".ljust(80, b" ")
    path.write_bytes(header + struct.pack("<I", len(records)) + b"".join(records))
    print(f"WROTE {path} triangles={len(records)} y_mm=[{MIN_Y_MM}, {MAX_Y_MM}]")


def main() -> None:
    repo = Path(__file__).resolve().parents[3]
    default_source = repo / "ros2_ws/src/so101_description/meshes/overhead_webcam_cam_mount_top.stl"
    default_output = repo / "ros2_ws/src/so101_description/meshes/overhead_webcam_cam_mount_top_hinge_removed.stl"
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=default_source)
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()
    clipped = []
    for triangle in read_triangles(args.source):
        clipped.extend(clip_triangle(triangle))
    write_triangles(args.output, clipped)


if __name__ == "__main__":
    main()
