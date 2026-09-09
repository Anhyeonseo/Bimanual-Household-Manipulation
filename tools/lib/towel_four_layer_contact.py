"""Continuous-surface contact gates for a four-layer towel pinch.

The calibrated towel mesh is intentionally coarse.  A real jaw can contact
the interior of a cloth triangle even when none of that triangle's vertices
lies on the 6 mm jaw face.  This module clips cloth triangles against the
finite inward contact prism of each registered jaw face and therefore keeps
contact qualification separate from the vertices used to impose the finite
element boundary condition.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Sequence

import numpy as np


class FourLayerContactError(RuntimeError):
    """The folded towel does not form a valid four-layer jaw pinch."""


@dataclass(frozen=True, slots=True)
class RegisteredFaceFrame:
    center_m: tuple[float, float, float]
    inward_normal: tuple[float, float, float]
    tangent_u: tuple[float, float, float]
    tangent_v: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class SurfaceContactAnchor:
    face: str
    s1_layer: str
    triangle_indices: tuple[int, int, int]
    barycentric_weights: tuple[float, float, float]
    point_world_m: tuple[float, float, float]
    face_local_nuv_m: tuple[float, float, float]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class FourLayerSurfacePinch:
    anchors: tuple[SurfaceContactAnchor, ...]
    support_vertex_indices: tuple[int, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "gate": "four_continuous_surface_contacts_on_registered_finite_faces",
            "physical_contact_count": len(self.anchors),
            "anchors": [anchor.to_dict() for anchor in self.anchors],
            "finite_element_support_vertex_indices": list(
                self.support_vertex_indices
            ),
            "finite_element_support_vertex_count": len(
                self.support_vertex_indices
            ),
        }


@dataclass(frozen=True, slots=True)
class SingleSheetSurfacePinch:
    """One continuous cloth sheet touching both opposing jaw faces."""

    anchors: tuple[SurfaceContactAnchor, SurfaceContactAnchor]
    support_vertex_indices: tuple[int, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "gate": "two_continuous_surface_contacts_on_registered_finite_faces",
            "physical_contact_count": len(self.anchors),
            "anchors": [anchor.to_dict() for anchor in self.anchors],
            "finite_element_support_vertex_indices": list(
                self.support_vertex_indices
            ),
            "finite_element_support_vertex_count": len(
                self.support_vertex_indices
            ),
        }


def _unit(value: Sequence[float], name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise FourLayerContactError(f"{name} must be a finite XYZ vector")
    length = float(np.linalg.norm(vector))
    if length <= 1.0e-9:
        raise FourLayerContactError(f"{name} must be non-zero")
    return vector / length


def _validated_frame(frame: RegisteredFaceFrame, name: str) -> tuple[np.ndarray, ...]:
    center = np.asarray(frame.center_m, dtype=np.float64)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise FourLayerContactError(f"{name} center must be finite XYZ")
    normal = _unit(frame.inward_normal, f"{name} inward normal")
    tangent_u = _unit(frame.tangent_u, f"{name} tangent u")
    tangent_u = tangent_u - normal * float(np.dot(tangent_u, normal))
    tangent_u = _unit(tangent_u, f"{name} orthogonal tangent u")
    tangent_v = np.cross(normal, tangent_u)
    if float(np.dot(tangent_v, _unit(frame.tangent_v, f"{name} tangent v"))) < 0.0:
        tangent_v = -tangent_v
    return center, normal, tangent_u, tangent_v


def _clip_polygon_axis(
    polygon: list[np.ndarray], axis: int, bound: float, keep_less_equal: bool
) -> list[np.ndarray]:
    if not polygon:
        return []

    def inside(point: np.ndarray) -> bool:
        return bool(
            point[axis] <= bound + 1.0e-12
            if keep_less_equal
            else point[axis] >= bound - 1.0e-12
        )

    result: list[np.ndarray] = []
    previous = polygon[-1]
    previous_inside = inside(previous)
    for current in polygon:
        current_inside = inside(current)
        if current_inside != previous_inside:
            denominator = float(current[axis] - previous[axis])
            if abs(denominator) > 1.0e-15:
                alpha = (bound - float(previous[axis])) / denominator
                result.append(previous + alpha * (current - previous))
        if current_inside:
            result.append(current)
        previous = current
        previous_inside = current_inside
    return result


def _clip_triangle_to_face_prism(
    triangle_world: np.ndarray,
    frame: RegisteredFaceFrame,
    *,
    half_face_size_m: float,
    inward_depth_m: float,
    backside_tolerance_m: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    center, normal, tangent_u, tangent_v = _validated_frame(frame, "jaw face")
    basis = np.stack((normal, tangent_u, tangent_v), axis=1)
    local_triangle = (triangle_world - center) @ basis
    polygon = [point.copy() for point in local_triangle]
    for axis, lower, upper in (
        (0, -backside_tolerance_m, inward_depth_m),
        (1, -half_face_size_m, half_face_size_m),
        (2, -half_face_size_m, half_face_size_m),
    ):
        polygon = _clip_polygon_axis(polygon, axis, lower, False)
        polygon = _clip_polygon_axis(polygon, axis, upper, True)
        if not polygon:
            return None
    local_point = np.mean(np.stack(polygon), axis=0)
    world_point = center + basis @ local_point
    return world_point, local_point


def _barycentric_weights(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    edge_0 = triangle[1] - triangle[0]
    edge_1 = triangle[2] - triangle[0]
    offset = point - triangle[0]
    d00 = float(np.dot(edge_0, edge_0))
    d01 = float(np.dot(edge_0, edge_1))
    d11 = float(np.dot(edge_1, edge_1))
    d20 = float(np.dot(offset, edge_0))
    d21 = float(np.dot(offset, edge_1))
    denominator = d00 * d11 - d01 * d01
    if abs(denominator) <= 1.0e-18:
        raise FourLayerContactError("contact triangle is degenerate")
    v = (d11 * d20 - d01 * d21) / denominator
    w = (d00 * d21 - d01 * d20) / denominator
    weights = np.asarray((1.0 - v - w, v, w), dtype=np.float64)
    weights[np.abs(weights) < 1.0e-10] = 0.0
    weights = np.clip(weights, 0.0, 1.0)
    weights /= float(np.sum(weights))
    return weights


def _closest_point_on_triangle(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    """Return the Euclidean closest point using Ericson's Voronoi regions."""
    a, b, c = triangle
    ab = b - a
    ac = c - a
    ap = point - a
    d1 = float(np.dot(ab, ap))
    d2 = float(np.dot(ac, ap))
    if d1 <= 0.0 and d2 <= 0.0:
        return a
    bp = point - b
    d3 = float(np.dot(ab, bp))
    d4 = float(np.dot(ac, bp))
    if d3 >= 0.0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return a + (d1 / (d1 - d3)) * ab
    cp = point - c
    d5 = float(np.dot(ab, cp))
    d6 = float(np.dot(ac, cp))
    if d6 >= 0.0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return a + (d2 / (d2 - d6)) * ac
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and d4 - d3 >= 0.0 and d5 - d6 >= 0.0:
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b)
    denominator = 1.0 / (va + vb + vc)
    return a + (vb * denominator) * ab + (vc * denominator) * ac


def _nearest_triangle_diagnostic(
    nodes: np.ndarray,
    triangles: Iterable[tuple[int, int, int]],
    frame: RegisteredFaceFrame,
) -> dict[str, object]:
    center, normal, tangent_u, tangent_v = _validated_frame(frame, "jaw face")
    basis = np.stack((normal, tangent_u, tangent_v), axis=1)
    records = []
    for triangle_indices in triangles:
        point = _closest_point_on_triangle(
            center, nodes[np.asarray(triangle_indices)]
        )
        local = (point - center) @ basis
        records.append(
            (
                float(np.linalg.norm(point - center)),
                triangle_indices,
                point,
                local,
            )
        )
    distance, triangle_indices, point, local = min(
        records, key=lambda item: (item[0], item[1])
    )
    return {
        "center_distance_m": distance,
        "triangle_indices": list(triangle_indices),
        "point_world_m": [float(value) for value in point],
        "face_local_nuv_m": [float(value) for value in local],
    }


def _nearest_face_diagnostic(
    nodes: np.ndarray,
    grid_side: int,
    layer: str,
    frame: RegisteredFaceFrame,
) -> dict[str, object]:
    return _nearest_triangle_diagnostic(
        nodes, _layer_triangles(grid_side, layer), frame
    )


def _layer_triangles(grid_side: int, layer: str) -> Iterable[tuple[int, int, int]]:
    half = grid_side // 2
    if layer == "s1_first_half":
        columns = range(0, half - 1)
    elif layer == "s1_second_half":
        columns = range(half, grid_side - 1)
    else:
        raise FourLayerContactError(f"unknown S1 layer: {layer}")
    for row in range(grid_side - 1):
        for column in columns:
            v0 = row * grid_side + column
            v1 = v0 + 1
            v3 = v0 + grid_side
            v2 = v3 + 1
            yield (v0, v1, v3)
            yield (v1, v2, v3)


def _single_sheet_triangles(grid_side: int) -> Iterable[tuple[int, int, int]]:
    for row in range(grid_side - 1):
        for column in range(grid_side - 1):
            v0 = row * grid_side + column
            v1 = v0 + 1
            v3 = v0 + grid_side
            v2 = v3 + 1
            yield (v0, v1, v3)
            yield (v1, v2, v3)


def _triangle_candidates(
    nodes: np.ndarray,
    triangles: Iterable[tuple[int, int, int]],
    layer: str,
    face_name: str,
    frame: RegisteredFaceFrame,
    *,
    half_face_size_m: float,
    inward_depth_m: float,
    backside_tolerance_m: float,
) -> list[SurfaceContactAnchor]:
    candidates: list[SurfaceContactAnchor] = []
    center = np.asarray(frame.center_m, dtype=np.float64)
    for triangle_indices in triangles:
        triangle = nodes[np.asarray(triangle_indices)]
        clipped = _clip_triangle_to_face_prism(
            triangle,
            frame,
            half_face_size_m=half_face_size_m,
            inward_depth_m=inward_depth_m,
            backside_tolerance_m=backside_tolerance_m,
        )
        if clipped is None:
            continue
        point, local_point = clipped
        weights = _barycentric_weights(point, triangle)
        candidates.append(
            SurfaceContactAnchor(
                face=face_name,
                s1_layer=layer,
                triangle_indices=triangle_indices,
                barycentric_weights=tuple(float(value) for value in weights),
                point_world_m=tuple(float(value) for value in point),
                face_local_nuv_m=tuple(float(value) for value in local_point),
            )
        )
    candidates.sort(
        key=lambda anchor: (
            math.dist(anchor.point_world_m, center),
            anchor.triangle_indices,
        )
    )
    return candidates


def _face_candidates(
    nodes: np.ndarray,
    grid_side: int,
    layer: str,
    face_name: str,
    frame: RegisteredFaceFrame,
    *,
    half_face_size_m: float,
    inward_depth_m: float,
    backside_tolerance_m: float,
) -> list[SurfaceContactAnchor]:
    return _triangle_candidates(
        nodes,
        _layer_triangles(grid_side, layer),
        layer,
        face_name,
        frame,
        half_face_size_m=half_face_size_m,
        inward_depth_m=inward_depth_m,
        backside_tolerance_m=backside_tolerance_m,
    )


def select_continuous_single_sheet_pinch(
    nodes_world_m: Sequence[Sequence[float]],
    *,
    grid_side: int,
    fixed_face: RegisteredFaceFrame,
    moving_face: RegisteredFaceFrame,
    face_size_m: float,
    inward_depth_m: float,
    backside_tolerance_m: float = 0.00075,
    maximum_topology_gap_cells: int = 2,
) -> SingleSheetSurfacePinch:
    """Require a single continuous towel surface on both finite jaw faces."""
    nodes = np.asarray(nodes_world_m, dtype=np.float64)
    if nodes.shape != (grid_side * grid_side, 3) or not np.all(np.isfinite(nodes)):
        raise FourLayerContactError("cloth nodes do not match the square grid")
    if grid_side < 2:
        raise FourLayerContactError("single-sheet topology requires a grid")
    fixed_normal = _unit(fixed_face.inward_normal, "fixed inward normal")
    moving_normal = _unit(moving_face.inward_normal, "moving inward normal")
    if float(np.dot(fixed_normal, moving_normal)) > -0.95:
        raise FourLayerContactError("registered jaw face normals are not opposing")
    half_face_size_m = 0.5 * face_size_m
    fixed_candidates = _triangle_candidates(
        nodes,
        _single_sheet_triangles(grid_side),
        "single_sheet",
        "fixed",
        fixed_face,
        half_face_size_m=half_face_size_m,
        inward_depth_m=inward_depth_m,
        backside_tolerance_m=backside_tolerance_m,
    )
    moving_candidates = _triangle_candidates(
        nodes,
        _single_sheet_triangles(grid_side),
        "single_sheet",
        "moving",
        moving_face,
        half_face_size_m=half_face_size_m,
        inward_depth_m=inward_depth_m,
        backside_tolerance_m=backside_tolerance_m,
    )
    pairs = [
        (fixed, moving)
        for fixed in fixed_candidates
        for moving in moving_candidates
        if _triangle_grid_distance(
            fixed.triangle_indices, moving.triangle_indices, grid_side
        )
        <= maximum_topology_gap_cells
    ]
    if not pairs:
        raise FourLayerContactError(
            "single sheet does not contact both registered jaw faces locally; "
            f"fixed_triangles={len(fixed_candidates)}, "
            f"moving_triangles={len(moving_candidates)}, "
            "nearest_fixed="
            f"{_nearest_triangle_diagnostic(nodes, _single_sheet_triangles(grid_side), fixed_face)}, "
            "nearest_moving="
            f"{_nearest_triangle_diagnostic(nodes, _single_sheet_triangles(grid_side), moving_face)}"
        )
    fixed, moving = min(
        pairs,
        key=lambda pair: (
            _triangle_grid_distance(
                pair[0].triangle_indices, pair[1].triangle_indices, grid_side
            ),
            math.dist(pair[0].point_world_m, pair[1].point_world_m),
            pair[0].triangle_indices,
            pair[1].triangle_indices,
        ),
    )
    support_vertices = tuple(
        sorted(set(fixed.triangle_indices) | set(moving.triangle_indices))
    )
    return SingleSheetSurfacePinch((fixed, moving), support_vertices)


def _triangle_grid_distance(
    first: tuple[int, int, int], second: tuple[int, int, int], grid_side: int
) -> int:
    return min(
        max(abs(a // grid_side - b // grid_side), abs(a % grid_side - b % grid_side))
        for a in first
        for b in second
    )


def select_continuous_four_layer_pinch(
    nodes_world_m: Sequence[Sequence[float]],
    *,
    grid_side: int,
    fixed_face: RegisteredFaceFrame,
    moving_face: RegisteredFaceFrame,
    face_size_m: float,
    inward_depth_m: float,
    backside_tolerance_m: float = 0.00075,
    maximum_topology_gap_cells: int = 2,
) -> FourLayerSurfacePinch:
    """Require two S1 layers to contact both opposing finite jaw faces.

    Four returned anchors represent the physical four-layer U-pinch: fixed and
    moving face contact for each of the two layers already present after S1.
    The support vertices are a separate finite-element implementation detail.
    """
    nodes = np.asarray(nodes_world_m, dtype=np.float64)
    if nodes.shape != (grid_side * grid_side, 3) or not np.all(np.isfinite(nodes)):
        raise FourLayerContactError("cloth nodes do not match the square grid")
    if grid_side < 4 or grid_side % 2 != 0:
        raise FourLayerContactError("four-layer topology requires an even grid side")
    for name, value in (
        ("face size", face_size_m),
        ("inward depth", inward_depth_m),
        ("backside tolerance", backside_tolerance_m),
    ):
        if not math.isfinite(value) or value < 0.0 or (name != "backside tolerance" and value == 0.0):
            raise FourLayerContactError(f"{name} must be finite and positive")
    fixed_normal = _unit(fixed_face.inward_normal, "fixed inward normal")
    moving_normal = _unit(moving_face.inward_normal, "moving inward normal")
    if float(np.dot(fixed_normal, moving_normal)) > -0.95:
        raise FourLayerContactError("registered jaw face normals are not opposing")

    selected: list[SurfaceContactAnchor] = []
    half_face_size_m = 0.5 * face_size_m
    for layer in ("s1_first_half", "s1_second_half"):
        fixed_candidates = _face_candidates(
            nodes,
            grid_side,
            layer,
            "fixed",
            fixed_face,
            half_face_size_m=half_face_size_m,
            inward_depth_m=inward_depth_m,
            backside_tolerance_m=backside_tolerance_m,
        )
        moving_candidates = _face_candidates(
            nodes,
            grid_side,
            layer,
            "moving",
            moving_face,
            half_face_size_m=half_face_size_m,
            inward_depth_m=inward_depth_m,
            backside_tolerance_m=backside_tolerance_m,
        )
        pairs = [
            (fixed, moving)
            for fixed in fixed_candidates
            for moving in moving_candidates
            if _triangle_grid_distance(
                fixed.triangle_indices, moving.triangle_indices, grid_side
            )
            <= maximum_topology_gap_cells
        ]
        if not pairs:
            raise FourLayerContactError(
                f"{layer} does not contact both registered jaw faces locally; "
                f"fixed_triangles={len(fixed_candidates)}, "
                f"moving_triangles={len(moving_candidates)}, "
                f"nearest_fixed={_nearest_face_diagnostic(nodes, grid_side, layer, fixed_face)}, "
                f"nearest_moving={_nearest_face_diagnostic(nodes, grid_side, layer, moving_face)}"
            )
        fixed, moving = min(
            pairs,
            key=lambda pair: (
                _triangle_grid_distance(
                    pair[0].triangle_indices, pair[1].triangle_indices, grid_side
                ),
                math.dist(pair[0].point_world_m, pair[1].point_world_m),
                pair[0].triangle_indices,
                pair[1].triangle_indices,
            ),
        )
        selected.extend((fixed, moving))

    if len(selected) != 4:
        raise FourLayerContactError("four-layer pinch must contain four surface contacts")
    support_vertices = tuple(
        sorted(
            {
                index
                for anchor in selected
                for index in anchor.triangle_indices
            }
        )
    )
    return FourLayerSurfacePinch(tuple(selected), support_vertices)
