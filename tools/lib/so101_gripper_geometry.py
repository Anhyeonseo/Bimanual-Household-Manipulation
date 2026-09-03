"""SO-101 physical-Q0 to detailed-jaw-model coordinate mapping."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


SIDES = ("left", "right")
LAYERS = (1, 4)


@dataclass(frozen=True, slots=True)
class GripperGeometryCandidate:
    path: Path
    status: str
    q0_gap_mm: dict[str, float]
    q0_gap_uncertainty_mm: float
    fixed_jaw_rubber_pad_thickness_m: float
    moving_jaw_has_matching_rubber_pad: bool
    measurements_include_fixed_jaw_rubber_pad: bool
    model_q_at_physical_q0_rad: float
    model_q_range_rad: tuple[float, float]
    grasp_project_rad: dict[str, dict[int, float]]
    project_limits_rad: dict[str, tuple[float, float]]
    release_project_rad: float
    simulation_release_open_fraction: float
    rubber_static_friction: float
    rubber_dynamic_friction: float
    rubber_restitution: float

    def project_to_model(self, project_rad: float) -> float:
        if not math.isfinite(project_rad):
            raise ValueError("project gripper position must be finite")
        return self.model_q_at_physical_q0_rad - project_rad

    def model_to_project(self, model_rad: float) -> float:
        if not math.isfinite(model_rad):
            raise ValueError("model gripper position must be finite")
        return self.model_q_at_physical_q0_rad - model_rad

    def grasp_model_rad(self, side: str, layers: int) -> float:
        return self.project_to_model(self.grasp_project_rad[side][layers])

    def model_limits_rad(self, side: str) -> tuple[float, float]:
        project_lower, project_upper = self.project_limits_rad[side]
        return (
            self.project_to_model(project_upper),
            self.project_to_model(project_lower),
        )

    @property
    def release_model_rad(self) -> float:
        return self.project_to_model(self.release_project_rad)

    def simulation_release_model_rad(self, side: str) -> float:
        """Return the side-specific medium-open simulation release target."""
        maximum_open_project_rad = self.project_limits_rad[side][0]
        project_rad = self.release_project_rad + self.simulation_release_open_fraction * (
            maximum_open_project_rad - self.release_project_rad
        )
        return self.project_to_model(project_rad)


def _finite(mapping: dict[str, Any], key: str, *, allow_zero: bool = False) -> float:
    value = float(mapping[key])
    if not math.isfinite(value) or (value < 0.0 if allow_zero else value <= 0.0):
        raise ValueError(f"invalid finite value for {key}: {value}")
    return value


def load_gripper_geometry_candidate(path: Path) -> GripperGeometryCandidate:
    resolved = path.expanduser().resolve()
    document = json.loads(resolved.read_text(encoding="utf-8"))
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("record_kind") != "so101_gripper_geometry_candidate"
        or document.get("status")
        != "R2_Q0_GAP_STATIC_RETENTION_ANCHORED_CANDIDATE"
        or document.get("simulation_only") is not True
        or document.get("motion_authorized") is not False
    ):
        raise ValueError("gripper geometry candidate identity or motion lock is invalid")

    q0 = document["q0_measurement"]
    geometry = document["geometry"]
    if (
        q0.get("coordinate") != "canonical_project_rad"
        or float(q0.get("canonical_project_q0_rad", math.nan)) != 0.0
        or q0.get("physical_reference") != "raw_2048"
        or q0.get("measurement_surface_reference")
        != "fixed-jaw rubber-pad outer surface to opposing moving-jaw face"
        or geometry.get("model_positive_direction") != "opens"
        or geometry.get("project_positive_direction") != "closes"
    ):
        raise ValueError("gripper Q0 coordinate or sign contract is invalid")

    model_q = _finite(geometry, "detailed_stl_model_q_at_physical_q0_rad")
    fixed_pad = geometry.get("fixed_jaw_rubber_pad", {})
    if (
        fixed_pad.get("installed_on_both_arms") is not True
        or fixed_pad.get("moving_jaw_has_matching_pad") is not False
    ):
        raise ValueError("fixed-jaw-only rubber pad geometry is invalid")
    fixed_pad_thickness_m = _finite(fixed_pad, "thickness_mm") * 0.001
    model_range = tuple(
        float(value)
        for value in geometry["detailed_stl_model_q_range_from_gap_uncertainty_rad"]
    )
    if (
        len(model_range) != 2
        or not all(math.isfinite(value) for value in model_range)
        or not model_range[0] <= model_q <= model_range[1]
    ):
        raise ValueError("gripper Q0 model range is invalid")

    commands = document["grasp_commands"]
    if (
        commands.get("coordinate") != "canonical_project_rad"
        or commands.get("pad_condition_already_included_in_measured_commands")
        is not True
    ):
        raise ValueError("grasp commands are not canonical project radians")
    grasp_project_rad: dict[str, dict[int, float]] = {}
    for side in SIDES:
        side_commands = commands[side]
        grasp_project_rad[side] = {}
        for layers, key in ((1, "one_layer"), (4, "four_layer")):
            entry = side_commands[key]
            if entry.get("use_operational_candidate") is not True:
                raise ValueError(f"{side} {layers}-layer operational candidate disabled")
            grasp_project_rad[side][layers] = _finite(
                entry, "operational_candidate_rad"
            )

    rubber = document["generic_rubber_cloth_material_candidate"]
    static_friction = _finite(rubber, "static_friction")
    dynamic_friction = _finite(rubber, "dynamic_friction")
    if static_friction < dynamic_friction or rubber.get("measured") is not False:
        raise ValueError("generic rubber friction candidate is invalid")

    limits_document = document["operational_project_limits_rad"]
    if limits_document.get("source") != "config/bimanual_operational_limits.json":
        raise ValueError("gripper operational limit provenance is invalid")
    project_limits_rad: dict[str, tuple[float, float]] = {}
    for side in SIDES:
        limits = tuple(float(value) for value in limits_document[side])
        if (
            len(limits) != 2
            or not all(math.isfinite(value) for value in limits)
            or limits[0] >= limits[1]
        ):
            raise ValueError(f"{side} gripper project limits are invalid")
        project_limits_rad[side] = (limits[0], limits[1])

    simulation_release_open_fraction = float(
        document["release"]["simulation_open_fraction_from_q0_to_max"]
    )
    if not 0.0 < simulation_release_open_fraction <= 1.0:
        raise ValueError("simulation release open fraction must be in (0, 1]")

    return GripperGeometryCandidate(
        path=resolved,
        status=str(document["status"]),
        q0_gap_mm={side: _finite(q0, f"{side}_gap_mm") for side in SIDES},
        q0_gap_uncertainty_mm=_finite(q0, "estimated_uncertainty_mm"),
        fixed_jaw_rubber_pad_thickness_m=fixed_pad_thickness_m,
        moving_jaw_has_matching_rubber_pad=False,
        measurements_include_fixed_jaw_rubber_pad=True,
        model_q_at_physical_q0_rad=model_q,
        model_q_range_rad=(model_range[0], model_range[1]),
        grasp_project_rad=grasp_project_rad,
        project_limits_rad=project_limits_rad,
        release_project_rad=float(document["release"]["canonical_project_rad"]),
        simulation_release_open_fraction=simulation_release_open_fraction,
        rubber_static_friction=static_friction,
        rubber_dynamic_friction=dynamic_friction,
        rubber_restitution=_finite(rubber, "restitution", allow_zero=True),
    )
