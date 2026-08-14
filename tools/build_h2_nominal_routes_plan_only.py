#!/usr/bin/env python3
"""Build exact-continuity H2 nominal routes without any execution API.

The builder SHA-pins the validated pick/lift/place endpoint plans and one H2
tracking-envelope artifact.  It asks MoveIt to collision-check seven fresh
segment chains whose starts are the preceding *planned* endpoints, then reuses
the existing full-cycle manifest assembler.

Passing this tool is intentionally not H2 motion authority.  MoveIt currently
checks the nominal robot path only; environment geometry and the measured
joint-error envelope still need a robust state-validity pass.  The report keeps
those gates false so a nominal plan cannot be mistaken for executable evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"
ASSEMBLER = ROOT / "tools" / "assemble_pick_place_plan_only.py"
STATUS = (
    "H2_NOMINAL_ROUTES_PLAN_ONLY_PASS_"
    "AWAITING_ROUTE_ENVELOPE_AND_ROBUST_COLLISION_CHECK"
)
JOINT_COUNT = 5
Q0 = (0.0,) * JOINT_COUNT
# Keep generated segments inside the same per-step bound enforced by the
# full-cycle assembler.  The segment planner otherwise defaults to 0.30 rad.
STAGE7_MAX_JOINT_STEP_RAD = 0.18


@dataclass(frozen=True, slots=True)
class EndpointPlan:
    path: Path
    sha256: str
    targets: dict[str, tuple[float, ...]]
    cartesian_targets: dict[str, tuple[float, ...]]
    yaw_rad: dict[str, float]


@dataclass(frozen=True, slots=True)
class PhaseSpec:
    name: str
    start: tuple[float, ...]
    source_plan: Path
    target_name: str


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _finite_vector(value: Any, count: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must contain exactly {count} values")
    result = tuple(float(item) for item in value)
    if not all(abs(item) < 10.0 for item in result):
        raise ValueError(f"{label} contains an implausible value")
    return result


def load_pinned_endpoint(path: Path, expected_sha256: str) -> EndpointPlan:
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(
            f"endpoint sha256 mismatch path={path} expected={expected_sha256.lower()} "
            f"actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != "PLAN_ONLY_PASS":
        raise ValueError(f"endpoint is not PLAN_ONLY_PASS: {path}")
    if document.get("execution_api_used") is not False:
        raise ValueError(f"endpoint used an execution API: {path}")
    if document.get("motion_authorized") is not False:
        raise ValueError(f"endpoint must keep motion_authorized=false: {path}")
    plans = document.get("plans")
    if not isinstance(plans, list) or len(plans) != 2:
        raise ValueError(f"endpoint must contain pregrasp and grasp: {path}")
    targets: dict[str, tuple[float, ...]] = {}
    cartesian: dict[str, tuple[float, ...]] = {}
    yaw: dict[str, float] = {}
    for plan in plans:
        name = plan.get("name")
        if name not in {"pregrasp", "grasp"} or name in targets:
            raise ValueError(f"endpoint target names are invalid: {path}")
        if plan.get("success") is not True or plan.get("moveit_error_code") != 1:
            raise ValueError(f"endpoint target is not MoveIt-successful: {path}:{name}")
        targets[name] = _finite_vector(
            plan.get("final_joint_positions_rad"), JOINT_COUNT, f"{path}:{name}.joint"
        )
        cartesian[name] = _finite_vector(
            plan.get("target_m"), 3, f"{path}:{name}.target_m"
        )
        yaw[name] = float(plan.get("yaw_rad"))
    return EndpointPlan(path, actual, targets, cartesian, yaw)


def load_tracking_envelope(path: Path, expected_sha256: str) -> dict[str, Any]:
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(
            f"H2 evidence sha256 mismatch expected={expected_sha256.lower()} actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    envelope = (document.get("startup_trend") or {}).get("h2_tracking_envelope")
    maximum = None if not isinstance(envelope, dict) else envelope.get("maximum_error_raw")
    valid_count = None if not isinstance(envelope, dict) else envelope.get("valid_leg_count")
    requested = (
        None if not isinstance(envelope, dict)
        else envelope.get("requested_completed_samples")
    )
    if (
        document.get("status") != "PICK_PLACE_ONCE_RESIDENT_COMPLETE"
        or not isinstance(maximum, list)
        or len(maximum) != 6
        or not all(isinstance(value, int) and value >= 0 for value in maximum)
        or not isinstance(valid_count, int)
        or valid_count <= 0
        or not isinstance(requested, int)
        or requested <= 0
    ):
        raise ValueError("complete H2 tracking envelope evidence is required")
    return {
        "path": str(path),
        "sha256": actual,
        "maximum_error_raw": maximum,
        "valid_leg_count": valid_count,
        "requested_completed_samples": requested,
        "plan_sha256": document.get("plan_sha256"),
    }


def build_phase_specs(
    pick: EndpointPlan, lift: EndpointPlan, place: EndpointPlan
) -> list[PhaseSpec]:
    pick_pregrasp = pick.targets["pregrasp"]
    pick_grasp = pick.targets["grasp"]
    lift20 = lift.targets["grasp"]
    place_pregrasp = place.targets["pregrasp"]
    place_grasp = place.targets["grasp"]
    return [
        PhaseSpec("q0_to_pick_pregrasp", Q0, pick.path, "pregrasp"),
        PhaseSpec("pick_pregrasp_to_grasp", pick_pregrasp, pick.path, "grasp"),
        PhaseSpec("pick_grasp_to_lift20", pick_grasp, lift.path, "grasp"),
        PhaseSpec("lift_to_place_pregrasp", lift20, place.path, "pregrasp"),
        PhaseSpec("place_pregrasp_to_place", place_pregrasp, place.path, "grasp"),
        PhaseSpec("place_to_retreat", place_grasp, place.path, "pregrasp"),
        # The existing assembler reverses this collision-checked phase for q0 return.
        PhaseSpec("q0_to_place_pregrasp", Q0, place.path, "pregrasp"),
    ]


def run_checked(command: list[str], label: str) -> None:
    completed = subprocess.run(
        command, cwd=str(ROOT), capture_output=True, text=True
    )
    output = completed.stdout + completed.stderr
    if output.strip():
        print(output.rstrip())
    if completed.returncode != 0:
        raise RuntimeError(f"{label} failed with exit {completed.returncode}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("pick", "lift", "place"):
        parser.add_argument(f"--{name}-plan", type=Path, required=True)
        parser.add_argument(f"--{name}-sha256", required=True)
    parser.add_argument("--h2-evidence", type=Path, required=True)
    parser.add_argument("--h2-evidence-sha256", required=True)
    parser.add_argument(
        "--calibration", type=Path, default=ROOT / "config" / "single_arm_calibration.json"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest-output", type=Path, required=True)
    parser.add_argument("--report-output", type=Path, required=True)
    parser.add_argument("--plan-only", action="store_true")
    arguments = parser.parse_args()
    if not arguments.plan_only:
        parser.error("--plan-only is required; no MoveIt request was sent")
    for field in ("pick_sha256", "lift_sha256", "place_sha256", "h2_evidence_sha256"):
        value = getattr(arguments, field)
        if len(value) != 64 or any(c not in "0123456789abcdefABCDEF" for c in value):
            parser.error(f"--{field.replace('_', '-')} must be a 64-hex SHA256")
    return arguments


def main() -> int:
    arguments = parse_args()
    try:
        pick = load_pinned_endpoint(arguments.pick_plan, arguments.pick_sha256)
        lift = load_pinned_endpoint(arguments.lift_plan, arguments.lift_sha256)
        place = load_pinned_endpoint(arguments.place_plan, arguments.place_sha256)
        envelope = load_tracking_envelope(
            arguments.h2_evidence, arguments.h2_evidence_sha256
        )
        expected_route_sha256 = {
            "pick": pick.sha256,
            "pick_lift": lift.sha256,
            "place": place.sha256,
        }
        envelope_route_matches = envelope.get("plan_sha256") == expected_route_sha256
        calibration_sha256 = sha256_file(arguments.calibration)
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        phase_paths: dict[str, Path] = {}
        for phase in build_phase_specs(pick, lift, place):
            output = arguments.output_dir / f"{phase.name}.json"
            run_checked(
                [
                    sys.executable,
                    str(PLANNER),
                    "--plan-only",
                    "--source-plan", str(phase.source_plan),
                    "--target-name", phase.target_name,
                    "--calibration", str(arguments.calibration),
                    "--start=" + ",".join(f"{value:.12f}" for value in phase.start),
                    "--max-joint-step", str(STAGE7_MAX_JOINT_STEP_RAD),
                    "--execution-step-limit", str(STAGE7_MAX_JOINT_STEP_RAD),
                    "--output", str(output),
                ],
                phase.name,
            )
            phase_paths[phase.name] = output

        place_target = place.cartesian_targets["grasp"]
        run_checked(
            [
                sys.executable,
                str(ASSEMBLER),
                "--plan-only",
                "--calibration", str(arguments.calibration),
                "--q0-to-pick-pregrasp", str(phase_paths["q0_to_pick_pregrasp"]),
                "--pick-pregrasp-to-grasp", str(phase_paths["pick_pregrasp_to_grasp"]),
                "--pick-grasp-to-lift20", str(phase_paths["pick_grasp_to_lift20"]),
                "--lift-to-place-pregrasp", str(phase_paths["lift_to_place_pregrasp"]),
                "--place-pregrasp-to-place", str(phase_paths["place_pregrasp_to_place"]),
                "--place-to-retreat", str(phase_paths["place_to_retreat"]),
                "--q0-to-place-pregrasp", str(phase_paths["q0_to_place_pregrasp"]),
                "--place-x", str(place_target[0]),
                "--place-y", str(place_target[1]),
                "--place-z", str(place_target[2]),
                "--place-yaw", str(place.yaw_rad["grasp"]),
                "--output", str(arguments.manifest_output),
            ],
            "full-cycle manifest",
        )
        phase_records = [
            {"name": name, "path": str(path), "sha256": sha256_file(path)}
            for name, path in phase_paths.items()
        ]
        report = {
            "schema_version": 1,
            "status": STATUS,
            "execution_api_used": False,
            "motion_authorized": False,
            "automatic_execution_permitted": False,
            "nominal_moveit_collision_check_passed": True,
            "maximum_joint_step_rad": STAGE7_MAX_JOINT_STEP_RAD,
            "tracking_envelope_collision_checked": False,
            "tracking_envelope_route_matches_inputs": envelope_route_matches,
            "environment_collision_geometry_verified": False,
            "input_plans": [
                {"role": role, "path": str(plan.path), "sha256": plan.sha256}
                for role, plan in (("pick", pick), ("lift20", lift), ("place", place))
            ],
            "calibration": {
                "path": str(arguments.calibration), "sha256": calibration_sha256
            },
            "tracking_envelope": envelope,
            "phases": phase_records,
            "manifest": {
                "path": str(arguments.manifest_output),
                "sha256": sha256_file(arguments.manifest_output),
            },
            "routes": [
                {
                    "name": "A",
                    "phase_directions": [
                        {"phase": "q0_to_pick_pregrasp", "reversed": False},
                        {"phase": "pick_pregrasp_to_grasp", "reversed": False},
                    ],
                    "terminal": "pick_close",
                },
                {
                    "name": "B",
                    "phase_directions": [
                        {"phase": "pick_grasp_to_lift20", "reversed": False},
                        {"phase": "lift_to_place_pregrasp", "reversed": False},
                        {"phase": "place_pregrasp_to_place", "reversed": False},
                    ],
                    "terminal": "place_release",
                },
                {
                    "name": "C",
                    "phase_directions": [
                        {"phase": "place_to_retreat", "reversed": False},
                        {"phase": "q0_to_place_pregrasp", "reversed": True},
                    ],
                    "terminal": None,
                },
            ],
            "next_required_step": (
                "collect a route-matched offset011 H1 tracking envelope; then load "
                "verified environment geometry and collision-check the nominal route "
                "under that measured joint tracking-error envelope"
            ),
        }
        arguments.report_output.parent.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
        arguments.report_output.write_text(encoded, encoding="utf-8")
        print(f"STATUS={STATUS}")
        print(f"MANIFEST={arguments.manifest_output} sha256={report['manifest']['sha256']}")
        print(f"REPORT={arguments.report_output} sha256={sha256(encoded.encode()).hexdigest()}")
        print("MOTION_AUTHORIZED=false")
        print(f"TRACKING_ENVELOPE_ROUTE_MATCH={str(envelope_route_matches).lower()}")
        print("NEXT=route-matched H1 envelope, then robust environment collision check")
        return 0
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"H2_NOMINAL_ROUTES_PLAN_ONLY_FAIL reason={error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
