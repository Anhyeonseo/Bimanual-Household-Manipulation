#!/usr/bin/env python3
"""Calibrate Newton VBD towel bending against the measured home tests.

This diagnostic reproduces the 36.33 mm / 45 degree table-edge cantilever
and the 100.1 mm two-corner edge release.  It is motion-free: no robot or
hardware API is imported.  The direct Newton robot-cloth scene runs in
centimetres, while the IsaacLab-coupled scene runs in metres.  Bending
coefficients are scale dependent, so this diagnostic can reproduce either
world-unit convention and must match the convention used by its consumer.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import warp as wp

import newton
from newton import ParticleFlags


CLOTH_SIDE_M = 0.30
CLOTH_MASS_KG = 0.05667
INITIAL_CLEARANCE_M = 0.0030
CANTILEVER_OVERHANG_M = 0.0363333333333
CANTILEVER_TARGET_ANGLE_DEG = 45.0
CANTILEVER_TOLERANCE_DEG = 5.0
EDGE_RELEASE_LIFT_M = 0.1001
EDGE_RELEASE_TARGET_S = 0.1805555556
VIDEO_FPS = 24.0
VIDEO_TIMING_RESOLUTION_S = 1.0 / VIDEO_FPS


@wp.kernel
def set_kinematic_particles(
    indices: wp.array[wp.int32],
    origins: wp.array[wp.vec3],
    lift_world: float,
    position_0: wp.array[wp.vec3],
    position_1: wp.array[wp.vec3],
    velocity_0: wp.array[wp.vec3],
    velocity_1: wp.array[wp.vec3],
):
    tid = wp.tid()
    particle_index = indices[tid]
    target = origins[tid] + wp.vec3(0.0, 0.0, lift_world)
    position_0[particle_index] = target
    position_1[particle_index] = target
    velocity_0[particle_index] = wp.vec3(0.0)
    velocity_1[particle_index] = wp.vec3(0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", choices=("cantilever", "edge-release"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--resolution", type=int, default=31)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--substeps", type=int, default=4)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument(
        "--world-units-per-meter",
        type=float,
        choices=(1.0, 100.0),
        default=100.0,
        help="1 for IsaacLab metre scenes; 100 for the direct centimetre scene",
    )
    parser.add_argument("--edge-stiffness", type=float, required=True)
    parser.add_argument("--edge-damping", type=float, required=True)
    parser.add_argument("--triangle-stiffness", type=float, default=1.0e4)
    parser.add_argument("--triangle-damping", type=float, default=1.5e-6)
    parser.add_argument("--contact-stiffness", type=float, default=1.0e4)
    parser.add_argument("--timeout-seconds", type=float, default=12.0)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if args.resolution < 4:
        parser.error("--resolution must be at least 4")
    for name in (
        "fps",
        "edge_stiffness",
        "triangle_stiffness",
        "contact_stiffness",
        "timeout_seconds",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("edge_damping", "triangle_damping"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    if args.substeps <= 0 or args.iterations <= 0:
        parser.error("--substeps and --iterations must be positive")
    return args


def selected_corner_patch_indices(resolution: int) -> np.ndarray:
    """Return the two 3x3 patches on the grid's y=0 edge."""

    side = resolution + 1
    result = []
    for y in range(3):
        for x in range(3):
            result.append(y * side + x)
        for x in range(resolution - 2, resolution + 1):
            result.append(y * side + x)
    return np.asarray(sorted(set(result)), dtype=np.int32)


def build_model(args: argparse.Namespace):
    units_per_m = args.world_units_per_meter
    cloth_side = CLOTH_SIDE_M * units_per_m
    cell_world = cloth_side / args.resolution
    particle_count = (args.resolution + 1) ** 2
    particle_mass_kg = CLOTH_MASS_KG / particle_count
    builder = newton.ModelBuilder(gravity=-9.81 * units_per_m)
    table_cfg = builder.default_shape_cfg.copy()
    table_cfg.ke = args.contact_stiffness
    table_cfg.kd = 1.0e-2
    table_cfg.mu = 0.74

    if args.experiment == "cantilever":
        # The static box ends exactly at x=0; the measured 36.33 mm hangs over it.
        builder.add_shape_box(
            -1,
            wp.transform(
                wp.vec3(-0.20 * units_per_m, 0.0, -0.05 * units_per_m),
                wp.quat_identity(),
            ),
            hx=0.20 * units_per_m,
            hy=0.25 * units_per_m,
            hz=0.05 * units_per_m,
            cfg=table_cfg,
        )
        cloth_x = -(cloth_side - CANTILEVER_OVERHANG_M * units_per_m)
    else:
        builder.add_shape_box(
            -1,
            wp.transform(
                wp.vec3(0.0, 0.0, -0.05 * units_per_m), wp.quat_identity()
            ),
            hx=0.20 * units_per_m,
            hy=0.20 * units_per_m,
            hz=0.05 * units_per_m,
            cfg=table_cfg,
        )
        cloth_x = -0.5 * cloth_side

    builder.add_cloth_grid(
        pos=wp.vec3(
            cloth_x,
            -0.5 * cloth_side,
            INITIAL_CLEARANCE_M * units_per_m,
        ),
        rot=wp.quat_identity(),
        vel=wp.vec3(0.0),
        dim_x=args.resolution,
        dim_y=args.resolution,
        cell_x=cell_world,
        cell_y=cell_world,
        mass=particle_mass_kg,
        tri_ke=args.triangle_stiffness,
        tri_ka=args.triangle_stiffness,
        tri_kd=args.triangle_damping,
        edge_ke=args.edge_stiffness,
        edge_kd=args.edge_damping,
        particle_radius=0.0015 * units_per_m,
    )
    builder.color(include_bending=True)
    model = builder.finalize(device=args.device)
    model.soft_contact_ke = args.contact_stiffness
    model.soft_contact_kd = 1.0e-2
    model.soft_contact_mu = 0.74
    return model


def make_runtime(model, args: argparse.Namespace):
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    units_per_m = args.world_units_per_meter
    collision_pipeline = newton.CollisionPipeline(
        model, soft_contact_margin=0.0015 * units_per_m
    )
    contacts = collision_pipeline.contacts()
    solver = newton.solvers.SolverVBD(
        model,
        iterations=args.iterations,
        particle_enable_self_contact=args.experiment == "edge-release",
        particle_self_contact_radius=0.0015 * units_per_m,
        particle_self_contact_margin=0.0025 * units_per_m,
        particle_topological_contact_filter_threshold=2,
        particle_rest_shape_contact_exclusion_radius=0.0,
        particle_collision_detection_interval=1,
    )
    sim_dt = 1.0 / args.fps / args.substeps
    return state_0, state_1, control, collision_pipeline, contacts, solver, sim_dt


def simulate_step(runtime, model, sim_dt: float) -> None:
    state_0, state_1, control, collision_pipeline, contacts, solver = runtime
    state_0.clear_forces()
    collision_pipeline.collide(state_0, contacts)
    solver.step(state_0, state_1, control, contacts, sim_dt)


def run_cantilever(model, args: argparse.Namespace, runtime_tuple, sim_dt: float):
    state_0, state_1, control, collision_pipeline, contacts, solver = runtime_tuple
    runtime = [state_0, state_1, control, collision_pipeline, contacts, solver]
    units_per_m = args.world_units_per_meter
    initial = state_0.particle_q.numpy().copy()
    side = args.resolution + 1
    free_edge_indices = np.arange(args.resolution, side * side, side, dtype=np.int32)
    supported_indices = np.flatnonzero(
        initial[:, 0] <= -0.02 * units_per_m
    ).astype(np.int32)
    video_steps = max(1, round((1.0 / VIDEO_FPS) / sim_dt))
    maximum_steps = math.ceil(args.timeout_seconds / sim_dt)
    previous = initial.copy()
    previous_angle = None
    motion_observed = False
    quiet_run = 0
    settled_step = None
    frame_samples = []
    final_speed_world_s = math.inf

    for step in range(1, maximum_steps + 1):
        simulate_step(runtime, model, sim_dt)
        runtime[0], runtime[1] = runtime[1], runtime[0]
        state_0, state_1 = runtime[0], runtime[1]
        if step % video_steps:
            continue
        current = state_0.particle_q.numpy()
        velocities = state_0.particle_qd.numpy()
        displacement_world = np.linalg.norm(current - previous, axis=-1)
        p95_world = float(np.quantile(displacement_world, 0.95))
        free_edge = np.mean(current[free_edge_indices], axis=0)
        horizontal_world = max(float(free_edge[0]), 1.0e-9)
        vertical_drop_world = max(
            INITIAL_CLEARANCE_M * units_per_m - float(free_edge[2]), 0.0
        )
        angle_deg = math.degrees(math.atan2(vertical_drop_world, horizontal_world))
        angle_delta_deg = math.inf if previous_angle is None else abs(angle_deg - previous_angle)
        motion_observed = motion_observed or p95_world >= 0.0010 * units_per_m
        quiet = (
            motion_observed
            and p95_world <= 0.0010 * units_per_m
            and angle_delta_deg <= 0.5
        )
        quiet_run = quiet_run + 1 if quiet else 0
        final_speed_world_s = float(np.max(np.linalg.norm(velocities, axis=-1)))
        frame_samples.append(
            {
                "time_s": step * sim_dt,
                "p95_frame_displacement_m": p95_world / units_per_m,
                "chord_angle_deg": angle_deg,
                "angle_delta_deg": None if not math.isfinite(angle_delta_deg) else angle_delta_deg,
            }
        )
        previous = current.copy()
        previous_angle = angle_deg
        if quiet_run >= 5:
            settled_step = step
            break

    final = state_0.particle_q.numpy()
    free_edge = np.mean(final[free_edge_indices], axis=0)
    horizontal_world = max(float(free_edge[0]), 1.0e-9)
    vertical_drop_world = max(
        INITIAL_CLEARANCE_M * units_per_m - float(free_edge[2]), 0.0
    )
    angle_deg = math.degrees(math.atan2(vertical_drop_world, horizontal_world))
    supported_slip_world = float(
        np.max(np.linalg.norm(final[supported_indices, :2] - initial[supported_indices, :2], axis=-1))
    )
    matched = (
        settled_step is not None
        and supported_slip_world <= 0.0030 * units_per_m
        and abs(angle_deg - CANTILEVER_TARGET_ANGLE_DEG) <= CANTILEVER_TOLERANCE_DEG
    )
    return {
        "matched": matched,
        "target_overhang_m": CANTILEVER_OVERHANG_M,
        "target_angle_deg": CANTILEVER_TARGET_ANGLE_DEG,
        "angle_tolerance_deg": CANTILEVER_TOLERANCE_DEG,
        "final_chord_angle_deg": angle_deg,
        "maximum_supported_slip_m": supported_slip_world / units_per_m,
        "shape_settle_time_s": None if settled_step is None else settled_step * sim_dt,
        "final_maximum_speed_m_s_diagnostic_only": final_speed_world_s / units_per_m,
        "frame_samples": frame_samples,
    }


def run_edge_release(model, args: argparse.Namespace, runtime_tuple, sim_dt: float):
    state_0, state_1, control, collision_pipeline, contacts, solver = runtime_tuple
    runtime = [state_0, state_1, control, collision_pipeline, contacts, solver]
    units_per_m = args.world_units_per_meter
    selected_np = selected_corner_patch_indices(args.resolution)
    selected = wp.array(selected_np, dtype=wp.int32, device=args.device)

    # Let the initially-flat towel establish contact before picking up its edge.
    pre_steps = round(2.0 / sim_dt)
    for _ in range(pre_steps):
        simulate_step(runtime, model, sim_dt)
        runtime[0], runtime[1] = runtime[1], runtime[0]

    state_0, state_1 = runtime[0], runtime[1]
    origins_np = state_0.particle_q.numpy()[selected_np].copy()
    origins = wp.array(origins_np, dtype=wp.vec3, device=args.device)
    flags = model.particle_flags.numpy()
    flags[selected_np] &= ~int(ParticleFlags.ACTIVE)
    model.particle_flags = wp.array(flags, dtype=model.particle_flags.dtype, device=args.device)

    lift_steps = round(3.0 / sim_dt)
    for step in range(1, lift_steps + 1):
        alpha = step / lift_steps
        wp.launch(
            set_kinematic_particles,
            dim=len(selected_np),
            inputs=[
                selected,
                origins,
                alpha * EDGE_RELEASE_LIFT_M * units_per_m,
                runtime[0].particle_q,
                runtime[1].particle_q,
                runtime[0].particle_qd,
                runtime[1].particle_qd,
            ],
            device=args.device,
        )
        simulate_step(runtime, model, sim_dt)
        runtime[0], runtime[1] = runtime[1], runtime[0]

    # Hold one second at exactly the measured release pose, then activate the patches.
    hold_steps = round(1.0 / sim_dt)
    for _ in range(hold_steps):
        wp.launch(
            set_kinematic_particles,
            dim=len(selected_np),
            inputs=[
                selected,
                origins,
                EDGE_RELEASE_LIFT_M * units_per_m,
                runtime[0].particle_q,
                runtime[1].particle_q,
                runtime[0].particle_qd,
                runtime[1].particle_qd,
            ],
            device=args.device,
        )
        simulate_step(runtime, model, sim_dt)
        runtime[0], runtime[1] = runtime[1], runtime[0]

    flags = model.particle_flags.numpy()
    flags[selected_np] |= int(ParticleFlags.ACTIVE)
    model.particle_flags = wp.array(flags, dtype=model.particle_flags.dtype, device=args.device)
    runtime[0].particle_qd.zero_()
    runtime[1].particle_qd.zero_()

    video_steps = max(1, round((1.0 / VIDEO_FPS) / sim_dt))
    maximum_steps = math.ceil(2.0 / sim_dt)
    previous = runtime[0].particle_q.numpy().copy()
    quiet_run = 0
    motion_observed = False
    onset_s = None
    confirmation_s = None
    frame_samples = []
    for step in range(1, maximum_steps + 1):
        simulate_step(runtime, model, sim_dt)
        runtime[0], runtime[1] = runtime[1], runtime[0]
        if step % video_steps:
            continue
        current = runtime[0].particle_q.numpy()
        displacement_world = np.linalg.norm(current - previous, axis=-1)
        p95_world = float(np.quantile(displacement_world, 0.95))
        median_world = float(np.median(displacement_world))
        motion_observed = motion_observed or p95_world >= 0.0010 * units_per_m
        quiet_run = (
            quiet_run + 1
            if motion_observed and p95_world <= 0.0010 * units_per_m
            else 0
        )
        time_s = step * sim_dt
        frame_samples.append(
            {
                "time_s": time_s,
                "p95_frame_displacement_m": p95_world / units_per_m,
                "median_frame_displacement_m": median_world / units_per_m,
            }
        )
        if onset_s is None and quiet_run >= 2:
            onset_s = time_s - 1.0 / VIDEO_FPS
            confirmation_s = time_s
        previous = current.copy()

    final_velocities = runtime[0].particle_qd.numpy()
    target_window = (
        EDGE_RELEASE_TARGET_S - VIDEO_TIMING_RESOLUTION_S,
        EDGE_RELEASE_TARGET_S + VIDEO_TIMING_RESOLUTION_S,
    )
    simulated_window = (
        None
        if onset_s is None
        else (onset_s - VIDEO_TIMING_RESOLUTION_S, onset_s + VIDEO_TIMING_RESOLUTION_S)
    )
    matched = bool(
        motion_observed
        and simulated_window is not None
        and simulated_window[0] <= target_window[1]
        and target_window[0] <= simulated_window[1]
    )
    return {
        "matched": matched,
        "lift_height_m": EDGE_RELEASE_LIFT_M,
        "selected_node_count": int(len(selected_np)),
        "target_settle_time_s": EDGE_RELEASE_TARGET_S,
        "target_window_s": list(target_window),
        "simulated_settle_time_s": onset_s,
        "simulated_window_s": None if simulated_window is None else list(simulated_window),
        "confirmation_time_s": confirmation_s,
        "motion_observed": motion_observed,
        "final_maximum_speed_m_s_diagnostic_only": float(
            np.max(np.linalg.norm(final_velocities, axis=-1)) / units_per_m
        ),
        "frame_samples": frame_samples,
    }


def main() -> int:
    args = parse_args()
    wp.set_device(args.device)
    model = build_model(args)
    state_0, state_1, control, collision_pipeline, contacts, solver, sim_dt = make_runtime(model, args)
    runtime = (state_0, state_1, control, collision_pipeline, contacts, solver)
    if args.experiment == "cantilever":
        observation = run_cantilever(model, args, runtime, sim_dt)
    else:
        observation = run_edge_release(model, args, runtime, sim_dt)

    result = {
        "schema_version": 1,
        "record_kind": "towel_newton_material_calibration",
        "status": (
            "R2_NEWTON_MATERIAL_CALIBRATION_MATCH"
            if observation["matched"]
            else "R2_NEWTON_MATERIAL_CALIBRATION_RECORDED_NOT_MATCHED"
        ),
        "generated_at_unix_s": time.time(),
        "motion_authorized": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "backend": {
            "name": "newton_vbd",
            "device": args.device,
            "world_units_per_meter": args.world_units_per_meter,
            "length_unit": (
                "meter" if args.world_units_per_meter == 1.0 else "centimeter"
            ),
        },
        "solver": {
            "fps": args.fps,
            "substeps": args.substeps,
            "iterations": args.iterations,
            "sim_dt_s": sim_dt,
        },
        "experiment": args.experiment,
        "cloth": {
            "side_m": CLOTH_SIDE_M,
            "mass_kg": CLOTH_MASS_KG,
            "resolution": [args.resolution, args.resolution],
        },
        "material": {
            "edge_stiffness_newton_units": args.edge_stiffness,
            "edge_damping_newton_units": args.edge_damping,
            "triangle_stiffness_newton_units": args.triangle_stiffness,
            "triangle_damping_newton_units": args.triangle_damping,
            "table_friction": 0.74,
            "contact_stiffness_newton_units": args.contact_stiffness,
        },
        "observation": observation,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(result["status"] + " " + json.dumps(observation, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
