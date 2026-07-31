# Stage 7 full Pick/Place plan-only chain

- Date: 2026-07-31
- Status: FULL PICK/PLACE PLAN-ONLY PASS; PHYSICAL PLACE NOT AUTHORIZED
- Robot state: bridge stopped, 12 V OFF
- MoveIt backend: mock, ROS domain 93

## Targets

The Pick target reuses the validated Top-to-base candidate:

```text
Pick object center = (0.371814352, -0.129674332, 0.006300000) m
Yaw                = -0.034597181 rad
```

The first Place candidate moves the object 60 mm in base `+Y` while preserving
height and yaw:

```text
Place object center = (0.371814352, -0.069674332, 0.006300000) m
Yaw                 = -0.034597181 rad
```

The Place center is inside the validated board rectangle and conservative
workspace. Its base radial distance is approximately `0.3783 m`; grasp and
pregrasp TCP heights are `0.0313 m` and `0.1063 m`.

## Plan-only result

MoveIt returned successful position-only plans for Pick pregrasp/grasp,
20 mm lift, and Place pregrasp/place. Every transition was replanned from an
explicit joint start state with a maximum per-joint step of `0.18 rad`.

| Phase | Arm segments | Result |
| --- | ---: | --- |
| q0 to Pick pregrasp | 10 | PASS |
| Pick pregrasp to grasp | 2 | PASS |
| Pick grasp to 20 mm lift | 1 | PASS |
| Lift to Place pregrasp | 2 | PASS |
| Place pregrasp to Place | 2 | PASS |
| Place to retreat | 2 | PASS |
| Place pregrasp to q0, reversed collision-free chain | 10 | PASS |

The assembled result contains 29 arm segments plus gripper close/open, for 31
command steps. It finishes at all-zero arm q0. The gripper targets are the
physically validated close/open positions `0.13 / 0.06 rad`.

## Fail-closed manifest

`tools/assemble_pick_place_plan_only.py` independently checks:

- source status and SHA-256;
- `execution_api_used=false`;
- `motion_authorized=false`;
- `robot_target_available=false`;
- exact arm joint order;
- successful, contiguous segment indices;
- recorded versus calculated joint delta;
- the `0.18 rad` Stage 7 step bound;
- calibration limits and hash `0x8AD27897`;
- Place board, Cartesian workspace, radial, grasp-z, and pregrasp-z gates;
- complete phase continuity and final q0.

The manifest explicitly sets `automatic_execution_permitted=false`. The first
arm segment of every phase and both gripper actions require a manual gate.
The assembler imports no ROS execution Action.

Final manifest:

```text
artifacts/stage7/2026-07-31/full_pick_place/full_pick_place_plan_only_manifest.json
SHA-256 b293149848c74ef62df7db193fab8e8e54030254f67a29e8853b75a8a494007a
```

## Verification

- Pick pose plan-only: PASS, 184/214 trajectory points
- Place pose plan-only: PASS, 171/203 trajectory points
- 20 mm lift pose plan-only: PASS
- All seven bounded phase plans: PASS
- Assembler focused tests: 6/6 PASS
- Repository Python suite: 305/305 PASS
- `git diff --check`: PASS
- Robot motion during this work: 0

## Required next gates

1. Physically mark and clear the proposed Place center.
2. Capture a fresh Pick observation and re-run transform, freshness,
   confidence, visibility, and workspace gates. This saved candidate is not a
   live robot target.
3. Build a manifest-hash-pinned supervisor that checks fresh start state and
   diagnostics at every manual phase boundary and never retries.
4. Perform the first physical Place run phase by phase with explicit
   approvals. Do not enable unattended execution.
5. Only after the complete path passes may the 50-trial Pick/Place benchmark
   begin.
