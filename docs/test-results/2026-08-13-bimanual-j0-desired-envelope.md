# Bimanual J0-D desired-envelope observation

- Date: 2026-08-13
- Firmware restored for observation: R4 `0x00023B00`
- Bridge mode: `BIMANUAL_READ_ONLY`
- Torque: verified off on both servo buses at bridge startup
- Motion source: operator manual movement only
- Automatic commands: none
- Status: **J0-D COMPLETE; J0-M AND MOTION AUTHORIZATION PENDING**

The operator confirmed that every movement in this series was free of cable
tension, twisting, connector pull and mount interference. These observations
therefore establish cable-safe desired motion, but they are not hard-stop or
mechanical-maximum measurements and do not automatically expand calibration.

## Desired arm-joint envelopes

| Joint | Left evidence | Right evidence | Assessment |
| --- | --- | --- | --- |
| BASE | raw `983..3041`, span 2058 | raw `1108..2996`, span 1888 | independent desired ranges; no wrap |
| SHOULDER | unwrapped `1899..4187`, wraps 4 | unwrapped `1859..4188`, wraps 8 | both arms require wrap-aware coordinates |
| ELBOW | raw `286..2492`, span 2206 | combined raw `297..2523` | near-matched desired ranges; no wrap |
| WRIST_FLEX | raw `170..2384`, span 2214 | raw `377..2438`, span 2061 | independent desired ranges; no wrap |
| WRIST_ROLL | raw `587..2838`, span 2251 | raw `749..2970`, span 2221 | independent cable-safe desired ranges; no wrap |

The right ELBOW combined interval uses the dedicated sweep `297..2517` and
the coordinated SHOULDER sweep high endpoint `2523`. Large same-arm spans in
some runs are retained as `coordinated_task_envelope` evidence rather than
mislabelled as isolated mechanical measurements.

## Gripper observations

| Item | Left | Right |
| --- | ---: | ---: |
| manual desired sweep | `1872..3255` | `1941..3299` |
| operator-confirmed unloaded closed | `1891` | `1907` |
| operator-confirmed task-open | `3257` | `3062` |
| stationary checkpoint span | `0` | `0` |

Raw increased monotonically in the opening direction throughout both manual
sweeps; the operator observed no reversal or over-center behavior. Task-open
means sufficient for the largest object expected in operation, not a
mechanical maximum. No further hard-stop search is required.

The existing left-arm loaded Pick/Place evidence remains semantically
important: raw `1963` / project `0.13 rad` was the close command and raw `2009`
/ project `0.06 rad` was release. This conflicts with current SRDF named-state
labels and must be resolved through the gripper mapping layer rather than by
copying raw extrema into URDF angle limits.

## Evidence files

All paths below are relative to `/home/pi/SO101-Bimanual-Manipulation`.

| Observation | Artifact | SHA-256 |
| --- | --- | --- |
| right BASE | `artifacts/joint_ranges/2026-08-13/right_base_q0_run01.json` | `592ec1a9b546171035494d0ad63a21690dce3895c8ad7b14be9dded7b3316630` |
| right SHOULDER | `artifacts/joint_ranges/2026-08-13/right_shoulder_desired_run02.json` | `92f6cb03ff7b0a7c87d1d275c2ff2179cf5d8b1f6884d924d2ccfd1da89f7589` |
| right ELBOW | `artifacts/joint_ranges/2026-08-13/right_elbow_desired_run01.json` | `bc19bf6988f4e7b095612c17ed272a3ad766ad4558dfaf893844931596f31e3e` |
| right WRIST_FLEX | `artifacts/joint_ranges/2026-08-13/right_wrist_flex_desired_run01.json` | `5c4722e39fe5a6bda64fb80103ce04da0a7722cc999af8711bb8a439e6cadab8` |
| right WRIST_ROLL | `artifacts/joint_ranges/2026-08-13/right_wrist_roll_desired_run01.json` | `3106c5980b79ec676e3dc312e6d7acd5d1ebb724e5c2f731952b1c6c00a47840` |
| right GRIPPER sweep | `artifacts/joint_ranges/2026-08-13/right_gripper_desired_run01.json` | `9fe63d03b805f52bb714df9fd129ad3864ddc6c9441c05529f8d2e67447606d2` |
| right GRIPPER closed | `artifacts/joint_ranges/2026-08-13/right_gripper_closed_checkpoint_run01.json` | `bb267e8fb33839be41163a0a236f75631568be5565260721747309591ed379a1` |
| right GRIPPER task-open | `artifacts/joint_ranges/2026-08-13/right_gripper_maximum_open_checkpoint_run01.json` | `0ceeb5b557c2d855ce4e83b9d11bbe10918cd8574ea844719958f9b2d9662021` |
| left BASE | `artifacts/joint_ranges/2026-08-13/left_base_desired_run01.json` | `fb8e522c30cf30a338592b8a1aa0434449642a1e5d4a0cd5f22949df39ec923f` |
| left SHOULDER | `artifacts/joint_ranges/2026-08-13/left_shoulder_desired_run01.json` | `8f941d39e720db4fe871a2152f691aa1cfc578e1f1e1e561f1e2558da7367efc` |
| left ELBOW | `artifacts/joint_ranges/2026-08-13/left_elbow_desired_run01.json` | `e776ef6292fe3d643366520a3585a9669455eb43d6606a8421d62d1a8e720632` |
| left WRIST_FLEX | `artifacts/joint_ranges/2026-08-13/left_wrist_flex_desired_run01.json` | `1d61468e9f15710f67b6ec12010c983b462d9054421118d4dcdc51a19e1e42f9` |
| left WRIST_ROLL | `artifacts/joint_ranges/2026-08-13/left_wrist_roll_desired_run01.json` | `6a17c6969d7225ec84a884a0786cdd250c1ca842f3aff5852beda7944acdbfff` |
| left GRIPPER sweep/closed plateau | `artifacts/joint_ranges/2026-08-13/left_gripper_desired_run01.json` | `d67c4fd9bb32611b8cb7afd2db0dec87e88ca283283b4d86571939c170bcb233` |
| left GRIPPER task-open | `artifacts/joint_ranges/2026-08-13/left_gripper_task_open_checkpoint_run01.json` | `b0fb7ad65acbc7299c9b9e9aade2b48ac3a70c945e48cbc70da86d87424be07d` |

## Gate decision

- J0-D: **PASS** for both arms and all 12 axes.
- Cable-safe desired movement: **operator-confirmed PASS**.
- J0-M physical outer envelope: **not measured**.
- Calibration/URDF/MoveIt/Isaac limit expansion: **not authorized**.
- Non-zero protocol-v2 output: **not authorized**.
- Next software gate: J1-W wrap-aware feedback and command coordinates in
  validation/shadow paths only.
- Before J2 active motion: prove that the final operational interval has an
  outer physical margin, or explicitly contract the required task interval.
