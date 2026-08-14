# Bimanual J1-W unwrapped-shadow candidate

- Date: 2026-08-13
- Candidate firmware: `0x00024000`
- Capabilities: `0x0F802000`
- Protocol: v2, 12 joints
- Motion authorization: **false**
- Servo goal output: **not connected**
- Hardware status: **PASS**

## Purpose

J0-D proved that both SHOULDER encoders cross raw `4095↔0` during desired
task motion. A linear `raw - zero_raw` mapping would therefore report a false
near-one-turn jump and reject a physically continuous pose. J1-W introduces an
explicitly bound, stateful unwrapped coordinate before any active-motion gate.

## Contract

- A SHA-bound 12-axis commissioning reference selects the initial encoder
  branch; raw feedback alone cannot authorize or guess a branch.
- Initial bind requires a unique nearest branch within a reviewed
  `1..2047 raw` window.
- Every later sample uses signed modular delta. An exactly half-turn sample,
  bad raw, invalid direction, reversed limit, or overflow fails closed.
- Feedback and joint-limit validation use unwrapped raw/radian coordinates.
- Command conversion validates the unwrapped limit first and only then emits
  modulo `0..4095` raw.
- The candidate verifies torque-off on both buses before every real-feedback
  snapshot. Executor output is discarded and no goal-position path is called.

The first `PREPARE_SHADOW` payload is `<HH12i>`: maximum reference delta,
reserved zero, and 12 explicit unwrapped raw references. A later empty
`PREPARE_SHADOW` updates all 12 persistent unwrap states from fresh feedback.
Candidate states are committed only after every axis converts successfully.

## Local evidence

- Targeted Python tests: `42 passed`
- Full Python suite with ROS Jazzy sourced: `1258 passed`
- C actuator-core executables: `6/6 passed`
- J1-W Cortex-M4 Release build: pass
- J1-W HEX:
  `build/stm32_g474_single_arm-j1w-shadow-release/stm32_g474_single_arm.hex`
- J1-W SHA-256:
  `7ddcfec7bf4d9e06200dd39ebff9939afda84d1934feab3172f937d2c95cc2c7`
- R4 regression SHA-256 remained:
  `b677bd97a7097ae8bb3be07ad5d5696dee6df803f33aa4ffe89282ad792acd8a`
- `git diff --check`: pass

The unit replay includes forward and reverse `4095↔0` crossings, ambiguous
half-turn rejection, explicit branch-window rejection, unwrapped limit
rejection, and modulo command conversion.

## Hardware evidence

1. R4 stationary reference: all 12 axes had span `0 raw`; left/right SHOULDER
   were `2390/2396`, away from an ambiguous branch. Artifact
   `artifacts/joint_ranges/2026-08-13/j1w_branch_reference_r4_run01.json`,
   SHA-256
   `018ce90e6e869958175750f829050deb5b68dfdf47f5f4df24fc9f05eee20985`.
2. J1-W explicit bind and three stationary stateful updates matched the
   independent host calculation exactly. The discarded-output executor
   applied `3/3` samples with `0 ms` maximum lateness, rejected-frame delta 0,
   and no stop latch. Artifact
   `artifacts/protocol_v2/2026-08-13/j1w_stationary_run01.json`, SHA-256
   `52927769af946770c1f02a33721e855987c62baf3533322a6895ea1c47ae428a`.
3. The first wrap-observer attempt intentionally failed closed because the
   arm had moved after reference capture: left ELBOW was `514 raw` from the
   strict `32 raw` stationary window. Both present masks remained `0x3F`.
   After reset, the observer used a `1024 raw` branch-selection window, still
   below the ambiguous half-turn boundary, without enabling torque or output.
4. The output-disconnected observer collected 754 samples. Left SHOULDER
   crossed twice, unwrapped `2375..4206`, with maximum sample step `93 raw`.
   Right SHOULDER crossed twice, unwrapped `2525..4232`, with maximum sample
   step `111 raw`. Firmware and independent host unwrappers matched on every
   sample. Artifact
   `artifacts/joint_ranges/2026-08-13/j1w_both_shoulder_wraps_run02.json`,
   SHA-256
   `249c2616a28748003a980e058b312813304594cc57310bf0bf64fdff86a6d397`.
5. The known R4 `0x00023B00` image was restored. Its 12-axis read-only soak
   passed 100/100 bimanual samples and 249 legacy-left samples. Artifact
   `artifacts/right_arm/2026-08-13/r4_after_j1w_soak_100.json`, SHA-256
   `86a6eb256a43ac1ca20a623f8f7a5b123eb4dc3bf4806d74da4c30c71a4a55c5`.

## Gate decision

- J1-W explicit branch binding: **PASS**.
- Stateful forward/reverse `4095↔0` tracking on both SHOULDER axes: **PASS**.
- R4 post-candidate regression: **PASS**.
- Hardware bus/cable fault evidence in this gate: **none**.
- Motion authorization remains **false**; the test never connected a servo
  goal output.

The next gate is J1 operational-limit and firmware/host/model parity. J2
bounded active motion and all non-zero protocol-v2 output remain unauthorized
until that parity and the J0-M/operational-margin decision pass.
