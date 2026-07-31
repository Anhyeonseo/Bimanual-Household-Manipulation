# Stage 7 supervised physical Pick and Place

Date: 2026-07-31
Result: **PASS — one supervised end-to-end cycle**

## Scope

This result closes the Stage 7 commissioning run, not the 50-cycle reliability
benchmark. Every physical transition was plan-only checked first, hash pinned,
fresh-state checked, explicitly approved once, and stopped on any failed gate.
Automatic retry and `CLEAR_FAULT` were not used.

## Accepted hardware and control identity

- firmware: `0x00021800`
- protocol: `1`
- calibration: `0x8AD27897`
- capabilities: `0x000003FF`
- Shoulder: P/D/I `32/32/0`, torque limit `780`
- Elbow: P/D/I `28/32/0`, torque limit `650`
- firmware HEX SHA-256:
  `4b9ca7c7b3927ce798048258fb1b3deecfb0718d660c6c1bd93308862ef3f317`
- guarded external executor SHA-256:
  `3d28e2f7856a91e792f98ec48fd87a5debda094a688001851e6c61bbd3d3ec7c`

Firmware `0x00021800` adds bounded servo-response stream
resynchronization, full UART recovery, and classified recovery diagnostics.
Before task motion it passed identity/capability, READ_ONLY six-axis physical
disable, MOTION_ENABLED no-motion readback, a five-minute heartbeat/feedback
run, injected single-sweep failure, and reset-free six-axis recovery.

## Execution safety contract

The external executor enforced the following before and after every arm
segment:

- exact protocol, calibration and six-servo identity;
- six-axis torque-enabled diagnostics;
- Shoulder temperature strictly below `50 C`;
- start/post-settle error `0.055 rad` for Shoulder and `0.050 rad` for the
  other arm axes;
- separate plan interpolation and fresh measured execution bounds;
- no resend after a soft abort; continue only after a two-second settle and a
  fresh state/diagnostic pass.

The bridge still implements the deliberately limited single-point Action
contract. For commissioning, long paths were therefore executed as several
independently settled and diagnosed Actions. This is a safety-validation
method, not the intended production-speed trajectory contract.

## Physical result

The final supervised cycle completed:

1. fresh grasp correction;
2. gripper close and verified object hold;
3. approximately 20 mm lift;
4. transfer to Place pregrasp;
5. Place descent plus two bounded 5 mm Z corrections;
6. confirmed object support and gripper release;
7. retreat to Place pregrasp;
8. collision-free return to q0;
9. warning-free Bridge shutdown, 12 V off, arm safe.

The final plans used for the accepted Place and return portion were:

| Phase | Plan SHA-256 | Segments |
|---|---|---:|
| Place pregrasp | `035a199a5bc74eee9d23bf6d33366c79ec6fe51b0a20a0d247466d3a4c1d0a9c` | 3 |
| Place grasp | `28140d35902df603c9ec8216ed276670df3e589af66bf651260ff1a046d94008` | 4 |
| First -5 mm Z correction | `c2c327655d9294fd5dfc0331da4b1eb9e01987e50d49224c0f61e65f95518e99` | 2 |
| Second -5 mm Z correction | `f3706fa902ac9d27e3c6768c683828a496bfd194fa3a5a258b838ba3a1b9fbb7` | 1 |
| Place retreat | `0aa5c127f398b44c19d3da1917d204f026ccb05a2e093b0186b8c3179aaaeea0` | 5 |
| q0 return | `2bb28414845dfaa7d09ef96163dfa59fcc935fa88d6d05c0c0447dd1f2657216` | 11 |

The q0 return completed 11/11 Actions. Final arm errors were approximately
`+0.004602, +0.004602, -0.004602, +0.001534, -0.007670 rad`; Shoulder
temperature was `36 C`. The user visually confirmed the Pick, lift, Place,
retreat, q0 pose and final safe shutdown.

## Repository verification

- ROS 2 symlink-install build: 8 packages passed;
- ROS-overlay Python regression: 336 passed;
- `git diff --check`: passed;
- generated plans, captures and local build/install outputs remain ignored.

## Acceptance boundary and next work

The commissioning sequence is complete. Formal Stage 7 task acceptance remains
`partial` because the roadmap requires 50 trials with at least 90% Pick and
Place success and zero unintended motion/collision. This single supervised run
also used operator-confirmed alignment and two manual Z corrections, so it is
not evidence of unattended perception-to-task autonomy.

Before the 50-cycle benchmark:

1. replace the stop-and-settle single-point chain with a separately designed
   multi-point/buffered trajectory contract. It must define timing, queue
   capacity, cancel/stop semantics, continuous diagnostics and path tracking
   before it is used on hardware. The public `FollowJointTrajectory` interface
   should remain unchanged;
2. replace the nominal Place TCP offset `0.025 m` with a measured
   TCP-to-contact contract. The accepted run required two additional bounded
   `-5 mm` corrections, indicating an initial candidate near `0.015 m`.
   Pick and Place offsets must be separated, plan-only/collision checked and
   physically validated once before the value is adopted.
