# Stage 7 Elbow P28 bounded candidate

- Date: 2026-07-31
- Status: ISOLATED AND LOADED FIRST-LIFT P28 GATES PASSED; FULL AUTONOMOUS PATH PENDING
- Physical robot state during work: bridge/MoveIt stopped, 12 V OFF, arm safe

## Evidence and decision

With firmware `0x00021500`, calibration `0x095CB9A5`, Shoulder P32, and
Elbow P24, the repeated P32 Stage 7 path produced the following settled
endpoint pattern:

| Segment class | Shoulder error (raw) | Elbow error (raw) |
| --- | ---: | ---: |
| pregrasp 1 through 4 | 2 to 4 | 17 to 19 |
| grasp approach 1 and 2 | 5 | 19 |
| first 20 mm lift segment | 26 | 16 |

The Elbow residual stayed in the same direction and remained below the
30-raw completion threshold, but it was consistently the largest residual
during the approach. Voltage stayed at 12.4 to 12.5 V and no servo protection
fault was reported. The bounded next experiment therefore changes only the
Elbow proportional gain from 24 to 28. Torque limits, D/I gains, safety
watchdogs, raw limits, and completion tolerance remain unchanged.

## Candidate identity

- Firmware version: `0x00021600`
- Calibration hash: `0x8AD27897`
- Base P16
- Shoulder P32
- Elbow P28
- Wrist Flex P16
- Wrist Roll P16
- Gripper P16
- Shoulder torque limit: 780 raw
- Elbow torque limit: 650 raw
- D gain: 32 for all axes
- I gain: 0 for all axes

## Local verification

- Calibration hash independently calculated from both authoritative JSON
  copies: `0x8AD27897`
- Host/firmware calibration table synchronization: PASS
- Firmware/host identity contract: PASS
- Repository Python suite: 213/213 PASS
- `single_arm_bridge` package: 21 tests, 0 errors, 0 failures
- STM32 Cortex-M4 hard-float Release build: PASS
- Firmware size: text 30228, data 112, bss 4160 bytes
- HEX SHA-256:
  `f84dad6cd40533916e9687f7b07faf112f47dc549cf0e9bce2dd68a17ee88e41`
- HEX-to-binary round-trip against the linked build output: PASS
- Host/firmware deployment archive SHA-256:
  `cc9d90213b5141ae274942bf0e895366f8b576723e40dff5ec8917183944552b`
- Pi deployment and `single_arm_bridge` rebuild: PASS
- Pre-flash 512 KiB rollback backup SHA-256:
  `0bdc5c2bbf9311612d28de28e1e53749f6368e900b1a1466d826914d646987a2`
- STM32 program/verify/reset: PASS
- Post-flash identity and heartbeat gate: PASS

Two preliminary CMake configuration invocations produced no firmware artifact:
one used a relative toolchain path from an external build directory, and one
requested unavailable Ninja. The final build used the absolute toolchain path
and installed `/usr/bin/make`.

## Isolated physical result

The no-motion readback confirmed the intended live configuration on all axes:
Shoulder P32/780, Elbow P28/650, D32, and I0. One approved two-second Elbow
move in the previously problematic ROS-positive/raw-decreasing direction
commanded raw 1550 from raw 1602.

- Terminal status: succeeded
- Firmware terminal detail: 13 raw
- Elbow goal/actual: 1550/1563 raw
- Elbow load/current: 100/5 raw
- Elbow voltage/temperature: 12.5 V/31 C
- Other-axis endpoint errors: 0 to 6 raw
- User physical observation: normal motion; no abnormal vibration or noise

The former P24 approach residual was 16 to 19 raw. The P28 isolated result is
therefore a 3 to 6 raw improvement without an observed stability penalty.
P28 is provisionally accepted; a loaded multiaxis lift remains the required
confirmation.

## Loaded first-lift result

After a stable contact grasp, one approved approximately 20 mm lift used the
new host commanded-setpoint preservation and held Wrist Flex fixed near its
lower raw limit. Firmware `0x00021600` completed the two-second multiaxis move
successfully:

- Terminal status: succeeded
- Firmware terminal detail: 26 raw
- Shoulder goal/actual: 3421/3447 raw (26 raw error)
- Elbow goal/actual: 1537/1553 raw (16 raw error)
- Wrist Flex goal/actual: 1204/1209 raw (5 raw error)
- Elbow load/current: 121/6 raw
- Elbow voltage/temperature: 12.5 V/34 C
- Gripper goal/actual: 1963/1984 raw
- Gripper load/current: 96/4 raw
- User physical observation: object lifted successfully

The Elbow P28 residual stayed at 16 raw under the loaded lift, with no
protection fault or loss of grasp. This matches the best P24 lift residual
while retaining the improved 13-raw isolated result. Elbow P28 is accepted for
continued bounded Stage 7 testing. No Wrist Flex or Wrist Roll gain increase
is justified by this result.

## Required physical gates

1. Keep P24 as the rollback candidate.
2. Before autonomous pick/place, repeat the complete perception-to-place path
   with the established one-shot physical approvals and collision checks.
3. Continue monitoring Elbow load/current and temperature during longer
   loaded paths; do not increase Wrist gains without axis-specific evidence.
