# Stage 7 gripper hold setpoint remediation

Date: 2026-07-31  
Status: HOST ROOT FIX PHYSICALLY VERIFIED THROUGH FIRST LOADED LIFT

## Physical observation

The `0x00021500` P32 arm reached the grasp pose and closed successfully around
the object. Immediately after the close:

- requested gripper position: `0.13 rad`, raw Goal Position `1963`;
- measured contact position: about `0.098 rad`, raw position `1984`;
- load magnitude: `96`; current: `4`; temperature: `37 C`.

The first approved 20 mm lift segment then completed with firmware status 6,
detail 26, and the user confirmed that the object physically lifted. However,
the post-lift diagnostic showed:

- gripper Goal Position changed from `1963` to `1984`;
- measured position was `1985`;
- load and current both fell to zero.

The second lift segment was therefore canceled. The object was recovered, both
ROS processes were stopped, 12 V was removed, and the arm was left safe.

## Root cause

Both ROS Actions ultimately send one six-axis STM32 setpoint. The arm adapter
accepted five arm targets and filled the omitted sixth axis from the latest
**measured** gripper feedback:

```text
arm target + actual gripper position
```

During a contact grasp, measured position is intentionally different from the
commanded closing target. Reusing measured position converted the contact
residual into a new relaxed gripper goal during the lift. The object remained
held only by geometry and passive friction, so the physical lift does not count
as a valid active-hold lift gate.

The symmetric defect also existed in the gripper adapter: it preserved measured
arm positions instead of the last successfully commanded arm targets.

## Host-only correction

A shared, thread-safe `CommandedSetpointState` now belongs to one bridge
instance and is passed to both Action adapters.

- Before any successful command, omitted axes fall back to fresh physical
  feedback for that one goal only.
- A full six-axis target is committed only after firmware reports successful
  motion completion.
- A later arm goal preserves the committed gripper target, including a contact
  target that differs from measured gripper position.
- A later gripper goal preserves the committed arm targets.
- Abort, cancel, connection loss, explicit fault recovery, transport fault,
  shutdown, and adapter destruction discard the stored target.
- Rejected or invalid feedback is never committed as command intent.

This is a host-only semantic correction. Firmware remains `0x00021500`,
calibration remains `0x095CB9A5`, and no torque, gain, limit, tolerance,
trajectory, protocol, or STM32 flash change is required.

## Local verification

- focused state/arm/gripper suite: 30/30 passed;
- ROS-overlay full repository suite: 212/212 passed;
- `single_arm_bridge` build: passed;
- installed module import: passed;
- `single_arm_bridge` ament result: 21 tests, 0 errors/failures.

The key regression simulates contact feedback at `0.07 rad` after a successful
`0.10 rad` gripper command. A following arm Action is required to transmit
`0.10 rad` (`100000 urad`) on the gripper axis rather than the measured
`0.07 rad`.

## Physical regression result

The corrected host was deployed on the Pi and tested with firmware
`0x00021600`, calibration `0x8AD27897`, Shoulder P32, and Elbow P28. The
gripper closed around the object with commanded Goal Position raw `1963`,
measured contact position raw `1984`, load `96`, and current `4`.

One approved approximately 20 mm arm lift then completed successfully with
firmware status 6 and detail 26. The immediate diagnostic confirmed:

- gripper Goal Position remained raw `1963`;
- measured contact position remained raw `1984`;
- load remained `96`;
- current remained `4`;
- torque remained enabled;
- the user confirmed that the object physically lifted.

The arm Action therefore preserved the active gripper contact command instead
of replacing it with measured feedback. The root correction is physically
verified for the first loaded lift.

## Remaining scope

This result does not authorize unattended or autonomous pick/place. The next
gate is a complete perception-to-place run with the existing collision,
workspace, diagnostics, and one-shot motion approvals. Abort/cancel and fault
recovery behavior remains covered by the automated tests and should continue
to be observed during that run.
