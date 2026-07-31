# Host DISABLE timeout contract

Date: 2026-07-31

## Observation

After flashing and validating STM32 firmware `0x00021700`, the first READ_ONLY
bridge startup failed while waiting for the `DISABLE` `STATE_FEEDBACK`:

```text
single_arm_bridge.transport.TransportError:
timeout waiting for STATE_FEEDBACK
```

The bridge was stopped, 12 V was turned off, and the arm was physically
supported before diagnosis continued.

## Root cause

The host allowed only 0.5 seconds for `DISABLE`. Firmware intentionally does
not acknowledge this request until it has attempted torque-off writes and
physical torque-register readbacks for all six servos.

The HAL timeout envelope is:

- six writes: `6 * (100 ms transmit + 2 ms optional status receive) = 612 ms`
- settling delay: `5 ms`
- six readbacks: `6 * (100 ms transmit + 100 ms receive) = 1200 ms`
- total firmware envelope: `1817 ms`

Therefore, the previous 500 ms host timeout could expire while firmware was
still performing its required physical safety checks.

## Correction

`DISABLE_RESPONSE_TIMEOUT_S` is now 2.5 seconds. This is a dedicated timeout;
heartbeat, position feedback, and diagnostics retain their tighter existing
bounds.

The firmware, protocol identity, calibration, and HEX are unchanged. No STM32
reflash is required for this host-only correction.

## Contract test

The physical-disable contract now reads the actual HAL timeout values from the
firmware source, computes the six-axis worst-case envelope, and requires the
host timeout to exceed it by at least 500 ms.

An initial test implementation selected the forward declarations of
`Servo_WriteData` and `Servo_ReadData` instead of their definitions. That test
parser error was corrected to select the final function definitions; it did
not indicate a product-code failure.

## Verification

- focused physical-disable and transport suite: `24 passed`
- `single_arm_bridge` local rebuild: passed
- complete Python/ROS suite: `323 passed`
- Pi transfer: not performed under this change approval
- STM32 modification or flash: not performed
- robot motion: not performed
