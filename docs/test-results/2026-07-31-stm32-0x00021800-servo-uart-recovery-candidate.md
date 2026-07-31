# STM32 0x00021800 servo UART recovery candidate

Date: 2026-07-31

## Problem statement

Firmware `0x00021700` correctly changed background position reads to a
three-strike fail-closed policy and exposed the first failed servo ID. It did
not, however, repair the underlying UART receive state. Repeated field failures
still appeared as `servo_id=1` because ID 1 is the first axis queried in every
sweep, and resetting the STM32 temporarily restored communication.

The previous bus implementation assumed that every status packet began at the
first received byte. A partial or late WRITE response, stale bytes, a response
for another ID, or a UART ORE/FE/NE/PE/RTO condition could therefore poison the
next fixed-length READ. The failure path flushed only a subset of this state and
did not retain enough evidence to distinguish framing loss from a disconnected
or electrically unstable bus.

## Candidate policy

- Firmware identity: `0x00021800`
- Capabilities: `0x000003FF`
- New capability bit: `0x00000200`, servo-bus recovery diagnostics
- Each servo READ uses a bounded byte-stream parser: 50 ms and at most 64 bytes.
- The parser scans for `FF FF`, tolerates repeated sync bytes, and validates ID,
  length, status, and checksum before returning data.
- Stale prefixes, late packets for another ID, malformed lengths, and corrupt
  checksums are discarded while the parser continues looking for the expected
  packet in the same transaction.
- A terminal failure performs `HAL_UART_Abort`, clears ORE/NE/PE/FE/RTO,
  flushes RX data, waits a 2 ms quiet interval, and clears RX state again.
- WRITE no longer starts a fixed six-byte receive that can time out partway
  through an optional status response. It lets that response settle for 2 ms
  and atomically flushes it; safety-critical writes retain register readback.
- The existing three-strike background policy and immediate motion-boundary
  fail-closed behavior remain unchanged.

## Failure diagnostics

The position-read failure `STATE_FEEDBACK` grows from the compatible 24-byte
prefix to 40 bytes when capability `0x00000200` is present. It adds:

- failure reason (`TX`, `RX timeout`, `UART`, `header`, `ID`, `length`, servo
  `status`, `checksum`, or `recovery`);
- HAL status and servo status;
- cumulative UART recovery count and discarded-byte count;
- UART `ErrorCode` and USART ISR snapshots.

If a malformed frame is followed by silence, the concrete parser rejection is
preserved instead of being overwritten by the final byte timeout. A true
no-response condition remains `RX timeout`; active UART flags remain `UART`.
The host and standalone protocol tool parse both the legacy 24-byte response
and the new 40-byte response.

## Fault injection

The native C parser test injects and verifies recovery from:

1. arbitrary stale prefix bytes;
2. an overlapping/repeated `FF FF FF` synchronization boundary;
3. a complete late response from the wrong servo ID followed by the expected
   response;
4. a bad-checksum response followed by the expected response;
5. an invalid-length header followed by the expected response;
6. a target-servo status error as a terminal, classified failure.

Static firmware contracts additionally verify the full HAL abort/flag-clear/RX
flush recovery sequence, bounded receive loop, removal of the partial WRITE
reply drain, firmware identity, capability, and 40-byte diagnostic payload.

## Verification

- Python/ROS regression suite: `329 passed`
- Native parser fault-injection test: passed with `-Wall -Wextra -Wpedantic
  -Werror`
- Native actuator C core: `1/1 passed`, warnings treated as errors
- `single_arm_bridge` local `colcon build --symlink-install`: passed
- STM32 ARM Release build: passed with no compiler warnings
- Firmware size: text 31560, data 112, bss 4176, total 35848 bytes
- `git diff --check`: passed

The first full-suite invocation stopped during collection because ROS 2 and the
local package overlay were not sourced. After sourcing ROS 2 Jazzy and adding
the local packages to `PYTHONPATH`, all 329 tests passed.

## Local build artifacts

- HEX: `/tmp/stm32_g474_single_arm_0x00021800.hex`
- HEX SHA-256:
  `4b9ca7c7b3927ce798048258fb1b3deecfb0718d660c6c1bd93308862ef3f317`
- ELF: `/tmp/stm32_g474_single_arm_0x00021800.elf`
- ELF SHA-256:
  `2fd820e03fb2624d5f77fdc43d2d40361058b849cc110dc53f123ecd3306d0ca`

## Scope boundary

This candidate was modified, fault-injection tested, and built locally only.
No files were transferred to the Pi. The STM32 was not flashed or reset,
`CLEAR_FAULT` was not issued, and no robot motion was requested.

## Required physical gates

Before any motion validation:

1. review the local diff and transfer only the reviewed host files and verified
   HEX to the Pi, with backups;
2. rebuild `single_arm_bridge` on the Pi and verify source, installed-module,
   and HEX hashes;
3. with 12 V off and the arm supported, back up the current STM32 flash;
4. separately approve exactly one `program verify reset` using the verified
   HEX SHA;
5. verify firmware `0x00021800`, calibration `0x8AD27897`, capabilities
   `0x000003FF`, latch-clear state, and heartbeat identity;
6. run READ_ONLY first and confirm six-axis physical torque disable;
7. only then perform separately approved no-motion and controlled
   fault-injection validation.
