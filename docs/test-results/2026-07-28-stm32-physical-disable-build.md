# STM32 physical torque-disable firmware build

- Date: 2026-07-28
- Board target: NUCLEO-G474RE / STM32G474RETx
- Firmware identity: `0x00020800`
- Calibration identity remains unchanged: `0x3DB42B48`
- Physical robot power during build/flash: OFF
- Flash performed: YES, OpenOCD program/verify PASS
- Final robot power state: OFF

## Safety change

The host `DISABLE` transaction now calls `Servo_DisableTorqueAll()` before it
can return a successful state response. The firmware:

1. writes `0` to STS3215 Torque Enable register `40` for all six servo IDs;
2. continues through all IDs even if one write fails;
3. reads register `40` back from all six servos;
4. reports a safety fault and latches stop if any write/readback fails;
5. invalidates the previous servo trajectory configuration.

The host identity gate was changed to require firmware `0x00020800`, so an old
`0x00020700` image cannot silently run with the new bridge.

## Build verification

Toolchain:

- `arm-none-eabi-gcc 13.2.1`
- `CMake 3.28.3`
- Cortex-M4 hard-float Release build

Result:

```text
text   data   bss    dec    hex
26116  112    3080   29308  727c
```

The ELF is `ELF32`, little-endian, ARM EABI5, hard-float, with entry point
`0x08003d81`. Its vector table begins with stack pointer `0x20020000` and reset
handler `0x08003d81`.

`arm-none-eabi-nm` and disassembly verified that:

- `Servo_DisableTorqueAll` is present at `0x080024dc`;
- the binary request handler calls it before the state response path;
- Torque Enable register address `40` is present in the compiled function.

Build artifacts were generated locally and verified, then intentionally removed from
Git because ELF/HEX/BIN/MAP are reproducible outputs. The immutable hashes below and
the source/build contract remain as the permanent evidence.

SHA-256:

```text
7aea1a4b63d3c6778246e19e130c65060f6831bada68f343568a2624891f8561  ELF
000a4737ad94ad8a0453d682fdc7fb0326e7ad2009cf3aef9ffae20cc643122a  HEX
8303373c37274b491702d98805a760f44ef048a798fe75df15160f5df27f20d5  BIN
```

The complete 512 KiB pre-flash rollback readback is stored on the Pi at:

```text
/home/pi/firmware_updates/backup/stm32_before_0x00020800.bin
SHA-256 021f386ae02889d4632baeac19e4bff81c7c1415d4e5eab7e0e39ad969beef76
```

Tests:

- STM32 physical-disable/host identity contract: PASS
- related bridge/action tests: 32 PASS
- platform-independent actuator C core: 1/1 PASS
- compiled ELF physical-disable symbol/call inspection: PASS

## Flash and physical acceptance

OpenOCD identified ST-LINK V3 and the STM32G47/G48 Cortex-M4 target at
`3.297 V`, with `512 KiB` dual-bank flash and RDP level 0. Programming and
verification both completed:

```text
Programming Finished
Verified OK
Resetting Target
```

The read-only HELLO identity check then reported:

```text
protocol=1
joints=6
firmware=0x00020800
calibration=0x3DB42B48
capabilities=0x0000000F
stop_latched=0
HOST_IDENTITY_GATE=PASS
```

With the arm mechanically supported, the bridge was started once with no
trajectory or setpoint command. It entered `MOTION_ENABLED`. Ctrl+C cleanly
terminated the bridge, no `DISABLE during shutdown failed` message appeared,
and the operator physically confirmed that the five arm axes and gripper lost
holding torque. The 12 V servo supply was then switched OFF.

Physical result:

- host/firmware identity gate: PASS
- bridge enable with no setpoint: PASS
- shutdown DISABLE acknowledgement: PASS
- actual six-servo torque release: PASS
- process clean shutdown: PASS
- unintended commanded motion: 0
- final 12 V state: OFF

The fail-closed readback-error path remains covered by source/host tests. No
fault was deliberately injected into the live servo bus during this acceptance
test.
