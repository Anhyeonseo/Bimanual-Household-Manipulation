# STM32 0x00021700 position-read recovery candidate

Date: 2026-07-31

## Incident

During the Stage 7 pregrasp-to-grasp transition, the executor sent no motion
command because fresh `/joint_states` stopped arriving. The bridge log showed:

1. a background `GET_STATE` response with `status=2`;
2. a heartbeat response 8 ms later with `status=0` and `latched=1`;
3. a second `GET_STATE` failure, after which the host entered its transport
   fault state.

The heartbeat was not late. An exhausted `Servo_ReadAllPositions()` sweep in
the background `GET_STATE` path had already latched the MCU stop. The failing
servo ID existed only in an internal firmware variable, and the host shared one
consecutive-error counter between heartbeat and feedback traffic.

## Candidate policy

- Firmware identity: `0x00021700`
- Capabilities: `0x000001FF`
- New capability bit: `0x00000100`, position-read failure diagnostics
- A background position sweep still uses the existing per-servo retries.
- One exhausted background sweep returns a 24-byte `STATE_FEEDBACK` containing:
  failed servo ID, failure streak, and configured failure limit.
- A successful complete sweep resets the background failure streak.
- The MCU latches only after 3 consecutive exhausted background sweeps.
- Motion-start and motion-final-verification read failures remain immediate
  fail-closed conditions.
- The host maintains independent heartbeat and feedback error counters.
- A reported MCU latch remains an immediate host fault.

At the configured 5 Hz feedback rate, three failed background periods span
approximately 0.4 seconds from the first failed response to the third. Because
each sweep already retries the failing servo three times, latching represents
nine exhausted per-servo attempts across those three periods.

## Changed surfaces

- STM32 version, capability, and failure-limit configuration
- STM32 background `GET_STATE` failure state machine and diagnostic response
- Host identity gate
- Host and tool protocol parsing for 20-, 24-, and 32-byte state responses
- Typed host position-read and stop-latch errors
- Independent host heartbeat and feedback recovery counters
- Protocol documentation and regression contracts

## Verification

- Python/ROS regression suite: `322 passed`
- Native actuator C core: `1/1 passed`, warnings treated as errors
- `single_arm_bridge` local `colcon build`: passed
- STM32 ARM Release build: passed with no compiler warnings
- Firmware size: text 30516, data 112, bss 4160, total 34788 bytes
- Scoped `git diff --check`: passed

Two earlier full-suite invocations failed for test-environment reasons only:

1. repository root was missing from `PYTHONPATH`;
2. the freshly built ROS overlay was not sourced.

After correcting both environment settings, the complete suite passed. Two
additional transport round-trip tests were then added and the final total
became 322 passing tests.

## Local build artifacts

- HEX: `/tmp/stm32_g474_single_arm_0x00021700.hex`
- HEX SHA-256:
  `0cd04c457780892a2dc07a288396a043c67375d0d8e2f2e3eec2f52ce709795a`
- ELF: `/tmp/stm32_g474_single_arm_0x00021700.elf`
- ELF SHA-256:
  `760cb1397d65dcfd208b3dc2f366a8385edcfb63be5984aabc7c5a5a34fffc1a`

## Scope boundary

This candidate was modified, tested, and built locally only. No files were
transferred to the Pi, the STM32 was not flashed or reset, and no robot motion
was performed.

## Required physical gates

Before motion validation:

1. transfer the reviewed host files and verified HEX to the Pi with backups;
2. rebuild `single_arm_bridge` and verify installed-module hashes;
3. with 12 V off and the arm supported, back up the current STM32 flash;
4. separately approve and perform exactly one `program verify reset`;
5. verify firmware, calibration, capability, and heartbeat identity gates;
6. run READ_ONLY first and confirm physical torque disable;
7. only then perform a separately approved MOTION_ENABLED no-motion test and
   controlled position-read fault validation.
