# Stage 7 Shoulder command/load diagnostics candidate

Date: 2026-07-31
Status: 0x00021500 P32 PHYSICAL GATE PASSED; CONDITIONALLY ACCEPTED

## Problem statement

Repeated Shoulder `-0.08 rad / 2 s` tests ended with final errors of 42 and 52 raw.
The last stalled snapshot reported `position_raw=3301`, `load_magnitude_raw=220`,
`current_raw=21`, `voltage=12.3 V`, `P=16`, and runtime torque limit 780. These
values do not prove torque-limit saturation. The previous diagnostics did not expose
the servo Goal Position register or EEPROM protection configuration, so command
application and physical load could not be separated.

## Official SO-101 baseline

- SO-101 follower: six STS3215 motors, all 1/345 gearing.
- 12 V STS3215 must use the 12 V supply; the 7.4 V and 12 V variants are not
  interchangeable.
- Current LeRobot follower defaults are P=16, I=0, D=32 and commands
  `Goal_Position` with a sync write.

References:

- https://huggingface.co/docs/lerobot/en/so101
- https://huggingface.co/docs/lerobot/main/assemble_so101
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/config_so_follower.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py

## Local diagnostic-only change

Corrected candidate identity is firmware `0x00021300`, capabilities `0x000000FF`. No gain,
torque cap, calibration limit, trajectory, or safety threshold was changed.

The per-joint diagnostic payload now exposes:

- runtime Goal Position register 42..43;
- model number and servo firmware version;
- EEPROM maximum torque limit;
- minimum startup force and CW/CCW dead zones;
- protection current, operating mode, protective torque/time, and overload torque.

This allows one later controlled test to distinguish:

1. Goal register mismatch: host/firmware/servo command path defect.
2. Goal matches and load/current stays low: servo configuration, gear/voltage variant,
   or position-loop authority issue.
3. Goal matches and load/current rises toward limits: real payload/moment deficit.

## Local verification

- ROS-sourced full Python suite: 290 passed.
- Native actuator core: build PASS, CTest 1/1 PASS.
- Cortex-M4 Release build: PASS.
- `git diff --check`: PASS.
- Rejected deployed `0x00021200` HEX SHA-256:
  `305a04abc30afbc9991597a295b1459fa9809d7c102008bf3d52725cb47a234c`
- Corrected local `0x00021300` HEX SHA-256:
  `52f8b62426bab7742ab8a9250b60439633bb8c12f6b35ef97bb5dc77515aaa7b`

## 0x00021200 deployment result

The host and firmware identity gates passed with firmware `0x00021200`,
calibration `0x4D62F8D5`, and capabilities `0x000000FF`. The first READ_ONLY
diagnostic request then failed closed before any robot movement:

```text
servo diagnostics rejected: diagnostic read failed:
joint_index=0 status=2 read_status=0x10
```

Root cause was deterministic in the firmware. `Servo_ReadData()` permits at
most 16 bytes per bus transaction, while the new protection diagnostic asked
for EEPROM addresses 13..39 in one 27-byte request. The function returned
`HAL_ERROR` locally without transmitting that request to the servo.

The `0x00021300` correction keeps the same register map and diagnostic payload,
but reads the block as two bounded transactions: addresses 13..28 (16 bytes)
and 29..39 (11 bytes). A contract test now rejects reintroduction of the
oversized read.

## 0x00021300 READ_ONLY deployment result

The corrected firmware passed host identity with firmware `0x00021300`,
calibration `0x4D62F8D5`, and capabilities `0x000000FF`. The six-axis READ_ONLY
diagnostic returned `success=True`; no motion or fault clear was requested.

All six servos reported model 777, firmware 3.10, EEPROM maximum torque 1000,
minimum startup force 16, CW/CCW dead zones 1/1, protection current 310,
operating mode 0, protective torque 20, protection time 200, and overload
torque 80. Stationary voltage was 12.3..12.4 V. Shoulder runtime settings were
P/D/I 16/32/0 and torque limit 780 as commanded.

Because READ_ONLY physically disables torque, load and current were zero and
Goal Position remained the last commanded value. The large Elbow
position/goal difference therefore records supported manual repositioning, not
a failed motion command. The next discriminating gate is one MOTION_ENABLED
command followed immediately by diagnostics while torque remains enabled.

## 0x00021300 single-motion diagnostic result

One approved Shoulder command moved the ROS target from 1.8775924844 to
1.7975924844 rad over 2 s while preserving the other arm joints. The firmware
returned a soft abort with final error 37 raw and did not latch the safety stop.
Immediate diagnostics showed:

- Shoulder Goal Position: 3220 raw (the exact requested target);
- Shoulder actual position: 3257 raw;
- measured residual: 37 raw, matching the terminal result;
- load magnitude: 160 raw; current: 13 raw; voltage: 12.3 V;
- runtime P/D/I: 16/32/0; torque limit: 780; maximum torque: 1000.

This rules out a dropped/incorrect Goal Position, host/firmware conversion
defect, torque-limit saturation, and stationary supply-voltage collapse. The
servo accepted the final goal but settled about 3.25 degrees away at low
reported load/current. The remaining dominant mechanism is insufficient
Shoulder position-loop stiffness under gravity combined with gearbox backlash
and structural compliance. Increasing the completion tolerance would hide the
error and is not the remedy. The next controlled candidate is Shoulder P=24
(the same value already used by Elbow), with torque and safety limits unchanged.

## 0x00021400 Shoulder P=24 candidate

The evidence-driven correction changes only the Shoulder runtime P gain from
16 to 24, matching the existing Elbow gain. Shoulder torque limit remains 780;
load/current watchdogs, raw ranges, trajectory timing, and final tolerance are
unchanged. Because P gain is part of the hardware calibration identity, the
host and firmware calibration hash changes together from `0x4D62F8D5` to
`0xAFCC3512`.

Local verification:

- ROS-sourced full Python suite: 290 passed;
- native actuator core: CTest 1/1 passed;
- Cortex-M4 Release build: passed;
- `git diff --check`: passed;
- `0x00021400` HEX SHA-256:
  `fb7613256ba6ab4f1e754fe97151223457a2a23d0b1f65b46b21cb7ddf2178fb`.

No Pi deployment, STM32 flash, fault clear, or robot movement was performed by
this local candidate build.

## 0x00021400 Shoulder P=24 motion result

The approved P=24 comparison command moved Shoulder by -0.08 rad over 2 s.
Goal Position was written correctly as 3537 raw; actual position settled at
3568 raw, producing a 31-raw soft abort without a safety latch. Immediate
diagnostics reported load 196, current 19, voltage 12.2 V, P/D/I 24/32/0,
and torque limit 780. No Shoulder oscillation or voltage collapse was reported.

Compared with the P=16 residual of 37 raw, P=24 reduced the residual to 31 raw
and increased achieved travel, but missed the unchanged 30-raw acceptance
limit by one count. Load and current remain far below the independent 800/320
watchdogs. This supports one final bounded proportional candidate, Shoulder
P=32, with a stop rule: if P=32 still cannot meet tolerance without oscillation,
do not keep increasing gain; move to gravity compensation/counterbalance or a
designed integral controller.

## 0x00021500 Shoulder P=32 final bounded candidate

The final proportional-only candidate changes Shoulder P gain from 24 to 32.
Shoulder torque limit remains 780, final tolerance remains 30 raw, and all
load/current watchdogs, trajectory timing, raw limits, and shutdown behavior are
unchanged. The synchronized host/firmware calibration identity is `0x095CB9A5`.

Local verification:

- ROS workspace: 8 packages built;
- ROS-sourced full Python suite: 290/290 passed;
- `single_arm_bridge` ament result: 21 tests, 0 errors/failures;
- native actuator core: CTest 1/1 passed;
- Cortex-M4 clean Release: passed, text 30228, data 112, bss 4160;
- `git diff --check`: passed;
- `0x00021500` HEX SHA-256:
  `6a78cd9eaaadd284af2f35333c7f1317c7c4afe99b023cfcddfe8c98d9c62c23`.

No Pi deployment, STM32 flash, fault clear, or robot movement was performed.
This is the stop-rule candidate: one comparison motion only. If it still misses
30 raw or introduces oscillation, proportional gain will not be raised again.
The next remedy must be gravity compensation/counterbalance or a separately
designed integral controller with its own bounded safety validation.

## 0x00021500 Shoulder P=32 physical result

The approved single comparison command moved Shoulder from 2.3009711818 to
2.2209711818 rad over 2 s. Firmware reported `state=succeeded`, status 6,
detail 26: the unchanged 30-raw final tolerance was met without a safety latch
or retry. Immediate diagnostics showed Goal Position 3496 raw, actual position
3522 raw, load 216, current 21, voltage 12.2 V, temperature 35 C, P/D/I
32/32/0, and runtime torque limit 780.

The user observed vibration, but judged it equal to the normal motion visible in
SO-ARM101 reference videos rather than a new P32 oscillation. Shutdown was clean,
12 V was removed, and the arm was left safe. The bounded progression therefore
closed as P16=37 raw, P24=31 raw, P32=26 raw. P32 is conditionally accepted for
Stage 7; no further proportional increase or repeat of this diagnostic motion is
authorized. Future trajectory trials must continue monitoring vibration,
temperature, voltage, load, and current. Any growth beyond the observed baseline
reopens the gate and requires gravity compensation/counterbalance or separately
designed integral control rather than P gain above 32.

## Required next gates

1. With 12 V OFF, physically confirm that Shoulder ID 2 is an STS3215 12 V
   follower motor with 1/345 gearing. Model register 777 alone cannot distinguish
   voltage or gear variant.
2. Deploy host and firmware together only after explicit approval.
3. Run READ_ONLY diagnostics first; no movement.
4. Run exactly one supported Shoulder motion and capture diagnostics immediately
   after the terminal result, before stopping the bridge.
5. Tune gain or redesign mechanics only after the branch above is proven.
