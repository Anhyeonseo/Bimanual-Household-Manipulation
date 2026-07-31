# STM32 0x00020F00 acknowledged-heartbeat 후보

## 목표

20E의 heartbeat RX starvation을 timeout 증가로 숨기지 않고, MCU 수신 순서와 host–MCU
계약을 함께 수정한다. 로봇 제어·안전 deadline은 유지하며 실제 수신 증거를 만든다.

## 변경

### MCU bounded RX drain

- binary mode에서 `BinaryControl_Service()`보다 먼저 host UART를 처리
- loop당 최대 64 byte, byte당 최대 1 ms
- 받은 각 byte를 즉시 COBS parser에 전달
- drain 뒤 기존 safety/motion service 실행
- bound 때문에 연속 host traffic이 safety service를 무한정 굶길 수 없음

### Acknowledged heartbeat

- 빈 `HEARTBEAT`를 수락한 뒤 같은 sequence의 `STATE_FEEDBACK` ACK 반환
- host는 250 ms 내 matching ACK를 받아야 성공
- ACK의 `status != 0` 또는 `stop_latched=1`이면 즉시 transport error
- capability bit `0x20`, 전체 capabilities `0x0000003F`
- host identity는 acknowledged-heartbeat capability가 없으면 fail-closed

### 진단 개선

- motion `status=8`의 detail에 상수 0 대신 실제 actuator safety state를 기록
- endpoint settling, load/current watchdog, torque limits와 calibration hash는 유지

## 로컬 검증

- heartbeat/RX 표적 계약 및 bridge core 포함: `28 passed`
- ROS 환경 전체 Python: `276 passed`
- actuator C core: `1/1 passed`
- single_arm_bridge ament identity: PASS
- Cortex-M4 hard-float Release: PASS, warning/link error 0
- image: text `26700`, data `112`, bss `3088`, total `29900` bytes

SHA-256:

```text
7f4e08027c996929a672aa46287f49e8b1364157e38db1e36b147409170edf78  stm32_g474_single_arm_0x00020F00.hex
78695545d575acdb185597e5e4a08d10eeff0237ae4db39bf53374a91de9de8d  single_arm_config.h
e1584d4ab5b8c1b92301146cd05dcc88ea4d45d43c8d2527063b5b2642ec270c  binary_control.c
27e0fe5786fc6b837543959aa1e6d67219598ee92ac2a5fb8fc0e9fef895e39b  single_arm_app.c
b2ad8c75fd219cafb58369923d5211db6855914b1108373a3ffc74d5a0f2cbf2  transport.py
4dbe0909cd2e6de7de387cc8cc91f433d87b08d3fb4f5aadc8e6bb1bab6ad2b5  hardware_identity.py
```

## 현재 상태와 다음 gate

- 현재 STM32: 20E, stop latched
- bridge: 종료
- 12V: OFF
- 20F host/HEX Pi 전송·host backup·single_arm_bridge rebuild: PASS
- Pi build module: `EXPECTED_FIRMWARE_VERSION=0x00020F00`
- Pi transport: `HEARTBEAT_RESPONSE_TIMEOUT_S=0.25`
- 현재 20E flash 512 KiB backup: PASS
  - path: `~/firmware_updates/backup/stm32_before_0x00020F00.bin`
  - SHA: `d8577ac39861d39489c60cd07f571fef98f29951bee6a917eb4e85472d365b66`
- 20F STM32 OpenOCD program/verify/reset: PASS
- post-flash identity + heartbeat ACK: PASS
  - protocol `1`, joints `6`, firmware `0x00020F00`
  - calibration `0x4D62F8D5`, capabilities `0x0000003F`
  - HELLO latch `0`, ACK status `0`, ACK latch `0`, heartbeat count `1`
- 첫 READ_ONLY connection/heartbeat/feedback/diagnostics: 통신 PASS, 물리 torque gate FAIL
  - 6축 모두 `torque_enabled=true`
  - 원인: host가 `allow_motion=false`에서 firmware `DISABLE`을 호출하지 않음
  - shutdown도 `allow_motion && motion_armed && !faulted` 조건과 선행 heartbeat에
    묶여 latched/faulted/READ_ONLY에서 physical disable을 보장하지 못함
- host-only 보강: READ_ONLY 시작, latched 시작, arming 예외, 모든 shutdown에서
  firmware의 6축 torque write/readback DISABLE을 강제하며 latched shutdown에는
  heartbeat를 선행하지 않음
- 보강 검증: 표적 `27 passed`, 전체 `280 passed`, 독립 ROS build/identity PASS
- 보강 Pi 전송·backup·rebuild: PASS
  - backup: `~/Manipulation/ros2_ws/.pre-readonly-physical-disable-20260731-023102`
  - source/build SHA: `93e1b61415020e5ba8ceeb4041cf33e97dc26e92b6e96aec8dde36bac2753e00`
- 보강 READ_ONLY 재시험: PASS
  - connection firmware `0x00020F00`, calibration `0x4D62F8D5`
  - diagnostics success, 6축 모두 `torque_enabled=false`
  - load/current 0, voltage `12.3..12.5 V`, heartbeat/feedback/latch 오류 없음
- MOTION_ENABLED 무동작 heartbeat ACK 지속성: PASS
  - 장시간 ACTIVE 뒤 diagnostics sample time 약 `243.5 s`
  - 6축 torque enabled와 P/D/I·torque limit readback 일치
  - Shoulder/Elbow torque `780/650`, current `2/3 raw`, voltage `12.3/12.4 V`
  - heartbeat/feedback/latch 오류 및 로봇 이동 없음
- shutdown 6축 physical DISABLE 사후 readback: PASS
  - Bridge 정상 종료, `DISABLE during shutdown failed` 없음
  - 별도 read-only diagnostics에서 6축 `torque_enabled=false`, current 0
  - 최종 raw `2279,3147,1601,1212,2136,1949`
- clear fault·로봇 이동: 미실행

다음 순서는 각 위험 동작을 분리 승인한다.

1. host 파일과 SHA 검증된 HEX를 Pi에 전송하고 workspace backup/rebuild
2. robot process 없음·12V OFF·팔 지지 확인
3. **현재 20E flash 전체 512 KiB backup**과 SHA 생성
4. 20F HEX SHA 재확인 후 별도 승인으로 program/verify/reset 1회
5. protocol 1, joints 6, firmware 20F, calibration `0x4D62F8D5`, capability `0x3F` 확인
6. READ_ONLY에서 heartbeat ACK를 연속 관측하고 diagnostics 1회
7. MOTION_ENABLED 무동작 heartbeat ACK 지속성 확인
8. 그 뒤에만 Shoulder 소각도 1회를 별도 승인

20F의 물리 heartbeat gate가 통과하기 전 grasp/lift/place는 실행하지 않는다.


## 최종 물리 판정 — 거절

READ_ONLY physical DISABLE, MOTION_ENABLED 무동작 약 243.5초, shutdown 6축 torque
OFF는 통과했다. 그러나 fresh Shoulder -0.08 rad / 2초 단 1회에서 heartbeat ACK
대기 경고 2회 뒤 terminal status=8 detail=4와 STM32 stop latch가 재현됐다.
20F는 실제 motion heartbeat gate를 통과하지 못했으므로 **물리 수락 거절**이다.

64-byte polling drain은 main loop가 servo UART 동기 transaction 안에 있는 동안 LPUART
하드웨어에서 이미 유실된 heartbeat byte를 복구할 수 없다. 후속 해법은 timeout 증가가
아니라 LPUART interrupt ring과 축별 cooperative servo I/O를 적용한
[0x00021000 로컬 후보](2026-07-31-stm32-0x00021000-interrupt-buffered-cooperative-motion-candidate.md)다.
