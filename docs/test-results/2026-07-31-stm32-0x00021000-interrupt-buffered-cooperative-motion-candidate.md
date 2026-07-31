# STM32 0x00021000 interrupt-buffered cooperative-motion 후보

## 판정

- 상태: **물리 배포 후 READ_ONLY 계약 gate 거절; 로봇 이동 0회**
- 이전 0x00020F00: 무동작 heartbeat는 통과했지만 실제 Shoulder setpoint에서
  status=8 detail=4와 stop latch가 재현되어 motion gate **거절**
- 목적: timeout 증가나 토크 증가가 아니라 host UART와 servo UART의 구조적 경합 제거

## 20F 실패 증거와 근본 원인

Fresh Shoulder -0.08 rad / 2초 단 1회에서 다음 순서가 관측됐다.

1. setpoint accepted
2. transient heartbeat delay 2회
3. terminal status=8 detail=4 (ACTUATOR_STATE_HOLD)
4. STM32 stop latch

정지 상태 MOTION_ENABLED는 약 243.5초 정상이고 실패는 motion service가 시작될 때만
재현됐다. MCU는 1Mbps servo UART에서 telemetry/position을 동기식으로 읽는 동안
115200bps host LPUART를 한 바이트 polling했다. heartbeat frame이 servo transaction과
겹치면 LPUART 하드웨어 수신이 overrun되어, 뒤에서 64바이트를 drain해도 이미 유실된
바이트는 복구할 수 없다. 따라서 20F의 bounded polling drain은 충분조건이 아니었다.

## 210 구조 변경

### Interrupt-buffered host RX

- LPUART1 RX interrupt와 1024-byte power-of-two ring buffer 추가
- ISR은 byte 저장과 다음 1-byte receive rearm만 수행; protocol parsing/송신 없음
- main loop가 ring을 bounded drain한 뒤 heartbeat safety를 평가
- ring overflow, UART error, receive rearm 실패는 원자적 fault flag로 전달
- main loop는 기존 fault를 byte parsing 전에 소비하고 invalidated ring을 전부 폐기
- 모든 RX fault는 ORE flag 유무와 무관하게 parser reset, rejected count 증가,
  actuator HOLD, stop latch로 fail-closed
- capability bit 0x40; 전체 capabilities 0x0000007F
- host identity가 bit 0x40 없는 firmware를 ARM 전에 거부

### Cooperative servo I/O

- 시작 위치 6축 읽기를 Servo_PositionSweepStep으로 분할: main-loop당 한 transaction
- motion safety telemetry를 16ms round-robin slot으로 분할: 6축 약 96ms sweep
- endpoint position verification도 main-loop당 한 축만 읽음
- heartbeat input은 servo transaction 중 ISR ring에 보존되고 transaction 사이에 ACK
- Action terminal 여유를 3.5초로 조정해 최악 시작 sweep + motion + 1초 settling 포함
- firmware heartbeat 500ms, host ACK timeout 250ms, load/current 기준과 torque limit,
  calibration hash 0x4D62F8D5는 변경하지 않음

## 검증

- interrupt RX/cooperative motion/physical disable/identity 표적: 15 passed
- ROS source 환경 전체 Python/계약: 283 passed
- native actuator protocol/safety C: 1/1 passed
- single_arm_bridge 독립 colcon build: PASS
- Cortex-M4 hard-float Release build: PASS, compiler/link warning 0
- image: text 30052, data 112, bss 4160, total 34324 bytes
- HEX size 84936 bytes, BIN size 30172 bytes

SHA-256:

~~~text
2c9b0f05063890e093d1a910ae4ff11778393cb550862c63badfef2a46bccfca  stm32_g474_single_arm.hex
4ff0f8fbae98ed053e05cac6f998ca9a892300dbf3b333fa49bfd5facc6f4d95  host_uart_rx.c
03464644e87fb7f5bc2b2f0c96a11b1e6c43cfb4f88ee3327e8ae9f0e655a729  servo_bus.c
edc88971f79c0d80ad869694ffeb27da9a55742d99ede3c7ef64cab3d9877443  binary_control.c
5edef7ad1d98a16b82db48c1989884766ba1670670b821f65b7da44c67133219  single_arm_app.c
e5f9a7fbb4f6ab967496ae0779da52e7bdaeca307dbb7b5e18da76c12a0d5371  hardware_identity.py
~~~

## 210 물리 적용 결과

- 20F full-flash backup: 524288 bytes, SHA
  `d2dcb2ded71d5f49f4507def9de0730ba4420476c166bc162460e12e8f97128b`
- 210 HEX program/verify/reset 및 post-flash identity PASS:
  firmware `0x00021000`, calibration `0x4D62F8D5`, capabilities `0x7F`
- 첫 READ_ONLY는 연결됐지만 재연결에서 `DISABLE rejected: status=1`로 bridge 종료
- latched 축 독립 진단: `STOP_LATCHED=1`, 6축 status 0/read status `0x00`,
  torque OFF, 전압 `12.3..12.5 V`, 위치 응답 정상
- 결론: 물리 torque OFF는 성공했지만 FAULT/ESTOP 상태의
  `actuator_safety_request_disable()`가 BAD_STATE를 반환했고, handler가 그 값을
  물리 DISABLE 결과로 그대로 노출했다. 서보·배선·전원 문제가 아니라 응답 계약 오류다.
- clear fault와 로봇 이동은 실행하지 않았다. 후속 수정은
  [0x00021100 idempotent physical DISABLE 후보](2026-07-31-stm32-0x00021100-idempotent-physical-disable-candidate.md)에 분리한다.

## 다음 물리 gate — 현재는 실행 금지

현재 하드웨어의 마지막 확인 상태는 0x00021000, READ_ONLY DISABLE 계약 실패 후 stop latched이며 6축 torque는 OFF다. Bridge 종료와 12V OFF를 먼저 사용자에게 재확인해야 한다. 이후 절차는 0x00021100 후보 문서를 따른다.

1. robot process 없음, 12V OFF, 팔 지지 확인
2. 현재 20F flash 512KiB 신규 backup과 SHA 생성
3. host/HEX Pi 전송, source SHA 재확인, single_arm_bridge rebuild
4. 로컬 HEX SHA와 Pi HEX SHA 일치 확인
5. 별도 명시 승인 후 210 program/verify/reset 1회
6. identity: protocol 1, joints 6, firmware 210, calibration 0x4D62F8D5, capability 0x7F
7. READ_ONLY 6축 physical torque OFF + heartbeat 60초
8. MOTION_ENABLED 무동작 5분 + heartbeat ACK count/diagnostics
9. 그 뒤에만 Shoulder -0.08 rad / 2초 단 1회

어느 gate에서든 heartbeat warning, RX fault, latch, identity mismatch가 나오면 동작 시험은
중단한다. grasp/lift/place 및 자동 재시도는 계속 금지한다.
