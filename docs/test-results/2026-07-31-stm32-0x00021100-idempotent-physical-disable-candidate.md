# STM32 0x00021100 idempotent physical DISABLE 후보

## 판정

- 상태: **identity·READ_ONLY 물리 수락; MOTION_ENABLED/로봇 이동 미실행**
- 대상 결함: 0x00021000이 latched READ_ONLY 재연결에서 물리 torque OFF에 성공하고도
  `DISABLE status=1(BAD_STATE)`을 반환한 계약 오류
- 안전 목표: fault/latch를 해제하지 않고, DISABLE의 물리 결과만 정확히 응답

## 물리 증거

0x00021000 latched 진단은 다음을 동시에 보였다.

- `STOP_LATCHED=1`
- servo ID 1..6 모두 command/read status 0
- 6축 모두 `TORQUE=0`
- 전압 `12.3..12.5 V`, position feedback 정상

따라서 서보, 배선, 전압 또는 torque-disable 자체의 실패가 아니다.

## 근본 원인과 계약 수정

기존 DISABLE handler는 먼저 `actuator_safety_request_disable()`을 호출했다.
FAULT/ESTOP에서는 이 함수가 BAD_STATE를 반환하지만, 뒤의
`Servo_DisableTorqueAll()`은 실제로 성공했다. handler가 논리 전이 결과를 물리
DISABLE 결과로 사용해 host가 안전한 torque OFF 상태를 실패로 오판했다.

0x00021100에서는 DISABLE을 다음과 같이 정의한다.

1. 모든 상태에서 6축 torque OFF write/readback을 수행한다.
2. 정상 상태면 기존처럼 SAFE_DISABLED로 전이한다.
3. FAULT/ESTOP이면 논리 상태와 stop latch를 보존한다. clear fault는 수행하지 않는다.
4. 6축 물리 readback 성공은 status 0, 실패는 status 2로 반환한다.
5. status 0은 motion 허용이 아니라 torque OFF 확인만 의미한다.

## 로컬 검증

- 전체 Python/ROS 및 계약 테스트: 284 passed
- native actuator protocol/safety C: 1/1 passed
- Cortex-M4 hard-float Release build: PASS, compiler/link warning 0
- image: text 30060, data 112, bss 4160, total 34332 bytes
- HEX size 84952 bytes, BIN size 30180 bytes
- regression: FAULT/ESTOP 보존, latch clear 금지, physical disable before ACK 확인

SHA-256:

~~~text
8fd11d901b141cd959c995ed3101f1f2556809b7f417cd967c228b1efb2a7858  stm32_g474_single_arm.hex
106ea27020f1c42a9d653a0f5c5b75032c8aaaff7c5c7bfda0e05f976a121d5a  binary_control.c
56e9c2693d04beb2704e42b63ffa2b82928c8aa299334041b00d2a31d7c9baa6  hardware_identity.py
~~~

## 물리 적용 결과

- 210 full-flash backup: 524288 bytes, SHA
  `b6cbd426e5409a84afadaa81030aa4d2c24a90cb97a31ea2e9182bd638861e93`
- 검증된 211 HEX program/verify/reset PASS
- post-flash identity: protocol 1, joints 6, firmware `0x00021100`,
  calibration `0x4D62F8D5`, capabilities `0x7F`, latch 0, heartbeat ACK 20
- 첫 READ_ONLY와 60초 유지 뒤 diagnostics: 6축 torque OFF, current/load 0,
  전압 `12.3..12.5 V`, heartbeat/feedback/DISABLE 오류 없음
- reset·clear fault·12V 재인가 없이 종료 후 두 번째 READ_ONLY 재연결 PASS;
  `DISABLE status=1` 재발 0회, 두 번째 diagnostics도 6축 torque OFF
- 첫 종료에서 ROS context invalid publish 경고를 별도 host lifecycle race로 분리
- timer quiescence/context guard host 보강: 전체 287 tests, 독립 ROS build PASS,
  실제 READ_ONLY 재연결·torque OFF·Ctrl+C 무경고 physical DISABLE 종료 PASS
- 이어서 MOTION_ENABLED 무동작 약 `330.98 s` 유지 PASS: heartbeat/feedback/latch
  경고 0, 6축 위치 고정, 전압 `12.3..12.5 V`, current 최대 1 raw, 온도 상승 최대 약 2°C
- 무경고 정상 종료 뒤 독립 diagnostics에서 6축 torque OFF, current 0 readback PASS
- clear fault와 로봇 이동은 실행하지 않았다.

## 다음 물리 gate — 현재는 실행 금지

0x00021100의 배포·identity·연속 READ_ONLY·6축 torque OFF·무경고 종료 gate는 통과했다. 남은 절차를 분리한다.

1. 팔 주변 비움, 지지, 12V ON 상태 재확인
2. MOTION_ENABLED 재연결과 현재 joint state 확인
3. 별도 명시 승인 후 Shoulder -0.08 rad / 2초 단 1회
4. terminal status, fresh feedback, heartbeat warning과 diagnostics 확인
5. 실패 시 자동 재시도 없이 즉시 shutdown; 성공 후에만 pregrasp 재검토

어느 gate에서든 warning, RX fault, identity mismatch 또는 예상하지 않은 torque ON이
나오면 동작 시험은 중단한다. grasp/lift/place와 자동 재시도는 계속 금지한다.
