# 단계 7 Shoulder 근본 원인 감사와 0x00020E00 진단·정착 후보

## 결론

현재 단계 7 차단은 하나의 문제가 아니라 두 문제가 겹친 결과다.

1. 펌웨어 `0x00020D00`은 2초 보간 종료 후 100 ms를 기다려 위치를 한 번만
   읽고 즉시 terminal `status=6`을 보냈다. 큰 Shoulder 명령은 이 terminal 뒤에도
   실제 위치가 목표 쪽으로 더 정착했으므로 일부 soft-abort는 조기 판정이었다.
2. 목표까지 약 `0.079155 rad` 남은 작은 Shoulder 명령은 충분히 기다린 fresh
   feedback에서도 거의 움직이지 않았다. torque-limit readback은 통과했지만 기존
   프로토콜이 position만 내보내 실제 P/D/I, torque enable/limit, load, current,
   voltage를 동시에 볼 수 없어 static error의 원인을 구분할 수 없었다.

따라서 final-error 허용치나 torque를 다시 임의로 키우지 않는다. 먼저 terminal
판정을 고치고 실제 서보 상태를 측정한 뒤 P gain, 전원/배선, 기구 하중 중 하나를
선택한다.

## 0x00020D00에서 확인된 물리 사실

- firmware `0x00020D00`, calibration `0x4D62F8D5`, capabilities `0x0000000F`
- torque-limit register `48..49` readback gate가 MOTION_ENABLED 진입에서 통과
- Shoulder/Elbow torque limit `780/650`, P gain `16/24` 구성
- 큰 Shoulder 감소 명령은 terminal에서 실패해도 fresh feedback에서 상당 부분
  이동한 사례가 반복됨
- 마지막 pregrasp 목표 대비 fresh 상태:
  - Shoulder: `1.8561167533 rad`, 목표 `1.7769621126 rad`, 잔차 `0.079155 rad`
  - Elbow/Wrist는 목표 근처
- 마지막 작은 Shoulder 마무리 명령은 `detail=52 raw` 뒤 fresh feedback에서도
  Shoulder가 사실상 유지됨
- Elbow 물리 방향은 `+q=펴짐`, `-q=접힘`으로 재확정. 반대 방향 명령 1회는
  즉시 원래 각도로 복구했고 이후 계획에서 이 규약을 고정함

## 0x00020E00 로컬 변경

### 1. bounded multi-sample settling

- firmware/host identity `0x00020E00`
- 보간 종료 뒤 `100 ms` 간격, 최대 `1000 ms` 동안 위치를 반복 측정
- `30 raw` 이내가 2회 연속일 때 조기 성공 판정
- 최대 시간에 도달하면 마지막 측정 오차로 기존 `status=6`을 보고
- settling 동안 기존 load/current watchdog를 계속 poll
- telemetry/read 실패와 load/current 초과는 기존 fail-closed latch 유지
- arm/gripper ROS Action completion 여유를 `1.0 s → 2.0 s`로 증가

### 2. on-demand servo diagnostics

기존 예약 message id `51 (DIAGNOSTICS)`을 사용한다. GET_STATE payload
`[0x02, joint_index]` 하나가 한 서보만 읽고, host가 관절 사이마다 heartbeat를
갱신한다. 여섯 축을 하나의 긴 MCU transaction으로 읽지 않아 500 ms watchdog을
굶기지 않는다.

각 관절 응답에는 다음 실제 register/telemetry가 포함된다.

- Torque Enable
- P/D/I gain
- Torque Limit
- position, speed, load
- voltage, temperature, current
- calibration hash와 sample tick

ROS bridge는 `/get_servo_diagnostics` (`std_srvs/Trigger`)를 제공한다. Action이나
다른 진단이 servo bus를 소유한 동안에는 요청을 거부하며, 성공 시 여섯 관절의
compact JSON을 반환한다. 진단 read 실패는 동작 명령으로 처리하지 않으며 임의
재시도나 위치 변경도 하지 않는다.

## 로컬 검증

- 핵심 protocol/identity/settling 계약: `48 passed`
- ROS 설치 공간을 source한 전체 Python/ROS: `264 passed`
- `single_arm_bridge` ament/identity/lint: `21 tests`, 오류 0
- actuator C core: `1/1 passed`
- Cortex-M4 hard-float Release build: PASS, warning/link error 0
- image: text `26628`, data `112`, bss `3088`, total `29828` bytes
- Pi 전송, STM32 flash, 실제 로봇 명령: **미실행**

SHA-256:

```text
7c042e346a0dbcb4f74d4d0c73f20eedfa9e527349edacee01191870d67d9e0e  stm32_g474_single_arm_0x00020E00.hex
1497543d3b6d6ecfd54e985be1f3924c2b2d66efa681ccab56fcf44615b35592  single_arm_config.h
c3ea378e0d5a2c0b46bd27b15ad46876a8e2567c10e8d25b0d00b7b08f31959e  binary_control.c
3ce94ec734edba32dd0e179666da50866509b5d099506b00280f8eb63ea9165a  protocol.py
f928d302a3e95b1121326ae22e9e33bf74f4c7e31d358d0a435f116b3a2121d2  transport.py
ce1de1eb002c8985c3e4be0929bb50882b505add5143139793d4d82f0826ac5a  hardware_identity.py
```

## 물리 수락 순서와 판정

각 flash와 이동은 기존처럼 별도 단 1회 승인 뒤에만 수행한다.

1. 현재 `0x00020D00` 512 KiB rollback image와 새 HEX SHA 재확인
2. 12V OFF·팔 지지·robot process 없음 확인 뒤 program/verify/reset
3. HELLO에서 firmware `0x00020E00`, capability `0x10`, calibration
   `0x4D62F8D5`, stop latch 0 확인
4. READ_ONLY에서 diagnostics 1회: 6축 응답과 register 값 확인
5. MOTION_ENABLED 무동작 diagnostics 1회:
   Shoulder `torque_enabled=true`, `P/D/I=16/32/0`, torque limit `780` 확인
6. 승인된 Shoulder 소각도 1회 전/후 diagnostics와 fresh joint state 기록
7. 결과 분기:
   - voltage가 동작 시 뚜렷하게 하락하거나 load/current가 safety 한계에
     근접하면 P gain을 올리지 않고 전원·배선·기구 하중·카운터밸런스를 점검
   - 전압과 load/current에 충분한 여유가 있는데 static error가 남으면
     Shoulder P gain을 한 단계만 올리는 새 calibration/firmware 후보를 생성
   - multi-sample terminal은 성공하지만 fresh feedback이 다시 처지면
     payload 질량/COM과 hold 성능 문제로 분류

이 gate 전에는 pregrasp 반복, grasp, lift/place를 실행하지 않는다. 손목 카메라와
mount의 실제 질량·무게중심을 측정하기 전에는 URDF/Isaac inertia도 추정값으로
넣지 않는다.

## 2026-07-31 물리 수락 결과: 거절

`0x00020E00`은 실제 Pi/STM32에 배포되어 identity, READ_ONLY,
MOTION_ENABLED와 torque/PID/voltage diagnostics까지 통과했다. 그러나 승인된 첫
Shoulder `-0.08 rad / 2초` 시험에서 `status=8 detail=0`과 stop latch가 발생했다.
즉시 bridge를 종료하고 12V를 껐으며 추가 이동은 하지 않았다.

따라서 이 문서의 20E 후보는 **물리 수락 거절**이다. 후속 원인 감사와 20F 수정은
[0x00020E00 물리 거절 기록](2026-07-31-stm32-0x00020e00-rejected-heartbeat-rx.md)과
[0x00020F00 acknowledged-heartbeat 후보](2026-07-31-stm32-0x00020f00-heartbeat-ack-candidate.md)에
이어 기록한다.

