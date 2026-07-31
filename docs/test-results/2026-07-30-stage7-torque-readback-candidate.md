# 단계 7 torque-limit readback fail-closed 후보 0x00020D00

## 목적

`0x00020C00`은 Shoulder/Elbow torque limit을 `780/650`으로 올렸지만
Shoulder `-0.08 rad / 2 s` 격리 이동이 변경 전과 같은 `59 raw` 오차로
실패했다. 설정 함수가 register `48..49`에 torque limit을 쓴 뒤 P/D/I만
읽어 확인하므로 실제 torque limit 적용이 증명되지 않았다.

## 변경

- firmware/host identity: `0x00020D00`
- `Servo_ConfigureForTrajectory()`가 register `48..49`를 2바이트로 readback
- read 실패 또는 little-endian torque-limit 불일치 시 `HAL_ERROR`
- 상위 `Servo_ConfigureAllForTrajectory()`의 기존 rollback이 현재 축까지
  Torque Enable register `40`을 `0`으로 설정
- Shoulder/Elbow torque limit `780/650` 유지
- Shoulder/Elbow P gain `16/24` 유지
- sustained load/current stop `800/320`, 연속 2회 조건 유지
- calibration hash `0x4D62F8D5` 유지

## 로컬 검증

- 집중 Python: `38 passed, 22 skipped`
- ROS Jazzy 환경 전체 Python: `265 passed`
- actuator C core: `1/1 passed`
- ROS `single_arm_bridge`: build PASS, identity unittest PASS
- Cortex-M4 hard-float Release build: PASS
- compiled `Servo_ConfigureForTrajectory`: `Servo_ReadData` 호출 2개 확인
  (P/D/I 및 torque limit)
- image size: text `26156`, data `112`, bss `3080`, total `29348`

SHA-256:

```text
c4b564145a32994c6601355cebccc08146bfe5287741ec1423a2b5f5c5012126  HEX
c87d2de68932b7e26aa1619a54b7708bb10c95bf1a9892d203cfa6fc80557997  ELF
7b852ea1cb691fbd07d0388e2c2c626a8a83353f9807d04cda14e737a5fea02a  BIN
f825bdc4ccb8aaf722bdc1bff96ff6f17978171a880c7e83557a93a15b151dfc  single_arm_config.h
eabb631bba033fca4e1e604d6e5328fad7c1647af5c12a9b95c6fa00fab5bbac  servo_bus.c
7e7a30796eb4c1009091d4f4e63136e24a2b125853630e571c7dcd5a510af781  hardware_identity.py
```

## 미실행 gate

Pi 전송, STM32 플래시와 로봇 이동은 수행하지 않았다. 다음 단계는 기존
`0x00020C00` 512 KiB rollback backup 확인, host/HEX 전송 및 SHA 재검증,
별도 승인된 program/verify/reset이다. 배포 뒤 identity, READ_ONLY,
MOTION_ENABLED 무동작이 통과해야 torque-limit readback gate가 실제
하드웨어에서 수락된 것이다. 그 뒤에도 Shoulder가 밀리면 토크 값을 더
올리지 않고 P gain 또는 Elbow 선행 접힘을 사용하는 torque-aware 경로를
평가한다.

## 후속 물리 결과

이 문서 작성 뒤 `0x00020D00`은 Pi 전송, OpenOCD program/verify/reset,
identity, READ_ONLY와 MOTION_ENABLED 무동작을 통과했다. 따라서 torque-limit
readback gate 자체는 실제 장비에서 수락됐다. 그러나 큰 Shoulder 명령은
terminal 뒤에도 추가 정착했고 작은 약 `0.079155 rad` 명령은 fresh feedback에서도
거의 움직이지 않아, 단발 endpoint 판정과 servo telemetry 관측성 문제를 별도
근본 원인으로 확인했다. 후속 설계와 현재 미실행 gate는
[0x00020E00 근본 해결 기록](2026-07-30-stage7-shoulder-root-cause-remediation.md)을
기준으로 한다.
