# 단계 7 Shoulder 부하 실패와 0x00020C00 물리 수락 실패

## 실기 실패 보존

전원 주기 뒤 실제 자세에서 pregrasp로 복귀하기 위해 충돌 없는 분할 경로를
계획했다. 모든 명령은 사용자별 단 1회 승인 아래 실행했고 자동 재시도는
없었다.

첫 `0.163079 rad / 2 s` 구간은 목표를 수락했지만 Shoulder 최종 오차
`59 raw > 30 raw`로 soft-abort됐다. STM32 stop latch는 걸리지 않았다.

더 작은 최대 `0.075068 rad / 2 s` 구간도 목표를 수락했지만 최종 오차
`44 raw > 30 raw`로 soft-abort됐다. 두 번째 실행에서 Shoulder는
`2.478913 → 2.492719 rad`로 변해 목표 `2.424917 rad`와 반대 방향으로
`0.013806 rad` 밀렸다. Elbow와 Wrist Flex는 목표 근처로 이동했다.

따라서 완료 허용치를 넓히거나 같은 경로를 재시도하지 않았다. 현재 증거는
관절 범위나 MoveIt 충돌 실패보다 설치된 카메라 payload와 중력에 대한
Shoulder/Elbow torque 여유 부족을 가리킨다.

## 승인된 로컬 후보

Pi 전송, STM32 플래시와 로봇 이동 없이 다음 후보만 로컬에 구현했다.

- firmware identity: `0x00020C00`
- Shoulder torque limit: `650 → 780`
- Elbow torque limit: `550 → 650`
- Shoulder/Elbow P gain: `16 / 24` 유지
- sustained load stop: `800` 유지
- sustained current stop: `320` 유지
- limit consecutive samples: `2` 유지
- calibration hash: `0x4D62F8D5` 유지

두 torque cap이 load stop `800`보다 작아야 한다는 전처리 계약을 추가했다.

## 로컬 검증

- torque/identity 집중 Python: `59 passed`
- 전체 Python: `264 passed`
- actuator C core: `1/1 passed`
- ROS `single_arm_bridge`: build PASS, identity test PASS
- Cortex-M4 hard-float Release clean build: PASS
- image size: text `26116`, data `112`, bss `3080`, total `29308`
- HEX SHA-256:
  `dc44537b914e95e93c543e8d1631ab137fed84f64c0dfc6bbdd8f1f17ee9e984`

## 배포와 물리 수락 결과

기존 firmware의 512 KiB readback backup을 확보하고 source/HEX SHA를 다시
검증한 뒤, 사용자 단 1회 승인 아래 OpenOCD program/verify/reset을
수행했다. 다음 gate를 통과했다.

- firmware `0x00020C00`, calibration `0x4D62F8D5` identity PASS
- `STOP_LATCHED=0`
- READ_ONLY PASS
- MOTION_ENABLED 무동작 PASS

Shoulder만 현재 `2.330117 rad`에서 `2.250117 rad`로 `-0.08 rad / 2 s`
이동시키고 다른 네 arm joint를 고정했다. 실제 Shoulder는
`2.357728 rad`로 목표 반대 방향에 밀렸고 terminal `59 raw > 30 raw`로
soft-abort됐다. latch와 자동 재시도는 없었다. 즉 `650 → 780` 변경 뒤에도
이전 `59 raw` 실패가 그대로 재현됐다.

코드 검토 결과 torque-limit register `48..49`에는 값을 쓰지만 readback은
P/D/I register `21..23`에만 수행해 실제 `780/650` 적용 여부를 증명하지
못했다. 토크를 더 올리거나 허용오차를 넓히지 않고, torque-limit readback
fail-closed 후보 `0x00020D00`으로 진단 gate를 분리했다.
