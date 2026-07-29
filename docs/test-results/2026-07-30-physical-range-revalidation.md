# 왼팔 물리 가동범위 재검증과 배포

## 판정

토크를 해제하고 팔 전체를 사람이 지지한 상태에서 실제 작업 자세를 수동으로
재현했다. Shoulder와 Elbow를 기계적 풀스톱까지 밀지 않았으며, 현재 검은 펜의
pregrasp 및 grasp 위치에 실제로 도달했다.

관측값에서 64 raw의 기계 여유를 둔 범위를 host calibration, firmware와
MoveIt에 함께 적용했다. `0x00020B00`을 Pi에 배포한 뒤 identity, READ_ONLY,
MOTION_ENABLED 무동작과 Shoulder/Elbow 소각도 격리 이동까지 통과했다. 이는
전체 Pick 승인이 아니라 단계 7의 다음 제한 접근 시험을 허용하는 gate다.

## 관측 결과

- 관측 firmware: `0x00020900`
- 관측 calibration: `0xC9EAA589`
- sample: `2182`, 600초, 5 Hz
- torque: `DISABLED`
- motion command: 없음

| 관절 | 관측 raw 최소 | 관측 raw 최대 | 적용 방침 |
|---|---:|---:|---|
| BASE | 2164 | 2315 | 기존 유지 |
| SHOULDER | 2046 | 3830 | 최대 3766 적용 |
| ELBOW | 563 | 2444 | 최소 627 적용 |
| WRIST_FLEX | 955 | 2318 | 기존 유지 |
| WRIST_ROLL | 1981 | 1988 | 측정 증거 포함, 기존 1874..2219 유지 |
| GRIPPER | 2056 | 2064 | 기존 유지 |

Shoulder와 Elbow는 관측 끝값에서 각각 64 raw, 약 5.625도의 여유를 뒀다.
이는 firmware 최종 오차 허용 20 raw의 3배보다 크다. Wrist Roll 관측값은
기존 범위 안에 있으므로 범위를 좁히거나 넓히지 않았다.

## 동기화 및 plan-only 결과

- Shoulder operational raw: `1988..3766`, MoveIt `-0.092038847..2.635378994 rad`
- Elbow operational raw: `627..2258`, MoveIt `-0.322135965..2.179786700 rad`
- 배포 firmware: `0x00020B00`
- 배포 calibration hash: `0x4D62F8D5`
- HEX SHA-256: `d1a6536c1833443629ff103ecba3452820e3880ab59f02a78d845eed4a72e405`
- 전체 회귀시험: `244/244` 통과
- STM32 ARM Release build: 통과
- ROS package build: 통과

현재 물체 중심 `(0.371814352, -0.129674332, 0.0063) m`의 MoveIt
plan-only 결과는 pregrasp `184`, grasp `216` trajectory points로 모두
SUCCESS였다. Execute API와 실제 로봇 이동은 사용하지 않았다.

## 실물 배포 acceptance

- OpenOCD program/verify/reset: PASS
- HELLO identity: firmware `0x00020B00`, calibration `0x4D62F8D5`
- READ_ONLY 무동작 연결: PASS
- MOTION_ENABLED 무동작 연결: PASS
- Shoulder `+0.08 rad / 2 s`: PASS, 약 2 raw 최종 오차
- Elbow `+0.08 rad / 2 s`: PASS, 19 raw 최종 오차
- 비명령 이동과 충돌: 0회

Elbow 결과는 허용치 20 raw에 가깝다. 중력과 실제 부하를 고려해 장거리 단일
명령으로 확대하지 않고 중간 waypoint 기반 접근에서 계속 감시한다.

## 실패에서 얻은 기준

`0x00020A00` settling 후보는 최종 오차 21 raw soft-abort 뒤 STM32 stop
latch가 다시 걸려 거부했다. `0x00020900`으로 rollback해 안전 상태를 복원한
뒤 해당 로직을 포함하지 않은 `0x00020B00`을 배포했다. 상세 기록은
[0x00020A00 거부 및 rollback](2026-07-29-stm32-0x00020a00-rejected.md)에 있다.

## 다음 gate

1. 자동 Pick 대신 충돌 없는 중간 waypoint를 둔 제한 pregrasp 접근
2. 각 구간 feedback/final error/stop latch 확인
3. 별도 승인 후 grasp와 lift를 순차 검증
4. 성공 후에만 place 상태 머신과 50회 반복 시험

그 전까지 `motion_authorized=false`, `robot_target_available=false`를 유지한다.

## 증거

- [토크 해제 관측](evidence/2026-07-30-physical-range-observation.yaml)
- [범위 적용·배포 요약](evidence/2026-07-30-physical-range-review.yaml)
- [기존 범위 plan-only 실패](evidence/2026-07-30-stage7-plan-only-limited-fail.json)
- [확장 범위 plan-only 통과](evidence/2026-07-30-stage7-plan-only-expanded-pass.json)
