# Motion-11 buffered Pick pregrasp

## 목적

실기 검증된 anchor→q0 dense 경로와 MoveIt 충돌 검사 q0→Pick pregrasp 경로를
하나의 20 ms buffered Action 후보로 결합하고, 단일 Action·자동 재시도 0회로
물리 통과시킨다. 일반 작업 권한 `motion_authorized=false`는 유지한다.

## 고정 계약

- firmware `0x00022500`, capabilities `0x00000FFF`, calibration `0x8AD27897`
- sample period `20 ms`, 총 `47000 ms` / `2351 samples`
- anchor→q0 `12000 ms`, q0→pregrasp `35000 ms`, q0 정착 대기 `0 ms`
- 궤적 profile: two-leg quintic minimum jerk `10t^3-15t^4+6t^5`
- 보수적 추종 rate `50 raw/s` (실측 약 `60 raw/s`)
- 허용 modeled peak error `100 raw`, terminal error `30 raw`
- initial first-sample lead `160 ms`, 안전 하한 `80 ms`, 최대 horizon `400 ms`
- startup prime / watermark / refill `16 / 10 / 16`, batch 최대 `9 samples`
- apply lateness `0..5 ms`, 초과 시 `MISSED_APPLY_TICK + safe stop`
- post-settle `≤30 raw`, 연속 2회 진단
- pregrasp 도달 허용치 `0.050 / 0.055 / 0.050 / 0.050 / 0.050 rad`
- Action 전송 `1회`, 자동 재시도 `0회`
- anchor 는 torque 유지 상태에서만 캡처한다

## Fail-closed 조건

- plan SHA, source route SHA, calibration SHA, contract SHA 중 하나라도 불일치하면 거부
- firmware candidate 가 `deployed` 아니면 거부
- fresh-start 오차가 허용치를 넘으면 거부
- anchor 표본 간 raw 변동이 허용치를 넘으면 torque 미유지로 보고 거부
- startup 잔여 lead 가 `80 ms` 미만이면 START frame 을 보내지 않고 latch
- firmware terminal 이 없거나 형식이 다르면 성공으로 인정하지 않는다
- 실패해도 자동 재시도하지 않는다. anchor 재캡처와 계획 재생성이 필요하다

## 실패 이력

- **1차 (9.1초)** — 경로는 따라갔으나 Shoulder/Wrist Flex 추종 부족으로
  post-settle `ABORTED`, 최종 오차 `545 / 286 raw`. 실측 추종률을 계약에 반영해
  47초 계획으로 재설계.
- **2차 (startup re-anchor)** — precompute `263.804 ms`가 `140 ms` lead를
  `79 ms`로 깎아 `80 ms` 하한에 걸려 START 미전송, fail-closed latch.
  lead를 `160 ms`로 상향.
- **3차 (lead160)** — `PLAN_GATE=PASS` 직후 `joint state timed out`.
  servo UART 정지. → 0x00022500 전원 도메인 수명주기 수정으로 해결.

## 완료 확인

- [x] 실기 검증된 anchor→q0 와 충돌 검사 q0→pregrasp 결합
- [x] 실측 추종률 기반 보수 계약 `50 raw/s` 적용
- [x] startup lead `160 ms` 상향과 `60–80 ms` firmware 경과 창 확보
- [x] servo UART 전원 도메인 수명주기 `0x00022500` 배포
- [x] torque 유지 anchor 캡처 도구와 spread 게이트 구현
- [x] plan-only 게이트 — peak `79.99 raw`, terminal `0.00 raw`
- [x] queue 시뮬레이션 underflow `0`
- [x] fresh-start 오차 `0.000000 rad`
- [x] Action `1회` 전송, 자동 재시도 `0회`
- [x] firmware terminal `state=succeeded`
- [x] apply lateness `5 ms` (허용 `0..5`)
- [x] post-settle `18 raw` (허용 `0..30`)
- [x] pregrasp 도달 `0.027950 rad` (허용 `0.050`)
- [x] terminal 형식 드리프트 수정과 양방향 소스 파싱 시험

### 후속 (A3 전)

- [ ] apply lateness 분포 계측 — 이번 실행이 상한 `5 ms`에 닿았고
      실행 중 heartbeat 경고 1회가 있었다. 최대값만으로는 특정 구간에
      몰리는지 전 구간에 퍼지는지 알 수 없다
- [ ] startup lead 여유 재검토 — `92 ms`로 하한 `80` 대비 `12 ms`뿐

세부 결과는
[Motion-11 물리 통과](../test-results/2026-08-06-motion11-buffered-pick-pregrasp.md)에
기록한다. 선행 조건인 servo UART 수정은
[0x00022500 물리 검증](../test-results/2026-08-06-stm32-0x00022500-servo-uart-power-domain-lifecycle.md)에
있다.
