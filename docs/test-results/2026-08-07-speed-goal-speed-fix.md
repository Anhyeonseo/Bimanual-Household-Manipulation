# 속도 개선 — 진짜 병목은 토크가 아니라 Goal_Speed였다

## 무엇을 물었나

buffered leg 동작이 느렸다. 속도 손잡이는 `CONSERVATIVE_TRACKING_RATE_RAW_S`
(기본 `50 raw/s`) 하나뿐이며, `select_duration_ms`가 이 가정으로 leg 시간을
정한다. 이 값의 근거(2026-08-04 관측 `60 raw/s`)는 서보 능력치가 아니라
**post-terminal 추종률**(궤적이 끝난 뒤 뒤처진 팔이 기어서 따라잡던 속도)이라
지나치게 보수적이었다.

## 1단계: 속도 램프로 실측

`tools/run_speed_ramp_pilot.py`(q0 → pregrasp만, grasp 없음)로 단계적으로
올리며 post-settle 오차를 관찰:

| rate raw/s | duration | post-settle | 하드 게이트(30)까지 여유 |
|---|---|---|---|
| 50 | 38000 ms | 16 | 14 |
| 70 | 27000 ms | 16 | 14 |
| 90 | 21000 ms | 22 | 8 |
| 120 | — | 실패(2.5 s 시한 초과, 283 raw 미수렴) | — |

## 2단계: 토크를 올렸으나 부분적으로만 개선

SHOULDER/ELBOW 토크 상한과 안전 감시 문턱을 올렸다(펌웨어
`single_arm_config.h`, `0x00022A00 → 0x00022B00`):

| 항목 | 이전 | 이후 |
|---|---|---|
| SHOULDER 토크 상한 | 780 (78%) | 900 (90%) |
| ELBOW 토크 상한 | 650 (65%) | 800 (80%) |
| 부하 안전 감시 문턱 | 800 | 950 |

전류 감시(2.08 A, 실제 걸림 감지)는 그대로 두었다. 결과: 120 raw/s는
통과했으나(post-settle 20 raw) 250 raw/s에서 SHOULDER가 713 raw(≈62.7°)
뒤처진 채 시작해, 따라잡는 속도가 토크 변경 **전후로 동일**(~65 raw/s)했다.
즉 토크가 병목이 아니었다.

## 3단계: 원인 확정 — Feetech Goal_Speed 레지스터

STS 서보 register map(주소 46-47 = Goal_Speed, 관례 `0=무제한`)을 확인한 결과,
`Servo_ConfigureForTrajectory`가 이 자리에 설명 없이 `65`(4095 중)를 쓰고
있었다. 관측된 따라잡기 속도(~65 raw/s)가 이 값과 거의 정확히 일치했다.
buffered 스트리밍은 20 ms마다 작은 위치 증분을 계속 갱신해 이 캡이 체감상
걸리지 않았지만, 궤적 종료 후 한 번에 크게 남은 거리를 가는 catch-up 동작에는
그대로 적용되고 있었다.

`SERVO_GOAL_SPEED_RAW`로 이름 붙여 `65 → 800`으로 올렸다
(`0x00022B00 → 0x00022C00`).

## 4단계: 최종 검증

같은 leg에서 200/250/300 raw/s 전부 **post-settle 16 raw로 동일**, 하드
게이트까지 14 raw 여유(`evidence/2026-08-07-speed-ramp-post-goal-speed-fix.json`,
토크만 올린 직후 결과는 `evidence/2026-08-07-speed-ramp-post-torque-increase.json`).
`MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S`를 `200 → 300`으로 올렸다(관절 속도
하드 게이트 326 raw/s까지 여유 26 raw/s).

`run_grasp_repeatability_pilot.py`에 leg별 독립 rate 인자
(`--pregrasp-tracking-rate-raw-s`/`--grasp-tracking-rate-raw-s`/
`--q0-return-tracking-rate-raw-s`)를 추가해 전체 pick-and-place 사이클
(pregrasp/grasp/q0_return)을 300 raw/s로 2회 반복 완주했다 — 6 leg 전부
통과, STALE_TICK 없음(`evidence/2026-08-07-final-pick-place-cycle-300raw-s.json`).
`CONSERVATIVE_TRACKING_RATE_RAW_S` 기본값은 안전하게 `50`으로 유지한다 — leg마다
관절 이동량이 달라 하나의 값으로 일반화할 수 없다(q0_return에 300을 기본값으로
흘렸다가 관절 속도 하드 게이트를 넘겨 계획이 거부된 사례로 확인됨). 빠르게
갈 leg는 그때그때 명시한다.

50 raw/s 대비 이 leg의 pregrasp 구간은 `38000 ms → 7000 ms`(약 5.4배).

## 남은 것

이 pose(`a45_top_shadow`) 하나에 대해서만 세 leg 모두 300 raw/s로 확정됐다.
다른 pose/leg는 각자 따로 속도 램프가 필요하다.
