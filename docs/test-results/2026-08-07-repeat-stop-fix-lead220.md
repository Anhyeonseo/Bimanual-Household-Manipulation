# 반복 실행 시 STALE_TICK 정지 — 원인과 수정

## 무엇을 물었나

buffered leg를 반복 실행하면 몇 회차 뒤 `ACTUATOR_QUEUE_STALE_TICK`으로 멈추는
현상이 있었다. 전날 가설은 "leg를 반복할수록 `reanchor` 재계산 비용이 쌓여
lead 여유가 깎인다"였다.

## 결과: "반복 누적" 가설은 실기로 반증됐다

`buffered_action_execution.start_goal`에 `reanchor_ms`/`prime_frame_1_ms`
계측을 넣고 Pi 실기로 34 leg를 이어서 측정했다.

| 시도 | 상황 | 실패 leg | reanchor_ms |
|---|---|---|---|
| v1 (재빌드 직후) | leg 5(grasp)에서 STALE_TICK | 실패 | 52.151 |
| v2 (clear_fault만) | leg 5(grasp)에서 다시 STALE_TICK | 실패 | 52.151 |
| v3 (재시작 없이 이어서) | leg 11~34, 8/8 완주 | 전부 통과 | 0.5~5.9 |

증거: `evidence/2026-08-07-repeat-stop-diagnosis-34-leg.json`
(`tools/aggregate_leg_telemetry.py`로 세 번의 pilot 호출을 세션 순서로 병합).

`reanchor` 비용은 반복할수록 오히려 감소했다(`TAG_SLOPE[grasp] reanchor=-0.692/leg`).
정지는 재시작 직후에만 몰려 있었다.

## 진짜 메커니즘

펌웨어 `setpoint_queue.c`: `lead = apply_tick - current_tick`이 `60 ms` 미만이면
거부한다. 평소 `prime_frame_1_ms`(첫 배치 왕복) 만으로 `160 ms` 예산 중
~50 ms가 고정 소모되어 남는 여유가 100~110 ms뿐이었다. `reanchor`가 튀면
(관측된 최대 52 ms) 여유가 60 ms 밑으로 떨어져 거부됐다.

## 수정

1. `INITIAL_FIRST_SAMPLE_LEAD_MS` `160 → 220`
   (`buffered_action_adapter.py`/계약 JSON/`buffered_trajectory.py`).
2. **부수 발견**: 실기 heartbeat 왕복이 시험 가정(20 ms)보다 훨씬 빨라서,
   재시도 횟수(`STARTUP_MAXIMUM_HEARTBEAT_GATES`)만 늘려서는 필요한 대기
   시간(`STARTUP_PRIME_MINIMUM_ELAPSED_MS`, 이제 120 ms)을 못 채웠다.
   `buffered_action_execution.py`에 host-side 명시적 `sleep`을 추가해
   첫 재시도가 이미 목표 구간 근처에서 시작하도록 했다. 재시도 상한은
   지터 흡수용 안전판으로 `3 → 8`.

## 실기 검증

`tools/run_grasp_repeatability_pilot.py`로 10회(leg 30개, pregrasp/grasp/
q0_return) 반복: **STALE_TICK 0건, 전부 완주**.
`first_sample_lead_ms` 93~94 ms로 안정, 반복해도 감소 추세 없음.

증거: `evidence/2026-08-07-a5-lead220-verify.json`.

파지 자체는 10/10 헛닫힘이었으나 잔여 간격(5 raw)이 "물체 없음" 신호와
일치해 이 진단과는 무관하다 — 전날 고정해 둔 pen 위치를 그대로 재사용한
탓으로 보인다.
