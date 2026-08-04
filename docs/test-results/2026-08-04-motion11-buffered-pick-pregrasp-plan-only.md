# Motion-11 buffered Pick pregrasp 계획 전용 검증

## 결론

Motion-10에서 실기 통과한 현재 anchor→q0 dense 경로와 기존 MoveIt
충돌 검사 q0→Pick pregrasp 경로를 하나의 20 ms buffered Action 후보로
결합했다. q0에서 별도 Action이나 정착 대기를 두지 않으며, 두 leg 모두
해석적 quintic minimum-jerk로 연결한다.

이번 결과는 로컬 plan-only 후보다. 실행 API, Action goal, buffered frame과
로봇 이동은 사용하지 않았고 `motion_authorized=false`를 유지한다. Pi 배포와
fresh READ_ONLY gate, 제한 실기는 별도 승인 뒤 진행한다.

## 검증된 입력

- firmware 계약: `0x00022100`
- capabilities: `0x00000FFF`
- calibration: `0x8AD27897`
- 현재 anchor raw: `2068 / 2227 / 1728 / 1831 / 2052 / 2002`
- 보존 gripper raw: `2002`
- q0 raw: `2048 / 2048 / 2048 / 2048 / 2048 / 2002`
- Pick pregrasp raw: `2278 / 3190 / 1625 / 1209 / 2146 / 2002`
- 기존 MoveIt plan-only source:
  `artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015/01_q0_to_pick_pregrasp.json`
- source SHA-256:
  `da5f3b3fc8200cbc4713e2fcf05d5b54387929ec399377ebc68ce1722587549f`

source의 12개 segment는 모두 성공이며 최종 pregrasp를 향한 정확한
`1/12` 간격의 직선 joint path다. 따라서 현재 anchor에서 pregrasp로 직접
새 직선을 만들지 않고, 이미 실기 검증된 anchor→q0와 충돌 검사된
q0→pregrasp를 결합했다.

## dense 계획

- 경로: 현재 anchor → q0 → Pick pregrasp
- anchor→q0: `2100 ms`
- q0→pregrasp: `7000 ms`
- 총 시간: `9100 ms`
- waypoint/sample: `456개`, `20 ms` 간격
- q0 별도 정착 대기: `0 ms`
- 최대 sample step: `0.009382 rad`
- 최대 이산 velocity: `0.469100 rad/s`
- 최대 이산 acceleration: `0.645000 rad/s²`
- 최대 이산 jerk: `2.875000 rad/s³`
- q0 진입/이탈 속도: 최대 `0.000200 / 0 rad/s`
- 검증 상한: velocity `0.5 rad/s`, acceleration `1.0 rad/s²`

STM32의 1 ms executor와 5 ms servo sync-write를 재현한 출력은 `1821개`다.
5 ms 출력 한 번의 arm 최대 변화는 `2 raw`이고 시작·q0·최종 raw가 계획과
일치한다.

## queue 계약

- startup prime / watermark / refill: `16 / 10 / 16`
- batch 수: `76`
- batch 최대 크기: `9 samples`
- accepted samples: `456`
- plan input 종료 시 applied / queued: `444 / 12`
- simulation state: `input_complete`
- safe stop required: `false`
- firmware terminal 없는 성공 판정: `false`

`input_complete`는 host 입력이 모두 queue에 들어갔다는 뜻이며 실행 성공
terminal을 뜻하지 않는다. 실기에서는 extended firmware terminal과
heartbeat-gated post-settle을 별도로 통과해야 한다.

## artifact와 one-shot sender

- plan artifact:
  `artifacts/motion/2026-08-04/motion11_buffered_pick_pregrasp_plan_only.json`
- plan SHA-256:
  `d27f66ec17fbd988cc7f08ecc931b9d9b86d4454b07a3058282e1b0a78f29522`
- generator:
  `tools/plan_buffered_pick_pregrasp.py`
- generator SHA-256:
  `fc1cc471a268e021d997c3366d87f1a5d6d0f7d2ada204dfa0052a17611935d2`
- sender:
  `tools/execute_buffered_pick_pregrasp_once.py`
- sender SHA-256:
  `3c6677a5f167cb41dcfc2b9a6d18502a916959d356ad54e2c7d525edd5c6b898`
- exact confirmation: `EXECUTE_MOTION11_PICK_PREGRASP_ONCE`
- Action result timeout: `20 s`
- Action 전송: 최대 `1회`
- 자동 재시도: `0회`

sender는 plan SHA뿐 아니라 artifact 전체를 source route·calibration·buffered
계약으로 다시 계산해 정확히 일치해야만 허용한다. joint order, fresh start,
최종 pregrasp, firmware terminal lateness `0–5 ms`와 post-settle `≤30 raw`도
검증한다.

## 로컬 검증

- Motion-11 신규 테스트: `12 passed`
- 관련 buffered Action 회귀: `115 passed`
- host/ROS 전체 회귀: `572 passed`
- 저장소 루트 무제한 pytest는 로컬에 없는 Isaac Lab의
  `isaaclab_tasks` 때문에 수집되지 않으므로 host/ROS 테스트 디렉터리를
  명시했다.

## 다음 gate

1. Pi에 generator·sender·plan과 필요한 host 파일을 SHA 고정 전송한다.
2. 12V ON 전에 latch와 프로세스 상태를 확인한다.
3. READ_ONLY 6축 진단으로 실제 anchor가 계획 허용치 안인지 확인한다.
4. anchor가 벗어나면 허용치를 늘리지 않고 계획을 새 anchor로 재생성한다.
5. MOTION_ENABLED 무동작 gate 뒤 Action goal 단 1회만 제한 실행한다.
6. 성공 terminal, post-settle, 최종 pregrasp와 physical DISABLE까지 통과해야
   Motion-11을 물리 통과로 승격한다.

