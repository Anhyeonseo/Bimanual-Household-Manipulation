# Motion-11 buffered Pick pregrasp 계획 전용 검증

## 결론

Motion-10에서 실기 통과한 현재 anchor→q0 dense 경로와 기존 MoveIt
충돌 검사 q0→Pick pregrasp 경로를 하나의 20 ms buffered Action 후보로
결합했다. 첫 9.1초 물리 시도는 경로를 따라 움직였지만 Shoulder와 Wrist
Flex가 계획 속도를 추종하지 못해 fail-closed ABORTED가 됐다. error trace로
측정한 약 60 raw/s를 보수적 50 raw/s 계약으로 낮춰 43초 후보를 다시
생성했다. q0 별도 Action이나 정착 대기는 추가하지 않는다.

이번 결과는 로컬 plan-only 후보다. 실행 API, Action goal, buffered frame과
로봇 이동은 사용하지 않았고 `motion_authorized=false`를 유지한다. Pi 배포와
fresh READ_ONLY gate, 제한 실기는 별도 승인 뒤 진행한다.

## 검증된 입력

- firmware 계약: `0x00022100`
- capabilities: `0x00000FFF`
- calibration: `0x8AD27897`
- torque-held fresh anchor raw: `2273 / 2330 / 1802 / 1941 / 2142 / 2002`
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

최초 계획 anchor `2068 / 2227 / 1728 / 1831 / 2052 / 2002`는 실기 전
READ_ONLY gate에서 Shoulder·Elbow·Wrist Flex가 허용치를 벗어나 실행 전에
거부됐다. torque OFF 상태에서 수동 안전 자세로 복귀한 뒤 첫 시도를
수행했으며, 실패 복구 뒤에는 Bridge와 torque를 유지한 상태에서 측정한 위
raw를 최종 43초 후보 anchor로 사용했다. 이전 계획을 자동 재시도하지 않았다.

## dense 계획

- 경로: 현재 anchor → q0 → Pick pregrasp
- anchor→q0: `8000 ms`
- q0→pregrasp: `35000 ms`
- 총 시간: `43000 ms`
- waypoint/sample: `2151개`, `20 ms` 간격
- q0 별도 정착 대기: `0 ms`
- 최대 sample step: `0.002028 rad`
- 최대 이산 velocity: `0.101400 rad/s`
- 최대 이산 acceleration: `0.042500 rad/s²`
- 최대 이산 jerk: `0.375000 rad/s³`
- 검증 상한: velocity `0.5 rad/s`, acceleration `1.0 rad/s²`

STM32의 1 ms executor와 5 ms servo sync-write를 재현한 출력은 `8601개`다.
5 ms 출력 한 번의 arm 최대 변화는 `1 raw`이고 시작·q0·최종 raw가 계획과
일치한다.

## 실측 추종률 계약

- 첫 시도 post-terminal Shoulder 추종률: 약 `60.0 raw/s`
- 첫 시도 post-terminal Wrist Flex 추종률: 약 `60.8 raw/s`
- 계획 검증용 보수적 rate: `50 raw/s`
- 1 ms rate-limited follower 모델 최대 peak error: `79.987 raw`
- 허용 peak error: `100 raw`
- 모델 terminal error: `0 raw`
- 허용 terminal error: `30 raw`

단순히 post-settle timeout을 늘리지 않았다. 큰 오차를 남긴 채 목표만 먼저
끝내면 servo가 뒤늦게 따라오면서 진동하기 때문이다. 계획 자체의 목표 변화율을
실측 추종률에 맞췄다.

## queue 계약

- startup prime / watermark / refill: `16 / 10 / 16`
- batch 수: `358`
- batch 최대 크기: `9 samples`
- accepted samples: `2151`
- plan input 종료 시 applied / queued: `2136 / 15`
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
  `975102ee7ba1b2ec066b3bc3934c19b53a26a345fc991025491c2f04e9aedcba`
- generator:
  `tools/plan_buffered_pick_pregrasp.py`
- generator SHA-256:
  `bcb12ab80ed1a0b18bc865c4cb70809862a9e208820fdb628d796ebcfec6524e`
- sender:
  `tools/execute_buffered_pick_pregrasp_once.py`
- sender SHA-256:
  `545dd1614c9b424dfc792bf7e9164c09eac0b5291128e8b628a1cc6e3b3a1938`
- exact confirmation: `EXECUTE_MOTION11_PICK_PREGRASP_ONCE`
- Action result timeout: `60 s`
- Action 전송: 최대 `1회`
- 자동 재시도: `0회`

sender는 plan SHA뿐 아니라 artifact 전체를 source route·calibration·buffered
계약으로 다시 계산해 정확히 일치해야만 허용한다. joint order, fresh start,
최종 pregrasp, firmware terminal lateness `0–5 ms`와 post-settle `≤30 raw`도
검증한다.

## 첫 물리 시도와 실패 분류

- 실행 plan SHA:
  `3d12608b726afac587b6d93af65ca4bdd072f0f09299dd74bec2793ba316ec4a`
- 실행 sender SHA:
  `3c6677a5f167cb41dcfc2b9a6d18502a916959d356ad54e2c7d525edd5c6b898`
- 실행 로그:
  `artifacts/motion/2026-08-04/motion11_pick_pregrasp_WVEM0H.log`
- 실행 로그 SHA:
  `1082a69c858f52e938b68b56568da3fbb81f614d168bad2d62b9369d5d9ab298`
- fresh-start 최대 오차: `0.003068 rad`
- Action goal: `1회`, 자동 재시도: `0회`
- 사용자 관찰: 경로 이동 정상, 진동은 허용 가능하지만 개선 필요
- host terminal: `ABORTED`
- 마지막 Shoulder / Wrist Flex 오차: `545 / 286 raw`
- best maximum error: `545 raw`
- 관측: `14회`, heartbeat gate: `15회`, elapsed: `2515 ms`
- latch: `1`, diagnostics fail-closed

이는 통신·queue 실패가 아니라 계획 속도 대비 물리 추종 부족이다. 첫 시도는
Motion-11 물리 통과로 세지 않으며 같은 계획을 자동 재시도하지 않았다.

## 로컬 검증

- Motion-11 신규 테스트: `13 passed`
- 관련 buffered Action 회귀: `116 passed`
- host/ROS 전체 회귀: `573 passed`
- 저장소 루트 무제한 pytest는 로컬에 없는 Isaac Lab의
  `isaaclab_tasks` 때문에 수집되지 않으므로 host/ROS 테스트 디렉터리를
  명시했다.

## 다음 gate

1. faulted bridge 종료와 12V OFF 상태에서 latch 복구를 1회 수행한다.
2. READ_ONLY 6축 진단으로 실제 anchor가 계획 허용치 안인지 확인한다.
3. 현재 위치가 바뀌었으므로 43초 후보를 fresh anchor로 재생성한다.
4. anchor가 벗어나면 허용치를 늘리지 않고 계획을 새 anchor로 재생성한다.
5. MOTION_ENABLED 무동작 gate 뒤 Action goal 단 1회만 제한 실행한다.
6. 성공 terminal, post-settle, 최종 pregrasp와 physical DISABLE까지 통과해야
   Motion-11을 물리 통과로 승격한다.
