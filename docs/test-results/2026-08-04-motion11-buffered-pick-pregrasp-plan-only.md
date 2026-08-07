# Motion-11 buffered Pick pregrasp 계획 전용 검증

## 결론

Motion-10에서 실기 통과한 현재 anchor→q0 dense 경로와 기존 MoveIt
충돌 검사 q0→Pick pregrasp 경로를 하나의 20 ms buffered Action 후보로
결합했다. 첫 9.1초 물리 시도는 경로를 따라 움직였지만 Shoulder와 Wrist
Flex가 계획 속도를 추종하지 못해 fail-closed ABORTED가 됐다. error trace로
측정한 약 60 raw/s를 보수적 50 raw/s 계약으로 낮춰 47초 후보를 다시
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
raw를 최종 47초 후보 anchor로 사용했다. 이전 계획을 자동 재시도하지 않았다.

## dense 계획

- 경로: 현재 anchor → q0 → Pick pregrasp
- anchor→q0: `12000 ms`
- q0→pregrasp: `35000 ms`
- 총 시간: `47000 ms`
- waypoint/sample: `2351개`, `20 ms` 간격
- q0 별도 정착 대기: `0 ms`
- 최대 sample step: `0.002316 rad`
- 최대 이산 velocity: `0.115800 rad/s`
- 최대 이산 acceleration: `0.032500 rad/s²`
- 최대 이산 jerk: `0.375000 rad/s³`
- 검증 상한: velocity `0.5 rad/s`, acceleration `1.0 rad/s²`

STM32의 1 ms executor와 5 ms servo sync-write를 재현한 출력은 `9401개`다.
5 ms 출력 한 번의 arm 최대 변화는 `1 raw`이고 시작·q0·최종 raw가 계획과
일치한다.

## 실측 추종률 계약

- 첫 시도 post-terminal Shoulder 추종률: 약 `60.0 raw/s`
- 첫 시도 post-terminal Wrist Flex 추종률: 약 `60.8 raw/s`
- 계획 검증용 보수적 rate: `50 raw/s`
- 1 ms rate-limited follower 모델 최대 peak error: `86.080 raw`
- 허용 peak error: `100 raw`
- 모델 terminal error: `0 raw`
- 허용 terminal error: `30 raw`

단순히 post-settle timeout을 늘리지 않았다. 큰 오차를 남긴 채 목표만 먼저
끝내면 servo가 뒤늦게 따라오면서 진동하기 때문이다. 계획 자체의 목표 변화율을
실측 추종률에 맞췄다.

## queue 계약

- startup prime / watermark / refill: `16 / 10 / 16`
- batch 수: `392`
- batch 최대 크기: `9 samples`
- accepted samples: `2351`
- plan input 종료 시 applied / queued: `2340 / 11`
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
  `874f2a50fc8c4bd68d97d57a358481234880bcd6d7c3c80145fc91f1656c5e46`
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

## startup re-anchor 160 ms 보강

startup-reanchor 첫 실기 시도는 경로 실행 전에 두 번째 prime frame의
heartbeat gate에서 중단됐다. precompute `263.804 ms`는 fresh re-anchor 전에
완료됐지만, 기존 initial first-sample lead `140 ms`에서 실측 firmware 경과가
`61 ms`가 되어 남은 lead가 `79 ms`였다. 안전 하한 `80 ms`보다 1 ms
작았으므로 `sequence=0`, `status=none` 상태에서 START frame을 보내지 않고
fail-closed latch를 설정했다. 자동 재시도는 수행하지 않았다.

initial first-sample lead를 `160 ms`로 올려 두 번째 7-sample prime frame이
maximum horizon `400 ms`에 들어오는 firmware 경과 창을 `60–80 ms`로
확보했다. first-sample 안전 하한 `80 ms`, prime `16 samples / 2 frames`,
maximum horizon `400 ms`, 자동 재시도 금지는 유지한다. 20 ms 모의
heartbeat에서는 최대 3회 gate를 허용하며 실측 `61 ms`, 양 끝 `60/80 ms`,
범위 밖 `59/81 ms`, uint32 tick wraparound를 모두 검증했다.

- 이전 140 ms plan SHA: `975102ee7ba1b2ec066b3bc3934c19b53a26a345fc991025491c2f04e9aedcba`
- 현재 160 ms plan SHA: `874f2a50fc8c4bd68d97d57a358481234880bcd6d7c3c80145fc91f1656c5e46`
- 현재 contract SHA: `2370d6443b082d82afd22dd7e2f16d917c10cbf722f20accae7b9b1a23291f6b`
- 첫/마지막 apply offset: `160 / 47160 ms`
- anchor·target·총 시간·sample 수·joint waypoint: 변경 없음

## 로컬 검증

- Motion-11 startup·plan·sender 관련 회귀: `86 passed`
- 전체 host/ROS 회귀: `594 passed`
- `single_arm_bridge` 패키지 테스트: `21 tests, 0 errors, 0 failures`
- 저장소 루트 무제한 pytest는 로컬에 없는 Isaac Lab의
  `isaaclab_tasks` 때문에 수집되지 않으므로 host/ROS 테스트 디렉터리를
  명시했다.

## 다음 gate

1. 160 ms host 파일과 계약·계획을 Pi에 SHA 검증 후 배포하고 rebuild한다.
2. 12V OFF·팔 지지 상태에서 latch 복구를 1회 수행한다.
3. persistent MOTION_ENABLED Bridge로 torque와 fresh anchor를 유지한다.
4. anchor가 계획 허용치를 벗어나면 허용치를 늘리지 않고 계획을 재생성한다.
5. 새 plan SHA의 Action goal을 단 1회만 제한 실행한다.
6. startup timing/accounting, firmware terminal, post-settle과 최종 pregrasp를
   모두 통과해야 Motion-11을 물리 통과로 승격한다.
