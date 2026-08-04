# Motion-10 buffered q0 왕복 물리 검증 결과

## 결론

독립 READ_ONLY로 확인한 자세를 anchor로 사용해 arm 5축이 q0에 도달한 뒤
같은 anchor로 돌아오는 dense buffered Action을 실제로 1회 통과했다.
firmware terminal, heartbeat-gated post-settle, 최종 anchor, 정상 bridge 종료와
6축 physical DISABLE이 모두 통과했고 자동 재시도는 0회였다.

희소 waypoint에서 발생했던 전 구간 떨림은 20 ms 해석적 quintic
minimum-jerk로 크게 개선됐다. 사용자는 최종 왕복이 잘 움직였고 비정상
소음은 없다고 확인했다. 다만 상승 구간에 약한 흔들림이 남아 있으므로 이는
Motion-10 차단 조건이 아니라 후속 추종·동작 품질 개선 항목으로 보존한다.
`motion_authorized=false`는 계속 유지한다.

## 입력 계약

- firmware: `0x00022100`
- capabilities: `0x00000FFF`
- calibration: `0x8AD27897`
- contract status: `PHYSICAL_ACTION_COMMISSIONED`
- anchor raw: `2068 / 2227 / 1728 / 1831 / 2052 / 2002`
- arm q0 raw: `2048 / 2048 / 2048 / 2048 / 2048`
- gripper: 현재 raw `2002` 보존

## 계획

- 경로: anchor → q0 → anchor
- 총 시간: `4200 ms`
- q0 도달: `2100 ms`
- waypoint: 해석적 quintic minimum-jerk 대칭 `211개`, `20 ms` 간격
- resampling: `20 ms`
- sample 수: `211`
- 최대 sample step: `0.008765 rad`
- 최대 이산 velocity: `0.438250 rad/s`
- 최대 이산 acceleration: `0.645000 rad/s²`
- 최대 이산 jerk: `2.875000 rad/s³`
- 시작·q0 양방향·종료 경계 이산 velocity: 최대 `0.000200 rad/s`
- 검증 상한: velocity `0.5 rad/s`, acceleration `1.0 rad/s²`
- q0 arm raw 오차: `0 raw`
- 최종 anchor 복귀 오차: `0 rad`

STM32의 기존 1 ms executor와 5 ms servo sync-write를 그대로 재현한 raw
출력은 총 `841개`이며 한 번의 5 ms 출력에서 arm 최대 변화는 `2 raw`다.
시작, q0와 최종 raw는 각각 계획값과 정확히 일치한다. 축별 unchanged-output
비율도 artifact에 기록해 후속 Shoulder/Elbow 추종 시험 입력으로 보존한다.

## queue 검증

- startup prime / watermark / refill: `16 / 10 / 16`
- batch 수: `35`
- batch: 첫 `9 / 7`, 이후 최대 6, 마지막 3 samples
- accepted samples: `211`
- simulation state: `input_complete`
- safe stop required: `false`
- firmware terminal 없는 성공 판정: `false`

## artifact

- 파일:
  `artifacts/motion/2026-08-04/motion10_buffered_q0_roundtrip_plan_only.json`
- SHA-256:
  `28ec9511a1a94c020138fe6ad908300671bf60a5938a933953d3a4f155ad634d`
- execution API used: `false`
- buffered frame encoded: `false`
- motion authorized: `false`

## one-shot sender 준비

- sender:
  `tools/execute_buffered_q0_roundtrip_once.py`
- sender SHA-256:
  `d5ed785a4a5a33c5b24da05a0321870cc46dd5f7ae5a79e8f1e8536973b9b431`
- exact confirmation:
  `EXECUTE_MOTION10_Q0_ROUNDTRIP_ONCE`
- plan SHA·contract SHA·calibration·joint order·q0 midpoint·211개 sample을
  실행 전에 다시 계산한다.
- sparse waypoint 또는 analytic profile 필드가 다른 계획은 SHA가 일치해도
  거부한다.
- fresh-start 실패 시 Action goal을 보내지 않는다.
- Action goal 전송은 정확히 1회이며 timeout은 cancel하고 자동 재시도하지
  않는다.
- 최종 plan·sender·settle 계약: `45 passed`
- post-settle host 수정 뒤 전체 로컬 회귀: `559 passed`
- `single_arm_bridge` symlink-install rebuild: PASS

## 첫 물리 시도와 실패 분류

- 기존 sparse plan SHA:
  `f6048cad638eb493eba2309b7590b1579840835c904bd0b81ce8e9f8b16b049c`
- 기존 sender SHA:
  `456ad4f67340362aa7e1a0904d510332b0be9e8d8138cc6a4ac94a19afcb58a5`
- 실행 로그 SHA:
  `5760eb205f803daab6be7d0d03a391be867b6c9421414ab87675fe09063571f8`
- Action goal: `1회`, 자동 재시도 `0회`
- 사용자가 확인한 물리 결과: q0 왕복과 anchor 복귀 완료, 비정상 소음 없음,
  전 구간 가시적 떨림 있음
- firmware setpoint 적용: 성공, 최대 apply lateness `4 ms`
- host terminal: `ABORTED`
- 이유: 전체 6축 진단 반복으로 연속 정착 snapshot 2회를 제한 시간 안에
  확보하지 못함. 마지막 보고 최대 오차 `19 raw`는 허용치 `30 raw` 안이다.
- 실패 뒤 bridge 정상 종료, `NO_ROBOT_PROCESS`, 12V OFF와 팔 지지를 확인했다.

## post-settle 수정

성공 판정은 position-only `GET_STATE` 2회 연속 `≤30 raw`를 먼저 요구한다.
통과한 뒤 torque·temperature 등을 포함한 전체 6축 진단은 1회만 수행한다.
position-only timeout, full diagnostics torque OFF/position 초과와 transport
오류는 기존처럼 SAFE_STOP으로 fail-closed 처리한다. firmware terminal과 각
position-only 조회 전 heartbeat gate를 두고 timeout을 `2.5 s`로 확장했다.
실패 시에는 관측별 6축 오차, 축별 최소·최종 오차, best maximum, 관측 수와
heartbeat gate 수를 기록하며 자동 재시도는 하지 않는다.

## 최종 물리 통과

- plan SHA:
  `032bf81ef3a18cc8126fb2955388b8d0363405d7584327d56f3bbcf534aa72b5`
- 실행 로그:
  `artifacts/motion/2026-08-04/motion10_4p2_q0_roundtrip_retry2_7J0qeB.log`
- 실행 로그 SHA:
  `e487e0e718d8d83df14b336ec1fe889ae673e55a64bb5fde85d350b1291360f3`
- fresh-start 최대 오차: `0 rad`
- Action goal: `1회`, 자동 재시도: `0회`
- firmware terminal: `SUCCEEDED`
- accepted/applied: 전체 `211 samples`
- maximum apply lateness: `4 ms`
- post-settle maximum error: `20 raw`
- 최종 round-trip 최대 오차: `0.021476 rad`
- bridge 정상 종료, `NO_ROBOT_PROCESS`, 6축 physical DISABLE: PASS
- 사용자 관찰: 왕복 동작 정상, 비정상 소음 없음, 상승 구간 약한 흔들림

Motion-10의 연속 q0 왕복과 host terminal 계약은 통과다. 다음 motion gate는
동일한 dense 실행 경로를 Pick pregrasp와 전체 Pick/Place로 확장하는 것이다.

## 후속 동작 품질 개선

상승 구간의 약한 흔들림은 Pick/Place 연속경로 차단 조건으로 확대하지 않는다.
반복 실기에서 재현되는지 먼저 확인하고, 재현되면 Shoulder/Elbow 축별
tracking error·load·current·goal 변화율을 동기 수집한다. 그 증거 없이 PID나
torque limit을 바로 변경하지 않는다. 개선 후보는 구간별 시간 스케일링,
가속도/jerk 상한과 servo deadband·부하 추종의 순서로 비교한다.
