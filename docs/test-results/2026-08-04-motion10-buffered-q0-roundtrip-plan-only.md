# Motion-10 buffered q0 왕복 plan-only 결과

## 결론

Motion-9 뒤 독립 READ_ONLY로 확인한 자세를 anchor로 사용해 arm 5축이 q0에
도달한 뒤 같은 anchor로 돌아오는 buffered 경로를 실제로 1회 실행했다.
물리 왕복과 복귀는 완료됐지만 전 구간에서 가시적 떨림이 있었고, host
post-settle 판정도 실패했으므로 Motion-10은 아직 통과가 아니다. 자동
재시도는 하지 않았다.

첫 계획의 희소한 9개 minimum-jerk 표본을 직선으로 연결한 것이 500 ms마다
속도 불연속을 만든다는 점을 확인했다. 수정 후보는 동일한 4초 경로를 해석적
quintic minimum-jerk 201개 점으로 직접 표현한다. 이 artifact는 실행 API를
포함하지 않고 `motion_authorized=false`를 유지한다.

## 입력 계약

- firmware: `0x00022100`
- capabilities: `0x00000FFF`
- calibration: `0x8AD27897`
- contract status: `PHYSICAL_ACTION_COMMISSIONED`
- anchor raw: `2075 / 2255 / 1785 / 1981 / 2070 / 2002`
- arm q0 raw: `2048 / 2048 / 2048 / 2048 / 2048`
- gripper: 현재 raw `2002` 보존

## 계획

- 경로: anchor → q0 → anchor
- 총 시간: `4000 ms`
- q0 도달: `2000 ms`
- waypoint: 해석적 quintic minimum-jerk 대칭 `201개`, `20 ms` 간격
- resampling: `20 ms`
- sample 수: `201`
- 최대 sample step: `0.007563 rad`
- 최대 이산 velocity: `0.378150 rad/s`
- 최대 이산 acceleration: `0.582500 rad/s²`
- 최대 이산 jerk: `2.875000 rad/s³`
- 시작·q0 양방향·종료 경계 이산 velocity: 최대 `0.000200 rad/s`
- 검증 상한: velocity `0.5 rad/s`, acceleration `1.0 rad/s²`
- q0 arm raw 오차: `0 raw`
- 최종 anchor 복귀 오차: `0 rad`

STM32의 기존 1 ms executor와 5 ms servo sync-write를 그대로 재현한 raw
출력은 총 `801개`이며 한 번의 5 ms 출력에서 arm 최대 변화는 `2 raw`다.
시작, q0와 최종 raw는 각각 계획값과 정확히 일치한다. 축별 unchanged-output
비율도 artifact에 기록해 후속 Shoulder/Elbow 추종 시험 입력으로 보존한다.

## queue 검증

- startup prime / watermark / refill: `16 / 10 / 16`
- batch 수: `33`
- batch: 첫 `9 / 7`, 이후 최대 6, 마지막 5 samples
- accepted samples: `201`
- simulation state: `input_complete`
- safe stop required: `false`
- firmware terminal 없는 성공 판정: `false`

## artifact

- 파일:
  `artifacts/motion/2026-08-04/motion10_buffered_q0_roundtrip_plan_only.json`
- SHA-256:
  `f5772313d1f3a3f9223e21e31a6fb3dd6a2219974b09734c4145275994cf8c5a`
- execution API used: `false`
- buffered frame encoded: `false`
- motion authorized: `false`

## one-shot sender 준비

- sender:
  `tools/execute_buffered_q0_roundtrip_once.py`
- sender SHA-256:
  `d01741ac46617befb1cb00dcb64d76e2c2b36b7abd2013545dd6aef000766e37`
- exact confirmation:
  `EXECUTE_MOTION10_Q0_ROUNDTRIP_ONCE`
- plan SHA·contract SHA·calibration·joint order·q0 midpoint·201개 sample을
  실행 전에 다시 계산한다.
- sparse waypoint 또는 analytic profile 필드가 다른 계획은 SHA가 일치해도
  거부한다.
- fresh-start 실패 시 Action goal을 보내지 않는다.
- Action goal 전송은 정확히 1회이며 timeout은 cancel하고 자동 재시도하지
  않는다.
- local dense plan·sender·settle 계약: `40 passed`
- ROS 환경 전체 로컬 회귀: `567 passed`
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
오류는 기존처럼 SAFE_STOP으로 fail-closed 처리한다. timeout만 늘리지는 않았다.

## 다음 gate

새 dense 계획과 position-only settle 경로의 전체 로컬 회귀·Pi 배포를 먼저
통과해야 한다. 그 뒤 Shoulder, Elbow와 다중 관절의 축별 추종을 제한 실기로
분리한 후 q0 왕복을 다시 1회만 실행한다. 자동 재시도, gripper 동작과
CLEAR_FAULT는 허용하지 않는다. dense 계획에서도 가시적 떨림이 남으면 PID를
즉시 바꾸지 않고 raw 양자화·축별 tracking/load/current 증거를 먼저 수집한다.
