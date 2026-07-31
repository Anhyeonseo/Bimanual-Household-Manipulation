# 단계 7 분할 grasp plan-only

## 목적

물리 pregrasp PASS 뒤 전체 grasp를 한 번에 실행하지 않고, 마지막 실제
pregrasp 관절 상태에서 저장된 grasp 해까지의 경로를 작은 관절 구간으로
분할해 MoveIt 충돌 계획만 검증했다.

## 안전 조건

- MoveIt backend: `mock`
- RViz: 비활성
- 실제 로봇 bridge: 미사용
- `execution_api_used=false`
- `motion_authorized=false`
- `robot_target_available=false`
- 최대 관절 간격: `0.18 rad`

## 결과

- 상태: `GRASP_SEGMENT_PLAN_ONLY_PASS`
- 구간 수: 2
- 각 구간 최대 관절 변화: `0.157416849 rad`
- 1번: MoveIt error code `1`, trajectory 28 points
- 2번: MoveIt error code `1`, trajectory 27 points
- 실행 API 호출: 0회
- 전체 Python 회귀: `262 passed`

계획 증거:
[2026-07-30-stage7-grasp-segments-plan-only.json](evidence/2026-07-30-stage7-grasp-segments-plan-only.json)

SHA-256:
`b782ef0315cc2be7213084ffc5301f8ebccb49315b898f09977aeb75e116b37c`

## 다음 gate

전원 재인가 뒤 fresh `/joint_states`가 계획 시작점과 `0.05 rad` 이내인지
확인한다. 실제 current-to-target가 `0.18 rad` 이내일 때만 grasp 1번 구간을
2초 동안 단 1회 실행한다. 자동 재시도, 2번 구간, gripper close, lift는
별도 승인 전까지 금지한다.
