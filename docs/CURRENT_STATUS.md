# 현재 상태

기준일: 2026-08-20

## 구현됨

- protocol v2 기반 12축 resident adapter와 measured feedback
- 승인된 양팔 operational limits와 양팔 URDF/MoveIt 구성
- 상단 카메라 수집, intrinsic/eye-to-hand 보정 도구
- 캔 OBB label bootstrap, dataset build, label validation
- 캔 장축과 jaw closing line의 교차각 계산
- wrist-roll operational-limit 분기 탐색과 5축 결합 해법
- 왼팔 캔 pick plan-only/validate-only/one-shot executor 골격
- jaw gap↔command 측정 도구

## 실제 동작을 막는 항목

| 항목 | 상태 | 해제 조건 |
|---|---|---|
| Top intrinsic | PASS 후보 | runtime 반영 및 동일 해상도 확인 |
| Top eye-to-hand | REJECTED | 독립 validation까지 재통과 |
| 작업대 homography | 구 보정에 묶임 | 새 eye-to-hand 기준 재구축 |
| jaw open command | 미실측 | 53 mm 캔 + yaw 오차 여유 확보 |
| contact/release residual | 미실측 | supervised probe artifact 승인 |
| 접근 기울기 한계 | 미실측 | finger-table 간섭 한계 확정 |
| 캔 실제 pick | 미승인 | 위 항목 + plan/validate gate 통과 |
| 수거함 place | 미구현 | 목적지 기하·충돌·release 검증 |

따라서 현재 정식 상태는 `PLAN_ONLY`, `motion_authorized=false`다.

## 다음 한 단계

상단 eye-to-hand 실패 원인을 해결하고 작업대 좌표계를 다시 고정한다. 그 뒤
jaw mapping을 실측해 `config/can_pick_contract.candidate.json`의 null 값을
채우고, 왼팔 캔 pick plan-only를 새 보정 SHA로 다시 생성한다.
