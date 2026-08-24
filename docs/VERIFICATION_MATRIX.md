# 300 mm 정사각형 수건 펼치기·2회 접기 검증 매트릭스

| Gate | 확인 대상 | 자동/실기 | 현재 |
|---|---|---|---|
| T0 | protocol manifest, firmware/host parity와 양팔 fault stop | 자동+HIL | 유지 |
| T1 | 양팔 URDF, operational limits, 300 mm proxy FK와 collision | 자동+MoveIt | 실제 동시 도달성 미검증 |
| T2 | 실제 Top/left/right wrist 장치 identity, intrinsic, timestamp | 실기+오프라인 | Top config drift, right wrist 미보정 |
| T3 | Top-to-base, 작업대 metric 영역, clear observation pose | 실기+오프라인 | 1280 eye-to-hand 거부 |
| T4 | 300 mm task contract, 물성 증빙과 episode 단위 데이터 분리 | 리뷰+자동 | nominal side만 확정 |
| T5 | 실제 mask, component, frame border와 robot occlusion | 오프라인+실기 | annotation backend만 구현 |
| T6 | corner, 말린 edge, layer ambiguity, height/flatness | 오프라인+실기 | 순수 기하만 구현 |
| T7 | jaw gap, 단일/다층 grasp, slip, 장력·속도 계약 | 실기 | 미구현 |
| T8 | 300 mm fold sequence의 MoveIt plan-only와 Isaac 보조 검증 | 자동+MoveIt | 순수 arc/fake backend만 구현 |
| T9 | 독립 primitive dry-run, supervised-once와 제한 반복 | 실기 | 미구현 |
| T10 | 평탄 수건의 1차 fold와 300×150 mm 검증 | 실기 | 미구현 |
| T11 | 2차 multi-layer fold와 150×150 mm 검증 | 실기 | 미구현 |
| T12 | 거친 펼치기와 slip/double-layer 중단 | 실기 | 미구현 |
| T13 | 정밀 평탄화, 정렬과 원인별 제한 복구 | 자동+실기 | offline replay만 구현 |
| T14 | 구김부터 최종 fold까지 30회, fault injection과 soak | 실기 | 미구현 |

## 단계별 승인 수치

| 단계 | 최소 승인 기준 |
|---|---|
| 작업셀 | 300×300 mm 수건과 승인된 외곽 여유가 검증된 Top metric 영역 안에 있음 |
| 30 cm reachability | 약 300 mm 1차 moving-edge separation, 약 150 mm arc 높이와 약 150 mm 2차 separation을 실제 grasp inset으로 재계산해 전 waypoint IK/collision 통과 |
| 승인 관측 | 양팔 clear pose, robot occlusion 분리, settle·freshness·calibration identity 통과 |
| 평탄화 | 네 모서리, 변·대각선 편차 ≤ 8%, 면적 ≥ 90%, hidden-layer false positive 0회 |
| primitive 승격 | 무수건 dry-run과 supervised-once 후 10회 중 사후 조건 ≥ 9회, 안전 사고 0회 |
| 1차 fold | standalone 20회 중 ≥ 19회, nominal 300×150 mm와 corner/fold-line/twist 기준 통과 |
| 2차 fold | standalone 20회 중 ≥ 19회, nominal 150×150 mm와 첫 fold 보존 기준 통과 |
| 최종 형상 | 모서리 평균 오차 ≤ 25 mm, 외곽선 IoU ≥ 0.85, 접힘선 평균 오차 ≤ 20 mm |
| 전체 실행 | 서로 다른 초기 구김 30회 중 ≥ 27회 성공, 충돌·낙하·workspace 이탈 0회 |

세부 perception과 hardware 임계값은 실제 수건·카메라·접촉 데이터로
확정한다. 최종 acceptance 수치는 근거 없이 완화하지 않는다.

## 실행 원칙

- 선행 gate가 실패하면 이후 gate를 실행하지 않는다.
- 실제 모터 gate는 명시적 confirmation과 물리 전원 차단 수단이 필요하다.
- perception, calibration, contract, URDF 또는 plan SHA가 다르면 실행을 거부한다.
- 작업대 homography는 들린 수건의 3D 점에 적용하지 않는다.
- 가려진 corner는 visual observation으로 승격하지 않는다.
- primitive 성공은 measured terminal feedback와 양팔 퇴피 후 새 clear
  observation을 모두 요구한다.
- 한 팔의 slip·tracking fault·workspace exit는 양팔 동시 정지로 이어진다.
- 복구는 실패 signature와 계약된 횟수를 함께 확인하며 같은 실패를 무한
  반복하지 않는다.
- 실제 동작 실패는 다음 실행 전에 검토 가능한 artifact를 남긴다.
