# 300 mm 정사각형 수건 펼치기·2회 접기 검증 매트릭스

| Gate | 확인 대상 | 자동/실기 | 현재 |
|---|---|---|---|
| T0 | protocol manifest, firmware/host parity와 양팔 fault stop | 자동+HIL | 유지 |
| T1 | 양팔 URDF, operational limits, 300 mm proxy FK와 collision | 자동+MoveIt | PASS: 등록 URDF·limits·카메라 mount·작업대와 strict full-state collision 검사 |
| T2 | 실제 Top/left/right wrist 장치 identity, intrinsic, timestamp | 실기+오프라인 | PASS: Top 1280, right intrinsic·torque-hold eye-in-hand·tabletop translation; left W3는 URDF 기록 보존, 새 staged 검증은 metric 사용 전 필요 |
| T3 | Top-to-base, 작업대 metric 영역, clear observation pose | 실기+오프라인 | PASS: left/right 등록·작업대 영역·right shadow·clear 왕복/무가림·right tabletop 독립 staged target |
| T4 | 300 mm task contract, 물성 증빙과 episode 단위 데이터 분리 | 리뷰+자동 | 개발 595/검수 103 + 물리 재배치 held-out 38/검수 35·robot OOD 3, split leakage 0; S1 재개 전 질량·4겹 두께·작업대 마찰 최소 실측 대기 |
| T5 | 실제 mask, component, frame border와 clear-view rejection | 오프라인+실기 | HELD-OUT MASK PASS: towel 30/30·empty 5/5, IoU mean 0.980284/min 0.965564, border FN 0/FP 1; motion 비승인 |
| T6 | corner, 말린 edge, layer ambiguity, height/flatness | 오프라인+실기 | PASS: 5×3 real burst 상태 일치, non-flat/fold `ALIGNED` 0; action-context fold IoU min 1차 0.903769/2차 0.859693 |
| T7 | jaw gap, 단일/다층 grasp, slip, 장력·속도 계약 | 실기 | 좌우 1/4겹 정적 retention PASS; 자동 contact·동적 slip/장력은 미구현 |
| T8a | 1차 양팔·보정/2차 단팔 sequence의 task-pose MoveIt plan-only | 자동+MoveIt | PASS: r0g URDF/shadow/tabletop 고정, 846구간·12,552 strict 상태, 미승인 접촉 0, mesh 3.810/4 mm, TCP 2.875/4 mm, motion command 0 |
| T8b | Isaac Lab S0–S3, heuristic baseline과 visual residual/unfolding policy | 자동+Isaac+오프라인 | 최신 아래→위·오른팔 canonical S0 proxy reset·114-phase articulation·FOV·3,383표본 collision PASS. 최신 S1 surface cloth의 direct gripper-link 9-node×2 attachment와 1차 fold→place/release PASS(`snap 0.046 mm`, `lift ≥20.933 mm`, rigid-transform follow `≤0.426 mm`, release patch–jaw `≥63.974 mm`). 8-env full-shape 차이 `31.648 mm`로 결정성 미통과. Self-contact는 간격 `3.007 mm`·table clearance `1.500 mm`를 유지했지만 20초 뒤 `0.0294 m/s > 0.015`로 settle FAIL. R2 추정 `64%`; 최소 물성 실측 전 중지, 2차 fold·S2/S3 미완료 |
| T9 | 독립 primitive dry-run, supervised-once와 제한 반복 | 실기 | 미구현 |
| T10 | 평탄 수건의 1차 fold와 300×150 mm 검증 | 실기 | 미구현 |
| T11 | 2차 multi-layer fold와 150×150 mm 검증 | 실기 | 미구현 |
| T12 | learned 거친 펼치기·정규화와 slip/double-layer 중단 | 오프라인+실기 | 미구현 |
| T13 | 정밀 평탄화, 정렬과 원인별 제한 복구 | 자동+실기 | offline replay만 구현 |
| T14 | 구김부터 최종 fold까지 30회, fault injection과 soak | 실기 | 미구현 |

## 단계별 승인 수치

| 단계 | 최소 승인 기준 |
|---|---|
| 작업셀 | 300×300 mm 수건과 승인된 외곽 여유가 검증된 Top metric 영역 안에 있음 |
| 30 cm reachability | 1차 양팔 edge-pair·보정과 2차 단팔 midpoint에서 TCP xyz+jaw yaw+phase별 approach cone의 5-DOF task-pose IK, full 6D FK 기록, robot/table/camera/cable gate 통과 |
| Isaac S0/S1 | S0는 FOV·접근·충돌만 주장하고, S1 surface cloth의 drop/settle·vertex-patch attachment·lift/place/release·termination이 seed별로 재생됨 |
| 학습 환경 | observation/action/reward/termination version과 seed/material/solver SHA 고정, oracle metric으로 reward exploit 회귀시험 통과 |
| learned 펼치기 | 완전 미사용 초기 상태에서 heuristic과 같은 action budget으로 비교해 성공률을 우선 개선하고 collision·drop·workspace 이탈 0회; 동률이면 시도 횟수·시간 개선 |
| 승인 관측 | 양팔 clear pose, clear-view validity, settle·freshness·calibration identity 통과 |
| 평탄화 | 네 모서리, 변·대각선 편차 ≤ 8%, 면적 ≥ 90%, hidden-layer false positive 0회 |
| primitive 승격 | 무수건 dry-run과 supervised-once 후 10회 중 사후 조건 ≥ 9회, 안전 사고 0회 |
| 1차 fold | 양팔 moving-edge endpoint coarse fold와 최대 2회 clear-observation correction으로 standalone 20회 중 ≥ 19회, nominal 300×150 mm와 corner/fold-line/twist 기준 통과 |
| 2차 fold | 오른팔 moving-edge midpoint 오른쪽→왼쪽 fold로 standalone 20회 중 ≥ 19회, nominal 150×150 mm와 첫 fold 보존 기준 통과 |
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
- learned policy는 승인된 primitive와 제한 파라미터만 제안하며 planner,
  collision, contact, confirmation과 recovery budget을 우회하지 않는다.
- simulator checkpoint의 성능만으로 실제 실행을 승인하지 않고, heuristic과
  학습 정책은 같은 held-out episode split과 action budget에서 비교한다.
- 한 팔의 slip·tracking fault·workspace exit는 양팔 동시 정지로 이어진다.
- 복구는 실패 signature와 계약된 횟수를 함께 확인하며 같은 실패를 무한
  반복하지 않는다.
- 실제 동작 실패는 다음 실행 전에 검토 가능한 artifact를 남긴다.
