# 정사각형 수건 펼치기·2회 접기 검증 매트릭스

| Gate | 확인 대상 | 자동/실기 | 현재 |
|---|---|---|---|
| T0 | protocol manifest, firmware/host parity | 자동 | 유지 |
| T1 | 양팔 URDF, operational limits, FK와 충돌 모델 | 자동 | 유지 |
| T2 | 상단·손목 intrinsic과 timestamp/freshness | 실기+오프라인 | 일부 재검증 필요 |
| T3 | eye-to-hand와 작업대 metric 좌표계 | 실기+오프라인 | REJECTED/재구축 필요 |
| T4 | 수건 task contract와 데이터 분리 | 리뷰+자동 | 미구현 |
| T5 | segmentation mask와 상태 분류 | 오프라인 | 미구현 |
| T6 | corner, boundary, height/flatness 오차 | 오프라인+실기 | 미구현 |
| T7 | grasp/jaw/장력/속도 계약 | 실기 | 미구현 |
| T8 | primitive plan-only, joint/collision/SHA | 자동+MoveIt | 미구현 |
| T9 | supervised primitive와 사후 관측 | 실기 | 미구현 |
| T10 | 거친 펼치기와 정밀 평탄화 | 실기 | 미구현 |
| T11 | 첫 번째 fold와 중간 형상 검증 | 실기 | 미구현 |
| T12 | 두 번째 fold와 최종 형상 검증 | 실기 | 미구현 |
| T13 | 제한 복구와 유한 종료 | 자동+실기 | 미구현 |
| T14 | 30회 반복성, fault injection과 soak | 실기 | 미구현 |

## 단계별 핵심 수치

| 단계 | 최소 승인 기준 |
|---|---|
| 평탄화 | 네 모서리 검출, 변·대각선 편차 ≤ 8%, 관측 면적 ≥ 예상 면적의 90% |
| 1차 접기 | 대응 모서리·접힘선 오차 기준 통과, 중간 형상 twist 없음 |
| 최종 접기 | 모서리 평균 오차 ≤ 25 mm, 외곽선 IoU ≥ 0.85 |
| 전체 실행 | 30회 중 성공률 ≥ 90%, 충돌·낙하·작업대 이탈 0회 |

세부 임계값은 실제 수건 측정과 R0 데이터 분석 후 candidate contract에
고정한다. 이 문서의 수치는 최종 acceptance 상한이며 근거 없이 완화하지 않는다.

## 실행 원칙

- 선행 gate가 실패하면 이후 gate를 실행하지 않는다.
- 실제 모터 gate는 명시적 confirmation과 물리 전원 차단 수단이 필요하다.
- perception·calibration·contract·plan SHA 중 하나라도 다르면 실행을 거부한다.
- primitive 성공은 measured terminal feedback와 새 visual observation을 모두
  요구한다.
- 복구는 계약된 횟수만 허용하며 같은 실패 원인의 무한 반복을 금지한다.
- 실제 동작 실패는 다음 실행 전에 검토 가능한 artifact를 남긴다.
