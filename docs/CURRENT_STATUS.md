# 현재 상태

기준일: 2026-08-23

최신 기록된 자동검증 결과는
[2026-08-20 수건 software foundation 검증](test-results/2026-08-20-towel-software-foundation.md)이다.
이후 nominal 수건 크기와 실행 로드맵을 300 mm 기준으로 갱신했으며 현재
회귀시험 결과는 repository test로 확인한다.

## 프로젝트 상태

최종 목표 수건은 nominal 300×300 mm로 확정됐다. 실제 side tolerance, 두께,
질량, 재질과 접촉·마찰 한계는 아직 측정되지 않았다. 프로젝트는 계속
`SOFTWARE_FOUNDATION` 단계이며 실제 수건 motion은 승인되지 않았고
`motion_authorized=false`다.

## 재사용 가능한 기반

- protocol v2 기반 STM32 12축 resident executor와 measured feedback
- 단일 serial owner를 유지하는 양팔 resident adapter
- 승인된 양팔 operational limits와 양팔 URDF/MoveIt 구성
- 상단·양 손목 카메라 수집과 phase scheduling
- intrinsic, eye-to-hand, worktable 보정 도구
- plan SHA, validate-only, one-shot 실행과 fail-closed 패턴
- 수건 annotation/schema, 순수 기하, 상태 gate와 offline replay
- 300 mm square의 직교 2회 fold geometry와 fake reachability fixture

## 수건 태스크 구현 상태

| 구성 | 현재 | 다음 승인 조건 |
|---|---|---|
| 태스크 범위 | nominal 300×300 mm, 최종 150×150 mm | 나머지 물성·허용편차 실측 |
| annotation 계약 | schema, validator, deterministic split manifest | 실제 episode index 생성 |
| 수건 데이터셋 | synthetic example만 있음 | 구김·평탄·1차/2차 fold 실제 데이터 |
| segmentation | reviewed polygon→metric observation backend | 실제 mask, component, border 검사 |
| corner/topology | 순수 기하와 confidence gate | 가림·말림·다층 ambiguity 검증 |
| temporal state | 3-frame 동일 상태 gate | timestamp, spread, settle, hysteresis |
| observation lifecycle | 설계에 clear/retreat/reobserve 계약 반영 | camera phase와 runtime 구현 |
| fold plan-only | 300 mm 직교 기하·arc와 fake selector | 실제 MoveIt IK/collision adapter |
| Isaac | 표시 전용 workcell | S0 rigid proxy부터 물리 layer 구축 |
| 조작 primitive | 미구현 | 개별 plan-only→supervised 제한 반복 |
| 펼쳐진 수건 2회 접기 | 미구현 | R4 standalone fold gate 통과 |
| 펼치기·평탄화 | 미구현 | R5/R6 단계 성공 기준 통과 |
| 통합 복구 | offline 유한 상태기계 | 실제 실패 signature와 feedback 연결 |
| hardware-free CI | 수건 계약·기하·replay workflow 구현 | 갱신된 300 mm fixture 회귀 확인 |

## 현재 blocker

1. active 640×480 worktable calibration의 검증 span은 약
   290.176×392.858 mm다. 한 축이 수건 한 변 300 mm보다 작아서 외곽 여유는
   물론 수건 전체도 검증 영역 안에 넣었다고 승인할 수 없다.
2. 1280×960 Top intrinsic은 독립 검증을 통과했지만 2026-08-18 eye-to-hand는
   거부됐고 해당 해상도의 worktable plane/homography는 재구축되지 않았다.
3. runtime camera config의 장치 경로·640×480 조건과 최신 실물
   `/hcd.0`·1280×960 기록이 일치하지 않는다.
4. left wrist는 gripper가 영상 일부를 영구 가리며, right wrist는 intrinsic,
   eye-in-hand와 URDF optical frame가 없다.
5. 1차 fold의 약 300 mm moving-edge separation과 두 fold의 약 150 mm arc
   높이를 양팔이 동시에 추종할 수 있는지 실제 MoveIt으로 검증되지 않았다.
6. jaw gap, 단일/다층 접촉, slip, 장력, 테이블 마찰과 수건 물성이 미측정이다.

## 바로 다음 작업

1. 수건 side tolerance, 두께, 질량, 재질과 상태 조건을 측정해 기존 task
   contract에 증빙과 함께 반영한다.
2. 실제 Top 장치 경로·해상도를 하나로 고정하고 camera config, intrinsic,
   Top-to-base와 worktable calibration을 같은 조건으로 재구축한다.
3. 300 mm 수건과 필요한 외곽 여유가 모두 들어가는 metric FOV 및 양팔
   `OBSERVE_CLEAR` 자세를 검증한다.
4. right wrist intrinsic/eye-in-hand/optical frame를 완성한다.
5. 300 mm rigid proxy로 x/y축·양 방향·팔 배정의 MoveIt plan-only go/no-go를
   수행한다.
6. 그 결과가 통과한 작업대 배치에서 실제 수건 데이터 수집과 mask backend를
   시작한다.
