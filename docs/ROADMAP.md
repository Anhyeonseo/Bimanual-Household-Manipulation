# 구겨진 정사각형 수건 펼치기·2회 접기 로드맵

로드맵의 최종 완료 조건은 [프로젝트 범위](SCOPE.md), 상세 상태와 primitive는
[수건 접기 설계](TOWEL_FOLDING.md), 단계별 승격은
[검증 매트릭스](VERIFICATION_MATRIX.md)를 따른다.

## R0 — 태스크 계약과 데이터 규약 (진행 중)

- 목표 수건의 한 변, 두께, 질량, 재질과 허용 편차 기록
- 시작 workspace, 카메라 시야, fold 방향과 최종 형상 고정
- grasp force/jaw gap, 장력 proxy, 속도와 recovery 한계 정의
- `CRUMPLED`부터 `FOLD_2_COMPLETE`까지 상태 annotation 규약 작성
- 대표 구김·부분 펼침·평탄·접힘 데이터 수집

현재 candidate task contract, annotation schema, validator, deterministic
manifest와 synthetic observation은 구현됐다. 실제 수건 규격과 실제 dataset
수집·분할은 남아 있다.

완료 조건: candidate task contract와 train/validation/test 분리 데이터셋이
재현 가능하고, 모든 실제 동작 임계값의 provenance가 기록된다.

## R1 — 수건 관측과 상태 추정 (순수 기하 기반 착수)

- segmentation mask와 경계 추출
- 높이·주름·겹침 feature 또는 RGB 다중 시점 대체 절차
- 노출 모서리와 grasp 후보 신뢰도
- 예상 전체 면적, 변·대각선, 평탄도와 작업대 축 회전 추정
- motion command가 없는 상태기계 입력 artifact

현재 reviewed polygon annotation→workcell observation backend, 네 corner의
변·대각선·축 정렬 metric, 단일 상태 분류와 3-frame 안정화 gate가 구현됐다.
실제 image mask inference backend는 남아 있다.

완료 조건: held-out 구김 상태에서 mask, corner, flatness와 상태 분류가
검증 임계값을 통과하고 불확실한 입력을 fail-closed한다.

## R2 — 안전 조작 primitive

- single/dual corner grasp
- lift-and-observe와 lay-flat
- tension spread와 제한된 controlled shake
- corner drag와 square alignment
- fold edge pair와 release/smooth
- 각 primitive의 plan-only, collision, 장력·속도·시간 gate

완료 조건: 수건 없는 dry-run과 supervised 단일 primitive 시험에서 충돌,
비명령 동작, session 재사용 없이 사전·사후 조건을 판정한다.

## R3 — 거친 펼치기

- 노출 지점 또는 모서리 하나를 들어 중력으로 겹침 완화
- lift 상태의 실루엣과 반대쪽 grasp 후보 재관측
- 양팔 장력 펼치기와 필요한 경우 제한된 작은 털기
- 장력을 유지한 lay-flat과 결과 재관측

완료 조건: 다양한 구김 입력이 정해진 시도 횟수 안에 `PARTIALLY_OPEN` 또는
`TWO_CORNERS_VISIBLE` 상태로 승격되고, 실패는 안전하게 종료된다.

## R4 — 정밀 평탄화와 정렬

- 겹치거나 말린 모서리 탐지
- 모서리별 외곽 corner drag
- 네 변·두 대각선·예상 면적 기반 평탄도 검증
- 작업대 축에 맞춘 정사각형 정렬

완료 조건: 네 모서리 검출, 변·대각선 편차 8% 이하, 예상 면적 대비 관측
면적 90% 이상을 만족하는 `ALIGNED` 상태를 만든다.

## R5 — 첫 번째 반 접기

- 한쪽 변의 두 모서리 동시 grasp
- 중심선을 지나는 동기 fold arc
- 반대쪽 모서리와 정렬하며 lay-down/release
- 중간 직사각형, 접힘선, layer twist 검증

현재 중심선, moving corner/target, expected footprint와 synchronized semicircle
arc는 plan-only로 생성된다. arm assignment, 도달성, 충돌과 실행은 미검증이다.

완료 조건: 첫 번째 fold가 모서리·접힘선 오차 기준을 통과하고 실패한 중간
형상에서는 두 번째 fold가 실행되지 않는다.

## R6 — 직교 방향 두 번째 반 접기

- 1차 접기 후 외곽선과 layer grasp point 재추정
- 여러 겹을 함께 잡는 접촉 검증
- 첫 접힘선을 보존하는 직교 fold arc
- 최종 정사각형 release, smoothing과 결과 검증

완료 조건: 최종 corner 오차, 외곽선 IoU와 두 접힘선 기준을 모두 통과한다.

## R7 — 통합 task manager와 제한 복구

- 관측→primitive→재관측의 전체 상태기계
- 모서리 재탐색 최대 3회
- lift-and-unfold 최대 2회
- corner drag 모서리당 최대 2회
- fold placement 보정 단계당 최대 1회
- 동일 원인 반복, stale 관측, fault의 즉시 종료

현재 observation replay용 유한 상태기계, recovery ledger와 terminal artifact가
구현됐다. 실제 primitive outcome과 measured feedback 연결은 남아 있다.

완료 조건: 모든 경로가 유한하게 `COMPLETE` 또는 `FAILED`로 끝나며 자동
복구가 confirmation, attempt counter와 artifact를 남긴다.

## R8 — 반복성·운영 승인

- 서로 다른 초기 구김 상태 최소 30회 benchmark
- 전체 성공률 90% 이상
- 장시간 camera/resident soak와 fault injection
- headless bringup, 로그 bundle, 비상 정지·복구 runbook

완료 조건: [완료 정의](SCOPE.md#완료-정의)의 품질·안전 기준을 모두 통과하고
실패 사례가 데이터셋과 회귀 시험에 반영된다.
