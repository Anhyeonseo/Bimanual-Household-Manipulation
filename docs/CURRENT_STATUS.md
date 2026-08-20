# 현재 상태

기준일: 2026-08-20

## 프로젝트 상태

최종 태스크와 단계별 승인 기준을 정의한 `ROADMAP_DEFINED` 단계다. 수건
전용 perception, planning 또는 실제 동작은 아직 승인되지 않았으며
`motion_authorized=false`다.

## 재사용 가능한 기반

- protocol v2 기반 STM32 12축 resident executor와 measured feedback
- 단일 serial owner를 유지하는 양팔 resident adapter
- 승인된 양팔 operational limits와 양팔 URDF/MoveIt 구성
- 상단·양 손목 카메라 수집과 phase scheduling
- intrinsic, eye-to-hand, worktable 보정 도구
- plan SHA, validate-only, one-shot 실행과 fail-closed 패턴
- 캔 OBB와 파지 기하에서 검증한 segmentation/방향/roll 기초 코드

## 수건 태스크 구현 상태

| 구성 | 현재 | 다음 승인 조건 |
|---|---|---|
| 태스크 범위와 성공 기준 | 문서화 완료 | 실제 수건 규격과 임계값 확정 |
| 수건 데이터셋 | 없음 | 대표 구김 상태 수집·분할 annotation |
| segmentation/height | 미구현 | held-out mask·높이 기준 통과 |
| 모서리·경계·평탄도 | 미구현 | 네 모서리 및 flatness 오차 검증 |
| 조작 primitive | 미구현 | 개별 plan-only와 제한 동작 검증 |
| 거친 펼치기 | 미구현 | 부분 펼침 상태 도달 반복성 확보 |
| 정밀 평탄화·정렬 | 미구현 | 네 모서리와 면적 기준 통과 |
| 첫 번째 접기 | 미구현 | 중간 직사각형 형상 검증 통과 |
| 두 번째 접기 | 미구현 | 최종 정사각형 형상 검증 통과 |
| 통합 복구 상태기계 | 미구현 | 시도 횟수와 실패 종료 보장 |

## 공통 기반 blocker

1280×960 상단 카메라 intrinsic은 독립 검증을 통과했지만 2026-08-18
eye-to-hand 후보는 거부됐다. 수건의 metric geometry와 실제 양팔 동작 전에
작업대 좌표계를 다시 독립 검증해야 한다.

또한 최종 구김 상태 인식을 RGB만으로 수행할지 상단 depth를 추가할지 결정해야
한다. RGB만 사용하는 경우 양 손목 다중 시점과 lift-and-observe 절차가
필수다.

## 바로 다음 작업

1. 실제 목표 수건의 한 변 길이, 두께, 재질과 질량을 기록한다.
2. 펼침·부분 펼침·구김 상태 데이터 수집 규약을 만든다.
3. `towel_task_contract.candidate.yaml`의 기하·복구·성공 임계값을 정의한다.
4. motion command가 없는 segmentation과 상태 분류 baseline부터 구현한다.
