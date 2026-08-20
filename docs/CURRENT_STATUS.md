# 현재 상태

기준일: 2026-08-20

최신 자동검증 결과는
[2026-08-20 수건 software foundation 검증](test-results/2026-08-20-towel-software-foundation.md)에
기록했다.

## 프로젝트 상태

최종 태스크와 단계별 승인 기준을 정의하고 하드웨어 독립 기반 구현을 시작한
`SOFTWARE_FOUNDATION` 단계다. 실제 수건 perception과 동작은 아직 승인되지
않았으며 `motion_authorized=false`다.

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
| 태스크 범위와 성공 기준 | candidate 계약 구현 | 실제 수건 규격과 임계값 확정 |
| annotation 계약 | schema, validator, deterministic split manifest 구현 | 실제 dataset index 생성 |
| 수건 데이터셋 | synthetic example만 있음 | 대표 구김 상태 수집·분할 annotation |
| segmentation/height | reviewed polygon→metric observation backend 구현 | mask inference와 component 검사 |
| 모서리·경계·평탄도 | 순수 기하와 상태 gate 구현 | 실제 mask→corner pipeline 검증 |
| temporal state | 3-frame 동일 상태 gate 구현 | timestamp/spread/hysteresis 추가 |
| task 상태기계 | 유한 recovery ledger와 offline replay 구현 | primitive outcome golden scenario 확장 |
| fold plan-only | 직교 2회 기하·arc, arm 배정 fixture selector 구현 | 실제 MoveIt reachability 연결 |
| 조작 primitive | 미구현 | 개별 plan-only와 제한 동작 검증 |
| 거친 펼치기 | 미구현 | 부분 펼침 상태 도달 반복성 확보 |
| 정밀 평탄화·정렬 | 미구현 | 네 모서리와 면적 기준 통과 |
| 첫 번째 접기 | 미구현 | 중간 직사각형 형상 검증 통과 |
| 두 번째 접기 | 미구현 | 최종 정사각형 형상 검증 통과 |
| 통합 복구 상태기계 | offline 유한 상태기계 구현 | 실제 primitive outcome 연결 |
| hardware-free CI | 계약·수건 시험·motion-lock workflow 구현 | GitHub 최초 실행 확인 |

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
3. 실제 mask 입력 backend와 component/frame-border 검사를 구현한다.
4. arm assignment cost와 fake reachability backend를 구현한다.
