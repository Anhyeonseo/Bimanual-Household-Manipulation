# 책상 정리 로드맵

## R0 — 저장소 범위 정리 (완료)

- 과거 펜·단일팔·연속 동작 자료를 동결 저장소로 분리
- 캔 WIP의 과거 태스크 모듈 의존성 제거
- 문서와 시험 범위를 책상 정리 기준으로 재작성

## R1 — 좌표계 복구

- 1280×960 intrinsic runtime 반영
- Top eye-to-hand 실패 원인 수정
- 독립 validation 통과
- 작업대 plane/homography와 OBB metric 오차 재검증

완료 조건: 새 calibration SHA에서 `motion_authorized=true` 후보를 만들고,
독립 물리 target 오차가 파지 허용치 안에 든다.

## R2 — 캔 파지

- jaw gap↔command와 hysteresis 측정
- 접촉/release residual 및 finger-table 간섭 한계 측정
- 왼팔 캔 plan-only와 validate-only 갱신
- supervised pick 1회, 연속 반복 시험

완료 조건: 충돌·낙하·비명령 동작 0회, 정의된 반복 성공률 통과.

## R3 — 캔 수거함 배치

- 수거함 collision geometry와 release pose
- 캔을 든 transit/retreat 계획
- release 확인과 배치 후 재인식

완료 조건: 한 개의 캔을 인식부터 수거함 확인까지 end-to-end로 처리.

## R4 — 책상 scene inventory

- 다물체 관측 병합과 stable object ID
- 물체별 목적지 규칙과 보류 상태
- 처리 순서, occlusion, 재스캔

완료 조건: 지원 물체 목록과 목표 scene diff가 일관되게 계산됨.

## R5 — 양팔 정리 scheduler

- 팔별 reachability map과 inter-arm collision
- 독립 영역 병렬 처리, 공유 영역 직렬화
- 한 팔 fault 시 coordinated stop과 재계획

완료 조건: 충돌 0회와 deterministic task ordering.

## R6 — 운영화

- headless bringup, systemd, udev, 로그/진단 bundle
- 30분·8시간·24시간 soak
- 비상 정지·복구 runbook과 최종 benchmark
