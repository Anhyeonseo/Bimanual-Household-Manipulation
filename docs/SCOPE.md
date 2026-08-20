# 프로젝트 범위

## 포함

- 책상 위 다물체 인식과 scene inventory
- 물체 종류별 목적지 규칙(수거함, 정리 구역, 보류 구역)
- 양팔 도달성·충돌·작업 순서 계획
- 캔 파지/운반/배치를 첫 번째 end-to-end 태스크로 구현
- 배치 후 재인식과 실패 상태 보고
- 실제 동작 전 plan-only, validate-only, supervised-once 승격 절차

## 제외

- 펜 연속 동작, 펜 전달, 과거 단일 팔 데모
- 범용 trajectory backend를 이용한 실제 task motion
- 학습되지 않았거나 검증되지 않은 policy의 실제 모터 제어
- 사람이 승인하지 않은 자동 실제 동작 재시도

제외된 이전 데모와 검증 이력은
[동결 저장소](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)에만 보존한다.

## 완료 정의

책상 정리 v1은 고정된 작업대에서 지원 물체를 모두 목록화하고, 정해진
목적지로 옮긴 뒤, 재인식 결과가 목표 scene과 일치할 때 완료다. 충돌과
비명령 동작은 0회여야 하고, 각 실패는 안전 정지와 재현 가능한 진단
artifact를 남겨야 한다.
