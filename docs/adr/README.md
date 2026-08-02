# 아키텍처 결정 기록(ADR)

| ADR | 결정 | 상태 |
|---|---|---|
| [0001](0001-system-partition.md) | Pi와 STM32 역할 분리 | 채택 |
| [0002](0002-motion-time-ownership.md) | trajectory 시간축 소유권 | 채택 |
| [0003](0003-five-dof-task-scope.md) | 5DOF 초기 작업 제약 | 채택 |
| [0004](0004-safety-and-arming.md) | `STANDBY`, `ARMING`, fault 원칙 | 채택 |
| [0005](0005-camera-and-policy-staging.md) | 카메라와 policy 구현 순서 | 채택 |
| [0006](0006-wire-protocol.md) | Pi–STM32 통신 규격 | 채택 |
| [0007](0007-camera-compute-budget.md) | Pi 카메라와 연산 자원 한도 | 제안 |
| [0008](0008-left-arm-first.md) | 왼팔 단일 팔 기준선 우선 | 채택 |
| [0009](0009-stm32-action-adapter-first.md) | 기존 bridge의 STM32 Action 확장 우선 | 채택 |
| [0010](0010-single-backend-entrypoint.md) | 하나의 bringup에서 backend 독점 선택 | 채택 |
| [0011](0011-top-eye-to-hand-gridboard.md) | TCP GridBoard 기반 Top eye-to-hand 등록 | 채택 |
| [0012](0012-arm-integration-and-pi-policy-deployment.md) | 단일 팔 우선 통합과 Pi 정책 배포 | 채택 |
| [0013](0013-buffered-trajectory-execution-contract.md) | 왼팔 buffered trajectory 실행 계약 | 채택 |

기존 결정을 바꿔야 할 때는 문서를 삭제하지 않는다. 기존 문서를 `대체됨(Superseded)`으로 표시하고 새 ADR을 추가해 변경 이유를 남긴다.
