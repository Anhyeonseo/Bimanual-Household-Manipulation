# ADR-0012: 단일 팔 우선 통합과 Pi 정책 배포

- 상태: 채택
- 날짜: 2026-08-01

## 상황

왼팔은 STM32 0x00021800과 Shoulder P32/Elbow P28 조합으로 감독형
Pick/Lift/Place/q0 복귀를 1회 완주했다. 정식 50회 반복 기준은 아직 남아 있다.
오른팔은 사용자가 정상 작동한다고 확인했지만 저장소의 왼팔과 같은
calibration·모델·MoveIt·실기 동등성 gate를 아직 통과하지 않았다.

세 UVC 카메라는 Raspberry Pi 5에 연결되어 있고 phase scheduler와 제어 격리
기준선이 존재한다. 향후 정책은 데스크탑의 Isaac Sim/Isaac Lab에서 충분히
학습한 뒤 ONNX로 내보내 Pi에서 실제 추론할 계획이다.

## 결정

### 팔 통합 순서

1. 왼팔의 연속 trajectory, perception, contact offset, visual correction과
   50회 benchmark를 먼저 완성한다.
2. 오른팔은 왼팔 software contract를 재사용하되 calibration과 물리 상수를
   추정하거나 복사하지 않고 단독 gate를 처음부터 통과한다.
3. 두 팔의 단독 기준선 이후에 coordinated stop, 개별 영역 병렬 실행,
   공유 영역 통합 계획 순서로 양팔을 활성화한다.

### 정책 배포 경계

- Isaac 학습·평가와 무거운 simulation은 데스크탑이 담당한다.
- Raspberry Pi 5는 검증된 policy.onnx와 versioned deployment manifest만
  받아 실제 관측으로 inference한다.
- 구조화 상태 policy를 우선하지만, 이미 학습된 policy가 이미지 tensor를
  요구하면 학습과 동일한 전처리·카메라 순서·시간 계약을 보존한다.
- policy action은 command arbiter, limit, freshness, collision과 deadline
  검사를 거친다.
- policy는 STM32, servo bus 또는 raw setpoint 안전 경계를 직접 우회하지 않는다.

### 동작 계산 경계

- MoveIt은 q0↔pregrasp, transfer, retreat처럼 전역 경로와 충돌 검사가 필요한
  구간을 담당한다.
- policy와 visual servo는 grasp/place 근처의 bounded joint 또는 Cartesian
  residual을 담당한다.
- Pi의 trajectory layer는 승인된 waypoint를 연속 시간축으로 바꾸고 STM32는
  해당 시간축과 하위 안전 정지를 소유한다.
- policy 주기마다 전체 MoveIt planning을 반복하지 않는다.

### 카메라·연산 경계

- Top 카메라는 전역 탐색과 결과 확인, 손목 카메라는 각 팔의 최종 상대 정렬을
  담당한다.
- 세 카메라는 모두 연결하지만 학습된 observation contract가 요구하지 않는
  decode·추론은 phase scheduler가 줄인다.
- 카메라, perception과 policy process는 STM32 bridge에서 분리한다.
- 오래된 frame, camera skew, inference deadline miss 또는 manifest mismatch가
  있으면 action을 폐기하고 motion을 차단한다.
- 시연 재현성은 카메라 각도·높이, 작업대–base transform과 물체 Z를 고정한
  상태에서 배경·조명·반사 변화에 견디는 성능으로 정의한다.
- mount 또는 물체 높이가 바뀌면 기존 calibration을 재사용하지 않고 별도
  재보정 gate를 거친다.

## 이유

왼팔에서 이미 얻은 실패 원인, 안전 gate와 실기 기준을 먼저 생산형 계약으로
완성하면 오른팔과 양팔에서 같은 문제를 반복할 가능성이 줄어든다. 정책 학습과
실시간 inference를 분리하면 Pi는 simulation을 실행하지 않고 제어에 필요한
연산만 수행할 수 있다. MoveIt, policy와 STM32의 역할을 분리하면 전역 충돌
안전, 국소 적응성과 하위 실시간 안전을 동시에 유지할 수 있다.

## 채택 gate

- 실제 ONNX 입력·출력·control_dt manifest 확정
- Pi 5 실제 모델 inference p50/p95/max와 3카메라 동시 부하 측정
- 부하 중 STM32 heartbeat/feedback 오류 0회
- 왼팔 50회 기준선 통과
- 오른팔 독립 calibration과 단독 실기 통과
- 한 팔 fault의 양팔 coordinated stop 통과
- policy shadow mode와 bounded-action supervisor 통과

## 영향

- [ADR-0005](0005-camera-and-policy-staging.md)의 정책 통합 순서는 이 결정으로
  구체화한다.
- [ADR-0007](0007-camera-compute-budget.md)의 고정 추론율은 실제 policy contract와
  Pi 측정값으로 다시 확정한다.
- [ADR-0008](0008-left-arm-first.md)의 왼팔 우선 결정은 유지하며, 정상 복구된
  오른팔을 다음 단독 동등성 대상으로 추가한다.
