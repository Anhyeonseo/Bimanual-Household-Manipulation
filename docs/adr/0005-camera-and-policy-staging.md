# ADR-0005: 카메라와 정책 구현 순서

- 상태: 채택
- 날짜: 2026-07-12

## 결정

카메라 관리는 초기 Raspberry Pi 성능 검증 단계에서 구현하지만, policy 학습은 결과를 반복해서 재현할 수 있는 Pick and Place를 완성한 뒤 진행한다.

순서:

1. 카메라 3대의 영상 수집과 최신 frame scheduler
2. 임시 영상 소비기(dummy consumer)를 이용한 Pi/USB 성능 측정
3. Top 카메라를 이용한 평면 인식(perception)
4. 현재 정상인 왼팔의 재현 가능한 Pick and Place
5. 손목(Wrist) 카메라 Visual Servo
6. 양팔의 재현 가능한 기준 동작(baseline)
7. Isaac Lab의 구조화 상태(structured-state) policy
8. 실제 명령을 내리지 않고 결과만 비교하는 shadow mode와 크기가 제한된 보정값(bounded residual)

원본 영상을 직접 입력받는 policy가 안전을 담당하는 하위 제어 계층을 우회하지 않게 한다.

2026-07-24에 단일 팔 side를 왼팔로 확정했다. 구현 순서는 유지하며 side
선정 근거는 [ADR-0008](0008-left-arm-first.md)에 기록한다.

## 2026-08-01 구체화

왼팔 감독형 Pick/Place 시운전이 완료되고 오른팔이 정상 복구되었다. policy는
데스크탑에서 충분히 학습한 뒤 ONNX로 내보내 Pi 5에서 추론한다. 따라서
구현 순서는 왼팔 생산 기준선, Pi 3카메라·policy 자원 기준선, 오른팔 단독
동등성, 양팔 통합과 policy 권한 확대 순으로 구체화한다. 상세 역할과 gate는
[ADR-0012](0012-arm-integration-and-pi-policy-deployment.md)를 따른다.
