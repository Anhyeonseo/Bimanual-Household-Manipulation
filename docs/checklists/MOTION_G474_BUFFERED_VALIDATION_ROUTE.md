# Motion-4 G474 buffered validation runtime route

## 목적

Motion-3의 dormant buffered command route를 G474 binary dispatcher에
연결하되, multi-sample 물리 출력은 허용하지 않는다. 다음 단계에서 Pi–VCP
timing을 로봇 무동작으로 측정할 수 있는 fail-closed 경계를 만든다.

## 구현 계약

- firmware identity: `0x00021900`
- capabilities: `0x000007FF`
- bit 10(`0x00000400`): buffered validation route
- candidate frame은 `VALIDATION_ONLY|CANDIDATE`가 모두 필요
- validation-only candidate는 물리 torque가 꺼진 `SAFE_DISABLED/READ_ONLY`에서 허용
- stop latch, `FAULT`, `ESTOPPED`, 진행 중 motion에서는 상태 오류로 거부
- BEGIN/START/END와 최대 9개 sample을 공통 C route로 검사
- 성공 응답은 `status=5`인 32-byte extended status
- validation 뒤 queue와 diagnostics를 원래 상태로 완전히 복원
- route 초기화 실패 시 bit 10을 HELLO에서 제거
- 이전 `0x00021800` 또는 bit 10 없는 firmware를 새 host가 거부

## 물리 출력 격리

- candidate handler는 `Host_StartBinaryMotion`을 호출하지 않는다.
- candidate handler는 route `start/step`을 호출하지 않는다.
- candidate handler는 `Servo_SyncWritePositions`을 호출하지 않는다.
- bit 0이 없는 candidate는 실행하지 않고 거부한다.
- 기존 flag 0, `sample_count=1` single-point 경로는 그대로 유지한다.

## 범위 밖

- Pi 전송과 STM32 flash/reset
- serial/VCP 접근
- ARM·ENABLE·SETPOINT 물리 동작
- 운영 lead/prime/watermark/refill 값 채택
- ROS multi-point Action adapter

## 다음 gate

1. 12V OFF/팔 지지에서 host와 HEX를 Pi에 배포하고 전체 flash backup을 남긴다.
2. 명시적 승인 후 program/verify/reset 1회와 identity gate를 확인한다.
3. READ_ONLY와 validation-only route로 Pi–VCP timing을 1000회 이상 측정한다.
4. 측정값 검토 전에는 물리 buffered execution을 연결하지 않는다.

로컬 검증 수치는
[Motion-4 결과](../test-results/2026-08-02-motion4-g474-buffered-validation-route.md)에
기록했다.
