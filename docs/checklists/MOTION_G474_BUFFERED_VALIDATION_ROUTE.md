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

## 완료된 실측 gate

1. [x] host/HEX Pi 배포, 기존 flash 전체 backup, program/verify/reset
2. [x] `0x00021900` identity·`0x000007FF` capability 확인
3. [x] READ_ONLY validation-only Pi–VCP timing 각 1000회 측정
4. [x] 60/400 ms lead와 16/10/16 queue 운영 입력 검토
5. [ ] 별도 gate에서 물리 buffered execution과 ROS Action 연결

실측은 `tools/capture_buffered_validation_timing.py`가 담당한다. 이 도구는
HELLO·HEARTBEAT와 `VALIDATION_ONLY|CANDIDATE` frame만 사용하며, 모든 응답에서
`status=5`, `SAFE_DISABLED`, queue/accepted/applied sample 0을 확인한다.
`CLOCK_MONOTONIC_RAW` 기준 serial RTT, 예정 dispatch 대비 host jitter, apply
deadline 대비 response lateness를 각각 1000개 이상 저장한다. 세 차례 계획된
20/40/80 ms host pause도 로봇 무동작 상태에서만 기록한다. 측정 결과는 운영
lead·prime·watermark 값을 자동 승인하지 않는다.

로컬 검증 수치는
[Motion-4 결과](../test-results/2026-08-02-motion4-g474-buffered-validation-route.md)에
기록했다.
