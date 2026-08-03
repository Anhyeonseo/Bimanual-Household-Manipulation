# Motion-1 연속 buffered trajectory 계약

## 범위

분할 single-point 시운전 체인을 연속 trajectory로 교체하기 전, 다중점
시간축과 queue 안전 의미를 host-only로 고정한다. Pi 전송, STM32 변경,
플래시와 실제 로봇 이동은 범위 밖이다.

## 구현 결과

- `buffered_trajectory_contract.json`: 현재 runtime이 여전히
  single-sample이며 motion authority가 없음을 고정
- `validate_buffered_trajectory`: 관절 순서, fresh start, 위치·속도·가속도,
  millisecond 시간축과 동적 필드 거부 검증
- `interpolate_buffered_trajectory`: 검증된 점 사이의 선형 위치 mock
- `BufferedSetpointQueueModel`: 원자적 batch, priming, 정상 완료, cancel,
  planned HOLD, underflow, missed tick, connection loss와 uint32 wrap 모델

## 안전 결정

- current firmware buffered support: `false`
- current Action adapter buffered support: `false`
- current maximum accepted sample count: `1`
- motion authorized: `false`
- underflow·missed tick: HOLD 후 safe-stop latch 필요
- cancel·reconnect: queue 폐기, 자동 재개·재전송 없음

## 확정된 실측 운영 입력

Pi–VCP validation-only 측정과 별도 reviewed derivation으로 아래 값을 고정했다.
물리 motion authority는 계속 `false`다.

- sample period: `20 ms`
- minimum/maximum lead: `60/400 ms`
- startup prime depth: `16`
- low watermark/refill target: `10/16`
- serial RTT worst p95/p99: `17.428593/17.533277 ms`
- host jitter worst p95: `0.062925 ms`

## 완료 확인

- [x] multi-point가 strictly increasing integer-ms 시간축을 요구한다.
- [x] zero-time 시작점과 fresh feedback 불일치를 거부한다.
- [x] 위치·속도·가속도 제한 위반을 거부한다.
- [x] batch 오류 시 queue 부분 반영이 0개다.
- [x] 정상 queue는 중간 stop 없이 마지막 sample까지 진행한다.
- [x] cancel, HOLD, underflow, missed tick과 connection loss를 구분한다.
- [x] terminal/HOLD 상태에서 자동 재개를 거부한다.
- [x] uint32 tick wrap 순서를 보존한다.
- [x] 기존 single-point 실행 경로를 변경하지 않는다.
- [x] STM32 공통 C core의 queue·선형 보간·terminal 후보를 구현했다.
- [ ] STM32 board execution path에 queue를 연결한다.
- [x] 실측 lead·watermark를 고정한다.
- [x] host-only 20 ms resampling·prime/refill·ACK/cancel 스케줄러를 구현한다.
- [ ] 제한 실기를 수행한다.

Motion-1 계약과 Motion-2 STM32 core 후보는 완료했지만 `MOT-003`의 실제
연속 실행 gate는 아직 부분 통과다.

## 로컬 검증 결과

- Python 전체 회귀: `473 passed`
- STM32 공통 C core: `1/1 passed`
- ROS package build: `single_arm_bridge` 1 package PASS
- serial 접근, Pi 전송, STM32 flash와 실제 로봇 이동: `0`
