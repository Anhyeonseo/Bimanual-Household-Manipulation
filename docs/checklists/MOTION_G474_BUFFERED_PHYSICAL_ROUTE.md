# Motion-8 G474 buffered 물리 실행 후보

## 목적

검증 전용 `0x00021900` route를 보존한 채 실제 서보 출력 route를 별도
capability로 구현한다. ROS Action과 실기 권한은 아직 연결하지 않는다.

## 고정 계약

- firmware identity: `0x00022000`
- capabilities: `0x00000FFF`
- validation-only: bit `0x00000400`
- physical execution candidate: bit `0x00000800`
- sample period: 20 ms
- 첫 wire sample: 검증된 trajectory `t=0` pose, 현재 tick 기준 100 ms lead
- interpolation anchor: 첫 sample보다 20 ms 앞선 tick, 같은 `t=0` pose
- 허용 lead: 60..400 ms
- startup prime / watermark / refill: `16 / 10 / 16`
- executor service: 고유한 HAL 1 ms tick마다 최대 1회
- 6축 SYNC_WRITE: 5 ms 주기와 마지막 sample
- frame 재전송 금지

## Fail-closed 조건

- 새 capability가 없거나 identity가 다르면 host 연결 거부
- BEGIN/continuation 순서, lead, queue, calibration, safety state 불일치 거부
- queue underflow와 apply tick 누락은 `HOLD + safe_stop_required`
- cancel, connection loss, tracking/sync-write fault는 extended terminal로 종료
- 분리된 BEGIN 뒤 START가 유실되면 anchor deadline에서 latch/terminal 종료
- SAFE_STOP은 physical HOLD/latch를 먼저 요청한 뒤 terminal 보고
- validation-only route는 계속 torque OFF 무동작으로 분리

## 완료 확인

- [x] physical route와 validation-only route 분리
- [x] fresh `t=0` anchor에 별도 servo read sweep 없음
- [x] 1 ms executor / 5 ms output service 연결
- [x] underflow·missed tick·cancel·connection loss·tracking fault terminal 연결
- [x] START frame 유실 시 PRIMING 무기한 유지 차단
- [x] host one-shot physical exchange와 capability gate 연결
- [x] C fault test와 전체 Python 회귀 통과
- [x] Cortex-M4 Release clean cross-build와 HEX 생성
- [ ] Pi host/HEX 배포
- [ ] 기존 `0x00021900` flash 전체 backup
- [ ] OpenOCD program/verify/reset
- [ ] READ_ONLY와 MOTION_ENABLED 무동작 gate
- [ ] torque ON 고정자세의 no-setpoint 안정성 gate
- [ ] 단일 관절 최소 변위 buffered 실기
- [ ] ROS Action runtime 연결과 연속 Pick/Place

세부 결과는
[Motion-8 결과](../test-results/2026-08-03-motion8-g474-buffered-physical-route.md)에
기록한다.
