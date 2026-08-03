# Motion-8 G474 buffered 물리 실행 후보

## 목적

검증 전용 `0x00021900` route를 보존한 채 실제 서보 출력 route를 별도
capability로 구현하고, 검증된 `0x00022100` queue를 ROS Action의 다중점
경로와 연결해 Pi 배포와 소형 다중 관절 왕복 실기까지 확인한다. 물리
경로는 commissioned 상태지만 일반 작업 권한 `motion_authorized=false`는
유지한다.

## 고정 계약

- firmware identity: `0x00022100`, Pi host와 STM32 배포·identity gate 통과
- capabilities: `0x00000FFF`
- validation-only: bit `0x00000400`
- physical execution candidate: bit `0x00000800`
- sample period: 20 ms
- 첫 wire sample: 검증된 trajectory `t=0` pose, 현재 tick 기준 140 ms lead
- interpolation anchor: 첫 sample보다 20 ms 앞선 tick, 같은 `t=0` pose
- 허용 lead: 60..400 ms
- startup prime / watermark / refill: `16 / 10 / 16`
- executor service: 고유한 HAL 1 ms tick마다 최대 1회
- 6축 SYNC_WRITE: 5 ms 주기와 마지막 sample
- validation route apply lateness: 0 ms(exact tick 유지)
- physical route apply lateness: 0..5 ms 허용, 5 ms 초과 즉시
  `MISSED_APPLY_TICK + safe stop`
- 성공 terminal `detail`: 전체 실행 중 최대 apply lateness(ms)
- frame 재전송 금지
- firmware `SUCCEEDED`: 마지막 setpoint가 servo bus에 적용된 상태만 의미
- 일반 host 성공: 6축 position 오차가 `30 raw` 이내인 진단 2회 연속 필수
- 단일 관절 observable commissioning: 계획 변위 `16 raw` 이상,
  명령 방향 실측 이동 `10 raw` 이상, 선택 축 목표 오차 `8 raw` 이하,
  나머지 축 오차 `30 raw` 이하를 진단 2회 연속 만족해야 physical PASS
- 단일 관절 commissioning은 성공·실패 모두 6축 physical DISABLE 후 종료
- 실패 시 SAFE_STOP latch를 보존하므로 DISABLE ACK까지만 확인하고,
  성공 시에는 DISABLE 뒤 6축 torque OFF readback까지 확인
- 현재 자세와 목표의 전 축 안전범위를 ARM/ENABLE 전에 preflight
- 115200 baud의 9+7 frame·ACK·중간 heartbeat wire 하한 `87.674 ms`를
  anchor 전에 수용하고 약 `32.326 ms`의 wire margin 확보

## Fail-closed 조건

- 새 capability가 없거나 identity가 다르면 host 연결 거부
- BEGIN/continuation 순서, lead, queue, calibration, safety state 불일치 거부
- queue underflow와 apply tick 누락은 `HOLD + safe_stop_required`
- apply tick 지연은 예정 tick을 anchor로 유지한 채 최대 5 ms까지만 적용
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
- [x] Pi host/HEX 배포
- [x] 기존 `0x00021900` flash 전체 backup
- [x] OpenOCD program/verify/reset
- [x] READ_ONLY와 MOTION_ENABLED 무동작 gate
- [x] torque ON 고정자세의 5분 no-setpoint 안정성 gate
- [x] terminal outer sequence 보존과 post-settle commissioning host gate
- [x] `0x00022000` 실기에서 `accepted=16 / applied=1 / reason=4` 증거 확보
- [x] 0..5 ms bounded lateness와 6 ms 초과 fail-closed 로컬 구현
- [x] success terminal 최대 lateness host gate와 fault-injection 테스트
- [x] `0x00022100` host/HEX Pi 배포와 flash
- [x] 단일 관절 `+0.015 rad` buffered terminal과 실제 위치 변화 확인
- [x] `accepted=16 / applied=16 / maximum apply lateness=1 ms` 확인
- [x] 독립 READ_ONLY에서 Wrist Roll `2043 -> 2048 raw` 변화 재확인
- [x] 미세 복귀 명령의 기존 `30 raw` false-positive 증거 확보
- [x] observable-motion commissioning host gate와 단위 테스트 구현
- [x] observable gate Pi 배포와 `0.03 rad` 가시 변위 실기
- [x] ROS Action runtime 로컬 연결과 전체 회귀
- [x] ROS Action runtime Pi 배포·무동작 gate
- [x] 짧은 다중 관절 연속경로 실기
- [ ] 연속 Pick/Place

세부 결과는
[Motion-8 결과](../test-results/2026-08-03-motion8-g474-buffered-physical-route.md)에
기록하며, bounded lateness 실기와 observable gate는
[0x221 observable 결과](../test-results/2026-08-04-motion8-buffered-observable-motion-gate.md)에
기록한다.
ROS Action 통합은
[Motion-9 결과](../test-results/2026-08-04-motion9-buffered-action-integration.md)에
기록한다.
