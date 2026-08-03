# Motion-8 G474 buffered apply-lateness 보정 로컬 결과

## 결론

`0x00022000` 단일 관절 commissioning에서 첫 t=0 sample만 적용되고
두 번째 20 ms tick을 놓친 원인을 보정한 `0x00022100` 로컬 후보를
구현했다. Pi 전송·STM32 flash·reset·실기 이동은 수행하지 않았다.

## 물리 실패 증거

- host/wire admission: `accepted=16`, `peak_queued=16`
- firmware progress: `applied=1`
- terminal: executor `HOLD`, reason `MISSED_APPLY_TICK(4)`
- safety: `safe_stop_required=true`, physical DISABLE ACK PASS
- Wrist Roll: 실행 전·사후 모두 `2043 raw`, 목표 `2053 raw`
- 실제 움직임·비정상 소음·진동 없음

첫 sample은 현재 자세와 같은 t=0 pose이므로 `applied=1`에서 위치가
변하지 않은 것은 terminal accounting과 일치한다.

## 0x00022100 변경

- validation-only route는 maximum apply lateness `0 ms` 유지
- physical execution route만 maximum apply lateness `5 ms` 적용
- `current_tick - scheduled_tick`이 `0..5 ms`이면 sample 적용
- sample anchor는 실제 지연 시각이 아니라 예정 tick을 유지
- 5 ms 초과는 기존 `MISSED_APPLY_TICK + safe stop` 유지
- 진단에 last/maximum apply lateness 누적
- 성공 terminal `detail`에 maximum apply lateness(ms) 기록
- host는 성공 detail `0..5`만 허용하고 초과 성공 보고를 거부
- firmware identity `0x00022100`, capabilities `0x00000FFF`

## 로컬 검증

- 0 ms exact 적용 회귀
- 1 ms와 경계 5 ms 지연 적용
- 6 ms 지연 fail-closed
- uint32 tick wraparound에서 3 ms 지연 적용
- accepted/applied/queued/peak queue accounting
- host success terminal 5 ms 허용·6 ms 거부
- STM32 actuator C ctest `2/2 passed`
- Python/ROS 전체 회귀 `526 passed`
- Cortex-M4 Release clean cross-build PASS, warning 0
- ELF text/data/bss: `35528 / 112 / 5416` bytes
- 배포 후보 HEX: `artifacts/firmware/2026-08-04/stm32_g474_single_arm_0x00022100.hex`
- HEX SHA-256: `a1683e6a4906d5356b4d8b095915cb1257222648c4d59a89498c926821718e20`

## 상태

- local candidate: PASS
- Pi host/HEX deploy: 미수행
- STM32 flash/reset: 미수행
- physical commissioning: 미수행
- ROS Action runtime: 미연결
- motion authorization: false

다음 gate는 HEX와 host 파일의 SHA를 고정한 뒤, 별도 승인으로 Pi host
백업·전송·rebuild를 수행하는 것이다.
