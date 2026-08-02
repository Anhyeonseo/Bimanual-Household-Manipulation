# Motion-3 G474 buffered command route·timing 계약

## 범위

Motion-2의 공통 C executor 앞에 다중 `SETPOINT_BATCH`를 해석하는 후보
라우트를 추가하고, 기존 16바이트 `SETPOINT_STATUS`와 호환되는 32바이트
확장 진단을 정의한다. 실제 `binary_control.c`, firmware identity, capability와
ROS Action은 변경하지 않는다. Pi 전송, 플래시, reset, CLEAR_FAULT, serial
접근과 로봇 이동은 범위 밖이다.

## 후보 명령 계약

- frame flag bit 0: validation-only
- bit 1: dormant buffered candidate 식별자(필수)
- bit 2/3/4: BEGIN/START/END
- payload: 기존 v1 header와 최대 9개 sample을 그대로 사용
- 첫 batch는 BEGIN, 실행 요청은 START, 마지막 입력은 END로 명시
- batch 전체 검증 뒤 한 번에 admission하며 일부 sample 반영 금지
- HOLD·cancel·connection loss·tracking error 뒤 자동 재개 금지

확장 상태의 첫 16바이트는 기존 status와 동일하다. 뒤 16바이트에는 executor
state, terminal reason, safe-stop 필요 여부, 마지막 queue 결과, 현재/최대 queue
깊이와 accepted/applied sample 수를 넣는다. Host parser는 두 길이를 모두 읽는다.

## 타이밍 gate

`analyze_buffered_timing.py`는 저장된 JSON만 읽으며 ROS, serial과 로봇을 열지
않는다. synthetic fault injection은 운영값을 승인할 수 없다. 실제 측정 입력은
Pi–VCP provenance, buffered capability, `CLOCK_MONOTONIC_RAW`, transport 오류 0,
각 series 1000개 이상을 모두 요구한다. 조건을 만족해도 lead, prime depth,
watermark와 refill target은 자동 채택하지 않고 별도 검토로 고정한다.

## 현재 결과

- 공통 C route decode/admission/start/terminal fault injection: PASS
- legacy/extended host codec와 잘못된 flag·길이·tick 거부: PASS
- synthetic timing analyzer: `HOST_ONLY_NOT_DEPLOYABLE`
- operational timing values authorized: `false`
- 현재 runtime: firmware `0x00021800`, capability `0x000003FF`, sample count `1`
- 실제 binary route·Action·motion authority: 변경 없음

## 남은 gate

- [ ] 별도 firmware 이슈에서 `binary_control.c` 후보 route 연결
- [ ] identity·capability 변경과 이전 host fail-closed 검증
- [ ] Pi–VCP 실제 timing 장기 측정 및 운영값 검토
- [ ] ROS multi-point Action adapter
- [ ] mock/plan-only 후 제한 실기

검증 수치는 [Motion-3 host-only 결과](../test-results/2026-08-02-motion3-buffered-command-route-timing.md)에
기록했다.
