# Motion-2 STM32 buffered trajectory queue 후보

## 범위

Motion-1에서 동결한 시간축과 fail-closed 의미를 STM32 공통 C core로
옮긴다. G474 보드 빌드는 소스를 컴파일하지만 binary command route,
firmware identity와 capability는 변경하지 않는다. Pi 전송, 플래시, reset,
CLEAR_FAULT와 실제 로봇 이동은 범위 밖이다.

## 구현 결과

- `actuator_setpoint_queue_peek`: 보간 중 다음 sample을 소비하지 않고 조회
- `actuator_buffered_executor`: PRIMING/RUNNING/HOLD/SUCCEEDED/CANCELED/
  ABORTED 상태기
- 검증된 현재 anchor와 다음 apply tick 사이의 6축 선형 µrad 보간
- 실행 중 원자적 refill과 명시적 input-complete
- planned HOLD, operator cancel, queue underflow, missed apply tick,
  connection loss와 tracking error를 서로 다른 terminal reason으로 기록
- accepted/applied/queued/peak depth, last apply tick, terminal tick,
  safe-stop 필요 여부 진단
- uint32 half-range ordering과 tick wrap 보존

## Fail-closed 경계

- 현재 runtime firmware: `0x00021800`
- 현재 capability: `0x000003FF`
- 현재 실제 수락 sample 수: `1`
- buffered binary command route: `false`
- buffered capability 광고: `false`
- timing parameter 실측 완료: `false`
- terminal/HOLD 뒤 init 없는 자동 재개: 금지
- batch 검증 실패 시 부분 queue 반영: `0`

minimum/maximum lead, startup prime depth, low watermark와 refill target은
이번 코드에 운영 기본값으로 넣지 않았다. API 인자로만 주입하며 host latency
측정 뒤 별도 이슈에서 고정한다.

## 로컬 검증

- STM32 공통 C core: C11 `-Wall -Wextra -Wpedantic -Werror`, ctest PASS
- fault injection: underflow, missed tick, cancel, connection loss,
  tracking error와 자동 재개 거부 PASS
- 정상 경로: prime, refill, multi-segment interpolation, completion PASS
- tick 경계: uint32 wrap interpolation PASS
- Cortex-M4 Release cross-build: PASS, text/data/bss `31560/112/4176` bytes
- command route가 없어 linker GC 뒤 HEX SHA는 수락된 0x218과 동일한
  `4b9ca7c7b3927ce798048258fb1b3deecfb0718d660c6c1bd93308862ef3f317`
- Python machine contract: candidate가 command route/capability를 광고하면
  fail-closed 거부
- serial 접근, Pi 전송, STM32 flash와 실제 로봇 이동: `0`

## 완료 확인

- [x] batch admission이 원자적이다.
- [x] 현재 위치 anchor부터 첫 sample 및 sample 사이를 선형 보간한다.
- [x] 실행 중 refill과 input-complete 정상 완료를 구분한다.
- [x] underflow와 missed tick은 queue를 폐기하고 safe-stop 필요 HOLD로 간다.
- [x] planned HOLD는 latched cancel과 구분된다.
- [x] cancel·connection loss·tracking error는 자동 재개할 수 없다.
- [x] uint32 wrap에서 보간과 apply 순서가 유지된다.
- [x] G474 Release cross-build가 새 core source를 경고 없이 컴파일한다.
- [x] 0x218 identity·0x3FF capability·single-sample runtime을 보존한다.
- [ ] binary command route와 terminal response를 연결한다.
- [ ] lead·prime·watermark·refill 값을 host 측정으로 고정한다.
- [ ] ROS multi-point Action adapter를 연결한다.
- [ ] 제한 실기를 수행한다.

Motion-2 이후 Motion-3에서 dormant route·확장 terminal codec과 timing 분석
도구까지 추가했지만 실제 route 연결, Pi–VCP timing, host adapter와 제한 실기는
남아 있어 `MOT-003` 전체는 계속 부분 통과다.
