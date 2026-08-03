# Motion-8 G474 buffered 물리 실행 후보 로컬 결과

## 결론

G474에 validation-only route와 분리된 physical buffered execution route를
구현했다. 로컬 후보 identity는 `0x00022000`, capabilities는
`0x00000FFF`이며 새 bit `0x00000800`이 물리 실행 후보를 나타낸다.

ROS Action runtime은 아직 연결하지 않았다. Pi host 배포와 `0x00022000`
flash, identity, READ_ONLY, MOTION_ENABLED 및 5분 무동작 검증까지 통과했다.
이후 단일 관절 commissioning에서 frame admission과 queue prime은
통과했지만 exact apply tick을 놓치는 실패가 재현됐다. 따라서 blanket
`motion_authorized=false`와 commissioning 미통과 상태를 유지한다.

## 구현 결과

- trajectory `t=0` pose를 첫 wire sample과 interpolation anchor에 함께 사용
- servo read sweep 없이 fresh start pose를 anchor로 고정
- reviewed timing `20 ms`, lead `60..400 ms`, prime/watermark/refill
  `16/10/16` 적용
- executor는 1 ms tick, 6축 SYNC_WRITE는 5 ms와 마지막 sample에서 실행
- queue underflow·missed apply tick·cancel·connection loss·tracking fault를
  extended terminal과 physical safe-stop 경로에 연결
- BEGIN 뒤 START frame이 오지 않으면 anchor deadline에서 tracking fault로
  종료해 PRIMING 무기한 유지를 차단
- validation-only candidate는 기존처럼 무동작 route로 분리 유지
- host physical exchange는 matching one-shot 응답만 허용하고 자동 재전송 금지
- firmware terminal은 마지막 setpoint의 servo bus 적용 완료까지만 보증
- host 성공은 6축 `30 raw` 이내 진단 2회 연속 뒤에만 인정
- 단일 관절 commissioning 도구는 모든 결과에서 physical DISABLE을 실행
- 실패 경로는 SAFE_STOP latch를 보존하고 DISABLE ACK로 종료하며 reset이나
  CLEAR_FAULT를 자동 수행하지 않음
- 첫 실기 시도는 현재 Wrist Flex `-0.33134 rad`가 안전범위 밖이라 buffered
  frame 전송 0회로 차단됐다. 이후 같은 조건을 ARM/ENABLE 전에 거부하도록
  preflight 순서를 강화했다.
- 두 번째 시도는 첫 9-sample ACK가 admission에서 거부돼 frame 1회,
  실제 움직임 0회로 SAFE_STOP/DISABLE됐다. 100 ms lead에서는 첫 frame
  wire time `43.229 ms` 뒤 최소 60 ms lead를 만족할 수 없고, START까지의
  wire 하한 `87.674 ms`도 80 ms anchor를 넘는 구조적 부족이 원인이었다.
  host 초기 lead를 140 ms로 올려 admission margin 약 `36.77 ms`,
  START-anchor wire margin 약 `32.33 ms`를 확보했다.
- 140 ms lead 재시험은 `accepted=16`, `peak_queued=16`, `applied=1`,
  `terminal_reason=4(MISSED_APPLY_TICK)`, `safe_stop_required=true`로
  종료됐다. 첫 sample은 t=0 현재 자세라 Wrist Roll은 `2043 raw`에서
  변하지 않았으며 실제 움직임·비정상 소음·진동도 없었다.
- 이 결과로 wire admission과 queue 적재 문제가 아니라 executor가
  `current_tick == apply_tick`을 요구해 두 번째 20 ms tick을 놓친 것이
  원인임을 확정했다. 후속 `0x00022100` bounded-lateness 후보는 별도
  [2026-08-04 결과](2026-08-04-motion8-g474-buffered-apply-lateness.md)에
  기록한다.

## 검증 결과

- 전체 Python/ROS 회귀: `520 passed`
- STM32 actuator C: ctest `2/2 passed`
- C fault injection: queue underflow와 missed apply tick 모두
  `HOLD + safe_stop_required`
- `single_arm_bridge` symlink-install rebuild: PASS
- Cortex-M4 Release clean cross-build: PASS, compiler warning 0
- ELF text/data/bss: `35456 / 112 / 5392` bytes
- HEX SHA-256:
  `b5b1780bfdf5fdfb9b90637b26925fef58a68b150321f0d307a85ebcedef4bee`

HEX:
`artifacts/firmware/2026-08-03/stm32_g474_single_arm_0x00022000.hex`

## 다음 gate

1. commissioning host 도구를 Pi에 host-only 배포하고 rebuild
2. READ_ONLY import/readback gate
3. 별도 승인으로 Wrist Roll `+0.015 rad / 300 ms` buffered 실행 1회
4. terminal 뒤 6축 `30 raw` 이내 진단 2회 연속과 physical DISABLE 확인
5. 성공 뒤에만 ROS Action runtime 연결과 연속 Pick/Place로 확대
