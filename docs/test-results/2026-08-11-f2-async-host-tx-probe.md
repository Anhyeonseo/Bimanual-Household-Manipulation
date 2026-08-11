# F2 — 비동기 host TX DMA: F0 왕복 probe 실기 통과

- 일자: 2026-08-11
- 대상: 왼팔 STM32G474, firmware `0x00023100`, calibration `0x2D90167E`
- 증거: [JSON](evidence/2026-08-11-f2-async-host-tx-probe.json)

## 결과

`left_wrist_roll_joint`를 `+0.03 rad` 움직였다가 원위치로 돌아오는 1200 ms
buffered Action(61 sample)을 자동 재시도 없이 1회 실행했다. terminal은
`succeeded`, apply lateness는 61 sample 모두 0 ms, post-settle 최대 오차는 6 raw,
원위치 복귀 오차는 `0.009204 rad`였다.

| F0 계측 | F0 blocking (`0x23000`) | F2 DMA (`0x23100`) |
|---|---:|---:|
| loop period 최대 | 5804 µs | **1177 µs** |
| loop work 최대 | 5803 µs | **1176 µs** |
| host TX control-path 최대 | 4691 µs | **7 µs** |
| servo sync-write 최대 | 269 µs | 269 µs |
| 최대 apply lateness | 3 ms | **0 ms** |
| lateness histogram | 52,2,5,2,0,0 | **61,0,0,0,0,0** |

## 결론

blocking `HAL_UART_Transmit`이 5 ms control loop를 5.804 ms까지 늘린다는 F0
가설이 확인됐고, F2의 bounded LPUART1 TX DMA queue가 이를 제거했다. host TX의
control-path 비용은 4.691 ms에서 7 µs로 내려갔고, 60-byte lateness profile을
refill acknowledgement에 다시 포함한 상태에서도 apply lateness는 관측되지
않았다.

`prime_frame_1_ms`는 buffered executor 시작 전의 host setup 시간(60.572 ms)으로,
F0 reset 뒤의 control loop 계측과 별개다. 이 결과는 H2 연속 leg 결합을 위한
필수 timing precondition을 통과시킨다. 다만 기존 segment plan의 endpoint
불연속성은 별도 H2 경로 검증에서 계속 차단한다.
