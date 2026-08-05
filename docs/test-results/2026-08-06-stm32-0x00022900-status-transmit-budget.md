# 0x00022900 — 버퍼드 응답 프레임의 전송시간이 apply lateness 예산이다

- 날짜: 2026-08-06
- 펌웨어: `0x00022900` (이전 배포 `0x00022800`)
- 상태: **빌드·회귀만.** 실기 검증 전이며 계약은 `deployed: false`
- 관련: [0x00022600 apply lateness 계측](2026-08-06-stm32-0x00022600-apply-lateness-instrumentation.md)

## 무엇이 일어났는가

2026-08-06 04:05 q0 복귀가 첫 sample에서 중단됐다.

```
state=hold reason=missed_apply_tick
precompute_ms=153.601 fresh_tick=31767 prime_tick=31837
first_sample_lead_ms=90 prime_heartbeat_gates=1 prime_frames=2
accepted=16 applied=0 queued=16
```

직전 회차와 **실패 지점이 다르다.** 그 전 다섯 번은 heartbeat가 응답을 못 받고
예산을 태웠고, 그건 host 수신 경로의 프레임 조각 폐기가 원인이었다(별도 수정).
이번에는 응답이 정상 도착해 prime을 통과했고, MCU가 **첫 setpoint를 적용하지
못했다.**

`applied=0`이므로 `actuator_buffered_executor_step`의
`apply_lateness > maximum_apply_lateness_ticks` 분기다. 즉 executor가 tick
`31927`에서 처음 stepping됐을 때 이미 5 ms 넘게 늦어 있었다.

## 원인

`Host_SendBinaryFrame`은 blocking `HAL_UART_Transmit`이다. 그것을 호출하는
협조적 루프가 곧 executor를 stepping하는 루프다
(`SingleArmApp_Process` → burst 처리 → `BinaryControl_Service` →
`Host_ServiceBufferedExecution`). **응답 프레임을 보내는 동안 executor는 멈춘다.**

따라서 응답 길이는 부수효과가 아니라 lateness에 직접 청구된다.

| | payload | 전선(COBS+CRC+구분자) | 115200 8N1 |
|---|---|---|---|
| `0x00022500` acknowledgement | 32 B | 54 B | **4.688 ms** |
| `0x00022800` acknowledgement | 60 B | 82 B | **7.118 ms** |

허용치는 `HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS = 5`.

`0x00022800`이 apply lateness histogram(버킷 6개 + 최악 sample index = 28 B)을
**모든** setpoint status에 실으면서 acknowledgement가 예산을 넘겼다.
**lateness를 측정하려고 만든 계측이 lateness를 만들었다.**

### 부수적으로 설명된 것

Motion-11이 관측한 max apply lateness가 **정확히 5 ms**였던 것은 우연이 아니다.
`ceil(4.688) = 5`. 그 숫자는 추종 오차가 아니라 응답 프레임의 전송시간이었고,
당시에도 예산의 94%가 이미 소모되고 있었다.

## 수정

`actuator_buffered_status_encode`에 `include_lateness`를 추가하고,
`binary_control.c`가 terminal 프레임(`status_code == 6`)에만 참을 넘긴다.
terminal은 실행이 이미 멈춘 뒤 나가므로 기다리는 apply tick이 없다.
host 파서는 16/32/60 세 길이를 이미 전부 받으므로 host 변경은 없다.

## 재발 방지

계측만으로는 부족하다. 이 제약은 지금까지 소스 어디에도 적혀 있지 않았다.

`binary_control.c`에 컴파일 타임 가드를 넣었다.

```c
#if HOST_BINARY_FRAME_TRANSMIT_MS(ACTUATOR_BUFFERED_STATUS_EXTENDED_SIZE) > \
    HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS
#error "Buffered acknowledgement transmit exceeds the apply lateness allowance"
#endif
```

**음성 검증:** payload를 32 → 36 B로 늘리자 Cortex-M4 Release 빌드가
`error: #error "Buffered acknowledgement transmit exceeds..."`로 거부했다.
되돌린 뒤 정상 빌드(`text 39672 / data 112 / bss 5936`).

`tests/test_stm32_status_frame_transmit_budget.py`(9건)가 이를 보강한다.
전선 길이는 공식이 아니라 **저장소의 실제 인코더**로 만들어 대조하고,
`main.c`의 `hlpuart1.Init.BaudRate`가 계산에 쓰인 baud와 같은지 확인한다.
`(status_code == HOST_BUFFERED_STATUS_TERMINAL)`을 `true`로 되돌리면
해당 시험이 실패함을 음성 검증했다.

## 남은 여유 — 숨기지 않는다

```
5 ms - 4.688 ms = 0.312 ms
```

**전선 기준 4바이트 미만이다.** acknowledgement payload가 32에서 36으로만
늘어도 빌드가 거부된다. 이건 여유가 아니라 벼랑이다.

근본 해결은 host frame 송신을 비동기(DMA + 유한 큐, 넘치면 fail-closed)로
돌려 프레임 길이가 lateness에 청구되지 않게 하는 것이다.
`docs/CURRENT_STATE_AND_NEXT_ROADMAP.md` D절에 양팔 진입 전 필수 항목으로
기록했다. 지금은 단일 팔 기준선을 되찾는 것이 우선이므로 여기서 멈춘다.

host 링크를 115200에서 올리면 같은 비용이 8배 줄지만, 그건 완화이지 제거가
아니다.

## 검증

| 항목 | 결과 |
|---|---|
| 전체 회귀 (`pytest -q`) | **666 passed** (이전 657, 신규 9) |
| actuator core native (`ctest`) | 2/2 passed |
| Cortex-M4 Release 크로스 빌드 | 성공 |
| 빌드 가드 음성 검증 | payload +4 B에서 `#error` 발동 확인 |
| 소스 단언 음성 검증 | terminal 조건 제거 시 시험 실패 확인 |

## 실기 검증

HEX `991e31d3d6b5d1c2506b59a87bf86cdea8eb233e5f1a59b64a83afe846c3a524`
(`artifacts/firmware/2026-08-06/stm32_g474_single_arm_0x00022900.hex`).

### 첫 시도 — identity 게이트가 막았다

플래시는 성공했으나 soak이 거부됐다.

```
HardwareIdentityError: firmware version mismatch: expected=0x00022800 actual=0x00022900
```

**MCU가 아니라 Pi의 host 패키지가 뒤처져 있었다.** `actual=0x00022900`을
읽었다는 것 자체가 새 펌웨어가 올라가 응답 중이라는 증거다. 원인은 배포
순서였다 — host 갱신이 soak보다 뒤에 있었다. 게이트는 제 역할을 했다.

### 재시도 — host 배포 후

명시된 9개 파일만 전송(파일별 양쪽 SHA 대조), `colcon build`, install space가
실제로 `EXPECTED_FIRMWARE_VERSION == 0x00022900`과 `_read_framed_packet`을
가지는지 확인한 뒤 soak을 시작했다.

| 항목 | 결과 |
|---|---|
| identity | `0x00022900` / `0x00000FFF` / `0x8AD27897`, latch 없음 |
| 70초 READ_ONLY soak | `passed=True`, `failure=None` |
| 호출 | heartbeat 700 · position 350 · diagnostics 2 |
| 오류 counter 12종 | **전부 0** (baseline부터 0, 증가 없음) |
| snapshot | `1.2 s`/`60.3 s` 모두 `recovery=0 fe=0`, receiver disarmed |
| motion 명령 | 0회 |

지금까지 회차 중 가장 깨끗하다. `0x00022500` 계열은 12 V 인가 구간의 일시
현상으로 `recovery=fe=resync=1`이 남았는데, 이번에는 baseline부터 전부 0이다.
`0x00022900`은 host 송신 프레임 길이만 바꿨으므로 servo UART 수명주기가
그대로임과 일치한다.

계약을 `deployed: true`로 전환했다(계약 JSON과 `buffered_trajectory.py` 동시).
`motion_authorized`는 `false` 유지 — 물리 동작은 매 회 별도 승인이다.

## 결론 — 분포가 진단을 확인했다

q0 복귀가 완주했고 apply lateness 분포를 처음으로 얻었다
([Motion-12](2026-08-06-motion12-buffered-q0-return.md)).

```
lateness_buckets = 1573, 96, 98, 90, 44, 0     (합 1901 = applied_samples)
```

이 분포는 위 진단을 **독립적으로 확인한다.**

1. 늦은 sample 328개 ÷ refill 추정 317회 = **1.035**.
   acknowledgement 1회당 늦은 sample 정확히 1개다.
2. bucket 1~3 의 비율이 `4.688 ms` 균일 모델의 27.1% 예측과 맞는다
   (관측 29.3 / 29.9 / 27.4%).
3. **bucket 5 가 비어 있다.** 전송이 `4.688 ms` 라 `5` 를 채울 수 없다.
   산술이 요구하는 그대로다.

82.75% 는 정확히 정시에 적용됐다. 남은 지연은 계통적 추종 오차가 아니라
**응답 프레임 전송이라는 단일하고 결정론적인 원인**이며, 그 크기는 전송시간에
고정되어 있다.

중단은 `lateness > 5`, 즉 `6 ms` 부터다. 이번 최악값은 `4`, Motion-11 은 `5`
였다 — `ceil(4.688) = 5` 경계 위에서 ms 양자화가 가른다. `6` 이 되려면 전송이
`5.0 ms` 를 넘어야 하고 그건 payload `+4 B` 이며, 빌드 가드가 정확히 거기서
거부한다.

**여유 `0.312 ms` 는 이론값이 아니라 실측으로 확인된 벼랑이다.**
1901 sample 중 44개가 이미 한 칸 아래에 있다.
