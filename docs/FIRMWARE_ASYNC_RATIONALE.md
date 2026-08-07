# 펌웨어 비동기 전환이 필요한 이유

양팔과 수건 접기 진입 전 필수 작업이다.

## 요약

- 펌웨어는 **협조적 단일 루프**다. 어느 한 작업이 길어지면 host 통신이 멈춘다
- heartbeat 를 **수신 시각이 아니라 처리 시각**으로 기록한다. 루프가 바쁘면
  MCU 는 host 와 끊긴 것으로 판단하고 **자기 500 ms watchdog 에 스스로
  latch** 한다. 시연 중이면 팔이 그 자리에서 멈춘다
- 이 불변식이 문서화되지 않아 **세 번 조용히 깨졌다**
- 현재 `0x00022A00` 은 한 팔 기준선에 충분하다. 다만 **남은 여유가
  `0.312 ms`** — 전선 기준 4바이트 미만이다
- 양팔은 서보 버스 2개, executor 2개, host 트래픽 2배다. 그 여유가 남을
  근거가 없다

---

## 1. 구조

`SingleArmApp_Process` 한 바퀴가 세 가지를 순서대로 한다.

1. host 바이트 처리
2. `BinaryControl_Service` 호출 (서보 위치 명령, 20 ms 주기)
3. heartbeat 기록

세 번째가 핵심이다. `binary_control.c` 의 `host_binary_last_heartbeat_ms` 는
**프레임이 도착한 시각이 아니라 루프가 그것을 처리한 시각**이다.

따라서 루프가 어딘가에서 오래 붙잡히면:

```
host 는 heartbeat 를 보냈다
  → MCU 가 처리하지 못했다
  → 기록상 연결이 끊긴 것이 된다
  → 500 ms watchdog 이 latch 한다
  → 팔이 멈춘다
```

**응답 지연으로 끝나지 않는다. MCU 가 스스로를 정지시킨다.**

---

## 2. 세 번 깨졌다

| 버전 | 무엇이 붙었나 | 결과 | 어떻게 발견 |
|---|---|---|---|
| `0x00022500` | 모든 servo write 앞에 `PrepareTransaction` (idle-high 대기 ≤20 ms) | DISABLE 봉투 `1817 → 1937 ms` | **산술** — 실기 전에 계산으로 |
| `0x00022600` | 같은 비용이 buffered 실행 중 motion-safety 폴링에 | host 관측 침묵 `365 ms` (한계 `500`) | **실기** — 계산으로 못 잡음 |
| `0x00022800` | host 응답 프레임에 lateness histogram | 전송 `4.688 → 7.118 ms`, q0 복귀가 **첫 sample 에서 중단** | **실기** |

`0x00022700` 이 buffered 실행 경로에서 servo read 를 제거해 두 번째를
해결했고, `tests/test_stm32_main_loop_blocking_budget.py` 가 예산을 소스
상수에서 계산해 회귀로 고정한다.

---

## 3. 세 번째가 가장 미묘하다

`Host_SendBinaryFrame` 은 blocking `HAL_UART_Transmit` 이다.
**그것을 호출하는 루프가 곧 executor 를 stepping 하는 루프다.**

결과: **응답 프레임의 길이가 apply lateness 로 그대로 청구된다.**

| 항목 | 값 |
|---|---|
| host 링크 | 115200 baud |
| refill 응답 1회 전송 | `4.688 ms` |
| apply lateness 허용치 | `5 ms` |
| **예산 사용률** | **94%** |

Motion-11 이 관측한 최대 apply lateness 가 정확히 `5 ms` 였다. 추종 오차가
아니라 이 전송 시간이었고, `ceil(4.688) = 5` 다.

그리고 `0x00022800` 에서 **그 지연을 측정하려고** histogram 을 응답에
실었더니 `7.118 ms` 가 되어 예산을 넘었고, q0 복귀가 첫 sample 에서 죽었다.

> 계측이 계측 대상을 바꿨다.

---

## 4. 현재 상태

`0x00022900` 이 histogram 을 terminal 프레임에만 싣는다. 그리고
`binary_control.c` 가 컴파일 단계에서 막는다.

```c
#if HOST_BINARY_FRAME_TRANSMIT_MS(ACTUATOR_BUFFERED_STATUS_EXTENDED_SIZE) > \
    HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS
#error "Buffered acknowledgement transmit exceeds the apply lateness allowance"
#endif
```

응답이 길어지면 **빌드가 거부된다.** `payload +4 B` 에서 발동함을 음성
검증했다.

**한 팔 기준선에는 이것으로 충분하다.** 2026-08-06 실기에서 apply lateness
는 내내 `4 ms` 였고 bucket 5 는 비어 있었다.

다만 남은 여유는 `0.312 ms` 다. 전선 기준 4바이트 미만이며, 이것을 여유라고
부르기는 어렵다.

---

## 5. 양팔에서 부족한 이유

| | 현재 | 양팔 |
|---|---|---|
| 서보 버스 | 1 | 2 |
| executor | 1 | 2 |
| host acknowledgement 트래픽 | 1배 | **2배** |
| 동작 성격 | Pick/Place | **수건 접기 — 더 길고 연속적** |

수렴 계층이 짧은 보정 leg 를 더하므로 Action 수도 늘어난다. 2026-08-06
실측에서 파지 1회당 보정 leg 1개가 추가됐다.

---

## 6. 전환 내용

세 가지다.

**6.1 서보 버스 I/O 를 비동기 상태기계로**
블로킹 대기를 제거한다.

**6.2 host 프레임 송신을 비동기로** (DMA + 유한 큐, 넘치면 fail-closed)
`0.312 ms` 문제를 구조적으로 없애는 유일한 방법이다. 프레임 길이가 더 이상
lateness 에 청구되지 않으므로 **진단을 늘려도 안전해진다.**

**6.3 executor tick 을 하드웨어 타이머 ISR 로**
main loop 가 바빠도 20 ms 주기가 밀리지 않는다.

---

## 7. 통신 속도는 제약이 아니다

자주 나오는 오해라 명시한다.

| 항목 | 값 |
|---|---|
| 서보 버스 | 1 Mbaud |
| sync write | `0.26 ms` |
| telemetry 왕복 | `0.23 ms` |
| 슬롯 | `5 ms` |

대역폭은 충분하다. **제약은 전부 blocking 구조다.**

host 링크를 115200 에서 올리면 같은 비용이 8분의 1로 줄지만 **완화이지
제거가 아니다.** 순서상 비동기가 먼저다.

---

## 8. 부수 효과 — 동작 중 진단이 되살아난다

현재 buffered 실행 경로에는 **부하·전류 감시가 없다.**
`Servo_MotionSafetyBegin/Poll` 이 `Host_ServiceBinaryMotion` 에만 있고,
blocking 구조 때문에 buffered 경로에 넣을 자리가 없어 제외했다.

그런데 2026-08-06 실측에서 **SHOULDER 가 명령한 위치에 도달하지 못하는
문제**가 확인됐다.

| 상황 | 오차 |
|---|---|
| 접는 방향 상승 | `6 raw` |
| 자세 유지 | `5~15 raw` |
| 펼친 채 드는 방향 | `32 raw` — **안전 게이트 초과, 동작 중단** |

`32 raw` 는 반경 `0.4 m` 에서 약 `20 mm` 다. 14회 관측이 `2685 ms` 동안
동일해 **평형 상태**임이 확인됐다 — 더 기다려도 가지 않는다.

이것이 **P-gain 한계인지 토크 포화인지** 구분하려면 동작 중 전류를 봐야
한다. 처방이 서로 다르다.

즉 비동기 전환은 양팔 진입 조건이자 **현재 한 팔의 한계를 진단하는
도구**이기도 하다.

---

## 9. 근거 기록

- [`0x00022600` 계측과 startup 중단](test-results/2026-08-06-stm32-0x00022600-apply-lateness-instrumentation.md)
- [`0x00022900` status 프레임 전송 예산](test-results/2026-08-06-stm32-0x00022900-status-transmit-budget.md)
- [C2 수렴 실기 — SHOULDER 정상상태 오차](test-results/2026-08-06-c2-convergence-physical.md)
- [A5 재현성 — 워밍업 전이](test-results/2026-08-06-a5-repeatability-pilot.md)
- 로드맵 D절 0번: `docs/CURRENT_STATE_AND_NEXT_ROADMAP.md`
