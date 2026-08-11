# 양팔 펌웨어 아키텍처 — 설계안

- 기준일: 2026-08-11
- 대상: `firmware/stm32_g474_single_arm` → 양팔 구조 전환
- 전제 문서: [FIRMWARE_ASYNC_RATIONALE.md](FIRMWARE_ASYNC_RATIONALE.md),
  [PLAN_CONVERGENCE_AND_BIMANUAL.md](PLAN_CONVERGENCE_AND_BIMANUAL.md),
  [ADR-0001](adr/0001-system-partition.md), [ADR-0002](adr/0002-motion-time-ownership.md)
- 이 문서는 **설계안**이다. 구현 전이며, 각 단계는 자체 gate 로 검증한다.

> **2026-08-11 정정.** 상위 계획은 [PLAN_CONTINUOUS_EXECUTION.md](PLAN_CONTINUOUS_EXECUTION.md)
> 다. 실측 결과 leg 사이 dead time `33 s` 중 펌웨어 몫은 **약 3 %** 이고
> 나머지는 host 실행 구조다. 따라서 **host 작업(H1~H3)이 이 문서의 F 단계보다
> 먼저**다. 이 문서에서 수정된 항목:
> - §3 의 mode enum 2개 → `horizon_end_tick` 필드 1개 + splice (§3.5)
> - §11 의 baud 상향 → **F2.5 로 앞당김**. 반응 지연의 1차 제약이다
> - §11 의 F8(STREAMING mode) → 삭제. horizon 필드로 흡수

---

## 0. 요약 — 결정 6개

| # | 결정 | 근거 |
|---|---|---|
| 1 | **FreeRTOS 를 쓰지 않는다.** 시간구동(time-triggered) 제어 코어 + 협조적 배경 루프 | 관측된 실패 3건이 전부 *scheduling* 이 아니라 *blocking I/O* 였다. RTOS 는 blocking 을 제거하지 않는다 |
| 2 | 제어 tick 을 **하드웨어 타이머 ISR** 로. 보간·sync-write 만 ISR 에서 수행 | tick jitter 를 loop 부하와 분리. ISR 진입 지연은 µs 급 |
| 3 | **Track A/B 를 mode 가 아니라 `stream_policy` + `horizon_end_tick` 으로 구분.** queue·보간기·출력단은 공통 | mode enum 조차 두지 않는다. 두 트랙은 같은 stream 을 다른 정책으로 연 것뿐 (§3A) |
| 4 | 팔당 `servo_bus_t` / `arm_context_t` **인스턴스화**. 전역 static 제거 | 현재 `servo_bus.c` 는 파일 static 전역이라 2개 인스턴스가 불가능 |
| 5 | 양팔 명령은 **공통 `apply_tick` 을 가진 12관절 단일 queue** 로 받는다 | skew 를 측정해서 줄이는 게 아니라 **구조적으로 0 으로 만든다.** ADR-0002 가 이미 규정 |
| 5b | **펌웨어는 두 stream 을 합산하지 않는다.** 중재는 Pi arbiter, 펌웨어엔 절대 목표 하나 | 이것을 어기는 순간 펌웨어가 Track A/B 를 구분하게 된다 (§3.6 A2) |
| 6 | 흔들림은 이 작업으로 **고쳐지지 않는다.** 대신 *측정 가능해진다* | 헌팅은 서보 내부 루프 안에 있다. 펌웨어가 그 위상 여유를 못 바꾼다. 4절 참고 |

---

## 1. FreeRTOS 판정

### 1.1 지금까지의 실패가 무엇이었는가

`FIRMWARE_ASYNC_RATIONALE.md` 가 기록한 3건:

| 버전 | 원인 | RTOS 가 고쳤을까 |
|---|---|---|
| `0x00022500` | servo write 앞 idle-high 대기 ≤20 ms | **아니다** — 대기 자체가 남는다 |
| `0x00022600` | buffered 실행 중 motion-safety 폴링 blocking | **아니다** — task 안에서 똑같이 막힌다 |
| `0x00022800` | `Host_SendBinaryFrame` 이 blocking `HAL_UART_Transmit` | **아니다** — task 가 blocking 하면 그 task 가 밀린다 |

**세 건 전부 "우선순위가 없어서" 가 아니라 "동기 I/O 가 제어 경로에 있어서"** 다.
FreeRTOS 는 blocking call 을 non-blocking 으로 바꾸지 않는다. DMA + 큐가 바꾼다.
그 둘은 서로 독립이다.

### 1.2 이 워크로드의 성격

| 항목 | 값 |
|---|---|
| 주기 작업 | 5 ms 출력 tick, 20 ms sample 주기 — **전부 고정 주기** |
| 최장 계산 | 6관절 선형 보간 + 한계 검사 ≈ 수 µs |
| 서보 트랜잭션 | sync-write `0.26 ms`, telemetry 왕복 `0.23 ms` |
| 5 ms 슬롯 사용률 | 두 경로 합쳐 **10 % 미만** |
| 선점이 필요한 긴 계산 | **없음** |

선점형 스케줄러의 값어치는 "길고 우선순위 낮은 계산이 짧고 급한 것을 막을 때"
나온다. 여기엔 그런 계산이 없다.

### 1.3 FreeRTOS 를 쓸 때 새로 생기는 것

- task 별 stack 산정(6 task × 512 B~1 KB), heap 정책, 오버플로 검출
- 우선순위 역전 분석, `FromISR` API 규율
- **tick 정밀도 하락**: `vTaskDelayUntil` 은 RTOS tick(보통 1 ms) 단위 +
  스케줄러 지연. 하드웨어 타이머 ISR 은 그보다 두 자릿수 정확하다
- **현재 자산 손상 위험**: `stm32_actuator` core 는 host 에서 컴파일·단위시험된다
  (회귀 841). RTOS primitive 가 core 로 새면 이 시험 경로가 깨진다

### 1.4 채택안 — 2계층 시간구동 구조

```text
┌─ 결정론 계층 (인터럽트) ──────────────────────────────┐
│ TIM6 5 ms tick ISR                                    │
│   1. 두 팔 setpoint 보간 (순수 산술)                  │
│   2. 관절 한계 + 속도 제한                            │
│   3. 두 버스 sync-write DMA 기동  ← skew 지점         │
│   4. 슬롯 잔여시간에 telemetry read 1건/팔 기동        │
│ DMA / UART ISR: 바이트 이동, 파싱 트리거만            │
└───────────────────────────────────────────────────────┘
┌─ 비결정론 계층 (배경 super-loop) ─────────────────────┐
│ host 프레임 파싱 → 명령 admission                     │
│ telemetry 집계, 진단 프레임 생성 → TX ring            │
│ 안전 감독(느린 판정), ARM/ENABLE 시퀀스, 복구         │
│ IWDG 급여                                             │
└───────────────────────────────────────────────────────┘
```

**핵심 불변식: 배경 루프가 아무리 오래 막혀도 제어 tick 과 heartbeat 기록은
영향을 받지 않는다.** 지금 구조가 깨졌던 지점이 정확히 이것이다.

### 1.5 그럼에도 FreeRTOS 로 가야 하는 조건 (falsifiable)

아래 중 **하나라도** 실측되면 재검토한다. 그 전에는 도입하지 않는다.

- 배경 루프 1회전 최악 시간이 `20 ms`(sample 주기)를 넘음
- 서로 blocking 하는 독립 활동이 3개 이상 동시에 필요해짐
- ISR 최악 실행시간이 `500 µs`(tick 의 10 %)를 넘음
- 파일 시스템 / TCP-IP / USB 스택 등 자체 task 를 요구하는 미들웨어 도입

### 1.6 (참고) FreeRTOS 로 갈 경우의 task 표

채택안이 아니지만, 위 조건이 발동했을 때 옮겨갈 대상 구조를 명시해 둔다.

| task | 우선순위 | 유형 | 주기/트리거 | stack | 역할 |
|---|---|---|---|---|---|
| `ctrl` | 5 (최고) | 주기 | 5 ms, timer notify | 1 KB | 보간·한계·dispatch |
| `busL` | 4 | 이벤트 | task notification (DMA 완료) | 768 B | 좌 버스 트랜잭션 FSM |
| `busR` | 4 | 이벤트 | task notification | 768 B | 우 버스 트랜잭션 FSM |
| `safety` | 3 | 주기 | 10 ms | 512 B | fault 집계·coordinated stop |
| `host` | 2 | 이벤트 | stream queue | 1.5 KB | 프레임 파싱·admission·telemetry 인코딩 |
| `idle`+IWDG | 0 | — | — | — | watchdog 급여, CPU 여유 측정 |

주의: 이 표대로 가더라도 `ctrl` 은 여전히 **타이머 ISR 이 notify** 해야 한다.
`vTaskDelayUntil` 만으로는 tick jitter 를 보장하지 못한다.

---

## 2. 전체 구조

```text
Raspberry Pi 5
  MoveIt2 / Task FSM ─┐
                      ├─→ Unified Command Interface (µrad, apply_tick, arm_mask)
  RL policy ──────────┘            │
                                   │ COBS + CRC-32C, LPUART1
                                   ▼
╔══════════════════════ STM32G474 ═══════════════════════╗
║                                                        ║
║  [host_link]  RX ISR ─→ byte ring ─→ parser ─→ frame   ║
║       ▲                    (SPSC)                 │    ║
║       │ DMA TX ←─ byte ring ←─ encoder ←──────────┤    ║
║                    (SPSC)                         │    ║
║                                                   ▼    ║
║                                          [command router]
║                                          arm_mask 로 분배 ║
║                                          │             ║
║                          ┌───────────────▼───────────┐ ║
║                          │  setpoint_queue (16)      │ ║
║                          │  sample = {apply_tick,    │ ║
║                          │    q[12]}  ← 양팔 한 덩어리│ ║
║                          │  executor 1개, 지평 1개   │ ║
║                          └───────────────┬───────────┘ ║
║                    arm_context[L] 한계·보정 │ arm_context[R]
║                              └────────┬────┴───────────┘║
║                                       ▼                 ║
║                        [control tick ISR — TIM6 5 ms]   ║
║                         보간 → 한계 → 속도제한 → dispatch║
║                              │                    │     ║
║                              ▼                    ▼     ║
║                    [servo_bus_t L]        [servo_bus_t R]
║                     USART1 @1 Mbaud        UART4 @1 Mbaud
║                     TX DMA / RX DMA ring   TX DMA / RX DMA ring
║                              │                    │     ║
║                        [safety supervisor — 팔별 + 시스템]
╚═════════════════════════╪══════════════════╪═══════════╝
                          ▼                  ▼
              Waveshare driver L      Waveshare driver R
                          │                  │
                    좌 STS3215 ×6      우 STS3215 ×6
                    (12 V 독립전원)     (12 V 독립전원)
```

### 2.1 하드웨어 자원 배정

| 자원 | 현재 | 양팔 |
|---|---|---|
| host link | LPUART1 (PA2/PA3, ST-LINK VCP) @115200 | 동일, F2.5 에서 921600 |
| 좌 서보 UART | USART1 (PC4/PC5) @1 Mbaud | 동일 |
| 우 서보 UART | — | **UART4: PC10 TX = CN7-1, PC11 RX = CN7-2 @1 Mbaud** |
| bus driver | Waveshare Bus Servo Adapter (A) L ×1 | L ×1 + R ×1, 각 팔 전용 |
| DMA | DMA1_Ch1 (USART1_RX) | +USART1_TX, +UART4_RX/TX, +LPUART1_TX = 5채널. DMA1/DMA2 각 8채널이므로 여유 충분 |
| 제어 tick | 없음 (loop 구동) | **TIM6** 5 ms |
| µs 타임스탬프 | 없음 | **TIM2** (32-bit) free-running @170 MHz, 5.9 ns 분해능, 25 s wrap |
| 계측 핀 | 없음 | GPIO 2~3개 (tick, busL TX, busR TX) — 로직 분석기용 |
| watchdog | 없음 | IWDG, **배경 루프에서만** 급여 |

**우팔 UART 결정 근거(2026-08-11 검증)**:

- STM32G474 데이터시트에서 PC10/PC11의 AF5가 각각 UART4_TX/RX다.
- NUCLEO-G474RE 회로도와 UM2505에서 두 신호가 인접한 Morpho
  `CN7-1/CN7-2`에 직접 나온다. 보드 기본 VCP·LED·버튼·SWD와도 겹치지 않는다.
- 후보였던 PB10/PB11은 `CN10-25/CN10-18`로 떨어져 있고 TIM2_CH3/CH4 및
  LPUART1 대체 기능을 공유한다. 성능 차이는 없으므로 더 단순한 배선과 TIM2
  여유를 택한다.

근거 원문은 [STM32G474 데이터시트](https://www.st.com/resource/en/datasheet/stm32g474re.pdf),
[NUCLEO-G474RE 사용자 매뉴얼 UM2505](https://www.st.com/resource/en/user_manual/dm00556337.pdf),
[MB1367-G474RE 회로도](https://www.st.com/resource/en/schematic_pack/mb1367-g474re-d01-schematic.pdf)다.
실제 경로는 팔마다 **STM32 UART 논리측 ↔ Waveshare Bus Servo Adapter (A)
↔ 12 V STS3215 bus**다. 우팔 배선은 **PC10/UART4_TX → driver RX**,
**PC11/UART4_RX ← driver TX**, 공통 GND다. 먼저 driver 논리측 출력 전압과
입력 high 임계값이 STM32 3.3 V IO와 호환됨을 실측한다. driver를 전기적
절연기로 가정하지 않으며, 12 V servo bus를 MCU 핀에 직접 연결하지 않는다.
USART2는 PA2/PA3에서 host
LPUART1과 충돌하므로 제외한다.

### 2.2 NVIC 우선순위 (그룹 4, preempt 4 bit)

| 우선순위 | 인터럽트 | 이유 |
|---|---|---|
| 1 | TIM6 (control tick) | 유일한 hard real-time |
| 2 | DMA1/2 (servo TX/RX 완료·idle) | tick 이 기동한 전송의 종료 처리 |
| 3 | USART error / IDLE | 오류 래치 |
| 4 | LPUART1 RX | **heartbeat 시각을 여기서 찍는다** |
| 5 | LPUART1 TX DMA | 배출만 |
| 15 | SysTick | HAL_GetTick. 제어를 선점하면 안 됨 |

**규칙: 어떤 ISR 에서도 `HAL_Delay` 를 호출하지 않는다.** SysTick 이 최하위이므로
높은 우선순위 ISR 안의 `HAL_Delay` 는 영구 정지다. 빌드 시 grep 회귀로 고정한다.

### 2.3 시간축 단일 소스

현재 `apply_tick` 은 `HAL_GetTick`(SysTick) 기준이다. 출력은 TIM6 이 낸다.
**두 시계가 다르면 apply lateness 는 두 시계의 drift 를 측정하게 된다.**

→ `timebase_now_ms()` 를 TIM6 tick ISR 이 유지하는 카운터로 바꾼다.
`apply_tick`, lateness, TIME_SYNC 응답이 전부 이 하나를 쓴다.
`HAL_GetTick` 은 HAL 내부 타임아웃 용도로만 남긴다.

---

## 3. 명령 추상화 — Track A 와 B 를 하나로

### 3.1 관찰

두 트랙의 자료형이 이미 같다. 기존 `actuator_setpoint_t` 가 둘 다 담는다.

```c
typedef struct {
    uint32_t apply_tick;
    int32_t  position_urad[ACTUATOR_JOINT_COUNT];
} actuator_setpoint_t;
```

**Track B 는 "queue 깊이가 얕고 lead 가 짧은 Track A"** 다. 보간기·출력단·한계
검사·안전 FSM 은 전부 공유한다. 갈리는 지점은 정확히 셋이다.

### 3.2 갈리는 것은 하나다 — `horizon_end_tick`

mode enum 두 개를 두려 했으나 더 단순하게 합쳐진다. stream 이
`horizon_end_tick`(= "여기까지는 내가 채우기로 했다") 필드 하나를 갖는다.

| queue 고갈 시점 | 의미 | 동작 |
|---|---|---|
| `now < horizon_end_tick` | host 가 채우기로 한 구간을 못 채웠다 | **fault** — 기존 `QUEUE_UNDERFLOW` |
| `now >= horizon_end_tick` | 선언한 끝에 도달 | **정상 종료** → HOLD |
| `horizon_end_tick` 미설정 (열린 stream) | RL 스트리밍 | 마지막 목표 유지 + stale 타이머 |

**Track A 는 "끝이 선언된 stream", Track B 는 "끝이 없는 stream" 이다.**
mode 분기가 사라지고, 연속 실행(궤적 여러 개를 이어 붙이기)도 같은 필드로
표현된다 — 지평을 계속 연장하면 된다.

부수적으로 갈리는 상수 2행:

| 항목 | 끝이 선언된 stream | 열린 stream |
|---|---|---|
| admission 창 | lead 60~400 ms | lead 20~100 ms |
| 타임아웃 | heartbeat 500 ms | + command timeout 100 ms |

### 3.3 STM32 는 출처를 모른다

```text
MoveIt2 ─┐                        ┌─ mode=TRAJECTORY, lead 60~400 ms
         ├→ 같은 프레임 형식 ────→┤
RL ──────┘                        └─ mode=STREAMING,  lead 10~100 ms
                                        │
                              동일한 queue / 보간기 / 출력단
```

STM32 가 아는 것은 mode 뿐이고, mode 는 **명령 헤더의 필드**다. "MoveIt 인가
RL 인가" 는 끝까지 등장하지 않는다. 중재(arbitration)는 Pi 의 command arbiter
책임이다 (ADR-0002).

### 3.4 출력단 — 두 트랙 공통

```text
executor.step(now) → target_urad[6]
        ↓
  [1] 관절 한계 clamp        (calibration 기반, 팔별)
        ↓
  [2] 속도 제한 rate limit   |Δq| ≤ MAX_STEP_URAD_PER_TICK
        ↓
  [3] µrad → raw 변환        (팔별 home/방향/범위)
        ↓
  [4] sync-write 프레임 조립 → DMA
```

**[2] 가 Track B 의 "smooth target transition" 을 해결한다.** RL 이 계단 목표를
내도 출력은 tick 당 제한된 증분으로만 움직인다. 동시에 Track A 에서도 계획
오류에 대한 backstop 이므로 **두 트랙 공통 안전 장치**이지 B 전용 기능이 아니다.
`actuator_core` 안, host 단위시험 대상.

### 3.5 splice — 반응 지연의 하한을 정하는 연산

지평은 불변 배치가 아니라 **덮어쓸 수 있는 rolling window** 다. 이것이
"연산이 끝나면 즉시 나간다" 를 구현하는 지점이다.

```text
tick →  ├──────────────────────────────────┤  실행 중인 지평
        ▲                    ▲
     현재 적용점        splice_at_tick
                       └─ 이후 sample 을 새 것으로 교체. 팔은 멈추지 않는다
```

수락 조건 (전부 fail-closed):

| 검사 | 거부 사유 |
|---|---|
| `splice_at_tick >= now + MINIMUM_SPLICE_LEAD` | 너무 늦게 도착 |
| `splice_at_tick > 마지막 적용 tick` | 이미 지나간 시점 |
| 첫 sample 이 그 시점 보간값에서 관절당 `MAX_STEP_URAD_PER_TICK` 이내 | 불연속 → 급발진 |

수락 시 `splice_at_tick` 이후 queue 내용만 폐기·교체한다. 그 이전 sample 은
계속 실행되므로 **동작이 끊기지 않는다.**

#### 반응 지연 예산

| 항목 | 115200 | **921600** |
|---|---|---|
| splice 3 sample 인코딩·전송 (186 B) | 16.1 ms | **2.0 ms** |
| 최소 splice lead | 60 ms | **20 ms** |
| 출력 tick 대기 (최대) | 5 ms | 5 ms |
| sync-write | 0.26 ms | 0.26 ms |
| **합계** | **~81 ms** | **~27 ms** |

**host 링크 속도가 반응성의 1차 제약**이므로 baud 상향은 splice 와 같은
단계에서 한다 (F2.5). 참고로 refill 배치 9 sample(498 B)은 115200 에서
`43.2 ms`, 921600 에서 `5.4 ms` 다.

### 3.6 무출처성(source-agnostic) 감사

"상단에 Track A 가 오든 B 가 오든 펌웨어가 몰라도 되는가" 를 항목별로 검사했다.

#### 통과 — 출처를 몰라도 되는 것

| 항목 | 근거 |
|---|---|
| sample 자료형 | `{apply_tick, q[12]}` 하나. 두 트랙 동일 |
| 보간·한계·속도제한·dispatch | 완전히 같은 경로. 분기 없음 |
| queue 고갈 / 종료 | `horizon_end_tick` 하나로 셋 다 표현 (§3.2) |
| splice | 수렴 보정과 RL 갱신이 **같은 연산** (§3.5) |
| 안전 FSM / coordinated stop | 출처와 무관 |
| gripper | 서보 테이블에 이미 6번 관절로 들어 있다. stream 이 원래 gripper 를 포함한다 |

#### 미통과 — 5건

**A1. 컴파일 시점 고정 상수가 Track A 를 가정한다.**

| 상수 | 문제 |
|---|---|
| `MINIMUM_START_SAMPLES = 16` (+`#error`) | RL 정책은 한 step 에 목표 1개를 낸다. **16개를 절대 못 채운다** |
| `SAMPLE_PERIOD_MS = 20` (+`#error`) | RL `control_dt` 는 50 ms. 지금은 5가 50을 나누어 우연히 맞지만 30 Hz(33.3 ms)면 깨진다 |
| heartbeat 500 / command timeout 100 | §3.2 에서 상수 표로 썼다 |
| `FINAL_ERROR_TOLERANCE_RAW = 30` | Track B 는 추종 지연이 정상인데 abort 될 수 있다 |

→ **전부 stream 선언 필드로 옮긴다.** 다만 안전 임계값은 규칙을 붙인다:
**펌웨어가 컴파일 시점 하드 상한을 갖고, stream 은 그것을 더 조일 수만 있다.**
느슨하게 하는 값은 거부한다. fail-closed 규율이 유지된다.

**A2. residual 합산을 펌웨어에 넣으면 즉시 갈라진다. (가장 큰 잠복 위험)**

로드맵 D/E 는 "정책은 관절 보정값 또는 제한된 Cartesian residual 을 출력한다"
고 규정한다. 즉 최종 목표 = MoveIt 기준 궤적 + RL residual 이다.

```text
[안전]  Pi arbiter 가 더한다 → 절대 목표 stream 1개 → 펌웨어는 출처를 모른다
[갈라짐] 펌웨어가 두 stream 을 받아 더한다 → 펌웨어가 A 와 B 를 구분하게 된다
```

> **불변식: 펌웨어는 두 stream 을 합산하지 않는다. 중재는 Pi 의 command
> arbiter 가 하고 펌웨어에 내려오는 것은 언제나 절대 목표 하나다** (ADR-0002).

이것을 적어두지 않으면 "residual 만 보내면 대역폭이 절약된다" 는 이유로 조용히
깨진다. 대역폭은 제약이 아니다(§3.5).

**A3. gripper 소유권 모델이 배타적이다. (host 계층)**

`motion_goal_arbiter.py` 의 `MotionGoalArbiter` 는 `{arm | gripper}` 중 **하나만**
활성화한다. RL 은 gripper 를 action 벡터의 일부로 팔과 **동시에** 낸다.

펌웨어는 이미 6관절(gripper 포함)을 실어 나르므로 문제없다. **깨지는 것은 host
arbiter 다** — 배타적 소유권에서 "stream 소유권 1개" 로 바꿔야 한다. H 트랙 항목.

**A4. feedback rate 는 갈래가 아니라 전제조건이다.**

Track A 는 leg 경계에서만 feedback 이 필요하다. Track B 는 20 Hz 관측이
필요하고 나이 상한이 있다. 현재 `/joint_states` 는 **5 Hz** 다.

설계상 30 ms sweep(33 Hz)이 나오므로 상위집합이고 갈래가 생기지 않는다. 다만
**F4·F8 이 Track B 의 전제조건**이라는 순서 제약이 된다.

**A5. host jitter 하의 매끄러움은 펌웨어가 못 고친다.**

queue 가 비면 마지막 목표를 유지한다 → 감속 후 정지 → 다음 sample 도착 시
재가속. 속도 제한이 크기를 막지만 **정지-재출발 자체는 남는다.**

→ **host 계약으로 못박는다: 두 트랙 모두 queue 에 최소 2 sample 을 유지한다.**
차이는 실패했을 때의 반응(fault vs hold)뿐이다.

**외삽(extrapolation)을 넣지 않는다.** 매끄러움을 위해 외삽하면 (1) Track B
전용 경로가 생겨 갈라지고 (2) 낡은 데이터로 팔이 계속 나아가는 안전 위험이
생긴다. 정지-재출발이 정직하고 안전하다.

#### 결론

**A1·A3 을 처리하면 펌웨어는 출처를 모른다. A2·A5 는 코드가 아니라 규율이고,
적어두지 않으면 나중에 조용히 깨진다.** A4 는 순서 제약이다.

---

## 3A. 해결 방안

§3.6 의 5건은 전부 해결 가능하다. A2·A5 는 규율로 두지 않고 **메커니즘**으로
바꾼다 — 규율은 사람이 잊지만 메커니즘은 잊지 않는다.

### A1 해결 — 상수를 3등급으로 나누고 stream 정책으로 선언한다

값을 "누가 다치는가" 로 분류한다.

| 등급 | 예 | 처리 |
|---|---|---|
| **1. 하드웨어 불변** | 출력 주기 5 ms, 관절 한계, load/current 한계, 토크 상한 | **컴파일 시점 고정.** 협상 대상 아님 |
| **2. 안전 임계값** | apply lateness, tracking error, heartbeat/command timeout, 최대 lead, 관절별 tick 당 최대 증분 | **컴파일 시점 하드 캡 + stream 이 조일 수만 있음** |
| **3. 타이밍 정책** | sample 주기, prime 깊이, 최소 lead, `horizon_end_tick` | **stream 이 자유 선언** (범위 검사만) |

#### stream 정책 — 열 때 1회 선언, 이후 재협상 없음

```c
typedef struct {
    /* 3등급 — 자유 */
    uint16_t minimum_start_samples;        /* 1 .. QUEUE_CAPACITY */
    uint32_t minimum_lead_ms;              /* >= HARD_MIN_LEAD_MS */
    uint32_t horizon_end_tick;             /* 0 = 열린 stream */

    /* 2등급 — 조이기만 가능 */
    uint32_t maximum_lead_ms;              /* <= HARD_MAX_LEAD_MS        (400) */
    uint32_t command_timeout_ms;           /* <= HARD_COMMAND_TIMEOUT_MS (500) */
    uint32_t maximum_apply_lateness_ms;    /* <= HARD_MAX_LATENESS_MS    (5)   */
    int32_t  tracking_error_limit_urad[ACTUATOR_JOINT_COUNT];  /* 관절별 */
    int32_t  maximum_step_urad_per_tick[ACTUATOR_JOINT_COUNT]; /* 관절별 */
} actuator_stream_policy_t;
```

**느슨하게 하는 값은 조용히 clamp 하지 않고 거부한다.** clamp 하면 host 는
`100 ms` 를 얻은 줄 알고 실제로는 `5 ms` 를 받는다 — 거짓말이다. `STREAM_OPEN`
전체를 거부하고 **어느 필드가 왜 거부됐는지** 보고한다.

#### `#error` 는 사라지지 않고 한 층 위로 올라간다

지금 `#error` 가 지키는 것은 "이 상수 조합이 성립하는가" 다. 그것을 **하드 캡
사이의 관계**로 옮기면 여전히 소스에서 정적으로 유도된다.

| 기존 | 이후 |
|---|---|
| `MAXIMUM_APPLY_LATENESS_MS == OUTPUT_PERIOD_MS` | `HARD_MAX_LATENESS_MS <= OUTPUT_PERIOD_MS` |
| `SAMPLE_PERIOD % OUTPUT_PERIOD == 0` | **삭제** (아래) |
| `MINIMUM_START_SAMPLES == 16` | `1 <= HARD_MIN_PRIME <= QUEUE_CAPACITY` |
| status 전송시간 ≤ lateness (F2 로 무의미해짐) | **`HOST_TX_RING_BYTES >= 최악 burst`** 로 교체 |
| — | `QUEUE_CAPACITY × 최소 sample 주기 >= HARD_MAX_LEAD_MS` |

**분할 가능성 `#error` 를 삭제하는 근거**: sample 을 `apply_tick` 이후 **첫
tick** 에 적용하므로 비정수배로 인한 지연은 구조적으로 `출력주기 − 1` 미만이다.
기존 lateness 상한이 이미 일반 경우를 덮고 있었고, `#error` 는 과잉 제약이었다.

다만 그러면 lateness 가 항상 `0~4` 로 나와 **굶김을 가린다.** 그래서 측정
정의를 바꾼다:

```text
lateness = 실제 적용 tick − (apply_tick 이상인 첫 출력 tick)
```

양자화(비정수배)와 굶김이 분리된다. 정상이면 여전히 `0`.

#### 회귀 시험

`#error` 가 담당하던 역할 중 stream 부분을 host 단위시험이 받는다:
**정책 필드를 하나씩 하드 캡보다 느슨하게 만들어 전부 거부되는지 확인**한다.
`actuator_core` 안이라 하드웨어 없이 돈다.

### A2 해결 — `arbiter_epoch`: 규율을 메커니즘으로

"펌웨어는 두 stream 을 합산하지 않는다" 를 사람이 기억하는 대신 프로토콜이
강제하게 만든다.

- 프로토콜에 **residual 플래그도, 두 번째 stream 슬롯도 만들지 않는다.**
  자리를 남기면 반드시 쓰인다
- 모든 명령 배치가 `arbiter_epoch`(uint32) 를 싣는다. Pi 의 command arbiter 가
  **명령 소유자가 바뀔 때마다** 증가시킨다
- 펌웨어는 그 숫자가 **무엇을 뜻하는지 모른다.** 바뀌었다는 것만 안다

```text
epoch 동일 → 평범한 지평 연장 (append)
epoch 변경 → splice 로만 수락. 연속성 검사 필수 (§3.5)
             현재 보간값과 첫 sample 이 관절당 한계 안이어야 함
```

얻는 것:

| | |
|---|---|
| 무출처성 | 유지 — 펌웨어는 "MoveIt" 도 "RL" 도 모른다 |
| **Track A→B 인계 안전** | 접근은 MoveIt, 조작은 RL 로 넘길 때가 가장 위험한 순간이다. 그 지점에 **연속성 검사가 강제로 걸린다** |
| 단일 중재자 불변식 | 두 소스가 arbiter 를 우회해 각자 쓰면 epoch 이 충돌해 **검출된다** |

> 규율이 메커니즘이 됐다. "합산하지 마라" 를 잊어도 프로토콜에 합산할 자리가
> 없고, 소유자 교체는 반드시 연속성 검사를 지난다.

### A3 해결 — 소유권을 `{arm | gripper}` 에서 `{stream}` 으로

펌웨어는 이미 gripper 를 6번 관절로 싣고 있다. 바꿀 것은 host 다.

```text
[현재] MotionGoalArbiter: {arm | gripper} 중 하나만 활성
        → RL 이 팔과 gripper 를 동시에 못 낸다

[목표] StreamOwner: stream 소유자 1명
        → gripper 는 그냥 stream 의 한 열
        → 파지 = gripper 열만 변하는 짧은 구간
```

`gripper_cmd` ActionServer 는 **없애지 않는다.** MoveIt 표준 호환을 위해
남기되, 배타적 소유권을 잡는 대신 **짧은 gripper 전용 구간을 계획해 splice
하는 얇은 wrapper** 로 구현한다. 하드웨어 경로가 하나로 합쳐진다.

부수 효과: 파지 확인(잔여 간격)이 정착 시점 1회가 아니라 **연속 telemetry
33 Hz** 로 읽힌다.

#### 여기서 나온 필수 세부 — tracking error 는 관절별이어야 한다

gripper 는 토크 상한이 `150` 이고 **물체를 물면 위치 오차가 정상적으로 남는다.**
스칼라 tracking error 게이트를 그대로 쓰면 **파지할 때마다 abort** 된다.

지금은 gripper 가 buffered 경로 밖에 있어 드러나지 않았지만, stream 에
합치는 순간 즉시 터진다. 그래서 A1 의 정책 구조에서
`tracking_error_limit_urad[]` 를 **관절별 배열**로 잡았다. 파지 구간에서는
gripper 항목만 사실상 해제한다.

### A4 해결 — 순서 제약으로 못박고 나이를 프로토콜에 넣는다

갈래가 아니라 의존성이다.

```text
F4 (동작 중 telemetry) ─┐
                        ├─→ Track B 활성화 가능
F8 (연속 telemetry)  ───┘
```

RL 계약이 "관측 stale → reject" 를 요구하므로 **telemetry 에 관절별 표본 나이**
(`sample_age_ms`)를 싣는다. Pi 가 추정하지 않고 그대로 읽는다.

### A5 해결 — 계약 + 위반을 보이게 만드는 계수기

- **계약**: 두 트랙 모두 queue 에 최소 2 sample 을 유지한다. 차이는 실패했을
  때의 반응(fault vs hold)뿐이다
- **관측**: 기존 `peak_queued_samples` 에 더해 `minimum_queued_samples` 와
  **`horizon_open_underrun_count`**(지평이 열려 있는데 queue 가 비어 hold 로
  떨어진 횟수)를 추가한다

열린 stream 의 고갈은 `fault` 가 아니므로 **계수하지 않으면 조용히 지나간다.**
계수기가 있으면 "host 가 계약을 못 지키고 있다" 가 metric 으로 보인다.

#### 버린 대안 — 외삽

매끄러움을 위해 마지막 속도로 외삽하는 방법이 있으나 채택하지 않는다.

| 이유 | |
|---|---|
| 갈라진다 | Track B 전용 경로가 생긴다. 이 문서 전체의 목적에 반한다 |
| **위험하다** | 명령이 끊겼는데 팔이 **계속 나아간다.** stale 데이터로 움직이는 것이 정확히 fail-closed 의 반대다 |

정지-재출발이 정직하고 안전하다. 매끄러움은 host 가 lead 를 유지해서 얻는다.

### 요약

| | 성격 | 작업량 | 어디 |
|---|---|---|---|
| A1 | 상수 → 3등급 + stream 정책 | **중** — `actuator_core` + 프로토콜 + 회귀 | 펌웨어 |
| A2 | `arbiter_epoch` + 합산 자리 미제공 | **소** — 필드 1개 + splice 재사용 | 프로토콜 |
| A3 | 소유권 모델 교체, tracking error 관절별 | **중** — host arbiter + gripper wrapper | host + 펌웨어 |
| A4 | 순서 의존 + `sample_age_ms` | **소** | 프로토콜 |
| A5 | 계약 + 계수기 2개 | **소** | 펌웨어 |

**A1 과 A3 이 실질 작업이고 나머지는 필드 추가 수준이다.** 전부 프로토콜 v2
설계 시점에 함께 결정해야 한다 — 나중에 끼워 넣으면 v3 가 된다.

---

## 4. 흔들림에 대해 — 정직하게

`session-2026-08-10` 진단 결론: **q0 근처에서 SHOULDER 중력 모멘트 ≈ 0 인 구간의
백래시 헌팅**. 관측 8개가 일치하며 유력하지만 확정은 아니다.

### 4.1 이 아키텍처가 고치지 못하는 것

**헌팅은 STS3215 내부 위치 루프 안에 있다.** 우리 펌웨어는 20 ms 마다 목표를
갱신하는 바깥 계층이고, 서보 내부 P/D 루프의 위상 여유를 바꿀 수단이 없다.
비동기 전환으로 흔들림이 사라진다고 말하면 그건 과장이다.

### 4.2 이 아키텍처가 실제로 주는 것

| # | 얻는 것 | 왜 지금은 없나 |
|---|---|---|
| 1 | **동작 중 관절별 load/current, 팔당 30 ms 전주기** | 현재 buffered 실행 경로엔 부하 감시가 **아예 없다**. blocking 구조라 넣을 자리가 없었다 (`FIRMWARE_ASYNC_RATIONALE` §8) |
| 2 | **leg 별 게인 스케줄링 경로가 싸진다** | 메모리 결론상 흔들림을 정말 고친다면 δ(q) 보다 leg별 P 가 낫다. servo write 가 non-blocking 이면 leg 경계 재설정 비용이 사라진다 |
| 3 | **측정에서 교란요인 제거** | 지금은 setpoint 계단의 jitter 와 헌팅이 섞여 보인다 |

즉 이 작업의 흔들림 관련 산출물은 **"고쳤다" 가 아니라 "백래시 가설을 전류
파형으로 확인/기각했다"** 다. 확인되면 leg별 P 로 넘어가고, 기각되면 다른
가설을 세운다. 어느 쪽이든 지금은 없는 데이터다.

### 4.3 범위 밖으로 두는 것

- 흔들림 자체의 해소 — 대기·귀환 자세에 갇혀 있고 작업 자세엔 나오지 않는다는
  판단은 유지한다 (사용자와 합의됨)
- 중력 feedforward δ(q) — 정확도는 이미 `execute_grasp_convergence_once.py`
  닫힌 루프가 담당한다
- `POST_SETTLE_TOLERANCE_RAW = 30` 변경 — P=64 에서 14 raw 여유가 있어 현재
  차단 요인이 아니다

---

## 5. 서보 버스 통신 상태기계

팔당 1개 인스턴스, 완전 독립. 좌 실패가 우 타이밍에 영향을 주지 않는다.

```text
                      ┌──────────────────────────────┐
                      │            IDLE              │
                      └──┬──────────────────────┬────┘
      tick: 명령 dispatch │                      │ tick: 슬롯 여유 있음
                         ▼                      ▼
              ┌────────────────────┐   ┌────────────────────┐
              │ TX_SYNC_WRITE      │   │ TX_READ_REQUEST    │
              │ DMA, 응답 없음     │   │ DMA, 8 B           │
              │ ≈0.26 ms           │   └─────────┬──────────┘
              └─────────┬──────────┘             │ TX 완료 ISR
                        │ TX 완료 ISR            ▼
                        │              ┌────────────────────┐
                        │              │ WAIT_REPLY         │
                        │              │ DMA RX ring 감시   │
                        │              │ deadline 2 ms      │
                        │              └───┬────────┬───────┘
                        │        프레임 완성 │        │ timeout/오류
                        │                   ▼        ▼
                        │         ┌──────────┐  ┌──────────────┐
                        │         │ PARSE    │  │ RECOVER      │
                        │         │ 체크섬   │  │ quiet 2 ms   │
                        │         │ 스냅샷   │  │ 수신기 재동기│
                        │         └────┬─────┘  │ 오류 래치    │
                        │              │        └──────┬───────┘
                        └──────────────┴───────────────┘
                                       ▼
                                     IDLE
```

### 5.1 5 ms 슬롯 시간표

```text
t+0.000  tick ISR 진입, 두 팔 보간 (~5 µs)
t+0.010  busL sync-write DMA 기동 ─┐ 두 UART 독립 → 병렬
t+0.012  busR sync-write DMA 기동 ─┘ skew ≈ 명령어 몇 개
t+0.28   두 TX 완료
t+0.30   busL/busR telemetry read 요청 기동 (관절 round-robin)
t+0.90   응답 도착·파싱 (DMA ISR)
t+0.90 ~ t+5.00   버스 유휴 — 여유 82 %
```

- **전주기 telemetry: 6관절 × 5 ms = 팔당 30 ms.** 현재 96 ms sweep 대비 3배,
  그리고 **동작 중에** 돈다
- 명령 경로가 telemetry 보다 절대 우선. telemetry 는 슬롯 잔여시간에만 발행하고
  deadline 초과 시 그 회차를 버린다 (재시도는 다음 슬롯)
- sync-write 는 broadcast 라 응답이 없다 → **명령 경로에 왕복이 없다**

### 5.2 blocking 을 남겨도 되는 곳

전부 없앨 필요는 없다. **제어 tick 이 도는 동안 실행되지 않는 경로**는 blocking
으로 남긴다. 유지 대상:

- `Servo_ConfigureAllForTrajectory` (ARM/ENABLE 시퀀스, `HAL_Delay` 포함)
- 부팅 자가진단, 복구 시퀀스, `Servo_DisableTorqueAll`

조건: 이 경로에 들어가기 전 executor 가 반드시 비활성 상태여야 하며, 상태
검사로 강제한다. 회귀 시험으로 고정한다.

---

## 6. 안전 상태기계 — 팔별 + 시스템

### 6.1 2계층

```text
arm_state[LEFT]   ∈ {BOOT, SAFE_DISABLED, ARMED, ACTIVE, HOLD, FAULT}
arm_state[RIGHT]  ∈ 같음
system_state      ∈ {BOOT, STANDBY, READY, RUNNING, COORDINATED_STOP, ESTOP}
```

기존 `actuator_state_t` 를 팔별로 인스턴스화하고, 시스템 상태를 위에 얹는다.

### 6.2 조율된 중단 (coordinated stop)

```text
어느 한 팔이 FAULT ─→ system = COORDINATED_STOP
                        ├─ 두 팔 executor 즉시 종료
                        ├─ 두 팔 현재 위치로 HOLD (토크 유지)
                        ├─ 원인 팔·사유를 fault report 로 송신
                        └─ 조작자 CLEAR_FAULT 까지 래치
```

**HOLD 이지 DISABLE 이 아니다.** DISABLE 은 팔을 떨어뜨린다
(`so101-arm-must-be-supported-before-disable`). 물체를 든 팔이 있을 수 있으므로
자동 반응은 항상 HOLD 로 고정하고, DISABLE 은 조작자 명시 명령으로만 도달한다.

### 6.3 fault 목록과 반응

| fault | 검출 위치 | 반응 |
|---|---|---|
| host heartbeat timeout 500 ms | **RX ISR 시각** vs tick | 양팔 HOLD |
| command stale (STREAMING) 100 ms | control tick | 해당 팔 HOLD → coordinated stop |
| queue underflow (TRAJECTORY) | executor | 해당 팔 ABORT → coordinated stop |
| apply lateness > 5 ms | control tick | ABORT → coordinated stop |
| servo read 실패 3연속 | bus FSM | degraded → 래치 |
| load/current 한계 2연속 | telemetry (신규: 동작 중) | 해당 팔 HOLD → coordinated stop |
| tracking error > 한계 | control tick (측정 vs 명령) | ABORT → coordinated stop |
| host TX ring overflow | host link | fail-closed, fault report, 카운트 |
| tick overrun (ISR 재진입) | tick ISR | hard latch |
| 배경 루프 정지 | **IWDG** | MCU reset → BOOT → SAFE_DISABLED |

IWDG 는 배경 루프에서만 급여한다. tick ISR 이 급여하면 루프가 죽어도 리셋이
안 걸린다. MCU reset 시 STS3215 는 토크를 유지하므로 팔은 떨어지지 않고,
부팅 상태는 `SAFE_DISABLED` 라 동작하지 않는다.

### 6.4 한 팔 fault 인데 다른 팔은 멀쩡한 경우

**"한 팔만 계속" 은 지원하지 않는다.** 수건 접기는 두 팔이 같은 천을 잡으므로
한 팔 정지는 다른 팔에게 즉시 위험이다. 독립 작업영역 단계에서도 규칙을
통일해 두는 편이 예외 경로를 줄인다.

---

## 7. 양팔 dispatch skew 최소화

### 7.1 구조적으로 0 으로 만든다

```text
[나쁨] 좌 프레임 수신 → 좌 실행 … 우 프레임 수신 → 우 실행
        skew = Pi 스케줄링 + USB 지연 = 수 ms, 측정 불가

[좋음] 하나의 프레임: arm_mask=0b11, apply_tick 공통, 12관절 payload
        → 같은 tick ISR 안에서 두 팔 보간
        → 두 DMA 를 연속 기동
        skew = 명령어 몇 개 ≈ 100 ns 미만
```

ADR-0002 가 이미 "좌우 팔의 목표는 공통 `apply_tick` 에서 한 번에 적용한다" 로
규정하고 있다. 이 설계는 그것을 구현하는 것이다.

### 7.2 남는 skew 요인

| 요인 | 크기 | 처리 |
|---|---|---|
| 두 DMA 기동 사이 명령어 | < 100 ns | 무시 가능. 필요하면 DMA 를 미리 무장하고 트리거만 동시 발행 |
| UART TX FIFO 시작 지연 | 동일 클럭·동일 설정이면 결정론적 | 동일 baud/설정 강제 |
| 서보 내부 처리 | 두 버스 동일 모델 | 상쇄 |
| **전원 도메인 차이** | 미측정 | 좌우 12 V 독립 전원 — 전압차가 응답을 바꿀 수 있다. **실측 항목** |

### 7.3 측정 방법

TIM2(170 MHz) 를 각 버스 TX 기동 시점에 캡처해 차이를 telemetry 로 보고한다.
스코프 없이 5.9 ns 분해능. 교차검증용으로 GPIO 2핀을 로직 분석기에 물린다.

### 7.4 "동시 제어" 에는 층이 넷이다

"양팔이 동시에 유기적으로 움직인다" 는 서로 다른 네 가지 요구를 한 문장에
담고 있다. 층마다 책임자가 다르다.

| 층 | 무엇 | 어디서 푸나 | 상태 |
|---|---|---|---|
| **L1 전기적 동시성** | 12개 서보가 같은 순간에 목표를 받는다 | 펌웨어 — tick ISR + 2 DMA 병렬 | **설계됨** (§7.1) |
| **L2 시간축 공유** | 두 팔이 같은 계획 지점에 있다 | 펌웨어 — 12관절 단일 queue | **설계됨** (§9.3) |
| **L3 기구학적 협조** | 두 손이 같은 물체에 대해 일관된 자세를 만든다 | **Pi — 12 DOF 계획** | **없음** (§7.5) |
| **L4 반응 결합** | 한 팔의 실제 상태가 다른 팔 명령에 반영된다 | **Pi — splice** | **없음** |

**L1·L2 만으로는 유기적이지 않다.** 두 팔이 완벽히 동기화된 채로 서로 어긋난
자세를 취할 수 있다. 유기성은 L3 가 만들고, 펌웨어는 그것을 **깨뜨리지 않는**
역할이다.

L4 를 펌웨어에 넣지 않는다 — 팔간 보상 제어는 ADR-0001 의 역할 분리를 깨고,
URDF 관성값이 미측정이라 모델도 없다. 대신 펌웨어는 **두 팔의 명령·실측을
하나의 timestamped 스냅샷**으로 보고해 Pi 가 상대 오차를 시각 불확실성 없이
계산할 수 있게 한다. 보정은 splice 로 내려온다(~27 ms, §3.5).

### 7.5 L3 의 선행 조건 — base 간 변환이 미측정이다

`so101_arm_macro.xacro` 는 이미 `prefix` / `mount_xyz` / `mount_rpy` 로
매개변수화돼 있어 두 번 인스턴스화하면 양팔 URDF 가 된다. **막는 것은 코드가
아니라 숫자다.**

| 항목 | 상태 |
|---|---|
| `so101_left.urdf.xacro` | 있음 |
| 오른팔 인스턴스 | 없음 |
| `so101_left.srdf` | 있음 (좌팔 단독 group) |
| `both_arms` planning group | 없음 |
| **오른팔 `mount_xyz` = base 간 변환** | **미측정** — 인벤토리에 "약 14 inch / 355.6 mm, 측정 필요" |

두 손이 같은 수건을 잡을 때 **base 간 변환 오차는 그대로 장력 오차가 된다.**
L3 계획이 아무리 정확해도 이 숫자가 틀리면 물리에서 어긋난다.

**측정 방법**: Top 카메라 eye-to-hand 등록([ADR-0011](adr/0011-top-eye-to-hand-gridboard.md),
`calibrate_top_base_table.py`)을 **두 팔 base 에 각각** 수행해 같은 Top frame
기준 좌표를 얻고 그 차를 취한다. 기존 도구를 그대로 쓴다. 별도 gate 로 분리한다.

#### q0와 base 변환은 같은 보정이 아니다

다음 세 값을 섞지 않는다.

| 값 | 뜻 | 결정 방법 |
|---|---|---|
| raw 2048 | 서보 전기적 중앙의 초기 기준 | 조립·READ_ONLY 확인용 |
| 팔별 `zero_raw[6]` | ROS 관절 0 rad에 대응하는 encoder 값 | 각 팔 독립 측정 |
| `mount_xyz/rpy` | 두 base 사이 외부 변환 | 같은 Top frame에서 측정 |

왼팔의 raw 2048 q0 계약은 프로젝트에서 이미 검증된 기준이지만, 오른팔에 그대로
복사할 근거는 아니다. 특히 base 등록만으로 관절 zero offset은 식별되지 않는다.
한 TCP 점은 여러 관절 offset 조합으로 설명할 수 있어 단일 자세 맞춤도 금지한다.

**F6/F6.5 보정 절차**:

1. 토크 OFF/READ_ONLY에서 각 팔의 ID, 방향, raw 범위와 기구 간섭을 확인한다.
2. 반복 가능한 조립 기준 자세에서 팔별 `zero_raw[6]` 초기값을 얻는다. gripper
   zero는 jaw open/closed 기하로 별도 보정하며 arm FK에 섞지 않는다.
3. gridboard로 Top↔left-base와 Top↔right-base를 각각 등록해
   `mount_xyz/rpy`를 먼저 고정한다.
4. 각 팔에서 특이점과 관절 한계에서 떨어진 다양한 자세를 여러 개 수집한다.
   raw 관절값과 외부에서 관측한 gripper fiducial 6D pose를 같은 timestamp로
   저장한다.
5. 링크 길이와 base 변환을 고정한 채 팔별 joint zero offset만 bounded fit한다.
   보정에 쓰지 않은 held-out 자세에서 FK 잔차와 반복 산포를 평가한다.

표본 수나 mm 임계값을 지금 추측하지 않는다. 먼저 카메라 반복성 및 현재 왼팔
held-out 기준선을 같은 절차로 측정하고, 오른팔이 그 기준선과 측정 불확실성
안에 드는지를 gate로 삼는다. 실패하면 표본 자세를 늘리거나 기구 조립을
수정하며, base 변환과 joint offset을 동시에 자유 최적화해 숫자만 맞추지 않는다.

---

## 8. 동시성 primitive 배정

**단일 코어이고, 각 자료구조의 생산자·소비자가 각각 하나뿐이다. 따라서 mutex 가
필요 없다.** 이것이 비-RTOS 설계의 가장 큰 실익이다.

| 자료 | 생산자 → 소비자 | primitive | 근거 |
|---|---|---|---|
| host RX 바이트 | LPUART ISR → 배경 루프 | **SPSC 바이트 ring** (lock-free) | 인덱스 각각 한쪽만 쓴다 |
| host TX 바이트 | 배경 루프 → TX DMA ISR | **SPSC 바이트 ring** | 넘치면 fail-closed |
| setpoint queue | 배경 루프(admission) → tick ISR | **SPSC ring** (head/tail 분리) | 현재 `head`/`count` 구조는 SPSC 안전하지 않음 → 수정 필요 |
| servo RX 바이트 | DMA → bus FSM | **순환 DMA ring + 절대 producer index** | 이미 구현돼 있고 올바르다 |
| telemetry 스냅샷 | tick ISR → 배경 루프 | **double buffer + seqlock** | 인터럽트 금지 없이 일관 읽기 |
| 명령/측정 위치 | 양방향 | **관절별 `int32` 원자 접근** | M4 에서 정렬된 32-bit 접근은 원자적 |
| fault 플래그 | 양쪽 set | **LDREX/STREX 원자 OR** | set-only, clear 는 배경 루프 단독 |
| ISR↔ISR 다중워드 | — | `__disable_irq` 임계구역 | **5 µs 미만**으로 제한, 회귀로 고정 |
| task notification / mutex / semaphore | — | **사용 안 함** | RTOS 미도입 |

---

## 9. Pi ↔ STM32 프로토콜 변경

현행: COBS + 16 B 헤더 + payload + CRC-32C, magic `0xA55A`, version 1.
`ACTUATOR_BUFFERED_COMMAND_UNSUPPORTED_RIGHT_SLOT` 로 우 슬롯이 예약만 돼 있다.

### 9.1 변경 목록

| # | 변경 | 이유 |
|---|---|---|
| 1 | `arm_mask` 우 슬롯 활성화. `0b01`/`0b10`/`0b11` = **유효한 열** 선언 | 양팔 명령 |
| 2 | sample 은 **항상 12관절**(gripper 2개 포함) | 공통 `apply_tick` = skew 0, 동기 이탈 불가 (§9.3) |
| 3 | `STREAM_OPEN` 에 `actuator_stream_policy_t` | Track A/B 를 mode 가 아니라 **선언된 정책**으로 구분 (§3A A1) |
| 4 | `horizon_end_tick` (0 = 열린 stream) | 종료·고갈·타임아웃 셋을 필드 하나로 (§3.2) |
| 5 | `SPLICE` 명령 (`splice_at_tick` + sample) | 반응 지연 하한. 수렴 보정과 RL 갱신이 같은 연산 (§3.5) |
| 6 | 배치마다 `arbiter_epoch` (uint32) | 소유자 교체 시 연속성 검사 강제 (§3A A2) |
| 7 | status payload 에 팔별 블록 | 기존 16/32/60 에 양팔 크기 추가. host 는 크기로 분기 |
| 8 | status 에 `sender_time_ms` 에코 | **command latency 를 host 가 직접 계산** — 헤더에 이미 필드가 있다 |
| 9 | telemetry 에 관절별 load/current/temp + **`sample_age_ms`** | 동작 중 진단, RL 의 stale 판정 (§3A A4) |
| 10 | status 에 `minimum_queued_samples`, `horizon_open_underrun_count` | host 의 lead 계약 위반을 보이게 (§3A A5) |
| 11 | HELLO/manifest에 `left_calibration_hash`, `right_calibration_hash` | 팔별 zero/방향/범위/게인을 독립 검증 |
| 12 | `ACTUATOR_PROTOCOL_VERSION` → **2** | 와이어 변경. host 를 동시에 이관 |

**만들지 않는 것**: residual 플래그, 두 번째 stream 슬롯, "mode" 필드.
자리를 남기면 반드시 쓰이고, 그 순간 펌웨어가 출처를 알게 된다 (§3A A2).

### 9.2 버전 정책

v1/v2 동시 지원은 하지 않는다. 단일 사용자 시스템이고
`tools/validate_protocol_manifest.py` + 생성 헤더가 host/firmware 동기를
강제하므로 lockstep 이관이 더 싸다. HELLO 응답의 version 불일치는 fail-closed.

### 9.3 queue 구조 — 팔별 2개가 아니라 12관절 1개 (2026-08-11 정정)

처음에는 팔당 queue 를 하나씩 두려 했다. **바꾼다.**

| | 팔별 queue 2개 | **12관절 queue 1개** (채택) |
|---|---|---|
| 두 팔이 같은 계획 지점에 있음 | **관례로 보장** (host 가 같은 tick 을 넣어야) | **구조로 보장** — 한 sample 이 곧 전신 자세 |
| 한쪽만 underflow | 가능 → 한 팔 정지·한 팔 진행 (검출까지 최대 1 tick 발산) | **불가능** |
| 팔간 충돌 검사 | 두 독립 계획이라 계약 밖 | 하나의 12 DOF 계획 → MoveIt 이 검사 |
| 독립·서로 다른 길이의 동시 궤적 | 가능 | **불가능** (§9.4) |

sample 정의:

```c
#define ACTUATOR_ARM_COUNT      2u
#define ACTUATOR_ARM_JOINTS     6u
#define ACTUATOR_JOINT_COUNT    (ACTUATOR_ARM_COUNT * ACTUATOR_ARM_JOINTS)  /* 12 */

typedef struct {
    uint32_t apply_tick;
    int32_t  position_urad[ACTUATOR_JOINT_COUNT];   /* [0..5]=L, [6..11]=R */
} actuator_setpoint_t;
```

배치 헤더의 `arm_mask` 는 **어느 팔의 열이 유효한지**를 선언한다. 유효하지
않은 팔의 관절은 마지막 명령값을 유지한다. 한 팔만 움직이는 경우도 같은
queue·같은 executor 를 쓴다.

이것은 로드맵 D.4 "공유 영역은 두 독립 계획이 아니라 하나의 충돌 검사 계약으로
실행한다" 와 같은 결정이다.

### 9.4 포기하는 것 — 명시

**두 팔이 서로 다른 길이의 독립 궤적을 동시에 실행하는 것은 지원하지 않는다.**

필요해지면 짧은 쪽을 마지막 자세로 채워 같은 길이로 만든다. 대역폭 손해는
있으나(항상 12관절 전송) 921600 에서 refill 배치가 `~10 ms` 라 문제되지 않는다.

수건 접기와 협조 조작은 애초에 하나의 12 DOF 계획이므로 이 제약이 걸리지 않는다.

### 9.5 유지하는 것

- COBS + CRC-32C, 16 B 헤더 배치
- queue 용량 16 (20 ms × 16 = 320 ms), 배치당 최대 9 sample
- 팔별 calibration hash로 zero·방향·게인·home·범위를 핀하는 불변식. v1의 단일
  hash는 F5까지 유지하고 v2에서 좌/우 두 필드로 원자 이관한다. 하나라도 다르면
  ARM/ENABLE을 거부한다
- **팔별로 갈리는 것**: 관절 한계, home/방향/raw 범위, 게인, 토크, 처짐.
  이것들은 `arm_context[]` 에 남는다 — 공통이 되는 것은 **시간축과 queue** 뿐이다

보정값을 바꾸면 해당 팔 hash, 생성 헤더, 활성 manifest와 테스트 fixture를 한
변경으로 갱신한다. 예전 실기 artifact는 수정하지 않고 superseded로 남기며,
현재 calibration으로 재생하려 하면 hash mismatch로 fail-closed되는 것이 정상이다.

---

## 10. 디렉터리 구조

```text
firmware/
├── actuator_core/                  ← 이식 가능. HAL 의존 없음. host 에서 단위시험
│   ├── include/actuator_core/
│   │   ├── protocol/   cobs.h crc32c.h framing.h message_ids.h
│   │   ├── motion/     setpoint_queue.h interpolator.h rate_limiter.h
│   │   │               executor.h stream_mode.h command_route.h
│   │   ├── safety/     arm_safety.h system_safety.h fault_codes.h
│   │   └── calib/      calibration.h joint_map.h
│   ├── src/            (동일 구성)
│   └── tests/          host 단위시험 — 현 자산 유지·확장
│
└── stm32_g474_dual_arm/            ← CubeIDE 프로젝트
    └── Core/
        ├── Inc|Src/app/
        │     app_main.c            배경 super-loop
        │     control_tick.c        TIM6 ISR — 이 파일이 유일한 hard RT 코드
        │     command_router.c      arm_mask 분배
        │     telemetry.c           집계·인코딩
        │     arm_context.c         팔별 상태 묶음
        ├── Inc|Src/platform/
        │     host_link.c           LPUART1 RX ISR + TX DMA ring
        │     servo_uart.c          UART 인스턴스 래퍼 (TX DMA / RX 순환 DMA)
        │     timebase.c            TIM6 tick + TIM2 µs 카운터
        │     metrics.c             jitter/skew/이용률 히스토그램
        │     gpio_probe.c          계측 핀
        ├── Inc|Src/drivers/
        │     sts3215_packet.c      패킷 조립·파싱 (HAL 무관, 시험 가능)
        │     servo_bus.c           `servo_bus_t` 인스턴스 FSM
        └── Inc/config/
              board_config.h        핀·UART·DMA·타이머 배정
              arm_left_config.h     ID, home, 방향, 범위, 게인, 토크
              arm_right_config.h    동일 (독립 값 — 두 팔 처짐은 다르다)
```

### 10.1 규칙

- `actuator_core` 에 HAL·RTOS·인터럽트 개념이 들어가지 않는다. 이 경계가 host
  단위시험을 살린다
- `control_tick.c` 는 **hard real-time 코드가 사는 유일한 파일**이다. 여기에
  추가되는 모든 것은 최악 실행시간을 다시 재야 한다. 파일 상단에 그 규칙과
  현재 실측값을 적는다
- 팔 설정은 **파일이 갈린다.** `#if ARM == LEFT` 로 분기하지 않는다
- `servo_bus.c` 의 파일 static 전역(`servo_uart_handle`, 진단, ring)은 전부
  `servo_bus_t` 필드로 이동한다 — 이번 전환에서 가장 큰 기계적 작업

---

## 11. 구현 순서

각 단계는 **그 자체로 동작하는 로봇**을 남기고, **자체 gate 로 검증**하며,
**왼팔 기준선을 후퇴시키지 않는다.**

### F0 — 계측 먼저 (동작 변경 없음)

- TIM2 free-running µs 카운터, GPIO 계측 핀
- 히스토그램/최댓값 수집: 루프 주기, tick 간격, servo 트랜잭션 시간, host TX 시간
- terminal 프레임으로만 보고 (기존 규칙 유지)

**Gate**: 현재 펌웨어의 before 수치가 artifact 로 남는다. 기존 물리 시험 전부 통과.

> 개선을 주장하려면 before 가 있어야 한다. 위험 0, 가치 최대.

### F1 — heartbeat 를 RX 시각에 기록

- `host_binary_last_heartbeat_ms` 를 처리 시각 → **RX ISR 시각**으로

**Gate**: 배경 루프에 400 ms 인공 정체를 주입해도 watchdog 이 래치되지 않음.
(현재 구조는 래치된다.)

> 세 번 조용히 깨졌던 불변식을 구조적으로 제거한다. 비용 대비 가치 최고.

### F2 — host TX 비동기 (DMA + ring, overflow fail-closed)

**Gate**: `0x00022800` 에서 실패했던 **60 B lateness 히스토그램을 refill 응답에
다시 실어도** apply lateness 가 변하지 않음. 실패했던 실험을 그대로 재현해
통과시킨다. `0.312 ms` 절벽 소멸.

### F2.5 — 프로토콜 v2: 링크 921600 + splice + stream 정책

**진입 gate**: v2 필드를 동결하기 전에 로봇을 움직이지 않는 ST-LINK VCP
loopback/echo 시험으로 계획된 최악 왕복 traffic의 2배 이상 goodput과 30분
frame loss·CRC error·TX overflow 0을 확인한다. 실패하면 USB CDC 직결 또는
별도 USB-UART를 먼저 결정하고, USB stack을 넣는 경우 ADR-0014 §6 조건 5에 따라
RTOS 판단도 다시 연다.

**v2 의 모든 요소를 한 번에 결정한다.** 나눠서 넣으면 v3 가 된다 (§3A 요약).

- LPUART1 921600. refill 배치 `43.2 → 5.4 ms`, splice `16.1 → 2.0 ms`
- `SPLICE` 명령 (§3.5): 연속성 검사 3개, 지평 부분 교체
- `actuator_stream_policy_t` (§3A A1): 상수 3등급 분리, 하드 캡 대비
  **조이기만 허용**, 느슨한 값은 `STREAM_OPEN` 거부
- `#error` 를 하드 캡 관계로 이전. 분할 가능성 guard 삭제, lateness 정의를
  "첫 due tick 기준" 으로 변경
- `arbiter_epoch` (§3A A2), `horizon_end_tick` (§3.2)
- `tracking_error_limit_urad[]` 를 **관절별**로 (§3A A3 — gripper 파지 abort 방지)
- `MINIMUM_LEAD_MS` `60 → 20` (splice 한정)

**Gate**:
- **보정 반응 지연 < 30 ms 실측.** 수렴 보정이 팔을 멈추지 않고 수행됨
- host 회귀: **정책 필드를 하나씩 하드 캡보다 느슨하게 만들어 전부 거부**
- `arbiter_epoch` 변경 시 불연속 splice 가 거부됨을 주입으로 확인
- STALE_TICK 0

> 앞서 이것을 이전 초안의 마지막 단계에 뒀던 것은 틀렸다. `PLAN_CONTINUOUS_EXECUTION.md`
> §3.4 의 예산표대로 링크 속도가 반응 지연의 1차 제약이다.

### F3 — 제어 tick 을 TIM6 ISR 로

- 보간·한계·속도제한·sync-write 기동을 ISR 로 이동
- `timebase_now_ms()` 를 TIM6 기준으로 단일화

**Gate**: tick jitter p99 < 100 µs, max < 250 µs. apply lateness 히스토그램이
bucket 0 으로 수렴. 배경 루프에 인공 정체를 줘도 tick 이 밀리지 않음.

### F4 — 서보 TX DMA + 트랜잭션 FSM (아직 한 팔)

- spin-wait 제거, 5 ms 슬롯 시간표 도입

**Gate**: 제어 경로 blocking 대기 0. **buffered 실행 중 관절별 load/current 가
30 ms 주기로 관측됨** — `FIRMWARE_ASYNC_RATIONALE` §8 이 요구한 진단 능력 확보.
여기서 **백래시 가설 검증 데이터를 수집한다** (4.2절).

### F5 — 인스턴스화 (아직 한 팔 구동)

- `servo_bus_t`, `arm_context_t`. 전역 static 제거. 두 인스턴스 컴파일, 우팔 미연결

**Gate**: 좌팔 동작이 F4 와 동일. calibration hash 불변. 회귀 전부 통과.
**순수 리팩터링이므로 동작이 바뀌면 그것이 버그다.**

### F6 — 우팔 하드웨어 (UART4 + 단독 검증)

- 오른팔 전용 Waveshare driver R의 논리측에 CN7-1 PC10/UART4_TX →
  driver RX, CN7-2 PC11/UART4_RX ← driver TX, 공통 GND로 교차 연결한다.
  먼저 무전원 continuity/short를 확인하고 servo 전원을 분리한 채 driver
  논리 high 출력과 STM32 입력 호환성을 실측한다
- 로드맵 C절: 우팔이 좌팔과 **같은 단독 gate를 독립 값으로 통과**한다.
  servo ID·방향·범위·q0·토크·PID 실측, READ_ONLY, 단일 관절 → 전체 →
  gripper → home → cancel/fault
- F6에서 얻은 `right_calibration_hash`를 manifest/HELLO에 고정한다

**Gate**: 우팔 단독 동등성. 좌팔 값을 복사하지 않고 §7.5 절차와 held-out
FK 잔차로 독립 검증. 물리 시험에서 DISABLE하거나 bridge를 종료하기 전에는
반드시 팔을 손으로 지지한다.

### F6.5 — base 간 변환 측정과 양팔 모델 (하드웨어만, 펌웨어 무관)

- Top eye-to-hand 등록을 **두 팔 base에 각각** 수행 → `mount_xyz` / `mount_rpy` 확정
- `so101_dual.urdf.xacro` (매크로 2회 인스턴스화), `both_arms` SRDF group
- 팔간 collision 검사 활성화

**Gate**: 보정에 쓰지 않은 자세에서 두 팔 FK의 gripper 상대 자세와 외부
실측을 비교한다. 카메라 반복성·왼팔 기준선·오른팔·양팔 상대 잔차를 같은
artifact에 기록하고, 사전에 측정한 기준선 밖이면 통과시키지 않는다.
**이 숫자 없이 L3 협조 계획은 의미가 없다** (§7.5). F7과 병행 가능하되
F7.5의 선행조건이다.

### F7 — 양팔 dispatch + coordinated stop

- 공통 `apply_tick` 12관절 프레임(단일 queue), 팔별/시스템 안전 FSM

**Gate**: **L/R skew p99 < 50 µs** (TIM2 실측). 한 팔 궤적 실행 중 다른 팔에
fault 를 주입 → 두 팔 모두 HOLD, 어느 팔도 계속 움직이지 않음.

### F7.5 — L3 협조 계획 (Pi)

- `both_arms` group 으로 **하나의 12 DOF 궤적** 계획. 팔간 충돌 검사 포함
- 물체를 함께 잡는 동작은 **물체 공간에서 계획**하고 고정 grasp 변환으로
  각 gripper 자세를 유도한 뒤 팔별 IK — 관절 공간에서 두 궤적을 맞추지 않는다

**Gate**: 두 gripper 상대 자세가 궤적 전 구간에서 계약 허용치 안. 물체 없이
먼저, 그다음 물체.

### F8 — 열린 stream (Track B) + 연속 telemetry

- `horizon_end_tick` 미설정 경로, command timeout 100 ms, 양팔 연속 telemetry

**Gate**: host 단위시험 + Pi 에서 20 Hz 합성 스트리밍 소스로 한 팔 구동
(shadow → 제한 실기). stale command 검출을 주입으로 확인. 링크 이용률 < 50 %,
TX overflow 0, 30 분 무오류.

### 순서의 근거

- **host 작업(H1~H3)이 F 단계 전체보다 먼저다.** 측정 구간의 약 97%가
  펌웨어 밖에서 소비됐고, 그중 프로세스·정착 오버헤드는 펌웨어 변경 없이
  줄일 수 있다 (`PLAN_CONTINUOUS_EXECUTION.md` §1)
- **F1~F4 는 양팔과 무관하게 현재 한 팔 로봇을 개선한다.** 양팔 하드웨어가
  늦어져도 낭비되지 않는다
- **F2.5 가 연속 실행의 반응성을 완성한다.** 수렴이 팔을 멈추지 않게 된다
- **F4 가 흔들림 진단 데이터를 준다.** 양팔을 기다릴 필요가 없다
- **F5 를 F6 앞에 둔다.** 리팩터링과 새 하드웨어를 같은 단계에 섞으면 실패
  원인이 갈리지 않는다
- **F8 을 뒤에 둔다.** 열린 stream 은 지평 기반 위에 얹히므로 기반이 측정으로
  확정된 뒤에 붙인다

---

## 12. 측정 metric

전부 TIM2(5.9 ns) 기반. 히스토그램 + 최댓값으로 보고하며, 최댓값 하나만
남기지 않는다 (`0x00022800` 교훈: 단일 최댓값은 스파이크와 계통 편차를 구분 못 함).

| metric | 정의 | 수집 | 목표 |
|---|---|---|---|
| **L/R dispatch skew** | 두 버스 TX 기동 TIM2 차 | tick ISR | p99 < 50 µs |
| **control-loop jitter** | 연속 tick ISR 진입 간격 − 5 ms | tick ISR | p99 < 100 µs, max < 250 µs |
| **command latency** | host `sender_time_ms` → 최초 적용 tick | status 에코, Pi 계산 | 중앙값 < 1 sample 주기 |
| apply lateness | 기존 히스토그램 (F2 이후 확장 안전) | executor | 100 % bucket 0 |
| **feedback latency** | telemetry 요청 → 파싱 완료 / 관절별 표본 나이 | bus FSM | 전주기 < 40 ms/팔 |
| **bus utilization** | Σ 트랜잭션 µs / 창 µs, 버스별 | bus FSM | < 30 % |
| host link utilization | (TX+RX 바이트) / (baud/10) | host link | < 50 % |
| **UART timeout/error** | 기존 `ServoBusHealth` 카운터, 팔별 | bus FSM | 30 분 0 |
| **stale command** | STREAMING 목표 나이 > 1 주기인 tick 수 | tick ISR | 0 |
| **queue overflow/underflow** | queue 카운터 + TX ring 최고수위 | queue / host link | 0 |
| ISR 최악 실행시간 | tick ISR 진입~퇴출 최댓값 | tick ISR | < 500 µs |
| tick overrun | 이전 tick 미완료 상태 재진입 횟수 | tick ISR | 0 |
| 배경 루프 최악 주기 | 루프 1회전 최댓값 | 배경 루프 | < 20 ms |

### 12.1 산출물

**단계별 before/after 표 하나.** F0 에서 잰 현재 값이 왼쪽 열, 각 단계 뒤가
오른쪽 열. 이것이 이 작업의 데모 산출물이다 — "FreeRTOS 를 썼다" 보다
"jitter 를 X → Y 로 줄였고 여기 숫자가 있다" 가 강하다.

`docs/test-results/` 규칙을 따라 단계마다 machine-readable artifact 를 남긴다.

---

## 13. 위험과 완화

| 위험 | 완화 |
|---|---|
| 리팩터링 중 좌팔 기준선 후퇴 | F5 를 순수 리팩터링으로 격리. calibration hash 불변을 gate 로 |
| tick ISR 이 비대해져 결정성 상실 | `control_tick.c` 를 유일한 hard RT 파일로 못박고, ISR 최악 실행시간을 회귀 metric 으로 |
| SPSC queue 경합 버그 | `actuator_core` 안에 두고 host 단위시험 + 인위적 인터리브 시험 |
| 좌우 전원 전압차가 skew/응답 차이를 만듦 | F6 에서 실측 항목으로 명시. 하드웨어 인벤토리의 "실제 출력 전압 측정 필요" 와 연결 |
| ST-LINK VCP 처리량 한계 | F2.5 설계 동결 전에 921600 왕복 부하를 실측. 부족하면 G474 USB FS 직결 또는 별도 USB-UART 검토 |
| 물리 E-stop 부재 | 하드웨어 인벤토리에 "양팔 단계 전 필수 검토" 로 이미 기록됨. F7 전에 결정 |
| 프로토콜 v2 이관 중 host/firmware 불일치 | 생성 헤더 + manifest 검증 + HELLO version fail-closed |

---

## 14. 이 설계가 지키는 규율

- **RTOS 도, DMA 도 목적이 아니다.** 각 항목이 어떤 실측된 문제를 없애는지 적었다
- **before 없이 after 를 주장하지 않는다.** F0 이 첫 단계인 이유
- **고치지 못하는 것을 고친다고 하지 않는다.** 흔들림은 4절에서 범위 밖으로 명시
- **단계마다 로봇이 동작한다.** 양팔 하드웨어를 기다리는 단계가 없다
- **fork 하지 않는다.** Track A/B 는 mode enum 하나로 갈리고 실행 경로는 공통
- **팔은 대칭이되 값은 독립이다.** 설정 파일이 갈리고, 두 팔의 처짐은 다르다
