# ADR-0014: FreeRTOS 미채택과 시간구동 제어 코어

- 상태: 제안
- 날짜: 2026-08-11
- 관련: [ADR-0001](0001-system-partition.md), [ADR-0002](0002-motion-time-ownership.md),
  [FIRMWARE_DUAL_ARM_ARCHITECTURE.md](../FIRMWARE_DUAL_ARM_ARCHITECTURE.md)

## 결정

양팔 전환에서 **FreeRTOS 를 채택하지 않는다.** 제어 tick 을 하드웨어 타이머
ISR 에 두고, 나머지를 협조적 배경 루프에 둔다.

재검토 조건을 §6 에 명시하고, 채택 시 이행 대상 구조를 §5 에 남긴다.

---

## 1. 먼저 — 유효하지 않은 논거를 버린다

RTOS 반대 논거로 흔히 드는 것들이 이 하드웨어에서는 성립하지 않는다.
실측(`build/stm32_g474_single_arm-0x225-release-make`):

| 자원 | 현재 사용 | 총량 | FreeRTOS 추가분(추정) | 추가 후 |
|---|---|---|---|---|
| flash (`text`) | 39.5 KB | 512 KB (7.7 %) | 커널 6~10 KB | ~9.7 % |
| RAM (`bss`) | 5.9 KB | 128 KB (4.6 %) | task 6개 stack+TCB ~6 KB, heap ~2 KB | ~11 % |
| CPU | 5 ms slot 사용률 10 % 미만 | — | tick 1 kHz + context switch ~1~2 % | 여전히 여유 |

**자원은 반대 이유가 아니다.** "MCU 가 작아서" 라는 서술은 이 보드에서 거짓이다.
따라서 아래 논거만 실제로 결정에 관여한다.

---

## 2. 결정적 논거 — 커널이 보호해야 할 것을 보호하지 못한다

Cortex-M FreeRTOS 포트의 규칙:

- FreeRTOS API 를 호출하는 ISR 은 우선순위가
  `configMAX_SYSCALL_INTERRUPT_PRIORITY` **이하**(숫자상 이상)여야 한다
- `taskENTER_CRITICAL` 은 `BASEPRI` 를 그 값으로 올려 **해당 ISR 전부를 마스킹**한다
- 그보다 높은 우선순위 ISR 은 절대 마스킹되지 않지만, **FreeRTOS API 를 하나도
  호출할 수 없다**

우리가 지켜야 하는 것은 5 ms 제어 tick 이다. 두 갈래뿐이다.

```text
(a) tick ISR 을 커널 인지 우선순위에 둔다
    → task 에 notify 할 수 있다
    → 그러나 커널·사용자 코드의 모든 critical section 이 tick 을 지연시킨다
    → 지연 상한이 "우리가 리뷰한 코드" 밖(커널 내부)에 있다

(b) tick ISR 을 커널 위 우선순위에 둔다
    → 커널이 절대 지연시키지 못한다
    → 그러나 FreeRTOS API 를 호출할 수 없다
    → 필요한 일을 ISR 안에서 전부 해야 한다
    → 그것이 곧 "RTOS 없는 설계"다
```

**(b) 가 옳은 선택이고, (b) 를 택하는 순간 실시간 부분에서 커널이 하는 일이
없다.** 남는 것은 배경 작업뿐인데, 배경 작업에는 선점이 필요 없다(§3).

### 2.1 크기는 어느 정도인가 — 과장하지 않는다

(a) 를 택했을 때 추가되는 jitter 는 critical section 길이 수준으로 보통
**1~5 µs** 다. 5 ms 주기 대비 **0.1 % 미만**이며, 이 로봇에서 수치상 치명적이지
않다.

문제는 크기가 아니라 **상한을 우리가 증명할 수 없다**는 점이다. 현재 설계의
tick 지연 상한은 "NVIC 지연 + 자기 실행시간" 으로 소스에서 유도된다. (a) 는
"+ 커널 내부와 모든 `taskENTER_CRITICAL` 의 최댓값" 이 되고, 이 저장소가
지금까지 쓰던 **소스 상수로부터 예산을 계산하는 방식**이 성립하지 않는다.

---

## 3. 배경 작업에 선점이 필요 없다

전환 후 배경 루프의 최악 1회전을 항목별로 본다.

| 작업 | 최악 |
|---|---|
| 프레임 디코드 (COBS + CRC-32C, ~500 B) | 10~20 µs (G4 CRC 유닛 사용 시 더 짧다) |
| 배치 admission (9 sample × 12 관절 한계·단조 검사 + 복사) | 20~50 µs |
| splice 연속성 검사 | ~10 µs |
| status 인코딩 | ~20 µs |
| telemetry 집계 + 안전 판정 | ~50 µs |
| **합계** | **약 100~200 µs** |

sample 주기 `20 ms` 대비 **1 % 수준**이다.

> 지금 배경 루프가 host 를 굶기는 이유는 계산량이 아니라 **blocking I/O** 다.
> DMA 로 그것을 없애면 굶길 계산이 남지 않는다. 선점의 값어치는 "길고 급하지
> 않은 계산이 짧고 급한 것을 막을 때" 나오는데, 그런 계산이 없다.

관측된 실패 3건(`0x00022500`/`0x00022600`/`0x00022800`)이 전부 blocking I/O
였다는 사실이 같은 이야기다. **RTOS 는 그중 어느 것도 고치지 못했을 것이다.**

---

## 4. 양팔 동기화는 task 분리를 반대한다

직관적으로는 팔 하나에 task 하나가 자연스럽다. 그러나 두 팔은 **시간축에서
독립이 아니다** — 같은 순간에 dispatch 해야 한다(ADR-0002).

| 구조 | dispatch skew |
|---|---|
| 단일 tick ISR 에서 두 DMA 연속 기동 | 명령어 몇 개, **~50~100 ns** |
| task 2개 + rendezvous | context switch 1회 이상, **~1~2 µs** |

**두 경우 다 목표치(50 µs) 안이므로 실격 사유는 아니다.** 다만 task 분리는
양팔에서 가장 중요한 지표를 10~40배 나쁘게 만들면서 아무것도 돌려주지 않는다.
"팔마다 task" 는 이 시스템에서 잘못된 분해다.

---

## 5. RTOS 가 실제로 더 나은 것 — 공정하게

반대 논거만 쓰면 정직하지 않다. 진짜 이점 셋이 있다.

### 5.1 순차 코드 (실질적 이점)

서보 트랜잭션과 복구는 본질적으로 순차적이다.

```c
/* RTOS: 약 20줄 */          /* 무 RTOS: 상태 + deadline 변수 */
send(request);               case TX_REQUEST: ...
if (!wait(reply, 2ms))       case WAIT_REPLY:  if (deadline_expired) ...
    recover();               case RECOVER:     if (quiet_elapsed) ...
parse(reply);                case PARSE:       ...
```

`servo_bus.c` 1 948 줄 중 트랜잭션·복구가 약 600 줄이고, FSM 화하면 30~50 %
늘어난다. 실재하는 비용이다.

**반론 두 가지.** (1) 타이머 ISR 설계에서는 어차피 FSM 이 필요하다 — RTOS 를
써도 tick 이 기동한 전송의 상태는 명시적이어야 한다. (2) **FSM 상태는
telemetry 로 관측 가능하지만 blocking task 의 상태는 스택 안에 있어 보이지
않는다.** 이 저장소는 `ServoBusDiagnostics`·failure snapshot 으로 실패를
사후 재구성해 온 이력이 있고, 그 능력을 잃는 것은 가볍지 않다.

### 5.2 blocking 초기화 경로의 격리 (가장 강한 이점)

`Servo_ConfigureAllForTrajectory` 에는 `HAL_Delay(20/50/500)` 이 있다. 무 RTOS
설계에서는 ARM/ENABLE 동안 배경 루프가 **600 ms 이상 정지**한다.

현재는 문제되지 않는다 — heartbeat 는 RX ISR 이 찍고(F1), tick ISR 은 executor
가 idle 이라 출력하지 않는다. **그러나 "한 팔이 움직이는 중에 다른 팔을 ARM
한다" 는 불가능해진다.**

지금 계획에서는 coordinated stop 이 두 팔을 함께 멈추고 함께 재무장하므로
필요가 없다. 필요해지면 그 시퀀스를 deadline 기반 FSM(약 5개 상태)으로 바꾸면
되고, 그것이 커널 도입보다 싸다. **다만 이것이 무 RTOS 설계가 실제로 포기하는
능력이며, 여기 기록해 둔다.**

### 5.3 미래 스택

USB CDC 직결, CAN, SD 로깅 등은 자체 task 를 전제하는 미들웨어를 데려온다.
그때는 재검토 대상이다(§6).

---

## 6. 재검토 조건 (falsifiable)

아래 중 **하나라도 실측**되면 이 ADR 을 다시 연다. 그 전에는 열지 않는다.

1. 배경 루프 최악 1회전 > `20 ms` (sample 주기)
2. tick ISR 최악 실행시간 > `500 µs` (주기의 10 %)
3. 서로 blocking 하는 독립 활동이 동시에 3개 이상 필요
4. 한 팔 동작 중 다른 팔의 blocking 초기화가 실제로 필요해짐 (§5.2)
5. 자체 task 를 요구하는 미들웨어 도입 (§5.3)

1·2 는 `metrics.c` 가 상시 수집하므로 **자동으로 감시된다.**

---

## 7. 선택지 비교

| | **A. 타이머 ISR + 협조 루프** (채택) | B. 타이머 ISR + 배경만 RTOS | C. 순수 FreeRTOS (tick = task) |
|---|---|---|---|
| tick jitter 상한 | NVIC 지연 + 자기 실행시간. **소스에서 유도 가능** | 동일 (tick 이 커널 위) | 커널 tick + 스케줄러 + critical section |
| 양팔 skew | ~50~100 ns | ~50~100 ns | ~1~2 µs |
| 컴파일타임 `#error` 예산 | **유지** | 대체로 유지 | 성립 안 함 |
| 서보 FSM 코드량 | FSM 필요 | FSM 필요 (task 화 가능) | 순차 가능 |
| blocking 초기화 격리 | 안 됨 (§5.2) | **됨** | 됨 |
| `actuator_core` host 시험 | **유지** | 규율 필요 | 규율 필요 |
| 새 실패 모드 | 없음 | stack overflow, 우선순위 역전 | 동일 + tick 지연 |
| 자원 | — | +9 KB RAM (총 11 %) | 동일 |

**B 는 합리적인 중간안이다.** RTOS 를 쓰기로 한다면 C 가 아니라 B 여야 한다 —
tick 을 `configMAX_SYSCALL_INTERRUPT_PRIORITY` **위**에 두어 커널 밖에 남기고,
서보 FSM·ARM 시퀀스·host 파싱만 task 로 옮긴다. §2 의 딜레마를 (b) 로 풀면서
§5.1·§5.2 의 이점을 얻는다.

**A 를 택하는 이유는 B 가 틀려서가 아니라, B 가 사는 값(커널 + 새 실패 모드
+ 예산 불변식 약화)에 비해 얻는 것(§5.1 코드량, §5.2 지금은 불필요한 능력)이
현재 작으므로다.** §6 의 조건 4가 발동하면 B 로 간다.

---

## 8. 결과

- 제어 tick 은 TIM6 ISR. 보간·한계·속도제한·dispatch 만 수행
- 배경 루프: 파싱·admission·telemetry·안전 판정·IWDG
- 모든 공유 자료는 SPSC(생산자·소비자 각 1) → **mutex 불필요**
- `#error` 기반 컴파일타임 예산 불변식 유지
- §6 조건 1·2 를 `metrics.c` 가 상시 감시
- 이 판단이 틀렸다면 그것은 §6 의 실측으로 드러난다. 취향이 아니라 숫자로 뒤집는다
