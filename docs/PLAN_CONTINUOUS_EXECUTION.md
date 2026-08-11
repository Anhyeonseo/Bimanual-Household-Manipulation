# 연속 실행 아키텍처 — dead time 을 없애는 계획

- 기준일: 2026-08-11
- 목표: **연산이 끝나는 즉시 동작이 나간다.** leg 사이 정지·대기를 구조적으로 제거한다
- 이 문서가 상위 계획이고, [FIRMWARE_DUAL_ARM_ARCHITECTURE.md](FIRMWARE_DUAL_ARM_ARCHITECTURE.md)
  가 그 안의 펌웨어 부분이다
- 검증된 계약을 깨는 변경을 포함한다 (사용자 승인)

---

## 1. 먼저 — 시간이 어디서 죽고 있나

`docs/test-results/evidence/2026-08-07-repeat-stop-diagnosis-34-leg.json`
(34 leg, Pi 실기)에서 leg 사이 간격을 직접 계산했다.

| 구간 | n | 중앙값 | 최소 | 최대 |
|---|---|---|---|---|
| `pregrasp → grasp` | 10 | **4 137 ms** | 3 848 | 4 311 |
| `grasp → q0_return` (gripper + 수렴 포함) | 10 | **23 189 ms** | 22 880 | 23 600 |
| `q0_return → pregrasp` | 9 | **5 659 ms** | 5 459 | 5 936 |

중앙값은 짝수 표본에서도 가운데 두 값의 평균을 쓰는 표준 median이다.

같은 기록의 `precompute_ms` — 즉 **실제 계획 연산 시간** — 은 `3.7 ~ 330 ms` 다.

> **간격의 95 % 이상이 연산이 아니다.**

### 1.1 4.2 초 안에 무엇이 있나

`run_pick_place_once.py` → `move_to()` 는 leg 하나마다 **파이썬 subprocess 를
3개 띄운다.**

```text
leg 1개 =
  read_joint_state()                    ROS 노드 생성 + /joint_states 5 Hz 대기
  ros_moveit_plan_pregrasp_segments.py  프로세스 + rclpy init + MoveIt 탐색 + 계획
  plan_buffered_segment_leg.py          프로세스 + numpy import (계산 78~330 ms)
  execute_buffered_segment_leg_once.py  프로세스 + rclpy init + action 탐색
                                        + prime 120 ms + lead 220 ms
  post-settle                           polling 0.1 s, 2회 연속, 상한 2.5~10 s
```

측정된 4.2 초의 구성 추정:

| 항목 | 크기 | 성격 |
|---|---|---|
| post-settle | 0.3 ~ 2.5 s | **모든 leg 에 걸림** |
| ROS 노드 생성 × 3 + action/service 탐색 | 1.5 ~ 3 s | 프로세스 구조 |
| prime sleep + first sample lead | 0.34 s | 펌웨어 계약 |
| **실제 계획 연산** | **0.08 ~ 0.33 s** | 유일하게 필요한 시간 |

### 1.2 생산 속도에서는 더 나쁘다

위 기록은 tracking rate `50 raw/s` 라 동작 자체가 길었다(38/26/56 s).
현재 운용값은 `250~300 raw/s` 이므로 동작은 약 6배 짧아지지만 **dead time 은
그대로다.**

| | 동작 | dead time | dead 비율 |
|---|---|---|---|
| 기록 당시 (50 raw/s) | 120 s | 33 s | 22 % |
| **현재 (300 raw/s)** | **~20 s** | **33 s** | **62 %** |
| 8단계 pick-place 전체 추정 | ~25 s | **~70 s** | **~74 %** |

**시연 시간의 3/4 이 대기다.**

### 1.3 펌웨어가 여기에 기여하는 몫

| 출처 | leg 당 | 33 s 중 비중 |
|---|---|---|
| prime sleep 120 ms + first sample lead 220 ms | 340 ms | **약 3 %** |
| 펌웨어 밖에서 소비된 나머지 시간 | ~3 900 ms | 약 97 % |

여기서 97% 전부가 제거 가능한 host 낭비라는 뜻은 아니다. gripper 물리 동작,
접촉 수렴과 필요한 정착도 이 구간에 포함된다. H1은 프로세스·탐색 오버헤드만,
H2는 안전하게 제거 가능한 중간 정착만 줄인다.

> **앞 문서의 펌웨어 비동기 전환만으로는 이 문제가 거의 해결되지 않는다.**
> 지금 요구한 "바로바로 나가는 구조" 의 주된 원인은 host 실행 구조다.
>
> 펌웨어 작업은 여전히 필요하다 — 다만 그 값어치는 **양팔**과 **반응
> 지연 하한**에 있지, leg 간 dead time 에 있지 않다. 순서를 바꾼다.

---

## 2. 목표 구조

```text
[현재]  계획 → 전송 → prime → 실행 → 완전정지 → 정착대기 → 측정 → 계획 → …
        ────────────── 8회 반복, 매번 팔이 멈춘다 ──────────────

[목표]  하나의 연속 stream. 정지는 물리적으로 필요한 곳에만.
        계획은 이전 동작이 도는 동안 끝나 있다.
        보정은 새 Action 이 아니라 진행 중인 stream 에 splice 한다.
```

### 2.1 정지가 물리적으로 필요한 곳은 2곳뿐이다

```text
q0 ─┐
    ├→ pick_pregrasp ─→ pick_grasp        [연속 궤적 1]
    │        └─ 접촉 전 수렴: splice
    ▼
  gripper close                            ★ 물리적 정지 필요
    │
    ├→ lift ─→ place_pregrasp ─→ place_grasp   [연속 궤적 2]
    │        └─ 접촉 전 수렴: splice
    ▼
  gripper open                             ★ 물리적 정지 필요
    │
    └→ retreat ─→ q0                       [연속 궤적 3]
```

**정지 지점: 10회 → 2회. Action: 10개 → 3개.**

gripper 동작은 팔이 멈춰야 한다(물체를 잡는 순간 팔이 움직이면 안 된다).
그 둘 외에 중간 waypoint 에서 속도를 0 으로 떨어뜨릴 이유가 없다.

### 2.2 왜 중간 정착이 필요 없나

post-settle 은 "다음 leg 를 **실측 관절값에서** 계획하기 때문에" 존재한다.
멈춰서 재야 다음 계획의 시작점이 정확하다.

그러나 하나의 연속 궤적 안에서는 그 재측정 자체가 없다. 추종 오차(처짐)는
**누적 오차가 아니라 추종 지연**이고, 궤적이 끝나면 정착한다. 중간에서 재는
것은 아무것도 개선하지 않는다.

**정확도가 필요한 지점은 접촉 직전 두 곳뿐이고, 거기엔 이미 수렴 계층이 있다**
(`execute_grasp_convergence_once.py`, 실측 잔차 7.6~8.5 mm 재현).

---

## 3. 네 개 계층

| 계층 | 무엇 | 없애는 시간 | 펌웨어 변경 |
|---|---|---|---|
| **H1** 상주 실행기 | subprocess 3개/leg → 장수명 노드 1개 | leg 당 1.5~3 s | 없음 |
| **H2** 궤적 병합 + 접촉 전용 정착 | 10 Action → 3 Action, 정착 8회 → 2회 | leg 당 0.3~2.5 s, 정지 8회 | 없음 |
| **H3** 파이프라인 계획 | 다음 궤적을 현재 동작 중에 계획 | 0.08~0.33 s | 없음 |
| **F2.5** splice + 링크 속도 | 반응 지연 3~5 s → ~25 ms | 수렴 leg 1개당 수 초 | **있음** |

**H1~H3 은 펌웨어를 건드리지 않는다.** 먼저 한다.

### 3.1 H1 — 상주 실행기

```text
[현재]                          [목표]
leg 마다:                        프로세스 1개, 세션 내내 유지:
  python 실행                      rclpy 노드 1개
  rclpy init                       MoveIt service client (1회 생성)
  MoveIt 탐색                      buffered action client (1회 생성)
  action 탐색                      /joint_states 구독 (상시 최신값 보유)
  ... 실행 ...                     ↓
  teardown                       leg = 함수 호출
```

- `read_joint_state()` 가 구독 상태에서 즉시 반환 (현재는 노드 생성 + 5 Hz 대기)
- MoveIt 계획이 service call 1회 (현재는 프로세스 + 탐색 + call)
- action goal 이 즉시 발행 (현재는 프로세스 + 클라이언트 탐색)

**기존 도구를 버리지 않는다.** `plan_buffered_segment_leg.py` 등의 계획 로직을
import 가능한 함수로 분리하고, CLI 는 그 함수를 감싸는 얇은 진입점으로 남긴다.
증거 생성·단독 실행·회귀 시험 경로가 그대로 유지된다.

**현재 구현 상태**: 상주 세션과 one-shot wrapper를 만들었고, H1의 primary
metric을 firmware tick 기반 전환 dead time으로 고쳤다. 부분 `JointState`가
오래된 다른 관절을 최신으로 위장하지 않도록 관절별 timestamp도 적용했다.
단위시험은 통과했으며, 실기 반복 evidence만 남아 있다.

H1 전환 dead time 정의:

```text
current.fresh_tick_ms
- (previous.prime_tick_ms + previous.first_sample_lead_ms + previous.duration_ms)
```

wall-clock phase 간격은 진단값일 뿐 gate에 쓰지 않는다. 최소 3개의 성공한
`pregrasp → grasp` 전환을 같은 정의로 모아 표준 median을 계산한다.

### 3.2 H2 — 궤적 병합과 접촉 전용 정착

- MoveIt 에서 waypoint 를 이어 **하나의 궤적**으로 계획하고 시간 파라미터화
  (TOTG). 중간 waypoint 속도가 0 이 아니게 된다
- `POST_SETTLE_*` 를 **leg 속성이 아니라 waypoint 속성**으로 바꾼다.
  `settle: required` 는 gripper 직전 2곳에만
- 안전 허용치(`POST_SETTLE_TOLERANCE_RAW = 30`)는 궤적 **종료 시점**에만 적용

H2는 “육안으로 부드러움”만으로 통과시키지 않는다. 먼저 단독 H1 실행에서
관절별 tracking error envelope를 측정하고, 팔·환경 collision geometry를 그
envelope만큼 보수적으로 팽창시켜 병합 궤적 전 구간을 검사한다. 실기에서는
의도한 gripper 접촉 외 접촉 0, 관절별 추종 오차가 envelope 이내, 두 접촉
지점의 terminal settle 통과, cancel/fault 시 HOLD를 모두 evidence에 남긴다.
영상은 연속성의 보조 증거다.

**깨지는 계약**: `buffered_action_execution.py` 의 leg 단위 정착 규약,
`buffered_trajectory_contract.json` 의 leg 정의. 이 변경이 "갈아엎는" 부분이다.

#### H2 preflight (2026-08-11)

H1의 3회 실기 완주는 H1 gate만 통과한 증거다. 이를 H2의 3개 Action
경로로 기계적으로 이어 붙일 수 있는지는 별도 검증했고, **현재는 불가**로
판정했다.

- 과거 `Motion-13` 3-leg 도구는 2026-07-31의 고정 manifest와 당시의
  50 raw/s 모델에 묶여 있다. 현재 승인된 2026-08-10 pick/lift/place
  SHA-pinned plan의 실행 후보가 아니다.
- H1 `run03`의 각 `*_segments.json`은 매 leg가 끝난 뒤 **실측 feedback을
  새 anchor로 삼아** 다시 collision-check한 기록이다. 따라서 앞 leg의
  계획 종점과 다음 leg의 계획 시작점이 정확히 같지 않다. 이 차이를
  minimum-jerk 직선으로 메워 병합하면 MoveIt이 검사한 경로 밖을 움직일 수
  있으므로 fail-closed 한다.
- pick/place convergence는 접촉 직전 실제 관측값에 따라 달라진다. 그 뒤의
  lift/place를 미리 고정한 H2 route로 선언할 수 없다. 계획 종점을 기준으로
  다음 route를 만들 수 있는 것은 H3의 재계획/재-anchoring 규칙까지 갖춘 뒤다.
- 더 근본적으로, 현재 buffered 경로에는 **동작 중 actual joint feedback이
  없다**. ROS Action adapter는 시작 시점 feedback 한 번만 내보내고, firmware는
  buffered 실행에서 `Servo_MotionSafetyBegin`/`Poll`을 시작하지 않는다. H1
  artifact도 leg별 terminal `post_settle_max_error_raw`만 남긴다. 따라서 이
  데이터만으로는 H2 수락 기준의 joint별 tracking-error envelope를 산출할 수
  없다.

그러므로 H2의 다음 구현 단위보다 먼저 **H2.0 — 동작 중 계측 전제조건**을
닫는다. 이는 제어 stream을 바꾸거나 motion을 허가하는 작업이 아니라,
servo bus 여유와 host link 영향이 측정된 in-motion position telemetry 설계·bench
이다. 이 계측이 buffered 송신을 굶기거나 frame 오류를 만들면 H2를 진행하지
않고 transport/firmware 설계를 먼저 수정한다.

H2.0 통과 뒤 H2의 다음 구현 단위는 실행기가 아니라, **접촉마다 끊긴 두 구간을
각각 현재 collision geometry와 tracking-error envelope로 다시 검증해 하나의
시간-파라미터화 route로 만드는 plan-only builder**다. 이 builder가 다음을
artifact에 모두 남기기 전에는 `buffered_action_execution.py`의 post-settle
규약이나 실행 경로를 바꾸지 않는다.

1. 각 연속 route의 exact SHA input, 시작/끝 joint state, 전 구간 collision
   검사 결과와 timing profile
2. H1 실측으로 정한 joint별 tracking-error envelope 및 그만큼 팽창한
   collision geometry
3. 두 gripper 직전의 terminal-settle 및 cancel/fault → HOLD 검증 계획

이는 H2를 보류한 것이 아니라, 이전 evidence를 새 구조의 안전 증거로
오인해서 재사용하지 않기 위한 진입 gate다.

### 3.3 H3 — 파이프라인 계획

```text
[현재]  ████계획████ ▶▶▶동작▶▶▶ ████계획████ ▶▶▶동작▶▶▶
[목표]  ████계획████ ▶▶▶동작▶▶▶▶▶▶동작▶▶▶▶▶▶동작▶▶▶
                     └─계획─┘  └─계획─┘        (동작 중에 끝나 있음)
```

궤적 N+1 을 궤적 N 이 도는 동안 계획한다. 시작점은 **실측이 아니라 궤적 N 의
계획된 종점**이다. `precompute_ms` 가 78~330 ms 이고 동작이 수 초이므로 항상
여유가 있다.

실측 재기준(re-anchor)은 접촉 전 2곳에서만 한다 — 거기서만 정확도가 필요하고,
거기선 어차피 멈춘다.

### 3.4 F2.5 — splice: 반응 지연의 하한

수렴 보정이 현재는 **새 Action** 이다. 그래서 정지 → 정착 → 계획 → prime →
lead 를 다시 다 치른다. `grasp → q0_return` 간격 23 초의 상당 부분이 이것이다.

목표: 진행 중인 stream 의 **가까운 지평을 덮어쓴다**(splice).

```text
        지금 실행 중인 지평
tick →  ├────────────────────────────────┤
        ▲                    ▲
        현재 적용점          splice_at_tick (= now + splice_lead)
                             └─ 이 지점 이후 sample 을 새 것으로 교체
```

#### 반응 지연 예산

| 항목 | 115200 | **921600** |
|---|---|---|
| splice 배치 3 sample 인코딩·전송 (186 B) | 16.1 ms | **2.0 ms** |
| refill 배치 9 sample (498 B) — 참고 | 43.2 ms | 5.4 ms |
| 최소 splice lead (전송 + Pi jitter 흡수) | 60 ms (현행 하한) | **20 ms** |
| 출력 tick 대기 (최대) | 5 ms | 5 ms |
| sync-write | 0.26 ms | 0.26 ms |
| **합계** | **~81 ms** | **~27 ms** |

**호스트 링크 속도가 반응성의 1차 제약이다.** 115200 에서 refill 배치 하나가
`43 ms` 다. 이전 초안에서 baud 상향을 마지막 단계에 뒀는데 **틀렸다.**
splice 와 함께 앞으로 당긴다.

다만 921600은 아직 ST-LINK VCP 실측값이 아니다. 프로토콜 v2를 동결하기 전에
로봇을 구동하지 않는 loopback/echo 시험으로 다음을 먼저 확인한다.

- 계획된 최악 왕복 traffic의 2배 이상 goodput(운용 이용률 50% 이하)
- 30분 동안 decoded frame loss, CRC error, TX overflow가 모두 0
- 실패하면 baud 숫자를 억지로 유지하지 않고 USB CDC 직결 또는 별도 USB-UART를
  선택한 뒤 ADR-0014의 미들웨어 재검토 조건도 함께 다시 본다

#### 비교

| | 현재 | splice 후 |
|---|---|---|
| 보정 명령이 팔에 도달하기까지 | **4 000 ~ 23 000 ms** | **~27 ms** |
| 중간에 팔이 멈추는가 | 멈춘다 | **멈추지 않는다** |
| 새 Action 이 필요한가 | 필요 | 불필요 |

---

## 4. 펌웨어 쪽 변경 — 앞 문서 대비 수정

### 4.1 mode enum 2개 → 지평(horizon) 1개 + splice

앞 문서는 `TRAJECTORY` / `STREAMING` 두 mode 를 제안했다. **더 단순한 하나로
합친다.**

명령 stream 이 `horizon_end_tick` 필드를 갖는다.

| 상황 | 의미 | 동작 |
|---|---|---|
| queue 고갈 && `now < horizon_end_tick` | host 가 채우기로 한 구간을 못 채웠다 | **fault** (기존 `QUEUE_UNDERFLOW`) |
| queue 고갈 && `now >= horizon_end_tick` | 선언된 끝에 도달 | **정상 종료** → HOLD |
| `horizon_end_tick` 미설정 (열린 stream) | RL 스트리밍 | 마지막 목표 유지 + stale 타이머 |

**필드 하나로 Track A / Track B / 연속 실행 셋을 전부 표현한다.** mode 분기가
없어지고, 궤적은 "끝이 선언된 stream" 이 된다.

### 4.2 splice 의 firmware 요구사항

```text
SPLICE 명령: (splice_at_tick, samples[], horizon_end_tick)

거부 조건 (fail-closed):
  splice_at_tick < now + MINIMUM_SPLICE_LEAD    너무 늦음
  splice_at_tick <= 마지막 적용 tick             이미 지나감
  첫 sample 이 그 시점 보간값에서 관절당 한계 초과  불연속

수락 시:
  splice_at_tick 이후 queue 내용을 폐기하고 새 sample 로 교체
  그 이전 sample 은 그대로 실행 중 — 팔은 멈추지 않는다
```

**연속성 보장**: splice 첫 sample 은 splice 시점의 보간값과 관절당
`MAX_STEP_URAD_PER_TICK` 이내여야 한다. 초과하면 거부한다. 출력단 속도
제한(앞 문서 §3.4 [2])이 최종 backstop 이다.

### 4.3 궤적이 "끝나고 다시 prime" 하는 구조를 제거

현재: `SUCCEEDED` → executor 재초기화 → 16 sample 재선적재 → lead 220 ms.
목표: 실행 중인 executor 가 **지평 연장을 그대로 수락**한다. prime 은 **동작
세션 시작 시 1회**만.

- `STARTUP_PRIME_SAMPLES = 16` → 세션당 1회 (leg 당 → 세션당)
- `INITIAL_FIRST_SAMPLE_LEAD_MS = 220` → 세션 시작에만 적용
- 연속 구간의 refill 은 기존 `LOW_WATERMARK_SAMPLES = 10` / `REFILL_TARGET = 16`
  경로 그대로

### 4.4 비동기 host TX 의 위치가 올라간다

연속 stream 은 refill acknowledgement 가 **동작 내내 계속** 발생한다.
현재 status 프레임 하나가 apply lateness 예산의 94 % 를 쓴다(`4.688 / 5 ms`,
여유 `0.312 ms`). leg 당 몇 번이던 것이 **초당 몇 번**이 된다.

> 앞 문서의 F2(비동기 host TX)는 "양팔 전제조건" 이었는데, 연속 실행에서는
> **한 팔에서도 전제조건**이 된다.

### 4.5 수정된 펌웨어 우선순위

| 앞 문서 | 수정 | 이유 |
|---|---|---|
| F0 계측 | **유지, 여전히 첫 단계** | before 없이 after 를 주장하지 않는다 |
| F1 heartbeat RX 시각 | **유지** | 비용 최소, 가치 최대 |
| F2 비동기 host TX | **유지, 중요도 상승** | 연속 refill 의 전제조건 |
| **(신규) F2.5 링크 921600 + splice** | **F3 앞으로 당김** | 반응 지연 81 → 27 ms. 이전 초안보다 앞당김 |
| F3 타이머 ISR tick | 유지 | |
| F4 서보 DMA + 동작 중 telemetry | 유지 | 흔들림 진단 데이터 |
| F5~F7 인스턴스화·우팔·양팔 | 유지 | |
| ~~F8 STREAMING mode~~ | **삭제** | §4.1 의 horizon 필드로 흡수됨 |
| 구형 마지막 baud 단계 | **삭제** | F2.5 로 이동 |

---

## 5. 실행 순서

**host 먼저.** 측정 구간의 약 97%가 펌웨어 밖에서 소비됐고, 그중
프로세스·불필요한 정착 오버헤드는 펌웨어 변경 없이 줄일 수 있다.

| 단계 | 내용 | Gate | 예상 회수 |
|---|---|---|---|
| **H1** | 상주 실행기. subprocess 제거 | 8단계 pick-place 완주. firmware tick 전환 dead time **표준 median 4 137 → 1 000 ms 미만**, 성공 표본 ≥3 | leg 당 ~3 s |
| **H2.0** | 동작 중 joint telemetry 계측 설계·bench | buffered 송신을 굶기지 않음, link/frame 오류 0, joint별 tracking-error envelope 산출 가능 | H2 진입 전제 |
| **H2** | 궤적 병합 3개 + 접촉 전용 정착 | 정지 10→2, 팽창 collision 검사·tracking envelope·접촉 0·terminal settle·cancel/fault 통과 | leg 당 ~1 s + 정지 8회 |
| **H3** | 파이프라인 계획 | 완주. **계획 시간이 동작 시간에 가려짐**(간격에서 `precompute_ms` 소멸) | ~0.3 s/궤적 |
| **H4** | 소유권 모델 교체 `{arm\|gripper}` → `{stream}` | gripper 가 stream 의 한 열로 동작. `gripper_cmd` Action 은 splice wrapper 로 유지. **팔과 gripper 동시 명령 가능** | Track B 전제조건 |
| **F0** | 계측 (µs 타임스탬프) | before 수치 artifact | — |
| **F1** | heartbeat RX 시각 | 400 ms 정체에도 무래치 | — |
| **F2** | 비동기 host TX | `0x00022800` 실패 실험 재현·통과 | — |
| **F2.5 진입 gate** | ST-LINK VCP 921600 무동작 부하 시험 | 최악 traffic 2배 goodput, 30분 frame/CRC/overflow 0 | 설계 재작업 방지 |
| **F2.5** | 921600 + splice | **보정 반응 지연 < 30 ms 실측.** 수렴이 팔을 멈추지 않고 수행됨 | 수렴당 수 초 |
| **F3~F4** | 타이머 ISR, 서보 DMA | jitter p99 < 100 µs, 동작 중 load/current | — |
| **F5~F7** | 인스턴스화 → 우팔 → 양팔 | skew p99 < 50 µs, coordinated stop | — |

### 5.1 단계별 예상 결과

| | 현재 | H1 | H2 | H3 | F2.5 |
|---|---|---|---|---|---|
| 8단계 cycle dead time | ~70 s | ~25 s | ~8 s | ~5 s | **~2 s** |
| 팔 정지 횟수 | 10 | 10 | 2 | 2 | **2** |
| 보정 반응 지연 | 4~23 s | ~2 s | ~2 s | ~2 s | **~27 ms** |

숫자는 §1 실측에서 유도한 추정이다. **각 단계 gate 에서 실측으로 대체한다.**

---

## 6. 무엇을 깨는가

사용자가 "조금 갈아엎어도 된다" 고 승인한 범위. 명시해 둔다.

| 깨지는 것 | 대체 | 위험 |
|---|---|---|
| leg 단위 `POST_SETTLE_*` 규약 | waypoint 속성 `settle: required` | 중간 정확도 보장이 사라진다 → 궤적 종료 시점 게이트로 이전 |
| `buffered_trajectory_contract.json` leg 정의 | 궤적 3개 계약 | 계약 재검증 필요 |
| leg 당 prime 16 sample | 세션당 1회 | 세션 중 underflow 시 fault (기존 경로) |
| `move_to()` subprocess 파이프라인 | 상주 실행기 함수 호출 | CLI 는 얇은 wrapper 로 유지 → 증거·회귀 경로 보존 |
| 수렴 = 새 Action | 수렴 = splice | splice 연속성 검사 필요 (§4.2) |
| 프로토콜 v1 | v2 (`horizon_end_tick`, splice, arm_mask) | host lockstep 이관 |
| `INITIAL_FIRST_SAMPLE_LEAD_MS = 220` | 세션 시작 전용 + splice lead 20 ms | STALE_TICK 재발 위험 → 921600 실측 gate 통과가 전제 |

### 6.1 깨지 않는 것

- **안전 계층 전부** — heartbeat, watchdog, SAFE_STOP, 관절 한계, load/current,
  fail-closed 규율
- calibration hash 가 게인·home·범위를 핀하는 불변식
- `actuator_core` host 단위시험 자산
- MoveIt 표준 `FollowJointTrajectory` 호환 (궤적을 합칠 뿐, 인터페이스는 유지)
- 증거 기반 gate 규율 — 단계마다 artifact

---

## 7. 규율

- **원인을 실측으로 먼저 찾았다.** 33 s 중 펌웨어 몫이 3 % 라는 것이 이 계획의
  출발점이다. 그 측정을 하지 않았으면 펌웨어를 몇 주 고치고 dead time 은
  그대로였을 것이다
- **싼 것부터.** H1~H3은 펌웨어 밖의 제거 가능한 dead time부터 줄인다
- **단계마다 로봇이 동작한다.** H1 만 해도 시연이 눈에 띄게 나아진다
- **깨는 것을 명시했다.** §6 이 그 목록이고, 안전 계층은 거기에 없다
