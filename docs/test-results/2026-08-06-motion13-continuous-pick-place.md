# Motion-13 — 연속 Pick/Place 경로를 buffered leg 3개로 실기 완주

- 날짜: 2026-08-06
- 펌웨어: `0x00022A00`, calibration `0xB317C672`
- bridge: `mode=MOTION_ENABLED`, **하나의 세션에서 전 구간 실행**
- 계약: `motion_authorized=false` 유지
- 증거: [evidence/2026-08-06-motion13-three-leg-execution.json](evidence/2026-08-06-motion13-three-leg-execution.json)
- 관련: [0x00022A00 관절 한계 여유](2026-08-06-stm32-0x00022A00-joint-limit-margin.md),
  [Motion-12 q0 복귀](2026-08-06-motion12-buffered-q0-return.md)

## 결과

`q0 → pick_pregrasp → pick_grasp → [close] → lift20 → place_pregrasp →
place → [open] → retreat → q0` 전 구간을 **buffered Action 3개와 gripper
명령 2개**로 완주했다. 사이에 q0 복귀는 없다.

| leg | 경로 | 시간 | sample | lateness | post-settle | 목표 도달 | anchor 이탈 |
|---|---|---:|---:|---:|---:|---:|---:|
| A | q0 → pregrasp → grasp | 41.0 s | 2051 | 4 ms | 14 raw | 0.022026 rad | 6 raw |
| B | grasp → lift → place | 14.0 s | 701 | 4 ms | 16 raw | 0.024294 rad | 17 raw |
| C | place → retreat → q0 | 39.0 s | 1951 | 4 ms | 6 raw | 0.009204 rad | 16 raw |
| | **합계** | **94.0 s** | **4703** | | | | |

- 3개 Action 전부 terminal `succeeded`
- apply lateness 전부 `4 ms` (허용 5)
- post-settle 전부 허용 `30 raw` 안
- Action 전송 3회, gripper 2회, **자동 재시도 0, 중단 0**
- leg 사이 q0 복귀 **0회**

### leg 연결이 실제로 작동한다

앞 leg 의 post-settle 오차가 다음 leg 의 시작 이탈이 된다. 설계상 그렇게
되도록 만들었고 실측이 그대로 나왔다.

```
leg A post-settle 14 raw  ->  leg B anchor 이탈 17 raw
leg B post-settle 16 raw  ->  leg C anchor 이탈 16 raw
```

한계 `40 raw` 안에서 여유 있게 이어진다.

## apply lateness — 다섯 번째 확인

합산 `3904, 217, 290, 200, 92, 0` (합 4703 = 전체 sample).

늦은 sample `799` ÷ refill 추정 `783.8` = **1.019**.

| 회차 | sample | 비율 | bucket 5 |
|---|---:|---:|---:|
| Motion-12 (04:46) | 1901 | 1.035 | 0 |
| Motion-12 (05:48) | 351 | 1.009 | 0 |
| leg A | 2051 | 0.977 | 0 |
| leg B | 701 | 1.036 | 0 |
| leg C | 1951 | 1.058 | 0 |
| **합산** | **4703** | **1.019** | **0** |

**sample 수가 5.8배 차이 나는 다섯 회차에서 refill acknowledgement 1회당 늦은
sample 이 정확히 1개다.** bucket 5 는 매번 비어 있다 — 전송이 `4.688 ms` 라
`5` 를 채울 수 없다는 산술 그대로다.

계통적 추종 지연이 아니라 응답 프레임 전송이라는 단일한 원인이며, 그 크기는
길이에 무관하게 고정되어 있다. 83% 는 정확히 정시에 적용됐다.

## 게이트가 실제로 막았다

leg A 첫 시도는 **거부됐다.**

```
ValueError: leg A anchor is off the collision-checked route:
  deviation_raw=[6, 110, 95, 3, 6] limit=40 expected_start=q0
```

세션 경계에서 팔이 q0 를 벗어나 있었다(`SHOULDER 110` / `ELBOW 95 raw`).
게이트가 없었다면 collision-checked 경로에서 `110 raw` 벗어난 시작점으로
41초 궤적을 만들어 보냈을 것이다. 팔은 움직이지 않았다.

세션 안에서 q0 복귀를 먼저 실행해(`4000 ms` / `201 sample`, post-settle
`6 raw`) 이탈을 `6 raw` 로 만든 뒤 진행했다.

> **운영 규칙:** 세션을 시작할 때마다 q0 복귀를 먼저 한다. leg 사이에는
> bridge 가 살아 torque 를 유지하므로 필요 없지만, 세션 경계는 항상 이렇다.
> 받쳐서 종료해도 자세는 유지되지 않는다 — 05:48 에 `0.009204 rad` 로 q0 에
> 도달했던 팔이 받침 종료와 플래시를 거쳐 `SHOULDER 2158 / ELBOW 1953` 이
> 되어 있었다.

## 증명되지 않은 것 — 물체를 실제로 집지 못했다

`pick_close` 는 **잔여 3 raw** 로 끝났다. 대조군(물체 없는 close)이 `5 raw`
이므로 이것은 **손가락 사이에 아무것도 없었다**는 뜻이다.

```
VERDICT=FAIL_NO_CONTACT   residual gap 3 raw is below 14
```

**gripper 자체는 정상 동작했다.** 끝까지 닫혔고, 판정 기준이 그것을 정확히
잡아냈다. 문제는 물체가 손가락 사이에 없었다는 것이다.

원인은 이미 계획에 있는 두 항목이다.

- **A4** — Place/Pick TCP-to-contact offset 이 아직 공칭 `0.025 m` 다.
  Stage 7 이 `-5 mm` 보정 2회를 필요로 했으므로 실측 후보는 `0.015 m` 다.
- **W3/W4** — 손목 eye-in-hand 보정과 bounded visual correction 이 없어
  물체의 실제 위치를 반영할 수 없다.

따라서 이 회차는 **기구를 증명하고 과제를 증명하지 않는다.** 체크리스트의
`물체 실제 파지·이동·해제` 는 미체크로 남긴다. 시퀀스가 돌았다고 파지가
됐다고 적으면 그 기록을 나중에 믿을 수 없게 된다.

`place_release` 는 빈 gripper 로 명령 위치에 도달했다(`REACHED`, 잔여
`6 raw`). 빈 상태에서는 그것이 정확한 검사다.

## 3분할 설계의 전제가 확인됐다

buffered 실행에는 load/current 감시가 없다(`Servo_MotionSafetyPoll` 은
비버퍼드 경로에만 있다). 그래서 접촉을 leg 경계로 뺐다.

이번 회차가 그 경계가 실제로 작동함을 보였다: gripper 명령이 두 번 다
`ACTION_STATUS=4` 로 정상 종료했고 latch 가 걸리지 않아 **다음 leg 가
막히지 않았다.** 팔 비명령 동작도 없었다(close `0.009204`, open `0.007670`
rad, 한계 `0.02`).

## 수락 기준 대조

| 기준 | 결과 |
|---|---|
| 3개 Action 전부 terminal `succeeded` | 통과 |
| 각 Action apply lateness ≤ 5 ms | 통과 (전부 4) |
| 각 Action post-settle ≤ 30 raw | 통과 (14 / 16 / 6) |
| gripper 2회 팔 비명령 동작 0 | 통과 |
| 자동 재시도 0회 | 통과 |
| leg 사이 q0 복귀 0회 | 통과 |
| 세 leg 의 lateness 분포 기록 | 통과 |
| **물체 실제 파지·이동·해제** | **미달** — A4 / W3·W4 이후 |
