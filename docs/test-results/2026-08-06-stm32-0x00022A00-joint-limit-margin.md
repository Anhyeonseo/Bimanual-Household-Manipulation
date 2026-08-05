# 0x00022A00 — q0 가 관절 한계선에 걸터앉지 않도록 여유를 넣는다

- 날짜: 2026-08-06
- 펌웨어: `0x00022A00` (이전 배포 `0x00022900`)
- calibration hash: `0x8AD27897` → **`0xB317C672`**
- 상태: **실기 검증 통과.** 계약 `deployed: true`, `motion_authorized: false`
- 관련: [0x00022900 status 전송 예산](2026-08-06-stm32-0x00022900-status-transmit-budget.md),
  [Motion-12 q0 복귀](2026-08-06-motion12-buffered-q0-return.md)

## 문제

`WRIST_FLEX` 는 `zero_raw == maximum_raw == 2048`, `BASE` 는
`zero_raw == minimum_raw == 2048` 이었다. **q0 가 두 축의 한계선 위에 있다.**

q0 복귀의 post-settle 실측이 `6 raw` 이므로, 복귀할 때마다 대략 반반 확률로
팔이 계약 범위 밖에 놓인다. 실제로 그렇게 됐다:

```
gripper goal rejected: joint feedback is outside safe range:
  ELBOW: target raw 2491 outside 627..2258        (torque OFF 중 중력 이탈)
...
WRIST_FLEX  2051   최대 2048   범위밖             (close 중 팔이 4 raw 이동)
```

두 번째가 이번 항목이다. gripper 명령이어도
`prepare_parallel_gripper_goal` 이 **6축 전체 피드백**을 `radians_to_urad` 로
엄격 검증하므로, 팔이 3 raw 밖이면 gripper 를 열 수조차 없다.

> 팔 Action 경로에는 `validate_feedback_recovery_envelope` 와
> `RECOVERY_FEEDBACK_OVERRUN_RAW = 20` 이 있어 통과한다. 그래서 q0 복귀는 두 번
> 다 성공했는데 gripper 만 막혔다. 경로 간 불일치이며 별도 항목으로 남긴다.
>
> 다만 recovery envelope 는 **일시적 이탈을 흡수하라고** 있는 것이지 정상
> 자세가 상시로 그 안에 걸쳐 있으라고 있는 것이 아니다. "항상 허용대 안"은
> "절대 실패하지 않는다" 가 아니다.

## 변경

| 축 | 변경 | URDF 여유 잔량 |
|---|---|---|
| `BASE` | `minimum_raw 2048 → 1988` | 하한쪽 1192 raw 남음 |
| `WRIST_FLEX` | `maximum_raw 2048 → 2108` | 상한쪽 282 raw 남음 |

`60 raw` 는 임의값이 아니다. `SHOULDER` 가 이미 하한쪽 `60 raw` 를 쓰고 있어
저장소의 기존 관례이고, 관측된 post-settle 오차 `6 raw` 의 **10배**다.

`GRIPPER` 도 `zero_raw == maximum_raw` 지만 건드리지 않았다. 그 `2048` 은 URDF
상한과 정확히 같아 넓힐 여유가 없고, 운영 중 gripper 는 `1963..2009` 만 쓰고
`2048` 에 머무르지 않으므로 문제가 되지 않는다.

사용자가 물리적 여유를 확인하고 승인했다.

## 해시 교차검증

calibration hash 는 펌웨어 `servo_bus.c` 의 `servo_joints` 표에서 나온다
(`Host_CalibrationHash`, 관절당 id/home/min/max/direction/p_gain 순 54 B
CRC-32C). host 는 같은 값을 JSON 에서 독립적으로 계산한다.

| 경로 | 결과 |
|---|---|
| 변경 전 펌웨어 표 모델 | `0x8AD27897` — 배포된 값과 일치(모델 검증) |
| 변경 후 펌웨어 표 (빌드된 소스에서 재유도) | `0xB317C672` |
| 변경 후 host JSON 계산 (`config/`) | `0xB317C672` |
| 변경 후 host JSON 계산 (bridge 사본) | `0xB317C672` |

**두 독립 구현이 일치한다.** 변경 전 값을 먼저 재현해 모델을 검증한 뒤에만
새 값을 신뢰했다.

## 예상하지 못한 파급 — artifact 신원 결합

**collision-checked artifact 22개가 옛 해시를 박고 있었다.** 도구들이
"manifest 와 로컬 보정 해시가 다르다" 로 거부했다 — fail-closed 가 옳게
작동한 것이다.

문제는 해시가 *보정이 바뀌었다* 만 말하고 *어떻게 바뀌었는지* 는 구분하지
못한다는 점이다. 두 종류는 결과가 완전히 다르다.

- **사상**(`zero_raw` / `positive_raw_direction` / `raw_units_per_turn`)이
  바뀌면 → 저장된 raw 가 다른 물리 자세를 뜻한다. 기존 경로는 전부 무효다.
- **한계**만 넓어지면 → 자세의 의미는 그대로이고, 더 좁은 옛 한계 안에서
  계획된 경로는 새 한계 안에도 들어간다. 경로는 유효하다.

이번은 후자다. 그래서 artifact 의 해시 필드를 재발행했다. **다만 그것을 조용히
하지 않았다.** `config/single_arm_calibration.json` 에
`superseded_calibrations` 로 옛 보정 전체(해시·전 관절 값·사유)를 기록하고,
`tests/test_calibration_supersession.py`(7건)가 재발행의 정당성을 강제한다.

- 사상 필드가 하나라도 바뀌면 실패 — **음성 검증됨**(`zero_raw` 를 바꿔 확인)
- 한계가 좁아지면 실패 — **음성 검증됨**(`minimum_raw` 를 올려 확인)
- `p_gain` 이 바뀌었는데 사유에 적혀 있지 않으면 실패
- q0 가 전 축(GRIPPER 제외) 양쪽으로 `60 raw` 여유를 갖는지 확인

사상이 바뀐 교체에서는 이 시험이 실패하고, 그때는 재발행이 아니라 **재계획**이
답이다. 그 경계를 기계가 지킨다.

### SHA 사슬 수렴

계약이 Motion-9 artifact 의 SHA 를 박고, Motion-9 는 계약의 SHA 를 박는다.
순환이므로 순서가 있다: 보정 SHA 재스탬프 → Motion-9 SHA → 계약 → 계약 SHA 를
박는 Motion-10/11 재생성 → 시험 재고정. Motion-10/11 은 계약에 고정되어 있지
않으므로 여기서 멈춘다.

`Motion-9` 는 `contract_sha256` 이 의도적으로 낡은 채로 남는다 —
`test_historical_plan_is_invalidated_by_deployed_contract` 가 그것을 요구한다.

## 함께 고친 것

여러 시험이 옛 한계에 의존하고 있었고, 값만 바꾸는 대신 **의도를 유지하도록**
표본을 옮겼다. 그 과정에서 잠재 결함 하나를 찾았다.

`test_feedback_beyond_firmware_recovery_envelope_is_rejected` 의 표본
`(2070, 2048-41, ...)` 은 index 1 이 `SHOULDER`(하한 1988)라 **봉투를 벗어나지
않았다.** 실제 거부는 `WRIST_FLEX` 초과가 유발한 bounded recovery 의 target
margin 규칙에서 나왔다. 이름과 다른 이유로 통과하고 있었다. `1988 - 41` 로
고쳐 이름대로 동작하게 했다.

## 검증

| 항목 | 결과 |
|---|---|
| 전체 회귀 (`pytest -q`) | **712 passed** (이전 703, 신규 8 + 재편) |
| actuator core native (`ctest`) | 2/2 passed |
| Cortex-M4 Release 크로스 빌드 | 성공 (`text 39672 / data 112 / bss 5936`) |
| 펌웨어 표에서 해시 재유도 | `0xB317C672` |
| host 두 사본에서 해시 계산 | `0xB317C672` |
| supersession 가드 음성 검증 | 사상 변경·한계 축소 각각 실패 확인 |

HEX: `artifacts/firmware/2026-08-06/stm32_g474_single_arm_0x00022A00.hex`
SHA-256 `5ee5f34ede5a3247b164c45f6cdfbb114c82d94fe14ee0eaa35f4c6d2f267f84`

## 실기 검증

HEX `5ee5f34ede5a3247b164c45f6cdfbb114c82d94fe14ee0eaa35f4c6d2f267f84`.
host 배포를 identity 보다 **먼저** 두었다 — calibration hash 도 함께 바뀌므로
순서를 틀리면 identity 가 거부한다(0x229 에서 겪은 실수).

| 항목 | 결과 |
|---|---|
| identity | `0x00022A00` / `0x00000FFF` / **`0xB317C672`**, latch 없음 |
| 70초 READ_ONLY soak | `PASSED=1`, `failure=None` |
| 호출 | heartbeat 700 · position 350 · diagnostics 2 |
| 오류 counter 12종 | **전부 0** (baseline 부터, 60 s 시점에도 `recovery=0 fe=0`) |
| 자동 재시도 / motion 명령 | 0 / 0 |

### 목적이 달성되었는지

```
POSITIONS = (2053, 2158, 1953, 2051, 2054, 1965)
```

`WRIST_FLEX 2051` 이다. 옛 상한 `2048` 에서는 **범위 밖**이라 gripper 명령이
거부되던 바로 그 자세이고, 새 상한 `2108` 안에 들어온다. 6축 전부 계약 안이다.

계약을 `deployed: true` 로 전환했다. `motion_authorized` 는 `false` 유지.

## 함께 고친 것 — 접촉 판정 기준

`--expect contact` 가 `reached_goal` 로 판정하고 있었다. 그것은 파지 증거가
아니다. `_finish_goal` 은 실행이 SUCCEEDED 이면 실제 위치와 무관하게
`prepared.target_position` 을 넣고 `reached_goal=True` 를 답한다.

2026-08-06 probe 실측이 그것을 드러냈다.

| 시점 | gripper raw |
|---|---:|
| close 명령 | 1963 |
| 물체 문 상태 | **1983** (잔여 20, `REACHED_GOAL=True`, `ACTION_STATUS=4`) |
| 물체 치운 뒤 | **1965** (잔여 2) |

물체를 치우자 서보가 명령값까지 마저 닫혔다. **잔여 간격이 곧 판별자**이므로
판정을 `RESIDUAL_GAP_RAW >= MINIMUM_CONTACT_GAP_RAW(8)` 로 바꿨다.

이 실측은 A3 3분할 설계의 전제도 확인해 준다: 물체를 문 close 가 abort 도
latch 도 없이 정상 종료하므로, leg 사이에 gripper 를 넣어도 다음 leg 가
막히지 않는다.

> 아직 없는 것: **물체 없는 close 의 대조군.** 잔여 20 raw 가 물체 때문인지
> 서보 정상 오차인지는 그 측정이 있어야 확정된다. 다만 물체를 치우자 2 raw 로
> 줄어든 것이 이미 강한 근거다. `MINIMUM_CONTACT_GAP_RAW = 8` 은 그 사이에
> 보수적으로 둔 값이며, 대조군 측정 후 재검토한다.
