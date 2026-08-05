# Motion-11 buffered Pick pregrasp 물리 통과

## 결론

47초·2351 sample 단일 buffered Action이 실기 완주했다. 팔이 현재 자세에서
q0를 거쳐 Pick pregrasp까지 이동했고 모든 게이트를 통과했다. Action은 1회만
전송했고 자동 재시도는 0회다. `motion_authorized=false`는 유지한다.

세 번의 실패 뒤 첫 통과다. 1차는 Shoulder/Wrist Flex 추종 부족(545/286 raw),
2차는 startup lead 부족(79 ms < 80 ms 하한), 3차는 servo UART 정지였다.

**단, sender가 정규식 결함으로 성공을 실패로 보고했다.** 물리 실행은 성공했고
host 파서만 틀렸다. 아래 「terminal 형식 드리프트」에 기록한다.

## 검증된 입력

- firmware `0x00022500`, capabilities `0x00000FFF`, calibration `0x8AD27897`
  ([0x225 물리 검증](2026-08-06-stm32-0x00022500-servo-uart-power-domain-lifecycle.md))
- bridge `mode=MOTION_ENABLED`, `allow_motion:=true`
- torque 유지 fresh anchor raw `2059 / 2180 / 1733 / 1840 / 2142 / 2002`
- anchor 캡처: 15 표본, **축별 spread 전부 `0`**
  ([증거](evidence/2026-08-06-motion11-anchor-capture.json))
- Pick pregrasp 목표 raw `2278 / 3190 / 1625 / 1209 / 2146 / 2002`
- source route `01_q0_to_pick_pregrasp.json`,
  SHA `da5f3b3fc8200cbc4713e2fcf05d5b54387929ec399377ebc68ce1722587549f`
- plan SHA `c6b3c49760a759c813071c2a6610ef816b00eb46dc140822ecd5e65e4a8ccebe`
  (1.1 MB이므로 저장소에 넣지 않고 해시만 고정한다)

anchor 는 반드시 torque 가 걸린 상태에서 읽어야 한다. bridge 가 serial 을
독점하므로 servo 직접 읽기가 불가능해
[`tools/capture_buffered_anchor_raw.py`](../../tools/capture_buffered_anchor_raw.py)
를 새로 만들었다. `/joint_states` 를 구독해 계획 생성기와 **같은** 변환
(`plan_buffered_q0_roundtrip.radians_to_raw`)으로 raw 를 만들고, 표본 간 raw
변동이 허용치를 넘으면 torque 미유지로 보고 거부한다.

## 계획 게이트 (plan-only)

| leg | 시간 | peak error | terminal error |
|---|---:|---:|---:|
| anchor → q0 | 12000 ms | `0.00 raw` | `0.00 raw` |
| q0 → pregrasp | 35000 ms | **`79.99 raw`** (SHOULDER) | `0.00 raw` |
| 허용치 | | `100 raw` | `30 raw` |

- 최대 sample step `0.001877 rad`
- 이산 velocity `0.093850 rad/s` (상한 `0.5`)
- 이산 acceleration `0.022500 rad/s²` (상한 `1.0`)
- queue: accepted `2351` / applied `2340` / queued `11`, underflow 없음
- firmware 출력 시뮬레이션 `9401`개, 5 ms 당 arm 최대 `1 raw`

SHOULDER peak `79.99 raw`가 이 계획의 핵심이다. 1차 시도에서 `545 raw`로
실패한 축이며, 실측 약 60 raw/s를 보수적 50 raw/s 계약으로 낮춰 계획 자체의
목표 변화율을 추종 능력에 맞춘 결과다.

## 실기 결과

```
PLAN_GATE=PASS
FRESH_START_MAX_ERROR_RAD=0.000000
ACTION_SEND_COUNT=1
```

bridge terminal ([증거](evidence/2026-08-06-motion11-execution.json)):

```
ARM_EXECUTION_TERMINAL state=succeeded sequence=4168 status=6 detail=5
reason=buffered trajectory completed;
  maximum_apply_lateness_ms=5 post_settle_max_error_raw=18;
  precompute_ms=234.817 fresh_tick=2578870 prime_tick=2578938
  first_sample_lead_ms=92 prime_heartbeat_gates=1 prime_frames=2
  accepted=16 applied=0 queued=16
```

| 항목 | 값 | 허용 | 여유 |
|---|---:|---:|---:|
| maximum apply lateness | `5 ms` | `0..5 ms` | **`0`** |
| post-settle max error | `18 raw` | `0..30 raw` | `12` |
| first sample lead | `92 ms` | `≥80 ms` | `12` |
| precompute | `234.817 ms` | — | — |
| 자동 재시도 | `0` | `0` | — |

최종 pregrasp 도달 (허용치 `0.050 / 0.055 / 0.050 / 0.050 / 0.050 rad`):

| 축 | 실제 rad | 목표 rad | 오차 |
|---|---:|---:|---:|
| base | `+0.346680` | `+0.352852` | `-0.006173` |
| shoulder | `+1.757942` | `+1.751287` | `+0.006655` |
| elbow | `+0.621262` | `+0.649212` | `-0.027950` |
| wrist_flex | `+1.273204` | `+1.286483` | `-0.013279` |
| wrist_roll | `+0.144194` | `+0.150604` | `-0.006410` |

`PREGRASP_MAX_ERROR_RAD=0.027950`, `PREGRASP_TARGET_GATE=PASS`.

1차 시도에서 `545 raw`였던 SHOULDER가 이번에는 `+4 raw`(`0.006655 rad`)다.

sender 는 physical DISABLE 을 수행하지 않는다. 팔이 pregrasp 에서 torque 를
유지한 채 남는 것이 정상 종료 상태다.

## terminal 형식 드리프트 (수정함)

sender 가 `RuntimeError: buffered Action terminal evidence is missing` 로
종료했다. **물리 실행은 성공했고 파서만 틀렸다.**

`validate_action_terminal` 의 검사 순서상 `action_status == SUCCEEDED` 와
`error_code == SUCCESSFUL` 은 이미 통과한 뒤였다. 실패 지점은 정규식이다.

- bridge (`buffered_action_execution.py:341-343`)는 성공 terminal 뒤에
  `"; {startup}"` 로 startup 진단을 덧붙인다.
- sender (`execute_buffered_action_plan_once.py:40-44`)의 패턴은
  `post_settle_max_error_raw=(\d+)$` 로 문자열 끝을 요구했다.

양쪽 시험이 각자의 가정으로만 문자열을 만들어 이 드리프트가 보이지 않았다.
sender 시험은 접미사 없는 옛 형식을 썼고, bridge 가 startup 진단을 덧붙이도록
바뀔 때 따라가지 않았다.

수정:

1. 패턴이 접미사를 선택적으로 허용하고 값을 `TerminalEvidence.startup_diagnostics`
   로 보존한다. 접미사 없는 옛 형식도 계속 읽힌다.
2. **`tests/test_buffered_terminal_format_contract.py`** 신규 (7개) —
   bridge 소스의 f-string 을 파싱해 실제 문자열을 재구성하고 sender 패턴에
   넣는다. 어느 쪽이 바뀌어도 깨진다. 범위 검사(`0..5 ms`, `0..30 raw`)와
   잘린 terminal 거부도 함께 본다.

이 결함은 sender 를 비정상 종료시켰을 뿐 팔의 거동에는 영향이 없었다.
마지막 pregrasp 목표 게이트는 같은 함수로 사후 수행해 통과를 확인했다.

## 후속 품질 항목 (A3 전에 검토)

두 신호가 같은 방향을 가리킨다.

1. **apply lateness 가 상한에 닿았다.** `5 ms`는 허용 `0..5`의 경계이고
   `6 ms`부터 `MISSED_APPLY_TICK + safe stop`이다. Motion-9와 Motion-10은
   모두 `4 ms`였다.
2. **실행 중 heartbeat 경고 1회.**
   `transient heartbeat delay (1/3): timeout waiting for STATE_FEEDBACK`.
   허용 3회 중 1회를 소비했다.

근거가 단일 실행이므로 원인을 단정하지 않는다. 다만 A3 연속 Pick/Place 는
이 경로의 3~4배(약 150초)이므로 초과 확률이 비례해 커진다. **A3 설계 전에
lateness 분포를 볼 수 있는 계측을 먼저 넣는다.** 최대값만으로는 특정 구간에
몰리는지 전 구간에 퍼지는지 구분할 수 없다.

`first_sample_lead_ms=92` 도 하한 `80`에 대해 여유가 `12 ms`뿐이다.
precompute 가 `234.817 ms`로 안정적이지만 여유 자체가 크지 않다.

## 로컬 회귀

```
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m pytest -q
```
**`615 passed`** (0x225 배포 후 608 + terminal 형식 계약 7)

## 다음 gate

1. lateness 분포 계측을 추가하고 재측정한다.
2. grasp / lift / place / retreat / q0 를 하나의 연속 buffered Action 으로
   확장한다 (`MOTION_G474_BUFFERED_PHYSICAL_ROUTE.md` 의 마지막 미체크 박스).
   sample 수가 약 8000, 시간 약 150초가 되므로 `simulate_admission_batches`
   로 refill 여유를 먼저 확인한다.
3. Place TCP-to-contact offset 을 분리해 다시 계측한다.
4. 10회 pilot.
