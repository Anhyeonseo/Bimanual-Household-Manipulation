# Motion-12 — buffered q0 복귀 실기 통과와 apply lateness 분포 첫 실측

- 날짜: 2026-08-06
- 펌웨어: `0x00022900`, calibration `0x8AD27897`
- bridge: `mode=MOTION_ENABLED`, `allow_motion:=true`
- 계약: `motion_authorized=false` 유지 (물리 동작은 매 회 별도 승인)
- 증거: [evidence/2026-08-06-motion12-q0-return-execution.json](evidence/2026-08-06-motion12-q0-return-execution.json)
- 관련: [0x00022900 전송 예산](2026-08-06-stm32-0x00022900-status-transmit-budget.md)

## 결과

Pick pregrasp 자세에서 q0 까지 **단일 buffered Action 1회로 완주**했다.

| 항목 | 값 | 기준 |
|---|---|---|
| terminal | `succeeded` | — |
| maximum apply lateness | **4 ms** | ≤5 |
| post-settle 최대 오차 | **6 raw** | ≤30 |
| q0 도달 오차 | **0.009204 rad** | — |
| fresh-start 오차 | `0.000000 rad` | 게이트 통과 |
| Action 전송 | 1회 | 재시도 0 |
| 자동 재시도 | **0회** | 0 |
| servo bus 오류 counter 8종 | **전부 0** (transaction 9330회) | delta 0 |

계획은 38000 ms / 1901 sample, duration 은 추종 계약을 통과하는 최소값으로
자동 탐색됐다(모델 peak 오차 `58.410 raw`, terminal `0.000`).

앞선 여섯 번의 중단은 두 개의 서로 다른 결함이었고 둘 다 이번에 해소됐다 —
host 수신 조각 폐기, 그리고 acknowledgement 프레임 길이.

## apply lateness 분포 — 첫 실측

지금까지 모든 회차가 `applied=0` 으로 끝나 한 번도 얻지 못했던 값이다.

```
lateness_buckets = 1573, 96, 98, 90, 44, 0
lateness_worst_sample = 94
```

| bucket | count | 비율 | 누적 |
|---|---|---|---|
| 0 ms | 1573 | 82.75% | 82.75% |
| 1 ms | 96 | 5.05% | 87.80% |
| 2 ms | 98 | 5.16% | 92.95% |
| 3 ms | 90 | 4.73% | 97.69% |
| 4 ms | 44 | 2.31% | 100.00% |
| 5 ms+ | **0** | 0.00% | 100.00% |

합 `1901` = `applied_samples`. 불변식 성립.

### 분포가 원인을 지목한다

늦은 sample 은 **328개**다. refill 은 watermark 10 에서 target 16 으로
채우므로 회당 6 sample, 1901 sample 이면 약 **317회**다.

```
328 / 317 = 1.035
```

**refill acknowledgement 1회당 늦은 sample 이 정확히 1개다.**

분포 모양도 맞다. blocking 전송이 `4.688 ms` 이고 그 구간에 apply tick 이
떨어지면 lateness 는 대략 `[0, 4.688]` 에 퍼진다.

| bucket | 균일 모델 예측 | 관측 |
|---|---|---|
| 1 | 27.1% | 29.3% |
| 2 | 27.1% | 29.9% |
| 3 | 27.1% | 27.4% |
| 4 | 18.7% | 13.4% |

bucket 1~3 은 잘 맞고 bucket 4 는 예측보다 낮다. bucket 4 는 `[4, 4.688]`
의 `0.688 ms` 조각뿐이라 ms 양자화에 민감하고, apply tick 은 20 ms 격자에
refill 은 약 120 ms 격자에 있어 균일 가정이 정확하지 않다.

**bucket 5 가 비어 있는 것이 결정적이다.** 전송이 `4.688 ms` 라 `5` 를
채울 수 없다. 산술이 요구하는 그대로다.

즉 이 분포는 계통적 추종 지연이 아니라, **응답 프레임 전송이라는 단일하고
결정론적인 원인**을 보여준다. 나머지 82.75% 는 정확히 정시에 적용됐다.

### 여유의 실제 크기

중단 조건은 `lateness > 5`, 즉 `6 ms` 부터다. 현재 최악값은
`ceil(4.688) = 5` 이고 이번 회차는 `4`, Motion-11 은 `5` 였다 — ms 양자화가
가르는 경계 위에 있다.

`6` 이 되려면 전송이 `5.0 ms` 를 넘어야 하고, 그건 payload `+4 B` 다.
`binary_control.c` 의 `#error` 가 정확히 그 지점에서 빌드를 거부한다.

**여유는 `0.312 ms`, 전선 기준 4바이트 미만이다.** 그리고 1901 sample 중
44개(2.31%)가 이미 한 칸 아래에 있다.

## 부수 확인

`host_tx_maximum_ms = 15` 는 실행 중 값이 아니라 실행 후 진단 프레임 자체의
전송시간이다(168 B → 이론 14.58 ms). 유휴 기준선과 같으므로 계측은 건전하다.
`host_tx_failure_count`/`timeout_count` 모두 0.

servo bus 는 9330 transaction 동안 8종 counter 전부 0. `0x00022500` 의 전원
도메인 수명주기가 실동작 구간에서도 유지된다.

## 이 회차에서 배운 운영상의 것

첫 시도는 `mode=READ_ONLY` 로 떠서 anchor 포착에서 멈췄다. `ros2 run` 으로
노드를 직접 띄우면 `bridge.launch.py` 가 겹쳐 로드하는 `bridge.local.yaml`
(`allow_motion: true`, by-id serial 경로)이 실리지 않고, Pi 쪽에
`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` 도 없어 desktop 과 다른 DDS 평면에 뜬다.

Pi 로그인 셸에는 둘 다 `unset` 이다. 물리 실행은 반드시
`ros2 launch single_arm_bridge bridge.launch.py` 로, DDS 환경을 명시해서
띄운다. 동작 전에 `mode=MOTION_ENABLED` 와 desktop 측 `/joint_states` 도달을
각각 확인한다.

> `bridge.local.yaml` 이 untracked 이고 Pi 에만 있으므로, 물리 실행 가능
> 여부가 저장소에 기록되지 않는 장비 상태에 달려 있다. 별도 항목으로 남긴다.

## 다음

A3(연속 Pick/Place) 진입 판단은 [ROADMAP](../CURRENT_STATE_AND_NEXT_ROADMAP.md) 에
기록한다. 요지는 이 분포가 **길이에 비례해 악화되지 않는다**는 것이다 —
refill 당 1 sample 이고 그 크기는 전송시간으로 고정되어 있다. 다만 여유가
`0.312 ms` 라는 사실은 변하지 않는다.
