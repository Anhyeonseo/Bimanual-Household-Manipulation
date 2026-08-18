# 캔 → 쓰레기통 계획 (개정판)

- 기준일: 2026-08-16
- 기준 firmware: F8.9 `0x00024809`, protocol v2, 12축 resident
- 직전 수락점: Top-camera 기반 왼팔→오른팔 펜 전달 1회 완주
- 이 문서의 지위: `docs/prompts/CAN_TO_BIN_HANDOFF_PROMPT.md.orig`(2026-08-15,
  git 미추적)의 **개정판**이며, 아래 §2에서 그 문서의 기하 전제 세 개를 실측으로
  반박하고 대체한다.

## 0. 요약

| 항목 | 2026-08-15 계획 | 이 계획 |
|---|---|---|
| v1 범위 | 좌우 분기 + 좌→우 핸드오버 | **오른팔 단독**, 핸드오버 없음 |
| 접근 | `vertical_from_above` | **작업대 법선에서 기울어진 실측 자세** |
| 최우선 블로커 | 좌우 base 변환 실측(M2) | **그리퍼 jaw gap 실측(M3)** |
| 캔 장축 모델 | `gripper_rotation @ jaw_axis` | **finger 축에 수직인 수평 방향** |
| wrist_roll 해법 | `roll += Δfinger_yaw` | **수치 근 탐색 + 한계 인지 분기 선택** |

핵심은 두 가지다. **핸드오버를 빼면 최우선 블로커가 같이 사라진다.** 그리고
**캔에서 진짜 미지수는 인식이 아니라 그리퍼다.**

## 1. v1 범위와 근거

**v1 = 오른팔 단독 캔 픽 → 고정 쓰레기통 release.** 왼팔은 q0 torque hold.

근거는 도달성 실측이다. 제안된 bin release 지점
`(0.260, -0.480, 0.095)` workcell에 대해:

| arm | 결과 | 자세 |
|---|---|---|
| left | **도달 불가**, 88.27 mm 부족 | — |
| right | 도달 | 접근축 기울기 84.7° |

왼팔이 못 닿는다는 사실 하나가 2026-08-15 계획에서 핸드오버를 강제했다.
그런데 핸드오버는 좌우 base 변환 실측(M2)에 정밀도가 묶여 있고, 그 변환은
지금 CAD-fit이며 실측이 아니다. **v1을 오른팔 단독으로 좁히면 핸드오버가
사라지고 M2가 임계 경로에서 빠진다.**

M2가 완전히 무관해지지는 않는다. 오른팔도 workcell 좌표를 자기 base 좌표로
바꿀 때 같은 변환을 쓴다. 다만 펜 작업이 이미 그 경로로 오른팔 pick을
반복 성공시켰고, 잔여 계통 오차는 화면축 보정 상수 `-29.47 mm`가 흡수하고
있다. 지름 53 mm 캔은 지름 15 mm 펜보다 관대하므로 v1은 이 상태로 성립한다.
M2는 v2(왼팔 분기 또는 핸드오버)의 선행 조건으로 남긴다.

v2 선택지는 두 개이며 v1 통과 후에 결정한다.

1. 두 팔이 모두 닿는 곳으로 쓰레기통을 옮긴다. 예: `(0.300, -0.330, 0.120)`은
   좌(75.9°)·우(23.5°) 모두 도달한다. 단 왼팔이 오른팔 위를 가로지르므로
   inter-arm collision 검사가 선행 조건이다.
2. 팔마다 자기 바깥쪽에 쓰레기통을 하나씩 둔다. 교차가 없어 가장 안전하지만
   쓰레기통이 두 개다.

핸드오버는 세 번째 선택지이며, 위 둘이 모두 불가할 때만 되살린다.

## 2. 실측으로 확인한 기하 — 이전 계획의 전제 세 개를 대체한다

모든 수치는 `so101_dual_right_data_fit_candidate.urdf`와
`config/bimanual_operational_limits.json`(승인본)으로 계산했다. 검증은
2026-08-16 session03의 **실제 실행된** 오른팔 `pick_grasp` 자세로 했다.
같은 자세에서 이 계산의 finger yaw가 계획 파일이 기록한
`achieved_finger_yaw_rad = -1.278442`와 일치한다. 즉 아래 수치는 새 모델이
아니라 이미 실기로 검증된 모델에서 나온 값이다.

### 2.1 `vertical_from_above`는 달성 불가능하다 — 폐기

`config/lying_can_upright_contract.candidate.json`의
`"approach": "vertical_from_above"`와 `"floor_sweep_authorized": false`는
이 팔이 이 작업대에서 만들 수 없는 자세다.

실행된 펜 `pick_grasp` 자세의 접근축은 수직에서 **64.28°** 기울어 있다.
`gripper_link` 원점에서 `gripper_frame_link` 원점으로 가는 98.4 mm 실제
링크 방향으로 프레임 규약과 무관하게 재확인했다: **64.36°**. 즉 도구는
수평보다 약 26° 위에서 들어온다.

TCP 위치를 유지한 채 기울기를 최소화해도 **51.5°**가 하한이다.

보정 영역 전체의 지도(작업대 높이 z=0.0053, roll=0):

| x | y | left 위치해 기울기 | left 최소 기울기 | right 위치해 기울기 | right 최소 기울기 |
|---:|---:|---:|---:|---:|---:|
| 0.34 | -0.28 | 64.5° | 54.7° | **14.0°** | 13.9° |
| 0.34 | -0.14 | 54.0° | 19.7° | 24.0° | 17.4° |
| 0.34 |  0.00 | 33.4° | **8.4°** | 49.0° | 42.2° |
| 0.40 | -0.28 | 도달 불가 | — | 55.8° | 35.6° |
| 0.40 | -0.14 | 53.0° | 41.6° | 73.7° | 39.0° |
| 0.40 |  0.00 | 44.2° | 30.5° | 도달 불가 | — |
| 0.46 | -0.28 | 도달 불가 | — | 77.2° | 75.7° |
| 0.46 |  0.00 | 72.3° | 63.1° | 도달 불가 | — |
| 0.52 | * | 도달 불가 | — | 도달 불가 | — |

두 가지가 나온다.

- **각 팔은 자기 base 정면에서만 수직에 가까워진다.** 오른팔은 y=-0.28,
  왼팔은 y=0.00 쪽이다. 캔 배치와 routing 규칙이 이 사실과 맞아야 한다.
- **보정 영역의 상당 부분이 작업대 높이에서 도달 불가다.** 특히 x ≥ 0.46.
  인식이 되는 영역과 잡을 수 있는 영역은 다르며, 계획기는 둘을 각각 검사해야
  한다.

**대체 계약**: 접근을 `vertical_from_above`로 명명하지 않는다. 매 계획마다
접근축 기울기를 FK로 계산해 기록하고, 계약은 상한 하나로 표현한다.
`maximum_approach_tilt_deg`는 M3에서 실측한 finger 간섭 한계로 정한다.
`floor_sweep_authorized: false`는 **유지한다** — 기울어진 접근과 바닥 훑기는
다른 문제이며, 하강은 계속 연직으로만 한다.

### 2.2 캔 장축은 jaw 축이 아니다 — 기존 핸드오버 산출물 무효

2026-08-15 계획 §2.3은 "평행 조 그리퍼가 원통을 잡으면 캔 장축 = 그리퍼 jaw
축"이라며 `gripper_rotation @ _jaw_axis_in_gripper`로 캔 축을 계산하라고 지시했다.

`_jaw_axis_in_gripper`는 **moving jaw의 경첩 회전축**이다. 캔 축이 아니다.
실행된 펜 grasp 자세에서 이 축은 `[0.4225, 0.1029, 0.9005]`, 즉 거의 연직이다.
작업대에 누운 캔의 축이 연직일 수는 없다.

올바른 모델은 이렇다. 평행 조가 원통을 물면 원통 축은 **닫힘 방향(finger
축)에 수직**이고 조 면 안에 있다. 캔이 작업대에 누워 있으므로 그 중에서
**수평**인 방향으로 확정된다.

```text
캔 장축 = finger 축에 수직인 수평 단위벡터
        = yaw(finger_yaw + 90°)
```

`finger_axis()`는 이미 `cross(jaw_axis, approach)`로 정확히 계산된다. 바꿀
코드는 없고, 바꿀 것은 캔 축을 유도하는 방식이다.

**영향**: `artifacts/can_to_bin/handover_pose_m1_plan_only.json`의
`can_axis_world = [0.7159, 0.1560, -0.6806]`과 `can_tilt_from_horizontal_deg
= 42.89`는 경첩축을 캔 축으로 오인한 값이다. "핸드오버 시점 캔이 28° 기운다"는
결론도 여기서 나왔다. **이 산출물과 결론은 폐기한다.** (핸드오버 자체를 v1에서
빼므로 재계산도 지금은 불필요하다.)

### 2.3 wrist_roll → finger yaw는 1:1이 아니다

`nearest_gripper_crossing_yaw()`와 `GraspYawKinematics.solve_wrist_roll()`은
둘 다 `roll_new = roll_now + Δfinger_yaw`, 즉 gain 1을 가정한다.

실제 gain은 자세에 따라 다르다.

| 자세 | d(finger_yaw)/d(wrist_roll) |
|---|---:|
| q0 | +0.4993 |
| 실행된 펜 `pick_grasp` | +0.4343 |

이유는 구조적이다. finger yaw는 finger 축을 **수평면에 투영한** 방위각이다.
회전축(=접근축)이 연직일 때만 투영이 1:1이 된다. 접근축이 수직에서 64°
기울어 있으면 투영이 압축되어 gain이 0.5 근처로 떨어진다. §2.1에서 접근축이
연직이 될 수 없다고 확인했으므로, **gain 1 가정은 이 로봇에서 성립하지 않는다.**

`solve_wrist_roll()`은 계산 후 `residual_rad`를 되돌려주므로 조용히 틀리지는
않는다. 그러나 수렴시키지 않으므로 그대로 쓰면 목표 yaw를 못 맞춘다.

**대체 계약**: 해석식을 버리고 `finger_yaw(roll) = target`을 **수치로 푼다.**
gain 부호가 일정하고 구간별 단조이므로 격자 + 이분법이면 충분하다.

### 2.4 wrist_roll은 접근축을 안 바꾸지만 TCP는 움직인다

| 검사 | 결과 |
|---|---|
| roll 전 구간에서 접근축 변화 | **0.00°** (q0와 off-q0 모두) |
| roll 전 구간에서 TCP 이동 | **최대 13.3 mm** |

앞의 것은 2026-08-15 계획 §2.1이 맞았다. 뒤의 것은 그 문서에 없다.

원인은 URDF에 있다. `right_gripper_frame_joint`의 origin은
`xyz = (-0.0079, -0.000218, -0.0981274)`다. TCP가 roll 축에서 **7.9 mm**
옆으로 나가 있어, roll을 돌리면 TCP가 지름 15.8 mm 원을 그린다.

**영향**: 펜 앱은 roll을 0에 고정했으므로 이 효과가 상수였다. 캔은 검출마다
roll이 달라지므로 상수가 아니다. **"MoveIt으로 xyz를 풀고 그 위에 roll을
얹는" 방식은 최대 15.8 mm 파지 오차를 만든다.** 지름 53 mm 캔에서 치명적이다.

반드시 **roll을 먼저 정하고, 그 roll을 고정한 상태에서 나머지 4축으로 TCP
xyz를 다시 푼다.** 기존 `solve_endpoint_pose_with_locked_wrist()`가 이미 이
구조다 — `locked_wrist_roll = 0.0` 하드코딩을 인자로 바꾸는 것이 변경의 전부다.

### 2.5 회전량과 분기 — 사용자 질문의 핵심

실행된 펜 grasp 자세에서 오른팔 roll 한계 `[-114.17°, +81.04°]`(span 195.21°)
전체를 훑어 얻은 결과다.

| 항목 | 결과 |
|---|---|
| finger yaw 도달 범위 | `[-89.9°, +89.9°]` |
| 캔 yaw 1° 구간 180개 중 도달 가능 | **180 / 180** |
| 한계 안에 분기가 **2개**인 구간 | **25 / 180** |
| q0에서의 필요 회전량 | 최대 98.5°, 평균 60.5° |

세 가지가 따라온다.

1. **모든 캔 방향을 잡을 수 있다.** roll span이 195°로 180°를 넘으므로 무방향
   장축의 어떤 yaw도 표현된다. 도달 불가 방향은 없다.
2. **180개 중 25개 구간에서는 유효한 해가 두 개다.** 두 해는 roll 축에서 약
   180° 떨어져 있고 둘 다 한계 안이다. 여기서 "가까운 쪽"을 골라야 불필요한
   반바퀴 회전이 사라진다. 이것이 사용자가 지적한 최단거리 요구다.
3. **나머지 155개 구간에서는 해가 하나뿐이다.** 이때 수학적으로 더 가까운
   분기는 한계 밖이다. 따라서 "가장 가까운 분기"를 무조건 고르면 한계 위반이고,
   **한계 검사가 분기 선택보다 먼저**여야 한다.

`nearest_gripper_crossing_yaw()`는 `(-90°, +90°]`로 감아 수학적 최근접만
계산하고 한계를 전혀 보지 않는다. `solve_wrist_roll()`은 한계를 `within_limits`
불리언으로 **보고만 하고** 대체 분기를 찾지 않는다. **둘 다 지금 상태로는
쓸 수 없다.**

## 3. 캔 방향 → 그리퍼 회전 계약

v1에서 확정할 알고리즘이다. 입력은 YOLO-OBB의 캔 장축 yaw(board 좌표,
`undirected_long_axis_modulo_pi`)와 현재 실측 roll이다.

```text
1. 목표 finger yaw
     target = wrap_half_turn(can_axis_yaw + pi/2)
   캔 장축과 손가락 닫힘선을 90°로 교차시킨다.

2. 분기 열거 (해석식 아님)
     f(roll) = wrap_half_turn(finger_yaw(arm_pose, roll) - target)
   roll 한계 구간을 격자로 훑어 f의 부호 변화 구간마다 이분법으로 근을 구한다.
   wrap 불연속은 인접 표본의 도약 크기로 걸러낸다.
   arm_pose 는 pick_pregrasp 4축 해를 쓴다. 자세마다 gain이 다르므로
   q0 에서 계산하지 않는다.

3. 한계 필터
     한계 밖 근은 여기서 버린다. 남은 것이 없으면 계획을 거부한다.
     (§2.5에 따르면 정상적으로는 항상 1개 이상 남는다. 0개는 버그 신호다.)

4. 최단 분기 선택
     남은 근 중 |roll - roll_now| 가 최소인 것.
     동률이면 관절 한계 여유가 큰 쪽.

5. roll 고정 후 위치 재해
     선택한 roll 을 고정하고 나머지 4축으로 TCP xyz 를 다시 푼다 (§2.4).

6. FK 재검증
     achieved_finger_yaw 를 FK 로 다시 계산하고
     |achieved - target| <= crossing_tolerance 를 확인한다.
     초과하면 거부한다. 이 메타데이터가 없는 계획은 실행기가 거부한다.
```

`crossing_tolerance`는 인식 오차에서 유도한다. 캔 OBB holdout의 yaw 오차는
p95 `2.36°`, 최대 `3.08°`다(§5). 여기에 계획 잔차와 서보 정착 오차를 더해
정한다. **임의의 상수로 정하지 않는다.**

교차각이 어긋나면 무엇을 잃는지는 계산된다. 길이 132.44 mm, 지름 53 mm 캔을
닫힘선이 수직에서 θ만큼 벗어나 무는 경우, 조가 벌려야 하는 폭은

```text
w(θ) = 53·cos θ + 132.44·sin θ
```

θ=0°에서 53 mm, θ=10°에서 75 mm, θ=36°에서 121 mm다. 실행된 펜 계획이
기록한 `crossing_residual_rad = 0.627` = **35.9°**가 바로 이 상황이며,
펜에서는 통했지만 캔에서는 조가 벌어지지 않는다. **roll을 푸는 것이 캔
작업의 필수 조건인 이유가 이 식이다.**

## 4. 상태기계 (오른팔 단독)

| 단계 | 계약 |
|---|---|
| `top_lock` | 정확히 1개, 완전 가시, 보정 영역 안, class/크기/aspect/yaw 통과 |
| `arm_reach_gate` | **신규.** 잠긴 target이 오른팔 작업대 높이 도달 영역 안인지 검사 (§2.1) |
| `roll_branch_solve` | §3. 실패 시 계획 거부 |
| `open_gripper` | 캔 지름 + 여유까지 개방. M3 값 전까지 거부 |
| `high_pregrasp` | 캔 위 충분한 clearance, 위에서 푼 roll 적용 |
| `vertical_descend` | 연직 하강만. floor sweep 금지 |
| `close_gripper` | 몸통 파지, 잔차 ≥ contact threshold (M3) |
| `lift_clear` | 연직 lift |
| `bin_transit` | 쓰레기통 개구부 위로 이동 |
| `bin_release` | 고정 높이 release, 잔차 ≤ release tolerance |
| `vertical_retreat` | 연직 후퇴 |
| `return_q0_hold` | 양팔 q0, torque hold |

`arm_reach_gate`는 새로 넣는다. §2.1에서 인식 영역과 파지 영역이 다르다는 것이
드러났으므로, 인식만 통과하고 IK에서 뒤늦게 실패하는 경로를 앞에서 막는다.

wrist 카메라는 v1에서 쓰지 않는다. 오른쪽 `wrist_b`는 지금 `device_path`만
있고 `frame_id`·`camera_info`·URDF optical frame이 모두 없다. v1은 top 카메라
단독으로 성립해야 한다.

## 5. 사람이 채워야 하는 값 (M 단계)

| ID | 내용 | 상태 |
|---|---|---|
| M1 | 캔 길이·지름·질량 | **완료.** 132.44 mm / 53.0 mm / 13 g, 2026-08-15 실측. 허용오차 미정 |
| M3 | **그리퍼 jaw gap·contact·offset** | **미완료 — v1 최우선 블로커** |
| M4 | 쓰레기통 기하·release 높이·collision | 기하/높이 있음, collision 미검사 |
| M5 | 캔 top OBB 데이터셋·학습·holdout | **사실상 완료** (아래) |
| M2 | 좌우 base 변환 실측 | v2로 이월 (§1) |
| M6 | wrist 상대 보정 대역 | v1 범위 밖 (§4) |

### M5는 이미 통과 수준이다

`artifacts/top_can_obb/2026-08-16/`의 번들이 holdout 23장(양성 20, 음성 3)에서:

| 지표 | 값 |
|---|---:|
| precision / recall | 0.997 / 1.000 |
| mAP50 / mAP50-95 | 0.995 / 0.972 |
| yaw 오차 mean / p95 / max | 0.98° / 2.36° / 3.08° |
| 중심 오차 mean / p95 / max | 1.78 / 3.40 / 3.55 px |
| 음성 오검출 | 0 |

2026-08-15 번들 대비 중심 오차 p95가 83.8% 줄었다. **캔 방향 인식은 블로커가
아니다.** 남은 일은 holdout이 23장으로 얇다는 점이며, v1 실기 전에 확장한다.

### M3가 진짜 블로커인 이유

jaw gap을 mm로 잰 기록이 저장소 어디에도 없다.
`docs/checklists/PHASE_5_GRIPPER_MAPPING_PLAN.md`는 측정 절차만 정의하고
결과표는 "사용자가 측정"으로 비어 있다. URDF의 `gripper_joint` 한계는
`0..1.91986 rad`이지만 이것은 각도이지 mm가 아니다.

즉 **지름 53 mm 캔이 이 그리퍼에 들어가는지 아무도 확인한 적이 없다.**
펜 작업이 쓴 개방값은 raw 2048(0 rad)로, 개방 범위의 거의 닫힌 끝이다.

기존 `tools/commission_can_gripper_probe_once.py`는 target raw를
`1948..2009`로 제한한다. 이것은 펜의 닫힘 대역이며 **캔에 필요한 개방을 명령할
수 없다.** 이 도구는 확장이 필요하다.

M3에서 확정할 값:

- jaw gap mm ↔ command rad 대응표 (단조성·hysteresis 포함)
- 캔 지름 53 mm에 필요한 개방 command와 그때의 여유
- 캔 몸통 접촉 잔차 raw (펜의 `contact_threshold_raw = 14`를 상속하지 않는다)
- release 잔차 tolerance
- 기울어진 접근에서 아래쪽 finger가 작업대에 닿기 전까지 허용되는 접근축
  기울기 → §2.1의 `maximum_approach_tilt_deg`

## 6. 작업 순서

각 단계를 통과하기 전에 다음으로 가지 않는다.

```text
W0  기하 검증 도구 + artifact          → §2 수치를 재현  [완료]
W1  can_to_bin_application.py 계약 모듈 → pytest, null gate
W2  can_to_bin_contract.candidate.json  → M 값 전부 null
W3  roll 분기 solver (§3)               → pytest, 180/180 및 25개 2분기 재현
[M3 그리퍼 실측]                        ← 사람  ★ v1 최우선 블로커
W4  plan_can_to_bin_once.py (plan-only) → plan-only PASS + SHA
W5  run_can_to_bin_application_once.py  → --validate-only, motion_commands=0
[M4 collision]                          ← 사람
    오른팔 감독 실기 1회 → 연속 2회
```

W0~W3와 W5의 `--validate-only`까지는 하드웨어 없이 지금 진행할 수 있다.
M3 없이 실기로 넘어가지 않는다.

### W0 산출물 (완료)

```bash
python3 tools/probe_can_grasp_geometry_plan_only.py
```

`artifacts/can_to_bin/can_grasp_geometry_plan_only.json`,
SHA-256 `686090d120a7d8d6854019b23d1136a47c6d7518bd02132775639582304601ed`.
`CAN_GRASP_GEOMETRY_PLAN_ONLY_PASS motion_commands=0 execution_api_used=false`.

이 도구는 시작할 때 실행된 오른팔 `pick_grasp` 자세의 finger yaw가 계획 파일이
기록한 `-1.278421260830095`와 `1e-6 rad` 안에서 일치하는지 먼저 검사하고,
어긋나면 즉시 실패한다. 즉 §2의 수치는 새 모델의 주장이 아니라 **이미 실기로
검증된 모델**에서 나온 값이다.

`solve_wrist_roll_branches()`의 계약은
`tests/test_can_grasp_roll_branches.py` 13개 시험이 고정한다.

## 7. 유지되는 하위 계약 — 바꾸지 않는다

- firmware를 수정하지 않는다. 이 작업은 전부 상단 애플리케이션 계층이다.
- 상단 앱이 serial을 직접 열지 않는다. 경로는 resident adapter 12축 하나뿐이다.
- `config/bimanual_operational_limits.json`의
  `general_trajectory_output_available=false`를 바꾸지 않는다.
- 명령은 canonical 12축 절대 rad. gripper는 index 5/11이며 별도 API가 아니다.
- 수락 판정은 firmware/resident 계약값을 그대로 쓴다. 앱이 더 엄격한 중복
  gate를 만들지 않는다.
- 자동 재시도 0. `accepted=false`면 즉시 status를 읽고 중단한다.
- fault 후 무인 자동 reset/clear/resume loop를 만들지 않는다.
- 정상 finite 완료는 torque-off가 아니다. 종료 시 명시적으로 STOP한다.
- STOP 후 팔이 처지므로 물리적으로 지지한 뒤 정지한다.
- 펜 앱의 `PICK_GRASP_OFFSET_M`, gripper raw 목표, contact threshold를 캔에
  상속하지 않는다.
- 쓰레기통에 삽입하지 않는다. 개구부 위 release만 한다.
- `lying_can_upright_application.py`의 기존 세우기 계약을 약화시키지 않는다.
  `bottom_end_sign` 완화는 이 앱의 호출부에서만 일어난다.

## 8. 관련 문서

- [양팔 상단 애플리케이션 인터페이스](BIMANUAL_UPPER_APPLICATION_INTERFACE.md)
- [Top-camera resident Pick/Place](TOP_CAMERA_RESIDENT_PICK_PLACE_APPLICATION.md)
- [현재 상태와 다음 로드맵](CURRENT_STATE_AND_NEXT_ROADMAP.md)
- [F8.9 최종 수락 결과](test-results/2026-08-16-f89-bimanual-pen-transfer.md)
