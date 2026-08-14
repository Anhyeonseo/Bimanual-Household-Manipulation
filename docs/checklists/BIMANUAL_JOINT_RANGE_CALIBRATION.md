# 양팔 관절 안전 가동범위 계측 체크리스트

## 목적과 배치

이 gate는 P2S real-feedback/no-output 실기 통과와 R4 read-only 복구 뒤,
non-zero protocol-v2 goal output을 연결하기 전에 수행한다. 같은 모델·같은 조립
방식이어도 케이블 routing, wrist camera mount, base 간격과 조립 공차가 다를 수
있으므로 왼팔과 오른팔을 독립적으로 계측한다.

raw `0..4095`나 서보가 더 회전할 수 있다는 사실은 로봇 관절의 안전 가동범위
증거가 아니다. 이 절차는 물리 hard stop을 모터 힘으로 찾지 않는다.

## 공통 금지 사항

- 자동 endpoint sweep, stall을 이용한 끝단 검출, hard-stop 접촉 후 추가 명령 금지
- 두 관절 이상 동시 명령, 한 번에 전체 후보 범위 왕복, 무인 측정 금지
- 관측 extrema를 그대로 firmware/URDF limit으로 자동 반영 금지
- 한 팔의 결과를 다른 팔에 복사해 measured limit이라고 표기 금지
- J0/J1 통과 전 torque enable 또는 goal-position 기반 범위 시험 금지

## J0 — torque-off physical/desired envelope 관측

선행 조건:

- P2S hardware gate 통과 후 R4 `0x00023B00` 복구와 identity/heartbeat 확인
- bridge가 `BIMANUAL_READ_ONLY`이며 양팔 Torque Enable=0 readback 완료
- 작업대와 주변 장애물 정리, 비상 전원 차단 가능, 사용자가 팔 무게를 지지

J0 evidence는 서로 목적이 다른 두 종류로 분리한다.

- **J0-M (mechanical/cable envelope)**: 각 팔의 관절을 한 축씩 격리해
  기구·케이블·마운트가 허용하는 물리 범위를 보수적으로 관측한다. 선택하지 않은
  같은 팔 관절은 손으로 지지해 자세 변화를 최소화한다.
- **J0-D (desired/task envelope)**: 사용자가 실제로 쓰고 싶은 workspace와
  대표 task 동작을 torque-off로 자연스럽게 재현한다. 여러 관절의 협조 움직임을
  허용하며 각 관절의 task-required extrema와 결합 자세를 기록한다.

J0-M에서는 선택 축을 천천히 움직이다가 다음 중 하나가 처음 나타나기 전에 멈춘다.

- 기구 간 접촉 또는 self-collision 접근
- 케이블 장력, 꺾임, 커넥터 당김
- wrist/base camera mount 간섭
- 물리 저항 증가 또는 hard stop 접근
- 사용 작업공간에 불필요한 자세

J0-M은 한 축·한 방향을 최소 3회 반복하고 raw extrema, 반복 편차, 제한 원인과 사진을
남긴다. 어깨/팔꿈치처럼 이웃 관절 자세에 따라 충돌 범위가 달라지면 q0와 실제
작업 대표 자세에서 별도 측정한다. 1차원 raw limit으로 표현할 수 없는 결합 제한은
억지로 넓은 scalar limit으로 만들지 않고 URDF/SRDF collision 및 workspace
constraint 항목으로 남긴다.

J0-D는 대표 접근·파지·들기·놓기·retreat 및 정책이 사용할 workspace를 최소 3회
재현한다. 한 관절을 선택해 시작했더라도 같은 팔 비선택 관절 span이 크면 실패로
폐기하지 않고 `coordinated_task_envelope`로 재분류한다. 단, 이것은 J0-M 단축
한계를 대신하지 않는다.

J0 산출물은 evidence일 뿐이며 다음을 고정한다:

```text
motion_authorized=false
apply_to_calibration=false
automatic_limit_expansion=false
```

### J0-G — gripper aperture mapping

gripper는 arm의 일반 revolute joint와 다르게 취급한다. servo raw sweep은 입력축
범위를 보여줄 뿐이며 jaw의 실제 벌어짐, 물체 접촉 방향 또는 linkage의
over-center 여부를 직접 증명하지 않는다. 따라서 J0-D의 raw min/max를 그대로
URDF joint angle이나 firmware command limit으로 복사하지 않는다.

오른팔 첫 J0-D 수동 open/close 관측은 raw `1941..3299`, reversals 7,
wrap 0이었다. 사용자는 이 관측을 포함한 이번 J0-D 전 구간에서 케이블 당김,
꼬임 또는 간섭이 없었다고 확인했다. 이 결과는 desired raw envelope evidence로
유효하지만 aperture mapping gate는 아직 통과하지 않았다.

후속 정지 checkpoint에서 작업자가 확인한 무부하 닫힘은 raw `1907`에서
span 0, 실제로 사용할 가장 큰 물체에 충분한 task-open은 raw `3062`에서 span
0이었다. task-open은 mechanical maximum이라는 주장이 아니며 사용자는 그보다
더 벌릴 필요가 없다고 판정했다. 따라서 hard stop이나 최대 opening을 추가로
탐색하지 않는다. 이미 cable-safe하게 관측된 raw `3299`는 task-open `3062`
바깥에 237 raw의 관측 여유가 있음을 보여줄 뿐 command endpoint로 채택하지
않는다.

현재 실기 검증된 close raw `1963`과 task-open raw `3062` 사이를 gripper의
J0-D required interval 후보로 둔다. 무부하 닫힘 `1907`은 close 쪽에 56 raw의
기구 기준 여유를 제공하지만, 이 값만으로 tracking/contact margin이 충분하다고
승인하지 않는다. raw `2009` release 명령은 이 required interval 안에 있다.

현재 계층에는 먼저 해소해야 할 의미 불일치도 있다. 물체를 실제로 든 왼팔
Pick/Place는 `0.13 rad`/raw `1963`을 close, `0.06 rad`/raw `2009`를 release로
검증했지만, SRDF named state와 기존 URDF 설명은 `q=0`을 closed, 큰 `+q`를
open으로 표기한다. 실기 증거가 우선이며 named state·주석을 근거로 오른팔
방향을 추정하지 않는다.

J1 전에 torque-off 상태에서 최소 다음 aperture checkpoint를 같은 jaw 면과
같은 측정 도구로 기록한다.

- 물체 없이 자연스럽게 닫힌 usable endpoint의 raw와 jaw gap mm
- 실제 Pick에 사용한 정도의 close/contact raw와 jaw gap mm
- 실제 release에 충분한 raw와 jaw gap mm
- 사용자가 원하는 task-open endpoint의 raw와 jaw gap mm
- 각 구간을 양방향으로 반복했을 때 gap이 단조롭게 변하는지와 hysteresis

raw가 한 방향으로 변하는 동안 gap 방향이 바뀌면 over-center 이후 구간은
운용범위에서 제외하거나 별도 piecewise mapping으로 모델링한다. monotonic usable
segment가 확인되면 다음 세 값을 분리해 고정한다.

1. firmware의 보수적 servo raw command limit
2. ROS/policy에 노출할 gripper coordinate와 raw↔coordinate mapping
3. URDF/MoveIt/Isaac의 실제 moving-jaw angle 또는 jaw-aperture geometry

1:1 servo-shaft-radian 가정은 jaw 계측으로 확인되지 않는 한 유지하지 않는다.

R4 bridge가 켜진 상태에서는
`tools/observe_bimanual_joint_range_torque_off.py`를 팔·관절 하나씩 실행한다.
도구는 `/bimanual_joint_states`만 구독하고 12축 rad를 calibration으로 raw에
역변환해 선택 관절 extrema, 방향 반전 횟수와 비선택 관절 span을 기록한다.
motion/torque/fault service는 호출하지 않는다.
같은 팔의 비선택 관절 span은 단축 격리 품질을, 반대 팔 span은 측정 중 공유
collision context가 유지됐는지를 각각 독립적으로 판정한다.
선택 관절이 12-bit encoder의 `4095↔0` 경계를 통과하면 선형 min/max를 범위로
쓰지 않고 연속 sample의 modular delta로 만든 `unwrapped_*` 결과를 사용한다.
`maximum_step_raw`가 너무 크면 sampling 사이의 방향을 확정할 수 없으므로 더
천천히 움직이고 endpoint에서 최소 2초 멈춰 재측정한다.

## J1 — 보수 운용한계 산출과 전체 계층 parity

팔별·관절별 operational limit 후보는 J0-M의 contracted safe envelope 안에
있으면서 J0-D의 expanded task envelope를 포함해야 한다. 기본 mechanical margin은
물리 관측 끝단에서 안쪽으로 64 raw다. task envelope에는 별도의 tracking/settling
margin을 바깥쪽으로 더한다.

```text
M_safe = [M_observed_min + mechanical_margin,
          M_observed_max - mechanical_margin]
D_required = [D_observed_min - tracking_margin,
              D_observed_max + tracking_margin]
require D_required subset_of M_safe
```

조건을 만족하지 못하면 limit을 억지로 넓히지 않고 desired workspace를 줄이거나
케이블/마운트/기구 배치를 수정한다. 다음 값 중 더 보수적인 margin을 사용한다.

- 반복 측정 편차와 encoder settle 오차를 덮는 값
- 케이블/마운트/충돌 불확실성을 덮는 값
- q0와 실제 task route가 endpoint에 놓이지 않게 하는 값
- 그리퍼 linkage와 물체 접촉을 고려한 별도 opening/closing margin

64 raw는 과거 왼팔 Shoulder `3830→3766`, Elbow `563→627`에서 사용한 출발점일
뿐 모든 관절에 충분하다는 보장은 아니다. limit 확대와 축소는 모두 근거를 남긴다.

후보가 정해지면 다음을 한 변경 단위로 맞춘다.

- 왼팔/오른팔 calibration JSON과 독립 arm-bound identity/hash
- STM32 calibration/command limit
- ROS raw↔rad mapping과 MoveIt joint limits
- URDF/Xacro 및 Isaac articulation limits
- q0, representative task routes와 collision scene plan-only 검사

양팔 측정값이 같을 때만 같은 hash를 허용한다. 다르면 팔별 hash가 다른 것이
정상이며 protocol-v2 HELLO에서 각각 검증한다.

### J1-W — encoder wrap 좌표 계약

J0-D 실기에서 양쪽 SHOULDER가 물리적으로 연속인 채 raw `4095↔0`을 반복
통과했다. 왼쪽은 unwrapped `1899..4187`/wraps 4, 오른쪽은
`1859..4188`/wraps 8이었다. wrap 관절은 선형 `raw - zero_raw`나
`minimum_raw..maximum_raw` 한 구간으로 표현하지 않는다.

- feedback은 인접 sample의 signed modular delta로 unwrapped joint coordinate를
  유지한다. 한 sample 간 이동이 반 바퀴 이상이면 방향이 모호하므로 fail-closed한다.
- 부팅 직후 raw만으로 branch를 추측해 motion을 승인하지 않는다. 검증된 q0/home,
  직전 command branch 또는 별도 operator-confirmed commissioning 절차로 branch를
  결박해야 한다.
- trajectory와 joint limit은 unwrapped coordinate에서 검증한 뒤에만 servo raw로
  modulo 변환한다. modulo 변환이 shortest-path나 반대 branch 명령을 만들지 않는지
  별도 시험한다.
- ROS/MoveIt/URDF/Isaac과 protocol-v2 모두 같은 unwrapped lower/upper를 사용한다.
- wrap-aware feedback, validation-only stream, shadow output을 먼저 통과하고 J2
  전까지 실제 goal dispatch는 계속 금지한다.

J1-W `0x00024000` 실기 순서는 다음과 같다.

1. R4 `BIMANUAL_READ_ONLY`에서 양팔 torque-off와 shoulder 중간 branch를 확인하고
   12축 reference를 SHA-bound artifact로 캡처한다.
2. J1-W 후보로 교체한 뒤 같은 자세에서 explicit bind와 stationary update를
   통과한다.
3. output-disconnected observer에서 왼쪽과 오른쪽 SHOULDER를 각각 천천히
   `4095↔0` 너머로 움직여 firmware/host unwrapped 값 exact match를 확인한다.
4. R4를 복구하고 12축 read-only soak를 재확인한다.

2026-08-13 위 네 단계는 모두 **PASS**했다. 양쪽 SHOULDER wrap count는 각각 2,
firmware/host mismatch는 0, R4 복구 soak는 100/100이었다. 이 결과는 wrap 좌표
계약만 승인하며 operational limit이나 실제 goal dispatch를 승인하지 않는다.

### J1-L — operational limit plan-only 후보

2026-08-13 J0-D reviewed manifest를 입력으로 arm 5축의 관측 양 끝을 각각
64 raw 안쪽으로 수축한 후보를 만들었다. 두 SHOULDER는 J1-W의 unwrapped
좌표를 사용하며 모든 후보는 q0 raw 2048을 포함한다. Gripper는 jaw aperture
의미 계약이 없으므로 자동 후보에서 제외했다.

기존 왼팔 offset 0.011 m Pick–Place manifest의 7개 phase, 1,031 trajectory
point를 전수 검사한 결과 arm 5축이 모두 후보 안이었다. 최소 limit clearance는
Shoulder의 0.130388 rad였다. 상세 수치와 SHA는
`docs/test-results/2026-08-13-bimanual-j1-operational-limit-candidate.md`에
고정한다.

사용자는 64 raw 수축안을 다음 검증 후보로 승인했다. `0x00024100` no-output
candidate의 arm 10축 firmware/host parity와 실기 shadow 검증을 통과했고, R4 복구
후 12축 read-only soak 10/10도 통과했다. 같은 수치를 simulation-only 양팔 URDF,
미참조 MoveIt candidate와 Isaac import URDF에 투영해 model-stack parity를 통과했다.
Gripper, J0-M, physical q0/base transform, 오른팔 대표 경로와 J2가 남았으므로 active
calibration, MoveIt launch와 실제 goal runtime에는 아직 반영하지 않는다.

## J2 — 축별 bounded active 검증

J1 parity가 모두 통과한 뒤 별도 motion candidate에서 한 팔·한 축씩 수행한다.
반대 팔은 torque-off 또는 물리 전원 차단 상태로 두고, 사용자가 대상 팔을 지지한다.

1. 현재 위치 hold와 torque/PID/speed/limit readback 확인
2. q0에서 후보 범위 내부 25% 지점까지 저속 이동 후 복귀
3. 이상이 없을 때 50%, 마지막으로 75% 지점까지 각각 별도 승인해 왕복
4. 각 leg에서 raw 목표/실측, tracking error, 전류·부하·전압·온도, stop latency 기록
5. cable tension, 진동, 소음, 간섭, 오차 비수렴 중 하나라도 있으면 즉시 양팔 disable

2026-08-13 J1-L에서 양팔 arm 5축의 양방향 25/50/75% 내부 목표를 plan-only로
파생했다. 산출물 `artifacts/joint_ranges/2026-08-13/j2_axis_targets_plan_only.json`의
SHA-256은 `63e99ca8a0fa5231e50777486d6c051ef5db8aaca7f5bd45ccd672958e20e87c`다.
첫 실기는 오른팔 Base upper 25%, raw `2048→2269→2048` 한 축만 R4
`0x00023B00` 누적 primitive로 수행한다. 왼팔 12 V OFF, q0±10 raw, 비선택축
torque-off, 최대 20 raw step을 강제하고 성공은 verified disable, 오류는 latched
stop으로 끝낸다. 이는 첫 evidence 후보이며 J0-M이나 최종 J2 승격이 아니다.

endpoint와 0%/100% 지점은 명령하지 않는다. 한 축이 실패하면 그 팔의 다축
trajectory와 양팔 공통 dispatch는 계속 금지한다.

## 승격 조건

- 12축 모두 J0 evidence와 제한 원인이 기록됨
- 팔별 J1 calibration/hash 및 firmware/host/URDF/MoveIt/Isaac parity 통과
- 12축 모두 J2 25/50/75% 왕복과 fault/disable 검증 통과
- 대표 q0/task route가 새 limit에서 plan-only 및 collision gate 통과

이 조건 뒤에만 multi-joint 단팔 trajectory를 재검증한다. 두 팔의 단팔 기준선까지
통과한 뒤에만 protocol-v2 공통 apply-tick dispatch와 양팔 motion gate로 진행한다.
