# 왼팔 physical Home q0 재정렬 및 READ_ONLY TF 검증

- 날짜: 2026-07-26
- 대상: 정상인 SO-ARM101 왼팔
- 환경: Ubuntu 24.04.4, ROS 2 Jazzy, MoveIt 2.12.4, Isaac Sim 6.0.1
- 판정: **VISUAL/FK PIPELINE PASS — 외부 계측 TCP parity는 후속 gate**
- 초기 READ_ONLY 구간 실제 motion command: **0회**
- 후속 Top 등록 자세 실기: 아래 별도 기록

## 정식 q0 계약

servo one-key centering으로 정한 physical raw 2048 Home을 사진과 Isaac
interactive FK로 정렬했다. 기존 upstream 모델에서 해당 자세를 만들던 arm 5축
값을 joint origin에 흡수했다.

| project joint | upstream pose absorbed into q0 |
|---|---:|
| `left_base_joint` | 0 deg |
| `left_shoulder_joint` | +90 deg |
| `left_elbow_joint` | -55 deg |
| `left_wrist_flex_joint` | -57.5 deg |
| `left_wrist_roll_joint` | -90 deg |

초기 wrist roll `+90 deg` 후보는 정지 외형에서는 구분하기 어려웠지만 MoveIt
mock gripper-open 시험에서 실제 장착 방향과 반대임이 확인되어 폐기했다.
`-90 deg` 수정 후 MoveIt mock과 Isaac Sim 6.0.1 Robot Poser에서 gripper
open/closed 물리 방향 및 closed 복구를 사용자 확인으로 통과했다.

hardware calibration의 arm `zero_raw=2048`, SRDF Home all-zero, Isaac bridge
arm offset 0과 q0 전용 recovery 의미는 변경하지 않았다. gripper raw 2048의
실제 개방 폭과 project rad mapping은 별도 미확정 항목이다.

## 정적·simulation 검증

- q0/FK/preview regression: 15 tests PASS
- Isaac gripper simulation-only preview: 4 tests PASS
- Isaac bridge mapping: 3 tests PASS
- xacro 확장 및 `check_urdf`: PASS
- Isaac 6.0.1 USD layer parse: PASS
- 공식 SO101 32x32 UVC Wrist Roll replacement 반영:
  - source STL SHA-256
    `b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4`
  - ROS/MoveIt visual + collision: mm→m `0.001` scale 적용
  - Isaac visual + convex-hull collision: `9,398` points, `20,852` faces
  - composed extent: `65.2 x 113.6 x 120.2 mm`
  - 기존 Wrist Roll CAD origin, q0, TCP 및 joint transform 유지
  - Isaac Sim 6.0.1 GUI에서 실제 장착 형상과의 외형 정합 사용자 확인 PASS
- USD joint anchor 8개:
  - 최대 위치 오차 `3.6717e-9 m`
  - 최대 회전 오차 `5.1619e-8 rad`
- ROS 패키지 build: `so101_description`, `so101_isaac_bridge`,
  `so101_moveit_config` PASS
- MoveIt mock random-valid → Home 복귀 PASS
- MoveIt mock gripper open/closed 방향 PASS
- Isaac gripper open/closed 방향 및 closed 복구 PASS

## 실제 READ_ONLY 관측

준비 조건은 무부하, 빈 workspace, 물리 Home 유지, 즉시 12 V 전원 차단 가능
상태였다. Pi bridge는 `allow_motion=false` READ_ONLY로 실행했고 Action/motion
publisher는 사용하지 않았다. 전원 ON에서 갑작스러운 움직임·진동·이상음 없이
자세를 유지했다.

`/joint_states` publisher는 `/single_arm_bridge` 하나였고 관절 순서가 계약과
일치했다. 한 샘플은 다음과 같다.

```text
left_base_joint        +0.006135923 rad
left_shoulder_joint    -0.001533981 rad
left_elbow_joint       -0.021475731 rad
left_wrist_flex_joint  -0.004601942 rad
left_wrist_roll_joint  +0.007669904 rad
left_gripper_joint     +0.136524290 rad
```

- arm 최대 q0 편차: `0.021475731 rad`, 기존 `0.03 rad` 기준 안
- 12초 rate 관측: 평균 `4.998 Hz`
- interval: `0.199..0.203 s`
- feedback/identity/통신 오류: 없음
- actual motion command: 0회

shoulder, elbow, wrist-flex 일부 값은 one-sided strict command range에서 소량
벗어나지만 기존 `strict ±40 raw` recovery envelope 안이다. 따라서 READ_ONLY
관측은 가능하되 일반 실제 motion은 계속 금지한다.

## TF/FK 비교

read-only `robot_state_publisher`만 실행해
`workcell_base_link → left_gripper_frame_link`를 관측했다.

```text
actual feedback TCP = [0.211401, -0.007018, 0.158733] m
model exact q0 TCP  = [0.211228951, -0.008077998, 0.165147839] m
delta norm          = 0.006504103 m
```

현재 관절 피드백이 정확한 all-zero가 아니므로 model q0 대비 TCP 차이는
`6.50 mm`다. 동일한 실제 joint feedback을 오프라인 URDF FK에 넣으면:

```text
offline actual FK TCP = [0.211400844, -0.007017760, 0.158733433] m
ROS TF TCP             = [0.211401000, -0.007018000, 0.158733000] m
delta norm             = 0.000000519 m
```

`0.52 µm` 차이이므로 STM32 feedback → ROS joint convention → corrected URDF TF
파이프라인은 PASS다. 이는 encoder와 모델 내부 일관성 증거이며 외부 센서로 실제
TCP 위치를 계측한 결과는 아니다.

## 2026-07-29 외부 강체 타깃 축 방향 보정

후속 Top eye-to-hand 단계에서 강체 고정한 2x2 ArUco GridBoard 네 자세를
관측했다. 현재 URDF의 회전 불변량은 최대 `54.454 deg` 어긋났고 training
잔차는 `66.186 mm / 22.297 deg`였다. arm 5축 부호 32개를 전수 평가했다.

```text
current +++++ = 66.186 mm RMS / 22.297 deg RMS (23/32)
corrected --++- = 3.155 mm RMS / 0.291 deg RMS (1/32)
```

따라서 ROS feedback의 양의 방향과 URDF 물리 회전축을 맞추기 위해
`base_joint`, `shoulder_joint`, `wrist_roll_joint`를 local `-Z`에서 `+Z`로
수정한다. elbow와 wrist-flex는 `-Z`를 유지한다. q0 origin, STM32 firmware,
calibration hash `0x3DB42B48` 및 raw↔radian 변환은 변경하지 않는다.

## 잔여 gate

- 외부 카메라/고정 기준점을 이용한 실제 TCP metric parity
- q0 변경 후 Top–base visual registration 재수행
- collision geometry와 cable clearance 실물 확인
- 카메라 모듈을 포함한 실제 `gripper_link` 질량/관성 측정 및 반영
- gripper raw↔aperture/rad mapping 확정
- actual mechanical range 확정

위 항목을 통과하기 전 camera 기반 task motion과 Pick and Place 실행은 계속
fail-closed로 유지한다.

## 후속 Top 등록 자세 실기

공식 Wrist Camera Mount에 강체 고정한 노란 marker가 q0와 BASE `+0.06 rad`
자세에서 Top 카메라 영상 밖에 있어, arm 5축 `+0.10 rad`, 2초 단일 목표를
승인 후 한 번 실행했다. MoveIt과 STM32는 목표를 접수했고 사용자는 5축 모두
움직인 뒤 정지했다고 확인했다. 목표는 joint limit 및 MoveIt collision 검사를
통과했지만 Action은 aborted 되었고 host fail-safe가 STM32 SAFE_STOP을
래치했다. 자동 재시도와 자동 fault clear는 수행하지 않았다.

래치 후 피드백은 다음과 같았다.

```text
left_base_joint        +0.092038847 rad
left_shoulder_joint    +0.085902924 rad
left_elbow_joint       +0.064427193 rad
left_wrist_flex_joint  +0.082834963 rad
left_wrist_roll_joint  +0.090504866 rad
left_gripper_joint     +0.136524290 rad
```

`+0.10 rad` 목표 대비 raw 잔차는 각각 약 `5, 9, 23, 11, 6`이므로, elbow의
약 `23 raw`가 host 성공 기준 `20 raw`를 3 raw 넘긴 것이 가장 유력한 abort
원인이다. 최초 실행 당시 firmware terminal `detail`을 로그로 보존하지 않아 이
수치는 사후 joint feedback 기반 판정이다.

재발 시 원인을 잃지 않도록 bridge terminal 결과를 다음 구조화 로그로
보강했다.

```text
ARM_EXECUTION_TERMINAL state=<state> sequence=<seq> status=<status> detail=<detail> reason=<reason>
```

- pure execution tests: 16 PASS
- ROS arm/gripper Action integration tests: 18 PASS
- Pi source/install 반영 및 `TERMINAL_DIAGNOSTIC=True` 확인
- Pi `single_arm_bridge` rebuild PASS

물리 점검 후 latch를 해제하고 q0 Home을 승인값으로 단 한 번 실행했다. 복귀는
다음 terminal 결과로 PASS했다.

```text
ARM_EXECUTION_TERMINAL state=succeeded sequence=7244 status=6 detail=6 reason=motion completed within final error tolerance
```

최종 Home 피드백의 arm 최대 편차는 BASE `0.009203885 rad`였고, 모든 축이
허용 범위 안이었다. 그러나 `+0.10 rad` 5축 자세에서도 노란 marker는 Top
카메라 영상에 나타나지 않았다. 따라서 더 큰 robot motion으로 FOV를 해결하지
않고, Top 카메라 높이/위치 조정 후 homography와 visual base registration을
새 session으로 다시 수행한다.

## 안전 종료

초기 READ_ONLY 관측은 워크스테이션 TF node, Pi bridge 순서로 종료했다. 후속
Top 등록 자세 실기는 MoveIt, Pi bridge 순서로 정상 종료한 뒤 12 V servo
power를 OFF했다. 최종 종료 시 실제 팔은 무부하 Home 자세였고 추가 motion은
없었다.

## 2026-07-30 외부 계측 정밀화

이 문서의 `-57.5 deg` wrist-flex 값은 2026-07-26 사진 기반 1차 정합의
역사적 결과다. 이후 고정 rigid target을 사용한 Top eye-to-hand 데이터
sensitivity 분석에서 wrist-flex model offset `-0.129124366605 rad`
(`-7.398281239 deg`)가 식별되었다.

따라서 현재 canonical upstream wrist-flex Home은 `-64.898281239 deg`이며,
이는 URDF model origin만 정밀화한다. 물리 raw 2048, ROS feedback q0,
firmware calibration과 bridge zero는 변경하지 않는다. 이 보정 자체는
실물 동작을 승인하지 않으며, 보정 모델 회귀 시험과 독립 검증을 통과할 때까지
기하 기반 motion은 fail-closed로 유지한다.
