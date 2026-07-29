# 단계 4 — 왼팔 Isaac Sim·MoveIt 통합 체크리스트

## 완료 판정

- 범위: 정상인 SO-ARM101 왼팔 1대의 simulation vertical slice
- 판정: **simulation vertical slice 통과 / physical q0 visual baseline 반영**
- 단계 4 진행률: **92%**
- 검증일: 2026-07-24, physical q0 visual registration 2026-07-26
- 실제 hardware 활성화: **아니요**

이 판정은 `mock`과 Isaac Sim에서 대표 arm/gripper trajectory가 계획되고
실행된 단일 왼팔 기준이다. 고장 난 반대편 팔, 실제 STM32 trajectory 실행,
simulated camera mount, 양팔 planning group은 후속 단계로 남긴다.

2026-07-26 servo one-key centering으로 정한 physical raw `2048` Home을
top/side/front 사진과 Isaac interactive FK로 시각 정합했다. 기존 모델에서
그 자세를 만들던 arm 5축 값 `0, +90, -55, -64.898281239, -90 deg`를 URDF
joint origin에 흡수해 raw 2048, ROS/MoveIt q0와 SRDF Home이 같은 자세를
뜻하도록 수정했다. 2026-07-30 보정값은 Isaac USD에도 동기화했으며, 이 값은
실물 명령이 아니라 모델 원점 보정이다.
Wrist flex는 2026-07-30 eye-to-hand 외부 계측으로 사진 기반 `-57.5 deg`
추정값을 `-7.398281239 deg` 추가 보정했다. raw 2048과 firmware/bridge zero는
변경하지 않았다.
자동 FK 동치와 shifted limit 검증을 통과했다. 2026-07-30 보정값도 Isaac
`base.usda`/`Physics/physics.usda`에 동기화하고 URDF↔USD anchor/limit parity를
통과했다. 다만 사진 기반
1차 정합이므로 정밀 실물 FK, 충돌 형상, TCP와 camera–base 등록을 다시 검증하기
전 실제 기하 기반 motion은 fail-closed로 유지한다.

## 고정한 실행 환경

| 항목 | 값 |
|---|---|
| Workstation OS | Ubuntu 24.04.4 LTS (`amd64`) |
| Isaac Sim | 6.0.1 |
| Isaac Sim executable | `~/isaacsim-6.0.1-venv/bin/isaacsim` |
| ROS 2 | Jazzy |
| MoveIt | 2.12.4 |
| RMW | `rmw_cyclonedds_cpp` |
| `ROS_DOMAIN_ID` | `30` |
| discovery range | `SUBNET` |
| 대상 | 왼팔 arm 5-DOF + gripper 1-DOF |

Isaac Sim은 desktop icon이 아니라 ROS 2 환경을 source한 terminal에서
실행한다. 그렇지 않으면 ROS 2 Bridge가
`libament_index_cpp.so: cannot open shared object file`로 시작하지 못한다.

## 좌표와 joint contract

- `+X`: 로봇 전방
- `+Y`: 로봇 기준 왼쪽
- `+Z`: 위
- intended physical home: 모든 project joint의 `q=0`
- physical raw-2048 home parity: **PROVISIONAL PASS — visual baseline**
- SRDF virtual joint: `world` → `workcell_base_link`
- URDF mount: `workcell_base_link` → `left_arm_mount_joint` →
  `left_base_link`

| 순서 | project joint | project `+q` 의미 |
|---:|---|---|
| 1 | `left_base_joint` | 끝단이 `+Y` 방향 |
| 2 | `left_shoulder_joint` | 끝단이 `+Z` 방향 |
| 3 | `left_elbow_joint` | flex, 끝단이 `+Z` 방향 |
| 4 | `left_wrist_flex_joint` | 끝단이 `+Z` 방향 |
| 5 | `left_wrist_roll_joint` | `+X` 축 right-hand rotation |
| 6 | `left_gripper_joint` | gripper 열림 |

### MoveIt contract

- group `left_arm`: `left_base_link`에서
  `left_gripper_frame_link`까지의 5-DOF chain
- group `left_gripper`: `left_gripper_joint`
- end effector `left_end_effector`
- KDL position-only IK
- named state: `left_arm/home` = arm joint 5개 모두 `0`
- named state: `left_gripper/closed` = `0`
- named state: `left_gripper/open` = `1.91986`
- default velocity/acceleration scaling: `0.5` / `0.5`

### Controller contract

| 기능 | ROS interface | type |
|---|---|---|
| arm trajectory | `/left_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` |
| gripper command | `/left_gripper_controller/gripper_cmd` | `ParallelGripperCommand` |
| project state | `/joint_states` | `sensor_msgs/msg/JointState` |

## Isaac Sim asset와 OmniGraph

- stage:
  `isaac_sim/assets/so101_new_calib/so101_new_calib.usda`
- Articulation root:
  `/so101_new_calib/Geometry`
- OmniGraph:
  `/Graph/ROS_JointStates`
- Isaac state:
  `/isaac/joint_states`
- Isaac command:
  `/isaac/joint_command`
- drive stiffness: `1000`
- drive damping: `100`
- drive max force: `10`
- arm drive target: `0 deg`
- gripper drive target: `-10 deg`

저장된 stage는 `payloads/`와 geometry를 포함하는 self-contained asset이다.
Isaac Sim 6.0.1에서 이 파일을 열고 `Play`해야 OmniGraph가 ROS topic을
발행하고 command를 articulation에 적용한다.

## Isaac과 project joint mapping

| project joint | Isaac joint | position mapping |
|---|---|---|
| `left_base_joint` | `shoulder_pan` | `q_project = -q_isaac` |
| `left_shoulder_joint` | `shoulder_lift` | `q_project = -q_isaac` |
| `left_elbow_joint` | `elbow_flex` | `q_project = -q_isaac` |
| `left_wrist_flex_joint` | `wrist_flex` | `q_project = -q_isaac` |
| `left_wrist_roll_joint` | `wrist_roll` | `q_project = -q_isaac` |
| `left_gripper_joint` | `gripper` | `q_project = q_isaac + 10 deg` |

따라서 project gripper `q=0`은 Isaac `-10 deg`, project open
`1.91986 rad`은 Isaac 약 `100 deg`다. 이 변환은
`so101_isaac_bridge`에만 존재하며 MoveIt task logic은 Isaac joint 이름이나
USD path를 알지 않는다.

## 재현 실행 순서

### 1. Isaac Sim 시작

실행 위치는 **Ubuntu workstation의 새 terminal A**다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
~/isaacsim-6.0.1-venv/bin/isaacsim \
  --enable isaacsim.ros2.bridge
```

GUI가 열리면 아래 stage를 연다.

```text
~/Documents/GitHub/SO101-Bimanual-Manipulation/
isaac_sim/assets/so101_new_calib/so101_new_calib.usda
```

그다음 `Play`를 누른다. 성공 시 로봇이 안정적으로 유지되고
`/isaac/joint_states`가 발행된다.

### 2. MoveIt과 Isaac backend 시작

실행 위치는 **같은 Ubuntu workstation의 새 terminal B**다.

```bash
cd ~/Documents/GitHub/SO101-Bimanual-Manipulation/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 launch so101_bringup isaac_moveit.launch.py
```

성공 시 RViz에 왼팔이 표시되고 arm/gripper planning group을 선택할 수
있으며 bridge에 다음 문장이 출력된다.

```text
Isaac backend ready: /isaac/joint_states -> /joint_states,
MoveIt actions -> /isaac/joint_command
```

이 launch는 simulation adapter를 사용한다. `single_arm_bridge`를 동시에
실행하면 안 되며 serial device나 STM32에 접근하지 않는다.

### 3. 종료

1. RViz에서 arm을 `home`, gripper를 `closed`로 실행한다.
2. terminal B에서 `Ctrl+C`로 MoveIt/adapter/RViz를 종료한다.
3. Isaac Sim에서 `Stop`을 누른다.
4. GUI는 필요하면 그대로 열어 두거나 정상 종료한다.

이 순서는 simulation을 재현 가능한 q0 상태로 되돌린 뒤 command source를
제거한다.

## 확인된 PASS

- Isaac Sim 6.0.1 GUI, empty stage, Play/Stop 안정성
- Ubuntu workstation ↔ Raspberry Pi 양방향 ROS 2 discovery
- 왼팔 URDF/Xacro visual, TF, q0, joint axis와 limit — simulation 내부 PASS
- physical raw-2048 Home → model q0 visual registration 및 자동 FK 동치 PASS
- wrist-roll `-90 deg` 수정 후 Isaac q0 ↔ URDF/RViz all-zero 정합 PASS
- Isaac Script Editor simulation-only gripper open/closed 물리 방향 사용자 확인 PASS
- 2026-07-30 Isaac↔URDF q0 TCP FK parity — 위치 `3.46e-8 m`, 회전 `2.15e-7 rad` 이하
- MoveIt Setup Assistant config, collision matrix, position-only IK
- mock controller에서 arm Plan/Execute
- mock controller에서 gripper Plan/Execute
- wrist-roll `-90 deg` q0 수정 후 mock gripper open/closed 물리 방향 사용자 확인 PASS
- Isaac ROS 2 Bridge startup
- Isaac Joint States OmniGraph state/command round trip
- Isaac articulation q0 안정 유지
- project↔Isaac mapping unit test 3개
- direct arm q0 action `SUCCEEDED`
- direct gripper q0 action `SUCCEEDED`
- MoveIt random valid arm Plan/Execute `SUCCEEDED`
- MoveIt gripper open Plan/Execute `SUCCEEDED`
- MoveIt gripper closed Plan/Execute `SUCCEEDED`
- MoveIt arm home Plan/Execute `SUCCEEDED`
- home 후 project state의 모든 joint가 `0.03 rad` tolerance 이내

상세 증거는
[2026-07-24 시험 결과](../test-results/2026-07-24-isaac-moveit-left-arm-integration.md)에
정리한다. physical q0 재정렬과 READ_ONLY TF 증거는
[2026-07-26 q0 재정렬 시험](../test-results/2026-07-26-left-arm-q0-realignment.md)에
정리한다.

## 알려진 비차단 항목

- 3D sensor plugin이 없으므로 MoveIt octomap 관련 경고가 보일 수 있다.
- `/recognize_objects` action server가 없다는 경고는 perception 미구현 상태와
  일치한다.
- 이 Jazzy 환경에서 `move_group` 종료가 늦어 5초 뒤 `SIGTERM`이 사용될 수
  있다. 실행 중 planning/execution 실패와는 구분한다.
- RViz plugin namespace 경고는 확인됐지만 planning/execution을 막지 않았다.
- Isaac Sim의 `Play` 중에는 중력 때문에 shoulder/elbow에 약
  `0.006`/`0.010 rad`의 정상상태 오차가 관찰됐다. `0.03 rad` 허용치
  이내이며 속도는 0으로 안정됐다.

## 2026-07-26 q0 정합 결과와 잔여 차단

- host calibration의 arm 5축 `zero_raw=2048`과 SRDF Home all-zero는 유지했다.
- model origin을 보정했으므로 Isaac bridge arm offset도 계속 `0`이다.
- 초기 wrist roll `+90 deg` 후보는 mock gripper open에서 반대 방향으로 판정해
  폐기했고, physical 기준 반대인 `-90 deg`를 model origin에 흡수했다.
- 수정 후 MoveIt mock에서 gripper open/closed 방향을 다시 확인해 PASS했다.
- 같은 수정이 반영된 Isaac Sim 6.0.1 asset에서도 simulation-only Robot Poser로
  gripper open/closed 방향과 closed 복구를 확인해 PASS했다.
- gripper raw 2048의 실제 개방 폭과 project gripper rad 변환은 아직 미확정이다.
- 사진 기반 1차 시각 정합이므로 계측 기반 joint-by-joint FK, TCP와 collision
  parity는 아직 미통과다.
- q0 변경 전 계산한 Top–base visual registration은 재사용하지 않는다.
- 위 정밀 검증과 재등록 전 task motion은 계속 금지한다.

## 후속 범위

- 실제 STM32 `ros2_control` hardware plugin과의 연결은 단계 5에서 한다.
- 실제 servo 동작 전에는 firmware verified safe limit과 URDF/MoveIt limit을
  다시 교차 검증한다.
- simulated top/wrist camera mount와 양팔 skeleton은 반대편 팔 복구 및
  측정 후 별도 gate로 검증한다.
- backend 선택은 한 번에 하나만 허용해야 하며 `mock`, `isaac`, `stm32`가
  같은 project joint contract를 유지해야 한다.
