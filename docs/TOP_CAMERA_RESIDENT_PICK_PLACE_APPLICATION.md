# Top 카메라 동적 Pick/Place 앱

## 동작 계약

이 앱은 영상에 검출된 물체 한 개의 **원본 이미지 중심 x 픽셀**로 사용할 팔을
선택한다. 양팔이 동시에 집는 동작은 만들지 않는다.

- `x < image_center - 20 px`: 왼팔
- `x > image_center + 20 px`: 오른팔
- 중앙 40 px: 모호한 구간으로 실행 거부
- target-lock 프레임 사이에 선택 팔이 바뀌면 실행 거부
- 선택 팔만 pick/pregrasp/lift/place/retreat/q0 경로를 수행
- 선택되지 않은 팔의 5축은 통합 복귀한 q0에 고정하고 gripper는 시작 위치를 유지
- 자동 재시도 없음; 성공 시 마지막 q0에서 torque hold
- transport/dispatch/heartbeat 이상은 coordinated STOP
- finite leg가 정상 완료되어 resident가 `ready`인 뒤 발생한 접촉/정밀도 판정 실패는
  현재 자세 torque hold를 보존하여 팔이 중력으로 넘어지지 않게 함

`TopObjectPose`는 `center_x_px`, `center_y_px`, `image_width_px`,
`image_height_px`를 직접 전달한다. 보드 좌표축에서 영상 좌우를 추정하지 않는다.

카메라 보정 좌표는 왼팔 base와 같은 `workcell_base_link` 기준이다. 계획 목표는 이
공통 좌표를 그대로 사용하며, 보수적 작업공간 검사만 선택 팔의 base 기준으로 한다.
오른팔을 선택하면 y=-232.064146 mm인 오른팔 base 원점만큼 평행이동해서 검사한다.

## 실행 구조

1. Pi resident adapter가 STM32 `0x00024809`과 12축 feedback을 소유한다.
2. PC Top perception이 물체의 board pose와 원본 픽셀 중심을 발행한다.
3. PC dual MoveIt이 `left_arm`과 `right_arm` 중 선택된 그룹만 plan-only한다.
4. 생성된 JSON과 SHA-256을 사람이 확인한다.
5. 실행기가 같은 owner로 양팔 5축을 q0로 복귀시키고 torque를 유지한다.
6. 토크 공백 없이 JSON을 resident 12축 명령으로 변환하며, 성공 시 q0 hold를 유지한다.

READY 상태에서는 서보 피드백 폴링이 정지하므로 `/feedback`의 `sample_age_ms`가
증가할 수 있다. 실행기는 계획을 만들기 직전에 `/refresh_anchor`를 호출하고 그
응답으로 새로 발행된 transient-local `anchor_joint_states`만 최초 자세로 사용한다.
resident node는 각 finite leg 완료 시에도 측정된 최종 자세로 이 anchor를
갱신한다. 실제 동작 중에는 fresh `/feedback`을 사용한다. 종료 판정은 firmware의
12회 연속 measured joint-pair 정착 검사와 resident의 완전한 12축 snapshot
freshness 검사를 통과한 뒤 `ACTIVE -> READY` 전환에서 새로 발행된 terminal
anchor를 사용한다.

PC MoveIt은 `allow_trajectory_execution=false`로 고정된다. 하드웨어 실행 경로는
`/bimanual_stream_adapter/command` 하나뿐이다.

이 one-shot 앱은 상단 애플리케이션 계약의 reference consumer다. 카메라/MoveIt
의미를 firmware에 추가하지 않으며, 완전한 finite route를 ROS service로 제출하면
resident가 내부에서 9점/400 ms wire window로 공급한다.

## Pi: resident adapter

```bash
cd /home/pi/SO101-Bimanual-Manipulation
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch single_arm_bridge bimanual_stream.launch.py motion_authorized:=true
```

준비 로그는 `firmware=0x00024809 motion_authorized=true`여야 한다. 이 노드는 기존
`/bimanual_stream_adapter/joint_states`와 MoveIt용 `/joint_states`를 함께 발행한다.

새 실행 전 status는 반드시 `ready`, `owner=null`, `arbiter_epoch=0`이어야 한다.
성공 뒤에는 `ready`, owner 유지, epoch 7인 torque-hold 상태이므로 영상 촬영이나
작업 확인을 마친 뒤 같은 owner로 STOP한다. STOP 후 `stopped` process를 다시
사용하지 않고 resident 종료, STM32 RESET, resident 재시작으로 새 session을 연다.
startup shadow status 2/3은 좌/우 verified torque-disable 실패다. 같은 요청을
자동 반복하지 말고 작업자가 전원·버스와 중복 process 부재를 확인한다.

## PC: camera manager, perception과 dual MoveIt

터미널 1 (카메라 캡처):

```bash
cd ~/Documents/GitHub/SO101-Bimanual-Manipulation
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch manipulation_camera_manager camera_manager.launch.py
```

별도 터미널에서 Top 카메라를 SEARCH phase로 전환한다.

```bash
ros2 topic pub --once /camera_phase std_msgs/msg/String "{data: SEARCH}"
```

터미널 2 (YOLO-OBB):

```bash
cd ~/Documents/GitHub/SO101-Bimanual-Manipulation
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch so101_top_perception top_obb_runtime_smoke.launch.py \
  python_executable:=/home/an-hyeonseo/Documents/GitHub/SO101-Bimanual-Manipulation/.venv-top-pen-obb/bin/python \
  bundle_manifest:=/home/an-hyeonseo/Documents/GitHub/SO101-Bimanual-Manipulation/artifacts/stage8/top_pen_yolo_obb_candidate_v3_finetune_2026-08-02/top_pen_yolo_obb_bundle.json \
  inference_hz:=4.0
```

터미널 3 (dual MoveIt):

```bash
cd ~/Documents/GitHub/SO101-Bimanual-Manipulation
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch so101_bringup external_bimanual_moveit.launch.py use_rviz:=false
```

## PC: 동적 plan-only

```bash
cd ~/Documents/GitHub/SO101-Bimanual-Manipulation
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

python3 tools/plan_top_camera_pick_place_once.py   --plan-only   --routing-deadband-px 40   --output artifacts/top_pick_place/2026-08-14/dynamic_plan_run01.json
```

성공 출력에는 `selected_arm=left` 또는 `selected_arm=right`, 픽셀 x/영상 폭,
계획 파일 SHA-256이 포함된다. 각 pick endpoint와 기존 place workcell 좌표를
선택 팔의 IK로 새로 풀며, 반대 팔 q0를 포함한 self-collision 검사를 통과해야 한다.
기존 place의 **관절각**은 재사용하지 않는다.

현재 펜 reference task는 wrist roll을 양팔 q0인 `0.0 rad`로 고정하고
나머지 4축으로 TCP xyz를 맞춘다. 물체 yaw는 진단값이며 펜 실행 조건으로
강제하지 않는다. 캔 task의 orientation-aware wrist roll은 다음 PR에서
별도 승격한다.

그리퍼는 `raw 2048`까지 연 뒤 접근하고 grasp에서 `raw 1948`까지 닫는다.
접촉 판정은 잔차 14 raw 이상이다. F8.9는 arm route tracking 90,000 µrad와
terminal 46,020 µrad를 유지하고, 그리퍼만 route/terminal 150,000 µrad,
firmware hard cap 160,000 µrad를 적용한다.

동적 grasp 목표는 object z 기준 `-0.001 m` offset을 사용한다. 계획에는
기준/선택 offset, 화면축 보정, homography SHA와 corrected target을 기록하고
실행기가 schema 12 계약과 plan SHA를 검증한다.

실행기는 50 ms 시각열을 만들고 resident가 긴 finite horizon을 내부
9점/400 ms wire batch로 공급한다. q0 복귀와 arm route는 연속 finite leg로
묶고, gripper open/close/release만 분리한다. firmware가 12회 연속 measured
pair를 확인하고 resident가 terminal snapshot freshness와 오차를 검증한 뒤에만
READY/HOLD로 전이한다.

## 무동작 resident gate

plan-only 출력의 SHA-256을 그대로 넣는다.

```bash
python3 tools/run_top_pick_place_application_once.py   --validate-only   --plan artifacts/top_pick_place/2026-08-14/dynamic_plan_run01.json   --plan-sha256 <PLAN_SHA256>   --output artifacts/top_pick_place/2026-08-14/validate_run01.json
```

`TOP_PICK_PLACE_DYNAMIC_VALIDATE_ONLY_PASS motion_commands=0 resident_services_called=0`가
나와야 한다. validate-only는 resident status/anchor/command service를 만들거나
호출하지 않으며 torque 상태를 바꾸지 않는다.

## 실기 승격 조건

- 왼팔 선택: 새 dynamic plan의 실제 target/경로를 먼저 검토한다.
- 오른팔 선택의 첫 실기는 기존 place workcell 좌표의 높이와 접근 자세를 확인하는
  감독 commissioning이다. 아직 검증됐다고 선포하지 않고
  `RUN_RIGHT_PLACE_HEIGHT_CHECK_ONCE` 토큰으로 1회를 승인한다. 성공 artifact와 작업자
  육안 확인이 함께 있어야 이후 `right place validated` 증거로 승격한다.
- plan 생성 후 300초가 지나면 실행기는 stale plan으로 거부한다.
- 실행 명령은 plan-only 결과 검토 후 별도로 제공한다.

## F8.9 end-to-end 실기 evidence

2026-08-16 session03에서 schema 12의 fresh plan을 팔마다 새로 만들었다.

- 왼팔 화면 보정: 오른쪽 13.72 mm
- 오른팔 화면 보정: 왼쪽 29.47 mm
- 왼팔 execute: PASS/HOLD, 최대 arm error 28.176 mrad
- 오른팔 execute: PASS/HOLD, 최대 arm error 13.806 mrad
- 전체 결과: `LEFT_RIGHT_PEN_TRANSFER_ONCE_PASS`
- automatic retry: 0

전체 journal SHA-256은
`408c21d6e7211834351123c5058cf7a8be50b8d20d064ec3f861230099198fbc`다.
세부 firmware·resident·stage 증거는
[F8.9 resident와 양팔 펜 전달 수락 결과](test-results/2026-08-16-f89-bimanual-pen-transfer.md)에
기록했다.
