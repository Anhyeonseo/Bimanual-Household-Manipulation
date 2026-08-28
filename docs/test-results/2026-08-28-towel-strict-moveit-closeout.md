# 2026-08-28 수건 canonical strict MoveIt plan-only closeout

## 범위

등록 완료 오른팔 모델과 보정 증빙을 고정하고 다음 canonical sequence를 실제
동작 없이 검증했다.

1. 양팔로 로봇 가까운 아래쪽 edge를 잡아 먼 위쪽으로 첫 fold
2. `OBSERVE_CLEAR` 복귀와 bounded correction envelope 8개
3. 오른팔로 오른쪽 moving-edge midpoint를 잡아 왼쪽으로 두 번째 fold
4. 각 동작 뒤 `OBSERVE_CLEAR` 복귀

이 결과는 기하·5-DOF task pose·robot/table/camera collision과 dense TCP 경로의
plan-only 승인이다. 실제 jaw contact, cloth attachment·변형, slip·장력·마찰과
실제 모터 동작은 승인하지 않는다.

## 고정 입력

- registered URDF: `artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g.urdf`
  - SHA-256: `1acc73ed9a2a538c16446a9b8d781214db08aa31508973db99cddcd221f7af52`
- manifest: `artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g.manifest.json`
  - SHA-256: `2b49ad2e49a93f48551d8b15d210fb355a017e5d0f963d9f2a5d50c004d61bd6`
- right workcell shadow와 right tabletop staged validation
- validated Top worktable homography, operational limits, cable-reviewed joint envelope

## 결과

- status: `TOWEL_BIMANUAL_THEN_SINGLE_TASK_POSE_PLAN_ONLY_PASS`
- selected candidate:
  `first_bimanual_robot_near_to_far__second_right_right_to_left_edge_midpoint`
- first direction: 아래→위 (`robot_near_to_far`)
- second arm/direction: 오른팔, 오른쪽→왼쪽 (`right_to_left`)
- correction probes: 8/8
- planning segments: 840
- strict state samples: 12,547
- unapproved contacts: 0
- maximum accepted shallow mesh depth: `3.8104546 mm` / limit `4 mm`
- maximum dense TCP path deviation: `2.8681970 mm` / limit `4 mm`
- minimum joint-limit margin: `0.0486396 rad`
- `automatic_execution_permitted=false`
- `motion_authorized=false`
- `execution_api_used=false`
- `motion_commands=0`

결과 artifact는
`tmp/towel_fold_sequence_strict_r0g_20260828.json`이며 SHA-256은
`e70315f7a2138b45d21f014be8ee791f71fb1cee0e7c1d0fcac9db410d00cfb0`다.

## 재현

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOG_DIR="$PWD/tmp/ros_logs"

ros2 launch so101_bringup towel_fold_plan_only.launch.py \
  urdf_path:="$PWD/artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g.urdf" \
  use_rviz:=true
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export SO101_DUAL_URDF_PATH="$PWD/artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g.urdf"

python3 tools/run/plan_towel_fold_sequence_once.py --plan-only \
  --output tmp/towel_fold_sequence_strict_r0g_20260828.json
```

기존 artifact를 RViz에 게시할 때는 터미널 2의 planner 대신 다음을 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

python3 tools/run/visualize_towel_fold_sequence.py \
  tmp/towel_fold_sequence_strict_r0g_20260828.json \
  --stage both --include-departure
```

RViz의 `Towel Fold Reachability` MarkerArray와 `MotionPlanning` display에서 두
fold를 함께 볼 수 있다. 이 시각화도 trajectory를 실제 controller에 보내지 않는다.

## 남은 gate

- Isaac Lab S0/S1에서 동일 artifact의 deterministic replay
- 무수건 dry-run과 supervised-once
- 자동 jaw open/close-to-contact와 실제 single-/multi-layer grasp
- 실제 cloth deformation, slip·장력·마찰과 fold 결과 관측
- R4의 단계별 20회 중 19회 품질 기준
