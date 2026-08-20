# manipulation_camera_manager

세 대의 UVC 카메라에서 MJPEG를 계속 수집하면서, 현재 작업 단계에 필요한 영상만 JPEG로 디코딩하는 ROS 2 패키지다.

## 동작 원리

- 카메라마다 독립된 V4L2 mmap capture thread를 사용한다.
- 카메라마다 압축된 최신 frame 한 장만 보관한다. 오래된 frame queue는 만들지 않는다.
- USB가 분리되면 500 ms 간격으로 자동 재연결한다.
- `camera_phase`에 따라 선택된 카메라만 지정된 `decode_hz`로 RGB 변환한다.
- 디코딩된 영상은 `camera/<name>/image_raw`에 `sensor_msgs/Image`로 발행한다.
- phase가 바뀌면 지연시간 통계를 초기화하여 작업 단계별 p50/p95/max를 따로 측정한다.
- `inference_hz`는 다음 perception node가 사용할 계산 예산이다. 이 패키지는 아직 inference를 실행하지 않는다.

## 장치와 topic

| 이름 | 장치 경로 | 디코딩 영상 topic |
|---|---|---|
| `top` | USB 1.1 고정 경로 | `/camera/top/image_raw` |
| `wrist_a` | USB 1.2 고정 경로 | `/camera/wrist_a/image_raw` |
| `wrist_b` | USB 1.3 고정 경로 | `/camera/wrist_b/image_raw` |

작업 단계 명령은 `/camera_phase`의 `std_msgs/msg/String`으로 받는다. 지원 phase는 다음과 같다.

- `STANDBY`
- `SEARCH`
- `APPROACH_RIGHT`
- `VISUAL_ALIGN_RIGHT`
- `TRANSFER_RIGHT`
- `VERIFY_RIGHT`
- `DUAL_PRIVATE`
- `RUNTIME_BASELINE`
- `POLICY_ASSIST`

실행에 사용하는 규칙은 `config/cameras.yaml`에 있다. 프로젝트 최상위의 `config/camera_schedule.json`은 vision/policy 전체 설계를 위한 같은 값의 기준 문서다.

Top 카메라의 `640x480` 내부 보정값은
`config/top_camera_info.yaml`에 있다. 이 파일은 카메라, 렌즈, 초점 또는
해상도가 바뀌면 다시 생성해야 한다. 카메라 mount를 움직이면 내부 보정값은
유지할 수 있지만 별도의 작업대 homography는 다시 측정해야 한다.

작업대 평면의 체스판 기준 변환은
`config/top_worktable_homography.yaml`에 있다. 검출 픽셀은 먼저
`top_camera_info.yaml`의 `K`, `D`, `P`로 왜곡을 보정한 뒤
`rectified_pixel_to_board_m` homography에 입력해야 한다. 현재 파일의
board-relative 변환은 검증됐지만 `left_base_link` 등록값은 자로 측정한
임시값이므로 `motion_authorized: false`다. 실제 로봇 목표에는 3점 이상의
robot-assisted base registration을 통과한 뒤에만 사용한다.

2026-07-26의 첫 visual TCP 3점 등록은 승인값으로 채택하지 않았다. 체스보드
49점으로 다시 계산한 카메라 자세의 재투영 RMS는 0.437 px로 정상이고 URDF FK와
캡처 TF의 최대 차이도 1.728 mm였지만, 세 점이 base 단일축의 짧은 호에 몰려
geometry condition ratio가 0.003이었다. 또한 robot FK 최대 span 25.951 mm에
대해 높이 보정된 marker span은 8.217 mm뿐이어서 비율이 0.317이고, 강체 SE(2)
fit RMS/max 잔차는 7.301/9.416 mm였다. 이는 현재 노란 마커 중심을
`left_gripper_frame_link`와 동일점으로 볼 수 없다는 뜻이다.

결과는 `output/top_base_registration_candidate.yaml`에 fail-closed 상태
`REJECTED_REGISTRATION_GEOMETRY_OR_RIGID_FIT`로 남긴다. 다음 시도는
marker-to-TCP 기하를 측정하고 서로 다른 관절 구성의 기하학적으로 분리된 점을
5개 이상 수집한 뒤, 별도의 검증점까지 통과해야 한다. 그 전까지
`motion_authorized`와 `robot_target_available`은 모두 false다.

계산 뒤 물리 점검에서 노란 종이 마커가 떨어진 사실을 확인했다. 따라서 위 3점의
불일치는 카메라·URDF 결함으로 확정하지 않으며, 해당 세션 전체를
`INVALID_MARKER_DETACHED_DURING_CAPTURE`로 무효 처리했다. 기존 영상과 수치는 실패
근거로 보존하되 이후 등록 계산에는 재사용하지 않는다. 새로 고정한 마커는 새
session ID와 새 캡처 파일을 사용해 q0부터 다시 수집한다.

두 번째 세션에서도 BASE `+0.02 rad` 캡처 시점에 종이 마커가 다시 떨어진 사실을
사용자가 확인했다. 이 세션도 `INVALID_MARKER_DETACHED_AT_V1_CAPTURE`로 무효
처리하며 느슨한 종이 부착 방식은 더 사용하지 않는다. 다음 등록은 움직여도
미끄러짐·휘어짐·탈락이 없는 반복 가능한 rigid marker mount를 만든 뒤 별도 새
세션으로 시작한다.

세 번째 세션은 공식 SO101 32x32 UVC Wrist Camera Mount에 노란 marker를 강체
고정해 탈락 문제를 제거했다. 하지만 q0, BASE `+0.06 rad`, arm 5축
`+0.10 rad` 자세에서도 marker가 Top 영상 아래쪽 FOV 밖에 있었다. 마지막
자세는 elbow 최종 잔차 약 23 raw로 host의 20 raw 성공 기준을 넘겨 fail-safe
SAFE_STOP 되었고, 진단 로그 보강 뒤 Home 복귀는 terminal `status=6,
detail=6`으로 통과했다. 자세를 더 확대해 FOV를 해결하지 않는다.

Top 카메라는 높이와 각도를 바꿀 수 없는 고정 조건으로 확정했다. 카메라·렌즈·
초점·640x480 설정을 바꾸지 않았으므로 기존 intrinsic과 작업대 homography는
계속 유효하다.

BASE `+0.40 rad`의 검증된 자세에서 marker를 더 작은 강체 사각형으로 교체한 뒤
`bbox=[13,408,34,42]`, `area=1057 px²`, center=`[29.3642,428.1457]`로
`TOP_TCP_MARKER_PASS`를 통과했다. 이 캡처는 새 등록 세션의 첫 가시성 점으로
보존하되, 아래 CAD frame 계약이 적용되기 전 계산에는 넣지 않는다.

공식 STL의 기울어진 카메라 장착판에서 네 M2 홀을 직접 원 피팅한 결과 홀 간격은
27 x 27 mm이고 중심은 장착판의 4 mm 두께 중앙이다. 이 CAD 기준은 URDF의
`left_wrist_camera_mount_center_link`로 추가했다. 이 프레임은
`left_gripper_frame_link` TCP와 다른 고정 프레임이며 등록 solver는 새 세션에서
marker frame FK를 사용한다.

물리 marker가 장착판 표면 또는 카메라 모듈 표면에 붙어 있으면 CAD mid-plane에서
local-Z 방향의 고정 두께 offset을 별도로 기록해야 한다. 이 값과 5개 이상의
기하학적으로 분리된 점, 독립 검증점이 확정될 때까지 모든 motion 권한은 false다.

## Raspberry Pi 빌드

JPEG 개발 library가 없으면 먼저 설치한다.

```bash
sudo apt update
sudo apt install -y libjpeg-dev
```

그 다음 패키지를 빌드하고 테스트한다.

```bash
cd ~/Manipulation/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select manipulation_camera_manager \
  --cmake-clean-cache

colcon test --packages-select manipulation_camera_manager
colcon test-result --verbose
source install/setup.bash
```

## 실행과 phase 변경

터미널 1:

```bash
cd ~/Manipulation/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch manipulation_camera_manager camera_manager.launch.py
```

터미널 2에서 phase를 변경한다.

```bash
cd ~/Manipulation/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic pub --once /camera_phase std_msgs/msg/String \
  "{data: SEARCH}"
```

예를 들어 `VISUAL_ALIGN_RIGHT`는 top을 2 Hz, wrist_b를 12 Hz로 디코딩하고 wrist_a는 디코딩하지 않는다.

```bash
ros2 topic pub --once /camera_phase std_msgs/msg/String \
  "{data: VISUAL_ALIGN_RIGHT}"

timeout 15 ros2 topic hz /camera/top/image_raw
timeout 15 ros2 topic hz /camera/wrist_b/image_raw
```

## 진단값

```bash
ros2 topic echo --once /camera_diagnostics
```

카메라별 주요 값:

- `phase`: 현재 작업 단계
- `configured_decode_hz`: 현재 phase의 목표 디코딩 속도
- `configured_inference_hz`: 다음 perception node에 허용할 추론 속도
- `decoded_frames`, `decode_failures`: 현재 phase에서의 디코딩 결과
- `decode_frame_age_p50_ms`, `decode_frame_age_p95_ms`, `decode_frame_age_max_ms`: capture부터 디코딩 선택까지의 frame age
- `decode_time_p50_ms`, `decode_time_p95_ms`, `decode_time_max_ms`: JPEG 디코딩 시간
- `reconnect_count`, `driver_frames_dropped`: USB 복구와 driver frame 손실 횟수

기본 경고 기준은 frame age p95 200 ms, JPEG decode p95 50 ms다. 현재 phase에서 디코딩하지 않는 카메라의 지연 통계는 `-1`이 정상이다.
