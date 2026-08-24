# 2026-08-18 Top 카메라 재캘리브레이션 기록

## 목적과 최종 상태

Top 카메라 해상도를 `1280x960`으로 올리고 intrinsic을 다시 구한 뒤,
왼팔 TCP 근처의 2x2 ArUco GridBoard를 이용해 Top camera-to-left-base
eye-to-hand 등록을 다시 수행했다.

| 단계 | 결과 | 현재 사용 가능 여부 |
|---|---|---|
| 카메라 스트림 및 노트북 전달 | PASS | 캘리브레이션 영상 확인용으로 사용 가능 |
| 1280x960 intrinsic 후보 | 독립 검증 PASS | 후속 물리 metric 검증 전 runtime 승격 금지 |
| 왼팔 eye-to-hand 캡처 | training 10 + validation 2 완료 | 계산 입력으로만 사용 가능 |
| 왼팔 eye-to-hand 해 | **REJECTED** | 좌표 변환 및 로봇 동작에 사용 금지 |
| 작업대 plane/homography 재구축 | 미실시 | eye-to-hand 합격 뒤 수행 |
| YOLO OBB 좌표 재검증 | 미실시 | 전체 좌표계 재구축 뒤 수행 |

모든 산출물은 `motion_authorized: false`를 유지한다. 이번 eye-to-hand
candidate는 합격하지 않았으므로 수건 조작 motion 목표 좌표에 연결하면 안 된다.

## 카메라 스트림 정리

- 실제 Top 카메라 장치:
  `/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1.1:1.0-video-index0`
- 이전 설정의 `platform-xhci-hcd.1-...` 경로는 현재 Pi에 존재하지 않아
  camera manager가 `DISCONNECTED` 상태였다.
- 카메라 지원 형식은 MJPEG `1280x960 @ 30 fps`이다.
- 캘리브레이션 중 Pi camera manager는 디코드 부하와 네트워크 전송량을
  제한하기 위해 `/camera/top/image_raw`를 약 `2 Hz`로 발행했다.
- 노트북의 `tools/diagnostics/top_camera_qos_relay.py`가 BEST_EFFORT 원본을 받아
  `/camera/top/calibration_image`로 중계했다. 초기 DDS 발견 지연 뒤 약
  `1.9 Hz`가 확인됐다.
- ROS 2 공통 설정은 `ROS_DOMAIN_ID=30`, 기본 RMW를 사용했다.
  설치되지 않은 `rmw_cyclonedds_cpp`를 강제로 지정하지 않는다.

## Intrinsic 캘리브레이션

### 입력 조건

- 해상도/형식: `1280x960`, MJPEG
- 체스보드: 실제 사각형 `10x7`, 내부 코너 `9x6`
- 한 칸 크기: `25 mm`
- 수집: 82 views
- 이상치 제외: 4 views
- 최종 fit 입력: 78 views

제외된 이미지는 `left-0001.png`, `left-0003.png`, `left-0011.png`,
`left-0078.png`이다. baseline per-view RMS의 robust threshold와 독립 자세
재투영 결과를 근거로 제외했다.

### Fit 결과

| 항목 | 결과 |
|---|---:|
| OpenCV RMS | `0.7484 px` |
| per-view mean | `0.7335 px` |
| per-view median | `0.7471 px` |
| per-view p95 | `0.9451 px` |
| per-view max | `1.0711 px` |

Intrinsic 행렬과 왜곡계수는 다음과 같다.

```text
K = [1473.702962641, 0,              679.059982723,
     0,              1481.643790265, 437.111253387,
     0,              0,              1]

D = [-0.458353919, 0.268424567, -0.001790406,
     -0.005346930, 0.0]
```

### 독립 검증

학습에 사용하지 않은 13개 자세를 별도로 촬영했고, 과도하게 기울어진
`tilt_bottom.png` 한 장을 사전 정의된 품질 게이트로 제외해 12개를
평가했다.

| 항목 | 결과 |
|---|---:|
| retained mean RMS | `0.6869 px` |
| retained median RMS | `0.6974 px` |
| retained p95 RMS | `0.8417 px` |
| retained max RMS | `0.9516 px` |
| 판정 | **PASS** |

후보 파일은
`artifacts/calibration/top_1280x960_9x6_25mm_20260818/`
아래에 보존했다. 이 PASS는 카메라 내부 파라미터에 대한 판정이며,
카메라-to-robot 변환이나 작업대 좌표까지 합격했다는 의미는 아니다.

## 왼팔 Top eye-to-hand 캘리브레이션

### 장비와 계약

- 보드 모델: OpenCV `DICT_4X4_50`
- ID: `0, 1, 2, 3`
- 배열: `2x2`
- 프로그램 입력 marker side: `20 mm`
- 프로그램 입력 marker separation: `5 mm`
- active marker 외곽: `45x45 mm`
- 로봇 토크/동작 권한: OFF, `motion_authorized=false`
- 사용 토픽: `/camera/top/calibration_image`, `/joint_states`
- 허용 timestamp skew: `0.6 s`

Pi에서는 calibration build를 사용했다.

```text
/home/pi/Bimanual-Household-Manipulation/ros2_ws/install_calibration
```

resident bridge는 12축 joint state를 약 `20 Hz`로 발행했다. 수동 자세 변경
중 operational limit/unwrap gate가 여러 번 `status=8`로 정지했으며, 매번
팔을 안전범위로 복귀하고 STM32를 물리 RESET한 뒤 재연결했다. 이후 자세
변경은 `bridge 종료 -> 자세 변경 및 고정 -> STM32 RESET -> bridge 재기동`
순서로 분리했다.

### 수집 결과

- training: `left_train_01` .. `left_train_10`, 총 10자세
- held-out validation: `left_validation_01`, `left_validation_02`, 총 2자세
- 자세별 정상 프레임: 20장
- 각 자세는 stationary/read-only capture gate를 통과했다.
- 불완전 marker, PnP RMS `1.5 px` 초과, 움직인 window는 자동 제외됐다.

세션 파일:

```text
artifacts/calibration/top_eye_to_hand_20260818/session.yaml
```

### Solver 결과

최종 판정은 `REJECTED_EYE_TO_HAND_CALIBRATION`이다.

| 게이트 | 기준 | 결과 | 판정 |
|---|---:|---:|---|
| training translation span | `>= 40 mm` | `165.50 mm` | PASS |
| training rotation span | `>= 15 deg` | `49.67 deg` | PASS |
| training translation RMS | `<= 3 mm` | `5.190 mm` | **FAIL** |
| training translation max | `<= 5 mm` | `8.953 mm` | **FAIL** |
| training rotation RMS | `<= 1 deg` | `1.391 deg` | **FAIL** |
| training rotation max | `<= 2 deg` | `2.052 deg` | **FAIL** |
| validation translation max | `<= 5 mm` | `4.228 mm` | PASS |
| validation rotation max | `<= 2 deg` | `2.713 deg` | **FAIL** |
| PnP RMS max | `<= 1.5 px` | `1.499 px` | PASS, 한계에 근접 |
| image border min | `>= 10 px` | `95.78 px` | PASS |

잔차가 특히 컸던 training 자세는 다음과 같다.

| 캡처 | 위치 잔차 | 회전 잔차 |
|---|---:|---:|
| `left_train_03` | `8.953 mm` | `2.052 deg` |
| `left_train_06` | `7.548 mm` | `1.532 deg` |
| `left_train_10` | `6.507 mm` | `1.789 deg` |

특정 한두 캡처만 제거하면 해결되는지 확인하기 위해 최소 8자세 조건을
유지하면서 전체 8개/9개 조합을 오프라인으로 다시 계산했다. 모든 조합이
REJECTED였으며, 가장 낮은 training translation RMS도 `4.078 mm`였다.
따라서 임의 이상치 삭제나 acceptance threshold 완화로 채택하지 않는다.

최종 거부 파일:

```text
artifacts/calibration/top_eye_to_hand_20260818/candidate.yaml
```

## 해석

영상 경계와 자세 다양성은 충분했고 intrinsic 독립 검증도 통과했다. 반면
여러 로봇 자세에서 하나의 고정된 `T_gripper_target`과 `T_base_camera`를
동시에 만족하지 못했다. 현재 우선 조사 대상은 다음과 같다.

1. GridBoard가 그리퍼 안에서 자세 사이에 미끄러졌거나 보드가 휘었는지
2. 실제 검은 marker side와 가로/세로 separation이 각각 `20/5/5 mm`인지
3. 왼팔 physical q0, joint axis, link length가 현재 URDF FK와 일치하는지
4. 토크 OFF 수동 배치에서 servo/관절 backlash와 구조 처짐이 누적됐는지
5. ArUco PnP 값이 반복적으로 `1.49 px` 근처까지 올라간 원인

전체 종이 크기 `70x70 mm`는 직접적인 GridBoard 모델 입력이 아니다.
검은 사각형 한 변, 인접 검은 사각형 사이의 흰 간격, 보드 평탄성과
그리퍼 고정 강성을 별도로 실측해야 한다.

## 다음 재개 순서

1. 캘리퍼스로 marker side, 가로 separation, 세로 separation을 실측한다.
2. 보드를 그리퍼에 둔 상태에서 미끄러짐·회전 유격·종이 휨을 점검한다.
3. 실제 보드 치수가 모델과 다르면 저장된 원본 이미지로 PnP를 재계산한 뒤
   먼저 기존 세션을 재평가한다.
4. 보드 치수가 맞으면 왼팔 q0/축/link/backlash metrology를 별도 수행한다.
5. 원인을 수정한 뒤 training과 독립 validation을 다시 수집한다.
6. eye-to-hand가 PASS한 뒤 20 mm ArUco의 left/center/right 물리 metric
   검증을 수행한다.
7. Top-to-base, 작업대 plane/homography를 1280x960 기준으로 재구축한다.
8. 마지막으로 YOLO OBB 중심·크기·yaw의 작업대 좌표 오차를 재검증한다.

## 관련 산출물

- Intrinsic 후보:
  `artifacts/calibration/top_1280x960_9x6_25mm_20260818/top_camera_info_1280x960_candidate.yaml`
- Intrinsic fit 검증:
  `artifacts/calibration/top_1280x960_9x6_25mm_20260818/intrinsic_validation_candidate.json`
- Intrinsic 독립 검증:
  `artifacts/calibration/top_1280x960_9x6_25mm_20260818/intrinsic_independent_validation.json`
- Eye-to-hand 캡처/세션:
  `artifacts/calibration/top_eye_to_hand_20260818/`
- 현재 승격 조건: [검증 매트릭스](../VERIFICATION_MATRIX.md)

## 2026-08-25 R0-A optical FOV·clear-pose 후속 확인

카메라를 움직이거나 기존 calibration을 승격하지 않고 실제 스트림과 nominal
300×300 mm 수건의 광학 포함 가능성을 다시 확인했다.

- 장치: `/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1.1:1.0-video-index0`
- UVC serial: `20250807114148`
- 설치 높이: 작업대까지 수직거리 약 `445 mm` (근사 실측)
- MJPEG 지원: 640×480, 800×600, 1280×960, 1280×720, 1920×1080 모두 30 fps
- 1280×960 intrinsic 후보는 기존 독립 검증 PASS 상태를 그대로 유지

동일 장면의 SIFT+RANSAC 정합 212 inlier에서 1280×960 전체가 1920×1080의
대략 `x=240..1679`, `y=0..1079`에 대응했다. 따라서 640×480과 1280×960은
같은 화각이며 1920×1080은 수직 FOV를 늘리지 않고 좌우만 약 33% 추가한다.
정사각형 수건의 제한축을 해결하기 위해 16:9로 바꿀 근거는 없었다.

실제 수건 윤곽의 네 모서리와 nominal 300 mm 평면 가정을 이용해 사방 30 mm
envelope를 투영했다. envelope 크기는 1280×960 안에 들어갔으며 작업대상의
배치 중심을 제한하면 전체가 보인다. 물리 360 mm marker를 별도로 설치하지
않았고 수건 side tolerance도 미측정이므로 이 결과는
`PASS_WITH_PLACEMENT_REGION` optical candidate다. 기존 worktable homography
밖의 metric 정확도는 승인하지 않는다.

두 번째 양팔 퇴피 후보 영상에서는 수건 본체와 네 모서리에 robot/cable
occlusion이 없었다. resident bridge의 torque-off refresh 결과는 다음과 같다.

```text
firmware=0x00024809 state=ready motion_authorized=false
torque_enabled=false present_mask=0xFFF
left =[-0.929592, 0.217825, -0.513884, -0.207087, -0.064427, 0.254641]
right=[ 0.814544, 0.181010, -0.509282, -0.401903,  0.076699, 0.151864]
```

최종 visual candidate JPEG SHA-256은
`44687603bc429f2bf6ab496ea3df3f0fc4566bcd94f3a55bbea55102d5f0b8ca`다.
이 자세는 수동 배치 한 번만 확인했으므로 MoveIt plan-only, collision, 실제
명령 왕복과 재관측을 통과하기 전에는 named pose나 motion target으로 사용하지
않는다. 모든 결과는 `motion_authorized=false`다.
