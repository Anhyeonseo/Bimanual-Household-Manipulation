# Raspberry Pi 카메라·연산 아키텍처

상태: `기준선 채택`. 카메라 수집·선택 decode는 실측 통과했고, 실제 검출기와 배포 policy를 함께 실행한 Pi 5 수치는 후속 gate에서 확정한다.

## 1. 설계 목표

Raspberry Pi 5 4GB에서 다음 작업을 동시에 수행하되 제어와 안전 처리가 영상 연산 부하에 밀리지 않게 한다.

- USB RGB 카메라 3대 연결
- 필요한 카메라 영상만 압축 해제(decode)하고 추론(inference) 수행
- MoveIt은 전역 충돌 회피 경로를 계산하고 작업 상태가 바뀔 때만 무거운 계획 수행
- policy와 Visual Servo는 검증된 전역 경로 주변의 제한된 보정만 담당
- Isaac Sim/Isaac Lab 학습은 데스크탑에서 수행
- 검증된 ONNX policy의 실제 inference는 Pi 5에서 수행
- STM32 heartbeat와 ROS 안전 상태는 영상 처리와 독립
- 원본 이미지는 영상 처리 process 밖으로 계속 publish하지 않음

핵심 원칙은 "카메라 3대를 연결했다고 해서 3대 영상을 항상 모두 추론하지 않는다"는 것이다.

## 2. 카메라별 역할

### Top 카메라 — 전체 상태 관측

주요 역할:

- 작업대와 전체 작업 영역 관측
- 검은색 마커펜 탐색과 후보 선택
- 펜의 평면 `x, y, yaw` 추정
- 펜꽂이 위치와 입구 영역 추정
- 사용할 팔(active arm)을 고르는 데 필요한 전역 좌표 제공
- grasp 후 물체가 원래 위치에서 사라졌는지 확인
- place 후 펜이 목표 영역에 들어갔는지 확인
- 양팔의 개별·공유 작업 영역을 전체적으로 관측

적용 기술:

- 카메라 내부 보정(intrinsic calibration)
- 카메라와 base 사이의 고정된 외부 보정값(extrinsic)
- 작업대 homography를 이용한 pixel→작업대 좌표 변환
- 색/명암·길쭉한 형상·윤곽선/PCA를 결합한 후보 생성기로 단순 threshold보다 배경 변화에 강하게 구성
- 대리석 무늬·반사·조명 변화가 큰 시연 환경은 Nano급 YOLO 또는 작은 특징점 검출기로 보강
- 촬영 시각, frame이 지난 시간과 검출 신뢰도를 이용해 데이터 최신성 검사

상단 RGB 카메라 한 대만으로 일반적인 3차원 자세를 복원할 수 있다고 가정하지 않는다. 초기 Z 좌표는 작업대와 물체 모델에서 이미 알고 있는 높이로 제한한다.

### Left wrist 카메라 — 왼팔 생산 기준선의 근접 정렬

주요 역할:

- pre-grasp 이후 그리퍼와 펜의 상대 오차 계산
- 마지막 수 cm 구간에서 중심과 yaw 오차 보정
- gripper를 닫기 직전에 물체가 있는지 확인
- 들어 올린 뒤 물체가 gripper와 함께 움직이는지 확인
- 펜꽂이 접근 시 입구 중심과 삽입 방향 미세 보정

적용 기술:

- 카메라 내부 보정(intrinsic calibration)
- eye-in-hand calibration으로 `left_gripper_frame_link → left_wrist_camera_optical_frame` 고정 transform 추정
- 필요한 영역만 자른 영상(ROI)과 작은 특징 검출기 사용
- 영상 기반 Visual Servo 또는 크기가 제한된 Cartesian 좌표 보정
- frame이 오래됐거나 신뢰도가 낮거나 timeout이 발생하면 즉시 중단

카메라는 세계 좌표에서 움직이지만 로봇팔 말단 장치(end-effector)와 카메라 사이의 보정값은 고정이다. TF가 매 순간 카메라 자세를 계산한다.

### Right wrist 카메라 — 오른팔 단독 동등성

왼팔 기준선과 같은 역할을 한다. 오른팔의 정식 추론 활성화는 오른팔 단독 calibration·MoveIt·안전·Pick/Place gate 뒤에 수행한다.

- 현재: 장치 연결과 데이터 최신성만 낮은 주기로 확인
- 오른팔 단독 동등성 통과 후: 오른팔 영상 정렬
- 양팔 개별 작업: 현재 작업 단계에 따라 좌우 손목 영상을 교대로 추론
- 수건 작업: 양쪽 grasp 지점과 수건 가장자리 확인

## 3. 카메라 보정(Calibration) 방법

체크보드 또는 ChArUco는 개발 중 카메라 보정에만 사용하고 시연 작업대에서는 제거한다.

| 카메라 | 내부 보정 | 외부 보정 |
|---|---|---|
| 상단 | 카메라별 보정 | eye-to-hand, base 기준 고정 |
| 왼쪽 손목 | 카메라별 보정 | eye-in-hand, 왼팔 tool 기준 고정 |
| 오른쪽 손목 | 카메라별 보정 | eye-in-hand, 오른팔 tool 기준 고정 |

보정 결과는 `camera_info` YAML과 transform YAML로 버전 관리한다. 해상도, 초점 또는 카메라 mount가 바뀌면 기존 보정값을 재사용하지 않는다.

## 4. 영상 처리 process 구조

```text
Top V4L2 Capture Thread ─────────┐
Left Wrist V4L2 Capture Thread ─┼→ compressed latest-frame slots
Right Wrist V4L2 Capture Thread ┘            ↓
                                      Phase Scheduler
                                             ↓
                              decode/resize only selected frame
                                             ↓
                               single vision inference lane
                                             ↓
                         detection/pose/confidence/timestamp only
                                             ↓
                                      ROS structured output
```

권장 구현:

- C++17
- V4L2 mmap을 우선 사용하고, 측정 결과에 따라 GStreamer로 대체
- UVC MJPEG 우선, 실측 공통 mode인 `640×480 @ 30FPS`로 수집
- 카메라별로 압축된 최신 frame 한 장만 유지
- 현재 사용하는 카메라 영상만 libjpeg-turbo/OpenCV로 decode
- inference input `320×320`부터 시작
- queue 크기는 1로 두고 오래된 frame 폐기
- debug image는 요청이 있을 때만 보내거나 1FPS 이하로 제한
- 원본 영상을 process 사이에서 DDS로 보내거나 rosbag으로 계속 기록하지 않음

카메라가 MJPEG를 지원하지 않거나 decode 비용이 더 큰 경우 `v4l2-ctl` 결과와 성능 측정값을 보고 YUYV를 선택한다.

## 5. 추론 구조

### 영상 추론(inference)

- 동시에 실행하는 고부하 추론 작업: 1개
- ONNX Runtime 실행 방식: 순차 실행(sequential)부터 시작
- `intra_op_num_threads`: 2부터 측정
- `inter_op_num_threads`: 1
- 상단/손목 영상에 같은 검출기를 사용할 수 있으면 ONNX session 1개 공유
- 서로 다른 모델이 필요해도 동시에 실행하지 않고 scheduler가 차례로 실행
- INT8은 FP32 기준 모델의 정확도와 지연 시간을 측정한 뒤 적용

### Policy 추론

기본 권장은 detector/kinematics가 만든 구조화 상태를 입력으로 쓰는 것이다.
다만 데스크탑에서 이미 학습·검증한 policy가 Top·왼쪽 손목·오른쪽 손목 RGB를
직접 관측했다면, 배포 시에도 camera order, resize/crop, color order,
normalization, history와 tensor shape를 포함한 **동일 observation 계약**을
보존한다. 어떤 경우에도 ROS의 무제한 raw image stream을 그대로 쌓지 않고,
카메라 manager의 최신 frame과 고정 전처리 결과만 policy에 전달한다.

입력 예:

```text
object x/y/yaw
target x/y
left/right end-effector pose
joint positions/velocities
gripper state
task phase
detection confidence
active arm
```

출력은 크기가 제한된 Cartesian 좌표 변화량, 사용할 팔 선택, grasp 판단 또는 보정값으로 제한한다.

- 별도 `policy_runtime` process
- 데스크탑에서 학습·평가하고 manifest/hash가 고정된 ONNX deployment bundle
- 구조화 상태 또는 학습 시점과 동일한 고정 크기 image tensor 계약
- 초기 rate 10Hz
- `intra_op_num_threads = 1`
- 입력 queue depth 1
- observation 또는 카메라 frame이 오래됐으면 결과 폐기
- 출력 크기·속도·workspace·collision·freshness gate를 통과하지 못하면 Hold
- 서보 raw 위치를 직접 출력하지 않음
- 저장 데이터 평가 → Pi shadow mode → 제한 residual 순서로만 실제 권한 확대
- Pi에서 Isaac Sim/Isaac Lab 학습이나 시뮬레이터를 실행하지 않음

Policy와 영상 추론을 동시에 실행할 수는 있지만, 영상 처리 thread 2개와 policy thread 1개를 초기 상한으로 둔다. 이렇게 해서 제어와 운영체제에 최소 한 코어 정도의 여유를 남긴다. 실제 CPU core 고정(affinity)은 성능 측정 전에 적용하지 않는다.

## 6. 작업 상태별 연산 일정

아래 수치는 초기 연산 자원 한도이며 `config/camera_schedule.json`이 전체 영상 추론 속도를 12Hz 이하로 검증한다.

현재 구현의 `APPROACH_RIGHT`, `VISUAL_ALIGN_RIGHT` 같은 이름은 초기
오른팔 중심 설정의 역사적 label이다. 동작 코드를 바꾸지 않은 채 문서에서
이름만 바꾸지 않으며, 후속 변경에서 `active_arm` 기반의 좌우 공통 phase로
일반화하고 왼팔 생산 기준선을 먼저 검증한다.

| 작업 상태 | 상단 추론 | 왼쪽 손목 | 오른쪽 손목 | Policy | 목적 |
|---|---:|---:|---:|---:|---|
| STANDBY | 1Hz | 0 | 0 | OFF | 장치·작업대 저주기 감시 |
| SEARCH | 8Hz | 0 | 0 | OFF | 펜 탐색 |
| APPROACH_RIGHT | 4Hz | 0 | 6Hz | OFF | 전역+근접 전환 |
| VISUAL_ALIGN_RIGHT | 1Hz | 0 | 10Hz | OFF | 오른팔 마지막 정렬 |
| TRANSFER_RIGHT | 2Hz | 0 | 1Hz | OFF | 운반 상태 확인 |
| VERIFY_RIGHT | 4Hz | 0 | 4Hz | OFF | grasp/place 결과 확인 |
| DUAL_PRIVATE | 4Hz | 4Hz | 4Hz | OFF | 양팔 영상을 번갈아 처리 |
| RUNTIME_BASELINE | 4Hz | 4Hz | 4Hz | 10Hz | 3카메라+policy 무동작 자원 계측 전용 |
| POLICY_ASSIST | 4Hz | 0 | 6Hz | 10Hz | 구조화 상태 보정값 평가 |

Policy 학습과 검증은 데스크탑에서 끝낸다. Pi에서는 deployment bundle을
먼저 shadow mode로 실행한다. Visual Servo와 policy를 동시에 실제 명령원으로
사용하지 않고 arbitration이 선택한 한 경로만 bounded command를 낸다.

## 7. 실행 우선순위

부하가 높을 때 기능을 줄이는 순서:

1. debug image 전송과 영상 기록 중단
2. 사용하지 않는 손목 영상의 decode와 추론 중단
3. 상단 또는 현재 사용하는 손목 영상의 추론 속도 감소
4. policy를 10Hz에서 5Hz로 낮추거나 일시 중단
5. 검출기 입력 크기 또는 모델 축소

절대로 자동으로 줄이지 않는 항목:

- STM32 heartbeat
- serial RX/TX와 관절 상태값
- 안전 및 fault 처리
- 명령 유효성 검사
- 카메라와 인식 결과의 최신성 검사

Policy 또는 인식 결과가 정해진 처리 시간을 넘기면 오래된 결과로 계속 움직이지 않고 Hold하거나 다시 탐색한다.

## 8. Pi 자원 사용 목표

초기 합격 기준:

| 지표 | 목표 |
|---|---|
| 전체 CPU 지속 평균 | 70% 이하 목표, 최대 75% 상한 검토 |
| 10초 평균 최댓값 | 90% 미만 |
| 메모리 사용 | 3.0GB 이하 |
| 사용 가능한 memory | 700MB 이상 목표 |
| swap-in/swap-out | 0 |
| 온도로 인한 성능 제한 | 0회 |
| 온도 | 80°C 미만 목표 |
| 카메라 queue | 카메라별 최신 frame 1장 |
| vision 부하 중 heartbeat 위반 | 0회 |
| memory 또는 queue가 계속 증가하는 현상 | 0 |

수치는 30분 부하 시험 후 조정하고 8시간 장시간 시험에서 다시 검증한다.

## 9. USB 배치 원칙

- STM32 ST-LINK VCP는 카메라 hub와 가능한 한 다른 Raspberry Pi 물리 포트에 연결
- 세 카메라는 전원 공급형 hub를 우선 사용하되 `lsusb -t`로 실제 USB 연결 구조 기록
- 1080p를 사용하지 않고 640×480부터 시작
- 세 카메라의 하드웨어 동기화를 가정하지 않음
- frame을 꺼낸 시각을 촬영 시각으로 기록하고 여러 카메라 결과의 최대 시각 차이 검사
- 장치 serial이 없으면 USB 물리 경로와 카메라 식별 자체 시험을 함께 사용

## 10. Process 분리

| Process | 역할 | 영상 처리 장애의 영향 |
|---|---|---|
| `vision_pipeline` | 영상 수집, decode, 검출, 자세 계산 | 재시작 가능 |
| `policy_runtime` | 배포 ONNX policy 추론 | 실패·stale·deadline 초과 시 출력 차단과 Hold |
| `move_group` | 경로 계획과 충돌 검사 | 작업 상태 전환이 늦어질 수 있음 |
| `robot_core` | 상태, 작업, 명령 중재 | 오래된 입력 거부 |
| `control_bridge` | ros2_control, STM32 VCP | 영상 처리와 독립 유지 |
| `safety_supervisor` | 최신성, fault, process 상태 확인 | 영상 처리 실패 시 Hold 요청 |

영상 처리 process가 종료되거나 CPU를 많이 사용해도 STM32 heartbeat와 안전 정지가 같은 process 또는 thread에 묶이지 않게 한다.

## 11. 단계별 확인 방법

1. 카메라 3대의 영상 수집만 실행
2. decode 없이 압축된 최신 frame buffer 검증
3. 카메라별 decode 지연 시간 측정
4. 임시 추론 부하 추가
5. 실제 검출기 추가
6. 실제 배포 ONNX policy를 shadow mode로 추가하고 observation 계약·지연 시간 검증
7. MoveIt 경로 계획을 짧은 시간에 반복하며 동시 부하 확인
8. STM32 heartbeat와 serial 왕복 시간 확인
9. 30분 부하 시험
10. 8시간 장시간 시험

각 단계에서 frame이 지난 시간, decode 및 추론 시간의 p50/p95/최댓값, CPU, memory, 온도, USB reset 횟수와 heartbeat 최대 간격을 기록한다.

## 12. 2026-08-01 현재 관측과 다음 계측

이 절의 시연 재현성 범위는 카메라 각도·높이, 작업대–base transform과
물체 Z가 고정된 상태다. 장소 이동으로 달라질 수 있는 배경, 주변 조명,
반사와 노출 변화에 대해서 검출·정렬 성능을 유지하는지를 검증한다.
mount 또는 물체 높이가 바뀌면 재현성 시험이 아니라 재보정 gate로 전환한다.

- 재배치한 Top 카메라의 `640×480 rgb8` frame 저장과 sharpness `87.93`을
  확인했고, 펜은 사람 눈으로 명확히 보였다.
- 같은 장면에서 기존 threshold 기반 검출은 대리석 무늬와 반사 때문에
  `detected 2 (ignored 2 fully outside)`로 fail-closed 됐다. 카메라 송출
  문제가 아니라 검출기 일반화 문제로 분리한다.
- 다음 구현은 먼저 로봇을 움직이지 않고 Top+양 손목 capture, 선택 decode,
  후보 검출기와 실제 policy ONNX를 함께 구동해 Pi 5 자원 기준선을 잰다.
- `tools/pi_runtime_resource_baseline.py`와 진단 전용
  `RUNTIME_BASELINE` phase를 추가했다. 카메라-only 계측은 즉시 가능하며,
  실제 ONNX bundle이 들어오면 동일 도구의 `--require-policy` gate로
  `config/policy_shadow_diagnostics_contract.json` 계약까지 함께 검증한다.
- 기록 항목은 카메라별 frame age, decode/detector/policy p50·p95·max,
  전체·process별 CPU와 RSS, 온도/throttling, USB reset, serial RTT와
  heartbeat 최대 간격이다.
- 이 기준선이 통과하기 전에는 세 카메라의 동시 full-frame inference,
  policy 실제 명령 권한, 양팔 통합을 허용하지 않는다.
