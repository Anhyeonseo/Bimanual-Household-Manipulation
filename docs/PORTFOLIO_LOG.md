# 포트폴리오 작업 기록

이 파일은 결과만 나열하지 않고 문제, 판단, 검증과 개선 과정을 기록한다.

날짜가 지난 항목은 당시의 판단을 남긴 기록이다. 현재 구현 상태는 [바이너리 제어 경로 검증 결과](test-results/2026-07-20-stm32-binary-control-plane.md)와 [검증 매트릭스](VERIFICATION_MATRIX.md)를 우선해서 확인한다.

## 기록 템플릿

### YYYY-MM-DD — 작업 제목

**목표**

- 이번 작업에서 확인하려는 내용

**구현/시험**

- 변경한 파일, 하드웨어 구성, 실행한 명령

**측정 결과**

| 지표 | 결과 | 조건 |
|---|---|---|
| 예시 |  |  |

**문제와 원인**

- 관찰된 증상
- 확인한 원인

**설계 판단**

- 채택한 방식
- 대안과 채택하지 않은 이유

**증거**

- 로그, 그래프, 사진, 영상, 성능 측정 결과 경로

**완료 판정**

- 통과/실패/차단
- 다음 단계 진입 가능 여부

---

## 2026-07-12 — 프로젝트 착수 및 단계 0 정의

**목표**

- 전체 프로젝트를 검증 게이트 방식으로 관리한다.
- 하드웨어, 안전, ROS, 펌웨어, 카메라, 시뮬레이션과 정책 학습의 선행 관계를 명확히 한다.

**구현/시험**

- 프로젝트 헌장 작성
- 하드웨어 인벤토리 작성
- 전체 로드맵 및 검증 매트릭스 작성
- 핵심 아키텍처 ADR 작성

**설계 판단**

- 오른팔의 재현 가능한 Pick and Place를 첫 통합 목표로 선택했다.
- Policy 학습 전에 측정할 수 있는 기준 동작(baseline)을 만든다.
- Raspberry Pi 카메라 성능 검증은 인식 기능 구현보다 먼저 수행한다.
- STM32는 실시간 actuator 제어와 안전 처리만 담당하게 제한한다.

**완료 판정**

- 문서 생성 후 검토 예정
- 다음 작업: 서보 12축의 하드웨어 기준선 기록

---

## 2026-07-12 — 단계 0 측정 데이터 자동 검증

**목표**

- 서보 기준선 측정의 누락과 안전 상태 모순을 자동으로 검출한다.
- 오른팔 우선 측정과 양팔 최종 게이트를 같은 데이터 형식으로 관리한다.

**구현/시험**

- `hardware/phase0_baseline.json` 측정 템플릿 추가
- `tools/validate_phase0.py`에 문제가 있으면 반드시 실패하도록 만든(fail-closed) 검증기 추가
- 정상 데이터, 오른팔 단독, 중복 관절, 비명령 동작, safe range 이탈 테스트 추가

**측정 결과**

| 지표 | 결과 | 조건 |
|---|---|---|
| 단위 테스트 | 5/5 통과 | Python unittest |
| 실제 오른팔 기준선 | 실패, 미입력 70개 | 측정 전 템플릿 |
| 공백 문자 검사 | 통과 | `git diff --check` |

**설계 판단**

- 측정 전 검증 실패를 정상 상태로 취급한다.
- 누락값을 임의 기본값으로 대체하지 않는다.
- 오른팔만 먼저 검증할 수 있지만 전체 Phase 0 완료는 양팔 데이터가 필요하다.

**증거**

- `tests/test_validate_phase0.py`
- `hardware/phase0_baseline.json`
- `tools/validate_phase0.py`

**완료 판정**

- 도구 구현 PASS
- 하드웨어 게이트는 실제 측정 전이므로 미실행 유지

---

## 2026-07-12 — Pi–STM32 통신 규격 초안

**목표**

- 펌웨어와 ROS hardware interface를 작성하기 전에 유선 통신 규격의 책임, frame 구성, 단위, 상태와 메시지 ID를 고정한다.

**구현/시험**

- COBS + CRC-32C 기반 frame 구조 제안
- little-endian 고정 크기 정수와 micro-radian 단위 정의
- 공통 `apply_tick`을 이용해 좌우 setpoint를 한 번에 적용하는 방식 정의
- session/state/motion/feedback 메시지 ID manifest 작성
- 중복 ID, 예약 범위, 필수 메시지, 잘못된 `ESTOP` software 메시지 검출 시험

**측정 결과**

| 지표 | 결과 | 조건 |
|---|---|---|
| 저장소 전체 단위 테스트 | 10/10 통과 | 단계 0 + 통신 규격 검증기 |
| 통신 규격 manifest | 통과 | 서로 다른 메시지 18개 |
| 문법/공백 문자 | 통과 | compileall, git diff --check |

**설계 판단**

- 소프트웨어 정지는 `SAFE_STOP`, 물리 정지는 E-stop으로 명확히 구분했다.
- Raspberry Pi는 서보 raw 위치가 아닌 관절 micro-radian 값을 전송한다.
- STM32가 보정 정보(calibration)와 최종 raw 제한을 적용한다.
- timeout, queue 크기와 제어 속도는 실제로 측정하기 전에 확정하지 않는다.

**증거**

- `protocol/README.md`
- `protocol/message_ids.json`
- `tools/validate_protocol_manifest.py`
- `tests/test_validate_protocol_manifest.py`

**완료 판정**

- 통신 규격 구조 검증 통과
- ADR-0006은 하드웨어 측정과 사용자 검토 전까지 제안 상태

---

## 2026-07-12 — Raspberry Pi 카메라 역할과 연산 자원 한도

**목표**

- 카메라 3대, 검출기, MoveIt과 향후 policy가 Raspberry Pi 5 4GB 자원을 무제한으로 경쟁하지 않도록 작업 단계별 연산 한도를 정의한다.

**구현/시험**

- 상단/왼쪽 손목/오른쪽 손목 카메라 역할과 보정 범위 정의
- 압축된 최신 frame, 선택적 decode, 단일 추론 실행 경로 정의
- 작업 상태 8개의 decode, 추론, policy 실행 속도 작성
- 전체 영상 추론 속도, queue 크기, thread 수, 원본 영상 policy 금지를 자동 검증

**측정 결과**

| 지표 | 결과 | 조건 |
|---|---|---|
| 저장소 단위 테스트 | 15/15 통과 | 기준선·통신 규격·카메라 검증기 |
| 카메라 실행 일정 | 통과 | 작업 상태 8개 |
| 최대 영상 추론 속도 | 12Hz | DUAL_PRIVATE |
| Policy 입력 | 구조화 상태 | 원본 영상 사용 안 함 |

**설계 판단**

- 상단 카메라는 전역 평면 자세와 결과 확인, 손목 카메라는 마지막 상대 정렬을 담당한다.
- 카메라 연결·영상 수집 속도와 decode·추론 속도를 분리한다.
- 영상 처리 thread 2개와 policy thread 1개를 초기 상한으로 두고 실제 CPU core 고정은 성능 측정 후 결정한다.
- 제어와 안전 처리는 부하를 줄이는 대상에서 제외한다.

**증거**

- `docs/CAMERA_COMPUTE_ARCHITECTURE.md`
- `config/camera_schedule.json`
- `tools/validate_camera_schedule.py`
- `tests/test_validate_camera_schedule.py`

**완료 판정**

- 아키텍처와 정적 연산 한도 검증 통과
- 실제 UVC/Raspberry Pi 성능 측정 전까지 ADR-0007은 제안 상태

---

## 2026-07-21 — 카메라 선택적 decode와 STM32 제어 격리

**목표**

- 카메라 세 대의 RGB decode와 DDS 전달 부하가 Raspberry Pi 자원과 STM32 heartbeat·feedback을 방해하지 않는지 실기로 확인한다.

**구현/시험**

- 작업 phase에 따라 필요한 카메라만 JPEG decode하는 scheduler 구현
- phase별 frame age와 decode 시간 p50/p95/max 진단 추가
- `DUAL_PRIVATE`에서 top 6Hz, wrist A/B 각 5Hz RGB topic 동시 소비
- STM32 bridge를 READ_ONLY, heartbeat 10Hz, joint feedback 5Hz로 동시 실행
- CPU, memory, 온도, swap과 joint feedback 간격을 120초간 측정

**측정 결과**

| 지표 | 결과 |
|---|---:|
| package 시험 | 14 tests, 실패 0 |
| phase별 목표 decode rate | 전부 일치 |
| JPEG decode 실패 | 0회 |
| frame age 전체 최댓값 | 35.98ms |
| JPEG decode 전체 최댓값 | 6.31ms |
| 부하 중 `/joint_states` | 5.008Hz |
| joint feedback 최대 간격 | 201.30ms |
| CPU 평균/1초 최대 | 6.38% / 8.73% |
| memory 사용 최대 | 465.3MB |
| 온도 최대 | 33.6°C |
| swap in/out | 0/0 |
| 시험 후 STM32 stop latch | 0 |

**설계 판단**

- 세 카메라는 capture를 유지하되 작업에 필요하지 않은 JPEG는 decode하지 않는다.
- RGB 원본은 queue에 쌓지 않고 sensor-data QoS depth 1로 전달한다.
- phase 전환 시 rolling 통계를 초기화해 서로 다른 작업 단계의 지연값이 섞이지 않게 한다.
- 제어와 영상 처리는 서로 다른 process에서 실행하고 영상 부하 때문에 heartbeat rate를 낮추지 않는다.

**증거**

- `docs/test-results/2026-07-21-camera-phase-decode-latency.md`
- `docs/test-results/2026-07-21-camera-decode-control-load.md`
- `tools/camera_control_load_test.py`
- `ros2_ws/src/manipulation_camera_manager`

**완료 판정**

- `CAM-003`, `CAM-005` 통과
- `RES-001`의 capture + decode + DDS 하위 gate 통과
- 실제 perception inference, MoveIt과 장시간 부하는 후속 단계에서 검증

---

## 2026-07-24 — 왼팔 MoveIt·Isaac Sim 6.0.1 통합

**목표**

- 고장 난 반대편 팔을 제외하고 정상인 왼팔 하나의 simulation vertical
  slice를 먼저 완성한다.
- URDF/Xacro, MoveIt, controller action과 Isaac articulation이 동일한
  joint 이름, radian, q0와 positive direction을 사용하게 한다.
- 실제 STM32/servo를 활성화하지 않고 대표 trajectory를 검증한다.

**구현/시험**

- TheRobotStudio SO-101 geometry를 pinned commit 기준으로 가져와 왼팔
  URDF/Xacro와 `ros2_control` mock interface 구성
- `left_arm` 5-DOF, `left_gripper` 1-DOF SRDF와 KDL position-only IK 구성
- mock controller에서 arm/gripper Plan/Execute 검증
- Isaac Sim 6.0.1 stage에 articulation drive와 ROS 2 Joint States
  OmniGraph 저장
- project joint와 Isaac joint 사이의 sign/offset adapter 구현
- MoveIt action을 Isaac `/isaac/joint_command`로 전달하고
  `/isaac/joint_states`를 project `/joint_states`로 변환

**측정 결과**

| 지표 | 결과 |
|---|---:|
| mapping unit test | 3/3 통과 |
| mock arm/gripper | 모두 Plan/Execute 성공 |
| direct Isaac arm/gripper action | 모두 `SUCCEEDED` |
| MoveIt → Isaac arm | random valid pose와 home 성공 |
| MoveIt → Isaac gripper | open/closed 성공 |
| home 후 최대 project joint 오차 | 약 `0.0097 rad` |
| goal tolerance | `0.03 rad` |
| 실제 servo 동작 | 0회 |

**문제와 원인**

- desktop에서 실행한 Isaac은 ROS library path가 없어 ROS 2 Bridge가
  시작되지 않았다.
- Isaac 6.0.1의 USD Python API와 Jazzy
  `ParallelGripperCommand` feedback schema가 초기 가정과 달랐다.
- bridge 종료 시 ROS context 이중 shutdown 예외가 있었다.

**설계 판단**

- Isaac Sim은 ROS 2 Jazzy 환경을 source한 terminal에서 시작한다.
- 다섯 arm joint는 sign을 반전하고 gripper는 sign 유지와 `+10 deg`
  project offset을 적용한다.
- Isaac topic과 USD path는 `so101_isaac_bridge` 안에 격리하고 MoveIt
  controller contract는 backend와 무관하게 유지한다.
- 단일 왼팔 단계 4를 완료하되 양팔·camera mount는 검증 없이 PASS로
  올리지 않는다.

**증거**

- `docs/checklists/PHASE_4_ISAAC_MOVEIT_INTEGRATION.md`
- `docs/test-results/2026-07-24-isaac-moveit-left-arm-integration.md`
- `ros2_ws/src/so101_isaac_bridge/test/test_mapping.py`
- `isaac_sim/assets/so101_new_calib/so101_new_calib.usda`

**완료 판정**

- 단계 4 단일 왼팔 simulation vertical slice 통과
- 실제 hardware는 비활성
- 단계 5 실제 hardware 진입 전 safe limit과 backend 단일 선택 조건 검토

---

## 2026-07-25 — 단계 5 실제 왼팔 MoveIt·STM32 통합

**목표**

- Pi가 STM32 serial을 단독 소유하고 워크스테이션 MoveIt이 표준 controller
  Action으로 실제 왼팔을 제어한다.
- cancel, SAFE_STOP, 명시적 recovery와 reconnect 뒤 stale goal 방지를
  실제 hardware에서 확인한다.
- 초기 B안은 검증된 single-point만 허용하고 multi-point 확장은 분리한다.

**구현/시험**

- Jazzy `FollowJointTrajectory`와 `ParallelGripperCommand` Action adapter 구현
- calibration hash와 strict raw range, no-clamp, default READ_ONLY gate 유지
- active motion 동안 servo bus와 충돌하는 `GET_STATE`를 일시 정지하고 완료 뒤
  정규 5 Hz feedback 자동 복귀
- 워크스테이션 전용 `external_stm32_moveit.launch.py`와 one-shot MoveIt 실행
  도구 추가
- arm q0, 0.05 rad, 0.10 rad와 gripper 0.08 rad 실제 MoveIt 경유 실행
- gripper cancel 뒤 SAFE_STOP latch와 사용자 승인 recovery 실행
- process 재시작 뒤 5초간 이전 goal 재개·재전송 0회 확인

**측정 결과**

| 지표 | 결과 |
|---|---:|
| Python 회귀 | 116/116 통과 |
| ROS workspace build | 6 packages 통과 |
| `/joint_states` | 5.000 Hz |
| representative arm 최대 final error | 18 raw |
| visible arm 최대 final error | 15 raw |
| gripper final error | 약 5 raw |
| 허용 final error | 20 raw |
| cancel Action status | CANCELED |
| reconnect stale goal | 0회 |
| MoveIt/bridge 실기 WARN·ERROR | 0회 |

**설계 판단**

- 현재 B안 single-point 계약을 단계 5 완료 기준으로 유지한다.
- 일반 OMPL multi-point 실행은 firmware queue/streaming 또는 ros2_control
  hardware interface 계약을 구현한 뒤 확장한다.
- 실제 Pick and Place는 단계 6 perception 정확도 gate 뒤 단계 7에서 진행한다.

**증거**

- `docs/test-results/2026-07-25-phase5-stm32-read-only.md`
- `docs/checklists/PHASE_5_LEFT_ARM_HARDWARE_BACKEND.md`
- `tools/ros_moveit_execute_once.py`

**완료 판정**

- 단계 5 초기 B안 single-point hardware milestone 통과
- 실제 로봇 q0 원복, MoveIt/Pi bridge 종료와 servo power OFF 완료
- 다음 단계: 단계 6 Top 카메라 인식

---

## 2026-07-30 — 단계 6 Top 작업대 물체 실제 좌표 검증

**목표**

- 검은 펜의 `top_board` 기준 `x/y/yaw`를 출력한 20 mm 검증지의 실제 배치와 비교한다.
- 보정 영역 밖 로봇팔 윤곽은 무시하되, 경계와 교차하는 물체는 계속 fail-closed로 차단한다.

**구현/시험**

- 완전히 보정 영역 밖인 contour만 object count에서 제외하는 필터 추가
- 영역 경계와 교차하는 contour는 relevant candidate로 유지
- 빈 작업대와 validation grid 인쇄선의 오검출 0개 확인
- 검은 펜 3위치·3각도에서 각 5프레임, 총 15프레임 검출
- 로봇 12 V OFF, 실제 이동 0회

**측정 결과**

| 지표 | 결과 |
|---|---:|
| 검출 성공 | `15/15` |
| 위치 평균/RMSE/최대 | `6.251 / 6.340 / 7.603 mm` |
| yaw 평균/RMSE/최대 | `1.609 / 1.899 / 2.911 deg` |
| 반복 위치 span 최대 | `0.385 mm` |
| 반복 yaw span 최대 | `0.022 deg` |
| 회귀시험 | `213/213` 통과 |

**완료 판정**

- `VIS-001` board-relative coarse perception 통과
- `motion_authorized=false`, `robot_target_available=false` 유지
- 다음 단계: base-frame shadow target, workspace와 freshness gate 검증

---

## 2026-07-30 — 단계 7 준비 base-frame shadow target

**목표**

- `top_board`의 물체 좌표를 `left_base_link` 후보 좌표로 변환한다.
- 변환 검증 전에는 진단용 shadow 결과만 제공하고 로봇 목표 생성을 차단한다.

**구현/시험**

- freshness, confidence, board footprint, workspace를 모두 검사하는 fail-closed
  shadow 변환 core와 ROS 2 node 구현
- 실제 Top 카메라 입력에서 후보 위치
  `(0.396118, -0.125855, 0.040223) m` 출력
- `shadow_pose_available=true`, `inside_workspace=true`, `fresh=true` 확인
- `transform_validated=false`, `motion_authorized=false`,
  `robot_target_available=false` 강제 확인
- 관련 회귀시험 `223/223`, shadow 단위시험 `14/14` 통과
- 로봇 제어 프로세스와 12 V 이동 없이 dry-run 수행

**판정**

- `VIS-002` 비명령 shadow target과 안전 gate 통과
- 최초 판정 당시 `118.216 mm` 불일치 때문에 실제 단계 7 Pick을 시작하지
  않았으며, 아래의 현재 Planar GridBoard 재검증에서 혼합 기준 오류로 정정
- 당시 다음 gate: 독립 물리 기준으로 `left_base_link ← top_board` 변환 검증

**증거**

- `docs/test-results/2026-07-30-top-base-shadow-target.md`
- `docs/test-results/evidence/2026-07-30-top-shadow-dry-run.yaml`


---

## 2026-07-30 — 현재 작업대–왼팔 base 등록 재검증

**교정**

- 앞선 `118.216 mm` 차이는 현재 작업대 좌표의 실패가 아니라, 폐기된 높이
  있는 목재 체스보드 pose와 다른 세대 eye-to-hand 결과를 섞어 비교한
  결과임을 확인
- 현재 사용하는 대형 Planar GridBoard(ID `10..29`)만으로 기준을 통일

**검증**

- 실제 작업대의 서로 떨어진 두 위치에서 GridBoard PnP 수행
- 위치 간 거리 `160.528 mm`
- PnP RMS 최대 `0.650 px`
- 평면 법선 차이 `0.847 deg`, 평균 높이 차이 `1.550 mm`
- 저장된 20 mm 물리 랜드마크의 현재 보정 재투영:
  위치 RMS/최대 `7.116 / 8.880 mm`, yaw RMS/최대
  `1.440 / 1.946 deg`
- 전체 회귀시험 `233/233` 통과

**판정**

- 현재 table–base transform `validated=true`
- `motion_authorized=false`, `robot_target_available=false` 유지
- GridBoard 제거 후 실제 검은 펜으로 최신 transform의 실시간 shadow 재확인:
  `(0.371814, -0.129674, 0.006300) m`, confidence `0.857`,
  `source_image_fully_visible=true`, `inside_workspace=true`
- 긴 물체의 전체 footprint 보정 사각형 조건을 카메라 전체 가시성 조건과
  grasp-point workspace 조건으로 분리
- 다음 gate: 명령 없는 plan-only grasp 후보 검증

**증거**

- `docs/test-results/2026-07-30-current-table-base-registration.md`
- `docs/test-results/evidence/2026-07-30-top-base-table-validation.yaml`
- `docs/test-results/evidence/2026-07-30-top-shadow-corrected-validation.yaml`


---

## 2026-07-30 — 단계 7 카메라–하드웨어 도달영역 감사

**발견**

- Top 카메라와 table–base transform은 유효
- 기존 shadow workspace가 현재 승인 관절범위에서 계산되지 않아 카메라에
  보이는 펜을 잘못 `inside_workspace=true`로 표시한 사실 확인
- 즉시 hardware-limit 기반 fail-closed workspace로 정정

**수치**

- 현재 펜: `(0.371814, -0.129674, 0.006300) m`
- 승인 관절범위 TCP 최대 x: `0.332350 m`
- pre-grasp/grasp 전역 최소 오차: `83.945 / 114.357 mm`
- 전체 URDF 한계에서는 두 목표 모두 위치 오차 `0.0 mm`
- 필요한 shoulder/elbow: pre-grasp `1.862 / 1.020 rad`,
  grasp `2.257 / 1.359 rad`

**판정**

- 카메라 재보정보다 shoulder/elbow 물리 안전범위 재측정이 선행
- torque-disabled raw observer 전에는 firmware/MoveIt 한계를 확장하지 않음
- MoveIt plan service만 사용했고 Execute API 및 실제 로봇 이동은 0회
- 전체 회귀시험 `236/236` 통과

**증거**

- `docs/test-results/2026-07-30-stage7-reachability-audit.md`
- `docs/test-results/evidence/2026-07-30-stage7-reachability-blocked.yaml`


---

## 2026-07-30 — torque-off 물리 범위 재검증과 plan-only 해제

**실측**

- 12V 전원과 팔 지지 절차 후 `DISABLE` 확인, motion command 없이 600초간
  2182 sample 수집
- 수동 Shoulder/Elbow 펼침, 현재 검은 펜의 pregrasp와 grasp 위치 도달
- Shoulder `2046..3830`, Elbow `563..2444`, Wrist Roll `1981..1988` 기록

**보수적 operational limit**

- 관측 끝값에서 64 raw, 약 5.625도 여유 적용
- Shoulder `1988..3766`, Elbow `627..2258`
- Wrist Roll은 기존 `1874..2219`를 유지하고 측정 증거에 포함
- 배포 firmware `0x00020B00`, calibration `0x4D62F8D5`
- HEX SHA-256 `d1a6536c1833443629ff103ecba3452820e3880ab59f02a78d845eed4a72e405`

**검증**

- 핵심 회귀시험 `64/64`, 최종 저장소 회귀시험 `244/244` 통과
- STM32 ARM Release 및 ROS package build 통과
- localhost 전용 Domain 93 MoveIt plan-only: pregrasp `184`, grasp `216`
  trajectory points, 모두 SUCCESS; Execute API 사용 0회
- Pi 전송 SHA 확인, OpenOCD program/verify/reset, identity gate 통과
- READ_ONLY와 MOTION_ENABLED 무동작 연결 통과
- Shoulder/Elbow 각 `+0.08 rad / 2 s` 격리 이동 통과
- `0x00020A00` heartbeat/settling 후보는 soft-abort 뒤 stop latch가 다시
  걸려 거부하고 `0x00020900`으로 rollback한 뒤 `0x00020B00`을 배포
- 다음 gate: 중간 waypoint를 둔 제한 pregrasp 접근; 전체 Pick은 아직 금지
