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

## 2026-07-30 — 분할 pregrasp 첫 구간과 완료 판정 분리

- 실제 READ_ONLY 시작 자세에서 pregrasp까지 5구간으로 분할, 구간 최대
  `0.299863 rad`, MoveIt plan-only 5/5 통과
- SHA 고정, fresh 시작 오차 `0.05 rad`, calibration과 one-shot/no-retry
  gate를 추가하고 전체 Python 회귀 `257/257` 통과
- 승인된 첫 구간은 2초간 목표 근처까지 이동했지만 terminal final error
  `26 raw`가 기존 host `20 raw`를 넘어 soft-abort; latch와 재시도 0회
- 실제 최대 잔차는 Elbow `0.026637 rad`; 최초 실기는 계약상 실패로 보존
- host completion만 `30 raw`로 조정하고 feedback recovery trigger와
  target margin은 `20 raw`로 분리 유지
- 보강 후 Python `259/259`, ROS 8 packages build와 21 tests 통과
- Pi 배포 SHA와 백업을 확인하고 READ_ONLY, MOTION_ENABLED 무동작을 재통과
- 전원 주기 뒤 fresh 시작점으로 이전 계획을 폐기하고 새 5구간 생성:
  SHA `664a3a0456facb73f7fafc5e2fa32efd9c0608d4db4321e4d880bb69c10c985e`,
  구간 최대 `0.266422 rad`
- 실제 현재→목표도 `0.30 rad` 이하인지 확인하는 gate를 추가해 Python
  `260/260` 통과
- 새 pregrasp 5구간 모두 MoveIt status `4`, error code `1`, 재시도 0회
- 최종 pregrasp 최대 잔차 Elbow `0.036544 rad`; 다음 gate는 분할 grasp

## 2026-07-30 — 분할 grasp plan-only

- pregrasp/grasp 공용 분할 계획 계약과 SHA 고정 one-shot 실행 gate로 확장
- 집중 테스트 `15/15`, 전체 Python 회귀 `262/262` 통과
- 마지막 실제 pregrasp 자세에서 grasp까지 `0.18 rad` 제한으로 2구간 생성
- 두 구간 최대 변화 `0.157417 rad`, MoveIt error code `1`, 각각
  `28 / 27` trajectory points로 plan-only PASS
- 실행 API와 실제 로봇 bridge는 사용하지 않았으며 자동 재시도 0회
- 계획 SHA:
  `b782ef0315cc2be7213084ffc5301f8ebccb49315b898f09977aeb75e116b37c`
- 다음 gate: fresh 시작 상태 검증 뒤 grasp 1번 구간 2초 단 1회

## 2026-07-30 — Shoulder torque 후보 0x00020C00

- pregrasp 복귀 `0.163079 rad` 구간은 Shoulder `59 raw` 오차로 soft-abort
- 최대 `0.075068 rad`로 세분화해도 `44 raw` 오차로 soft-abort
- 두 번째 실행에서 Shoulder가 목표 반대 방향으로 `0.013806 rad` 밀렸고,
  Elbow/Wrist는 목표 근처로 이동; latch와 자동 재시도 0회
- Shoulder/Elbow torque를 `650/550 → 780/650`, P gain은 `16/24` 유지
- load/current stop `800/320`, 연속 2회 조건과 calibration
  `0x4D62F8D5` 유지
- firmware `0x00020C00` 로컬 후보: Python `264/264`, C core `1/1`,
  ROS bridge build/test, ARM Release build PASS
- HEX SHA:
  `dc44537b914e95e93c543e8d1631ab137fed84f64c0dfc6bbdd8f1f17ee9e984`
- 기존 `0x00020B00` 512 KiB 백업 뒤 Pi 전송, OpenOCD program/verify/reset,
  host identity, READ_ONLY, MOTION_ENABLED 무동작 PASS
- Shoulder `2.330117 → 2.250117 rad`, 2초 격리 명령은 실제
  `2.357728 rad`로 목표 반대 방향에 밀렸고 `59 raw > 30 raw`로
  soft-abort; latch와 자동 재시도 없음
- `0x00020C00` 물리 수락은 실패했으며 토크를 더 올리지 않음

## 2026-07-30 — Torque register readback 후보 0x00020D00

- `0x00020C00`은 torque-limit register `48..49`에 값을 썼지만 P/D/I만
  readback해 `780/650` 실제 적용을 증명하지 못하는 누락을 확인
- 모든 축 trajectory 설정에서 torque-limit 16비트 readback을 요구하고,
  읽기 실패 또는 불일치 시 `HAL_ERROR`를 반환해 상위 rollback이 구성된
  축의 torque를 끄는 fail-closed gate 추가
- Shoulder/Elbow torque `780/650`, P gain `16/24`, load/current stop
  `800/320`, calibration `0x4D62F8D5` 유지
- 집중 Python `38 passed, 22 skipped`, 전체 ROS 환경 Python `265/265`,
  actuator C core `1/1`, ROS bridge build/identity test, Cortex-M4 Release
  build PASS
- image size: text `26156`, data `112`, bss `3080`, total `29348`
- HEX SHA:
  `c4b564145a32994c6601355cebccc08146bfe5287741ec1423a2b5f5c5012126`
- Pi 전송·플래시·로봇 이동은 미실행

## 2026-07-30 — Shoulder 근본 원인 감사와 diagnostics/settling 후보 0x00020E00

- 실제 `0x00020D00` torque-limit readback gate는 배포·MOTION_ENABLED에서 통과
- 큰 Shoulder 명령은 `status=6` 뒤 fresh feedback에서 추가 정착했지만,
  마지막 약 `0.079155 rad` 소각도는 장시간 뒤에도 사실상 정지
- firmware가 보간 종료 100 ms 뒤 위치를 한 번만 읽고 terminal을 보내는
  조기 판정과, load/current/voltage/PID가 보이지 않는 관측성 결손을 분리
- `0x00020E00`: 100 ms 간격, 최대 1초, 30 raw 이내 2회 연속 endpoint
  settling; 정착 중 load/current watchdog 지속
- GET_STATE `[0x02, joint_index]`와 message id `51`로 torque enable/limit,
  P/D/I, position/speed/load/voltage/temperature/current를 on-demand 제공
- `/get_servo_diagnostics`는 Action과 servo bus 소유권을 공유해 동작 중 거부,
  관절 사이 heartbeat로 500 ms watchdog starvation 방지
- Python/ROS `264/264`, ament `21`, C core `1/1`, Cortex-M4 Release build PASS
- HEX SHA `7c042e346a0dbcb4f74d4d0c73f20eedfa9e527349edacee01191870d67d9e0e`
- Pi 전송·flash·로봇 이동 0회. 물리 진단 전 P gain/torque 추가 변경 금지

## 2026-07-31 — 0x00020E00 물리 거절과 acknowledged-heartbeat 후보 0x00020F00

- `0x00020E00`을 실제 배포해 calibration `0x4D62F8D5`, capability `0x1F`,
  READ_ONLY/MOTION_ENABLED와 6축 diagnostics를 통과
- Shoulder/Elbow 실제 설정은 torque `780/650`, P/D/I `16/32/0`, `24/32/0`,
  전압 `12.3..12.5 V`로 확인되어 register 미적용 가설 제거
- 첫 Shoulder `-0.08 rad / 2초` 단 1회에서 `status=8 detail=0`과 stop latch가
  재현되어 20E 물리 수락 거절; 즉시 bridge 종료, 12V OFF, 팔 안전 확보
- 사후 상태는 `STOP_LATCHED=1`, heartbeat count `72`, rejected frames `11`.
  heartbeat age는 bridge 종료 뒤 측정된 누적값이므로 단독 원인 증거로 쓰지 않음
- 근본 원인은 main loop가 safety를 먼저 검사한 뒤 UART를 1 byte만 처리하는 구조와,
  host가 실제 MCU 수신을 확인할 수 없는 fire-and-forget heartbeat 계약의 결합으로 판정
- `0x00020F00`: safety 검사 전 최대 64 byte bounded RX drain, heartbeat별 동일
  sequence state ACK, host 250 ms ACK gate, capability bit `0x20`, status 8 detail에
  실제 safety state 기록
- 표적 `28/28`, 전체 Python/ROS `276/276`, C core `1/1`, ament identity,
  Cortex-M4 Release build PASS; image text `26700`, data `112`, bss `3088`, total `29900`
- HEX SHA `7f4e08027c996929a672aa46287f49e8b1364157e38db1e36b147409170edf78`
- Pi host/HEX 전송, host backup, `single_arm_bridge` rebuild PASS. 실제 build module의
  expected firmware `0x00020F00`, heartbeat ACK timeout `0.25 s` 확인
- 현재 20E flash 512 KiB backup PASS:
  `/home/pi/firmware_updates/backup/stm32_before_0x00020F00.bin`, SHA
  `d8577ac39861d39489c60cd07f571fef98f29951bee6a917eb4e85472d365b66`
- SHA 검증된 20F HEX의 OpenOCD program/verify/reset PASS
- post-flash identity/heartbeat ACK gate PASS: protocol `1`, joints `6`, firmware
  `0x00020F00`, calibration `0x4D62F8D5`, capabilities `0x3F`, latch `0`, ACK `0`
- 첫 READ_ONLY 통신과 diagnostics는 응답했지만 6축 모두 torque enabled로 확인되어
  물리 수락 거절. `allow_motion=false`가 ROS command만 막고 firmware DISABLE을 호출하지
  않는 host 누락과, shutdown disable이 motion/fault/heartbeat 조건에 묶인 결함 확인
- host-only fail-closed 보강: READ_ONLY·latched startup·arming 예외·모든 shutdown에
  6축 physical DISABLE write/readback 강제, latched shutdown heartbeat 선행 제거
- 보강 검증: 표적 `27/27`, 전체 `280/280`, 독립 ROS build/identity PASS
- 보강 Pi 전송·backup·rebuild PASS: source/build SHA
  `93e1b61415020e5ba8ceeb4041cf33e97dc26e92b6e96aec8dde36bac2753e00`
- 보강 READ_ONLY 물리 재시험 PASS: diagnostics 6축 torque OFF, load/current 0,
  voltage `12.3..12.5 V`, heartbeat/feedback/latch 오류 없음
- MOTION_ENABLED 무동작 heartbeat ACK 지속성 PASS: ACTIVE 약 `243.5 s` 뒤에도
  6축 diagnostics 정상, Shoulder/Elbow torque `780/650`, current `2/3 raw`,
  voltage `12.3/12.4 V`, heartbeat/feedback/latch 오류와 로봇 이동 없음
- shutdown physical DISABLE 사후 readback PASS: 별도 transport diagnostics에서
  6축 torque OFF, current 0; startup disable에 의해 결과가 가려지지 않음
- clear fault·로봇 이동 미실행


## 2026-07-31 — 20F 실제 motion 거절과 0x00021000 통신 구조 교체

- 보강 20F의 READ_ONLY physical disable, MOTION_ENABLED 무동작 약 243.5초,
  shutdown 6축 torque OFF는 통과
- fresh Shoulder -0.08 rad / 2초 단 1회에서 heartbeat ACK timeout 경고 2회,
  terminal status=8 detail=4(HOLD), stop latch 재현; 20F motion gate 최종 거절
- 정지 상태에서만 정상이고 motion service에서만 실패하는 증거를 MCU 호출 경로와 대조
- 1Mbps servo UART 동기 read 중 115200bps host LPUART를 polling해 heartbeat byte가
  하드웨어에서 유실되며, 사후 64-byte drain으로 복구할 수 없음을 근본 원인으로 판정
- 0x00021000: LPUART1 interrupt RX, 1024B ring, overflow/error/rearm fault의 원자적
  parser reset + HOLD/latch, capability 0x40 및 host identity fail-closed gate 추가
- 시작 위치, motion safety telemetry, endpoint verification을 one-joint cooperative
  step으로 분할; safety full sweep 약 96ms, Action terminal margin 3.5초
- 전체 Python/ROS 283/283, native C 1/1, single_arm_bridge 독립 build,
  Cortex-M4 Release build PASS; text 30052, data 112, bss 4160
- RX fault 선처리·invalidated ring 폐기까지 fail-closed 보강
- 로컬 HEX SHA 2c9b0f05063890e093d1a910ae4ff11778393cb550862c63badfef2a46bccfca
- Pi 전송·STM32 flash/reset·clear fault·로봇 이동 0회. 다음 물리 gate는 현재
  20F flash 신규 512KiB backup과 12V OFF 재확인부터 분리 수행


## 2026-07-31 — 210 READ_ONLY 계약 거절과 idempotent DISABLE 후보 0x00021100

- 0x00021000 HEX program/verify/reset 및 identity/capability 0x7F gate PASS
- latched READ_ONLY 재연결에서 6축 torque OFF 성공 후에도 DISABLE status=1로 bridge 종료
- 독립 진단에서 stop latch 1, 6축 status/read status 0, torque OFF, 전압 12.3..12.5 V 확인
- 서보 하드웨어가 아니라 FAULT/ESTOP 논리 전이와 물리 DISABLE 결과를 혼합한 계약 오류로 확정
- 0x00021100: fault/latch 보존, 6축 physical disable/readback 성공 status 0, 실패 status 2
- 전체 Python/ROS 284/284, native C 1/1, Cortex-M4 Release build PASS
- HEX SHA 8fd11d901b141cd959c995ed3101f1f2556809b7f417cd967c228b1efb2a7858
- Pi 전송·flash·reset·clear fault·로봇 이동 0회


## 2026-07-31 — 0x00021100 READ_ONLY 물리 수락과 shutdown quiescence

- 210 full-flash 512KiB backup SHA b6cbd426e5409a84afadaa81030aa4d2c24a90cb97a31ea2e9182bd638861e93
- 211 HEX SHA 8fd11d901b141cd959c995ed3101f1f2556809b7f417cd967c228b1efb2a7858 program/verify/reset PASS
- identity: firmware 0x00021100, calibration 0x4D62F8D5, capability 0x7F, latch 0, heartbeat ACK 20
- 첫 READ_ONLY 60초와 연속 두 번째 READ_ONLY에서 6축 torque OFF, 전압 12.3..12.5V; DISABLE status=1 재발 0회
- Ctrl+C context-invalid publish 경고를 host lifecycle race로 분리하고 timer quiescence/context guard 적용
- 전체 287 tests, 독립 ROS build, Pi host backup/rebuild, 실제 무경고 Ctrl+C physical DISABLE 종료 PASS
- clear fault·MOTION_ENABLED·로봇 이동 0회; 다음 gate는 MOTION_ENABLED 무동작 5분


## 2026-07-31 — 0x00021100 MOTION_ENABLED 무동작 수락

- 별도 승인 후 setpoint 없이 MOTION_ENABLED 연결
- 시작 diagnostics: 6축 torque ON, Shoulder/Elbow limit 780/650, 전압 12.3..12.5V, current 최대 2 raw
- 약 330.98초 후 diagnostics: 위치 변화 사실상 0, heartbeat/feedback/latch 경고 0, current 최대 1 raw, 온도 상승 최대 약 2°C
- 보강 host로 Ctrl+C 무경고 종료하고 독립 readback에서 6축 torque OFF, current 0 확인
- clear fault·setpoint·로봇 이동 0회; 다음 gate는 Shoulder -0.08 rad / 2초 단 1회

## 2026-07-31 — 0x00021800 감독형 실제 Pick/Place 완주

- servo UART frame 재동기화·완전 복구 펌웨어 `0x00021800`, calibration
  `0x8AD27897`, capability `0x000003FF` 배포 및 identity gate 통과
- READ_ONLY 6축 physical disable, MOTION_ENABLED 설정 readback, 5분 무동작
  heartbeat/feedback, 단일 exhausted-sweep fault injection과 reset 없는 6축
  복구 통과
- Shoulder P32, Elbow P28과 축별 start/post-settle gate 적용; 매 arm 구간
  6축 진단과 Shoulder `<50 C`, soft-abort 무재전송 정책 강제
- 실제 grasp, 약 20 mm lift, Place 이동, 두 차례 제한된 5 mm Z correction,
  object release, 5구간 retreat와 11구간 q0 복귀 성공
- 자동 재시도 0회. 최종 q0 arm 최대 오차 `0.007670 rad`, Shoulder `36 C`
- Bridge 무경고 종료, 12 V OFF, 팔 안전 확인
- 감독형 시운전 체크리스트는 100%; 정식 단계 7 합격 조건인 50회/90%
  benchmark 전에는 multi-point/buffered trajectory 시간축 계약을 구현·검증
- 상세 증거:
  `docs/test-results/2026-07-31-stage7-supervised-pick-place-complete.md`

## 2026-08-01 — 현재 분기점, Top 카메라 재배치, 오른팔 복구 보고

- 재배치한 Top 카메라에서 `640×480 rgb8`, sharpness `87.93` frame 저장 성공:
  `/tmp/top_relocated_check.png`
- 송출 영상과 저장 frame에서 검은 마커펜이 작업 영역 안에 선명하게 보였다.
  카메라와 ROS image 경로 자체는 정상으로 판정했다.
- 기존 threshold 검출기는 대리석 무늬·반사 배경에서
  `detected 2 (ignored 2 fully outside)`로 fail-closed 됐다.
  송출 문제가 아니라 시연 환경에 대한 검출기 일반화 부족으로 분리했다.
- 이 확인 과정에서 로봇 명령과 실제 이동은 없었다.
  인식이 하나의 유효 물체를 확정하기 전 motion authorization은 계속 false다.
- 사용자는 오른팔이 정상 동작한다고 확인했다.
  정식 수락은 identity·calibration·MoveIt/Isaac·안전·단독 반복성 gate 뒤로 분리한다.
- 통합 순서를 **왼팔 생산 기준선 → 오른팔 단독 동등성 → 양팔 통합**으로 확정했다.
  왼팔의 감독형 1회 완주는 유지하되 50회/90% 전 단계 7은 부분 통과다.
- Isaac Sim/Isaac Lab 학습은 데스크탑에서 수행하고 검증된 ONNX policy를
  Pi 5에 배포해 실제 inference한다. Pi에서 Isaac 학습은 수행하지 않는다.
- MoveIt은 전역 충돌 회피, policy/Visual Servo는 bounded residual,
  STM32는 servo timing·watchdog·latch를 담당하는 책임 경계를 채택했다.
- 다음 구현 순서는 1) 무동작 Pi 5 실제 자원 기준선,
  2) 시연 배경에 강한 펜 검출, 3) multi-point/buffered trajectory,
  4) Pick/Place 접촉 Z 분리 보정이다.

## 2026-08-02 — Pi 5 3카메라·STM32 READ_ONLY 30분 기준선

- 진단 전용 `RUNTIME_BASELINE` phase에서 Top 6 Hz, 양 손목 각 5 Hz RGB
  decode·DDS와 STM32 READ_ONLY bridge를 1,800초 동시 실행했다.
- CPU 평균/p95/최대는 7.94/10.34/34.98%, memory 사용 최대 644.21 MB,
  가용 최소 3,339.75 MB, 온도 최대 40.8°C였다.
- swap, throttling, 카메라 reconnect/decode 실패와 bridge
  heartbeat·feedback·safety-latch 오류는 모두 0이었다.
- `/joint_states`는 5.00049 Hz, 최대 간격 206.41 ms를 유지했다.
- robot command publisher는 0이며 측정 도구는 `/camera_phase`만 발행했다.
- 실제 detector·MoveIt·ONNX policy는 없었으므로 전체 통합 자원 gate는
  부분 통과다. 다음은 deployment manifest를 동결한 policy shadow 계측이다.
- 증거:
  `docs/test-results/2026-08-02-pi-runtime-camera-only-30m.md`,
  `artifacts/stage9/2026-08-02/pi_runtime_camera_only_30m.json`.


## 2026-08-02 — 시연 환경 Top 펜 holdout·legacy 기준선

- Top 카메라 각도·높이와 작업대–base 기하를 고정하고 배경 2종, 조명 3종,
  반사 조건에서 positive 12장과 hard-negative 6장을 수집했다.
- 같은 조건의 펜 제거 영상과의 차영상으로 positive 후보를 만든 뒤 사용자가
  12장 모두의 box와 center를 승인했다.
- yaw는 뚜껑 방향이 아니라 180도 대칭인 무방향 장축(`modulo pi`)으로
  정의하고, 뚜껑 방향은 label하지 않았다.
- 기존 `legacy_dark_threshold`는 positive miss 12/12(100%), hard-negative
  false positive 4/6(66.7%), processing error 2건으로 예상 실패했다.
- 결과는 카메라 송출 문제가 아니라 대리석 무늬·반사·방해물에 대한 기존
  검출기의 일반화 부족을 수치로 고정한다.
- 이 18장은 학습에 사용하지 않는 holdout이며, 다음 경량 YOLO-OBB/ONNX
  후보를 같은 계약으로 비교한다.
- 로봇 명령, bridge, MoveIt과 12 V 동작은 없었다.
- 증거:
  `docs/test-results/2026-08-02-top-pen-holdout-legacy-baseline.md`,
  `artifacts/stage8/top_pen_dataset/manifest.json`,
  `artifacts/stage8/top_pen_detection_legacy_baseline.json`.

## 2026-08-13~15 — 12축 resident firmware와 Top-camera reference app 수락

- 오른팔 bus를 복구하고 좌우 12축 READ_ONLY feedback/torque-off를 확인했다.
  cable-safe desired sweep와 gripper checkpoint를 작업자가 직접 승인했고, shoulder
  4095→0 연속 branch를 unwrapped coordinate로 고정했다.
- 921600 baud host link를 motion 없는 상태에서 30분, 90,000 frame으로 검증했다.
  actual wire 32,000 B/s, transport error 0, rejected delta 0이었다.
- protocol v2의 finite/open/append/splice, 12축 absolute-radian contract와
  owner/epoch/STOP 상태 머신을 구현했다. 상단 앱은 servo raw나 serial protocol을
  직접 다루지 않는다.
- F7 0x00024604에서 공통 5 ms executor와 좌우 paired TX DMA를 실기 수락했다.
  current-pose hold와 base +0.03 rad 왕복, 오른쪽 DMA fault 후 좌우 coordinated
  torque-off를 통과했다.
- F8 0x00024700에서 paired position read와 route-time tracking을 추가했다.
  정상 hold의 tracking pair와 별도 100,000 urad fault injection을 모두 통과했다.
- F8.1 0x00024800에서 12축 measured-feedback snapshot과 ROS feedback topic을
  추가했다. no-output full mask와 actual rolling feedback age 최대 27 ms를 확인했다.
- F8.6 0x00024806은 in-motion position-read 단발 실패와 hard fault를 분리했다.
  3회 연속 실패만 stop으로 승격하고 성공한 pair에서 streak를 복구한다.
- F8.7 0x00024807은 arm terminal 46,020 urad를 유지하면서 gripper contact
  terminal을 90,000 urad로 분리하고, 12회 연속 fresh measured pair 뒤에만
  finite success를 선언한다.
- Pi resident는 motion 직전 torque-off fresh anchor를 취득하고 full finite route를
  내부 9점/400 ms wire window로 공급한다. current-pose finite 2회 재사용,
  explicit STOP과 no-motion gate를 F8.7에서 통과했다.
- Top YOLO-OBB의 원본 pixel x로 팔을 선택하는 reference app이 왼팔 Pick/Place를
  연속 두 번 완주했다. run20/run22 모두 automatic retry 0, q0와 6개 task action,
  최종 epoch 7 armed READY/HOLD였다. 작업 확인 뒤 작업자가 STOP했다.
- 전체 Python/ROS 회귀는 최종 문서 계약 테스트 포함 1416 passed, native
  actuator core 9/9, F8.7 Cortex-M4 Release와 ROS 5 package build를 통과했다.
- 규범 계약:
  docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md
- 최종 수락:
  docs/test-results/2026-08-15-f87-resident-top-camera-pick-place.md
- 다음 범위는 오른팔 task-level Pick/Place, 좌우 반복성 benchmark,
  pretrained-policy shadow/제한 실기와 통합 soak다.
