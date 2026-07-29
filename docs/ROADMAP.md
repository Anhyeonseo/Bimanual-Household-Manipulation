# 검증 게이트 기반 전체 로드맵

## 진행 규칙

- 각 단계의 검증 결과를 `docs/PORTFOLIO_LOG.md`에 기록한다.
- 수치 결과는 `benchmark/results/`에 원본과 요약을 분리해 보관한다.
- 실제 하드웨어 활성화는 이전 단계의 완료 조건을 충족한 뒤 진행한다.

## 단계 0 — 하드웨어 기준선과 요구사항 동결

- 서보 12축의 ID, 방향, raw 범위와 상태값(feedback) 확인
- 전원, 배선, adapter, MCU, 카메라 인벤토리 완성
- 시스템 역할과 안전 상태 확정
- 완료 조건: 미확정 하드웨어 상수 목록과 측정 계획이 모두 존재

## 단계 1 — 저장소·인터페이스·모의 장치(Mock) 골격

- `dual_arm_interfaces`, `dual_arm_description`, `dual_arm_control`
- `dual_arm_safety`, `dual_arm_bringup`, `dual_arm_benchmark`
- SO-101 Xacro, SRDF, ros2_control 모의 하드웨어
- STANDBY/ARMING 최소 상태 머신
- 완료 조건: 새로 내려받은 저장소에서 build, test, 모의 장치 실행 성공

## 단계 2 — STM32 제어 기반

- ST-LINK VCP 바이너리 통신 규격
- CRC, 전송 순서 번호, heartbeat, fault latch
- 단일 팔 UART와 6축 동시 쓰기/읽기
- 공통 제어 주기(tick)와 크기가 제한된 trajectory buffer
- 완료 조건: 단일 팔 통신·동작·SAFE_STOP 실기 시험과 protocol 자동 시험 통과
- 양팔용 독립 UART와 8시간 반복 시험은 단계 10에서 추가

## 단계 3 — Pi 카메라 관리와 성능 기준선

- 카메라 3대의 영상 수집 thread
- 최신 frame 하나만 보관하는 buffer와 queue
- 상태 기반 scheduler와 임시 영상 소비기(dummy consumer)
- USB, FPS, frame age, 재연결, CPU, memory, 온도 측정
- 완료 조건: 카메라와 STM32를 동시에 사용해도 제어 heartbeat 위반이 없음

## 단계 4 — MoveIt/Isaac Sim 기구학 검증

- 정상인 왼팔 단일 planning group을 먼저 검증
- 충돌 모델, 작업 가능 공간(workspace) 계산, 공유·개별 작업 영역
- Isaac Sim에 URDF를 불러오고 카메라 mount 검증
- 완료 조건: 모의 하드웨어와 Isaac 환경에서 대표 trajectory 검증
- 2026-07-24 판정: 왼팔 arm/gripper 대표 trajectory까지 통과. 반대편 팔,
  양팔 planning group, 공유 workspace와 simulated camera mount는 해당
  하드웨어 복구 및 측정 후 후속 gate로 유지
- 2026-07-26 q0 정합: physical raw 2048 Home을 사진과 Isaac interactive FK로
  맞춰 arm 5축 model origin에 흡수. 그리퍼 좌우 개폐 방향도 확인했으며
  URDF, Isaac USD, SRDF Home, bridge zero offset을 하나의 `q=0` 계약으로
  동기화했다. 자동 FK/limit/USD anchor 검증은 통과했지만 사진 기반 1차
  정합이므로 정밀 실물 FK·TCP·collision 계측과 Top–base 재등록 전 task
  motion은 계속 금지
- 2026-07-26 READ_ONLY 재검증: arm 최대 q0 편차 `0.02148 rad`, feedback
  `4.998 Hz`, 실제 feedback 기반 ROS TF와 오프라인 URDF FK 차이 `0.52 µm`로
  encoder→ROS→모델 파이프라인 PASS. 외부 계측 TCP parity와 Top–base 재등록은
  후속 gate
- 2026-07-30 wrist-flex q0 외부 계측 보정: eye-to-hand rigid-target
  sensitivity 분석으로 기존 사진 기반 `-57.5 deg`에 `-7.398281239 deg`를
  추가해 canonical upstream Home을 `-64.898281239 deg`로 정밀화. raw 2048,
  firmware, bridge zero는 유지. 보정 URDF 재계산과 Isaac USD parity는 통과
- 2026-07-30 새 독립 held-out 2자세 eye-to-hand 검증: translation
  RMS/max `0.901 / 1.099 mm`, rotation max `0.521 deg`로 PASS. 이 데이터는
  q0 offset 선택에 사용하지 않았다. 다음 단계인 작업대 물체 실제
  `x/y/yaw` 대조 전에는 `motion_authorized=false` 유지

## 단계 5 — 실제 왼팔 제어

- 초기 B안: MoveIt standard Action → `single_arm_bridge` → STM32 → 정상인 왼팔
- ros2_control hardware interface 전환은 multi-point 계약과 함께 후속 확장
- 단일 관절, 전체 팔, home, 취소, fault 복구
- 완료 조건: 반복 trajectory와 통신 단절 시험 통과
- 2026-07-25 판정: single-point arm/gripper, cancel, SAFE_STOP, 명시적
  recovery, reconnect stale-goal 방지와 MoveIt end-to-end 실기 통과

## 단계 6 — Top 카메라 인식(Perception)

- 카메라 내부 보정(intrinsic calibration)과 작업대 homography
- 임시 입력 → 녹화 영상 → 전통 영상 검출기 → YOLO 순서로 검증
- 펜의 `x, y, yaw`, 검출 신뢰도와 데이터 최신성 출력
- 완료 조건: 위치 오차가 grasp 허용 오차 이내
- 2026-07-30 Top eye-to-hand: 보정 URDF와 독립 held-out 2자세 기준 PASS.
  candidate 자체는 robot target을 제공하지 않으며 작업대 물체 계측 gate가
  남아 있으므로 motion authorization은 계속 false
- 2026-07-30 실제 검은 펜 3배치·15프레임 대조: 위치 RMSE/max
  `6.340 / 7.603 mm`, 장축 yaw RMSE/max `1.899 / 2.911 deg`로
  board-relative coarse perception PASS. table–base 등록과 독립 held-out
  검증까지 통과해 단계 6을 완료했으며, 자동 이동 승인은 단계 7 gate에서
  계속 별도로 관리한다.

## 단계 7 — 재현 가능한 Pick and Place

- 왼팔 상태 머신
- grasp/place 검증
- 50회 반복 시험
- 완료 조건: Pick/Place 각각 90% 이상, 비명령 동작·충돌 0회
- 2026-07-30 비명령 base-frame shadow target 경로와
  freshness/confidence/board-footprint/workspace gate 구현. 실제 Top 입력에서
  후보 `(0.396118, -0.125855, 0.040227) m`가 workspace 내부로 계산됐지만
  `motion_authorized=false`, `robot_target_available=false`를 강제한 상태로
  `VIS-002` 통과
- 2026-07-30 현재 Planar GridBoard를 실제 작업대의 두 위치에서 재측정해
  table–base 등록 통과. 두 위치 간 거리 `160.528 mm`, PnP RMS 최대
  `0.650 px`, 법선 차이 `0.847 deg`, 높이 차이 `1.550 mm`; 기존 20 mm
  검증지 재투영 위치/각도 최대 `8.880 mm / 1.946 deg`
- 기존 `118.216 mm` 차이는 폐기된 높이 있는 목재 체스보드 pose와 다른 세대
  eye-to-hand 결과를 혼합한 비교로 확인해 현재 변환의 차단 사유에서 제거
- 2026-07-30 긴 펜의 전체 footprint를 작은 보정 사각형에 강제하던 임시
  조건을 카메라 전체 가시성 gate와 grasp-point workspace gate로 분리.
  실제 입력에서 후보 `(0.371814, -0.129674, 0.006300) m`,
  `inside_workspace=true`, `source_image_fully_visible=true`,
  `transform_validated=true`로 최신 실시간 shadow 재확인 통과
- 2026-07-30 최초 plan-only 감사에서 기존 operational limit은 현재 펜의
  pregrasp/grasp에 각각 `83.945 / 114.357 mm` 부족해 실패했다. 이 실패는
  로봇 명령 없이 보존했으며, 카메라 보정보다 물리 가동범위가 원인이었다.
- torque-disabled 600초/2182 sample 실측으로 Shoulder 최대 `3830`,
  Elbow 최소 `563`을 확인하고 각각 64 raw 여유를 둔 operational limit
  `3766 / 627`을 채택했다. Wrist Roll은 기존 `1874..2219`를 유지했다.
- 확장 범위의 MoveIt plan-only 재시험은 pregrasp `184`, grasp `216`
  points로 모두 통과했고 Execute API는 사용하지 않았다.
- `0x00020A00`은 최종 오차 soft-abort 뒤 STM32 stop latch가 다시 걸려
  거부했으며, 실패 원인과 rollback을 별도 시험 기록으로 보존했다.
- 최종 `0x00020B00 / 0x4D62F8D5`를 Pi에 플래시하고 identity gate,
  READ_ONLY, MOTION_ENABLED 무동작, Shoulder/Elbow 각 `+0.08 rad / 2 s`
  격리 이동까지 통과했다.
- 다음 gate는 한 번에 Pick을 실행하지 않고 중간 waypoint를 둔 제한
  pregrasp 접근이다. grasp/place 상태 머신과 50회 반복 시험 전까지
  `motion_authorized=false`와 `robot_target_available=false`를 유지한다.

## 단계 8 — 손목(Wrist) 카메라 Visual Servo

- 손목 카메라 위치 보정(eye-in-hand calibration)
- 현재 사용하는 손목 카메라만 처리하도록 일정 관리
- 크기가 제한된 Cartesian 좌표 보정
- 완료 조건: 오래됐거나 신뢰도가 낮은 입력을 차단하고 최종 정렬 오차 목표 충족

## 단계 9 — Raspberry Pi Headless 통합

- ARM64 Release build와 ONNX Runtime, systemd, udev, journald 설정
- watchdog, 재연결, 원격 제어, 안전 종료
- 완료 조건: 반복해서 부팅해도 `STANDBY` 유지, fault 강제 발생 시험, 8시간/24시간 장시간 시험 통과

## 단계 10 — 양팔 병렬·공유 영역

- 왼팔 단독 기준 통과
- 공통 MCU 제어 주기, 실제 시작 시각 차이 측정
- 개별 작업 영역에서 병렬 실행하고 공유 영역은 하나의 계획으로 실행
- 완료 조건: 충돌 0회, 한 팔 fault 발생 시 양팔 동시 정지

## 단계 11 — Isaac Lab policy와 Edge 추론

- 구조화 상태(structured-state) policy
- 시뮬레이션 → 저장 데이터 평가 → 실제 명령 없는 비교(shadow) → 제한된 보정값 적용 순서로 진행
- ONNX로 내보낸 뒤 Raspberry Pi 추론 지연 시간 검증
- 완료 조건: 재현 가능한 기준 동작과 비교해 수치상 개선

## 단계 12 — 수건 접기와 최종 포트폴리오

- 영역 분할(segmentation), 특징점(keypoint), 수건 상태, 양팔 grasp
- 단계별 fold와 재인식
- 최종 benchmark, 영상, 아키텍처·장애복구 보고서
