# 검증 게이트 기반 전체 로드맵

## 진행 규칙

- 각 단계의 검증 결과를 `docs/PORTFOLIO_LOG.md`에 기록한다.
- 수치 결과는 `benchmark/results/`에 원본과 요약을 분리해 보관한다.
- 실제 하드웨어 활성화는 이전 단계의 완료 조건을 충족한 뒤 진행한다.

## 2026-08-01 현재 분기점

- 왼팔은 `0x00021800 / 0x8AD27897`, Shoulder P32, Elbow P28 기준으로
  감독형 Pick–20 mm lift–Place–release–retreat–q0 전 과정을 1회 완주했다.
  7단계 시운전 체크리스트는 100%지만 50회 중 90% 이상이라는 정식 반복성
  기준은 아직 통과하지 않았으므로 단계 7 상태는 `부분 통과`다.
- 현재 11구간 q0 복귀 같은 촘촘한 single-point 연쇄는 안전한 시운전 수단이지
  최종 운용 방식이 아니다. 왼팔에서 시간축·queue·cancel/stop semantics가
  검증된 multi-point/buffered trajectory를 먼저 완성한다.
- 사용자는 오른팔이 정상 동작한다고 확인했다. 다만 저장소의 정식 수락은
  identity·calibration·모델/MoveIt·READ_ONLY/MOTION_ENABLED·단독 Pick/Place
  반복성 증거가 갖춰진 뒤로 분리한다.
- 통합 순서는 **왼팔 생산 기준선 → 오른팔 단독 동등성 → 양팔 통합**으로 한다.
  양팔을 동시에 디버깅하며 단일 팔 결함을 가리는 방식은 사용하지 않는다.
- Isaac Sim/Isaac Lab 학습은 데스크탑에서 수행하고, 검증된 policy만 ONNX
  배포 묶음으로 Pi 5에 넣어 실제 추론한다. Pi에서 Isaac 학습이나 시뮬레이터를
  실행하지 않는다.
- 상세 상태와 남은 gate는
  [현재 상태와 남은 로드맵](CURRENT_STATE_AND_NEXT_ROADMAP.md), 결정 근거는
  [ADR-0012](adr/0012-arm-integration-and-pi-policy-deployment.md)를 따른다.

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
- 양팔용 독립 UART와 8시간 반복 시험은 단계 11에서 추가

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
  grasp 접근이다. lift/place 상태 머신과 50회 반복 시험 전까지
  `motion_authorized=false`와 `robot_target_available=false`를 유지한다.
- 2026-07-31 통신 구조 교체 펌웨어 `0x00021000`은 identity/capability `0x7F`까지
  통과했으나, latched 상태의 READ_ONLY 재연결에서 6축 torque OFF readback 성공 후에도
  `DISABLE status=1(BAD_STATE)`을 반환해 물리 수락을 거절했다. 독립 진단은 6축
  status 0, torque OFF, 전압 `12.3..12.5 V`로 서보 하드웨어 이상을 배제했다.
- `0x00021100`은 DISABLE을 멱등적인 물리 안전 계약으로 수정했다. Pi 배포와
  program/verify/reset, identity `0x7F`, 60초 READ_ONLY, reset·clear fault 없는 연속
  READ_ONLY 재연결, 두 차례 6축 torque OFF readback을 모두 통과했다.
- host shutdown timer/context race도 별도 보강해 전체 287 tests, 독립 build와 실제
  Ctrl+C 무경고 physical DISABLE 종료를 통과했다.
- MOTION_ENABLED 무동작 약 `330.98 s` 동안 heartbeat/feedback/latch 경고 0,
  전압 `12.3..12.5 V`, current 최대 1 raw였고 정상 종료 후 6축 torque OFF를
  독립 readback했다. Shoulder 격리 이동 전까지 task motion은 계속 차단한다.
- 실제 시작 자세 기반 5구간 plan-only는 모두 통과했다. 첫 구간 실기는
  목표 근처까지 이동했지만 terminal `26 raw > 20 raw`로 soft-abort됐고
  latch와 재시도는 없었다. host completion만 `30 raw`로 늘리고 feedback
  recovery는 `20 raw`로 유지하는 수정이 로컬 `259/259`, ROS 21 tests를
  통과했다. Pi 배포·READ_ONLY·MOTION_ENABLED 무동작 재검증 후 fresh
  시작점에서 만든 최대 `0.266422 rad` 5구간을 모두 재시도 없이 실행해
  pregrasp 도달을 PASS했다. 최종 최대 잔차는 Elbow `0.036544 rad`였다.
- 마지막 실제 pregrasp 자세에서 grasp까지 더 촘촘한 `0.18 rad` gate로
  분할했다. 최대 `0.157417 rad`인 2구간 모두 MoveIt plan-only를 통과했고
  실행 API는 사용하지 않았다. 다음 gate는 fresh 시작 오차 `0.05 rad`와
  실제 current-to-target `0.18 rad`를 재확인한 grasp 1번 구간 단 1회다.
- 전원 주기 뒤 pregrasp 복귀 실기에서 Shoulder가 중력을 거스르는 방향으로
  들리지 않아 `59 raw`, 세분화 뒤 `44 raw` soft-abort됐다. latch와 자동
  재시도는 없었다. Shoulder/Elbow torque를 `780 / 650`으로 조정한
  `0x00020C00` 실기도 실패했다.
- torque-limit register `48..49` readback을 fail-closed로 만든 `0x00020D00`을
  실제 배포해 identity, READ_ONLY, MOTION_ENABLED와 register gate를 통과했다.
  큰 Shoulder 명령은 terminal 뒤에도 목표 쪽으로 더 정착했지만 마지막
  `0.079155 rad` 소각도 명령은 fresh feedback에서도 거의 움직이지 않았다.
  이는 100 ms 단발 endpoint 판정과 관측 불가능한 static-control 문제가
  겹친 것으로 분리했다.
- `0x00020E00`은 Pi 배포·flash·READ_ONLY/MOTION_ENABLED diagnostics까지
  통과했으나 첫 Shoulder `-0.08 rad / 2초` 시험에서 terminal `status=8`,
  stop latch가 재현되어 물리 수락을 거절했다. 실제 torque limit/PID/전압은
  정상 readback됐으므로 토크를 더 올리는 방향은 중단했다.
- 코드 감사 결과 binary main loop가 safety service를 먼저 호출하고 host UART를
  한 바이트만 읽어, settling telemetry 중 UART에 도착한 heartbeat frame이 완전히
  decode되기 전에 500 ms deadline을 넘길 수 있었다. 기존 heartbeat는 ACK 없는
  fire-and-forget이라 host도 실제 수신 여부를 알 수 없었다.
- 로컬 `0x00020F00` 후보는 safety service 전에 최대 64 byte를 bounded drain하고,
  heartbeat마다 동일 sequence의 `STATE_FEEDBACK` ACK를 반환한다. Host는 250 ms
  안에 ACK와 unlatched 상태를 확인해야 heartbeat 성공으로 인정한다. Python/ROS
  `276/276`, 표적 `28/28`, C core `1/1`, ament identity, Cortex-M4 Release build를
  통과했다. Pi host/HEX 전송·host backup·single_arm_bridge rebuild도 통과했으며,
  STM32 flash와 post-flash identity/heartbeat ACK까지 통과했다. 첫 READ_ONLY
  diagnostics에서 6축 torque가 모두 켜진 host 초기화 누락을 발견해 수락을 거절했다.
  READ_ONLY·latched startup·모든 shutdown 경로에 firmware DISABLE write/readback을
  강제하는 host-only 수정은 표적 `27/27`, 전체 `280/280`, 독립 ROS build를
  통과했으며 Pi 재배포는 미실행이다. grasp/lift/place는 계속 금지한다.
- 보강된 20F는 READ_ONLY physical disable, ACTIVE 무동작 약 243.5초, shutdown
  6축 torque OFF까지 통과했지만 fresh Shoulder -0.08 rad / 2초 실제 setpoint에서
  heartbeat delay 2회와 terminal status=8 detail=4(HOLD), stop latch가 재현되어
  최종 물리 거절됐다. polling drain은 servo UART 동기 transaction 중 LPUART
  하드웨어에서 이미 유실된 byte를 복구하지 못한다.
- 로컬 0x00021000 후보는 LPUART1 RX interrupt + 1024B ring, RX fault의 원자적
  HOLD/latch, 시작 위치·안전 telemetry·endpoint 검증의 축별 cooperative step을
  적용한다. 전체 283 tests, C core 1/1, 독립 ROS build, Cortex-M4 Release build를
  통과했으며 Pi 전송·플래시·reset·로봇 이동은 0회다. 다음은 20F 신규 backup부터
  시작하는 분리 gate이며 자동 재시도는 금지한다.

- `0x00021600 / 0x8AD27897`에서 Shoulder P32, Elbow P28을 채택했다. 실제
  grasp 뒤 약 20 mm lift는 terminal detail 26 raw로 성공했고, Elbow는
  goal/actual `1537/1553` raw였다. Arm Action 동안 gripper contact goal
  `1963` raw가 유지되고 actual/load/current `1984/96/4`가 보존되어 shared
  commanded-setpoint host 보강을 실물에서 확인했다.
- 물체를 다시 내려놓고 gripper를 연 뒤, 검증된 grasp/pregrasp 경로를
  역순으로 묶은 fail-closed 8구간 q0 복귀를 한 번의 감독 실행으로 완료했다.
  8/8 구간이 성공했고 최종 arm q0 오차는 축별 `2..6` raw였다. 다음 gate는
  perception 입력부터 place까지의 plan-only 상태 머신과 고정된 place
  target 계약이며, 50회 반복 전까지 자동 motion authorization은 false다.

- 기존 Pick에서 base `+Y 60 mm`인 Place 후보까지 mock MoveIt 전 경로를
  최대 `0.18 rad`로 분할해 29 arm segment 모두 plan-only PASS했다. Close와
  release를 포함한 31-step manifest는 calibration `0x8AD27897`, source
  SHA-256, phase 연속성, Place workspace/board, 최종 q0를 독립 검증하며
  `automatic_execution_permitted=false`를 강제한다. 다음은 물리 Place 지점
  확인과 manifest-hash-pinned 수동 gate supervisor다.

- 최종 `0x00021800 / 0x8AD27897 / capabilities 0x000003FF`는 서보 UART
  frame 재동기화·완전 복구와 확장 failure cause를 추가했다. 5분 무동작
  heartbeat/feedback, fault injection의 단일 sweep 실패, reset 없는 6축
  복구를 통과했다.
- Shoulder `0.055 rad`, 나머지 arm 축 `0.050 rad`의 축별 start/post-settle
  gate, 매 구간 6축 진단, Shoulder `<50 C`, soft-abort 무재전송 계약을
  적용해 grasp, 약 20 mm lift, Place, release, retreat와 q0 복귀를 실제로
  1회 완주했다. Place는 최종 두 차례의 제한된 5 mm Z 보정을 포함했고,
  q0 복귀 11/11 구간 뒤 최대 arm 오차는 Wrist Roll `0.007670 rad`였다.
  Bridge는 경고 없이 종료됐고 12 V OFF와 팔 안전 상태를 확인했다.
- 이 결과로 단계 7 **감독형 시운전 체크리스트는 100%**다. 다만 정식 완료
  조건은 Pick/Place 50회 중 90% 이상이므로 검증 매트릭스는 `부분 통과`를
  유지한다. 반복 시험 전에 현재 single-point Action 연쇄를 시간축·queue·
  cancel/stop semantics가 검증된 multi-point/buffered trajectory로 교체한다.
  또한 nominal Place TCP offset `0.025 m`는 실제 안착에서 총 `-10 mm`
  추가 하강이 필요했으므로, Pick/Place offset을 분리하고 Place
  TCP-to-contact 후보 `0.015 m`를 plan-only·충돌 검사·실기 1회로 다시
  보정한 뒤 채택한다.
  상세 결과는
  [감독형 실제 Pick/Place](test-results/2026-07-31-stage7-supervised-pick-place-complete.md)에
  기록했다.

## 단계 8 — 왼팔 생산 기준선과 Visual Servo

- single-point 연쇄를 multi-point/buffered trajectory로 교체하고 시간축,
  queue, cancel, soft-abort, SAFE_STOP 계약을 실물에서 검증
- 2026-08-02: `HOST_MOCK_ONLY`, `motion_authorized=false`인 Motion-1 계약에서
  다중점 검증, 선형 보간, 원자적 queue, cancel/HOLD/underflow와 uint32 wrap
  테스트를 통과했다. STM32 연결·timing 실측·실기 전까지 부분 통과다.
- 2026-08-02 Motion-2: STM32 공통 C core에 원자적 refill, 6축 µrad 선형
  보간, 정상 완료와 planned HOLD/cancel/underflow/missed tick/connection loss/
  tracking-error 진단을 구현했다. fault injection, C11 경고-as-error와
  Cortex-M4 Release cross-build를 통과했다. 0x218 identity·0x3FF capability와
  실제 single-sample route는 유지했으므로 다음은 command route·timing 계측이다.
- 2026-08-02 Motion-3: dormant BEGIN/START/END route, 16/32바이트 terminal
  codec과 host-only timing 분석기를 구현했다. synthetic 자료는 운영값을
  승인하지 않는다. 실제 `binary_control.c`, 0x218 identity·0x3FF capability와
  single-sample runtime은 유지했다.
- 2026-08-02 Motion-4: `binary_control.c`에 candidate validation-only
  route를 연결하고 identity `0x00021900`, capability `0x000007FF`와
  host fail-closed를 추가했다. 검증 성공 응답에서도 queue/accepted/applied는
  0이며 multi-sample servo output은 금지한다.
- 2026-08-03 Motion-4 timing: READ_ONLY Pi–VCP에서 100/80/60 ms lead와
  400 ms horizon을 각 1000회 오류 없이 통과했고 40 ms는 queue admission에서
  fail-closed 거부됐다. reviewed 운영 입력은 20 ms period, 60/400 ms lead,
  prime/watermark/refill `16/10/16`이다. 물리 execution과 ROS Action은 다음
  gate이며 `motion_authorized=false`를 유지한다.
- 2026-08-03 Motion-5 host adapter: 검증된 다중점 경로를 20 ms로 재샘플링하고
  첫 lead 100 ms, 9+7 prime, watermark 10, refill target 16을 적용하는 순수
  host 스케줄러를 구현했다. 80 ms outage의 9+2 refill, ACK 불일치, late lead,
  underflow, cancel과 uint32 wrap mock을 통과했다. ROS Action·serial execution은
  아직 미연결이고 `motion_authorized=false`다.
- 2026-08-03 Motion-6 status mapping: 32-byte extended admission ACK와
  SUCCEEDED/HOLD/CANCELED/ABORTED terminal의 state·reason·safe-stop 조합을
  host scheduler에 연결했다. timeout·legacy/mismatch는 pending frame을 폐기하고
  무재전송 abort한다. transport send와 physical route는 아직 미연결이다.
- 2026-08-03 Motion-7 mock transport: batch binary encode, one-shot exchange,
  outer/payload sequence 일치, timeout·legacy response·terminal-before-ACK
  fail-closed를 mock port에서 검증했다. 실제 serial method는 미연결이다.
- 2026-08-03 Motion-8 physical route 후보: validation route와 분리된
  `0x00022000 / 0x00000FFF` G474 execution route와 host one-shot serial
  method를 구현했다. `t=0` fresh anchor, 1 ms executor, 5 ms 6축 출력,
  underflow/missed-tick/cancel/connection-loss/tracking terminal을 연결했고
  전체 `494` Python/ROS tests, C `2/2`, Cortex-M4 Release build를 통과했다.
- 2026-08-04 Motion-8/9: `0x00022100` bounded lateness에서 Wrist Roll
  `+0.03 rad` 가시 이동을 `accepted=16 / applied=16 / max lateness=2 ms`,
  실측 `+16 raw`, 목표 오차 `4 raw`로 확인했다. 이어 다중점
  `FollowJointTrajectory`를 20 ms streamed queue에 로컬 연결했다.
  heartbeat 기반 진행도는 refill에만 쓰고 extended terminal과 post-settle
  2회 전에는 성공할 수 없다. MoveIt nanosecond timestamp와 검증된
  velocity/acceleration 필드를 지원한다. 이어 Motion-9 Action runtime을 Pi에
  배포하고 Base `+0.015 rad`, Shoulder `+0.015 rad`, Wrist Roll `+0.030 rad`의
  1.2초 왕복 경로를 61 sample·단일 Action goal로 실기 통과했다. 최대 apply
  lateness는 `4 ms`, firmware/독립 readback 최대 오차는 모두 `6 raw`, 자동
  재시도는 0회였고 6축 physical DISABLE도 확인했다. buffered Action 경로는
  `PHYSICAL_ACTION_COMMISSIONED`지만 일반 작업 권한 `motion_authorized=false`는
  유지한다. Motion-10은 20 ms 간격의 해석적 quintic minimum-jerk
  `211`점과 heartbeat-gated 2.5초 post-settle을 적용한 4.2초 q0 왕복을
  단일 Action으로 실기 통과했다. maximum apply lateness `4 ms`,
  post-settle `20 raw`, 최종 최대 오차 `0.021476 rad`, 자동 재시도 0회였고
  bridge 정상 종료와 6축 physical DISABLE도 확인했다. 전 구간 떨림은 크게
  개선됐으나 상승 구간의 약한 흔들림은 후속 품질 항목으로 남긴다.
  Motion-11은 현재 anchor→q0의 실기 검증 경로와 기존 MoveIt 충돌 검사
  q0→Pick pregrasp 12구간을 하나의 dense Action 후보로 결합했다. 첫
  9.1초·456 sample 실기는 경로를 따라 움직였지만 Shoulder/Wrist Flex가
  속도를 추종하지 못해 host post-settle에서 fail-closed ABORTED가 됐다.
  자동 재시도 없이 실측 약 60 raw/s를 보수적 50 raw/s 계약으로 반영해
  43초·2151 sample 후보를 생성했다. 현재는 재검증 plan-only이며
  `motion_authorized=false`를 유지한다. 다음 gate는 fresh anchor 재생성과
  제한 실기 뒤 grasp/lift/place/retreat 연속경로로 확대하는 것이다.
- Pick과 Place의 접촉 Z를 분리하고 Place TCP-to-contact 후보 `0.015 m`를
  plan-only·충돌 검사·제한 실기 순서로 보정
- 대리석 무늬·반사·조명 변화에서도 펜 하나만 검출하도록 색/형상 기반
  후보 생성과 소형 ONNX 검출기를 비교하고 fail-closed gate 유지
- 왼쪽 손목 카메라 eye-in-hand 보정과 마지막 수 cm의 제한된 Cartesian
  visual residual 구현
- 10회 예비 반복 뒤 50회 Pick/Place에서 각각 90% 이상, 비명령 동작·충돌
  0회 달성
- 완료 조건: 카메라 각도·높이·물체 Z를 고정한 채 배경·조명만 다른
  집/시연 환경에서도 왼팔이 같은 성능으로 재현되고, 7단계의 정식 반복성
  gate가 통과

## 단계 9 — Pi 5 세 카메라·Policy Runtime·Headless 기준선

- Top·왼쪽 손목·오른쪽 손목 카메라의 압축 latest-frame slot과 phase
  scheduler를 유지하고 필요한 영상만 decode/inference
- 데스크탑에서 학습·평가한 policy를 versioned ONNX deployment bundle로
  내보내 Pi 5에서 실제 inference
- 학습 입력이 구조화 상태면 구조화 상태 계약을, 세 RGB tensor면 전처리·
  정규화·shape·camera order를 포함한 동일 observation 계약을 보존
- ARM64 Release build, systemd, udev, journald, watchdog, 재연결,
  원격 제어와 안전 종료 구성
- 3카메라+검출기+policy+MoveIt+bridge 동시 부하에서 latency p50/p95/max,
  CPU, memory, 온도, USB reset, heartbeat를 계측
- 완료 조건: 반복 부팅 STANDBY, 30분 부하, 8시간 후 24시간 시험,
  heartbeat 위반 0회와 stale policy/vision 출력의 100% 차단

## 단계 10 — 오른팔 단독 동등성

- P2S real-feedback/no-output gate와 R4 복구를 먼저 끝낸 뒤,
  `docs/checklists/BIMANUAL_JOINT_RANGE_CALIBRATION.md`의 J0..J2를 수행
- 2026-08-13 J0-D는 양팔 12축의 cable-safe desired envelope와 gripper
  closed/task-open checkpoint까지 완료했다. J0-M physical outer margin은 J2 전
  별도 결정으로 남긴다.
- J1-W `0x00024000` 상태형 wrap-aware no-output 후보는 전체 Python `1258/1258`,
  C `6/6`, Cortex-M4 build와 실기 양 SHOULDER forward/reverse wrap, R4 복구 후
  100-sample 12축 soak를 통과했다.
- J1-L에서는 J0-D 끝단을 양쪽에서 64 raw 수축한 arm 5축 후보를 만들었고,
  기존 왼팔 Pick–Place 7개 phase의 1,031점을 전수 검사해 모두 후보 안임을
  확인했다. 최소 여유는 Shoulder 0.130388 rad다. 사용자 승인 뒤 `0x00024100`
  firmware/host parity, 실기 no-output shadow, R4 복구 12축 soak 10/10을 통과했다.
  simulation-only 양팔 URDF, 미참조 MoveIt candidate, Isaac import URDF도 같은
  10축 limit으로 plan-only parity를 통과했다. J0-M, gripper mapping, physical q0와
  base transform, 오른팔 대표 route, J2 bounded active 검증은 남아 있다.
- [역사 기록] J2 진입용 arm 10축 양방향 25/50/75% 내부 목표를 SHA-bound plan-only artifact로
  만들었다. 첫 실기는 오른팔 Base upper 25% raw `2048→2269→2048` 한 축만
  verified disable까지 누적된 R4 `0x00023B00` primitive로 수행한다. 왼팔
  12 V OFF, q0 preflight,
  비선택축 torque-off/drift 감시, 최대 20 raw step, 성공 시 verified disable,
  실패 시 latched stop을 강제한다. 아직 실기 evidence가 없으므로 J2와 일반
  trajectory는 승인되지 않았다.
- 왼팔과 오른팔을 독립적으로 torque-off 수동 계측해 6축 ID·방향·raw
  mechanical/cable envelope·q0·전원·온도·PID/torque readback 확정
- 2026-08-14 작업자가 이미 직접 검증한 J0-D 전체 끝값을 inclusive firmware
  운용 한계로 승인했다. 기존 inset64와 축별 25/50/75% 추가 승인은 더 이상
  runtime blocker가 아니며, canonical 표와 raw↔rad/unwrap parity만 자동 검증한다.
- firmware/host가 같은 canonical limit manifest를 사용한다. URDF/MoveIt/Isaac의
  물리 zero와 collision 모델 정밀화는 명령 범위 승인과 분리해 후속 보정한다
- 오른팔 URDF/MoveIt/Isaac FK와 실제 encoder→ROS→모델 parity 검증
- READ_ONLY physical disable, MOTION_ENABLED 무동작, 단일 축 격리 이동,
  multi-point trajectory, cancel/stop/fault 복구 검증
- 오른쪽 손목 카메라 eye-in-hand 보정과 오른팔 단독 Pick/Place 반복 시험
- 완료 조건: 왼팔과 동일한 하드웨어·모델·안전·태스크 수락 기준을 오른팔이
  독립적으로 통과

## 단계 11 — 양팔 통합과 Policy 권한 확대

- 2026-08-14 작업자 승인으로 수동 검증한 J0-D 전체 범위를 firmware 운용
  범위로 채택했고 추가 J2 축별 25/50/75% gate는 폐기했다. heartbeat/stop/tracking
  안전 계약과 양팔 실제 dispatch gate는 유지한다.
- F7-A에서는 HAL 독립 12축 goal map과 공통 STS3215 SYNC WRITE encoder를
  구현했다. `0x00024500 / 0x607FFFFF`는 아직 일반 12축 UART 출력을 연결하지
  않은 순수 리팩터링 후보이며, 기존 `0x00024400`과 R4 HEX SHA는 불변이다.
- F7 `0x00024604` 후보는 공통 5 ms tick의 12축 executor 출력을 좌우 TX DMA로
  쌍 기동하고, 미완료/전송/tick/heartbeat fault를 양팔 stop으로 묶었다. 자동
  branch anchor와 dispatch 진단을 연결했고, 실기 no-output과 current-pose hold에서
  `36/36` paired DMA, 최대 시작 시차 `2 us`, 최대 launch lateness `46 us`, 정상
  torque-off를 확인했고, base `+0.03 rad` 왕복도 `71/71`, `2 us`, `49 us`로
  통과했다. 별도 one-shot 오른쪽 DMA fault에서도 `8/8` 뒤 failure `+1`과 좌우 verified
  torque-off를 확인했다. 정상 `0x00024604` 복구 no-output까지 통과했으므로
  F7 dispatch/coordinated-stop gate는 완료다. 실행 중 tracking-error feedback은
  다음 F8 항목에서 별도로 완료했다.
- F8 tracking-feedback 후보 `0x00024700`은 각 paired dispatch 뒤 같은 관절의
  좌우 present position을 비동기 병렬 READ하고, 요청 시점 명령값과 비교한다.
  한쪽 응답 실패·4 ms timeout·tracking-error 초과는 다음 tick 출력을 막고 양팔
  coordinated stop으로 수렴한다. 로컬 C `9/9`, 선택 Python `40`, 전체
  Python `1329`과 F7 HEX byte-identical 회귀를 통과했다. 실기 no-output과
  zero-delta hold도 dispatch/feedback `35/35`, 최대 피드백 지연 `2 ms`,
  최대 tracking error `0 urad`, 종료 verified torque-off로 통과했다.
  별도 `0x00024701` tracking-error fault injection도 8번째 paired feedback에서
  정확히 `100000 urad`를 검출하고 다음 출력을 차단했으며 좌우 verified
  torque-off를 통과했다. 정상 `0x00024700` 복구 no-output까지 통과했으므로
  F8 route-time tracking feedback gate는 완료다.
- F8 resident 실행기 `0x00024703`은 finite 재사용, open/append/splice와 명시적
  coordinated STOP을 하나의 Pi serial owner에서 실기 통과했다. F8.1
  `0x00024800`은 별도 12축 measured-feedback snapshot과 ROS
  `/bimanual_stream_adapter/feedback`을 추가했다. direct no-output에서
  `present_mask=0x0FFF`, 실제 양팔 base `+0.03 rad` rolling 왕복에서 feedback
  8 sample, 최대 sample age `27 ms`, commands/epochs `7 / 1,1,2,2,2,2,2`와
  최종 STOP을 통과했다. 이 firmware/ROS 경계는
  [상단 애플리케이션 인터페이스 계약](BIMANUAL_UPPER_APPLICATION_INTERFACE.md)으로
  고정하며, 이후 MoveIt/FSM/pretrained-policy 통합은 이 경계 위에서 진행한다.
- F8.6 `0x00024806`은 position-read failure에만 3회 연속 fault 조건을 적용하고
  정상 pair에서 streak를 복구해 단발 telemetry 손실과 즉시 정지 fault를
  분리했다. F8.7 `0x00024807`은 arm terminal `46,020 urad`를 유지하고 gripper
  contact terminal만 `90,000 urad`로 분리했으며, 12회 연속 fresh pair와 resident
  terminal snapshot을 완료 조건으로 고정했다. F8.7 no-motion, current-pose finite
  2회, fresh-anchor 회귀와 Top-camera 왼팔 Pick/Place 2회(run20/run22)를
  자동 재시도 없이 통과했다. 결과와 남은 오른팔 task/policy/soak 범위는
  [F8.7 최종 수락 결과](test-results/2026-08-15-f87-resident-top-camera-pick-place.md)를
  따른다.
- 좌우 공통 운용 범위, unwrap parity와 왼팔·오른팔 단독 기준선이 준비된 뒤
  dual planning group과 공유
  collision scene 활성화
- 개별 작업 영역에서는 병렬 실행하고 공유 영역에서는 하나의 조정된 계획 사용
- 공통 시간 기준의 실제 시작 시각 차이와 한 팔 fault 시 양팔 동시 정지 검증
- policy는 저장 데이터 평가 → Pi shadow mode → 제한된 residual/팔 선택
  순서로만 권한 확대
- MoveIt은 전역 충돌 회피 경로, policy/visual servo는 bounded residual,
  STM32는 servo timing·watchdog·latch를 담당
- 완료 조건: 충돌 0회, 연동 정지 100%, baseline 대비 policy의 수치상 개선,
  policy 장애 시 Hold 또는 검증된 비정책 경로로만 전이

## 단계 12 — 수건 접기와 최종 포트폴리오

- 영역 분할(segmentation), 특징점(keypoint), 수건 상태, 양팔 grasp
- 단계별 fold와 재인식
- 환경·카메라·모델·policy bundle을 고정한 재현성 시연
- 최종 benchmark, 영상, 아키텍처·장애복구·자원 사용 보고서
