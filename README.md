# Headless 양팔 로봇 조작 시스템

Raspberry Pi 5, ROS 2 Jazzy, STM32G474, 두 대의 SO-ARM101과 세 대의 USB 카메라를 통합하는 멀티카메라 듀얼암 로봇 프로젝트다.

현재 기준선은 상단 카메라로 마커펜을 왼팔이 집어 중간에 놓고 오른팔이 다시 집어 옮기는 양팔 연속 Pick and Place다. 다음 범위는 누운 캔 한 개의 수직 접근 파지이며, 캔→쓰레기통과 handover는 이후 PR로 분리한다.

## 핵심 원칙

- Raspberry Pi는 인식, TF, 계획, 상태 머신과 운영을 담당한다.
- STM32는 서보 버스 타이밍, 짧은 setpoint 보간, 제한, watchdog과 fault 처리를 담당한다.
- 부팅과 재연결만으로 로봇이 움직이지 않는다. 기본 상태는 `STANDBY`다.
- 모든 기능은 검증 게이트를 통과한 뒤 다음 단계로 이동한다.
- 성능과 안정성은 추측하지 않고 측정 결과를 남긴다.
- 실제 하드웨어 상수는 측정 전 코드 기본값으로 사용하지 않는다.

## 현재 상태

- resident firmware: F8.9 `0x00024809`, protocol 2, 12 joints,
  capabilities `0xEFFFFFFF`.
- STM32는 공통 5 ms executor, paired DMA dispatch, operational limits,
  measured tracking, heartbeat/watchdog와 coordinated torque-off를 소유한다.
- Pi의 `bimanual_stream_adapter`만 serial을 소유하며 fresh anchor,
  owner/epoch, finite trajectory와 terminal feedback을 ROS로 제공한다.
- 팔 관절 route tracking 한계는 90,000 µrad로 유지한다. 물체 접촉 시
  그리퍼만 150,000 µrad를 허용하며 firmware hard cap은 160,000 µrad다.
- 상단 카메라/작업대 보정과 오른팔 data-fit URDF를 적용했다.
- 계획 스키마 12는 화면축 좌우 보정을 homography와 plan SHA에 고정한다.
  왼팔은 화면 오른쪽 13.72 mm, 오른팔은 화면 왼쪽 29.47 mm다.
- F8.9 no-motion, current-pose hold 2회와
  왼팔→오른팔 펜 전달을 automatic retry 없이 완주했다.
- legacy `single_arm_bridge` 일반 trajectory backend는 비승인이다.
  양팔 motion은 resident adapter 경로만 사용한다.
- 다음 PR은 Top YOLO-OBB, 수직 접근, 캔 장축에 수직인 gripper closing axis,
  nearest-equivalent wrist roll과 감독 1회 파지만 다룬다.

규범 경계는
[양팔 상단 애플리케이션 인터페이스](docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md),
최종 실기 결과는
[F8.9 resident와 양팔 펜 전달](docs/test-results/2026-08-16-f89-bimanual-pen-transfer.md)에
기록했다.

## 새 개발 환경 준비

필수 도구:

- STM32CubeIDE 2.2.0 이상과 STM32CubeG4 package
- Python 3.12 이상
- host-side C core를 빌드할 경우 CMake와 C11 compiler

Windows PowerShell에서 Python 환경과 자동 테스트를 준비한다.

```powershell
py -3.12 -m venv .venv-host
.\.venv-host\Scripts\Activate.ps1
python -m pip install -r requirements-host.txt
python -m unittest discover -s tests -p "test_*.py"
python tools\validate_protocol_manifest.py
```

STM32CubeIDE에서는 `firmware/stm32_g474_single_arm`을 Existing Project로 import한다. 상위의 `firmware/stm32_actuator`가 linked resource로 연결되므로 두 디렉터리의 상대 위치를 바꾸지 않는다. `Debug/` 산출물과 개인별 `.launch` 설정은 저장소에 포함하지 않으며 각 PC에서 다시 생성한다.

실제 모터를 사용하는 기본 점검(smoke) 및 동작 시험 도구는 `tools/stm32_*_test.py`에 있다. 전원 차단 수단과 작업 공간을 확보한 뒤 실행한다.

## 공식 MoveIt bringup

backend는 공식 진입점 하나에서 독점 선택한다. 기본값은 실제 장치를 열지 않는
`mock`이며, `stm32`도 명시적으로 허용하기 전에는 READ_ONLY다.

```bash
ros2 launch so101_bringup so101_moveit.launch.py backend:=mock
ros2 launch so101_bringup so101_moveit.launch.py backend:=isaac
ros2 launch so101_bringup so101_moveit.launch.py backend:=stm32
```

동시에 두 bringup을 실행하면 두 번째 실행은 provider node를 시작하기 전에
runtime lock에서 거부된다.

Pi가 STM32 bridge와 serial을 소유하고 워크스테이션이 MoveIt/RViz를 실행하는
분산 구성은 다음 전용 launch를 사용한다. 이 launch는 로컬 hardware provider를
추가로 만들지 않는다.

```bash
ros2 launch so101_bringup external_stm32_moveit.launch.py
```

## 장치별 로컬 설정

공개 저장소에는 개인 NUCLEO의 ST-LINK serial을 넣지 않는다. `single_arm_bridge`의 공개 기본값 `serial_device: auto`는 다음 순서로 장치를 찾는다.

1. `/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_*-if02`와 일치하는 장치가 정확히 하나면 사용
2. by-id 장치가 없고 `/dev/ttyACM0`가 있으면 fallback으로 사용
3. ST-LINK가 여러 개면 임의로 선택하지 않고 실행을 거부

여러 보드를 연결하거나 장치를 명시적으로 고정하려면 아래 example을 복사한다.

```bash
cd ~/Manipulation/ros2_ws/src/single_arm_bridge/config
cp bridge.local.yaml.example bridge.local.yaml
```

`bridge.local.yaml`에 실제 by-id 경로를 넣고 package를 다시 build하면 기존 `ros2 launch single_arm_bridge bridge.launch.py` 명령이 local 설정을 자동으로 우선 적용한다. `*.local.yaml`은 Git에서 제외되므로 공개 저장소에 장치 식별자가 올라가지 않는다. 자세한 내용은 [로컬 하드웨어 설정](docs/LOCAL_HARDWARE_CONFIG.md)에 기록했다.

## 문서 안내

- [양팔 상단 애플리케이션 인터페이스 계약](docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md)
- [상단 애플리케이션 개발 인계 프롬프트](docs/prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md)
- [F8.7 resident·Top 카메라 Pick/Place 최종 수락 결과](docs/test-results/2026-08-15-f87-resident-top-camera-pick-place.md)
- [F8.9 resident·양팔 펜 전달 최종 수락 결과](docs/test-results/2026-08-16-f89-bimanual-pen-transfer.md)
- [프로젝트 헌장](docs/PROJECT_CHARTER.md)
- [현재 분기점과 남은 로드맵](docs/CURRENT_STATE_AND_NEXT_ROADMAP.md)
- [전체 로드맵](docs/ROADMAP.md)
- [하드웨어 인벤토리](docs/HARDWARE_INVENTORY.md)
- [단계 0 하드웨어 검사](docs/checklists/PHASE_0_HARDWARE_BASELINE.md)
- [단계 0 측정 데이터](hardware/phase0_baseline.json)
- [검증 매트릭스](docs/VERIFICATION_MATRIX.md)
- [포트폴리오 작업 기록](docs/PORTFOLIO_LOG.md)
- [아키텍처 결정 기록](docs/adr/README.md)
- [Pi–STM32 통신 규격 초안](protocol/README.md)
- [Pi 카메라·연산 아키텍처](docs/CAMERA_COMPUTE_ARCHITECTURE.md)
- [STM32 모듈 구조와 Isaac Sim 확장 경계](docs/STM32_MODULAR_ARCHITECTURE.md)
- [STM32 단일 팔 실기 체크리스트](docs/checklists/STM32_SINGLE_ARM_BRINGUP.md)
- [단계 4 왼팔 Isaac Sim·MoveIt 체크리스트](docs/checklists/PHASE_4_ISAAC_MOVEIT_INTEGRATION.md)
- [단계 4 시험 결과](docs/test-results/2026-07-24-isaac-moveit-left-arm-integration.md)
- [단계 5 왼팔 hardware backend 체크리스트](docs/checklists/PHASE_5_LEFT_ARM_HARDWARE_BACKEND.md)
- [단계 5 gripper mapping 측정 계획](docs/checklists/PHASE_5_GRIPPER_MAPPING_PLAN.md)
- [단계 5 acceptance와 rollback 기준](docs/checklists/PHASE_5_ACCEPTANCE_ROLLBACK.md)
- [단계 5 STM32 READ_ONLY 실기 결과](docs/test-results/2026-07-25-phase5-stm32-read-only.md)
- [단계 6 Top 물체 좌표 검증](docs/test-results/2026-07-30-top-object-ground-truth-validation.md)
- [단계 8 Top 펜 검출 데이터 기준선](docs/checklists/STAGE8_TOP_PEN_DETECTION_BASELINE.md)
- [단계 8 경량 YOLO-OBB 펜 검출 후보](docs/checklists/STAGE8_TOP_PEN_YOLO_OBB.md)
- [단계 8 Top 펜 holdout·legacy 결과](docs/test-results/2026-08-02-top-pen-holdout-legacy-baseline.md)
- [단계 9 Policy ONNX 배포 번들 계약](docs/checklists/STAGE9_POLICY_DEPLOYMENT_BUNDLE.md)
- [Motion-1 연속 buffered trajectory 계약](docs/checklists/MOTION_BUFFERED_TRAJECTORY_CONTRACT.md)
- [Motion-2 STM32 buffered queue 후보](docs/checklists/MOTION_STM32_BUFFERED_QUEUE.md)
- [Motion-3 G474 buffered command route·timing 계약](docs/checklists/MOTION_BUFFERED_COMMAND_ROUTE_TIMING.md)
- [Motion-4 G474 buffered validation runtime route](docs/checklists/MOTION_G474_BUFFERED_VALIDATION_ROUTE.md)
- [Motion-4 로컬 검증 결과](docs/test-results/2026-08-02-motion4-g474-buffered-validation-route.md)
- [단계 7 물리 범위 재검증·배포 결과](docs/test-results/2026-07-30-physical-range-revalidation.md)
- [단계 7 Shoulder 근본 원인과 0x00020E00 후보](docs/test-results/2026-07-30-stage7-shoulder-root-cause-remediation.md)
- [단계 7 감독형 실제 Pick/Place 1회 완주](docs/test-results/2026-07-31-stage7-supervised-pick-place-complete.md)
- [0x00020E00 물리 거절: heartbeat RX starvation](docs/test-results/2026-07-31-stm32-0x00020e00-rejected-heartbeat-rx.md)
- [0x00020F00 acknowledged-heartbeat 후보와 물리 거절](docs/test-results/2026-07-31-stm32-0x00020f00-heartbeat-ack-candidate.md)
- [0x00021000 interrupt-buffered cooperative-motion 로컬 후보](docs/test-results/2026-07-31-stm32-0x00021000-interrupt-buffered-cooperative-motion-candidate.md)
- [eye-to-hand 보정 세션 정리 기록](docs/test-results/2026-07-30-top-eye-to-hand-session-cleanup.md)
- [로컬 하드웨어 설정](docs/LOCAL_HARDWARE_CONFIG.md)
- [제3자 license 고지](THIRD_PARTY_NOTICES.md)

## 저장소 구조

```text
Manipulation/
├── docs/
├── protocol/
├── firmware/stm32_actuator/          # 플랫폼 독립 C core
├── firmware/stm32_g474_single_arm/   # CubeIDE board project
├── ros2_ws/src/single_arm_bridge/    # Pi binary transport와 ROS 2 bridge
├── ros2_ws/src/so101_description/    # 왼팔 URDF/Xacro와 mesh
├── ros2_ws/src/so101_moveit_config/  # SRDF, planning, controller contract
├── ros2_ws/src/so101_bringup/        # mock/Isaac/STM32 통합 launch
├── ros2_ws/src/so101_isaac_bridge/   # MoveIt ↔ Isaac adapter
├── ros2_ws/src/manipulation_camera_manager/ # V4L2 capture와 phase scheduler
├── isaac_sim/assets/                 # 검증된 Isaac Sim 6.0.1 stage
├── config/
├── hardware/
├── tests/
├── tools/
└── requirements-host.txt
```

Isaac Sim/Isaac Lab 학습과 평가는 데스크탑에서 수행하고, 검증된 policy만
ONNX deployment bundle로 Raspberry Pi 5에 배포한다. 실제 policy의 입력,
출력과 `control_dt`는 Pi 자원 기준선에서 동결한다. 현재 `isaac_sim/`은
단계 4에서 검증한 왼팔 simulation asset을 포함하며, 오른팔은 단독 동등성
gate 뒤에 통합한다.

## 자동 판정

```bash
python3 -m unittest discover -s tests -v
python3 tools/validate_protocol_manifest.py
python3 tools/validate_camera_schedule.py
```

Pi에서 ROS package까지 확인할 때는 다음을 추가로 실행한다.

```bash
cd ~/Manipulation/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## License

자체 작성 코드는 [Apache License 2.0](LICENSE)으로 공개한다. STM32 HAL, CMSIS와 BSP는 각 원본 파일 및 [제3자 license 고지](THIRD_PARTY_NOTICES.md)에 적힌 조건을 따른다.
