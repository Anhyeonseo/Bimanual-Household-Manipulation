# 양팔 책상 정리 시스템

Raspberry Pi 5, ROS 2 Jazzy, STM32G474, SO-ARM101 두 대와 상단·손목
카메라를 이용해 책상 위 물체를 인식하고 지정된 수납 위치로 옮기는 시스템이다.

이 저장소는 이제 **책상 정리 태스크만** 다룬다. 이전의 펜 연속 동작과
단일 팔 데모는 [Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)에
동결돼 있으며 여기서는 개발하거나 회귀 대상으로 유지하지 않는다.

## 목표 동작

```text
책상 스캔 → 물체 목록/자세 추정 → 목적지 규칙 적용 → 팔/경로 선택
          → 파지 → 운반/배치 → 결과 재인식 → 다음 물체
```

- 상단 카메라가 작업대의 물체 종류, 중심, 장축 방향을 추정한다.
- 상위 task manager가 물체별 목적지와 처리 순서를 결정한다.
- MoveIt이 도달성·관절 한계·충돌을 검사한다.
- Pi의 resident 양팔 adapter만 STM32 serial을 소유한다.
- STM32가 12축 동기 출력, tracking, watchdog, coordinated stop을 담당한다.
- 실패한 단계는 자동으로 실제 동작을 재시도하지 않고 안전 정지한다.

## 현재 상태

- 캔 OBB 데이터 준비·검증 도구와 캔 파지 기하/roll 분기 solver가 있다.
- 현재 실행 후보는 **왼팔 캔 pick-only**다. 쓰레기통 place와 다물체 정리는
  아직 구현되지 않았다.
- jaw gap↔command 실측, 접촉 잔차, 허용 접근 기울기가 비어 있어 실제 캔
  파지는 승인되지 않았다.
- 1280×960 상단 카메라 intrinsic은 독립 검증을 통과했지만 2026-08-18
  eye-to-hand 후보가 거부됐다. 따라서 좌표 변환 기반 실제 동작은 계속
  `motion_authorized=false`다.

정확한 재개 지점은 [현재 상태](docs/CURRENT_STATUS.md), 단계별 목표는
[로드맵](docs/ROADMAP.md)을 따른다.

## 빠른 확인

```powershell
py -3.11 -m venv .venv-host
.\.venv-host\Scripts\Activate.ps1
python -m pip install -r requirements/host.txt
python -m pytest -c config/pytest.ini --rootdir=. -q `
  tests/test_desk_task_runtime.py `
  tests/test_can_grasp_roll_branches.py `
  tests/test_can_jaw_gap_map.py `
  tests/test_can_pick_application.py `
  tests/test_can_pick_left_executor.py `
  tests/test_can_pick_left_plan_steps.py
python tools\run\validate_protocol_manifest.py
python tools\run\validate_camera_schedule.py
```

전체 firmware/ROS 회귀와 MoveIt 연동 시험은 ROS 2 Jazzy, OpenCV, xacro,
ARM toolchain과 workspace overlay가 준비된 Linux/Pi 환경에서 실행한다. 실제
모터 전원은 [검증 매트릭스](docs/VERIFICATION_MATRIX.md)의 선행 gate를 모두
통과한 뒤에만 켠다.

## 저장소 구조

```text
config/                         # 책상 정리 계약과 승인된 운용 한계
docs/                           # 범위, 구조, 현재 상태, 로드맵, 검증 기준
firmware/                       # STM32 12축 resident 제어 기반
protocol/                       # Pi↔STM32 protocol v2
ros2_ws/src/
  manipulation_camera_manager/ # 멀티카메라 수집과 scheduling
  so101_top_perception/         # 상단 물체 관측과 fail-closed gate
  single_arm_bridge/            # 기존 package명; 양팔 resident adapter 포함
  so101_description/            # 양팔 URDF/Xacro
  so101_moveit_config/          # 양팔 planning 설정
tests/                          # 유지 중인 기반·책상 정리 회귀 시험
tools/                          # 역할별 run/lib/setup/diagnostics/contract_evidence
requirements/                   # host와 Pi perception Python 의존성
```

`single_arm_bridge`라는 ROS package명과 `stm32_g474_single_arm` 디렉터리명은
배포·linked-resource 호환성을 위해 유지한다. 책상 정리 motion의 승인 경로는
그 안의 `bimanual_stream_adapter`뿐이다.

## 핵심 문서

- [프로젝트 범위](docs/SCOPE.md)
- [시스템 구조](docs/ARCHITECTURE.md)
- [현재 상태](docs/CURRENT_STATUS.md)
- [캔 → 수거함 파이프라인](docs/CAN_TO_BIN.md)
- [로드맵](docs/ROADMAP.md)
- [검증 매트릭스](docs/VERIFICATION_MATRIX.md)
- [도구 구조와 진입점](tools/README.md)
- [제3자 고지](docs/THIRD_PARTY_NOTICES.md)
- [상단 카메라 재보정 결과](docs/test-results/2026-08-18-top-camera-recalibration.md)

## License

자체 작성 코드는 [Apache License 2.0](LICENSE)으로 공개한다. STM32 HAL,
CMSIS와 BSP는 각 원본 파일 및 [제3자 고지](docs/THIRD_PARTY_NOTICES.md)의 조건을
따른다.
