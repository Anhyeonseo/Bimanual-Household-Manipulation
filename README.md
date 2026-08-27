# 양팔 정사각형 수건 접기 시스템

Raspberry Pi 5, ROS 2 Jazzy, STM32G474, SO-ARM101 두 대와 상단·손목
카메라를 이용해 **구겨진 300×300 mm 정사각형 수건 한 장을 펼치고 두 번
접는** 양팔 가정용 조작 시스템이다.

이 저장소의 최종 목표는 임의로 구겨져 놓인 수건을 양팔로 평탄화한 뒤,
서로 직교하는 두 중심선을 따라 접어 원래 넓이의 1/4인 정사각형으로 만드는
것이다. 이전 펜 연속 동작과 단일 팔 데모는
[Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)에
동결돼 있으며 이 저장소의 개발 범위가 아니다.

## 최종 동작

```text
초기 관측
  → 구김 상태와 노출 grasp 후보 추정
  → 들어 올리기·장력 펼치기·제한된 털기·모서리 당기기
  → 네 모서리와 평탄도 검증
  → 작업대 축 정렬
  → 첫 번째 반 접기
  → 중간 형상 검증
  → 직교 방향 두 번째 반 접기
  → 최종 정사각형 검증
```

수건은 변형체이므로 한 번 계산한 pose를 끝까지 사용하지 않는다. 각 조작
primitive가 끝날 때마다 상단·손목 카메라로 상태를 다시 추정하고, 신뢰도나
기하 조건이 부족하면 다음 동작을 승인하지 않는다.

## 목표 범위

- nominal 300×300 mm인 목표 정사각형 수건 한 장
- 작업대 안에 완전히 들어온 임의의 구김 상태
- 구김 해소, 네 모서리 복원, 평탄화와 축 정렬
- 서로 직교하는 중심선을 따른 두 번의 반 접기
- 단계별 관측 검증과 횟수가 제한된 복구
- perception, 계획, 명령, measured feedback, 결과 artifact 저장

매듭이 생긴 수건, 다른 물체 아래에 깔린 수건, 여러 장이 겹친 상태와 작업대
밖에서 시작한 상태는 초기 버전의 범위 밖이다. 자세한 계약은
[프로젝트 범위](docs/SCOPE.md)와 [수건 접기 설계](docs/TOWEL_FOLDING.md)를
따른다.

## 현재 상태

양팔 resident 제어, protocol v2, operational limits, URDF/MoveIt, 멀티카메라
수집과 보정 도구는 구현돼 있다. 캔 OBB와 파지 계획 코드는 강체 물체 단계에서
만든 선행 실험으로 유지하지만 최종 태스크는 아니다.

실제 영상 segmentation과 모서리 복원, 펼치기 primitive, 두 단계 fold
executor는 아직 구현되지 않았다. 다만 annotation→metric observation 변환,
구김 상태 추정, 유한 상태기계, 직교 2회 접기 기하·반원 arc와 offline replay는
구현됐다. 현재 상태는 `SOFTWARE_FOUNDATION`이며 수건 동작은 승인되지 않았다.

정확한 구현 상태는 [현재 상태](docs/CURRENT_STATUS.md), 개발 순서는
[최종 로드맵](docs/ROADMAP.md), 승인 기준은
[검증 매트릭스](docs/VERIFICATION_MATRIX.md)를 따른다.

## 빠른 확인

```powershell
py -3.11 -m venv .venv-host
.\.venv-host\Scripts\Activate.ps1
python -m pip install -r requirements/host.txt
python -m pytest -c config/pytest.ini --rootdir=. -q `
  tests/test_towel_geometry.py `
  tests/test_towel_fold_path.py `
  tests/test_towel_fake_reachability.py `
  tests/test_towel_dataset.py `
  tests/test_towel_perception.py `
  tests/test_towel_task_runtime.py `
  tests/test_towel_task_planning.py `
  tests/test_towel_task_replay.py `
  tests/test_towel_schemas.py `
  tests/test_desk_task_runtime.py `
  tests/test_can_grasp_roll_branches.py `
  tests/test_can_pick_application.py
python tools\run\validate_protocol_manifest.py
python tools\run\validate_camera_schedule.py
python tools\run\validate_towel_contract.py
python tools\run\validate_towel_schemas.py
python tools\run\select_towel_fake_reachability.py `
  config/towel_fake_reachability.example.json `
  --output tmp/towel_fake_reachability.json
python tools\run\validate_towel_dataset.py `
  config/towel_annotation.example.json `
  --output tmp/towel_dataset_manifest.json
python tools\run\plan_towel_task_once.py `
  config/towel_observation.example.json `
  --output tmp/towel_plan_example.json
python tools\run\replay_towel_task.py `
  config/towel_replay.example.json `
  --output tmp/towel_replay_example.json
```

위 시험은 현재 공통 기반, 수건 순수 기하·상태·plan-only 계약과 선행 기하
코드의 회귀 확인이다. 전체 firmware/ROS/MoveIt 연동 시험은 ROS 2 Jazzy,
OpenCV, xacro, ARM toolchain과 workspace overlay가 준비된 Linux/Pi 환경에서
실행한다.

## 저장소 구조

```text
config/                         # 운용 한계, 카메라와 motion-locked 수건 task 계약
docs/                           # 범위, 설계, 현재 상태, 로드맵, 검증 기준
firmware/                       # STM32 12축 resident 제어 기반
hardware/                       # 배선과 하드웨어 자료
isaac_sim/                      # 양팔 workcell과 simulation 자산
protocol/                       # Pi↔STM32 protocol v2
requirements/                   # host와 Pi perception Python 의존성
ros2_ws/src/
  manipulation_camera_manager/ # 상단·손목 카메라 수집과 scheduling
  so101_top_perception/         # 상단 관측과 fail-closed gate
  single_arm_bridge/            # legacy 이름; 양팔 resident adapter 포함
  so101_description/            # 양팔 URDF/Xacro
  so101_moveit_config/          # 양팔 planning 설정
tests/                          # 공통 기반과 태스크 계약 회귀 시험
tools/                          # run/lib/setup/diagnostics/contract_evidence
```

`single_arm_bridge`와 `stm32_g474_single_arm` 이름은 배포·linked-resource
호환성을 위해 유지한다. 실제 태스크 motion의 승인 경로는 resident 양팔
adapter 하나로 제한한다.

## 핵심 문서

- [수건 접기 최종 설계](docs/TOWEL_FOLDING.md)
- [하드웨어 없는 개발 백로그](docs/HARDWARE_FREE_BACKLOG.md)
- [프로젝트 범위](docs/SCOPE.md)
- [시스템 구조](docs/ARCHITECTURE.md)
- [현재 상태](docs/CURRENT_STATUS.md)
- [최종 로드맵](docs/ROADMAP.md)
- [검증 매트릭스](docs/VERIFICATION_MATRIX.md)
- [선행 캔 파지 파이프라인](docs/CAN_TO_BIN.md)
- [도구 구조와 진입점](tools/README.md)
- [제3자 고지](docs/THIRD_PARTY_NOTICES.md)

## License

자체 작성 코드는 [Apache License 2.0](LICENSE)으로 공개한다. STM32 HAL,
CMSIS와 BSP는 각 원본 파일 및
[제3자 고지](docs/THIRD_PARTY_NOTICES.md)의 조건을 따른다.
