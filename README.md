# ALOHA Mini Home Robot

ALOHA Mini 1 기반의 자취방·소형 주거공간용 모바일 매니퓰레이터다. 첫 목표는 **“거실 소파에서 리모컨을 찾아 침대에 내려놓기”**를 반복해서 수행하는 것이다.

기존 [Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)의 Classical P&P를 재사용한다. 해당 기반은 상단 카메라 인식부터 왼팔 집기, 오른팔 전달, 내려놓기까지 실기 완료한 시스템이다. 모바일 베이스와 리프트로 팔이 작업하기 좋은 상대 위치·높이를 재현하고, 새 장착 조건에 맞춰 통합한다.

## 개발 방향

- **우선 Classical 완성:** Nav2 이동, RGB-D 인식·미세 정렬, 리프트, 기존 P&P, 작업 상태 머신을 단계적으로 연결한다.
- **이후 VLA 확장·비교:** 리더암 시연으로 영상·관절 상태·동작 데이터를 수집하고 VLA를 모방학습한다. 같은 작업 조건에서 Classical과 성능을 비교한다.
- 초기 VLA의 제어 범위는 정지한 베이스·리프트 위의 팔과 그리퍼다. 주행과 리프트는 공통 실행 계층에 남긴다.
- 알려진 평탄한 실내와 가벼운 강체부터 시작한다. 여러 가사 작업과 변형체는 후속 확장이다.

## 대표 동작

```text
요청 → 소파 근처 주행 → 정지·리모컨 인식 → 베이스 미세 정렬·정지
     → 리프트 조절·정지·재인식 → 집기·집기 확인 → 운반 자세
     → 침대 근처 주행 → 정지·놓을 면 확인 → 필요 시 미세 정렬·정지
     → 리프트 조절·정지·재인식 → 놓기·배달 확인
```

Nav2 도착만으로 집기를 시작하지 않는다. 베이스·리프트·팔 조작은 순차 실행하며, 주행 중에는 팔과 리프트를 검증된 운반 상태로 유지한다. 양팔 플랫폼이지만 모든 물건을 양팔로 집거나 손 사이에서 전달할 필요는 없다.

## 현재 상태

현재 저장소에는 STM32 팔 펌웨어, `so101_arm_bridge`, 팔 참조 모델, `home_robot_tasks`의 요청 검증·7단계 계획 출력이 있다. 기존 P&P 기반은 별도 저장소에 있으며 모바일 환경으로의 이식은 아직 진행 전이다. 베이스·Nav2·RGB-D·리프트·전체 실행기는 아직 연결되지 않았다.

플랫폼 기준은 ALOHA Mini 1의 3륜 옴니 베이스와 수직 리프트다. 실제 바퀴·리프트 프로토콜, 피드백, 장착 치수는 확인이 필요하다. 센서는 작업용 RGB-D, 주행용 2D LiDAR와 IMU를 구성 방향으로 두고 제품·설치 위치·컴퓨팅 배치를 선정한다. 현재 최우선은 **베이스 저수준 제어·피드백 확인과 ROS 2/Nav2 연결 가능성 검증**이다.

## 하드웨어 없는 확인

저장소 루트에서 실행한다. 아래 명령은 모터나 시뮬레이션을 실행하지 않는다.

```bash
python3 -m venv .venv-host
source .venv-host/bin/activate
python -m pip install -r requirements/host.txt
python -m pytest -c config/pytest.ini --rootdir=. -q
python tools/run/validate_protocol_manifest.py
python tools/setup/firmware/generate_protocol_header.py --check
```

예제 요청의 계획 출력:

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python -m home_robot_tasks.cli \
  --world config/home.example.json \
  --request config/fetch_remote.example.json
```

결과는 `plan_only`, `executable=false`다. 예제는 소파 탐색과 침대의 지정 영역을 의미 이름으로 표현한다. 실제 지도 좌표가 없고, 현재 7단계 출력에는 미세 정렬·리프트·재인식 상태가 아직 분리되어 있지 않다. 손에 든 채 운반하는 대표 경로의 자세·적재 한계는 실물 검증이 필요하다.

펌웨어 공통 코어 시험:

```bash
cmake -S firmware/stm32_actuator -B build/stm32_actuator-host
cmake --build build/stm32_actuator-host
ctest --test-dir build/stm32_actuator-host --output-on-failure
```

## 구성과 문서

| 경로 | 역할 |
|---|---|
| `firmware/` | 기존 STM32 제어기와 독립 C 코어 |
| `ros2_ws/src/so101_arm_bridge/` | 팔 명령·피드백·정지 브릿지 |
| `ros2_ws/src/so101_interfaces/` | 양팔 명령·피드백 ROS 메시지 |
| `ros2_ws/src/so101_description/` | SO101 팔 기구학과 형상 |
| `ros2_ws/src/home_robot_tasks/` | 가사 작업 요청과 계획 |
| `config/`, `hardware/` | 관절 한계, 예제 요청, 하드웨어 참조 |
| `protocol/`, `tools/`, `tests/` | 통신 규격, 검증 도구, 회귀 시험 |

[현재 상태](docs/CURRENT_STATUS.md) · [로드맵](docs/ROADMAP.md) · [시스템 구조](docs/ARCHITECTURE.md) · [팔 브릿지 사용](ros2_ws/src/so101_arm_bridge/README.md)

이전 수건 접기 개발은 [SO101-Towel-Folding](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80)에 보관한다.

## License

자체 작성 코드와 문서는 [Apache License 2.0](LICENSE)을 따른다. 로봇 모델과 STM32 구성 요소의 조건은 [제3자 고지](docs/THIRD_PARTY_NOTICES.md)에 정리했다.
