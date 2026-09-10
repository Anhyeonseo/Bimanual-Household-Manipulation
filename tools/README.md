# 검증 도구

이 디렉터리는 펌웨어 검사에 필요한 도구만 유지한다. 가사 작업 계획은 `ros2_ws/src/home_robot_tasks`에서 관리한다.

| 진입점 | 역할 |
|---|---|
| `run/validate_protocol_manifest.py` | 기존 펌웨어 메시지 ID·필수 명령 검사 |
| `setup/firmware/generate_protocol_header.py --check` | manifest와 생성된 C 헤더 일치 검사 |
| `setup/firmware/validate_phase0.py` | 수동 기록한 하드웨어 기본 측정값 검사 |
| `lib/actuator_protocol.py` | 보존 펌웨어의 v1 codec 단위 시험 지원 |

`hardware/phase0_baseline.json`은 빈 측정 양식이며 검증 완료 자료가 아니다. 예전 접기·캔 집기·카메라 보정·실물 자세 반복 도구는 현재 실행 경로에서 제거했다.
