# Tools

정사각형 수건 접기 시스템의 실행·설정·검증 도구를 역할별로 분리한다. 새 도구는 성격에
맞는 하위 폴더에만 추가하고 `tools/` 바로 아래에는 Python 스크립트를 두지 않는다.

| 경로 | 용도 |
|---|---|
| `run/` | 사람이 직접 호출하는 수건 태스크와 저장소 검증 진입점 |
| `lib/` | 여러 진입점이 공유하는 수건 기하, 계획, runtime, protocol 코드 |
| `setup/camera_calibration/` | 카메라 표적 생성, 캡처, 보정, 보정 상태 모니터링 |
| `setup/can_perception/` | 선행 캔 OBB 데이터 준비·검증과 gripper 실측 |
| `setup/firmware/` | protocol header 생성과 초기 firmware gate |
| `setup/isaac/` | Isaac Sim workcell·preview 자산 생성 |
| `setup/resident_gate/` | resident 양팔 adapter의 무동작·제한 동작 승인 gate |
| `diagnostics/` | read-only 관측 또는 명시적으로 제한된 진단 |
| `contract_evidence/` | STM32/양팔 계약의 재현 가능한 증빙 수집 |

대표 명령은 저장소 루트에서 실행한다.

```powershell
python tools/run/validate_protocol_manifest.py
python tools/run/validate_camera_schedule.py
python tools/run/validate_towel_contract.py
python tools/run/validate_towel_schemas.py
python tools/run/select_towel_fake_reachability.py config/towel_fake_reachability.example.json --output tmp/towel_fake_reachability.json
python tools/run/validate_towel_dataset.py config/towel_annotation.example.json --output tmp/towel_dataset_manifest.json
python tools/run/plan_towel_task_once.py config/towel_observation.example.json --output tmp/towel_plan.json
python tools/run/replay_towel_task.py config/towel_replay.example.json --output tmp/towel_replay.json
```

실제 모터를 움직일 수 있는 도구는 파일의 confirmation·전원 조건을 우회하지
않는다. 현재 승인 상태와 실행 전 gate는 `docs/CURRENT_STATUS.md`와
`docs/VERIFICATION_MATRIX.md`를 따른다.

수건의 motion-locked contract validator, annotation→metric observation,
유한 replay와 동기 반원 fold arc를 포함한 기하 plan-only 도구는 구현됐다.
perception backend와 executor는 `docs/ROADMAP.md`의 해당 gate가 시작될 때
추가한다. 남은 순서는 `docs/HARDWARE_FREE_BACKLOG.md`를 따른다.
