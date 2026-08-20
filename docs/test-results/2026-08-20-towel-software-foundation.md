# 수건 software foundation 검증

검증일: 2026-08-20  
범위: 실제 카메라·로봇·MoveIt을 사용하지 않는 계약, 데이터, 기하, 상태기계,
plan-only와 fixture backend

## 통과 결과

- 수건 전용 시험: **67 passed**
- 수건 + 기존 비하드웨어 핵심 회귀: **113 passed, 29 skipped**
- protocol manifest: 30개 고유 message 통과
- camera schedule: 9개 phase 통과
- motion-locked towel contract: 미측정 hardware limit 10개가 모두 `null`
- JSON Schema: 2개 schema와 repository example 6개 통과
- dataset example: annotation 1개와 deterministic manifest SHA 통과
- aligned plan: `ALIGNED → FOLD_FIRST`, motion command 0개
- nominal replay: terminal `COMPLETE`, motion command 0개
- fake reachability: `x-pos-left-first` 결정적 선택, motion command 0개
- 생성한 plan/replay/fake artifact 모두 `motion_authorized=false`,
  `motion_commands=0`, `execution_api_used=false`
- Markdown local link와 `git diff --check` 통과

## 의도적으로 통과시키지 않은 항목

전체 test collection은 현재 Windows Python 환경에 OpenCV, pyserial과 ROS
`ament_index_python`이 없어서 실행하지 못했다. 또한 일부 기존 시험은 Windows
기본 CP949 대신 UTF-8 환경을 요구한다. 위 핵심 회귀는 `PYTHONUTF8=1`에서
실행했다.

Phase 0 hardware gate는 138개 실측 필드가 비어 있어 `FAIL`이다. 이는 software
foundation에서 채우거나 우회할 값이 아니다. 실제 수건 규격, 전원·servo,
접촉, 장력, 양팔 속도와 camera/worktable 보정을 감독 하에 측정하기 전까지
motion 승격은 금지한다.

## 재현 명령

```powershell
$env:PYTHONUTF8 = "1"
$towelTestFiles = Get-ChildItem tests\test_towel_*.py | ForEach-Object FullName
python -m pytest -c config/pytest.ini --rootdir=. -q $towelTestFiles
python tools\run\validate_towel_contract.py
python tools\run\validate_towel_schemas.py
python tools\run\validate_towel_dataset.py config\towel_annotation.example.json
python tools\run\plan_towel_task_once.py config\towel_observation.example.json --output tmp\towel_plan.json
python tools\run\replay_towel_task.py config\towel_replay.example.json --output tmp\towel_replay.json
python tools\run\select_towel_fake_reachability.py config\towel_fake_reachability.example.json --output tmp\towel_fake_reachability.json
```
