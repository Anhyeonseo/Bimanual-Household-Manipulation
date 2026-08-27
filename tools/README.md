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
| `setup/isaac/` | Isaac Sim workcell·preview 생성과 legacy rigid grasp 진단; towel Isaac Lab 학습기는 아직 없음 |
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
python tools/run/capture_towel_yolo_interactive.py --category 01_flat
python tools/run/capture_towel_yolo_interactive.py \
  --host pi@<PI_IP> --session 20260827_top_validation_01 \
  --split validation --category 01_flat
python tools/run/bootstrap_towel_segmentation_pilot.py \
  datasets/towel_yolo_source/20260826_top_01 --per-category 5
python tools/run/validate_towel_observation_burst.py \
  datasets/towel_yolo_source/20260827_top_lifecycle_validation_01
```

실제 모터를 움직일 수 있는 도구는 파일의 confirmation·전원 조건을 우회하지
않는다. 현재 승인 상태와 실행 전 gate는 `docs/CURRENT_STATUS.md`와
`docs/VERIFICATION_MATRIX.md`를 따른다.

수건의 motion-locked contract validator, annotation→metric observation,
유한 replay와 기하 plan-only 도구를 유지한다. R0의
`plan_towel_fold_sequence_once.py`는 검증된 로컬 calibration artifact와
등록 URDF를 fail-closed로 확인한 뒤 5-DOF task-pose IK, MoveIt segment와 dense
collision을 검사하며 publisher·controller·resident motion client를 만들지 않는다.
`towel_observation_lifecycle.py`는 R1의 clear-view freshness, settle, clear pose와
calibration/model/URDF identity를 검증하고 primitive 전후 관측 artifact를
연결한다. 이 모듈 자체에는 motion client나 실행 API가 없다.
`bootstrap_towel_segmentation_pilot.py`는 범주별 원본을 결정적으로 뽑아
review overlay, contract proposal과 LabelMe workspace를 `tmp/`에 만든다. 모든
non-empty 제안은 `AMBIGUOUS`, `training_labels_authorized=false`로 남아 사람
검수 전 학습에 사용할 수 없다. 검수 import는 명시적 confirmation 뒤 segmentation
label만 승인하며 state label은 승인하지 않는다. robot-occluded 이미지는 단일
polygon을 만들지 않고 clear-view rejection 전용으로 보존한다. lifecycle의 필수
게이트는 정밀 robot pixel mask가 아니라 승인 clear pose, settle, fresh frame과
보수적 clear-view validity다.
`capture_towel_yolo_interactive.py --frames-per-episode 3`은 한 번의 물리 재배치
확인 뒤 같은 `episode_id`로 settled frame burst를 기록한다. 기본값 1은 기존
per-frame held-out protocol과 호환된다. `towel_perception.py`의 image backend는
고정 파란 수건 존재 gate, GrabCut mask, 3 px border evidence와 outline topology를
계산하며 raw pixel을 `K/D/P`로 보정한 뒤 table homography에 넣는다. RGB 면적만으로
hidden layer나 fold count를 승인하지 않는다.
`validate_towel_observation_burst.py`는 실제 5개 배치×3프레임의 SHA, frame 순서,
presence, clear-view와 feature spread를 재검사한다. fold count는 검증된 action
context에서만 주입하고, 비그립 봉제 고리는 segmentation에 유지한 채 20 mm 이하
폭만 metric fold-body outline에서 제외한다.
`diagnose_towel_fold_kinematics.py`는 입력 evidence가 아직 없을 때 canonical
1·2차 fold의 full-FK pose 해만 별도 JSON으로 기록한다. 이 결과는 MoveIt 경로와
충돌을 승인하지 않는다. `visualize_towel_fold_sequence.py`는 두 결과 형식을 모두
RViz marker로 표시하며, full-FK-only 관절 animation은 명시적 opt-in일 때만
publish한다.
새로운 학습 inference backend와 실제 executor는 `docs/ROADMAP.md`의 해당 gate가
시작될 때 추가한다. 현재 image/annotation perception backend는 R1 범위에서
검증 완료됐으며, 남은 순서는 `docs/HARDWARE_FREE_BACKLOG.md`를 따른다.
