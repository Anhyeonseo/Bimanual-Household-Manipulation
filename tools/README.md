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
| `setup/isaac/` | Isaac Sim workcell·preview, towel S0 rigid-proxy gates와 S1 surface-cloth drop/settle smoke; 학습기는 아직 없음 |
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
python tools/run/plan_towel_fold_sequence_once.py --plan-only \
  --output tmp/towel_fold_sequence_strict_r0g_20260828.json
python tools/run/build_towel_isaac_s0_manifest.py \
  tmp/towel_fold_sequence_strict_r0g_20260828.json \
  --environments 8 --seed 42 \
  --output tmp/towel_isaac_s0_manifest.json
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_vectorized_reset.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_vectorized_reset.json --device cuda:0
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_articulation_replay.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_replay.json --device cuda:0 --viz none
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_articulation_replay.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_replay_gui.json --device cuda:0 \
  --phase-seconds 0.45 --keep-open --viz kit
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_camera_fov.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_fov.json --device cuda:0 --viz none
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_collision_replay.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_collision.json --device cpu --viz none
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s0_collision_replay.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s0_collision_gui.json --device cpu --viz kit \
  --stop-on-first-forbidden --keep-open
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s1_surface_drop_settle.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s1_surface_drop_settle.json \
  --device cuda:0 --viz none
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s1_vertex_patch_lift.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_isaac_s1_place_release_gui.json \
  --place-release --fold-phase-seconds 0.20 --settle-timeout-s 16 \
  --device cuda:0 --viz kit --keep-open
python tools/run/export_towel_yolo_segmentation.py \
  --output tmp/towel_yolo_segmentation
.venv-yolo/bin/python tools/run/bootstrap_towel_yolo_assisted_review.py \
  --device cpu
.venv-yolo/bin/python tools/run/evaluate_towel_yolo_segmentation.py \
  tmp/towel_yolo_runs/yolo26n_seg_r0/weights/best.pt \
  --output tmp/towel_yolo_segmentation_evaluation.json
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
`export_towel_yolo_segmentation.py`는 승인된 review manifest 네 개만 읽어 사람
검수 train 540장과 독립 validation 35장을 YOLO segmentation 형식으로 내보낸다.
기존 split을 그대로 보존하고 source SHA/capture 누수, 미검수 label, robot-occluded
training label과 review digest 불일치를 거절한다. empty frame은 빈 label 파일로
남기며 모든 polygon은 export 뒤 raster round-trip IoU `0.999` 이상을 요구한다.
출력의 `dataset.yaml`은 학습 입력이고 `export_manifest.json`은 원본 review·image·
label SHA를 기록한다. exporter 통과는 dataset 준비 완료일 뿐 학습 model이나
held-out inference 성능을 승인하지 않는다.
`evaluate_towel_yolo_segmentation.py`는 학습 weight SHA와 export manifest identity를
함께 기록하고 validation의 수건 검출, empty rejection과 fixed-threshold pixel mask
IoU를 계산한다. Ultralytics mAP만으로 runtime 승격하지 않으며 새 test session 없이
validation 결과를 최종 일반화 성능으로 해석하지 않는다.
`bootstrap_towel_yolo_assisted_review.py`는 기존 검수 train 103장을 제외한 개발
원본에 `best.pt` mask 초안을 만든다. 학습 후보 444장은 LabelMe workspace로,
robot-occluded 48장은 OOD overlay로 분리한다. confidence, 다중 검출, border와
OpenCV fallback을 이용해 high/medium/low 검수 순서를 만들지만 모든 출력은 명시적
LabelMe 확인 전 `training_labels_authorized=false`다.
현재 전수 검수 closeout은 444장 중 437장을 승인하고 상태가 나쁜 7장을 제외했으며,
robot-occluded 48장은 계속 OOD 전용으로 유지한다.
승인 train 540장으로 재학습한 expanded weight는 같은 validation에서 수건 30/30,
empty 5/5, mask IoU 평균 0.980166·최저 0.966108을 기록했다. weight와 학습 run은
대부분 Git ignore 대상이지만 팀 검증용 canonical
`artifacts/models/towel_yolo26n_seg_expanded_r1/best.pt` 하나는 SHA와 함께
버전 관리한다. 새 독립 test 전에는 runtime backend를 교체하지 않는다.
`diagnose_towel_fold_kinematics.py`는 입력 evidence가 아직 없을 때 canonical
1·2차 fold의 full-FK pose 해만 별도 JSON으로 기록한다. 이 결과는 MoveIt 경로와
충돌을 승인하지 않는다. `visualize_towel_fold_sequence.py`는 두 결과 형식을 모두
RViz marker로 표시하며, full-FK-only 관절 animation은 명시적 opt-in일 때만
publish한다.
`build_towel_isaac_s0_manifest.py`는 motion-locked strict MoveIt JSON을 검증하고
300 mm rigid proxy, 검증 worktable, strict MoveIt/FCL mesh-contact 계약, 12축 clear
state와 원본 MoveIt trajectory를 결정적인 vectorized reset/replay batch로 만든다.
host 계약만 생성하며 Isaac stage, FOV, 접근, 충돌을
실행했다고 주장하지 않는다. 실제 S0 PASS는 후속 Isaac Lab runner가 같은 source와
reset/replay SHA를 소비해 simulator gate를 채운 뒤에만 가능하다.
`run_towel_s0_vectorized_reset.py`는 해당 manifest를 Isaac Lab/PhysX에서 소비해
300 mm rigid proxy와 table을 환경별로 clone하고 root pose를 한 번에 reset한다.
최신 아래→위·오른팔 strict artifact와 S0 manifest를 SHA로 고정한다. 8-env 최대
위치 오차 `7.45e-9 m`와 source/replay identity 일치를 확인했지만 로봇
articulation, phase replay, camera FOV와 collision gate를 단독으로 실행하지 않으므로
이 runner만으로 S0 전체 PASS를 주장하지 않는다.
`run_towel_s0_articulation_replay.py`는 manifest에 SHA가 고정된 최신 r0g URDF의
canonical 12축을 이름 순서대로 매핑하고 114개 phase를 모든 환경에 직접 재생한다. headless gate와
GUI 직접 확인이 같은 manifest를 사용한다. rigid proxy도 manifest의 작업대 pose로
명시 reset해 GUI와 headless 위치를 같게 유지한다. 이 단계는 중력과 robot collision을
끄는 기구학 replay이므로 상태는
`S0_ISAACLAB_ARTICULATION_REPLAY_PASS_COLLISION_NOT_RUN`이다. calibrated Top FOV는
별도 runner에서 PASS했다. transition collision runner는 Isaac PhysX에서 114 phase의
원본 MoveIt trajectory를 3,383개 표본으로 재생하고 self/table 금지 접촉 0으로 PASS했다.
rigid proxy는 cloth 변형·attachment를 증명하지 않으므로 S1 승격 근거로만 사용한다.
`run_towel_s1_surface_drop_settle.py`는 동일 manifest의 검증 작업대에 31×31
element/1,024-node surface mesh를 떨어뜨리는 첫 S1 smoke다. 기본 collision offset의
`20 mm` 공중 정착을 hover gate로 거부한 뒤 실제 shape offset을 고정했고, 8환경에서
40 step 이하 settle, clearance `1.5 mm`, 환경 간 최대 차이 `8.94e-8 m`로 PASS했다.
후보 질량·물성은
실측 전 값이며 이 drop runner만으로 attachment, lift/place/release, 접기와 학습을 승인하지 않는다.
`run_towel_s1_vertex_patch_lift.py`는 좌우 각각 16 mm 반경의 9-node patch를
`OmniPhysicsVtxXformAttachment`로 gripper rigid link 아래 attachment frame에 붙이고 r0g의 첫 fold 변위를
추종한다. 첫 단순 Xform target 결과는 PhysX의 non-rigid target 경고로 폐기했다.
direct articulation-link low-lift smoke는 snap `0.125 mm`, patch lift 최소
`20.871 mm`, gripper 추종 오차 최대 `0.782 mm`, 8-env patch 차이 `0.057 mm`로
PASS한 이전 후보 증거다. 최신 manifest의 `--place-release` 경로는 1차 16-sample
fold 뒤 attachment를 끄고 jaw open·retreat·settle까지 실행해 snap `0.046 mm`,
patch lift 최소 `20.933 mm`, 강체회전 포함 추종 오차 `0.426 mm`, release 뒤
patch–jaw 거리 최소 `63.974 mm`, 최종 clearance `1.500 mm`로 PASS했다. 자유 면
전체 8-env 차이 `31.648 mm`는 별도 미통과 진단값이며 self-contact·물성 보정은 미완료다.
`--self-contact`는 topology 두 칸을 제외한 비이웃 vertex 간격도 검사한다. `1/240 s`
최선 진단에서 최소 간격 `3.007 mm`와 table clearance `1.500 mm`는 유지했지만 20초 뒤
최대 속도 `0.0294 m/s`로 settle 한계 `0.015 m/s`를 넘어서 결과 파일을 생성하지 않았다.
새로운 학습 inference backend와 실제 executor는 `docs/ROADMAP.md`의 해당 gate가
시작될 때 추가한다. 현재 image/annotation perception backend는 R1 범위에서
검증 완료됐으며, 남은 순서는 `docs/HARDWARE_FREE_BACKLOG.md`를 따른다.
