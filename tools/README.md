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
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:. \
python3 tools/run/diagnose_towel_suspended_gravity_fold_kinematics.py \
  --output tmp/towel_first_fold_surface_drag_full_fk.json
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s1_vertex_patch_lift.py \
  tmp/towel_isaac_s0_manifest.json \
  --output tmp/towel_first_fold_surface_drag_gui.json \
  --place-release --self-contact \
  --physics-backend newton-coupled-vbd \
  --kinematic-replay tmp/towel_first_fold_surface_drag_full_fk.json \
  --urdf-override artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf \
  --actual-jaw-mesh-contact --newton-rubber-friction 100 \
  --environment-count 1 --disable-cubric-visual-sync \
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
`run_towel_s1_surface_drop_settle.py`는 S1의 독립 drop/settle smoke다.
최종 1차 접기 경로는 아래 세 구성으로 단순화했다.

- `lib/towel_suspended_gravity_fold_planning.py`: 완전 들기, 자유단 착지,
  36 mm 표면 드래그, L 형성, 15 mm 선보정과 중력 laydown task pose
- `run/diagnose_towel_suspended_gravity_fold_kinematics.py`: 52 phase full-FK,
  관절 한계와 approach tilt의 motion-free 검사
- `setup/isaac/run_towel_s1_vertex_patch_lift.py`: CoupledMJWarp+VBD actual-jaw
  contact, self-contact, hold-until-Q0-open retention, release와 최종 형상 gate

실측값은 `config/towel_isaac_s1_material.json`, Q0와 jaw 형상은
`config/so101_gripper_geometry.candidate.json`, 수직 접촉 기준은
`config/towel_first_fold_vertical_contact.candidate.json`에 있다.
`run_towel_s1_material_calibration.py`와
`run_towel_newton_material_calibration.py`는 물성 보정 재현용이다.

최종 3회 실행의 최악값은 layer `51.609/48.391`, paired-vertex p95
`16.398 mm`, 높이 `26.488 mm`, 폭 `156.332 mm`, terminal Z/curl 0이다.
독립 실행 간 전체 1,024-node 최대 차이는 `0.0116 mm`다.
근접 fallback과 최종 vertex attachment는 사용하지 않으며 실제 모터 명령은 0이다.
11.3 mm 미세보정과 이전 residual/seam/regrasp 독립 실험 스택은 개선이 없어 제거했다.

실행 명령은 [Isaac Sim README](../isaac_sim/README.md)에 유지한다. R1의 실제
image/annotation perception은 완료 상태이며, 이후 순서는
`docs/HARDWARE_FREE_BACKLOG.md`와 `docs/ROADMAP.md`를 따른다.
