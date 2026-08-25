# 구겨진 정사각형 수건 펼치기·2회 접기 설계

## 1. 목표

작업대에 임의로 구겨져 놓인 nominal 300×300 mm 정사각형 수건 한 장을
양팔로 완전히 펼친 뒤, 서로 직교하는 두 중심선을 따라 접어 nominal
150×150 mm 정사각형으로 만든다.

개발 단계에서는 perception, primitive, 첫 번째 fold처럼 일부만 구현할 수
있지만 최종 태스크 정의 자체를 단순한 평탄 수건 접기로 축소하지 않는다.

## 2. 입력과 결과 계약

### 입력

- nominal 300×300 mm인 한 장의 정사각형 수건
- side tolerance, 근사 두께, 재질·건조 상태는 task contract에 등록돼 있음
- 질량과 작업대 마찰은 해당 동적 primitive 전에 실측하며 미측정 동안 이를
  소비하는 drag, shake와 동역학 기반 명령은 비활성화함
- 수건 전체가 작업대와 상단 카메라 시야 안에서 시작함
- 초기 자세와 구김은 임의지만 매듭이나 외부 물체 얽힘은 없음
- 작업대에는 수건 조작을 방해하는 다른 물체가 없음

### 결과

- 첫 번째 중심선을 따라 nominal 300×150 mm로 반 접힌 직사각형
- 직교하는 두 번째 중심선을 따라 nominal 150×150 mm로 접힌 정사각형
- 최종 넓이는 펼친 수건의 약 1/4
- 목표 corner, outline, fold-line 품질 기준 통과
- 실행의 모든 입력·판정·명령·feedback·결과가 artifact로 저장됨

### 300 mm 기준 기하

| 단계 | nominal footprint | 면적 | 순수 기하 기준 |
|---|---:|---:|---|
| 펼침 | 300×300 mm | 0.0900 m² | 네 모서리와 전체 edge 노출 |
| 1차 접기 | 300×150 mm | 0.0450 m² | 이동 edge 파지점 간격 최대 약 300 mm |
| 2차 접기 | 150×150 mm | 0.0225 m² | 이동 edge 파지점 간격 최대 약 150 mm |

두 중심선 접기의 이상적인 corner 이동 거리는 각각 300 mm이고, 반원 arc의
반경과 최대 높이는 150 mm다. 실제 명령은 jaw가 모서리에서 안쪽으로 들어가는
grasp inset, 수건 처짐과 작업대 마찰을 반영하므로 이 수치를 그대로 복사하지
않는다. 다만 양팔 동시 IK, TCP separation, 카메라 FOV와 작업대 여유를
검증하는 최소 설계 envelope로 사용한다.

## 3. 전체 상태기계

```text
OBSERVE_INITIAL
  → COARSE_UNFOLD
  → REOBSERVE
  → CORNER_RECOVERY
  → FLATTEN
  → ALIGN
  → VERIFY_FLAT
  → FOLD_FIRST
  → VERIFY_FIRST
  → FOLD_SECOND
  → VERIFY_FINAL
  → COMPLETE
```

어느 단계에서든 관측이 stale하거나 confidence, workspace, collision, 장력
계약을 만족하지 못하면 다음 실제 동작을 승인하지 않는다. 복구 가능 횟수가
남아 있으면 해당 recovery state로 이동하고, 아니면 `FAILED`로 종료한다.

각 화살표의 실제 실행은 다음 공통 주기를 따른다.

```text
OBSERVE_CLEAR
  → PLAN_AND_VALIDATE
  → APPROACH_AND_GRASP_VERIFY
  → EXECUTE_BOUNDED_PRIMITIVE
  → RETREAT_AND_SETTLE
  → REOBSERVE_CLEAR
```

네 모서리, 평탄도와 fold 결과는 양팔이 지정된 clear pose로 물러난 관측에서만
승인한다. 조작 중 영상은 slip, workspace exit와 fault 감지에는 사용할 수 있지만
가려진 전체 cloth state를 새로 확정하는 근거로 쓰지 않는다.

## 4. Perception 계약

한 프레임의 OBB만으로는 구김과 layer topology를 판단할 수 없다. 각 관측은
최소한 다음 정보를 제공해야 한다.

| 출력 | 의미 |
|---|---|
| `segmentation_mask` | 카메라 영상의 전체 수건 영역 |
| `boundary` | 외곽 contour와 불연속·가림 후보 |
| `height_or_wrinkle_map` | 구김 높이 또는 RGB 다중 시점 기반 대체 feature |
| `corner_candidates` | 위치, 노출도, grasp 가능성, confidence |
| `corner_evidence_source` | visual, held TCP constraint 또는 unknown 구분 |
| `robot_occlusion_mask` | 팔·gripper가 가린 영역과 전체 가림 비율 |
| `layer_ambiguity` | grasp 후보가 단일 layer인지 확인되지 않은 정도 |
| `visible_area_ratio` | 등록된 전체 수건 면적 대비 현재 투영 면적 |
| `edge_and_diagonal_metrics` | 네 변과 두 대각선의 길이·편차 |
| `flatness_score` | 높이, 면적, 경계로 계산한 평탄도 |
| `state_label` | 구김, 부분 펼침, 평탄, 1차/2차 접힘 상태 |
| `source_stamp` | 프레임 timestamp와 calibration/bundle SHA |
| `settled` | release·퇴피 뒤 형상 진동이 허용 범위 안인지 여부 |

### 권장 관측 방식

상단 RGB-D가 있으면 높이와 layer 겹침 판정이 가장 직접적이다. 상단 RGB만
사용하는 경우에는 다음 절차로 부족한 깊이 정보를 보완한다.

1. 양 손목 카메라의 비스듬한 다중 시점 관측
2. 한 지점을 잡아 올린 뒤 실루엣을 다시 보는 `lift_and_observe`
3. gripper separation과 수건 윤곽 변화의 시간 이력
4. 낮은 confidence에서 동작을 거부하는 fail-closed gate

작업대 homography는 작업대 평면에 놓인 점에만 적용한다. `lift_and_observe`,
fold arc와 같이 수건이 들린 상태에서는 RGB-D, 검증된 다중 시점 또는
gripper에 붙은 조건부 TCP constraint가 없으면 3D corner를 만들지 않는다.

Top 영상의 팔 가림은 검증된 camera-to-base와 URDF로 robot mask를 투영해
분리한다. 가려진 픽셀을 cloth로 추정해 채우지 않는다. right wrist의 intrinsic,
torque-hold eye-in-hand와 optical frame는 R0에서 검증됐지만, 검증된 자세·화면
경계 밖으로 외삽하거나 손목 영상 하나만으로 들린 수건의 3D 점을 만들지 않는다.
left wrist의 고정 gripper 가림과 양쪽 wrist의 robot mask는 관측 confidence에
반영한다.

## 5. 수건 상태 표현

| 상태 | 최소 조건 |
|---|---|
| `CRUMPLED` | 면적 부족, 높은 주름 또는 모서리 식별 불가 |
| `PARTIALLY_OPEN` | 면적 증가, 일부 경계와 grasp 후보 확보 |
| `TWO_CORNERS_VISIBLE` | 서로 다른 두 모서리를 양팔이 접근 가능 |
| `FOUR_CORNERS_VISIBLE` | 네 모서리와 사각형 topology 후보 확보 |
| `FLAT_BUT_ROTATED` | 평탄도는 통과하지만 작업대 축과 정렬되지 않음 |
| `ALIGNED` | 평탄도·면적·변·대각선·축 정렬 기준 통과 |
| `FOLD_1_COMPLETE` | 목표 직사각형과 첫 접힘선 검증 통과 |
| `FOLD_2_COMPLETE` | 최종 정사각형 검증 통과 |

정사각형 수건은 모서리의 의미론적 ID가 없다. 장기간 A/B/C/D를 추적하기보다
각 관측에서 작업대 x/y축 기준으로 `top_left`, `top_right`, `bottom_left`,
`bottom_right`를 다시 부여한다.

## 6. 조작 primitive

모든 primitive는 공통으로 precondition, 계획 SHA, 최대 시간·거리·속도,
양팔 TCP separation, 접촉 또는 hold 조건, 중단 조건, terminal measured
feedback와 새 visual postcondition을 가진다. 한 팔에서 slip, tracking fault나
workspace 이탈이 발생하면 다른 팔도 같은 session에서 정지한다.

### `grasp_exposed_corner`

노출도와 접근 가능성이 높은 모서리 하나를 잡는다. cloth-only 영역인지,
접근 중 작업대와 충돌하지 않는지, jaw closing 뒤 접촉이 있는지를 확인한다.

### `grasp_two_corners`

두 팔이 서로 다른 모서리를 잡는다. grasp target 사이 거리, 팔 교차 여부와
양쪽 grasp timestamp 차이를 제한한다.

### `lift_and_observe`

수건 일부를 낮은 높이로 들어 중력이 겹침을 줄이게 하고, 상단·손목 카메라로
늘어진 윤곽을 다시 관측한다. 이 단계는 큰 이동이나 털기를 포함하지 않는다.

### `tension_spread`

양팔 간격을 천천히 늘려 수건을 펼친다. 최대 TCP separation, 속도 차이,
gripper tracking residual과 추정 장력을 제한한다.

### `controlled_shake`

겹친 layer를 분리하기 위한 작은 진폭의 제한 동작이다. 진폭, 주기, 반복
횟수를 contract에 고정하고 독립 primitive 시험을 통과하기 전에는 비활성화한다.

### `drag_corner`

말리거나 겹친 모서리 하나를 작업대 위에서 외곽 방향으로 당긴다. 수건 밖
영역을 긁거나 다른 모서리를 다시 접지 않도록 매 동작 뒤 재관측한다.

### `lay_flat`과 `align_square`

장력을 유지하며 내려놓고 네 모서리, 변, 대각선과 작업대 축을 기준으로 최종
평탄·정렬 상태를 만든다.

### `fold_edge_pair`

한쪽 변의 두 모서리를 동시에 잡아 중심선을 지나는 fold arc로 반대쪽에
정렬한다. 첫 번째 fold와 여러 겹을 잡는 두 번째 fold는 별도 contact
contract를 사용한다.

## 7. 거친 펼치기 전략

1. 가장 높은 노출 지점 또는 confidence가 높은 모서리를 한 팔이 잡는다.
2. 낮은 높이로 들어 올려 중력으로 겹침을 완화한다.
3. 늘어진 윤곽에서 반대쪽의 가장 먼 grasp 후보를 다시 계산한다.
4. 다른 팔이 두 번째 grasp를 확보한다.
5. 양팔 간격을 천천히 늘려 장력을 건다.
6. 필요하고 승인된 경우에만 제한된 `controlled_shake`를 실행한다.
7. 장력을 유지하며 작업대에 내려놓고 전체 상태를 다시 관측한다.

이 단계의 목표는 즉시 완전 평탄화가 아니라 정밀 corner recovery가 가능한
`PARTIALLY_OPEN` 또는 `TWO_CORNERS_VISIBLE` 상태에 도달하는 것이다.

초기에는 위 절차를 deterministic baseline으로 구현한다. 최종 시스템에서는
임의 구김과 self-occlusion 때문에 1·3·6번의 grasp/primitive 선택을 learned
policy가 담당한다. 정책은 관절 명령을 직접 내리지 않고 다음 후보를 출력한다.

- primitive 종류와 좌/우 또는 양팔 배정
- image/workcell상의 pick point와 필요하면 place point
- contract 안의 lift height, separation, 속도·반복 횟수
- confidence와 `abstain`

후보는 기존 workspace, collision, contact와 recovery gate를 모두 통과해야 한다.
실제 episode는 성공뿐 아니라 action 전후 visible area, corner/topology state,
slip, 시도 횟수와 실패 원인을 저장해 simulator randomization과 policy 학습에
되먹임한다.

## 8. 정밀 평탄화와 정렬

부분적으로 펼쳐진 수건에서 겹친 모서리와 말린 edge를 식별하고, 필요한
모서리만 `drag_corner`로 보정한다. 평탄화 완료 후보는 다음 조건을 만족한다.

- 네 모서리 모두 검출
- 네 변 길이 편차 8% 이하
- 두 대각선 길이 편차 8% 이하
- 예상 전체 면적 대비 관측 면적 90% 이상
- 높이·주름 기반 flatness threshold 통과
- 말린 edge, 내부 겹침과 다층 corner의 ambiguity가 승인 한계 이하
- robot occlusion을 제외한 clear observation에서 위 조건이 확인됨
- 네 모서리 모두 양팔의 승인 workspace 안에 있음

그 뒤 최소 회전 방향으로 작업대 x/y축에 맞춰 `ALIGNED` 상태를 만든다.

## 9. 첫 번째 반 접기

1. 한쪽 변의 두 모서리에 양팔 pregrasp를 배치한다.
2. 양쪽 contact를 확인하고 수직으로 들어 올린다.
3. 중심선을 지나는 동기 fold arc를 실행한다.
4. 이동 모서리를 반대편 두 모서리 위에 정렬한다.
5. 장력을 줄이며 내려놓고 동시에 release한다.
6. 새 직사각형 outline, 대응 모서리와 첫 접힘선을 검증한다.

중간 형상이 비틀렸거나 corner/fold-line 기준을 통과하지 못하면 두 번째
fold를 실행하지 않는다.

300 mm 수건의 nominal 목표는 300×150 mm다. moving edge의 두 grasp point는
corner inset 전 기준 약 300 mm 떨어지고, 각 점은 중심선 반경 약 150 mm의
경로를 지난다. 실제 path는 수직 lift, 장력 유지, 저속 lay-down과 동시 release
구간으로 나누며 순수 반원은 후보 생성의 기준으로만 사용한다.

## 10. 두 번째 반 접기

1차 fold 뒤에는 수건이 여러 겹이므로 새 외곽선을 다시 추정한다. 한쪽 짧은
변의 두 layer grasp point를 잡아 첫 접힘선과 직교하는 중심선으로 접는다.
첫 fold를 펴지 않도록 접근 높이와 fold arc를 별도로 검증한다.

release 뒤 필요하면 제한된 `release_and_smooth`를 실행하고 다음을 확인한다.

- 최종 대응 모서리 평균 오차 25 mm 이하
- 목표 정사각형 대비 외곽선 IoU 0.85 이상
- 두 접힘선 평균 위치 오차 20 mm 이하
- 수건 일부가 목표 stack 밖으로 과도하게 돌출되지 않음

nominal 목표는 150×150 mm다. 이 단계는 여러 겹을 함께 잡아야 하므로 1차
fold의 single-layer grasp와 별도 접촉 기준을 사용한다. 첫 접힘이 끌려 펴지거나
release 뒤 반발하는 경우에는 다음 task로 진행하지 않고 허용된 1회 placement
correction 또는 `FAILED`로 끝낸다.

## 11. 제한 복구

| 복구 | 최대 횟수 |
|---|---:|
| 모서리 재탐색 | 3회 |
| lift-and-unfold | 2회 |
| corner drag | 모서리당 2회 |
| fold placement 보정 | fold 단계당 1회 |

각 시도는 독립 plan과 confirmation, attempt counter, 전후 observation을
기록한다. 같은 실패 원인이 반복되거나 fault, stale calibration, workspace
이탈이 발생하면 남은 횟수와 관계없이 안전 정지한다.

## 12. Isaac Lab과 학습 사용 범위

현재 Isaac workcell은 표시 전용이므로 수건 mesh만 추가해서는 cloth 실험이
성립하지 않는다. 기존 stage를 보존하고 다음 네 단계의 별도 physics layer로
승격한다.

1. `S0 rigid proxy`: 300×300 mm 평판으로 FOV, 양팔 IK, collision과 두 fold
   envelope를 검증한다.
2. `S1 attached cloth`: 삼각 surface-deformable mesh와 작업대 collider를 만들고,
   파지된 vertex patch를 gripper frame에 명시적으로 attach해 경로·release
   순서를 검증한다.
3. `S2 contact/randomized cloth`: 질량, 두께, bend, damping과 dynamic friction을
   실측 범위에서 바꾸며 실패 사례와 perception 데이터를 생성한다.
4. `S3 Isaac Lab policy task`: S1/S2를 vectorized 환경으로 감싸고 제한된
   primitive action, 실제 카메라와 동형인 observation, progress/safety reward와
   유한 termination을 구현한다.

시뮬레이션의 solver, mesh, material, attachment와 random seed는 artifact에
기록한다. Isaac 결과는 경로·충돌·가림과 실패 사례 생성에는 사용할 수 있지만
실제 jaw force, 정지 마찰, fling 속도나 motion authorization의 근거로 사용하지
않는다.

### 학습 순서와 선택 기준

1. R3의 scripted primitive를 S1에서 replay해 contact·attachment·termination이
   맞는지 먼저 확인한다.
2. heuristic 펼치기 정책을 S2의 고정 seed suite에서 baseline으로 기록한다.
3. 같은 action/observation 계약으로 behavior cloning 또는 self-supervised
   policy와 RL을 학습한다.
4. simulator held-out material/shape/lighting에서 성공률, coverage, action 수,
   collision/drop을 비교한다.
5. 실제 수건에서는 supervised-once와 작은 고정 evaluation set으로 시작하고,
   sim-to-real gap이 크면 실제 replay buffer 기반 fine-tuning을 우선한다.

학습을 R5에서 처음 시작하지 않는다. R1 실제 데이터 수집부터 episode split과
action outcome을 보존하고, R2에서 Isaac Lab 환경·baseline·평가기를 만들며,
R3/R4의 검증된 primitive와 평탄 수건 접기 결과를 학습 action과 성공 판정의
ground truth로 재사용한다.

강화학습은 특히 임의 구김의 펼치기·정규화에 우선 적용한다. 평탄 수건의 두
번 접기는 기하 baseline을 유지하고, learned scorer가 필요할 때 grasp/placement
후보만 개선한다. 이는 학습을 축소하는 것이 아니라 탐색이 필요한 상태 선택과
정확한 실행·안전 검증을 분리해 실제 데이터 효율과 재현성을 높이기 위함이다.

이 구조는 임의 cloth를 learned bimanual primitive로 펼친
[FlingBot](https://proceedings.mlr.press/v164/ha22a.html), 정규화 뒤 단순한
downstream fold를 사용한 [Cloth Funnels](https://clothfunnels.cs.columbia.edu/),
학습된 grasp pair와 구조화된 primitive를 결합한
[SpeedFolding](https://pantor.github.io/speedfolding/), 한 시간의 실제
self-supervised 경험으로 goal-conditioned pick-and-place를 학습한
[Learning Arbitrary-Goal Fabric Folding](https://proceedings.mlr.press/v155/lee21a.html)의
공통 패턴을 따른다. Isaac Lab의 surface-deformable cloth 지원은 활용하되 현재
[VBD/Newton cloth 경로도 새 task마다 asset, material, contact와 coupling을
검증·튜닝해야 한다](https://isaac-sim.github.io/IsaacLab/develop/source/overview/core-concepts/physical-backends/newton/using-vbd-solver.html)는
공식 범위를 감안해 실제 validation을 없애지 않는다.

## 13. 최종 benchmark

서로 다른 초기 구김 상태 최소 30회를 사용한다. 성공뿐 아니라 각 단계의
조건부 성공률과 실패 원인을 함께 기록한다.

| 지표 | 목표 |
|---|---:|
| 전체 end-to-end 성공률 | 90% 이상 |
| 펼치기 성공률 | 95% 이상 |
| 첫 번째 fold 성공률 | 95% 이상 |
| 두 번째 fold 성공률 | 95% 이상 |
| 충돌·비명령 동작 | 0회 |
| 수건 낙하·작업대 이탈 | 0회 |
| 무한 또는 미기록 복구 | 0회 |

## 14. 구현된 소프트웨어 기반과 후속 구성

```text
config/
  towel_task_contract.candidate.yaml
  towel_annotation.schema.json
  towel_state_observation.schema.json
  towel_observation.example.json
  towel_annotation.example.json
  towel_replay.example.json
  towel_fake_reachability.example.json
docs/
  TOWEL_FOLDING.md
tools/
  lib/towel_geometry.py
  lib/towel_fold_path.py
  lib/towel_dataset.py
  lib/towel_perception.py
  lib/towel_task_runtime.py
  lib/towel_task_planning.py
  lib/towel_task_replay.py
  lib/towel_fake_reachability.py
  run/validate_towel_contract.py
  run/validate_towel_schemas.py
  run/validate_towel_dataset.py
  run/plan_towel_task_once.py
  run/replay_towel_task.py
  run/select_towel_fake_reachability.py
tests/
  test_towel_geometry.py
  test_towel_fold_path.py
  test_towel_dataset.py
  test_towel_perception.py
  test_towel_task_runtime.py
  test_towel_task_planning.py
  test_towel_task_replay.py
  test_towel_fake_reachability.py
  test_towel_schemas.py
```

위 목록은 현재 구현된 motion-free 기반이다. 이후 실제 mask backend,
`run_towel_task_once.py`, perception 진단 도구와 fold executor는 해당 로드맵
gate가 시작될 때만 추가하며 빈 placeholder를 먼저 만들지 않는다.
