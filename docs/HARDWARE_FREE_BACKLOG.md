# 하드웨어 없이 진행하는 개발 백로그

이 문서는 로봇, 실제 카메라 또는 추가 수건 실측 없이 진행할 수 있는 작업과
실제 하드웨어 증빙이 필요한 작업의 경계를 고정한다. nominal 수건 크기는
300×300 mm이며 우선순위는 P0, P1, P2 순서다.

## 완료된 기반

- motion-locked candidate task contract
- 수건 observation annotation JSON Schema
- 사각형 corner ordering, 변·대각선·정렬 metric
- 두 직교 half-fold의 기하 plan과 polygon IoU
- 두 moving corner의 동기 반원 fold arc와 기하 불변조건
- 단일 observation 상태 분류와 3-frame 안정화 gate
- fault/workspace exit 비재시도와 유한 recovery ledger
- SHA가 기록된 motion-free plan-only JSON artifact
- synthetic aligned observation과 회귀 시험
- annotation validator와 deterministic split-safe dataset manifest
- observation sequence offline replay와 유한 terminal artifact
- 양팔 1차·bounded correction·단팔 2차의 canonical task-pose/full-FK IK gate

## P0 — 데이터·관측 계약

### P0.1 Annotation validator (완료)

- JSON Schema를 기준으로 annotation 파일과 dataset index 검사
- 중복 observation ID, split 간 source SHA 중복과 누락 파일 거부
- polygon bounds, corner label 중복, fold-line 개수 검사

완료 조건: 잘못된 fixture를 각각 한 가지 이유로 거부하고 정상 dataset index를
결정적으로 검증한다.

### P0.2 Dataset manifest와 split (software pipeline 완료)

- train, validation, test source SHA 분리
- 구김 강도, 회전, 가림, 조명과 fold state 분포 기록
- 실제 데이터 전 synthetic fixture로 manifest pipeline 검증

완료 조건: 데이터 추가 순서와 관계없이 동일한 manifest SHA를 생성한다.

### P0.3 Observation artifact versioning

- pixel annotation과 metric workcell observation을 명시적으로 분리
- camera/calibration/model/dataset SHA와 timestamp 필수화
- schema migration은 새 버전으로만 허용

완료 조건: stale calibration, 알 수 없는 schema version과 digest mismatch가
planning 전에 거부된다.

## P1 — Offline perception

### P1.1 Segmentation interface (annotation backend 구현)

- image 입력을 mask, confidence, inference metadata로 바꾸는 backend 경계
- 실제 model이 없어도 prerecorded mask backend로 전체 pipeline 시험
- 빈 mask, 여러 component, frame-border 접촉을 fail-closed

완료 조건: backend를 바꿔도 동일한 TowelObservation 계약을 출력한다.

현재 reviewed polygon annotation을 homography로 투영하는 offline backend와
낮은 topology confidence의 `ALIGNED` 승격 차단은 구현됐다. mask inference와
component/frame-border 검사는 남아 있다.

### P1.2 Contour와 corner 후보

- mask contour 단순화와 convexity/concavity feature
- 네 corner 후보, visibility와 graspability 분리
- 정사각형 대칭 때문에 corner ID를 프레임마다 재부여

완료 조건: 회전·반사·점 순서·작은 contour noise에 metric이 불변이다.

### P1.3 Flatness와 topology confidence

- visible area, edge/diagonal, concavity, optional height feature 결합
- depth가 없으면 height_available=false와 낮은 topology confidence
- hidden layer를 확인할 수 없는 입력은 AMBIGUOUS

완료 조건: confidence가 낮은 입력이 절대 ALIGNED로 승격되지 않는다.

### P1.4 Temporal stabilizer

- 현재 3-frame 동일 상태 gate를 timestamp, spread와 hysteresis까지 확장
- calibration/model identity가 다른 frame 혼합 금지
- frame replay에서 결정론적 상태 전이 보장

완료 조건: flicker, stale frame, out-of-order timestamp 회귀 시험 통과.

## P1 — Planning과 상태기계

### P1.5 Primitive pre/postcondition contract

- 열 개 primitive의 입력 상태, 예상 출력 상태와 reject code
- 최대 거리·시간·속도의 hardware-independent 필드
- hardware-dependent 필드는 null이면 plan-only만 허용

완료 조건: primitive마다 plan-only 가능 여부와 blocker가 JSON으로 설명된다.

### P1.6 Fold-axis와 arm assignment cost (fixture selector 완료)

- x/y축, 양 fold 방향, 좌우 팔 grasp 배정 후보 열거
- joint-space distance, inter-arm crossing, workspace margin cost
- 비용 동률의 deterministic tie-break

완료 조건: 후보 입력 순서가 바뀌어도 선택 결과와 artifact SHA가 같다.

현재 axis/direction/arm assignment, joint distance/crossing/workspace penalty와
결정적 tie-break를 갖는 motion-free fixture selector가 구현됐다.

### P1.7 Geometric fold path (순수 기하 완료)

- corner lift, centerline 통과, target lay-down의 parameterized arc
- 양팔 TCP separation과 상대 속도의 symbolic constraint
- MoveIt 없이 기하 검증 후 fake reachability backend 연결

현재 두 moving corner의 synchronized semicircle, 시작·목표 일치, 중간 높이와
corner 간격 보존 시험과 fake backend 연결이 구현됐다. artifact는 실제
reachability/collision 미검증을 명시한다.

완료 조건: waypoint가 fold 방향·footprint와 일치하고 잘못된 arc는 거부된다.

### P1.8 Finite task replay (기본 구현 완료)

- observation sequence와 primitive outcome을 입력하는 offline task replay
- 모든 경로가 COMPLETE 또는 FAILED로 유한 종료
- recovery counter와 실패 원인을 artifact로 저장

완료 조건: 정상·flicker·fault·예산 소진 scenario의 golden test 통과.

## P2 — Simulation과 개발 도구

### P2.1 Visualization

- mask, corners, fold axis, moving edge, target footprint overlay
- state/reject reason과 confidence 표시
- image와 JSON report 동시 저장

### P2.2 Fake MoveIt/reachability backend (기본 fixture 구현)

- 좌우 grasp 가능 여부와 collision 결과를 fixture로 주입
- 모든 후보 거부, 한 축만 가능, 팔 배정 swap scenario

reachable/collision fixture 주입과 전 후보 거부 회귀시험은 유지한다. 실제
MoveIt task-pose planner는 R0-G에서 별도 연결한다. 현재 canonical 후보의
full-FK IK는 통과했고 strict collision은 등록 artifact 복원 뒤 재실행한다.
이 절의 fake backend는 hardware-free 정책 회귀용이다.

### P2.3 Isaac Lab cloth와 학습 환경

- 300×300 mm rigid proxy는 FOV, 5-DOF task-constrained 접근과 collision만
  검증하고 fold 성공 근거로 사용하지 않음
- 1차 양팔 fold·bounded correction과 2차 단팔 midpoint fold를 서로 다른
  primitive로 모델링
- 304×296 mm 삼각 surface deformable과 명시적 vertex-patch attachment를 후속
  layer로 분리
- Isaac Lab vectorized reset/observation/action/reward/termination 계약 구현
- 승인된 primitive, bounded pick/place·pull 파라미터와 `ACCEPT/RETRY`만
  action으로 노출
- heuristic baseline 뒤 self-supervised/모방학습과 RL을 같은 seed에서 비교
- simulation은 planner 정답이 아니라 rollout·실패 사례·perception data에 사용
- solver/material parameter provenance 저장
- reward exploit, collision/drop과 workspace 이탈 회귀시험
- simulation 성공을 실제 motion 승인 근거로 사용하지 않음; sim-to-real gap이
  크면 실제 replay 기반 fine-tuning으로 전환
- 첫 학습 목표는 Top 현재/목표 mask와 action history에서 다음 fold correction을
  고르는 residual policy이며 실제 무작위 exploration은 금지

### P2.4 CI (hardware-free workflow 구현)

- contract, schema/dataset, geometry/runtime 시험
- example observation에서 plan-only artifact 재생성·검증
- 문서 링크와 git diff 검사

현재 GitHub Actions가 수건 전용 시험, 계약/schema/dataset 검증, example artifact
재생성과 `motion_authorized=false`를 확인한다. 문서 링크 검사는 로컬 QA에
남아 있다.

## 후속 동적 gate 전까지 고정하지 않는 값

- 수건 질량과 batch별 재질 편차
- jaw open/contact command와 접촉 residual
- 최대 장력 proxy와 TCP separation
- 양팔 속도 차이
- controlled shake 진폭·주파수·횟수
- 수건-작업대 마찰
- 실제 mask/topology perception acceptance threshold

한 변 `304/296/304/296 mm`, 1/2/4겹 근사 두께 `3/7/13 mm`, 면 100%·건조·
미세탁 조건, Top/right camera 오차와 좌우 1/4겹 정적 retention은 R0에서 이미
고정했다. 위 동적 값은 이를 처음 소비하는 R2/R3 gate까지 candidate contract의
null을 유지하며, 그 전 artifact는 `motion_authorized=false`,
`motion_commands=0`이어야 한다.
