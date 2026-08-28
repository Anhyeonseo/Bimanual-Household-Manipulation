# 300 mm 정사각형 수건 펼치기·2회 접기 로드맵

로드맵의 최종 완료 조건은 [프로젝트 범위](SCOPE.md), 상태·관측·primitive의
설계는 [수건 접기 설계](TOWEL_FOLDING.md), 단계별 증빙은
[검증 매트릭스](VERIFICATION_MATRIX.md)를 따른다. 이 문서는 실행 순서와
승격 조건만 소유하며 세부 설계를 중복하지 않는다.

모든 실제 동작은 `plan-only → validate-only → 무수건 dry-run → supervised-once
→ 제한 반복 → 통합` 순서로만 승격한다. 앞 gate가 실패하면 뒤 단계의 실제
동작을 실행하지 않는다.

학습은 최종 목표에서 선택 사항이 아니다. 사람이 펴 둔 수건의 큰 접기 동작에는
기하·MoveIt baseline을 먼저 사용하지만, 1차 coarse fold 뒤의 자잘한 오차·구김
보정과 임의 구김의 펼치기·정규화에는 실제 데이터와 시뮬레이션을 결합한
learned policy를 R5의 필수 구성으로 둔다. 다만 `강화학습`이라는 이름만으로
방법을 고정하지 않는다. 같은
관측·primitive·평가 split에서 heuristic, self-supervised/모방학습, model-based
planning과 RL을 비교하고, held-out 성능과 실제 안전성이 가장 좋은 방법을
승격한다. 학습 출력은 저수준 관절·토크가 아니라 승인된 primitive와 제한된
grasp/placement 파라미터이며 MoveIt과 실행 gate를 우회할 수 없다.

## R0 — 물리 계약·작업셀 go/no-go (완료)

nominal 수건 크기는 300×300 mm로 확정됐다. 다음 값과 좌표계를 실제 증빙으로
고정한다.

2026-08-25 R0-A에서 1280×960 optical FOV의 300 mm 수건+사방 30 mm 배치
가능성과 수동 `OBSERVE_CLEAR` visual candidate를 확인했다. 이는 metric
calibration이나 자동 자세 재현을 승인하지 않으며 아래 R0 조건은 그대로 남는다.

같은 날 R0-B에서 왼팔 Top eye-to-hand와 `377.296×371.513 mm` 작업대 metric
영역을 독립 검증했고, R0-C에서 torque-on terminal anchor 기반 오른팔 등록도
독립 validation 위치 max `3.272 mm`, 회전 max `0.966 deg`로 통과했다. 오른팔
workcell shadow는 x/y max `3.272 mm`, yaw max `0.515 deg`로 통과했다. 이어
7구간 MoveIt 경로의 469개 상태 충돌 검사와 supervised
`현재→OBSERVE_CLEAR→현재→OBSERVE_CLEAR`를 통과했고, 두 도착의 300×300 mm
수건 전체·네 모서리에 robot 가림이 없었다. 이 결과는 clear observation 자세만
승격하며 URDF/제어 영점이나 일반 실제 motion을 승인하지 않는다.

R0-D에서는 실제 마른 미세탁 면 100% 수건의 네 변 `304/296/304/296 mm`, 자로
잰 몸통 두께 `1/2/4겹≈3/7/13 mm`를 등록했다. 좌우 그리퍼에서 각각 1겹과
4겹을 현재 자세로 두 차례 유지하고 가볍게 당겼을 때 빠짐이나 가시적 미끄러짐이
없었으며, 모든 실행이 coordinated STOP과 torque-off로 끝났다. 이는 수동으로
만든 접촉점의 정적 retention 증거이지 자동 close-to-contact, 동적 slip, lift,
장력 또는 fold 승인은 아니다.

R0-E에서는 right wrist `640×480` intrinsic을 34개 training과 완전 미사용
8개 validation 영상으로 검증했고 validation reprojection max는 `0.891 px`였다.
초기 torque-off eye-in-hand 세트의 위치 오차가 최대 `30.739 mm`였던 원인은
R0-C의 torque-on FK 등록과 source load state가 달랐기 때문으로 확인했다.
resident hold owner/epoch와 terminal measured anchor가 일치하는 6개 training과
완전 미사용 validation 2개를 다시 수집한 결과 training RMS/max
`8.463/13.517 mm`, validation RMS/max `5.473/5.625 mm`, validation 회전 max
`0.898 deg`로 통과했다. 검증 transform을 right wrist optical frame에 반영했지만
이는 손목 영상 단독 3D나 일반 실제 motion을 승인하지 않는다.

R0-F에서는 로봇팔이 Top을 가리는 실제 구조 때문에 한 시점의 동시 촬영을
강제하지 않고, 고정한 GridBoard를 팔이 비운 Top stage와 resident hold의 wrist
stage로 나눠 찍는 fail-closed 계약을 사용했다. controlled training 2자세만으로
right eye-in-hand의 그리퍼 기준 평행이동 3축을 적합하고 회전은 고정했다. 보정
크기는 `17.466 mm`였으며, 보정에 전혀 쓰지 않은 staged validation 2자세에서
XY RMS/max `10.327/12.309 mm`, Z max `12.313 mm`, yaw max `1.232 deg`였다.
전체 4자세의 XY span은 `223.716 mm`, 비공선 높이는 `111.456 mm`였다. 이
결과는 tabletop 물체 좌표와 300 mm rigid-proxy plan-only 사용만 승인하며 실제
motion은 계속 승인하지 않는다.

R0-G의 canonical 후보는 펼쳐진 300 mm edge를 양팔로 먼저 접고, 300×150 mm
결과의 짧은 moving edge 중앙을 가까운 한 팔로 접는다. 첫 fold가 어긋나면
두 번째 fold를 금지하고 clear 재관측 뒤 `micro_drag` 또는 `lift_pull_place`를
최대 2회만 허용한다.

SO-101 한 팔의 5-DOF를 반영해 임의 exact 6D pose를 요구하지 않는다. 모든
phase는 TCP xyz와 jaw opening-line yaw를 검사하고 full 6D FK를 기록한다.
새 grasp의 contact/pregrasp에는 70° downward approach cone을 적용하며, 이미
cloth가 붙은 transfer·laydown은 물리적 의미와 기존 dense Cartesian 검증에
맞춰 최대 90°를 허용한다. 2차 laydown TCP는 도달 불가능한 16 mm를 주장하지
않고 검증된 40 mm release 높이를 사용한다.

현재 후보의 전체 full-FK IK와 software regression에 이어 최종 strict MoveIt
plan-only gate도 통과했다. 등록 완료 r0g URDF manifest, workcell shadow와 right
tabletop validation을 고정해 1차 양팔 아래→위, bounded correction 8개, 2차
오른팔 오른쪽→왼쪽 후보의 840개 구간·12,547개 상태를 검사했다. 미승인 접촉은
0건이고 mesh 접촉 최대 `3.810/4 mm`, TCP 편차 최대 `2.868/4 mm`다. data-fit
candidate URDF의 약 16.2 mm 카메라 마운트 충돌은 모델을 섞거나 예외 처리하지
않고 이전 fail-closed 증거로 남긴다.

현재까지 고정된 R0 계약은 다음과 같다.

- 완료: side tolerance, 근사 두께, 재질과 세탁·건조 조건
- 완료: 좌우 한 겹/4겹 정적 cloth retention
- 완료: 실제 Top 장치 경로·해상도와 runtime camera config의 일치
- 완료: Top-to-base, 작업대 plane, right wrist intrinsic·eye-in-hand·tabletop 교차검증
- 보존: left wrist intrinsic과 과거 eye-in-hand URDF 값. 새 staged tabletop
  교차검증은 R1에서 metric 융합 전에 수행
- 완료: 양팔이 수건을 가리지 않는 `OBSERVE_CLEAR` 왕복과 안전 정지
- 완료: 300 mm 수건 전체와 승인된 외곽 여유가 Top의 검증된 metric 영역 안에 있음
- 연기: 질량과 수건-작업대 마찰은 이를 소비하는 동적 gate 전에 측정
- 완료: 오른팔 FK+wrist tabletop 물체 좌표의 독립 target 검증
- 완료: 1차 양팔·폐루프 보정과 2차 단팔 접기의 task-pose MoveIt plan-only envelope
- 미완료: 자동 contact/slip, 허용 TCP separation과 양팔 속도 차이는 이를
  소비하는 primitive 전에 commission

300 mm rigid proxy와 MoveIt으로 아래 R0 최소 envelope를 검증한다.

- 펼침 footprint 300×300 mm
- 1차 양팔 moving-edge endpoint grasp와 동기 fold arc
- clear 재관측에서 현재/목표 mask·corner·fold-line 오차를 계산하는 bounded 보정
- 2차 fold의 짧아진 moving-edge midpoint 단팔 grasp
- 모든 pregrasp, lift, lay-down, release와 retreat에서 robot·작업대·카메라
  거치대·케이블 keep-out 위반 없음
- 정적 proxy 도달성은 cloth가 실제로 따라오거나 집힌다는 증거로 사용하지 않음

완료 판정: 물리·좌표계, clear observation과 canonical task-pose MoveIt plan-only
후보 하나가 등록 완료 URDF와 strict collision gate를 통과해야 한다. 후보 탐색은
고정된 우선순위에서 첫 승인 해가 나오면 멈추고 앞선 거부 이유를 기록한다. 이
R0 plan-only 판정은 r0g artifact에서 완료됐다. 실제 fold, 자동 jaw contact와
cloth deformation은 승인하지 않으며 `motion_authorized=false`를 유지한다.

## R1 — 실제 관측·가림·topology

- `OBSERVE_CLEAR → primitive → RETREAT_AND_SETTLE → REOBSERVE_CLEAR` phase 구현
- 실제 image segmentation, component와 frame-border 검사
- 승인된 양팔 clear pose joint tolerance와 보수적 clear-view validity 검사
- robot-occluded frame을 clear-view 거절/OOD 세트로 유지; 정밀 pixel mask는 실패 근거가 있을 때만 추가
- visual corner, held TCP constraint와 unknown의 증거 출처 분리
- contour, 말린 edge, 내부 주름, layer ambiguity와 flatness 추정
- 들린 수건에 평면 homography를 사용하지 않는 3D/조건부 관측 경계
- timestamp, calibration/model identity, spread와 hysteresis가 포함된 stabilizer
- 구김·부분 펼침·정렬·1차/2차 fold 실제 데이터의 episode 단위 split

R1의 작업대 평면 상태 판정은 검증된 `OBSERVE_CLEAR`와 고정 Top RGB만으로
완료했다. 이 구조에서는 정밀 robot pixel mask와 wrist fusion을 추가해도 승인
정보가 늘지 않으므로 보류한다. 이후 들린 수건이나 hidden layer를 실제로 판정할
때 Top RGB만으로 안전하게 거부하지 못하면 wrist 다중 시점, RGB-D 또는 고정
사선 카메라를 근거 순서대로 추가한다.

완료 결과: 1280×960 RGB 개발 원본 595장(empty 16, 평탄 100, 가벼운 구김 120,
심한 구김 107, 말림·겹침 73, 1차 fold 81, 2차 fold 50, robot 가림 48)을
통합했고 전수 decode·크기 검사를 통과했다. 두 차례 LabelMe 검수로 non-empty
90장과 empty 13장, 총 103개 segmentation annotation을 계약 import했고 잘못된
empty 2장은 거절했다. 자동 제안은 두 batch 모두 평균 IoU 0.9 이상이지만 각 1장씩
완전 실패가 있어 무검수 승격하지 않는다. 독립 held-out session 38장은 매 frame
물리 재배치와 capture ID/SHA를 기록했고, 그중 35장(empty 5, non-empty 30)을
사람 검수 segmentation으로 승인했으며 robot-occluded 3장은 rejection/OOD다.
reviewed mask는 empty 13개, 단일 connected component 90개, 다중 component 0개이며
22개가 image border에 닿는다. border 접촉 mask는 보이는 영역의 유효 training
label로 남기되 runtime에서는 `clear_view_valid=false`로 거절한다.
held-out mask 후보는 towel 30/30·empty 5/5, non-empty IoU 평균 0.980284·최저
0.965564이고 border 잘림 4/4, false negative 0, 보수적 false reject 1이다.
원본 pixel은 K/D/P 왜곡 보정 후 table homography로 투영하고 visible area는 실측
`0.304×0.296 m` 면적과 metric으로 비교한다. mask outline backend는 hidden layer와
fold count를 추측하지 않아 non-flat/fold held-out을 `ALIGNED`로 승인하지 않는다.
motion-free lifecycle은 freshness, settle, clear pose, clear-view validity,
연속 3-frame과 calibration/model/URDF identity를 fail-closed로 검사하고 primitive
전후 episode evidence를 만든다. 실제 `20260827_top_lifecycle_validation_01`의
5개 물리 배치에서 각 3장씩 15장을 검증했고 presence·clear view와 상태가 모든
window에서 일치했다. fold count는 RGB에서 추측하지 않고 검증된 fold action
context에서만 주입한다. 20 mm 이하 비그립 봉제 고리는 segmentation에는 보존하되
metric fold-body outline에서 제외했으며, 1차 fold IoU 최저 `0.903769`, 2차 fold
최저 `0.859693`으로 기존 `0.82/0.85` 기준을 통과했다. 따라서 R1은 완료다.

완료 조건: 실제 held-out episode에서 mask, corner, flatness, clear-view rejection과 상태
분류가 검증 임계값을 통과하고, 가려짐·들림·다층 ambiguity를 `ALIGNED`로
승인하지 않는다.

## R2 — Isaac Lab cloth와 학습 기반

- R0의 canonical task-pose sequence와 실제 R1 observation을 공통 baseline으로 사용
- `S0`에서 R0 plan artifact 재생, vectorized reset과 FOV·접근·충돌 재현성을 검사
- `S1` 삼각 surface deformable과 명시적 vertex-patch attachment로 양팔 1차 fold,
  bounded correction과 단팔 2차 fold의 grasp/release 순서 검증
- `S2` 실측 범위 material randomization으로 실패 사례와 영상 생성
- Isaac Lab 환경을 `reset/observation/action/reward/termination` 계약으로 구현
- 행동 공간은 승인된 단팔/양팔 primitive, pick/place·높이·장력과
  `ACCEPT/RETRY`로 제한
- coverage, corner/topology, 정렬 진전과 collision·drop·workspace·시도 비용을
  분리해 reward hacking을 replay와 oracle state로 검사
- 병렬 randomized rollout과 scripted baseline을 먼저 통과한 뒤 RL 학습 시작
- 모든 결과에 contract/calibration/URDF/plan/material/seed SHA 기록

Isaac 성공은 실제 cloth dynamics나 grasp의 승인 근거로 사용하지 않는다. 실제
수건에서 관측된 settling, friction, grasp/slip과 action outcome 분포를 설명하지
못하면 simulator를 더 복잡하게 맞추기보다 real self-supervised/offline 학습으로
전환한다.

완료 조건: R0의 300 mm canonical plan artifact가 Isaac Lab S0 vectorized smoke
test에서 결정적으로 재생되고, S1의
`drop→settle→attach→lift→place→release` 및 correction trajectory가 결정적으로
재생되며, 학습 전에 실제와 비교할 상태·행동·결과 metric이 확정돼 있다.

## R3 — 안전 primitive와 접촉 계약

다음 순서로 primitive를 독립 승격한다.

1. `grasp_exposed_corner`, single-layer contact와 낮은 lift 확인
2. `grasp_two_corners`, 양쪽 contact timestamp와 교차 금지
3. `lift_and_observe`, `lay_flat`
4. `tension_spread`
5. `drag_corner`, `align_square`
6. `bimanual_edge_pair`, `micro_drag`, `lift_pull_place`
7. `single_arm_edge_midpoint_fold`, `release_and_smooth`
8. `controlled_shake`는 앞의 저속 primitive가 부족하다는 증거가 있을 때만 추가

각 primitive는 pre/postcondition, timeout, 최대 이동·속도, 장력 proxy, slip과
fault 중단, terminal measured feedback와 새 clear observation을 가진다. 한 팔의
fault는 같은 session의 양팔 정지로 이어진다.

완료 조건: 각 primitive가 무수건 dry-run과 supervised-once를 통과하고, 통합에
사용할 primitive는 최소 10회 제한 반복에서 안전 사고 0회와 사후 조건 9회
이상을 달성한다.

## R4 — 펼쳐진 300 mm 수건의 두 단계 접기

전체 구김 문제와 분리해 사람이 평탄·정렬한 300×300 mm 수건에서 먼저 fold
executor를 완성한다.

- 1차 아래쪽 moving edge 양 끝의 single-layer 양팔 grasp와 아래→위 fold,
  nominal 300×150 mm coarse 결과 검증
- clear 재관측 뒤 평행이동·회전·느슨함을 최대 2회의 bounded correction으로 보정
- stationary-half가 함께 미끄러지면 pull 축소 또는 승인된 passive pin 사용
- 2차 오른쪽 moving edge midpoint의 multi-layer 오른팔 단독 grasp와
  오른쪽→왼쪽 fold; 왼팔 반대 방향은 bounded fallback으로만 유지
- nominal 150×150 mm 결과, rebound와 stack 돌출 검사
- 실패한 1차 fold에서 2차 fold 금지

완료 조건: 고정된 수건·작업셀 조건에서 각 fold 단계가 독립 20회 중 19회
이상 품질 기준을 통과하고 충돌, 낙하와 workspace 이탈이 0회다.

이 baseline은 R5 학습의 비교군이자 안전한 fold executor다. 첫 실기 시연은
사람이 correction point만 승인하고 MoveIt·executor가 자동 실행하는
assisted-autonomous 단계로 시작할 수 있다. 이후 학습 정책이 fold
grasp/placement를 제안하더라도 동일한 executor와 사후 검증을 사용한다.

## R5 — 학습 기반 residual correction과 거친 펼치기

- 가장 안전한 노출 지점의 single grasp와 낮은 lift
- 늘어진 실루엣에서 반대쪽 grasp 후보 재관측
- 저속 tension spread와 장력 유지 lay-flat
- 필요한 경우에만 제한된 작은 shake
- slip, 다층 grasp와 workspace 이탈의 즉시 중단
- Top/손목 관측에서 primitive와 양팔 grasp 파라미터를 고르는 learned policy
- 1차 coarse fold 뒤 현재/목표 mask, corner·edge 오차와 직전 action history로
  다음 bounded correction 또는 `ACCEPT/RETRY`를 고르는 goal-conditioned policy
- Isaac Lab domain randomization pretraining과 실제 episode fine-tuning 비교
- 같은 episode split에서 heuristic, behavior cloning/self-supervised, RL을 비교
- policy confidence가 낮거나 분포 밖이면 `abstain`하고 제한 복구 또는 정지

첫 학습 목표는 end-to-end 12축 제어나 teleop trajectory 복제가 아니다. 평탄
수건의 1차 coarse fold 뒤 최대 2회 macro-action으로 오차를 줄이는 visual
residual policy를 먼저 만들고, 같은 action 계약을 `CRUMPLED`에서
`PARTIALLY_OPEN/TWO_CORNERS_VISIBLE`, 이어 `FOUR_CORNERS_VISIBLE`로 가는 선택에
확장한다. Fling처럼 고속 동작은 SO-101의 가동범위·그립·추종오차 실측이 저속
primitive의 한계를 입증하고 별도 안전 gate를 통과한 뒤에만 후보로 추가한다.

완료 조건: fold residual policy는 고정된 held-out coarse-fold 오류에서 같은
최대 2회 action budget의 heuristic보다 1차 fold 승인률을 개선해야 한다. 거친
펼치기 정책은 대표 구김 입력 20회 중 19회 이상을 계약된 시도 횟수 안에
`PARTIALLY_OPEN` 또는 `TWO_CORNERS_VISIBLE`로 승격하고 안전 사고가 0회여야
한다. 동률이면 시도 횟수·시간을 비교하며 simulator checkpoint만으로 실제
실행을 승인하지 않는다.

## R6 — 정밀 평탄화와 정렬

- 말린 edge와 숨은 layer 검출
- 원인별 corner drag와 필요 최소 횟수 보정
- 네 변·대각선·면적뿐 아니라 topology/layer ambiguity 검증
- 300×300 mm footprint와 작업대 축 정렬
- 모든 승인 관측을 양팔 clear pose에서 수행
- R5 policy의 canonicalization 결과를 입력으로 받고, 필요하면 learned
  grasp/correction scorer가 `drag_corner`와 재파지 후보의 순위만 제안

완료 조건: 대표 부분 펼침 입력 20회 중 19회 이상이 `ALIGNED`가 되고, 잘못된
topology를 `ALIGNED`로 승인한 false positive가 0회다.

## R7 — 통합 task manager와 원인별 제한 복구

- 관측→primitive→퇴피·settle→재관측의 전체 상태기계
- 모서리 재탐색 최대 3회, lift-and-unfold 최대 2회
- corner drag 모서리당 최대 2회, fold correction 단계당 최대 1회
- `NO_VISIBLE_CORNER`, `MULTI_LAYER_GRASP`, `SLIP`, `OCCLUSION`,
  `FOLD_MISALIGNMENT`처럼 실패 signature별 복구 분기
- 같은 실패 signature 반복, stale calibration, fault와 workspace exit의 종료
- primitive outcome, measured feedback와 전후 observation artifact 연결
- learned policy의 dataset/checkpoint SHA, confidence, 선택 후보와 `abstain` 기록
- 같은 실패 signature의 policy 재호출도 기존 recovery budget 안에서만 허용

완료 조건: 정상·가림·slip·다층 grasp·fault·예산 소진 scenario가 모두 유한하게
`COMPLETE` 또는 `FAILED`로 끝나고, 실제 pilot 실행에서도 같은 session이나
실패 plan을 재사용하지 않는다.

## R8 — 최종 반복성·운영 승인

- 크기와 상태가 기록된 서로 다른 초기 구김 30회 benchmark
- 전체 성공 27/30 이상과 최종 150×150 mm 품질 기준 통과
- 단계별 조건부 성공률과 실패 signature 보고
- 동일 초기 상태 split에서 heuristic/learned policy ablation과 일반화 실패 보고
- camera/resident soak, USB reconnect, stale frame, tracking fault와 stop injection
- headless bringup, 비상 정지·복구, artifact 보존 절차 검증
- 실패 사례를 dataset과 hardware-free replay 회귀시험에 반영

완료 조건: [완료 정의](SCOPE.md#완료-정의)의 품질·안전 기준을 모두 통과하고,
충돌·비명령 동작·수건 낙하·workspace 이탈과 미기록 복구가 0회다.
