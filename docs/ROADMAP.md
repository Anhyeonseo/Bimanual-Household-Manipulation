# 300 mm 정사각형 수건 펼치기·2회 접기 로드맵

로드맵의 최종 완료 조건은 [프로젝트 범위](SCOPE.md), 상태·관측·primitive의
설계는 [수건 접기 설계](TOWEL_FOLDING.md), 단계별 증빙은
[검증 매트릭스](VERIFICATION_MATRIX.md)를 따른다. 이 문서는 실행 순서와
승격 조건만 소유하며 세부 설계를 중복하지 않는다.

모든 실제 동작은 `plan-only → validate-only → 무수건 dry-run → supervised-once
→ 제한 반복 → 통합` 순서로만 승격한다. 앞 gate가 실패하면 뒤 단계의 실제
동작을 실행하지 않는다.

## R0 — 물리 계약·작업셀 go/no-go (진행 중)

nominal 수건 크기는 300×300 mm로 확정됐다. 다음 값과 좌표계를 실제 증빙으로
고정한다.

2026-08-25 R0-A에서 1280×960 optical FOV의 300 mm 수건+사방 30 mm 배치
가능성과 수동 `OBSERVE_CLEAR` visual candidate를 확인했다. 이는 metric
calibration이나 자동 자세 재현을 승인하지 않으며 아래 R0 조건은 그대로 남는다.

- side tolerance, 두께, 질량, 재질과 세탁·방향 조건
- 좌우 jaw gap, 한 겹/다층 cloth contact와 slip 기준
- 수건-작업대 마찰, 허용 TCP separation과 양팔 속도 차이
- 실제 Top 장치 경로·해상도와 runtime camera config의 일치
- Top-to-base, 작업대 plane, left/right wrist intrinsic·eye-in-hand
- 양팔이 수건을 가리지 않는 `OBSERVE_CLEAR`와 안전 퇴피 자세
- 300 mm 수건 전체와 승인된 외곽 여유가 Top의 검증된 metric 영역 안에 있음

동시에 300 mm rigid proxy와 실제 MoveIt으로 아래 최소 envelope를 검증한다.

- 펼침 footprint 300×300 mm
- 1차 fold moving-edge separation 최대 약 300 mm
- 두 fold의 corner 이동 약 300 mm와 기준 arc 높이 약 150 mm
- 2차 fold moving-edge separation 최대 약 150 mm
- 모든 pregrasp, lift, lay-down, release와 retreat에서 양팔·작업대 collision 없음

완료 조건: 선택 가능한 축·방향·팔 배정이 적어도 하나 존재하고, 카메라·작업대
좌표와 수건 전체가 같은 검증된 workcell frame에 들어온다. 값이 비어 있거나
좌표계가 거부된 동안 `motion_authorized=false`를 유지한다.

## R1 — 실제 관측·가림·topology

- `OBSERVE_CLEAR → primitive → RETREAT_AND_SETTLE → REOBSERVE_CLEAR` phase 구현
- 실제 image segmentation, component와 frame-border 검사
- URDF 기반 robot occlusion mask와 가림 비율
- visual corner, held TCP constraint와 unknown의 증거 출처 분리
- contour, 말린 edge, 내부 주름, layer ambiguity와 flatness 추정
- 들린 수건에 평면 homography를 사용하지 않는 3D/조건부 관측 경계
- timestamp, calibration/model identity, spread와 hysteresis가 포함된 stabilizer
- 구김·부분 펼침·정렬·1차/2차 fold 실제 데이터의 episode 단위 split

초기 구현은 RGB와 양 손목 다중 시점으로 시작하되 held-out 데이터에서 hidden
layer를 안전하게 거부하지 못하면 RGB-D 또는 고정 사선 카메라를 추가한다.

완료 조건: 실제 held-out episode에서 mask, corner, flatness, occlusion과 상태
분류가 검증 임계값을 통과하고, 가려짐·들림·다층 ambiguity를 `ALIGNED`로
승인하지 않는다.

## R2 — 30 cm plan-only와 Isaac 보조 검증

- 실제 MoveIt request/response reachability backend 연결
- x/y축, 양 방향, arm assignment와 grasp inset 후보 전수 평가
- 수직 lift, tension 유지, fold, 저속 lay-down, 동시 release, retreat로 path 분할
- waypoint별 양팔 IK, joint margin, self/world collision과 cable keep-out 검사
- `S0` 300 mm rigid proxy로 FOV·충돌·fold envelope 검증
- `S1` surface deformable과 명시적 vertex attachment로 grasp/release 순서 검증
- `S2` 실측 범위 material randomization으로 실패 사례와 영상 생성
- 모든 결과에 contract/calibration/URDF/plan/material/seed SHA 기록

Isaac 성공은 실제 cloth dynamics나 grasp의 승인 근거로 사용하지 않는다.

완료 조건: 300 mm nominal observation에서 적어도 한 개의 두-fold sequence가
MoveIt plan-only를 통과하고, 모든 거부 후보는 재현 가능한 이유를 남긴다.

## R3 — 안전 primitive와 접촉 계약

다음 순서로 primitive를 독립 승격한다.

1. `grasp_exposed_corner`, single-layer contact와 낮은 lift 확인
2. `grasp_two_corners`, 양쪽 contact timestamp와 교차 금지
3. `lift_and_observe`, `lay_flat`
4. `tension_spread`
5. `drag_corner`, `align_square`
6. `fold_edge_pair`, `release_and_smooth`
7. `controlled_shake`는 앞의 저속 primitive가 부족하다는 증거가 있을 때만 추가

각 primitive는 pre/postcondition, timeout, 최대 이동·속도, 장력 proxy, slip과
fault 중단, terminal measured feedback와 새 clear observation을 가진다. 한 팔의
fault는 같은 session의 양팔 정지로 이어진다.

완료 조건: 각 primitive가 무수건 dry-run과 supervised-once를 통과하고, 통합에
사용할 primitive는 최소 10회 제한 반복에서 안전 사고 0회와 사후 조건 9회
이상을 달성한다.

## R4 — 펼쳐진 300 mm 수건의 두 단계 접기

전체 구김 문제와 분리해 사람이 평탄·정렬한 300×300 mm 수건에서 먼저 fold
executor를 완성한다.

- 1차 single-layer dual grasp와 nominal 300×150 mm 결과 검증
- corner 대응, fold-line, twist와 stationary-half 미끄러짐 검사
- 2차 multi-layer bundle grasp와 첫 접힘 보존
- nominal 150×150 mm 결과, rebound와 stack 돌출 검사
- 실패한 1차 fold에서 2차 fold 금지

완료 조건: 고정된 수건·작업셀 조건에서 각 fold 단계가 독립 20회 중 19회
이상 품질 기준을 통과하고 충돌, 낙하와 workspace 이탈이 0회다.

## R5 — 거친 펼치기

- 가장 안전한 노출 지점의 single grasp와 낮은 lift
- 늘어진 실루엣에서 반대쪽 grasp 후보 재관측
- 저속 tension spread와 장력 유지 lay-flat
- 필요한 경우에만 제한된 작은 shake
- slip, 다층 grasp와 workspace 이탈의 즉시 중단

완료 조건: 대표 구김 입력 20회 중 19회 이상이 계약된 시도 횟수 안에
`PARTIALLY_OPEN` 또는 `TWO_CORNERS_VISIBLE`로 승격되고 안전 사고가 0회다.

## R6 — 정밀 평탄화와 정렬

- 말린 edge와 숨은 layer 검출
- 원인별 corner drag와 필요 최소 횟수 보정
- 네 변·대각선·면적뿐 아니라 topology/layer ambiguity 검증
- 300×300 mm footprint와 작업대 축 정렬
- 모든 승인 관측을 양팔 clear pose에서 수행

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

완료 조건: 정상·가림·slip·다층 grasp·fault·예산 소진 scenario가 모두 유한하게
`COMPLETE` 또는 `FAILED`로 끝나고, 실제 pilot 실행에서도 같은 session이나
실패 plan을 재사용하지 않는다.

## R8 — 최종 반복성·운영 승인

- 크기와 상태가 기록된 서로 다른 초기 구김 30회 benchmark
- 전체 성공 27/30 이상과 최종 150×150 mm 품질 기준 통과
- 단계별 조건부 성공률과 실패 signature 보고
- camera/resident soak, USB reconnect, stale frame, tracking fault와 stop injection
- headless bringup, 비상 정지·복구, artifact 보존 절차 검증
- 실패 사례를 dataset과 hardware-free replay 회귀시험에 반영

완료 조건: [완료 정의](SCOPE.md#완료-정의)의 품질·안전 기준을 모두 통과하고,
충돌·비명령 동작·수건 낙하·workspace 이탈과 미기록 복구가 0회다.
