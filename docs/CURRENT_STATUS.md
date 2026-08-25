# 현재 상태

기준일: 2026-08-25

최신 기록된 자동검증 결과는
[2026-08-20 수건 software foundation 검증](test-results/2026-08-20-towel-software-foundation.md)이다.
이후 nominal 수건 크기와 실행 로드맵을 300 mm 기준으로 갱신했으며 현재
회귀시험 결과는 repository test로 확인한다.

## 프로젝트 상태

최종 목표 수건은 nominal 300×300 mm로 확정됐다. 실제 side tolerance, 두께,
질량, 재질과 접촉·마찰 한계는 아직 측정되지 않았다. 프로젝트는 계속
`SOFTWARE_FOUNDATION` 단계이며 실제 수건 motion은 승인되지 않았고
`motion_authorized=false`다.

## 재사용 가능한 기반

- protocol v2 기반 STM32 12축 resident executor와 measured feedback
- 단일 serial owner를 유지하는 양팔 resident adapter
- 승인된 양팔 operational limits와 양팔 URDF/MoveIt 구성
- 상단·양 손목 카메라 수집과 phase scheduling
- intrinsic, eye-to-hand, worktable 보정 도구
- plan SHA, validate-only, one-shot 실행과 fail-closed 패턴
- 수건 annotation/schema, 순수 기하, 상태 gate와 offline replay
- 300 mm square의 직교 2회 fold geometry와 fake reachability fixture

## 수건 태스크 구현 상태

| 구성 | 현재 | 다음 승인 조건 |
|---|---|---|
| 태스크 범위 | nominal 300×300 mm, 최종 150×150 mm | 나머지 물성·허용편차 실측 |
| annotation 계약 | schema, validator, deterministic split manifest | 실제 episode index 생성 |
| 수건 데이터셋 | synthetic example만 있음 | 구김·평탄·1차/2차 fold 실제 데이터 |
| segmentation | reviewed polygon→metric observation backend | 실제 mask, component, border 검사 |
| corner/topology | 순수 기하와 confidence gate | 가림·말림·다층 ambiguity 검증 |
| temporal state | 3-frame 동일 상태 gate | timestamp, spread, settle, hysteresis |
| observation lifecycle | Top metric 영역·실기 clear 왕복과 무가림 재관측 PASS | camera phase와 runtime 상태기계 구현 |
| fold plan-only | 300 mm 직교 기하·arc와 fake selector | 실제 MoveIt IK/collision adapter |
| Isaac/학습 | 표시 전용 workcell과 legacy 단일팔 rigid scripted grasp; Isaac Lab towel env·policy는 없음 | S0/S1 physics와 vectorized smoke test, heuristic baseline부터 구축 |
| 조작 primitive | 미구현 | 개별 plan-only→supervised 제한 반복 |
| 펼쳐진 수건 2회 접기 | 미구현 | R4 standalone fold gate 통과 |
| 펼치기·평탄화 | 미구현 | R5/R6 단계 성공 기준 통과 |
| 통합 복구 | offline 유한 상태기계 | 실제 실패 signature와 feedback 연결 |
| hardware-free CI | 수건 계약·기하·replay workflow 구현 | 갱신된 300 mm fixture 회귀 확인 |

Top 카메라 R0-A 실기 확인에서 장치
`/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1.1:1.0-video-index0`, MJPEG
1280×960@30 fps와 약 445 mm 설치 높이를 확인했다. 640×480과 1280×960은
같은 화각이고, 1920×1080은 수직 화각을 늘리지 않고 좌우만 약 33% 추가한다.
실제 nominal 300 mm 수건의 네 모서리로 계산한 사방 30 mm 투영 envelope는
1280×960 안에 배치 가능했다. 이는 optical containment 후보 PASS이지 기존
homography 바깥의 metric 정확도를 승인한 결과는 아니다.

수동으로 배치했던 `OBSERVE_CLEAR` 후보는 R0-C에서 실제 왕복으로 승격했다.
검증된 worktable collision과 오른팔 등록 URDF를 사용한 7구간 MoveIt plan-only
경로를 0.01 rad 간격 469개 상태로 재검사했고, 비승인 접촉 0건과 허용된 메시
접촉 최대 `2.451 mm`(제한 `4 mm`)를 확인했다. resident firmware
`0x00024809`에서 `현재→clear→현재→clear`를 실행한 세 leg의 terminal 오차는
최대 `0.013805 rad`, 두 clear 도착 간 관절 반복 오차는 `0 rad`였다. 두 도착의
1280×960 Top 영상 모두 실제 300×300 mm 수건 전체와 네 모서리가 보였고
robot/gripper 가림은 없었다. 마지막 coordinated STOP과 torque-off도 확인했다.

R0-B에서는 Top `1280x960` eye-to-hand를 training `left_train_01..08,10`과
완전 미사용 validation `left_validation_01..02`로 재수집했다. 관절 끝단에
놓였던 `left_train_09`는 기존 해의 위치 잔차가 약 `14.852 mm`인 작업영역
outlier로 기록만 보존하고 해에는 사용하지 않았다. 고정 세트의 training 위치
RMS/max는 `3.630/5.166 mm`, 회전 RMS/max는 `0.921/1.711 deg`였고, 독립
validation 위치 max는 `4.250 mm`, 회전 max는 `1.429 deg`로
`EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED`를 통과했다. 원본과 해는
`artifacts/calibration/top_eye_to_hand_20260825_r0b/`에 보존한다.

R0-C에서는 오른팔의 실제 torque-on terminal anchor가 torque-off 관절값과 같은
정지 자세에서 마커 위치 기준 `0.063 mm`, 회전 기준 `0.068 deg` 차이임을 먼저
확인했다. 이후 resident hold의 owner/epoch와 terminal measured anchor가 일치한
서로 다른 training 6개와 완전 미사용 validation 2개를 수집했다. 오른팔 nominal
URDF만 사용한 해는 training RMS/max `6.112/9.751 mm`로 거절됐지만, 검증된
왼팔 workcell-to-camera를 고정하고 오른팔 mount와 식별 가능한 shoulder/elbow/
wrist-flex 영점만 training으로 적합한 해는 training RMS/max
`2.211/2.804 mm`, validation RMS/max `2.781/3.272 mm`, validation 회전 max
`0.966 deg`로 통과했다. 추정 영점은 각각 `-2.492/+2.615/+1.268 deg`이며
base와 wrist-roll은 gauge freedom 때문에 0으로 고정했다. training 하나씩을
제외한 민감도 검사에서도 validation max는 `3.131..4.176 mm`였다. 결과는
`artifacts/calibration/top_eye_to_hand_20260825_r0c/`에 보존하지만 실제 동작
승인은 아니며 `motion_authorized=false`를 유지한다.

같은 등록 후보를 workcell shadow에 적용한 완전 미사용 validation 2개에서는
작업대 x/y 오차 최대 `3.272 mm`, yaw 오차 최대 `0.515 deg`로 통과했다. 이는
gripper marker의 workcell 좌표 검증이며, 오른팔 FK를 실제 tabletop 물체 motion
target으로 사용하는 승인은 아니다. 이어서 위 `OBSERVE_CLEAR` plan-only와
supervised 왕복도 통과했으므로 R0-C의 wrist camera 이전 범위는 완료했다.

같은 고정 조건의 worktable 보정은 `calibration_01..06,07b,08`만 plane fit에
사용하고 `validation_01..02`는 해에 넣지 않았다. 영상 경계가 `4.25 px`였던
초기 `calibration_07`은 기록만 보존하고 제외했다. 10 mm coverage-hull inset
뒤 독립 검증 영역은 `377.296x371.513 mm`, plane RMS/max는
`1.128/3.846 mm`, validation metric XY max는 `1.608 mm`, plane-height max는
`3.259 mm`로 통과했다. 따라서 nominal 300 mm 수건과 사방 30 mm 영역을 두
축 모두 포함한다. 검증된 1280x960 intrinsic과 homography를 active runtime
config로 승격했다. Pi 재빌드 뒤 Top `1280x960@30`, 두 wrist `640x480@30`이
동시에 `STREAMING`했고 reconnect와 capture/decode error는 없었다.
`motion_authorized=false`는 유지한다.

## 현재 blocker

1. Top left/right metric registration, workcell shadow와 `OBSERVE_CLEAR` 실기
   왕복은 통과했다. 다만 left wrist는 gripper가 영상 일부를 영구 가리며,
   right wrist는 intrinsic, eye-in-hand와 URDF optical frame가 없다. 오른팔 FK의
   tabletop 물체 motion target 사용도 별도 독립 검증 전에는 금지한다.
2. 1차 fold의 약 300 mm moving-edge separation과 두 fold의 약 150 mm arc
   높이를 양팔이 동시에 추종할 수 있는지 실제 MoveIt으로 검증되지 않았다.
3. jaw gap, 단일/다층 접촉, slip, 장력, 테이블 마찰과 수건 물성이 미측정이다.

## 바로 다음 작업

1. 수건 side tolerance, 두께, 질량, 재질과 상태 조건을 측정해 기존 task
   contract에 증빙과 함께 반영한다.
2. right wrist intrinsic/eye-in-hand/optical frame를 완성하고 left wrist의
   고정 gripper 가림을 calibration/관측 계약에 명시한다.
3. 오른팔 FK의 tabletop 물체 좌표를 독립 target으로 검증한다.
4. 300 mm rigid proxy로 x/y축·양 방향·팔 배정의 MoveIt plan-only go/no-go를
   수행한다.
5. 그 결과가 통과한 작업대 배치에서 실제 수건 데이터 수집과 mask backend를
   시작한다.
6. R1 episode에 primitive 전후 state·outcome을 함께 기록하고, R2에서 Isaac Lab
   S0/S1 재현성과 heuristic unfolding baseline을 만든다. 임의 구김용 learned
   policy는 이 공통 action/observation 계약 위에서 R5 전에 학습·비교한다.
