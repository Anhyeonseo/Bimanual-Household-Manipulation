# 현재 상태

기준일: 2026-08-27

최신 별도 파일로 추적되는 자동검증 결과는
[2026-08-20 수건 software foundation 검증](test-results/2026-08-20-towel-software-foundation.md)이다.
이후 R0 실기 결과와 최종 plan-only gate는 이 문서에 통합했고 새 결과 문서는
추가하지 않았다. 현재 회귀시험 결과는 repository test로 확인한다.

## 프로젝트 상태

최종 목표 수건은 nominal 300×300 mm로 확정됐다. 실제 네 변, 근사 두께,
면 100%·건조·미세탁 조건과 좌우 1/4겹 정적 retention을 등록했다. 질량,
작업대 마찰, 자동 contact와 동적 slip·장력 한계는 아직 측정되지 않았다.
R0 물리·카메라·작업셀 기반과 canonical 접기 task-pose 후보를 통합했다. 접기
순서는 1차 양팔 아래→위, 2차 가까운 한 팔 오른쪽→왼쪽 edge-midpoint다. software와
full-FK IK 검증은 통과했지만 strict MoveIt 최종 승격에는 로컬에 없는 등록 완료
URDF·workcell shadow·right tabletop artifact가 필요하다. 실제 수건 motion은
승인되지 않았고 `motion_authorized=false`다.

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
| 태스크 범위 | 실측 304/296/304/296 mm, 면 100%, dry/unwashed, 최종 nominal 150×150 mm | 질량은 동적 모델/primitive 전 측정 |
| cloth contact | 좌우 1겹·4겹 current-pose hold 2회와 가벼운 pull PASS | 자동 open/close-to-contact, 동적 slip·장력 승격 |
| annotation 계약 | schema, validator, deterministic split manifest | 실제 episode index 생성 |
| 수건 데이터셋 | synthetic example만 있음 | 구김·평탄·1차/2차 fold 실제 데이터 |
| segmentation | reviewed polygon→metric observation backend | 실제 mask, component, border 검사 |
| corner/topology | 순수 기하와 confidence gate | 가림·말림·다층 ambiguity 검증 |
| temporal state | 3-frame 동일 상태 gate | timestamp, spread, settle, hysteresis |
| observation lifecycle | Top metric 영역·실기 clear 왕복과 무가림 재관측 PASS | camera phase와 runtime 상태기계 구현 |
| fold plan-only | 후보: 양팔 1차·오른팔 edge-midpoint 2차 full-FK IK PASS; strict MoveIt은 등록 artifact 부재로 BLOCKED | 등록 완료 URDF/shadow/tabletop evidence 복원 뒤 dense collision 재실행 |
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

R0-E에서는 right wrist `640x480` intrinsic을 43개 완전 검출 영상에서
결정적 34 training/8 held-out split으로 검증했다. training OpenCV RMS는
`0.550 px`, held-out per-view max는 `0.891 px`였다. 최초 right eye-in-hand
세트는 torque-off joint state로 수집되어 R0-C의 torque-on 오른팔 등록과 load
state가 달랐고, 그 결과 training 위치 RMS/max `14.005/30.739 mm`로 거절됐다.
사진 크기·1280 Top intrinsic·URDF·GridBoard `95x120 mm`를 재검증한 뒤,
resident hold owner/epoch와 terminal measured anchor가 일치하는 6개 training과
완전 미사용 validation 2개를 재수집했다. 최종 training 위치 RMS/max는
`8.463/13.517 mm`, 회전 RMS/max는 `1.296/2.053 deg`였고, validation 위치
RMS/max는 `5.473/5.625 mm`, 회전 max는 `0.898 deg`로
`EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED`를 통과했다. 결과를
`right_wrist_camera_mount_center_link` 기준 xyz
`0.000752553/0.012058633/-0.008833815 m`, optical rpy
`0.018617155/0.007963772/3.122102339 rad`로 dual URDF에 반영했다. right capture
조립기와 solver는 이제 torque-off source를 fail-closed로 거절한다.

R0-F에서는 동일 보드를 움직이지 않은 채 팔이 비운 Top stage와 controlled
resident hold의 wrist stage를 시간 순서대로 결합해, 구조적인 Top 가림 없이
tabletop 좌표를 검증했다. 동시촬영 controlled 01/02만 translation correction
학습에 사용하고 staged validation 01/02는 완전히 제외했다. 회전을 고정한
그리퍼 기준 평행이동 보정은 `[5.993, 16.265, 2.143] mm`였고 크기는
`17.466 mm`였다. 독립 validation의 XY RMS/max는 `10.327/12.309 mm`, Z max는
`12.313 mm`, yaw max는 `1.232 deg`였으며, 둘 모두 wrist 영상 경계 여유
`≥30.5 px`에서 측정됐다. 따라서 right wrist optical xyz를 mount-center 기준
`[-0.005240079, -0.001776520, -0.000017289] m`로 갱신했고 회전은 그대로
유지했다. 이 결과는 rigid-proxy plan-only 좌표만 승인하며 실제 motion은
승인하지 않는다.

left wrist는 기록이 없는 상태가 아니다. `640x480` intrinsic은 44장 중 43장을
사용해 RMS `0.5666 px`로 통과했고, 과거 W3의 10 training/2 validation
eye-in-hand 결과는 validation 위치/회전 RMS `8.18 mm/1.63 deg`로 URDF에
반영돼 있다. 다만 현재 저장소에는 당시 원본 candidate/session artifact가 없고
이번 right처럼 독립 staged tabletop 교차검증을 새로 수행하지 않았다. R0의
metric 경로가 Top+right wrist이므로 재수집은 연기하되, left wrist를 metric
target fusion이나 motion correction에 쓰기 전 같은 staged gate를 통과시킨다.

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

R0-G canonical 후보는 300 mm 수건을 검증된 작업대 중앙에 두고 로봇 가까운
아래쪽 moving edge 양 끝을 양팔로 잡아 먼 위쪽으로 먼저 접는다. clear 재관측과
bounded correction 뒤에는 짧아진 두 겹 edge의 중앙을 가까운 오른팔이
오른쪽→왼쪽으로 접는다. 왼팔과 왼쪽→오른쪽 방향은 bounded fallback 후보로만
유지한다. X/Y 부호는 artifact의 좌표 재현용 metadata에만 남긴다.

SO-101 한 팔은 5-DOF이므로 임의 exact 6D pose를 주장하지 않는다. 각 phase는
TCP xyz와 jaw opening-line yaw를 검사하고 full 6D FK를 기록한다. contact와
pregrasp에는 70 deg downward cone을 적용하고, attached transfer·laydown에는
최대 90 deg를 명시한다. 2차 contact는 bundle 높이에서 시작하되 laydown은
기존 dense Cartesian 검증과 실제 도달 한계를 반영한 TCP 40 mm release다.

software regression 79개와 canonical 후보의 전체 full-FK IK는 통과했다.
하지만 strict MoveIt 진단은 저장소의 data-fit candidate URDF에서 초기 clear
자세의 카메라 마운트와 팔 메시가 최대 약 16.2 mm 겹쳐 fail-closed됐다. 최종
runner가 요구하는 등록 완료 URDF manifest, workcell shadow, right tabletop
validation artifact는 Git과 로컬에 없으므로 PASS artifact를 만들지 않았다.
실제 controller·resident motion API는 사용하지 않았고 `motion_commands=0`이다.

입력 파일명과 내부 status인 `towel_task_contract.candidate.yaml`/
`R0_STATIC_CONTACT_CANDIDATE`는 의도적으로 유지한다. 자동·동적 contact가 아직
승인되지 않은 motion-lock 상태이며, R0 최종 완료에는 strict plan-only PASS와
`motion_authorized=false`가 함께 필요하다.

## R0 종료 시 남은 비승인 항목

다음 항목은 누락이 아니라 소비 단계까지 명시적으로 연기한 gate다.

1. left wrist의 새 staged metric 검증과 양쪽 wrist robot mask/confidence는 R1에서
   실제 multi-view metric fusion 또는 motion correction에 쓰기 전에 연결한다.
2. 자동 jaw open/close-to-contact, 동적 slip·장력과 테이블 마찰은 R3 primitive
   전에 측정·commission한다. 현재 정적 retention만 승인됐다.
3. rigid proxy는 cloth attachment·변형을 증명하지 않는다. surface cloth와
   vertex-patch attachment, 물성 randomization은 R2에서 검증한다.
4. 케이블은 명시적 mesh가 아니라 operator-reviewed joint envelope로 검사했다.
   실제 primitive dry-run 전에 케이블·접촉 gate를 다시 확인한다.

## 바로 다음 작업 — R1

1. `OBSERVE_CLEAR→primitive→RETREAT_AND_SETTLE→REOBSERVE_CLEAR` phase와 freshness
   계약을 구현한다.
2. 통과한 작업대 배치에서 실제 수건 episode를 수집하고 mask/component/border,
   robot occlusion과 topology backend를 검증한다.
3. left wrist 고정 gripper 가림과 양쪽 wrist mask를 confidence에 반영하고,
   가려짐·들림·다층 ambiguity를 `UNKNOWN`으로 유지한다.
4. primitive 전후 state·outcome을 기록해 R2 Isaac Lab S0/S1과 heuristic
   correction baseline, 이후 goal-conditioned residual policy가 같은 계약을
   소비하게 한다.
