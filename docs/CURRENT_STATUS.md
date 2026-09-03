# 현재 상태

기준일: 2026-09-03

R0 실기 결과, 최종 plan-only gate와 R1 관측 검증은 이 문서에 통합한다.
현재 자동검증 결과는 repository test와 `VERIFICATION_MATRIX.md`로 확인하며,
단계마다 별도 결과 문서를 추가하지 않는다.

## 프로젝트 상태

R0 작업셀·양팔 기구·카메라·MoveIt plan-only와 R1 실제 Top 관측 검증은 완료됐다.
실제 목표 수건은 면 100%, nominal 300×300 mm이며 네 변은
`304/296/304/296 mm`다. 실측값은 질량 평균 `56.7 g`, 4겹 두께 중앙값
`18.1 mm`, 작업대 유효 정지/이동 마찰 `0.701/0.737`, 100.1 mm
edge-release 평균 `0.181 s`, 45° cantilever overhang 평균 `36.33 mm`다.
원시값과 simulator 파생값은 `config/towel_isaac_s1_material.json`에 고정한다.

R2 S0의 vectorized reset, articulation, FOV와 transition collision gate는 통과했다.
S1은 Isaac Lab CoupledMJWarp+VBD, 등록 R0G 양팔 URDF, 실제 jaw STL, 실측 Q0
간격 `16.7 mm`, fixed-jaw `2.2 mm` 고무패드를 사용한다. fixed/moving jaw가 같은
수건 입자를 실제로 접촉한 뒤에만 실물에서 확인한 “닫힌 동안 유지, Q0로 열면 해제”
조건을 `nodal_kinematic_target`으로 적용한다. 근접 입자 fallback이나 임의
vertex attachment는 최종 경로에서 사용하지 않는다.

2026-09-03 최종 1차 접기는 아래 양끝을 집어 완전히 든 뒤, 자유단을 작업대에
접촉시키고 36 mm 전진 표면 드래그로 펴며 L 형상을 만든 다음, 방향을 바꿔 윗단을
덮는 경로다. 표면 드래그가 만든 약 30 mm 이동의 절반인 15 mm를 되접기 전에
선보정한다. 최종 3회 독립 실행의 최대 layer 비율은 `51.609%`, paired-vertex
p95 XY 오차는 `16.398 mm`, 높이는 `26.488 mm`, footprint 폭은
`156.332 mm`였다. 모두 `55/45`, `30 mm`, `30 mm`, `180 mm` gate를
통과했고 terminal Z/curl amplitude와 fraction은 모두 0이었다. Q0 개방 잔차는
`1.30e-5 rad` 이하이고 해제 뒤 patch가 jaw를 따라가지 않았다. 독립 실행 간 전체
1,024-node 최대 차이는 `0.0116 mm`로 `1 mm` 반복 gate를 통과했다. 11.3 mm
미세보정은 비율 개선 없이 p95를 `18.523 mm`로 악화시켜 폐기했다.
고정 요약은
`artifacts/bimanual/planning/towel_first_fold_surface_drag_r2_s1_summary.json`이다.

이 결과는 시뮬레이션 1차 접기 기준 성공본이다. 실제 로봇 motion은 여전히
`motion_authorized=false`이며, 순수 마찰계수만으로 집기 유지가 검증됐다는 뜻도
아니다. R2 추정 진행률은 `75%`다. 남은 핵심은 solver/material seed 반복 성공률,
2차 접기, S2 material randomization과 S3 환경·정책 계약이다.

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
| 태스크 범위 | 실측 304/296/304/296 mm, 질량 평균 56.7 g, 4겹 두께 평균/중앙값 18.26/18.1 mm, 작업대 유효 마찰 0.701/0.737, 100.1 mm edge-release 평균 0.181 s, 45° overhang 평균 36.33 mm, 면 100%, dry/unwashed | 면내 인장 응답은 S1 완료 전 필수로 승격하지 않고 randomization 범위로 보존 |
| cloth contact | 좌우 1겹·4겹 current-pose hold 2회와 가벼운 pull PASS | 자동 open/close-to-contact, 동적 slip·장력 승격 |
| annotation 계약 | schema, validator, deterministic split와 실제 capture/episode manifest | R2 sim/real action-outcome episode에 동일 identity 계약 적용 |
| 수건 데이터셋 | 개발 595장 중 검수 train 540장(기존 103 + assisted 승인 437, 제외 7) + held-out 38장 중 검수 35장·robot OOD 3장 + 실제 3-frame 5 episode/15장; split leakage 0 | R2 sim/real episode 계약 유지 |
| segmentation | 기존 backend towel 30/30·empty 5/5, IoU 평균 0.980284·최저 0.965564; train 540장 expanded YOLO26n-seg도 30/30·5/5, IoU 평균 0.980166·최저 0.966108로 103장 baseline의 0.979250·0.942682보다 개선 | 새 독립 test와 실시간 카메라 검증 뒤 runtime backend 결정, 무검수 pseudo-label 금지 |
| corner/topology | outline quadrilateral·metric area·flatness, non-flat/fold `ALIGNED` 0건; 검증된 action context에서만 fold outline 판정 | 들림·다층 ambiguity는 wrist/RGB-D 근거 전까지 UNKNOWN |
| temporal state | 실제 5 episode/15장 3-frame 상태 일치; 1차 IoU min 0.903769, 2차 min 0.859693 | 실제 primitive 전후 동일 계약 재사용 |
| observation lifecycle | `OBSERVE_CLEAR→primitive→RETREAT_AND_SETTLE→REOBSERVE_CLEAR`, freshness·settle·identity·3-frame fail-closed gate 실데이터 PASS | R3 primitive runner와 연결 |
| fold plan-only | r0g strict PASS: 1차 양팔 아래→위, correction 8개, 2차 오른팔 오른쪽→왼쪽; 846구간·12,552상태·미승인 접촉 0 | R3 무수건 dry-run; 실제 cloth/contact 승인은 별도 |
| Isaac/학습 | coupled VBD actual-contact gate, 실측 hold-until-Q0-open retention과 중력+표면 드래그 1차 접기 3회 PASS. 최악값: layer `51.609/48.391`, p95 `16.398 mm`, 높이 `26.488 mm`, 폭 `156.332 mm`, terminal Z/curl 0. R2 `75%` | seed/material 반복 성공률→2차 접기→S2 randomization→S3 환경·정책 계약 |
| 조작 primitive | 미구현 | 개별 plan-only→supervised 제한 반복 |
| 펼쳐진 수건 2회 접기 | 미구현 | R4 standalone fold gate 통과 |
| 펼치기·평탄화 | 미구현 | R5/R6 단계 성공 기준 통과 |
| 통합 복구 | offline 유한 상태기계 | 실제 실패 signature와 feedback 연결 |
| hardware-free CI | 수건·YOLO·S0/S1 host/replay 계약과 ROS overlay 유지; 관련 정적 회귀 40개 PASS | cloth backend 결정 뒤 전체 회귀와 8환경 반복 재실행 |

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

최신 canonical 후보는 300 mm 수건을 검증된 작업대 중앙에 두고 로봇 가까운
아래쪽 moving edge 양 끝을 양팔로 잡아 먼 위쪽으로 먼저 접는다. clear 재관측과
bounded correction 뒤에는 짧아진 두 겹 오른쪽 edge의 중앙을 오른팔이
오른쪽→왼쪽으로 접는다. X/Y 부호는 artifact의 좌표 재현용 metadata에 함께 남긴다.

SO-101 한 팔은 5-DOF이므로 임의 exact 6D pose를 주장하지 않는다. 각 phase는
TCP xyz와 jaw opening-line yaw를 검사하고 full 6D FK를 기록한다. contact와
pregrasp에는 70 deg downward cone을 적용하고, attached transfer·laydown에는
최대 90 deg를 명시한다. 2차 contact는 bundle 높이에서 시작하되 laydown은
기존 dense Cartesian 검증과 실제 도달 한계를 반영한 TCP 40 mm release다.

data-fit candidate URDF에서 초기 clear 자세의 카메라 마운트와 팔 메시가 최대
약 16.2 mm 겹친 이전 strict 진단은 그대로 fail-closed 증거로 보존한다. 이후
등록 완료 `so101_dual_preview_right_registered_r0g.urdf`와 manifest, workcell
shadow, right tabletop validation을 복원해 최종 runner를 다시 실행했다. 선택된
canonical 후보는 1차 양팔 아래→위, 2차 오른팔 오른쪽→왼쪽이며 bounded
correction 8개를 포함해 846개 경로 구간과 12,552개 strict 상태를 통과했다.
미승인 접촉은 0건, 허용된 얕은 동일팔 mesh 접촉 최대는 `3.810 mm / 4 mm`,
dense TCP 경로 편차 최대는 `2.875 mm / 4 mm`였다. 시작 departure는 FCL만
통과하던 어깨 단독 우회를 제거하고 베이스+어깨 동시 이동과 25% 베이스 부분 복귀로
교체해 PhysX에서도 금지 접촉 0을 확인했다. 결과는
`artifacts/bimanual/planning/towel_bimanual_then_single_robot_near_to_far_r2_s0.json`이며
SHA-256은 `c9d9d93974996603ab1c64b3711d5c32c71f8e3f8e35d4cdfc65dc459c35632b`다.
실제 controller·resident motion API는 사용하지 않았고 `motion_commands=0`이다.

full-FK 결과는 1차·2차를 한 artifact에 기록하고 RViz에서 `first`, `second`,
`both`로 나누어 볼 수 있다. RViz marker는 항상 사용할 수 있지만 strict MoveIt
artifact가 아닌 full-FK-only 관절 pose animation은 충돌 미검사 경고와 명시적
옵션 없이 publish하지 않는다.

strict MoveIt 경로의 dense 검사는 각 관절 상태의 충돌뿐 아니라 각 active TCP가
인접 task waypoint chord에서 벗어난 거리도 검사한다. 최대 허용 편차는 기존
dense Cartesian 검증과 같은 `4 mm`이며 contact·attachment 구간에만 강제한다.
free-space departure는 편차를 기록하되 collision과 joint limit을 적용한다. 따라서
phase endpoint만 맞고 수건을 든 중간 TCP가 크게
휘는 OMPL 경로는 거부한다.

기존 로컬의 1차·2차 독립 candidate sweep runner는 canonical geometry와 중복되어
복사하지 않았다. 그 결과에서 채택한 inset, sample 수, arm/direction, release
높이와 dense TCP audit만 공통 planner에 통합했다. 로컬의 정밀 camera-mount mesh
URDF도 clear pose에서 큰 self-collision을 만들기 때문에 등록 모델로 승격하지
않았다. RViz MarkerArray 설정, stage별 시각화와 execution-disabled launch는 새
canonical 형식으로 이식했다.

입력 파일명 `towel_task_contract.candidate.yaml`은 유지하되 내부 status는 실제
3-frame 검증을 반영해 `R1_OBSERVATION_CANDIDATE`로 승격했다. 자동·동적 contact와
수건 motion은 여전히 승인되지 않았으며 `motion_authorized=false`를 유지한다.

## R0 종료 시 남은 비승인 항목

다음 항목은 누락이 아니라 소비 단계까지 명시적으로 연기한 gate다.

1. left wrist의 새 staged metric 검증은 R1에서 실제 multi-view metric fusion 또는
   motion correction에 쓰기 전에 연결한다. 양쪽 wrist/robot pixel mask는 실제
   clear-view 거절 실패가 필요성을 보일 때만 추가한다.
2. 자동 jaw open/close-to-contact, 동적 slip·장력과 테이블 마찰은 R3 primitive
   전에 측정·commission한다. 현재 정적 retention만 승인됐다.
3. rigid proxy는 cloth attachment·변형을 증명하지 않는다. surface cloth와
   vertex-patch attachment, 물성 randomization은 R2에서 검증한다.
4. 케이블은 명시적 mesh가 아니라 operator-reviewed joint envelope로 검사했다.
   실제 primitive dry-run 전에 케이블·접촉 gate를 다시 확인한다.

## 바로 다음 작업 — R2

1. 현재 15 mm surface-drag 보정 성공본을 solver/material seed별 반복 실행해 성공률을 기록한다.
2. 같은 실제 jaw/Q0/고무패드와 release gate로 2차 접기 full-FK와 Isaac 경로를 연결한다.
3. 1·2차 scripted baseline을 고정한 뒤 S2 material randomization을 수행한다.
4. S3의 observation/action/reward/termination 계약과 제한된 정책 학습을 시작한다.
5. 실제 모터 실행은 별도의 R3 dry-run·supervised 승인 전까지 금지한다.
