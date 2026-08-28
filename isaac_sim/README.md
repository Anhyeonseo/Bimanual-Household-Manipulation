# Isaac Sim assets

검증 기준은 Isaac Sim 6.0.1이다.

## 현재 자산과 Isaac Lab의 경계

`assets/so101_new_calib/so101_rl_asset.usd`와
`tools/setup/isaac/isaac_*gate2*`, `isaac_scripted_grasp_test.py`는 선행 단일팔
rigid box grasp의 수동/scripted Gate 2·3 자산이다. 파일명에 `rl` 또는
`training`이 있어도 vectorized environment, observation/action/reward,
termination, trainer, checkpoint와 held-out evaluation이 없으므로 Isaac Lab
강화학습 구현으로 간주하지 않는다. 과거 rigid grasp 회귀·참고용으로 보존하며
300 mm 수건 학습의 성공 증거로 재사용하지 않는다.

수건용 Isaac Lab은 [수건 로드맵](../docs/ROADMAP.md)의 R2에 따라 별도 S0~S3
순서로 구축한다. 첫 학습 대상은 저수준 joint control이 아니라 임의 구김에서
승인된 양팔 primitive와 grasp/placement 파라미터를 선택하는 정책이다. S1의
scripted attachment/release가 실제 관측 metric과 맞지 않으면 S2/S3 학습을
진행하지 않는다.

첫 S0 host 계약은 `tools/run/build_towel_isaac_s0_manifest.py`에 있다. strict MoveIt
artifact의 motion lock과 300 mm 배치, 12축 상태, phase별 joint target을 검사하고
검증 worktable geometry, strict MoveIt/FCL shallow-mesh 계약과 동일한 vectorized
reset/replay batch의 source/replay/reset SHA를 기록한다.
현재 S0는 최신 아래→위·오른팔 strict artifact SHA를 고정한다. 이전 위→아래·왼팔
후보의 S1 물리 smoke는 보존하지만 최신 manifest의 S1 완료 증거로 섞지 않는다.
`run_towel_s0_vectorized_reset.py`는 이를 Isaac Lab 3.0/Isaac Sim 6.0.1 PhysX에서
소비한다. 8-env rigid proxy reset 2회의 최대 위치 오차는 `7.45e-9 m`이고 결과
SHA가 일치했다. `run_towel_s0_articulation_replay.py`는 manifest에 path/SHA가
고정된 최신 r0g 양팔 URDF를 사용해 canonical 12축 이름/순서를 고정하고 114개 phase를 8환경에서
재생했다. 최대 관절 오차는 `0 rad`이며 `--viz kit --keep-open`으로 같은 장면을
GUI에서 확인할 수 있다. 이 replay에서는 중력과 robot collision을 명시적으로
끄므로 상태는 `S0_ISAACLAB_ARTICULATION_REPLAY_PASS_COLLISION_NOT_RUN`이다. Top
camera FOV는 최소 image margin `29.409 px`, calibrated board margin `5.756 mm`로
PASS했다. rigid proxy는 manifest의 작업대 pose로 명시 reset하며 위치 오차가
`1e-6 m`를 넘으면 중단한다. `run_towel_s0_collision_replay.py`는 PhysX contact
report와 13/13 body API
coverage/liveness를 확인한 뒤 원본 MoveIt trajectory를 0.02 rad 이하로 재보간한
3,383개 표본을 self/table collision이 켜진 상태로 재생한다. strict MoveIt/FCL에서만
허용된 shallow-mesh 쌍과 4 mm 한계는 manifest SHA로 고정하며 다른 접촉에는 적용하지
않는다. 결과는 금지 접촉 0의 `S0_ISAACLAB_TRANSITION_COLLISION_PASS`다. GUI에서는
`--stop-on-first-forbidden --keep-open --viz kit`으로 첫 충돌 pose와 빨간 표시를
확인한다.

`run_towel_s1_surface_drop_settle.py`는 S0 manifest의 작업대 형상을 그대로 사용해
31×31 element/1,024-node surface-deformable mesh의 drop/settle만 fail-closed로
검사한다. concrete collision shape가 기본 offset 합계 때문에 작업대보다 `20 mm`
위에서 멈추는 초기 실패를 hover gate로 차단했고, mm 단위 contact/rest offset을
고정한 뒤 8환경에서 40 step 이하 settle, clearance `1.5 mm`, 환경 간 최대 차이
`8.94e-8 m`로
`S1_ISAACLAB_SURFACE_DROP_SETTLE_SMOKE_PASS_MATERIAL_UNCOMMISSIONED`를 기록했다.
후보 질량·마찰·탄성은 실측 보정 전이다. 최신 canonical의 attachment와 1차
lift/place/release는 아래 runner로 통과했지만 2차 fold와 학습은 아직 통과하지 않았다.

`run_towel_s1_vertex_patch_lift.py`는 같은 1,024-node surface mesh를 안착 높이에서
시작해 좌우 gripper rigid link 아래 전용 attachment frame에 각각 9개 vertex를
명시적으로 붙이고, r0g의
`first_contact→first_fold_01` 변위를 따라 낮게 드는 smoke다. 첫 실행의 단순 Xform
target은 화면상 lift와 달리 PhysX가 rigid actor가 아니라고 경고했으므로 결과를
폐기했다. direct articulation-link 방식은 attachment snap 최대 `0.125 mm`, patch
lift 최소 `20.871 mm`, gripper 추종 오차 최대 `0.782 mm`, 8-env patch 차이 최대
`0.057 mm`로 PASS했다. 자유 면 전체 차이 `8.660 mm`는 미보정 물성 진단값이며
이전 후보 증거로 보존한다. 최신 manifest에 `--place-release`를 적용한 1차 fold는
snap `0.046 mm`, patch lift 최소 `20.933 mm`, 강체회전 포함 추종 오차 `0.426 mm`,
release 뒤 patch–jaw 거리 최소 `63.974 mm`, 최종 clearance `1.500 mm`로 PASS했다.
8-env 자유 면 전체 차이 `31.648 mm`는 exploratory `20 mm`를 넘었으므로 full-shape
결정성은 미통과다. `--self-contact` 진단은 topology 두 칸 filter와 `1/240 s`에서
비이웃 간격 최소 `3.007 mm`, table clearance `1.500 mm`를 유지했지만 20초 뒤
최대 속도 `0.0294 m/s`로 `0.015 m/s` settle gate를 통과하지 못했다. 실측 물성 전에는
임계값을 완화하지 않는다.

현재 미보정 후보값은 질량 `0.100 kg`, 정지/동적 마찰 `0.50/0.40`, 영률
`1.0 MPa`, 포아송비 `0.30`, surface thickness `3 mm`다. 이 값은 물리 충실도
증거가 아니다. 다음 실행 전 최소 실측은 수건 질량, 4겹 두께 5지점과 작업대
가로/세로 정지·동적 마찰 각 3회다. 직접 늘어남, 영률·포아송비와 수건-수건
마찰은 우선 일반값으로 두며, 처짐·낙하시간은 최소 보정 후 필요할 때 추가한다.
측정 전에는 self-contact solver를 더 조정하거나 S2/S3로 진행하지 않는다.

공식 3.0 beta installer는 현재 전용 venv에 PyTorch `2.10.0+cu128`과 coverage
`7.6.1`을 설치하지만 Isaac Sim 6.0.1 package metadata는 각각 `2.11.0`과 `7.4.4`를
요구해 `pip check` 경고가 남는다. GPU headless empty stage와 위 PhysX reset은 실제
PASS했지만 이 packaging 불일치를 숨기지 않으며, 이후 업데이트 전마다 두 smoke를
다시 실행한다.

## SO-101 왼팔 stage

```text
assets/so101_new_calib/so101_new_calib.usda
```

이 stage에는 `/so101_new_calib/Geometry` articulation과
`/Graph/ROS_JointStates` OmniGraph가 저장돼 있다.

- publish: `/isaac/joint_states`
- subscribe: `/isaac/joint_command`
- drive stiffness/damping/maxForce: `1000` / `100` / `10`
- arm target: `0 deg`
- gripper target: `-10 deg`
- wrist flex q0: upstream `-64.898281239 deg`를 model origin에 흡수

2026-07-30 eye-to-hand 외부 계측 보정은 custom camera/workcell/OmniGraph를
보존하기 위해 전체 URDF 재임포트 대신 q0를 소유하는 두 authored layer에만
동기화했다.

- visual baseline: `payloads/base.usda`의 `wrist_link` transform
- physics anchor/limit: `payloads/Physics/physics.usda`의 `wrist_flex`
- raw 2048, bridge zero, drive target 0은 변경하지 않음

## 확정된 오버헤드 카메라 작업셀

사용자가 RViz에서 실물 조립 방향과 10.0 mm 홈 삽입을 확인한 정적 작업셀은
별도 레이어로 합성된다.

```text
assets/so101_new_calib/payloads/overhead_workcell.usd
/so101_new_calib/Workcell
```

Isaac Sim 6.0.1 Python으로 원본 STL 해시를 확인하며 다시 생성한다.

```bash
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/generate_isaac_overhead_workcell.py
```

이 레이어는 표시 전용이다. 기존 articulation, robotLinks, joint drive,
질량·관성에는 참여하지 않으며 collision API와 실제 `UsdGeom.Camera`
센서도 만들지 않는다. `top_camera_link`와
`top_camera_optical_frame`은 외부 파라미터 보정을 위한 고정 기준 프레임
뿐이다. 실제 카메라–베이스 외부 파라미터가 측정되기 전에는 이 프레임을
인식 기반 모션 권한에 사용하지 않는다.

GUI에서는 `assets/so101_new_calib/so101_new_calib.usda`를 열고
Stage의 `/so101_new_calib/Workcell` 아래에서 확정 형상을 확인한다.
작업셀은 물리 시뮬레이션 대상이 아니므로 `Play` 여부와 무관하게 정지해
있어야 한다. 이 형상은 2026-07-28 Isaac Sim 6.0.1 GUI에서 사용자가
실물 조립과 육안 정합했으며 결과는
`../docs/test-results/2026-07-28-overhead-camera-workcell.md`에 기록했다.

ROS 2 Bridge가 필요한 경우 Isaac Sim을 ROS 2 Jazzy 환경을 source한
terminal에서 실행한다. 전체 실행 순서와 joint mapping은
`../docs/checklists/PHASE_4_ISAAC_MOVEIT_INTEGRATION.md`를 따른다.

`assets/so101_new_calib`의 geometry는 TheRobotStudio SO-101 asset의
commit `fda892cba81032c46c40976a48c9ceadbf40a9ca`에서 가져왔다.
license는 `docs/THIRD_PARTY_NOTICES.md`와 root `LICENSE`를 확인한다.

## 양팔 q0 시각 정합 후보

양팔은 기존 STL 기반 매크로를 두 번 사용한 preview 전용 URDF로 먼저
확인한다. 아직 측정하지 않은 오른팔 base 변환이 MoveIt이나 실기 제어로
유입되지 않도록 별도 엔트리포인트이며 `ros2_control`을 포함하지 않는다.
좌우 팔 밑에는 동일한 arm-base STL을 하나씩 결합한다. 2026-07-28에 검증한
오버헤드 카메라 bottom/top tower는 위에서 봤을 때
`왼쪽 base - camera mount - 오른쪽 base` 순서로 가운데에 둔다. URDF 폐루프를
피하기 위한 논리적 parent는
왼쪽 base 하나만 사용한다. 검증된 왼팔 wrist-camera mount/frame도 기본으로
합성한다. 오른팔도 같은 camera-mount용 wrist STL과 같은 wrist joint origin을
사용한다. 다만 오른쪽 camera optical frame은 실제 카메라 설치와 eye-in-hand
보정 전까지 왼쪽 값을 임의 복제하지 않는다.

```bash
python3 tools/setup/isaac/generate_isaac_bimanual_preview_urdf.py
```

2026-08-13부터 이 simulation-only preview의 arm 10축 limit은
`config/bimanual_j1_operational_limits.approved.json` SHA256
`ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff`에
고정된다. Gripper limit은 의미 계약 전이므로 J1-L parity 대상에서 제외된다.
생성 manifest는 `runtime_change_authorized=false`를 유지하며, 활성 단일팔 MoveIt
launch나 실기 motion 권한을 변경하지 않는다.

**File > Import**에서 출력된 URDF를 선택한다. 오른쪽
**Model > ROS Package List** 표에는 생성기가 출력한 package name/path를
한 행으로 지정한다.
기본 오른팔 위치 `0 -0.232064146 0` m는 STL 경계로 계산한 CAD-fit 후보이다.
가운데 camera mount가 왼쪽과 오른쪽 base 홈에 각각 10 mm 삽입되는 위치이며,
이전 약 14 inch 중심 간격 후보를 대체한다. 실물 정합과 외부 계측 전까지는
`motion_authorized=false`이다. 실제 base `xyz/rpy`와 오른팔 joint zero는
Top 카메라 기반 외부 계측으로 각각 확정한 뒤 새 후보를 생성한다.
