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

수건용 Isaac Lab은 [수건 로드맵](../docs/ROADMAP.md)의 R2 S0–S3 순서를
따른다. 실제 모터 명령은 사용하지 않으며 모든 결과는
`motion_authorized=false`다.

S0는 `tools/run/build_towel_isaac_s0_manifest.py`가 strict MoveIt artifact,
등록 R0G URDF, 작업대 형상과 12축 replay SHA를 고정한다. vectorized reset,
articulation replay, Top FOV와 transition collision gate는 모두 통과했다.

S1의 실측값은 `config/towel_isaac_s1_material.json`, 그리퍼 형상과 Q0는
`config/so101_gripper_geometry.candidate.json`에 있다. 최종 실행은 Isaac Lab
CoupledMJWarp+VBD와 실제 jaw STL을 사용한다. fixed/moving jaw가 같은 수건
입자를 실제로 접촉한 경우에만 실물에서 확인한 “닫힌 동안 유지, Q0로 열면 해제”
조건을 `nodal_kinematic_target`으로 적용한다. 근접 fallback은 없다.

최종 1차 접기는 다음 순서다.

1. 로봇 가까운 변의 양끝을 집고 수건 전체를 든다.
2. 팔을 전진시키며 자유단을 작업대에 접촉시킨다.
3. TCP 높이를 유지한 채 36 mm 전진해 작업대 마찰로 자유단을 편다.
4. L 형상을 완성하며 표면 드래그 이동의 절반인 15 mm를 선보정한다.
5. 팔 방향을 바꿔 윗단을 덮고 Q0로 연 뒤 수직 이탈한다.

3회 독립 실행의 최악값은 layer `51.609/48.391`, paired-vertex p95
`16.398 mm`, 높이 `26.488 mm`, footprint 폭 `156.332 mm`,
terminal Z/curl 0이며 독립 실행 간 전체 node 최대 차이는 `0.0116 mm`다.
11.3 mm 미세보정은 비율 개선 없이 p95를
`18.523 mm`로 악화시켜 사용하지 않는다.
고정 요약은
`artifacts/bimanual/planning/towel_first_fold_surface_drag_r2_s1_summary.json`이다.

FK 진단:

```bash
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:. \
python3 tools/run/diagnose_towel_suspended_gravity_fold_kinematics.py \
  --output tmp/towel_first_fold_surface_drag_full_fk.json
```

headless 물리 검증:

```bash
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/run_towel_s1_vertex_patch_lift.py \
  tmp/towel_isaac_s0_manifest_final_20260828.json \
  --output tmp/towel_first_fold_surface_drag.json \
  --place-release --self-contact \
  --physics-backend newton-coupled-vbd \
  --kinematic-replay tmp/towel_first_fold_surface_drag_full_fk.json \
  --urdf-override \
  artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf \
  --actual-jaw-mesh-contact --newton-rubber-friction 100 \
  --environment-count 1 --disable-cubric-visual-sync --viz none
```

GUI 확인은 마지막 `--viz none`을 `--viz kit --keep-open`으로 바꾼다.
실측 물성 재현 도구는
`tools/setup/isaac/run_towel_s1_material_calibration.py`와
`tools/setup/isaac/run_towel_newton_material_calibration.py`다.

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
