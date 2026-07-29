# Isaac Sim assets

검증 기준은 Isaac Sim 6.0.1이다.

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
  tools/generate_isaac_overhead_workcell.py
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
license는 root `THIRD_PARTY_NOTICES.md`와 `LICENSE`를 확인한다.
