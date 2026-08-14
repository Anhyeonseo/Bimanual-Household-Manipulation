# 양팔 J1-L 보수 운용한계 plan-only 후보

날짜: 2026-08-13

상태: **사용자 승인 / firmware·host·hardware·URDF·MoveIt·Isaac candidate parity PASS**

```text
motion_authorized=false
runtime_change_authorized=false
execution_api_used=false
```

## 판정

J0-D에서 사용자가 직접 확인한 cable-safe desired envelope의 양 끝을 각각
64 raw 안쪽으로 수축해 arm 5축의 J1-L 후보를 만들었다. 모든 후보는 q0 raw
2048을 포함한다. SHOULDER는 J1-W에서 검증한 unwrapped raw 좌표를 사용한다.

이 후보는 기계적 hard stop이나 J0-M physical outer envelope라고 주장하지
않는다. 사용자는 64 raw 수축안을 다음 검증 후보로 승인했다. active calibration과 policy runtime은 변경하지 않았다. 활성 단일팔 URDF의 확장
결과와 MoveIt launch도 유지했다. 승인 범위는 별도 no-output firmware와
simulation-only 양팔 preview, 미참조 MoveIt candidate에만 연결했다.

## Arm 5축 후보

| 팔 | 관절 | 후보 raw / unwrapped raw | 후보 rad | 좌표 |
|---|---|---:|---:|---|
| Left | Base | 1047..2977 | -1.535515..1.425068 | raw |
| Left | Shoulder | 1963..4123 | -0.130388..3.183010 | unwrapped raw |
| Left | Elbow | 350..2428 | -0.582913..2.604699 | raw |
| Left | Wrist Flex | 234..2320 | -0.417243..2.782641 | raw |
| Left | Wrist Roll | 651..2774 | -2.142971..1.113670 | raw |
| Right | Base | 1172..2932 | -1.343767..1.356039 | raw |
| Right | Shoulder | 1923..4124 | -0.191748..3.184544 | unwrapped raw |
| Right | Elbow | 361..2459 | -0.630466..2.587826 | raw |
| Right | Wrist Flex | 441..2374 | -0.500078..2.465107 | raw |
| Right | Wrist Roll | 813..2906 | -1.894466..1.316156 | raw |

## Gripper 판정

Gripper는 arm 관절처럼 자동 변환하지 않았다. raw 증가가 opening 방향이라는
실기 관측은 있지만 jaw gap mm, loaded contact, hysteresis 및 linkage mapping이
아직 하나의 좌표 계약으로 확정되지 않았다.

- 왼쪽: loaded close 1963, release 2009, task-open 3257
- 오른쪽: unloaded close 1907, task-open 3062; loaded grasp 명령 미검증

따라서 두 gripper 모두 `BLOCKED_SEMANTIC_GRIPPER_MAPPING_REQUIRED`다.

## 기존 왼팔 대표 경로 coverage

기존 offset 0.011 m Pick–Place manifest의 7개 phase, trajectory 1,031점을
SHA-bound로 전수 검사했다.

| 관절 | 경로 범위 rad | 후보 limit까지 최소 여유 rad | 판정 |
|---|---:|---:|---|
| Left Base | 0.000000..0.359298 | 1.065770 | PASS |
| Left Shoulder | 0.000000..2.482252 | 0.130388 | PASS |
| Left Elbow | 0.000000..1.238016 | 0.582913 | PASS |
| Left Wrist Flex | 0.000000..1.303966 | 0.417243 | PASS |
| Left Wrist Roll | -0.167215..0.145815 | 0.967855 | PASS |

이 결과는 기존 왼팔 명목 경로가 후보 arm limit 안에 있다는 뜻이다. 환경
collision, tracking envelope, 다른 물체/정책 workspace 또는 오른팔 경로를
승인하는 결과는 아니다.

## Evidence

- Reviewed J0-D manifest:
  `config/bimanual_j0_desired_envelope.reviewed.json`
  SHA256 `c1d6d41c402de15c0ac03ceca7c9eeb2d2ffe166dd794599e3fdc8b2db87a48e`
- J1-L candidate:
  `artifacts/joint_ranges/2026-08-13/j1_operational_limits_candidate_inset64.json`
  SHA256 `0dbaa9bc267a3b500e4686430b8b367f4d00d3afb61bd1a096c7fb1f651c4719`
- Left nominal route manifest:
  `artifacts/h2/2026-08-12/offset011_nominal_routes/manifest.json`
  SHA256 `94f6eb82eab531276518ab2409b1fbf97e43b7e72b8768f0c3ddfafc7028a297`
- Route coverage:
  `artifacts/joint_ranges/2026-08-13/j1_left_nominal_route_coverage_inset64.json`
  SHA256 `a1f15e57d5e335532e2f65f41768617f37ca9ffeffb220e972b8ef4b74042dc0`
- Approved candidate manifest:
  `config/bimanual_j1_operational_limits.approved.json`
  SHA256 `ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff`
- Firmware/host parity:
  `artifacts/joint_ranges/2026-08-13/j1_firmware_host_parity_plan_only.json`
  SHA256 `0605e75834dff9dfc003f215a88bc4287ddeeab1265ab95bc09fe2e04ef899b0`
- J1-L no-output firmware candidate: `0x00024100`,
  `build/stm32_g474_single_arm-j1l-shadow-release/stm32_g474_single_arm.hex`,
  SHA256 `5aa144ff5f2b0069f99d38f36638727457ef294e1df979732da53f1453765c02`

- J1-L hardware shadow no-output:
  `artifacts/protocol_v2/2026-08-13/j1l_arm_limits_shadow_run01.json`
  SHA256 `c9e9c2c302a1010d3b0a3f42fbfd92212b37059ad0212ef2c133db485c66a2e7`
- R4 restore bimanual soak 10/10:
  `artifacts/right_arm/2026-08-13/r4_after_j1l_soak_10.json`
  SHA256 `c92e8b8aadbc0575fcdeaf161eabd28fd6e72975cb3b5bcdb091293fa6e911db`
- URDF/MoveIt/Isaac candidate parity:
  `artifacts/joint_ranges/2026-08-13/j1_model_stack_parity_plan_only.json`
  SHA256 `3d6cb9199bdec43d1a96f74a447a6998c228b7c59954060d194d43ccbce1e683`
- MoveIt candidate (not loaded):
  `ros2_ws/src/so101_moveit_config/config/bimanual_j1_joint_limits.candidate.yaml`
  SHA256 `348558b07f2018c1154e2e0f7c985d6c8cf0140558aef269d2fcb12d1a99ce7e`
- Isaac import candidate URDF:
  `artifacts/bimanual/j1l_model/so101_dual_j1l_preview.urdf`
  SHA256 `ec8071817a9b827a7d22e76f609345aa375dccad0faf142ab7a51d30dce10f2c`

## 남은 blocker

1. J0-M physical outer envelope와 반복 편차는 아직 독립적으로 계측하지 않았다.
2. gripper raw↔jaw aperture/contact 의미 계약이 남아 있다.
3. physical q0 link angle, 두 base 상대 변환과 모델 parity가 남아 있다.
4. 오른팔 대표 경로와 J2 축별 bounded active 검증이 없다.

이 blocker가 해소되기 전에는 후보를 런타임 계층에 반영하거나 protocol-v2 실제
goal dispatch를 승인하지 않는다.
