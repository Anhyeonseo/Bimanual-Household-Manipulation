# Wrist flex q0 외부 계측 보정

## 범위와 안전 상태

- 범위: URDF/model q0 및 연동 테스트·문서
- 실물 로봇 이동: 없음
- firmware 변경·STM32 flash: 없음
- raw 2048 및 bridge calibration 변경: 없음
- motion authorization: `false`

## 근거

`session04_directional/session_r1.yaml`의 8개 training pose와 2개 held-out
validation pose를 사용한 read-only sensitivity 분석에서 wrist-flex offset
`-0.129124366605 rad` (`-7.398281239 deg`)가 식별되었다.

보정 전 기준:

- training translation RMS/max: `3.200638 / 6.228009 mm`
- training rotation RMS/max: `1.001883 / 1.3717 deg`
- validation translation max: `6.080678 mm`

보정 URDF 재계산 후:

- training translation RMS/max: `2.140809 / 3.004875 mm`
- training rotation RMS/max: `0.686173 / 0.955771 deg`
- validation translation max: `3.848063 mm`
- validation rotation max: `1.041201 deg`

## 적용값

Wrist flex joint axis는 `0 0 -1`이므로 effective q offset
`-0.129124366605 rad`를 적용하려면 origin yaw를 같은 크기만큼 증가시킨다.

| 항목 | 보정 전 | 보정 후 |
|---|---:|---:|
| upstream Home | `-57.5 deg` | `-64.898281239 deg` |
| origin yaw | `-0.567235680103261 rad` | `-0.438111313497813 rad` |
| lower limit | `-0.654495680103261 rad` | `-0.525371313497813 rad` |
| upper limit | `2.661624319896740 rad` | `2.790748686502188 rad` |

Limit도 origin과 함께 `+0.129124366605448 rad` 평행 이동하여 upstream
물리 가동 범위를 보존한다.

## 실제 재계산 산출물

- candidate: `evidence/2026-07-30-wrist-flex-q0-refined.yaml`
- rejected/incomplete 세션 정리: [보정 세션 정리 기록](2026-07-30-top-eye-to-hand-session-cleanup.md)
- SHA-256: `1e67c0f404b36e0af6c693eec39e815b3b2a306d90b080cc52ba06eed312c332`
- status: `EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED`
- `motion_authorized: false`
- `robot_target_available: false`
- acceptance failure reasons: none

## Isaac Sim 6.0.1 USD 동기화

전체 URDF 재임포트로 custom wrist camera geometry, overhead workcell과
OmniGraph를 덮어쓰지 않도록 q0를 소유하는 authored layer만 수정했다.

- `payloads/base.usda`: `wrist_link` 기본 quaternion
- `payloads/Physics/physics.usda`: `wrist_flex` localRot0 및 degree limits
- drive/state target: 0 유지
- raw 2048, bridge mapping/offset: 변경 없음
- Isaac Sim 6.0.1 `Sdf` layer 및 composed stage parse: PASS
- URDF↔Isaac q0 TCP 위치 오차: `3.462553857083e-08 m`
- URDF↔Isaac q0 TCP 회전 오차: `2.149076029935e-07 rad`
- 전체 회귀: `214 passed`
- ROS 패키지 빌드: `so101_description`, `so101_moveit_config`, `so101_isaac_bridge` PASS

Layer SHA-256:

- `base.usda`: `13f4fda9e82cede74d6dc7c20bdf95d3081d9c6de88948b6611a4b0335aa28af`
- `Physics/physics.usda`: `7094d078645a32bae93914f4edfc1a53b35672cdfa44ed667c238940c6a477cd`
- root `so101_new_calib.usda`: `4d69817df7c1d0bd214b4a3928cd9867bafb0647d18b92977b2f7e576e1b5e49`

URDF origin과 두 USD quaternion의 회전행렬, joint position anchor, axis-sign
변환을 적용한 limits를 자동 비교하는 parity 회귀 테스트를 추가했다.

## 남은 gate

코드 회귀 시험, 보정 URDF 기반 solver 재계산, Isaac USD parity와 새 독립
held-out 2자세 실기 검증은 통과했다. 독립 결과는
`2026-07-30-top-eye-to-hand-independent-validation.md`에 기록한다.
작업대 물체의 실제 `x/y/yaw` 대조 전까지 실제 기하 기반 motion은 승인하지
않는다.
