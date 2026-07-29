# Top eye-to-hand 독립 held-out 검증

## 판정

- 날짜: 2026-07-30
- 판정: **PASS — motion authorization은 다음 gate까지 false**
- training: 기존 고정 8자세
- 독립 validation: 새 2자세
- q0/model 재조정: 없음
- firmware/flash 변경: 없음

기존 `repeat_validation_01/02`는 wrist-flex q0 offset 선택 과정에서 확인했기
때문에 독립 검증으로 재사용하지 않았다. 새 `heldout_03/04`는 offset과
eye-to-hand transform을 고정한 후 처음 수집했다.

## 실기 접근과 캡처

각 validation은 상단 준비 자세에서 arm 5축을 같은 감소 방향으로 하강시켜
정착한 뒤 20프레임을 수집했다. 그리퍼는 유지했다.

- `heldout_03`
  - 상단 준비: 첫 terminal 판정은 final error `59 raw` soft-abort,
    safety latch 없음. 정착 후 board 4 marker와 `123.6 px` 경계 여유 확인
  - 하강 정착: Action `SUCCEEDED`
  - 캡처: 20/20, joint span `0.000000 rad`
- `heldout_04`
  - 상단 준비: 첫 terminal 판정은 final error `114 raw` soft-abort,
    safety latch 없음. 정착 후 board 4 marker와 `107.2 px` 경계 여유 확인
  - 하강 정착: Action `SUCCEEDED`
  - 캡처: 20/20, joint span `0.000000 rad`

두 soft-abort 뒤 추가 명령 없이 읽은 최종 feedback은 목표 근처에서
안정됐으며 bridge Action server와 `allow_motion=true`는 유지됐다.

## 독립 오차

| capture | translation residual | rotation residual | PnP RMS | border min |
|---|---:|---:|---:|---:|
| `top_e2h_independent_heldout_03` | `1.099450 mm` | `0.418634 deg` | `1.068618 px` | `153.707 px` |
| `top_e2h_independent_heldout_04` | `0.644049 mm` | `0.520613 deg` | `0.741702 px` | `94.990 px` |

집계:

- validation translation RMS/max: `0.900997 / 1.099450 mm`
- validation rotation RMS/max: `0.472383 / 0.520613 deg`
- failure reasons: none

## 산출물과 안전 상태

- session:
  `evidence/2026-07-30-top-eye-to-hand-independent-validation.yaml`
- candidate:
  `evidence/2026-07-30-top-eye-to-hand-candidate.yaml`
- candidate SHA-256:
  `b0e251a159f86a76915b1ae437e37a325ba42e9161da186d9173e36df549ea37`
- status: `EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED`
- `motion_authorized: false`
- `robot_target_available: false`

Eye-to-hand transform의 독립 gate는 통과했다. 다음 단계는 이 transform을
고정한 채 작업대의 계측된 물체 위치와 perception 출력의 `x/y/yaw`를
비교하는 것이다. 그 검증 전에는 candidate를 motion authorization으로
사용하지 않는다.
