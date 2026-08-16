# F8.9 resident와 양팔 펜 전달 수락 결과

- 날짜: 2026-08-16
- firmware: `0x00024809`, protocol 2, 12 joints
- 결론: **PASS — 좌우 Top-camera Pick/Place와 연속 left→right 전달 완료**

## 변경 이유

오른팔 그리퍼가 펜을 정상 파지했을 때 측정된 접촉 잔차
`97,784 µrad`가 기존 공통 tracking 한계 `90,000 µrad`를 넘어
전체 torque-off를 유발했다. F8.9는 팔 관절 안전 한계를 유지하면서
그리퍼 2축에만 별도 접촉 범위를 적용한다.

| 항목 | 한계 |
|---|---:|
| arm route tracking | 90,000 µrad |
| arm terminal settle | 46,020 µrad |
| gripper route/terminal | 150,000 µrad |
| firmware gripper hard cap | 160,000 µrad |

DMA, dispatch, heartbeat, operational-limit, unwrap 및 arm tracking fault의
즉시 coordinated stop 정책은 그대로다. 자동 재시도는 없다.

## 배포 산출물

| 산출물 | SHA-256 |
|---|---|
| ELF | `5875a02c17055dc37fb968b148cfbb206ff7ae7c06b2ba74573e871a0383dd79` |
| HEX | `a916a5ade13200df3572717f1c0a86c207cb5b6e91344fd9b78d276c60a619b0` |

로컬 경로:
`artifacts/firmware/2026-08-16/f89_gripper_contact/`.

## Resident gate

| gate | 결과 | artifact SHA-256 |
|---|---|---|
| motion-disabled adapter | `RESIDENT_BIMANUAL_ADAPTER_NO_MOTION_PASS` | `248ee592fa6dd9f68134574afd4a21ff5679bf939cffa01c7e8d7cd652c687d8` |
| current-pose hold twice | `RESIDENT_BIMANUAL_CURRENT_POSE_HOLD_TWICE_PASS` | `860f626d2e8a6e5ec5a5bcc5f3a38952ce67ef751b482950168dc6ff562a5f41` |

## Top-camera 보정과 연속 전달

계획 스키마 12는 카메라 화면축 보정을 homography의 로컬 Jacobian으로
workcell XY에 변환하고 plan SHA에 고정한다.

- 왼팔: 화면 오른쪽 `13.72 mm`
- 오른팔: 화면 왼쪽 `29.47 mm`
- 시작점: 매 실행 직전 fresh measured 12축 anchor
- 실패 정책: 다음 stage 전송 없음, 자동 재시도 없음
- terminal: 양팔 q0, resident READY, torque HOLD

실기 session:
`artifacts/top_pick_place/2026-08-16/pen_interarm_continuous_session03/`.

| stage | 결과 | artifact SHA-256 |
|---|---|---|
| left plan/validate/execute | PASS/HOLD, 최대 arm error 28.176 mrad | `d2d4069a11a8933c12880bf5ceb256c859bc90bafe9058e19510045d7bee6861` |
| fresh right plan/validate/execute | PASS/HOLD, 최대 arm error 13.806 mrad | `ffcf190f37402b8e96090063f1760237797f03868e33147ffbd2252fb49784d4` |
| 전체 journal | `LEFT_RIGHT_PEN_TRANSFER_ONCE_PASS` | `408c21d6e7211834351123c5058cf7a8be50b8d20d064ec3f861230099198fbc` |

## 다음 범위

다음 PR은 캔 한 개의 수직 접근 파지만 다룬다. Top OBB의 무방향 장축
yaw에 대해 그리퍼 닫힘 축을 90도로 맞추고, 현재 wrist 자세에서
`(-90°, +90°]` 안의 가장 가까운 동치 분기를 명시적으로 고정한다.
캔→쓰레기통 및 양팔 handover는 이 gate가 통과한 뒤 별도 PR로 진행한다.
