# 책상 정리 검증 매트릭스

| Gate | 확인 대상 | 자동/실기 | 현재 |
|---|---|---|---|
| V0 | protocol manifest, firmware/host parity | 자동 | 유지 |
| V1 | 양팔 URDF, operational limits, FK | 자동 | 유지 |
| V2 | Top intrinsic 독립 검증 | 실기+오프라인 | PASS 후보 |
| V3 | eye-to-hand 독립 검증 | 실기+오프라인 | REJECTED |
| V4 | 작업대 metric/homography | 실기+오프라인 | 재검증 필요 |
| V5 | 캔 OBB precision/recall/pose error | 오프라인 | 후보 있음 |
| V6 | jaw gap/contact/tilt 계약 | 실기 | 미완료 |
| V7 | 캔 plan-only collision/limit/SHA | 자동+MoveIt | 재생성 필요 |
| V8 | validate-only, motion command 0 | 자동 | 코드 있음 |
| V9 | supervised 캔 pick | 실기 | 미승인 |
| V10 | 수거함 transit/release | 실기 | 미구현 |
| V11 | 배치 후 scene verification | 실기 | 미구현 |
| V12 | 다물체/양팔 반복성과 soak | 실기 | 미구현 |

## 실행 원칙

- 선행 gate가 실패하면 이후 gate를 실행하지 않는다.
- 실제 모터를 쓰는 gate는 명시적 confirmation과 물리 전원 차단 수단이 필요하다.
- 같은 실제 동작을 자동 재시도하지 않는다.
- 모든 plan은 robot description, operational limits, calibration, perception
  bundle의 SHA를 기록한다.
- 성공은 measured terminal feedback과 배치 후 scene 검증이 모두 통과해야 한다.
