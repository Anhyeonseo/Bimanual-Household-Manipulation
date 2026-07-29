# 현재 작업대–왼팔 base 등록 검증

## 판정

현재 사용하는 대형 Planar GridBoard(`DICT_4X4_50`, ID `10..29`)를 실제
작업대의 서로 떨어진 두 위치에 배치해 `left_base_link`와 작업대 평면의
관계를 다시 검증했다. 두 배치의 PnP, 평면 법선, 높이 및 기존 20 mm 검증지
랜드마크 재투영이 모두 합격 기준 안에 들어와 현재 table–base 등록을
`transform_validated=true`로 판정한다.

이 결과는 좌표계만 검증한다. `motion_authorized=false`와
`robot_target_available=false`는 계속 유지하며, 다음 단계는 실제 이동이
없는 plan-only grasp 후보 검증이다.

## 현재 좌표 계약

- 기준 frame: `left_base_link`
- `top_board` 원점: `(0.340, -0.280, -0.005) m`
- `top_board +X/+Y`: 각각 `left_base_link +X/+Y`와 평행
- 보정 영역: `0.180 × 0.280 m`
- 실제 작업대 높이: `left_base_link z=-0.005 m`
- 보수적 shadow workspace:
  `x=0.34..0.46 m`, `y=-0.28..0.00 m`, `z=0.00..0.15 m`,
  radial `0.34..0.47 m`

## 두 위치 독립 검증

| 지표 | 결과 | 합격 기준 |
|---|---:|---:|
| GridBoard 위치 간 거리 | `160.528 mm` | `≥100 mm` |
| PnP RMS 최대 | `0.650 px` | `≤1.5 px` |
| 평면 법선 차이 | `0.847 deg` | `≤2.0 deg` |
| 평균 높이 차이 | `1.550 mm` | `≤4.0 mm` |
| 개별 배치 최대 높이 오차 | `6.089 mm` | `≤8.0 mm` |
| GridBoard corner metric 최대 오차 | `4.504 mm` | 기록 |

추가로 기존 실제 물체 검증 때 저장한 20 mm 격자의 물리 랜드마크 픽셀을
현재 보정으로 다시 투영했다.

| 지표 | 결과 | 합격 기준 |
|---|---:|---:|
| 위치 RMS / 최대 | `7.116 / 8.880 mm` | 최대 `≤10 mm` |
| yaw RMS / 최대 | `1.440 / 1.946 deg` | 최대 `≤5 deg` |

## 폐기한 혼합 기준

이전에 보고된 `118.216 mm` 차이는 현재 작업대 기준의 실패가 아니었다.
서로 다른 세대의 자료인 다음 두 항목을 잘못 결합해 비교한 결과다.

- 이전에 사용하던 높이가 있는 목재 체스보드 pose
- 이후 세대의 eye-to-hand 보정 결과

해당 체스보드는 현재 캘리브레이션 대상이 아니며 현재 3D 변환 계산에는
사용하지 않았다. 원본은 감사 추적을 위해
`evidence/2026-07-30-top-base-obsolete-raised-board.yaml`
에 보존했다.

## 증거

- 검증 요약:
  `evidence/2026-07-30-top-base-table-validation.yaml`
- 1차 배치:
  `evidence/2026-07-30-top-base-table-capture-01.png`
- 2차 배치:
  `evidence/2026-07-30-top-base-table-capture-02.png`
- 보정 도구: `tools/calibrate_top_base_table.py`
- 자동 시험: `tests/test_top_base_table_calibration.py`
- 전체 회귀시험: `233/233` 통과

## 다음 gate

GridBoard를 제거한 실제 검은 펜 입력으로 최신 transform의 실시간 shadow를
재확인했다. 펜 전체는 작은 보정 사각형을 벗어나지만 영상에는 완전히 보였고,
중심점 `(0.371814, -0.129674, 0.006300) m`은 보정영역 안이었다.
후속 hardware-limit 감사에서 기존 workspace가 승인 관절범위로 계산되지
않았음이 확인되어 `inside_workspace=true` 판정은 폐기했다. 정정된 결과는
`SHADOW_OUTSIDE_WORKSPACE`, `motion_authorized=false`,
`robot_target_available=false`다.

다음 gate는 shoulder/elbow torque-disabled 물리 범위 재검증이다. 상세
수치는 [단계 7 카메라–하드웨어 도달영역 감사](2026-07-30-stage7-reachability-audit.md)
를 따른다. 실제 로봇 이동 권한은 별도 승인 전까지 계속 차단한다.
