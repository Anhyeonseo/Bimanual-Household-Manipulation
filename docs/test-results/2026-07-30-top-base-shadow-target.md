# Top 물체 base-frame shadow target 검증

## 판정

`top_board` 물체 좌표를 `left_base_link`로 변환하는 비명령 shadow 경로를
구현하고 실제 Top 카메라 입력으로 검증했다. 후보 좌표와 안전 gate 진단은
출력하지만 `motion_authorized=false`, `robot_target_available=false`를
강제하므로 MoveIt 목표나 로봇 명령으로 사용할 수 없다.

이 문서의 최초 dry-run은 당시 미검증 transform을 사용한 역사적 결과다.
후속 검토에서 `118.216 mm` 차이는 현재 작업대가 아니라, 폐기된 높이 있는
목재 체스보드 pose와 다른 세대 eye-to-hand 결과를 혼합한 비교에서 생긴
것으로 확인했다. 현재 Planar GridBoard 두 위치 검증으로 table–base
transform은 통과했지만 실제 Pick 이동 권한은 여전히 부여하지 않는다.

## 변환과 gate

`left_base_link ← top_board` 후보 변환:

```text
[ 0.9994338331,  0.0297555507, -0.0157041564,  0.3084597148]
[-0.0299200058,  0.9994987814, -0.0103430795, -0.2190669472]
[ 0.0153885211,  0.0108070920,  0.9998231845,  0.0265840073]
[ 0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000]
```

- board PnP RMS: `0.436663 px`
- 독립 eye-to-hand 검증 최대 오차: `1.099450 mm`, `0.520613 deg`
- 입력 gate: age `≤0.2 s`, confidence `≥0.70`, footprint 전체가 board 내부
- 보수적 작업공간: `x=0.20..0.46 m`, `y=-0.30..0.08 m`,
  `z=0.02..0.15 m`, radial `0.20..0.46 m`
- URDF 무작위 30,000 자세 FK의 최대 gripper 반경: `0.478281 m`

## 실제 카메라 dry-run

| 항목 | 결과 |
|---|---:|
| 출력 frame | `left_base_link` |
| 후보 위치 | `(0.396118, -0.125855, 0.040223) m` |
| 후보 yaw | `-0.021378 rad` |
| source age | `0.034568 s` |
| confidence | `0.718466` |
| board footprint | `true` |
| workspace / freshness | `true / true` |
| shadow pose available | `true` |
| transform validated | `false` (최초 dry-run 당시) |
| motion / robot target | `false / false` |
| status | `SHADOW_CANDIDATE_TRANSFORM_UNVALIDATED` |

실행 노드는 `/camera_manager`, `/top_object_pose`, `/top_shadow_target`였고,
MoveIt·single-arm bridge·Isaac Sim 로봇 제어 프로세스는 실행하지 않았다.
따라서 이 시험 중 실제 로봇 이동은 0회다.

## 소프트웨어 검증

- `so101_interfaces`, `so101_top_perception` 빌드 통과
- shadow core 단위시험 `14/14` 통과
- 관련 전체 회귀시험 `223/223` 통과(최초 dry-run 당시)
- flake8, pep257 및 `git diff --check` 통과

원시 실시간 결과는
`evidence/2026-07-30-top-shadow-dry-run.yaml`에 보존했다.

## 후속 판정

현재 사용하는 Planar GridBoard를 실제 작업대의 서로 떨어진 두 위치에서
재측정해 table–base transform을 검증했다. 최신 판정과 수치는
[현재 작업대–왼팔 base 등록 검증](2026-07-30-current-table-base-registration.md)
을 따른다. 다음 gate는 최신 변환의 실시간 shadow 재확인과 plan-only grasp
후보 검증이며, MoveIt 실행과 실제 Pick은 계속 금지한다.
