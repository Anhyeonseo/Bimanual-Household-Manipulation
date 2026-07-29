# 단계 7 카메라–하드웨어 도달영역 감사

## 판정

Top 카메라와 `left_base_link` 좌표 변환은 유효하지만, 현재 승인된
shoulder/elbow 관절범위로는 카메라에 보이는 검은 펜의 저높이 pre-grasp와
grasp pose에 도달할 수 없다. 따라서 단계 7 실제 Pick은 차단하고
shoulder/elbow의 물리 안전범위를 torque-disabled 상태에서 재측정한다.

로봇 명령과 실제 이동은 수행하지 않았다.

## 현재 펜과 plan-only 결과

- 물체 중심: `(0.371814, -0.129674, 0.006300) m`
- pre-grasp TCP: 물체 중심보다 `100 mm` 위
- grasp TCP: 물체 중심보다 `25 mm` 위
- MoveIt mock `/plan_kinematic_path`: 두 목표 모두 해 없음
- Execute API 사용: `false`

현재 승인 관절범위의 전역 최적화 최소 오차:

| 목표 | 최소 TCP 거리 오차 |
|---|---:|
| pre-grasp | `83.945 mm` |
| grasp | `114.357 mm` |

## 원인 분리

현재 MoveIt 제한에서 TCP 최대 `x`는 `0.332350 m`다. 반면 현재 펜 중심은
`x=0.371814 m`이다. 낮은 grasp와 pre-grasp 높이를 모두 만족하는 보수적
공통 XY 영역은 대략 `x=0.15..0.23 m`, `y=-0.13..-0.02 m`지만, 대표점
`(0.20, -0.065) m`은 Top 영상에서 `(219.8, 575.5) px`로 투영되어
`640×480` 영상 아래쪽 바깥이다.

따라서 현재 승인 범위 기준으로는 카메라 가시영역과 저높이 Pick 도달영역이
겹치지 않는다.

## 전체 URDF 기구학 비교

URDF 전체 기구학 한계로 같은 목표를 최적화하면 두 목표 모두 위치 오차
`0.0 mm`로 도달한다.

| 관절 | 현재 최대 | pre-grasp 해 | grasp 해 |
|---|---:|---:|---:|
| base | `0.862 rad` | `0.386 rad` | `0.391 rad` |
| shoulder | `1.055 rad` | `1.862 rad` | `2.257 rad` |
| elbow | `0.560 rad` | `1.020 rad` | `1.359 rad` |
| wrist flex | `1.310 rad` | `0.844 rad` | `0.571 rad` |

카메라 위치나 table–base 보정이 아니라 shoulder/elbow의 현재 승인범위가
직접적인 차단 원인이다. 다만 이 수치만으로 firmware 한계를 즉시 넓혀서는
안 된다. 기구학적으로 가능하다는 뜻일 뿐, 실물 케이블·자체충돌·기계
스토퍼·중력 하중 안전을 보장하지 않는다.

## 다음 gate

1. `12 V OFF`, 팔 지지 상태에서 물리 간섭을 확인한다.
2. torque-disabled raw observer만 실행한다.
3. shoulder/elbow를 작은 증분으로 수동 이동해 필요한 pose와 raw 범위를
   기록한다.
4. 실물 안전 여유를 적용한 뒤 host/firmware/URDF/MoveIt 범위를 함께
   변경한다.
5. READ_ONLY와 mock plan-only를 다시 통과한 후에만 제한된 실제 이동을
   별도 승인한다.

## 증거

- `evidence/2026-07-30-stage7-reachability-blocked.yaml`
- 초기 orientation 포함 실패 결과는 position-only 재시험으로 대체되어 제거
- `evidence/2026-07-30-stage7-plan-only-limited-fail.json`
- `tools/ros_moveit_plan_grasp.py`
- 전체 회귀시험 `236/236` 통과
