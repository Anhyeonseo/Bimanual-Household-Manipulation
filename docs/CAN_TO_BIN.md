# 캔 → 수거함 파이프라인

## 현재 구현 범위

현재 코드는 **왼팔 캔 pick-only**까지 준비돼 있다. 캔을 집은 뒤 수거함으로
옮기고 놓는 단계는 아직 없다. 실행 후보를 실제보다 크게 설명하지 않는다.

관련 파일:

- `config/can_pick_contract.candidate.json`
- `tools/lib/can_pick_application.py`
- `tools/lib/grasp_yaw_kinematics.py`
- `tools/run/plan_can_pick_left_once.py`
- `tools/run/run_can_pick_left_once.py`
- `tools/setup/can_perception/commission_can_jaw_gap_map_once.py`

## 파지 계약

1. 상단 카메라에서 정확히 한 개의 완전 가시 캔을 잠근다.
2. 작업대 보정 영역과 왼팔 도달 영역을 각각 검사한다.
3. 캔 장축에 수직인 finger yaw를 계산한다.
4. wrist-roll 한계 안의 해를 열거하고 현재 자세에서 가장 가까운 분기를 고른다.
5. 선택한 roll을 고정한 채 나머지 4축과 TCP 위치를 함께 푼다.
6. MoveIt으로 각 구간의 관절 한계와 충돌을 검사한다.
7. jaw 실측값, plan SHA, calibration SHA가 모두 유효할 때만 validate-only를
   통과시킨다.

## 승격 순서

```text
camera calibration PASS
→ jaw mapping 측정
→ pick plan-only
→ validate-only (motion_commands=0)
→ open-at-grasp-height supervised check
→ supervised pick once
→ 반복 pick 검증
→ 수거함 geometry/collision 추가
→ supervised place once
→ 배치 후 재인식
```

실제 동작은 각 단계의 artifact가 승인되기 전 다음 단계로 넘어가지 않는다.

## 수거함 단계에서 새로 필요한 것

- 수거함 개구부 위치·크기·높이와 collision geometry
- 선택 팔에서 수거함까지의 전 구간 도달성
- 캔을 든 상태의 collision envelope
- release command와 release 확인 기준
- 배치 후 상단 카메라 재인식 규칙
- 실패 시 캔을 떨어뜨리지 않는 hold/retreat 정책
