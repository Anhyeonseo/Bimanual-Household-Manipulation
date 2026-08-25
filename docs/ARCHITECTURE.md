# 시스템 구조

## 데이터 흐름

```text
Top + wrist cameras
  → manipulation_camera_manager
  → towel segmentation / height / boundary observations
  → deformable-state estimator
  → learned action proposer + deterministic baseline
  → towel task state machine and bounded recovery gate
  → bimanual grasp/fold planner + MoveIt collision checks
  → towel-task executor
  → bimanual_stream_adapter
  → protocol v2
  → STM32 12-axis resident executor
  → measured feedback + visual result verification
```

실제 task의 최소 실행 단위는 연속 vision servo가 아니라 아래의 닫힌
관측-동작 주기다.

```text
OBSERVE_CLEAR (양팔 퇴피, Top 중심 관측)
  → PLAN_AND_VALIDATE
  → APPROACH_AND_GRASP_VERIFY (손목 관측 + 접촉 feedback)
  → EXECUTE_BOUNDED_PRIMITIVE
  → LAY_DOWN_OR_SAFE_HOLD
  → RETREAT_AND_SETTLE
  → REOBSERVE_CLEAR
```

Top 영상에서 팔이나 gripper에 가려진 영역은 추정으로 채우지 않고 `unknown`으로
남긴다. 잡고 있는 모서리는 visual corner가 아니라 TCP에 붙어 있다는 조건부
`held_corner_constraint`로 표현하며 slip이 의심되면 즉시 무효화한다.

## 책임 경계

| 계층 | 책임 | 금지 |
|---|---|---|
| Camera manager | 상단·손목 프레임, timestamp, phase별 rate | task 판단, motion 명령 |
| Towel perception | mask, 높이, 경계, 모서리, grasp 후보, 신뢰도 | robot command 생성 |
| State estimator | 구김·부분 펼침·평탄·접힘 상태와 관측 이력 | stale 관측 승인 |
| Learning proposer | 펼치기·복구 primitive와 bounded grasp/placement 후보, confidence·abstain | 관절/토크 명령, planner·gate 우회 |
| Task manager | heuristic/learned 후보 선택, 시도 횟수, 실패·복구 전이 | serial 직접 접근 |
| Planner/MoveIt | 양팔 IK, fold arc, 장력 proxy, joint/collision 검사 | 안전 gate 우회 |
| Towel executor | SHA 고정 plan, 단계 동기화, terminal 검증 | 무한 자동 재시도 |
| Resident adapter | 12축 owner/epoch, finite stream, feedback | 복수 serial owner |
| STM32 | 동기 출력, tracking, heartbeat, stop/latch | 수건 상태 판단 |

## 현재 연결된 motion-free 경로

```text
reviewed pixel annotation
  → homography 기반 metric observation
  → fail-closed state estimate
  → bounded task decision / offline replay
  → two orthogonal FoldSpec
  → synchronized geometric semicircle
  → fake reachability candidate selection
  → JSON artifact (motion_authorized=false, motion_commands=0)
```

이 경로는 executor, ROS action client, serial과 motor API를 생성하지 않는다.
기하 arc와 fake reachability 결과는 설계·회귀용이며 IK, self/world collision,
장력 또는 실제 도달성을 증명하지 않는다.

## 변형체 상태 모델

```text
CRUMPLED
  → PARTIALLY_OPEN
  → TWO_CORNERS_VISIBLE
  → FOUR_CORNERS_VISIBLE
  → FLAT_BUT_ROTATED
  → ALIGNED
  → FOLD_1_COMPLETE
  → FOLD_2_COMPLETE
```

모든 전이는 새로운 관측으로 확인한다. 요구 신뢰도나 기하 조건을 만족하지
못하면 `FAILED` 또는 승인된 복구 전이로 이동하며, 이전 pose를 그대로 사용해
다음 실제 동작을 실행하지 않는다.

## 조작 primitive 경계

- `grasp_exposed_corner`
- `grasp_two_corners`
- `lift_and_observe`
- `tension_spread`
- `controlled_shake`
- `drag_corner`
- `lay_flat`
- `align_square`
- `fold_edge_pair`
- `release_and_smooth`

각 primitive는 사전 조건, 최대 시간·거리·속도·시도 횟수, terminal measured
feedback, 사후 visual condition을 별도 계약으로 가진다.

## 학습 경계

임의 구김은 규칙만으로 열거하기 어려우므로 `COARSE_UNFOLD`와 recovery의
action selection은 학습 대상이다. 반면 동기 양팔 trajectory, collision 검사,
contact 제한과 stop은 deterministic 계층에 남긴다.

```text
versioned observation
  → heuristic 또는 learned proposer
  → primitive + bounded parameters + confidence/abstain
  → task/recovery budget gate
  → MoveIt + contact/safety validation
  → finite resident execution
  → measured/visual outcome → episode dataset
```

학습기는 Isaac Lab randomized rollout, 실제 self-supervised episode 또는 둘을
함께 사용할 수 있다. 어느 방법이든 dataset split, environment/checkpoint SHA와
baseline 비교 없이 승격하지 않는다. end-to-end image-to-joint 정책은 이 책임
경계를 우회하므로 현재 구조에서 허용하지 않는다.

## 안전 불변식

1. 부팅과 재연결만으로 모터가 움직이지 않는다.
2. 상위 앱은 serial을 직접 열지 않는다.
3. 한 팔 동작도 반대 팔 hold를 포함한 12축 command다.
4. calibration, observation, plan 또는 contract SHA가 stale이면 거부한다.
5. 양팔 TCP 간격과 속도 차이는 수건 장력 제한 안에 있어야 한다.
6. terminal measured feedback와 새 visual observation 전에는 성공이 아니다.
7. fault 뒤 session이나 실패한 plan을 재사용하지 않는다.
8. 복구는 계약에 기록된 횟수만 허용하고 한도를 넘으면 안전 정지한다.
9. 작업대 homography는 작업대 평면상의 점에만 사용하고 들린 수건에 적용하지
   않는다.
10. 네 모서리·평탄도·fold 결과의 승인 관측은 양팔이 지정된 clear pose에 있고
    수건이 settle된 뒤에만 생성한다.

## 유지하는 공통 기반

펌웨어, protocol, 양팔 URDF/MoveIt, resident adapter, 카메라 manager와 기존
보정 도구는 수건 시스템의 기반으로 유지한다. 일부 디렉터리와 package의
`single_arm` 이름은 STM32CubeIDE와 ROS 배포 호환성을 위한 legacy 식별자다.
