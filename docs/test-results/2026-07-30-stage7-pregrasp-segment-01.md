# 단계 7 분할 pregrasp 1번 구간 최초 실기

## 범위와 사전 gate

전체 Pick을 실행하지 않고 현재 실물 자세에서 pregrasp까지의 관절 경로만
최대 `0.30 rad` 간격으로 분할했다. 실제 명령 전 다음 gate를 통과했다.

- localhost Domain 93 mock MoveIt: q0 기준 6구간, 6/6 plan-only PASS
- STM32 READ_ONLY 시작 자세:
  `(0.056757, 0.277651, 0.358952, 0.250039, -0.092039) rad`
- 실제 시작 자세 기준: 5구간, 5/5 plan-only PASS
- 구간별 최대 변화량: `0.299863 rad`
- plan-only 파일 SHA-256:
  `a1c7e5ab39fafa7b85344d6ac25722dbcfac0e0a3b4f443ea4b09be1a04d4737`
- 실행 도구는 SHA 일치, 전체 구간 성공, calibration 범위, 최신
  `/joint_states` 시작 오차 `0.05 rad` 이내를 요구하며 재시도하지 않음

실행 직전 실제 시작점의 최대 오차는 `0.001534 rad`였다. 사용자가
`pregrasp 1/5, 2초 단 1회`를 승인한 뒤에만 첫 목표를 전송했다.

## 최초 실기 결과

첫 목표는 Action에서 수락됐고 2초 동안 이동했지만 최종 판정은 실패했다.

```text
PREGRASP_SEGMENT_EXECUTE_GOAL_ACCEPTED
ARM_EXECUTION_TERMINAL state=aborted status=6 detail=26
reason=final error 26 exceeds 20 raw; soft abort without safety latch
MoveIt error_code=-4
```

- 자동 재시도: 0회
- SAFE_STOP 요청: 0회
- STM32 stop latch: 걸리지 않음
- 워크스테이션 MoveIt, Pi bridge와 12 V 전원: 순서대로 종료

실패 직후 fresh feedback과 목표의 차이는 다음과 같다.

| 관절 | 목표 rad | feedback rad | 절대 잔차 rad |
|---|---:|---:|---:|
| Base | 0.114249 | 0.107379 | 0.006870 |
| Shoulder | 0.577513 | 0.570641 | 0.006872 |
| Elbow | 0.422404 | 0.395767 | 0.026637 |
| Wrist Flex | 0.457376 | 0.443320 | 0.014056 |
| Wrist Roll | -0.044630 | -0.053689 | 0.009059 |

물리적으로는 첫 목표 근처까지 도달했지만 Action 계약상 실패이므로 PASS로
소급하지 않는다. terminal 순간 최대 `26 raw`와 이후 fresh feedback의 더
작은 잔차 차이는 중력·서보 정착 과정의 transient로 판단한다.

## 로컬 host 보강

사용자 승인 아래 firmware와 calibration은 바꾸지 않고 host 판정만
분리했다.

- motion completion 허용치: `20 → 30 raw`
- out-of-range feedback recovery trigger: `20 raw` 유지
- recovery 목표의 strict-range 안쪽 margin: `20 raw` 유지
- Python 회귀: `259/259` PASS
- ROS 2: 8 packages build, 21 tests PASS
- 최초 로컬 검증 시점의 Pi 전송·배포: 미실행
- 펌웨어 변경·플래시: 없음

완료 허용치 변경이 feedback recovery 범위를 함께 넓히지 않도록 서로 다른
상수와 테스트로 고정했다.

## Pi 배포와 분할 pregrasp 재실기

수정된 host 두 파일만 Pi에 전송하고 원본을 다음 위치에 백업했다.

```text
~/Manipulation/ros2_ws/.pre-stage7-host30-backup-ysTMqr
```

- `action_execution.py` SHA:
  `e944576a52144b4f48fbd9d781c2161e61a2f3ee36ab9698dd9bc1c09755f5d0`
- `follow_joint_trajectory_server.py` SHA:
  `186ba0daa29db68c06f9d3410ccb9e1dc9fef890a7fb51a57fbff2b8180628a4`
- Pi `single_arm_bridge` rebuild: PASS
- READ_ONLY identity/feedback: PASS
- MOTION_ENABLED 무동작 연결: PASS
- 펌웨어 변경·플래시: 없음

전원 주기 뒤 자세가 달라졌으므로 최초 plan SHA를 재사용하지 않았다. fresh
READ_ONLY 시작값은 다음과 같았다.

```text
(0.108913, 0.444854, 0.556835, 0.454058, -0.053689) rad
```

이 시작점에서 pregrasp까지 다시 5구간으로 나눴으며 모든 구간의 plan-only가
통과했다.

- 새 plan SHA:
  `664a3a0456facb73f7fafc5e2fa32efd9c0608d4db4321e4d880bb69c10c985e`
- 구간별 계획 최대 변화: `0.266422 rad`
- 실행 직전 실제 current-to-target 변화도 `0.30 rad` 이하로 별도 검증
- 새 실행 gate 보강 후 Python 회귀: `260/260` PASS

사용자는 각 구간을 별도로 승인했다. 5개 구간은 모두 다음 결과로 끝났다.

```text
MoveIt status=4
MoveIt error_code=1
retries=0
```

최종 feedback은 다음과 같다.

```text
(0.337476, 1.782486, 0.639670, 1.273204, 0.134990) rad
```

pregrasp 목표와의 최대 절대 잔차는 Elbow `0.036544 rad`로 후속 시작 gate
`0.05 rad` 안이다. 따라서 **분할 pregrasp 접근은 PASS**다.

## 다음 gate

1. 현재 pregrasp에서 grasp까지의 경로를 새로 분할하고 plan-only 검증
2. 실제 current-to-target `0.30 rad`, fresh 시작 오차 `0.05 rad` 유지
3. 각 grasp 구간을 별도 승인 아래 단 1회 실행
4. grasp 도달 뒤에만 gripper close와 lift를 각각 별도 검증

전체 Pick 자동화, lift, place와 자동 재시도는 계속 금지한다.
