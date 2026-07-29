# 단계 5 — 왼팔 실제 hardware trajectory backend

## 현재 판정

- 상태: **COMPLETE**
- 단계 5 진행률: **100% — 초기 B안 single-point hardware milestone PASS**
- 대상: 정상인 SO-ARM101 왼팔
- 추가 servo motion: **각 시험별 사용자 승인 전 금지**
- 검증된 실제 범위: arm/gripper single-point, cancel, SAFE_STOP, recovery,
  reconnect와 MoveIt end-to-end

단계 4에서 검증한 MoveIt controller contract를 실제 STM32 backend에
연결한다. 단계 5의 완료는 기존 custom `/joint_command` 시험이 아니라
MoveIt의 standard action에서 STM32까지 이어지는 end-to-end 검증을
의미한다.

## Source of truth

충돌은 다음 순서로 판정한다.

1. 실제 측정 결과
2. 현재 repository code
3. 최신 `docs/test-results`
4. 최신 ADR
5. `docs/ROADMAP.md`
6. 과거 대화와 인계 문서

calibration, firmware safe limit, torque limit은 사용자 승인 없이 변경하지
않는다.

## 이미 확인한 기반

| 항목 | 상태 | 근거 |
|---|---|---|
| STM32 firmware `0x00020700` | PASS | binary smoke |
| protocol v1, COBS, CRC-32C | PASS | firmware와 host test |
| calibration hash `0x3DB42B48` | PASS | code와 실제 smoke |
| physical folded home raw 2048 | PASS | 실제 calibration |
| actual position feedback | PASS | Pi `/joint_states` 약 5 Hz |
| custom single-point `/joint_command` | PASS | 실제 소각도·home 시험 |
| heartbeat, SAFE_STOP, fault latch | PASS | 실제 시험 |
| MoveIt mock arm/gripper action | PASS | 단계 4 |
| MoveIt → Isaac arm/gripper action | PASS | 단계 4 |
| actual hardware `FollowJointTrajectory` | PASS | q0, BASE 왕복, 팔 5축 소각도 |
| actual hardware gripper action | PASS | 0.08 rad, 0.13 rad safe subrange |
| 실제 controller cancel | PASS | 300 ms cancel, SAFE_STOP latch, recovery |
| reconnect stale goal 방지 | PASS | 재연결 5초 무명령·무동작·오류 0회 |
| MoveIt → STM32 실제 제어 | PASS | home, arm 0.05/0.10 rad, gripper 0.08 rad |
| preempt | 의도적으로 미지원 | 동시 goal은 acceptance 전 reject |

관련 source가 변경되지 않는 한 위 PASS 시험을 이유 없이 반복하지 않는다.

## 고정할 상위 ROS contract

### Joint state

- topic: `/joint_states`
- type: `sensor_msgs/msg/JointState`
- 순서:
  1. `left_base_joint`
  2. `left_shoulder_joint`
  3. `left_elbow_joint`
  4. `left_wrist_flex_joint`
  5. `left_wrist_roll_joint`
  6. `left_gripper_joint`
- unit: radian
- q=0: physical folded home

### Arm action

- action:
  `/left_arm_controller/follow_joint_trajectory`
- type:
  `control_msgs/action/FollowJointTrajectory`
- joints: arm 5개

### Gripper action

- action:
  `/left_gripper_controller/gripper_cmd`
- type:
  `control_msgs/action/ParallelGripperCommand`
- command joint:
  `left_gripper_joint`

MoveIt, task node와 향후 policy는 servo ID, raw position, serial packet 또는
Isaac joint 이름을 알면 안 된다.

## 현재 custom bridge contract

`single_arm_bridge`는 다음 interface를 제공한다.

- `/joint_states`
- `/left_arm_controller/follow_joint_trajectory`
  - `control_msgs/action/FollowJointTrajectory`
  - arm 5축, single point, `300..2000 ms`
  - 최신 feedback의 gripper 위치를 6번째 STM32 target으로 보존
  - success, cancel, abort와 connection-loss result 전파
- `/left_gripper_controller/gripper_cmd`
  - Jazzy `control_msgs/action/ParallelGripperCommand`
  - `JointState` command에서 gripper 1축만 허용
  - arm 5축은 최신 feedback 값으로 보존
  - hardware-safe `0..0.174873810 rad`, 고정 duration `1000 ms`
  - success, cancel, abort와 connection-loss result 전파
- arm/gripper 공용 motion arbiter
  - 어느 한쪽 goal 실행 중 다른 Action goal은 acceptance 전에 reject
- `/joint_command`
  - `trajectory_msgs/msg/JointTrajectory`
  - Action adapter 활성 시 이중 명령 방지를 위해 reject
- `/clear_fault`
- `allow_motion=false` 기본값
- serial transport, heartbeat, feedback와 SAFE_STOP 소유

현재 없는 기능:

- multi-point execution
- preempt — 동시 goal은 reject
- controller lifecycle

## 구현 방식 결정

ADR-0009에 따라 단계 5의 첫 STM32 backend는 **B안**으로 확정한다.
기존 Python transport와 safety path를 유지하고 MoveIt 표준 Action
interface를 추가한다.

### A. `ros2_control` hardware interface — 향후 대체안

```text
MoveIt
→ JointTrajectoryController / ParallelGripperActionController
→ ros2_control SystemInterface
→ STM32 binary protocol
```

장점:

- roadmap의 최종 구조와 일치
- standard controller가 trajectory와 cancel을 소유
- mock과 실제 controller 구성이 가까움

주의:

- 현재 Python serial transport를 복제하면 안 됨
- protocol/calibration 구현을 한 source로 유지할 방법 필요
- controller update rate와 STM32 약 50 Hz 실행 특성을 맞춰야 함

- firmware queue/streaming, non-blocking transport와 상태 freshness
  contract를 먼저 확정해야 함
### B. 기존 bridge의 action adapter — 단계 5 채택

```text
MoveIt
→ hardware action adapter
→ existing single_arm_bridge transport
→ STM32 binary protocol
```

장점:

- 검증된 Python transport와 safety path 재사용
- 초기 READ_ONLY와 single-point 검증 범위가 작음

주의:

- trajectory interpolation과 cancel 책임을 새 adapter가 떠안게 됨
- 장기적으로 `ros2_control` controller와 중복될 위험
- 임시 interface가 최종 architecture로 굳지 않게 종료 조건 필요

구현 원칙:

- Action adapter와 transport를 하나의 STM32 backend 경계로 관리
- serial device owner는 항상 하나
- 최초 hardware goal은 verified safe range 안의 single point로 제한
- MoveIt과 상위 task에 custom topic, raw 값 또는 binary protocol을 노출하지
  않음

### 후속 firmware safe-range 확장 gate

현재 arm/gripper raw 범위는 실제 mechanical range가 아니라 소각도 실기 시험을
통과한 보수적인 operational envelope다. Top–base 등록을 진행하는 동안에는 이
범위를 임의로 완화하지 않는다. 다음 firmware 변경 작업에 아래 항목을 함께
포함한다.

- 전원 OFF 수동 점검으로 관절별 충돌, cable tension과 기계적 끝점을 확인하고
  끝점보다 여유를 둔 보수적인 raw minimum/maximum 측정
- host calibration, STM32 firmware safe limit, hardware 전용 MoveIt limit을
  같은 측정표와 calibration hash로 갱신
- 범위 안 경계값과 범위 밖 `1 raw`에 대한 accept/reject 회귀 시험
- 낮은 속도의 관절별 단발 왕복 후 전체 팔 대표 자세 시험
- 자동 clamp, 자동 recovery와 기존 goal 재전송은 계속 금지
- B안과 향후 A안을 동시에 실행하지 않음

A안은 STM32 multi-sample queue/streaming, 제어 주기, 비동기 transport와
state semantics가 확정되고 B안의 실제 지연·jitter 측정이 끝난 뒤 별도
ADR에서 재검토한다.

## Backend exclusivity

동시에 하나만 실행돼야 한다.

- `backend:=mock`
- `backend:=isaac`
- `backend:=stm32`

ADR-0010에 따라 공식 진입점은 다음 하나로 통합한다.

```text
ros2 launch so101_bringup so101_moveit.launch.py backend:=mock|isaac|stm32
```

필수 조건:

- STM32 backend를 선택하지 않으면 serial device를 열지 않음
- Isaac backend와 STM32 backend가 같은 action을 동시에 제공하지 않음
- 통합 launch의 runtime backend guard가 non-blocking exclusive lock을 소유
- 두 번째 bringup은 backend provider를 시작하기 전에 실패
- STM32 serial은 POSIX exclusive mode로 추가 보호
- STM32 backend 선택 시 Isaac command topic을 발행하지 않음
- `backend:=stm32`만으로 `allow_motion=true`가 되지 않음
- backend 변경 후 이전 goal을 재개하지 않음

selector, launch-process guard, STM32 Action adapter와 serial exclusive open을
구현했다. 기존 mock/Isaac launch는 고정 backend 값을 통합 entrypoint에 넘기는
호환 wrapper다. mock 통합 vertical slice에서 arm/gripper Action과 세 controller가
각각 하나씩 활성화되는 것을 확인했다. STM32 선택은 `allow_motion=false`가
기본이며 실제 serial 검증은 아직 실행하지 않았다.

## Calibration ownership

current calibration:

| Joint | ID | zero | raw min..max | direction | verified project range |
|---|---:|---:|---:|---:|---:|
| base | 1 | 2048 | 2048..2389 | +1 | 약 0..0.5231 rad |
| shoulder | 2 | 2048 | 2048..2162 | +1 | 약 0..0.1749 rad |
| elbow | 3 | 2048 | 1934..2048 | -1 | 약 0..0.1749 rad |
| wrist flex | 4 | 2048 | 1934..2048 | -1 | 약 0..0.1749 rad |
| wrist roll | 5 | 2048 | 2048..2219 | +1 | 약 0..0.2623 rad |
| gripper | 6 | 2048 | 1934..2048 | -1 | 약 0..0.1749 rad |

원칙:

- raw↔radian 변환은 STM32 backend 아래에서만 수행
- firmware와 host calibration hash가 일치하지 않으면 연결 실패
- target이 firmware safe range 밖이면 clamp하지 않고 reject
- URDF physical limit을 firmware safe limit으로 간주하지 않음
- firmware safe limit을 mechanical range로 간주하지 않음

## Gripper range reconciliation gate

현재 충돌:

- MoveIt named `open`: `1.91986 rad`
- Isaac open mapping: project 약 `1.91986 rad`
- firmware verified gripper range: 약 `0..0.1749 rad`

현재 정보만으로 다음 중 어느 것이 맞는지 확정하지 않는다.

- project gripper joint가 servo shaft angle인지
- jaw mechanism angle인지
- normalized aperture를 radian처럼 사용한 값인지
- linkage mapping이 필요한지

필요한 READ_ONLY/측정 증거:

1. physical closed에서 raw와 jaw gap
2. firmware safe 범위의 여러 raw sample과 jaw gap
3. URDF gripper joint가 나타내는 물리량
4. Isaac gripper joint의 물리량
5. MoveIt `ParallelGripperCommand` position 의미

상세 sample, 중단 조건과 판정 방법은
[gripper mapping 측정 계획](PHASE_5_GRIPPER_MAPPING_PLAN.md)에 정의한다.

mapping을 확정하기 전에는 MoveIt `open=1.91986`을 STM32에 보내지 않는다.

## 검증 순서

### Gate 5A — Repository와 설계

- [x] Ubuntu local과 GitHub `main` 동기화
- [x] firmware/calibration/controller/camera/Isaac contract audit
- [x] 구현 방식 A/B 결정 — B안 채택, 향후 A안 재검토(ADR-0009)
- [x] backend exclusivity 설계 — 통합 entrypoint와 이중 lock(ADR-0010)
- [x] gripper mapping 측정 계획 승인 — motion 승인은 별도
- [x] [acceptance criteria와 rollback](PHASE_5_ACCEPTANCE_ROLLBACK.md) 승인 — motion 승인은 별도

### Gate 5B — Static/unit test

- [x] joint 이름·순서 검사
- [x] radians↔raw round trip
- [x] NaN/Inf reject
- [x] firmware safe range 밖 target reject
- [x] calibration hash mismatch reject
- [x] duplicate backend reject
- [x] incomplete trajectory reject
- [x] non-monotonic timestamp reject
- [x] cancel 상태 전이 test
- [x] reconnect 후 이전 goal 미재개 test
- [x] backend selector 허용값·기본 mock·READ_ONLY 기본값 test
- [x] launch-to-STM32 child lock handoff 검증 test
- [x] invalid/duplicate backend에서 provider process 0개

### Gate 5C — 기존 simulation 보호

- [x] 통합 `backend:=mock` vertical slice regression
- [x] Isaac adapter source 미변경 — 단계 4 결과 유지, selector unit test PASS
- [x] 실제 serial/STM32는 이 gate에서 사용하지 않음

### Gate 5D — Hardware READ_ONLY

사용자 준비 확인 후 한 번에 하나씩 실행한다.

- [x] serial 후보가 정확히 1개
- [x] `allow_motion=false`
- [x] firmware version `0x00020700`
- [x] protocol version 1
- [x] calibration hash `0x3DB42B48`
- [x] 6개 joint feedback
- [x] joint 이름·순서
- [x] q=0/sign
- [ ] 모든 current position이 firmware safe range 안
- [x] stop latch 상태
- [x] STM32 backend 외 command backend 비활성

2026-07-25 실측에서 `/joint_states`는 `4.998..4.999 Hz`였고 motion
Action은 0개였다. 최종 raw는
`2070, 2043, 2041, 2071, 2080, 1965`였다. shoulder `2043`과
wrist flex `2071`이 strict command 범위 밖이지만 firmware clear-stop
복구 범위 `strict ±40 raw` 안이다. 따라서 Gate 5D는 PARTIAL이며 실제
motion은 계속 금지한다.

host에는 이 경계 상태에서 일반 goal과 gripper goal을 거부하고, 팔 5축
all-zero q0를 정확히 `2000 ms`로 보내는 복구 goal만 허용하는 단방향
gate를 추가했다. 모든 전송 target에는 기존 strict range를 그대로
적용하며 clamp하지 않는다. 이 변경은 host fake transport에서만 검증됐고
실제 q0 motion은 아직 실행하지 않았다.

60초 counter 진단에서 `stop_latched=0`, heartbeat `0 → 599`, rejected frame
`0 → 0`을 확인했다. 같은 배포본의 `/joint_states`는 318개 이상 sample 동안
평균 `5.000 Hz`, interval `0.199..0.201 s`를 유지했다. Gate 5D의 통신·
identity·feedback 항목은 완료됐다.

별도 사용자 승인 아래 all-zero q0 `2000 ms` Action을 정확히 1회 실행했다.
Action은 `SUCCEEDED`, physical motion은 소각도·약 2초 정지·gripper 유지였고
비정상 소음·진동은 없었다. 최종 raw는
`2051, 2043, 2051, 2057, 2053, 1965`로 최대 q0 오차 9 raw였다.
q0가 one-sided strict range의 경계이므로 일부 feedback은 strict 밖이지만 기존
성공 기준 20 raw 안이다. host는 이제 strict 밖 1..20 raw는 완료 잔차로,
21..40 raw는 q0 2000 ms 전용 복구로 구분한다. command target strict와
gripper 보존 target 검사는 변경하지 않았다.

새 gate 배포 후 별도 승인 아래 BASE `+0.05 rad`, `2000 ms`를 정확히 1회
실행했다. Action은 `SUCCEEDED`, BASE raw는 `2051 → 2077`로 이동했고 목표
raw 약 2081 대비 오차는 4 raw였다. SHOULDER, ELBOW, WRIST_FLEX,
WRIST_ROLL과 GRIPPER raw는 모두 명령 전 값을 유지했다. 비정상 동작은 없었다.
별도 재승인 뒤 BASE q0 `2000 ms` 복귀도 `SUCCEEDED`였고 최종 raw는
`2051, 2043, 2052, 2057, 2053, 1965`, 최대 q0 오차는 9 raw였다.

다음 별도 승인에서 팔 5축 각각 `+0.05 rad`, `2000 ms`를 정확히 1회
실행했다. Action은 `SUCCEEDED`, 최종 raw는
`2077, 2069, 2033, 2025, 2075, 1965`였고 최대 target 오차는 ELBOW
18 raw였다. 팔 5축 모두 strict 범위 안이며 gripper는 유지됐다. motion 직후
`STATE_FEEDBACK` timeout 경고가 2회 있었지만 각각 `1/3`에서 회복됐고,
이후 60초 READ_ONLY는 평균 5.000 Hz, 경고 0회였다.

팔 5축이 strict 안인 상태에서 별도 승인 아래 gripper `0.08 rad`, 고정
`1000 ms` goal을 정확히 1회 실행했다. result position은 `0.08744 rad`,
raw 약 1991, target raw 약 1996 대비 오차 약 5 raw였다. `stalled=false`,
`reached_goal=true`, terminal state `SUCCEEDED`였고 팔 5축은 유지됐다.
completion 직후 같은 transient STATE timeout이 1회 재현되어 후속 motion을
중단했다. 첫 timeout 연장 수정 배포 뒤 gripper `0.13 rad` 복귀도
`SUCCEEDED`, result position `0.11965 rad`, 팔 5축 유지였으나 같은 경고가
다시 1회 재현됐다. 첫 수정은 실제 hardware에서 FAIL로 판정했다. host의 `GET_STATE` fresh sequence 1회 재요청 수정 뒤 같은 `0.08 rad`
복귀를 별도 승인 아래 실행했다. Action ID
`0e061e7216384339a0dfa07ecdede639`, result position `0.08744 rad`,
`SUCCEEDED`, 팔 5축 유지였으나 같은 경고가 다시 재현됐다. 즉시 재요청도
STM32 final verification busy window와 겹쳐 두 번째 수정 역시 FAIL이다.

세 번째 피드백 주기 연기 수정 뒤 별도 승인 아래 gripper `0.13 rad`,
`1000 ms`를 실행했다. Action ID `24c7d6c327d5439b8e7c1176acc8193a`,
result position `0.12272 rad`, `SUCCEEDED`, 팔 5축 유지였으나 경고 시각
`1784916484.432`는 완료 시각 `1784916485.357`보다 약 0.93초 빨랐다. 따라서
경고 원인은 completion ordering이 아니라 active trajectory 동안 STM32 servo bus
작업과 host position read가 겹치는 것이다.

현재 수정은 execution core가 active인 동안 `GET_STATE`를 보내지 않는다. Action
poll은 UART buffer에 이미 도착한 unsolicited terminal result만 수집하며, 완료 후
첫 정규 5 Hz 주기부터 실제 position feedback을 재개한다. 일반 STATE timeout은
기존 경고·fault 기준을 유지하고 motion goal은 재전송하지 않는다. 전체
111/111 test와 ROS package 5/5 build는 PASS다. Pi 재배포 뒤
READ_ONLY를 다시 통과했고, 별도 승인 아래 gripper `0.08 rad`,
`1000 ms`를 정확히 1회 실행했다. Action과 물리 동작은 정상이고 동작 중·완료
직후 WARN/ERROR는 0회였다. 완료 후 다음 정규 cycle의 `/joint_states`에서
6축이 모두 출력됐고 gripper `0.0874369 rad`, 팔 5축 유지가 확인됐다. 이 수정은
실제 hardware PASS다.

별도 승인 아래 고정 gripper target `0.13 rad`, cancel delay `300 ms` 시험을
정확히 1회 실행했다. cancel request는 수락됐고 Action result는 status 5
`CANCELED`, `reached_goal=0`, `stalled=0`이었다. bridge는 SAFE_STOP latch를
확인했고 `STATE_FEEDBACK timeout`은 없었다. 이어 별도 승인으로 `/clear_fault`를
정확히 1회 호출해 `fault cleared; commands enabled`를 확인했다. 이전 goal은
재개되지 않았고 `/joint_states` 6축이 복귀했으며 gripper는 `0.0874369 rad`,
팔 5축은 유지됐다. cancel, SAFE_STOP과 명시적 recovery는 실제 hardware PASS다.

READ_ONLY가 PASS하기 전 motion command를 제시하지 않는다.

### Gate 5E — 제한된 실제 motion

다음 조건을 모두 충족하고 사용자가 명시적으로 승인한 뒤 진행한다.

- [x] workspace 비움
- [x] 무부하
- [x] 낮은 velocity/acceleration
- [x] target이 firmware verified safe range 안
- [x] backend STM32 하나만 활성
- [x] fault latch 없음
- [x] physical q0와 feedback 일치 — 최대 9 raw, 20 raw 성공 허용치 안

시험 순서:

1. home hold
2. 단일 arm joint 소각도
3. 해당 joint home
4. arm 5축 소각도
5. arm home
6. gripper safe subrange
7. cancel
8. SAFE_STOP
9. 명시적 recovery
10. reconnect 후 이전 goal 미재개

random target과 full URDF range는 사용하지 않는다.

### Gate 5F — MoveIt end-to-end

- [x] MoveIt named home
- [x] firmware safe range 안의 representative arm trajectory
- [x] hardware-safe gripper state
- [x] cancel result 전파
- [x] SAFE_STOP result 전파
- [x] communication loss result 전파 — 자동 ROS integration과 firmware heartbeat 시험
- [x] final error 기록
- [x] 실제 controller와 `/joint_states` rate 기록

실제 cable pull은 수행하지 않았고 connection-loss Action result는 자동 integration,
firmware heartbeat 안전 정지는 기존 실기 결과로 각각 검증했다.

## 완료 기준

- MoveIt controller contract가 mock/Isaac/STM32에서 동일
- STM32 backend만 serial device를 소유
- backend 두 개 동시 실행 불가
- arm `FollowJointTrajectory` 성공·취소·실패 result 확인
- gripper hardware-safe command 성공·실패 result 확인
- firmware safe range 밖 command 100% 거부
- calibration mismatch에서 motion 0
- SAFE_STOP 후 자동 goal 재개 0
- reconnect 후 stale goal 재전송 0
- 실제 시험이 모두 낮은 속도와 verified safe range에서 수행
- test-results에 실행 환경, commit, 명령, 결과 기록

## Rollback

software rollback:

- 새 backend launch를 사용하지 않고 기존 `allow_motion=false`
  `single_arm_bridge`로 복귀
- 기존 calibration과 firmware safe limit 파일은 변경하지 않음
- mock/Isaac launch를 독립적으로 유지

hardware rollback:

- 새 command 전송 중지
- SAFE_STOP
- 자동 recovery 금지
- servo power를 안전 절차에 따라 분리
- 원인을 확인하기 전 fault clear 또는 trajectory 재전송 금지

Git rollback은 파일 삭제, `git reset --hard`, `git clean`을 사용하지 않는다.
사용자가 검토한 commit 단위로만 관리한다.

## 알려진 차단·미확정 항목

- physical E-stop 없음
- actual mechanical range UNKNOWN
- physical raw-2048 Home은 2026-07-26 사진/Isaac interactive FK 기준으로
  URDF·Isaac q0에 반영했고 자동 FK/limit/USD anchor 검증을 통과했다. 다만
  계측 기반 실물 FK, TCP와 collision parity는 아직 미통과다.
- gripper 좌우 개폐 방향은 확인했지만 raw 2048 개방 폭과 rad mapping은 미확정
- q0 변경 전 Top–base 등록 결과는 무효이며 재등록 전 기하 기반 task motion 차단
- q0 변경 후 READ_ONLY에서 arm 최대 편차 `0.02148 rad`, feedback `4.998 Hz`,
  ROS TF와 offline actual-joint FK 차이 `0.52 µm`로 내부 파이프라인 PASS.
  [시험 기록](../test-results/2026-07-26-left-arm-q0-realignment.md)
- camera의 left Wrist A/B semantic mapping UNKNOWN
- camera phase에 과거 `RIGHT` 이름이 남아 있음
- Windows/Pi local source가 GitHub와 완전히 같은지 UNKNOWN

camera naming은 단계 6 전에 해결한다. 단계 5에서는 controller와 active
hardware backend 안전을 우선한다.
