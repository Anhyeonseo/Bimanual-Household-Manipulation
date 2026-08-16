# 양팔 상단 애플리케이션 인터페이스 계약

- 계약 버전: `F8.9 proven bimanual task contract / 2026-08-16`
- 대상: Raspberry Pi 5의 상단 애플리케이션, MoveIt/FSM adapter, 이미 학습된 policy inference runtime
- STM32 기준 firmware: `0x00024809`, protocol `2`, joint count `12`
- 이 문서에서 명시하지 않은 STM32 serial/wire API는 상단 앱의 공개 API가 아니다.

## 1. 고정된 배포 identity

| 항목 | 값 |
|---|---|
| firmware | `0x00024809` |
| HEX SHA-256 | `a916a5ade13200df3572717f1c0a86c207cb5b6e91344fd9b78d276c60a619b0` |
| protocol / joints | `2 / 12` |
| capabilities | `0xEFFFFFFF` |
| left/right calibration hash | `0x2D90167E / 0x2D90167E` |
| operational-limit manifest SHA-256 | `436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263` |
| ROS command definition SHA-256 | `64526ffe0fc91c6b66bea150f283dd85e6953245dd481c4cead9aad08a938c32` |
| ROS feedback definition SHA-256 | `5d95ee42e3e91af7206542a5d912a1a24d42aa4d34f7c9c3844d873f866782ff` |

`config/bimanual_operational_limits.json`의
`general_trajectory_output_available=false`는 legacy `single_arm_bridge`
trajectory backend를 계속 금지하는 보수적 플래그다. 이번에 승인된 경로는 이
문서의 `bimanual_stream_adapter` resident 12축 경로뿐이다. 두 경로의 권한을
같은 것으로 해석하거나 이 플래그를 임의로 바꾸지 않는다.

## 2. 시스템 경계

```text
MoveIt trajectory ─┐
Task FSM ──────────┼─> 상단 command arbiter ─> ROS resident adapter ─> STM32
Pretrained policy ─┘       절대 12축 rad             protocol v2        서보 12축

카메라/태스크/정책 ─> 상단 앱
STM32 실측 12축 ─────> ROS feedback ────────────────> 상단 앱
```

- 학습은 Pi/STM32 밖에서 끝난다. Pi에는 추론 runtime과 상단 앱만 둔다.
- STM32는 MoveIt, FSM, policy를 구분하지 않는다. 항상 같은 절대 12축 목표와 시간 계약만 처리한다.
- 상단 앱은 serial device를 직접 열거나 protocol frame을 만들지 않는다.
- Pi에서 resident node 하나만 STM32 serial과 backend lease를 소유한다.
- 여러 명령 소스는 ROS service 앞의 단일 상단 arbiter에서 선택한다. 여러 프로세스가 서로 다른 `owner`로 service를 경쟁 호출하지 않는다.

F8.9는 startup torque-disable의 bounded register-40 readback recovery를
유지한다. 팔 관절의 route tracking/terminal 계약은 변경하지 않고, 정상 물체
접촉을 tracking fault로 오인하지 않도록 gripper 2축에만 150,000 µrad
route/terminal 한계와 160,000 µrad firmware hard cap을 적용한다. F8.9는
no-motion, finite reuse와 실제 left→right Top-camera 전달을 통과했다.

F8.7의 `failed_pairs`는 누적 degraded-telemetry 진단값이다. in-motion 위치
read가 1~2회 연속 실패해도 다음 성공 pair에서 streak를 0으로 복구하며,
3회 연속 실패할 때 firmware가 coordinated stop을 latch한다. 반면 측정된
tracking error 초과, DMA/dispatch fault, heartbeat timeout은 기존처럼 즉시
정지한다. 상단 앱은 누적 `failed_pairs > 0`만으로 별도 STOP을 중복 요청하지
않고 resident의 `faulted`/stop-latched 상태를 따른다.

## 3. 실행

Pi에서 양팔 12 V, STM32, ROS 환경을 준비한 뒤 실행한다.

```bash
cd /home/pi/SO101-Bimanual-Manipulation
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch single_arm_bridge bimanual_stream.launch.py \
  motion_authorized:=true
```

관찰·통합 시험만 할 때는 `motion_authorized:=false`를 사용한다. 이 상태에서는
유효한 command도 실행 전에 거부된다. 운영 앱은 다음 준비 로그와 status를 모두
확인한 뒤에만 명령을 보낸다.

```text
resident bimanual stream ready firmware=0x00024809 motion_authorized=true
```

## 4. 공개 ROS API

노드 이름은 `/bimanual_stream_adapter`다.

| 종류 | 이름 | 타입 | 의미 |
|---|---|---|---|
| service | `/bimanual_stream_adapter/command` | `so101_interfaces/srv/BimanualStreamCommand` | 유일한 motion command gate |
| service | `/bimanual_stream_adapter/status` | `std_srvs/srv/Trigger` | adapter state/owner/epoch/권한 |
| service | `/bimanual_stream_adapter/refresh_anchor` | `std_srvs/srv/Trigger` | 최초 motion 직전 torque-off 12축 anchor 재취득 |
| topic | `/bimanual_stream_adapter/anchor_joint_states` | `sensor_msgs/msg/JointState` | 명시적 refresh 또는 finite 완료의 실측 anchor |
| topic | `/bimanual_stream_adapter/joint_states` | `sensor_msgs/msg/JointState` | STM32 실측 feedback의 표준 표현 |
| topic | `/joint_states` | `sensor_msgs/msg/JointState` | 같은 12축 feedback의 MoveIt/robot_state_publisher 표준 alias |
| topic | `/bimanual_stream_adapter/feedback` | `so101_interfaces/msg/BimanualJointFeedback` | 실측값과 관절별 sample age |

`status.message`는 다음 키를 가진 JSON 문자열이다.

```json
{
  "state": "ready",
  "owner": null,
  "arbiter_epoch": 0,
  "motion_authorized": true,
  "firmware_version": "0x00024809"
}
```

### 시작과 재시작의 normative gate

새 motion session의 유일한 정상 시작 상태는 다음 조합이다.

```text
state=ready, owner=null, arbiter_epoch=0, motion_authorized=true
```

- `ready`이더라도 owner가 남아 있거나 epoch가 0이 아니면 이전 armed session이다.
- `stopped`/`faulted`에서는 `refresh_anchor`, START 또는 같은 계획 재전송을 시도하지 않는다.
- STOP은 현재 process의 terminal operation이다. resident process 종료, STM32 RESET,
  새 resident process 시작을 거쳐야 다시 위 startup gate로 돌아간다.
- HELLO의 `stop_latched=true`는 RESET이 필요한 상태다. 무인 clear/reset loop를 만들지 않는다.
- startup shadow의 status `2`/`3`은 각각 왼팔/오른팔 verified torque-disable
  실패다. firmware는 이를 latch하므로 앱이 같은 요청을 blind retry하지 않는다.
  작업자가 전원·버스 상태와 중복 resident 부재를 확인한 뒤 감독하에 reset할 수
  있으며, 같은 status가 반복되면 하드웨어/firmware 진단으로 승격한다.
- Pi backend lease와 POSIX exclusive serial open이 resident 중복 소유를 막는다.
  상단 앱은 이 보호를 우회하는 별도 serial client를 만들지 않는다.

## 5. canonical 12축 계약

명령은 정확히 다음 12개 이름을 한 번씩 포함해야 한다. 입력 순서는 달라도 되지만
중복·누락은 거부된다. 모든 feedback은 아래 순서로 발행된다.

1. `left_base_joint`
2. `left_shoulder_joint`
3. `left_elbow_joint`
4. `left_wrist_flex_joint`
5. `left_wrist_roll_joint`
6. `left_gripper_joint`
7. `right_base_joint`
8. `right_shoulder_joint`
9. `right_elbow_joint`
10. `right_wrist_flex_joint`
11. `right_wrist_roll_joint`
12. `right_gripper_joint`

공개 위치 단위는 항상 `rad`이며 목표는 velocity나 delta가 아닌 **절대 위치**다.
정책이 delta/velocity를 출력한다면 상단 앱이 최신 실측 위치에 적분하고 clamp한
뒤 절대 위치로 변환한다. raw encoder와 µrad는 상단 앱 API에 노출하지 않는다.

승인된 inclusive 명령 범위는 다음과 같다.

| joint | minimum rad | maximum rad |
|---|---:|---:|
| left base | -1.633689 | 1.523243 |
| left shoulder | -0.228563 | 3.281185 |
| left elbow | -0.681087 | 2.702874 |
| left wrist flex | -0.515418 | 2.880816 |
| left wrist roll | -2.241146 | 1.211845 |
| left gripper | -1.854583 | 0.269981 |
| right base | -1.441942 | 1.454214 |
| right shoulder | -0.289922 | 3.282719 |
| right elbow | -0.728641 | 2.686000 |
| right wrist flex | -0.598252 | 2.563282 |
| right wrist roll | -1.992641 | 1.414330 |
| right gripper | -1.919010 | 0.216291 |

Shoulder는 raw `4095→0`을 연속 통과하는 unwrapped 좌표다. 상단 앱은 이를
특별 처리하지 않고 연속 rad 값으로 유지한다. Gripper 범위는 jaw gap이 아니라
servo command 좌표다.

## 6. command service

요청:

```text
uint8 operation
string owner
string[] joint_names
uint32 splice_offset_ms
trajectory_msgs/JointTrajectoryPoint[] points
```

응답:

```text
bool accepted
string adapter_state
uint32 arbiter_epoch
string diagnostic
```

공통 규칙:

- `owner`는 비어 있으면 안 된다. 운영에서는 상단 arbiter의 고정된 문자열 하나를 세션 전체에서 사용한다.
- 각 point는 position 12개만 사용한다. `velocities`, `accelerations`, `effort`가 있으면 거부된다.
- `time_from_start`는 service 요청이 수락되는 시점에 상대적인 command-local offset이다. ROS absolute time이 아니다.
- offset과 point 간격은 양수이면서 `5 ms`의 배수여야 한다.
- 최초 START는 최소 `2` point이며 최초 lead는 최소 `20 ms`다.
- 위치 변화는 각 `5 ms`당 최대 `9,000 µrad`다. 긴 point 간격에서는 같은 비율로 허용량이 늘어난다.
- 운영 tracking error limit은 arm 10축 `90,000 µrad`, gripper 2축 `150,000 µrad`이며 최대 apply lateness는 `5 ms`다.
- `1..9 points`와 `400 ms` maximum lead는 STM32 wire window의 제한이다.
  ROS `START_FINITE`의 완전한 route 길이/종료 시각 제한으로 해석하지 않는다.

### `START_FINITE = 1`

- `ready`에서 유한 궤적 하나를 시작한다. 상단 앱은 9점을 넘거나 400 ms보다 긴
  **완전한 finite route 전체**를 한 요청으로 보낸다.
- resident는 전체 route의 증가하는 5 ms-grid timestamp, joint limit, step limit을
  ARM 전에 먼저 검증한 뒤 내부 feeder가 최대 9점/400 ms wire window로 나누어
  STM32에 공급한다. 상단 앱이 finite route를 APPEND로 수동 분할하지 않는다.
- 마지막 point tick이 정상 horizon이다.
- firmware는 마지막 dispatch 뒤에도 feedback sweep을 계속한다. arm 10축은
  최종 목표의 `46,020 µrad`, gripper 2축은 접촉 hold를 고려한 `150,000 µrad`
  이내인 완전 sweep이 2회 연속 확인되어야 terminal settle을 성공시킨다.
  최대 대기 시간은 `1000 ms`다.
- Pi adapter는 firmware 성공 뒤에도 `sample_age_ms <= 150`인 완전 실측
  snapshot을 다시 검사한다. 최종 목표와의 오차가 같은 허용치 밖이면 coordinated
  STOP/fault이며 `ready`를 노출하지 않는다.
- 다음 leg의 anchor는 명령 tail이 아니라 이 최종 실측 snapshot이다.
- 정상 완료 시 adapter는 같은 process 안에서 다시 `ready`가 된다.
- 정상 finite 완료는 torque disable이나 STOP이 아니다. 다음 finite leg를 바로 보낼 수 있고, task 종료 시에는 반드시 명시적으로 `STOP`한다.
- 최초 START로 torque가 활성화된 뒤에는 adapter가 `active`뿐 아니라 leg 사이의 armed `ready`에서도 100 ms 주기로 heartbeat를 소유한다. 상단 앱은 STM32 heartbeat를 직접 보내지 않는다. runtime serial 응답 제한은 120 ms로 두어 500 ms firmware watchdog보다 짧게 실패시킨다.

### `START_OPEN = 2`

- `ready`에서 rolling horizon을 시작한다.
- 초기 request는 `2..9` points이고 각 offset은 `400 ms` 이내다. 이후
  `APPEND`/`SPLICE`가 계속 들어와야 한다.
- open command timeout은 `100 ms`다. 상단 앱은 scheduler jitter를 고려해 **50 ms 이하 주기**로 새 batch 또는 keepalive horizon을 보낸다.

### `APPEND = 3`

- 현재 active route의 tail 뒤에 새 point를 추가한다.
- request당 `1..9` points이고 마지막 offset은 command 시점에서 `400 ms` 이내다.
- 같은 owner와 epoch를 유지한다.
- point offset은 매 APPEND 요청 시점 기준이다. 첫 새 tick은 기존 admitted tail보다 뒤여야 한다.

### `SPLICE = 4`

- 현재 미래 route 일부를 새 계획으로 교체한다.
- `splice_offset_ms`는 `20 ms` 이상이며 `5 ms` 배수다.
- 공급 point는 `1..8`개이고 첫 point offset은 `splice_offset_ms`보다 커야 한다.
- adapter가 기존 route에서 splice 시점의 정확한 continuity point를 합성한다.
- 성공하면 epoch가 증가한다. 상단 앱은 응답 epoch를 새 기준으로 사용한다.

실기에서 검증된 모양은 `splice_offset_ms=100`, replacement offsets
`150, 200 ms`다. 이 값은 유일한 값이 아니라 안전하게 검증된 예다.

### `STOP = 5`

- point 없이 현재 owner로 호출한다.
- 양팔 SAFE_STOP, verified torque disable, stop latch를 하나의 coordinated operation으로 수행한다.
- 성공 후 state는 `stopped`이며 같은 node에서 새 motion을 재개하지 않는다.
- 다음 세션은 STM32 RESET 후 resident node를 다시 시작한다.

## 7. owner, epoch, state

```text
startup -> ready(owner=null, epoch=0)
  START -> active(owner=X, epoch=1)
    APPEND -> active(epoch 유지)
    SPLICE -> active(epoch + 1)
    finite 정상 완료 -> ready(owner=X, epoch 유지)
    STOP/fault -> stopped 또는 faulted
```

- owner는 firmware mode가 아니라 host-side single-writer lease다.
- `ready(owner=null, epoch=0)`는 torque-off인 **unarmed READY**다. 첫 경로는
  반드시 fresh anchor에서 만든다.
- finite 완료 뒤 `ready(owner=X, epoch>0)`는 torque와 heartbeat가 유지되는
  **armed READY/HOLD**다. 다음 leg를 토크 공백 없이 시작할 수 있지만 새 독립
  session의 startup 상태로 재사용하면 안 된다.
- 첫 START가 owner를 claim한다. 다른 owner의 호출은 거부된다.
- 같은 owner의 다음 START와 SPLICE는 epoch를 증가시킨다.
- 응답의 epoch를 예상값과 비교하되, 상단 앱이 epoch를 임의로 지정하지 않는다.
- `accepted=false`면 blind retry하지 않는다. 즉시 status를 읽고 motion 생산을 중단한다. `active`라면 같은 owner로 STOP을 시도하고, `faulted/stopped`면 reset 절차로 간다.

## 8. feedback 계약

`/feedback`은 기본 `20 Hz`로 다음 값을 보낸다.

- `joint_names[12]`: canonical 순서
- `positions[12]`: STM32가 보존한 실측 위치, rad
- `sample_age_ms[12]`: 각 위치 표본의 STM32 기준 나이
- `present_mask`: 정상 완전 표본은 `0x0FFF`
- `firmware_tick_ms`: STM32 timebase
- `completed_pairs`: 좌우 같은 관절을 함께 읽은 pair의 누적 완료 수

정책 또는 closed-loop 보정 입력에 사용할 때는 최소한 다음을 검사한다.

1. 정확히 12축이며 이름이 canonical과 일치
2. `present_mask == 0x0FFF`
3. 모든 position이 finite
4. 필요한 모든 관절의 `sample_age_ms <= 150`
5. active 중 `completed_pairs`가 진행

`header.stamp`는 Pi ROS publish 시각이고, `firmware_tick_ms`는 STM32 시각이다. 서로 같은 clock으로 빼지 않는다.

중요한 현재 동작:

- node 시작의 `ready`에서는 torque-off shadow read가 cache를 seed한다. 이 unarmed `ready`에는 heartbeat가 필요하지 않다.
- 최초 START 뒤의 armed `ready`와 `active`에서는 resident adapter가 heartbeat를 계속 보낸다. 상태 조회가 잦아도 status callback과 독립 timer가 같은 rate-limited keepalive를 공유한다.
- active tracking 중에는 양팔 관절 pair가 순환 갱신되어 실측값이 fresh해진다.
- 최초 unarmed `ready`의 startup cache는 동작 기준으로 직접 쓰지 않는다. 상단
  arbiter는 경로 생성 직전에 `/refresh_anchor`를 호출하고 새로 발행된 anchor로
  첫 절대 경로를 만든다. resident는 첫 START 내부에서도 두 버스를 다시 읽어
  경로 연속성을 검증하므로, 오래된 anchor 기반 경로는 ARM/ENABLE 전에 거부된다.
- `stopped`에서는 feedback publish를 중단한다.

## 9. 상단 소스별 변환

### MoveIt

- MoveIt 결과를 canonical 12축 절대 rad로 재배열한다.
- 한 팔 계획이어도 다른 팔 6축을 최신 hold target으로 채워 12축을 만든다.
- timestamp를 5 ms grid로 resample하고 joint/step/limit을 service 호출 전에 검증한다.
- 완결된 경로는 `START_FINITE`; 실행 중 재계획은 `SPLICE`를 사용한다.

### Task FSM

- gripper도 같은 12축 vector의 index 5/11이다. 별도 STM32 gripper API를 만들지 않는다.
- 물리적으로 멈춰야 하는 grasp/release에는 finite horizon과 명시적 state gate를 사용한다.
- active 중 사용자 중지, vision stale, producer/planning failure는 STOP으로 수렴한다.
  이미 finite가 정상 완료된 armed READY에서 contact/정밀도 semantic gate만 실패한
  경우에는 새 명령을 막고 `HOLD_REQUIRED`로 전환한 뒤 작업자가 STOP을 결정한다.

### Pretrained policy

- policy output rate, observation preprocessing, camera order와 model SHA를 별도 deployment bundle에 고정한다.
- delta/velocity 출력은 상단 앱이 절대 rad로 적분한다.
- 첫 2개 이상 point로 `START_OPEN`, 이후 50 ms 이하 주기로 `APPEND`한다.
- 새 policy rollout이나 visual residual이 기존 미래 route를 대체하면 `SPLICE`한다.
- stale observation, NaN/Inf, limit 위반, missed deadline은 command를 보내지 않고 STOP한다. 마지막 action을 재사용하지 않는다.

## 10. 안전 및 실패 처리

- `motion_authorized` 기본값은 false다.
- heartbeat timeout, command timeout, queue/timeline 오류, apply lateness, tracking error, 한쪽 DMA/UART/feedback 실패는 fail-closed coordinated stop 대상이다.
- 한 팔만 계속 움직이는 fallback은 없다.
- service timeout이나 ROS graph 단절을 성공으로 간주하지 않는다.
- STOP 뒤 torque-off를 전제로 팔이 처질 수 있으므로 물리적으로 지지한다.
- 물리 E-stop/12 V 차단은 ROS STOP과 별개의 마지막 수단으로 유지한다.
- fault 후 자동 clear/retry/reset loop를 구현하지 않는다. 작업자 확인 뒤 reset한다.
- transport/firmware/feedback 안전 실패와 task semantic 실패를 구분한다. 전자는
  coordinated STOP으로 수렴한다. 반면 finite가 정상 완료되어 armed READY가 된 뒤
  contact/vision/정밀도 같은 상위 판정이 실패했다면 새 motion을 차단하고 현재
  torque hold를 보존한 `HOLD_REQUIRED`로 보고할 수 있다. 팔을 지지할 준비 없이
  예외 처리만으로 즉시 torque-off하지 않는다.
- arm terminal acceptance는 firmware/resident와 같은 `46,020 µrad`, gripper는
  접촉 hold용 `150,000 µrad`다. 앱이 별도의 더 엄격한 30 mrad 같은 중복
  gate를 만들어 정상 firmware 완료를 실패로 뒤집지 않는다.

## 11. 검증된 evidence

| 검증 | 핵심 결과 | artifact SHA-256 |
|---|---|---|
| host link 30분 | 90,000 frames, 32,000 B/s, error 0 | `34bca9414863ae9e4edce021a2d03f06a267c34bb4765c232a33b57c2bd8b659` |
| F7 paired dispatch | start skew max 2 µs, launch lateness max 46 µs | `a3cf28c3e209b8fcaa0a64bc8f90b690e2faeed163ac3ac8384cd617ce3a6ae4` |
| F7 fault stop | right DMA fault 후 양팔 torque disable | `55e617941ac75754f9ea44723fa8fa3772500c2e889e7c7e29b1ec693fbfce8e` |
| F8 tracking hold | 35/35 pairs, latency max 2 ms | `ad1c6aad9e887742435331c641b7582d9a4e8f221c399fa3c30cec96f8e6f21e` |
| F8 tracking fault | 100,000 µrad 오류 검출, 양팔 stop | `365c7e66b796c736757f5b560f65437f9c7fe778cb052caadefe99629c860937` |
| F8.1 direct no-output | mask 0xFFF, age 57 ms, launch 0 | `18c520f293835195a948ea63524873287006ab1e6be74ecb99306bb051bae6e6` |
| F8.1 ROS no-motion | 12축, ready, motion 차단 | `971d4876dda7443ff32fe82b3bda9d11055a48ee6067468210f1d250101785a8` |
| F8.1 rolling actual | +0.03 rad 왕복, feedback age max 27 ms, STOP | `055d79fef4b0590f439fdd5943e2a024ab7e85e839d7e9ddc4221d14257ba6ec` |
| F8.7 ROS no-motion + fresh anchor | torque-off, mask 0xFFF, age 6 ms, motion 차단 | `ff3c168d178b165b1dccebf62fa6bf663a4ca2ae7ebccb8e78331989f9cddb84` |
| F8.7 resident finite reuse | current-pose hold 2회, epoch 1/2, 매 leg `ACTIVE -> READY`, 명시적 STOP | `019c84f95207c06cf2ff3c1727510145734fd76fd9f40b839a0423478fec82df` |
| Top-camera application run20 | fresh anchor, Q0 + 6 actions, epoch 7, 최종 armed READY/HOLD | `67d2d1de5035c937c670a5f23ed0447392479ec81145c607a00ec4ca41aebd1a` |
| Top-camera application run22 | 두 번째 end-to-end Pick/Place, arm error max 21.476 mrad, 최종 armed READY/HOLD | `c887c8c723a5b870841cd404ab7673040f7dd0e26c58994ea068c45d0f1edd4c` |
| F8.9 ROS no-motion | 12축 ready, motion 차단 | `248ee592fa6dd9f68134574afd4a21ff5679bf939cffa01c7e8d7cd652c687d8` |
| F8.9 resident finite reuse | current-pose hold 2회, epoch 1/2, 명시적 STOP | `860f626d2e8a6e5ec5a5bcc5f3a38952ce67ef751b482950168dc6ff562a5f41` |
| F8.9 left→right pen transfer | fresh plan/validate/execute 2회, retry 0, 최종 READY/HOLD | `408c21d6e7211834351123c5058cf7a8be50b8d20d064ec3f861230099198fbc` |

F8.9 session03은 source-image x로 왼팔과 오른팔을 순서대로 선택하고,
각 팔에서 fresh plan과 SHA를 생성했다. gripper open/close, pick/lift/place/release,
q0 복귀를 automatic retry 없이 완주하고 최종 torque hold를 유지했다.

## 12. 상단 앱 완료 조건

- exact identity가 다르면 motion을 시작하지 않는다.
- ROS API만 사용하며 serial direct access가 없다.
- 모든 명령이 canonical absolute 12축 rad다.
- finite/open/append/splice/stop을 하나의 owner로 처리한다.
- long finite route를 한 번에 제출하고 resident 내부 wire feeder에 맡긴다.
- feedback freshness gate와 stale STOP이 자동시험으로 증명된다.
- unarmed READY, armed READY/HOLD, STOPPED/FAULTED를 서로 다른 상태로 처리한다.
- terminal acceptance 수치를 firmware/resident 계약에서 가져오며 더 엄격한 앱
  상수를 중복 정의하지 않는다.
- MoveIt/FSM/policy source 전환은 firmware가 아니라 상단 arbiter에서 이뤄진다.
- command reject, service timeout, node death, stale vision/policy, limit 위반을 각각 fault injection하여 양팔 STOP 또는 motion 미시작을 확인한다.
- 실제 task 확대 전에는 별도로 양팔 URDF/base transform, self/inter-arm collision, 카메라 보정과 task-level 반복성을 검증한다. 이들은 firmware stream 수락과 별도 gate다.
