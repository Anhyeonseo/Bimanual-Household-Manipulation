# 상단 애플리케이션 개발 인계 프롬프트

아래 본문을 상단 애플리케이션을 개발하는 새 작업 세션에 그대로 전달한다.

---

당신은 `SO101-Bimanual-Manipulation` 저장소의 Raspberry Pi 5 상단
애플리케이션을 구현한다. 학습 코드를 STM32나 Pi firmware 계층으로 옮기는 작업이
아니다. 외부에서 이미 학습·검증된 policy inference, MoveIt trajectory, task FSM을
하나의 source-agnostic 양팔 command interface에 연결하는 작업이다.

먼저 다음 파일을 읽고, 코드와 문서가 다르면 추측하지 말고 fail-closed로 보고하라.

1. `docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md`
2. `ros2_ws/src/so101_interfaces/srv/BimanualStreamCommand.srv`
3. `ros2_ws/src/so101_interfaces/msg/BimanualJointFeedback.msg`
4. `ros2_ws/src/single_arm_bridge/single_arm_bridge/bimanual_stream_adapter.py`
5. `ros2_ws/src/single_arm_bridge/single_arm_bridge/bimanual_stream_node.py`
6. `config/bimanual_operational_limits.json`
7. `docs/TOP_CAMERA_RESIDENT_PICK_PLACE_APPLICATION.md`
8. `tools/run_top_pick_place_application_once.py`

7~8은 source-specific firmware API가 아니라, 동일한 resident 계약 위에서 실제
카메라 Pick/Place를 두 번 완주한 상단 앱 reference다. 해당 one-shot 도구를 운영
daemon으로 복사하지 말고 상태/실패/torque-hold 패턴을 재사용하라.

## 고정된 하위 계약

- STM32 firmware `0x00024809`
- protocol 2, 12 joints, capabilities `0xEFFFFFFF`
- left/right calibration hash `0x2D90167E`
- firmware HEX SHA-256
  `9a9cd49247428478cae831d948977274d1188e9b0b0756d02de8c7c47fd431aa`
- 누적 `failed_pairs`는 degraded-telemetry 진단값이다. firmware가 in-motion
  read 3회 연속 실패에서 정지하므로, 상단 앱이 단발 누적값만 보고 STOP을
  중복 요청하면 안 된다. 실제 tracking error/DMA/dispatch/heartbeat fault는
  즉시 정지한다.
- motion API는 `/bimanual_stream_adapter/command` 하나뿐이다.
- 상단 앱은 serial port나 protocol-v2 wire frame을 직접 다루면 안 된다.
- 출력은 항상 canonical 12축 absolute radians다. delta/velocity/raw를 service에
  직접 보내면 안 된다.
- owner는 상단 command arbiter의 고정 문자열 하나를 세션 전체에서 사용한다.
- 정상 새 session 시작 상태는 정확히 `ready + owner null + epoch 0`이다.
  finite 완료 뒤의 `ready + owner non-null + epoch > 0`은 torque가 유지되는
  armed READY/HOLD이며 같은 session의 다음 leg에만 사용한다.
- 최초 절대 경로를 만들기 직전에 `/bimanual_stream_adapter/refresh_anchor`를
  호출하고, 그 호출 뒤 새로 발행된 `anchor_joint_states`만 시작 자세로 사용한다.
  startup 시 latched된 오래된 anchor를 motion 기준으로 사용하지 않는다.
- firmware는 MoveIt/FSM/policy source를 구분하지 않는다. source arbitration은
  반드시 상단 앱에서 끝낸다.
- arm terminal acceptance는 `46,020 µrad`, gripper contact-hold acceptance는
  `150,000 µrad`다. 앱에서 더 엄격한 30 mrad 같은 중복 판정을 만들지 않는다.
- ROS `START_FINITE`는 **완전한 finite route 전체**를 받는다. 9 points/400 ms는
  resident가 내부에서 관리하는 STM32 wire window이며 ROS finite route 제한이 아니다.

## 구현 목표

다음 모듈을 억지로 거대한 노드 하나에 넣지 말고, 책임이 실제로 분리되는 경계로
구성하라.

1. **Observation gate**
   - `/bimanual_stream_adapter/feedback` 구독
   - canonical 12 names, `present_mask == 0x0FFF`, finite positions 검사
   - active policy/closed-loop 입력은 모든 필요한 축의 `sample_age_ms <= 150`
   - stale/누락/NaN/clock regression 시 새 action을 만들지 않고 STOP 요청
   - ROS header clock과 STM32 `firmware_tick_ms`를 직접 빼지 않음

2. **Command arbiter**
   - MoveIt, task FSM, pretrained policy 중 동시에 하나만 선택
   - producer별 우선순위와 명시적 handoff state
   - 한 producer가 active인 동안 다른 producer의 command를 service로 통과시키지 않음
   - 하나의 stable owner를 사용하고 응답 epoch를 추적
   - `accepted=false`, service timeout, unexpected state/epoch를 terminal fault로 처리
   - 최초 motion 전에 refresh-anchor 응답과 새 anchor 수신을 모두 확인
   - startup의 `stopped`, `faulted`, owner non-null 상태에서 refresh/START 금지

3. **Canonical trajectory layer**
   - 입력 joint names를 정확한 12축으로 정규화
   - rad absolute target, inclusive operational limit, 5 ms grid,
     9,000 µrad/5 ms step limit 검사
   - 한 팔 MoveIt 경로는 반대 팔을 최신 hold target으로 채워 12축으로 변환
   - gripper를 별도 firmware API로 빼지 않고 index 5/11에 포함

4. **Finite adapter**
   - 완결된 MoveIt/FSM 경로를 `START_FINITE`로 변환
   - 최소 2 points와 최초 20 ms lead를 준수하되 완전한 long route를 한 번에 제출
   - finite 경로를 9점씩 APPEND로 재구현하지 않고 resident feeder에 맡김
   - 정상 완료 후 `ready` 전이를 기다리고 다음 leg를 이어 보냄
   - finite 정상 완료가 torque-off가 아님을 전제로 task 끝에 반드시 STOP

5. **Rolling policy adapter**
   - 초기 2개 이상 target으로 `START_OPEN`
   - 100 ms firmware timeout보다 충분히 빠른 50 ms 이하 scheduler로 APPEND
   - replanning/residual 교체는 continuity가 있는 SPLICE
   - 검증된 초기값은 splice lead 100 ms, replacement target 150/200 ms
   - inference miss나 stale observation 때 마지막 action을 재전송하지 말고 STOP

6. **Supervisor and shutdown**
   - startup status가 `ready`, owner null, epoch 0, motion 권한 true, firmware
     `0x00024809`인지 확인
   - 정상 task 종료, 사용자 중지, SIGINT/SIGTERM, source crash, camera/policy stale,
     planning 실패가 모두 같은 coordinated STOP 경로로 수렴
   - STOP 뒤 state `stopped` 확인. 자동 reset/clear/retry loop는 만들지 않음
   - 다음 세션은 작업자 확인과 STM32 RESET 뒤 새 resident node로 시작
   - transport/safety fault는 STOP. 정상 finite 완료 뒤 task-level contact/vision
     판정 실패는 새 command를 차단하고 `HOLD_REQUIRED`로 노출하여 torque hold를
     보존할 수 있음. 작업자가 팔을 지지한 뒤 명시적으로 STOP
   - startup shadow status 2/3은 verified torque-disable bus failure로 보고하고
     blind retry하지 않음

## 상태 모델

다음 모델을 코드와 테스트의 기준으로 사용하라.

```text
BOOTSTRAP -> READY_UNARMED(owner=null, epoch=0)
              -> ACTIVE_FINITE -> READY_ARMED_HOLD -> ACTIVE_FINITE
              -> ACTIVE_OPEN -------------------------> STOPPING -> STOPPED
READY_ARMED_HOLD -- task semantic rejection ---------> HOLD_REQUIRED
any transport/safety fault --------------------------> FAULT/STOPPED
```

Firmware/adapter 상태를 감추는 별도 낙관적 상태를 만들지 말고, 상단 상태는
ROS status와 마지막 command response로 증명되어야 한다.

## 반드시 작성할 자동시험

- exact 12 joint order와 shuffled input normalization
- duplicate/missing/extra joint rejection
- NaN/Inf, rad limit 양 끝/바깥, maximum step 경계
- 5 ms가 아닌 timestamp, rolling point lead >400 ms 거부
- 147/184 point처럼 400 ms보다 긴 완전한 finite route가 ROS 요청에서 수락되고,
  resident가 9-point/400-ms wire window로만 내부 분할하는지 확인
- START_FINITE 정상 2회와 epoch 진행
- startup anchor 뒤 팔이 처진 경우 최신 anchor를 재취득하고, stale 경로는 ARM
  전에 거부되는지 확인
- START_OPEN → APPEND → SPLICE → APPEND → STOP
- 다른 producer가 active session을 가로채지 못함
- stale feedback, incomplete mask, stopped feedback, service timeout에서 STOP
- MoveIt 한 팔 경로가 반대 팔 hold를 보존
- policy delta/velocity가 absolute rad로 변환되고 limit에서 clamp 또는 reject
- SIGINT/SIGTERM과 producer exception에서 STOP exactly once
- arm terminal `42,951 µrad`는 수락하고 `47,000 µrad`는 거부하며,
  gripper는 별도 `150,000 µrad` 경계를 쓰는지 확인
- armed READY의 post-terminal task rejection이 새 motion을 막되 torque hold를
  보존하고, 작업자 STOP 뒤 `stopped`를 새 session으로 재사용하지 않는지 확인
- `motion_authorized=false` integration에서 모든 motion command 거부

Mock 시험은 service response, status transition, feedback age를 모두 모델링해야 한다.
실제 로봇 시험은 별도 confirmation 문자열과 artifact SHA를 남기는 one-shot 도구로
분리한다. 테스트를 통과시키기 위해 safety gate를 완화하지 않는다.

## 금지 사항

- STM32 firmware에서 policy 종류를 분기하지 말 것
- 상단 앱이 `/dev/ttyACM*` 또는 `/dev/serial/by-id/*`를 열지 말 것
- left/right 독립 command stream 두 개를 만들지 말 것
- raw 0..4095 wrap을 상단 앱에서 임의 해석하지 말 것
- `accepted=false`를 같은 요청 재전송으로 복구하지 말 것
- stale observation에서 마지막 action 유지/replay하지 말 것
- fault 후 무인 자동 reset, clear-fault, motion resume을 만들지 말 것
- legacy `single_arm_bridge` trajectory backend를 새 resident 경로와 혼용하지 말 것
- ROS finite route를 firmware wire batch 크기에 맞춰 수동 APPEND 분할하지 말 것
- `stopped` 상태에서 refresh-anchor 또는 START를 재시도하지 말 것
- firmware가 성공시킨 arm terminal을 별도 30 mrad 앱 gate로 실패 처리하지 말 것

## 산출물

1. source-agnostic upper command arbiter
2. MoveIt finite adapter
3. pretrained-policy rolling adapter
4. task FSM/gripper adapter
5. observation freshness gate와 supervisor
6. launch/config와 systemd-safe shutdown 경로
7. 단위·통합·fault-injection 테스트
8. 운영자용 bringup/runbook
9. policy deployment bundle contract(model SHA, input/output, control period,
   preprocessing, camera order)
10. 실제 하드웨어에 명령을 보내지 않는 dry-run artifact
11. F8.9 current reference evidence와의 compatibility report:
    - `artifacts/resident_adapter/2026-08-16/f89_no_motion_run01.json`
      SHA-256 `248ee592fa6dd9f68134574afd4a21ff5679bf939cffa01c7e8d7cd652c687d8`
    - `artifacts/resident_adapter/2026-08-16/f89_armed_ready_soak_run01.json`
      SHA-256 `860f626d2e8a6e5ec5a5bcc5f3a38952ce67ef751b482950168dc6ff562a5f41`
    - `artifacts/top_pick_place/2026-08-16/pen_interarm_continuous_session03/transfer_journal.json`
      SHA-256 `408c21d6e7211834351123c5058cf7a8be50b8d20d064ec3f861230099198fbc`
12. F8.7 predecessor evidence와의 compatibility report:
    - `artifacts/resident_adapter/2026-08-15/no_motion_fresh_anchor_24807_run01.json`
      SHA-256 `ff3c168d178b165b1dccebf62fa6bf663a4ca2ae7ebccb8e78331989f9cddb84`
    - `artifacts/resident_adapter/2026-08-15/current_pose_hold_twice_fresh_anchor_24807_run01.json`
      SHA-256 `019c84f95207c06cf2ff3c1727510145734fd76fd9f40b839a0423478fec82df`
    - `artifacts/top_pick_place/2026-08-15/application_run20.json`
      SHA-256 `67d2d1de5035c937c670a5f23ed0447392479ec81145c607a00ec4ca41aebd1a`
    - `artifacts/top_pick_place/2026-08-15/application_run22.json`
      SHA-256 `c887c8c723a5b870841cd404ab7673040f7dd0e26c58994ea068c45d0f1edd4c`

구현 전에는 먼저 현재 저장소 구조를 검사하고 기존 모듈을 재사용하라. 공개 계약을
바꿔야 한다면 변경 이유, 영향 받는 firmware/ROS version, migration plan, 새
no-motion/actual-motion gate를 제안하고 승인 전에는 구현하지 말라.

---
