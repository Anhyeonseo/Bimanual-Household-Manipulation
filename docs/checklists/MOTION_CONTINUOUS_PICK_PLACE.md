# Motion-13 — 연속 Pick/Place (buffered leg 3개 + gripper 2회)

`docs/checklists/MOTION_G474_BUFFERED_PHYSICAL_ROUTE.md` 의 마지막 미체크
박스 `[ ] 연속 Pick/Place` 를 닫기 위한 체크리스트.

- 펌웨어: `0x00022900` (배포·검증 완료)
- 계약: `continuous_pick_place_candidate`, `deployed: false` / `motion_authorized: false`
- 경로: `full_pick_place_plan_only_manifest.json`
  SHA `a5ed8d0335e534ef49eff056dd4b3e6415598a613183c9bebca43f12c7d8c405`

## 왜 3분할인가 (설계 근거)

`Servo_MotionSafetyBegin`/`Poll` 은 비버퍼드 경로(`Host_ServiceBinaryMotion`)
에만 있다. **buffered 실행에는 load/current 감시가 없다** — `0x00022700` 에서
servo read 가 host UART 처리를 굶겨 제거했기 때문이다.

단일 Action 으로 gripper 를 sample stream 안에서 닫으면, 물체를 문 뒤 남은
약 60초 동안 gripper 가 명령 위치에 도달하지 못한 채 stall 하고 그 구간
전체가 무감시가 된다. leg 경계를 gripper 동작 지점에 두면 접촉은 감시가 있는
기존 gripper 명령 경로에서 일어난다. 사이에 q0 복귀가 없으므로 팔 입장에서는
여전히 연속이다.

이 경계는 계약의 exact-match dict 가 지킨다
(`test_continuous_pick_place_route_is_plan_only_and_split_at_the_gripper`).

## 운영 규칙 — torque OFF 와 중력 (2026-08-06 실측)

**q0 는 중력 안정 자세가 아니다.** `04:46` 에 q0(오차 `0.009204 rad`)로 복귀한
팔이 torque OFF 상태로 45분 뒤 팔꿈치 raw `2491` 이었다. 상한 `2258` 을 `233`
넘었고 q0(`2048`)에서 `443 raw` 움직였다.

정착 자세가 관절 계약 밖이므로 **"중력 정착 자세에 park 한다" 는 해법이
존재하지 않는다.** 그리고 범위를 벗어나면 `CLEAR_FAULT` 가 거부된다
(`Host_BinaryClearStopIsSafe` 는 `min-40 .. max+40` 을 요구한다). 그러면 다음
세션은 손으로 자세를 고쳐야 시작할 수 있고, 시연 중이면 그대로 멈춘다.

따라야 할 것:

- [ ] **한 세션 = 한 bridge 인스턴스.** 단계 사이에 bridge 를 죽이지 않는다.
      죽이면 DISABLE 이 걸리고 팔이 처진다. 진단 counter 를 읽으려고 중간에
      종료하지 않는다 — 그것 때문에 2026-08-06 에 복구를 세 번 반복했다.
- [ ] **DISABLE 은 운영자가 팔을 받친 상태에서만.** `A3-safe-shutdown.sh` 가
      받침 확인 → DISABLE → 6축 torque readback → 종료 자세 판정 순서를 강제한다.
- [ ] 종료 자세가 범위 밖이면 기록하고, 다음 세션 시작 전에 손으로 옮긴다.

> 소프트웨어로 막을 수 없는 항목이다. 양팔 수건접기 시연에서 이 상태가 되면
> 팔이 그 자리에서 멈추므로, 비동기 펌웨어 항목과 같은 급으로 다룬다.

## 자세 사전 게이트

gripper 명령이어도 `prepare_parallel_gripper_goal` 이 **6축 전체 피드백**을
검증한다. 팔이 처져 있으면 gripper 만 움직이는 명령조차 시작되지 않는다.
2026-08-06 probe 1차가 여기서 거부됐다:

```
gripper goal rejected: joint feedback is outside safe range:
  ELBOW: target raw 2491 outside 627..2258
```

- [ ] 어떤 동작 전에도 6축이 계약 범위 안인지 먼저 확인한다

## 계획 (하드웨어 없음)

- [x] 경로 7개 phase 가 전부 관절공간 직선임을 확인 (최대 이탈 `2.2e-16` rad)
- [x] 이음매 불일치 `0.0` 확인
- [x] key pose 8개를 manifest 에서만 유도 (손으로 적은 pose 0개)
- [x] leg 별 duration 자동 탐색, 모델 peak 오차 ≤ 70 raw 목표
- [x] **stage 간 추종 오차 이어받기** — 경유점마다 팔이 목표에 정확히
      있다고 가정하지 않는다
- [x] admission 시뮬레이션 underflow 0 (세 leg 전부 `input_complete`)
- [x] batch 상한 9 준수
- [x] gripper 가 buffered leg 안에서 움직이지 않음을 시뮬레이션으로 확인
- [x] anchor 이탈 한계 40 raw 게이트
- [x] leg 별 승인 문구 분리
- [x] 전체 회귀 통과

| leg | 경로 | 시간 | sample | 모델 peak |
|---|---|---:|---:|---:|
| A | q0 → pick_pregrasp → pick_grasp | 41.0 s | 2051 | 65.03 raw |
| B | pick_grasp → lift20 → place_pregrasp → place | 14.0 s | 701 | 42.22 raw |
| C | place → retreat → q0 | 39.0 s | 1951 | 58.09 raw |

## 실기 순서

### 0. gripper 접촉 probe — **먼저 해야 한다**

**아직 실측된 적이 없는 것:** 물체를 문 gripper 가 action 결과로 무엇을
보고하는가.

`ParallelGripperCommandActionAdapter._finish_goal` 은 실행이 SUCCEEDED 일
때만 `reached_goal=True` 를 낸다. firmware 의 최종 정착 검사는
`SERVO_FINAL_ERROR_TOLERANCE_RAW = 30` 인데, open(raw 2009) 과
close(raw 1963) 사이 전체 이동량이 **46 raw 뿐**이다. 물체가 그 46 중 얼마를
막느냐에 따라 정착 오차가 30 을 넘을 수도, 안 넘을 수도 있다. 넘으면 abort
이고 stop latch 까지 갈 수 있으며, 그러면 다음 leg 가 시작되지 않는다.

Stage 7 의 supervised 실행은 "gripper close and verified object hold" 를
기록했지만 명령값도 action 결과도 남기지 않았다.

- [ ] 팔은 q0 에 두고, gripper 앞에 **실제 사용할 물체**를 손으로 대고
      `--label probe --expect report` 로 close 1회
- [ ] `REACHED_GOAL` / `RESIDUAL_GAP_RAW` / `ACTION_STATUS` 기록
- [ ] latch 되었는지 확인
- [ ] open 1회로 되돌리고 같은 값 기록
- [ ] 결과를 계약의 `gripper_contact_behavior_measured` 에 반영

> **판정:** `RESIDUAL_GAP_RAW > 30` 인데 action 이 succeeded 면 정착 검사가
> gripper 를 그렇게 보지 않는다는 뜻이다. abort 면 close 목표값을 물체
> 두께에 맞게 올려야 하고, 그 값이 계약에 들어가야 한다.
> **이 결과를 보기 전에는 leg A 를 보내지 않는다.**

### 1. leg A — q0 → pick grasp

- [ ] 12 V ON, 주변 정리, 비상 정지 접근 가능
- [ ] bridge 를 `ros2 launch single_arm_bridge bridge.launch.py` 로 기동
      (`ros2 run` 은 `bridge.local.yaml` 을 싣지 않아 READ_ONLY 가 된다)
- [ ] Pi/desktop 양쪽 `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- [ ] `mode=MOTION_ENABLED` 확인
- [ ] desktop 에서 `/joint_states` 도달 확인
- [ ] fresh anchor 포착, spread 0
- [ ] 계획 생성 → `ANCHOR_DEVIATION_RAW` 전 축 ≤ 40
- [ ] Action 1회, 재시도 0
- [ ] terminal `succeeded`, apply lateness ≤ 5 ms, post-settle ≤ 30 raw
- [ ] `lateness_buckets` 기록

### 2. gripper close

- [ ] probe 결과에 따라 `--expect` 결정
- [ ] 1회 전송, `ARM_MOTION_RAD` ≤ 0.02 확인 (팔이 움직이면 안 된다)
- [ ] 물체를 실제로 물었는지 **눈으로** 확인

### 3. leg B — grasp → place

- [ ] fresh anchor, 이탈 ≤ 40 raw
- [ ] Action 1회
- [ ] terminal 통과, `lateness_buckets` 기록
- [ ] **물체가 이동 중 떨어지지 않았는지 눈으로 확인**

### 4. gripper release

- [ ] `--expect reached` (물체를 놓으면 gripper 는 명령 위치에 도달해야 한다)
- [ ] 물체가 놓인 위치를 눈으로 확인

> **A4 의존:** 공칭 Place offset `0.025 m` 는 Stage 7 에서 `-5 mm` 보정 2회를
> 필요로 했다(실측 후보 ≈ `0.015 m`). 이 회차의 place 자세는 그 보정 전
> 값이므로 물체가 예상보다 높은 데서 놓일 수 있다. Motion-13 은 기구를
> 증명하고, 값은 A4 에서 고친다.

### 5. leg C — place → q0

- [ ] fresh anchor, 이탈 ≤ 40 raw
- [ ] Action 1회
- [ ] terminal 통과, q0 도달
- [ ] physical DISABLE + torque-OFF readback
- [ ] 12 V OFF

## 전체 수락 기준

- [ ] 3개 Action 전부 terminal `succeeded`
- [ ] 각 Action apply lateness ≤ 5 ms, post-settle ≤ 30 raw
- [ ] gripper 2회 모두 팔 비명령 동작 0
- [ ] 물체 실제 파지·이동·해제
- [ ] 자동 재시도 0회
- [ ] leg 사이 q0 복귀 0회
- [ ] servo bus 오류 counter delta 0
- [ ] 세 leg 의 `lateness_buckets` 합산 기록

## 유예 항목

- [ ] Place TCP-to-contact offset 재계측 (A4)
- [ ] 10회 pilot (A5) — A4 이후
- [ ] 50회 benchmark — 손목 visual correction 이후로 유예
