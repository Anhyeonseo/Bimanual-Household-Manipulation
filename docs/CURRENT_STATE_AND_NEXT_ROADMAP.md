# 현재 상태와 다음 로드맵

- 기준일: 2026-08-15
- 현재 기준선: STM32 F8.7 `0x00024807`, protocol v2, 12축 resident 실행
- 목적: 검증된 firmware/ROS 경계를 동결하고 상단 애플리케이션과 pretrained
  policy 개발이 그 경계를 재구현하지 않게 한다.

## 1. 현재 판정

프로젝트는 단일 왼팔 prototype을 넘어 한 STM32에서 두 SO-ARM101의 12축을
동시에 실행하는 resident backend와 실제 Top-camera Pick/Place reference
application까지 통과했다.

### 완료된 기반

- NUCLEO-G474RE 한 대가 USART1/UART4의 두 servo bus를 제어한다.
- 좌우 6축 present position, torque-off, q0와 작업자 승인 운용 범위를 읽고
  firmware/host/model에 같은 manifest를 연결했다.
- shoulder의 4095→0 연속 이동은 unwrapped coordinate로 처리한다.
- 공통 5 ms executor가 12축 목표를 좌우 TX DMA에 paired dispatch한다.
- 양팔 current-pose hold와 base +0.03 rad 왕복에서 시작 시차 최대 2~6 us,
  launch lateness 최대 49 us 수준을 실측했다.
- 오른쪽 DMA fault와 tracking-error fault injection에서 다음 출력을 차단하고
  좌우 torque-off로 수렴했다.
- F8.1 measured-feedback snapshot, ROS 12축 feedback과 fresh anchor를 제공한다.
- F8.6은 position-read 단발 실패와 hard fault를 분리했다. in-motion read 실패만
  3회 연속일 때 정지하고 성공한 pair에서 streak를 복구한다.
- F8.7은 arm terminal `46,020 urad`, gripper contact terminal
  `90,000 urad`, 12회 연속 fresh measured pair 완료 조건을 적용한다.
- resident adapter의 no-motion, current-pose finite 2회 재사용과 명시적 STOP을
  F8.7에서 다시 통과했다.
- Top YOLO-OBB 픽셀 x로 왼팔/오른팔을 선택하는 reference app이 실제 왼팔
  Pick/Place를 연속 두 번 완주했다. 자동 재시도 0, 최종 epoch 7 HOLD였다.

최종 실기·artifact·SHA는
[F8.7 resident·Top 카메라 Pick/Place 수락 결과](test-results/2026-08-15-f87-resident-top-camera-pick-place.md)에
모았다.

## 2. 동결한 계층 경계

```text
Desktop / upper application
├─ camera/perception
├─ MoveIt planning and collision checking
├─ task FSM / pretrained policy / command arbiter
└─ complete 12-axis finite route or rolling batch
                  │ ROS service/message
                  ▼
Raspberry Pi resident adapter
├─ one serial/backend lease
├─ owner + arbiter epoch
├─ full-route validation
├─ finite route → 9-point/400-ms wire windows
├─ fresh/terminal 12-axis anchors
└─ READY / ACTIVE / STOPPED / FAULTED
                  │ protocol v2
                  ▼
STM32 F8.7
├─ operational limits and shoulder unwrap
├─ 5-ms executor
├─ paired USART1/UART4 DMA dispatch
├─ tracking and measured feedback
└─ heartbeat/watchdog/coordinated stop/torque-off
```

상단 애플리케이션은 firmware protocol을 직접 소유하거나 servo raw packet을 만들지
않는다. 규범 경계는
[양팔 상단 애플리케이션 인터페이스](BIMANUAL_UPPER_APPLICATION_INTERFACE.md)다.

## 3. 운영 불변식

1. 새 motion session은 `ready`, `owner=null`, `arbiter_epoch=0`,
   `motion_authorized=true`에서만 시작한다.
2. ARM 직전 `/refresh_anchor`로 torque-off measured 12축 anchor를 취득한다.
3. finite trajectory 전체를 ROS 요청 하나로 제출한다. 상단 앱이 wire window
   크기에 맞춰 APPEND로 쪼개지 않는다.
4. 한 팔 task도 반대 팔의 최신 hold target을 포함한 12축 명령이다.
5. firmware와 resident의 terminal 판정이 끝나기 전 성공으로 간주하지 않는다.
6. 성공 뒤 `ready`는 torque-on HOLD일 수 있다. 팔을 지지하지 않은 상태에서
   자동 torque-off하지 않는다.
7. STOP은 terminal이며 같은 process/session을 재사용하지 않는다.
8. status 2/3 startup shadow는 좌/우 verified torque-disable 실패다. 자동 반복하지
   않고 전원·bus·중복 backend를 확인한 뒤 감독 reset한다.
9. transport/dispatch/heartbeat/tracking/limit fault는 자동 재시도하지 않는다.
10. 상단 앱의 task 실패 판정이 안전한 finite 완료 뒤 발생했다면 operator가 팔을
    지지하고 STOP할 때까지 HOLD를 보존할 수 있다.

## 4. 완료로 보지 않는 범위

- 왼팔 Top-camera reference task 2회는 interface 수락 증거이지 50회 생산
  반복성 benchmark가 아니다.
- 오른팔 bus, feedback, limits, base 제한 왕복과 양팔 dispatch는 검증했지만
  오른팔 선택 camera Pick/Place의 place 높이와 접근 자세는 아직 별도 수락 전이다.
- wrist roll은 현재 reference task에서 q0 hold다. 물체 yaw에 따른 가장 가까운
  동치각 선택은 후속 기능이며 360도 강제 회전은 금지한다.
- 현재 grasp z offset과 gripper raw 값은 이 작업대/펜 reference task 값이다.
  범용 물체 정책 상수로 승격하지 않는다.
- pretrained policy bundle, Pi 실제 inference latency와 policy shadow/실기 gate는
  아직 미완료다.
- systemd 부팅, 8시간/24시간 soak와 현장 복구 runbook은 별도다.

## 5. 다음 우선순위

### A. PR과 interface 동결

1. protocol manifest, firmware core/board glue, ROS message/service와 resident node,
   승인된 operational-limit 파일을 한 PR의 일관된 변경으로 review한다.
2. 상단 앱 개발자는
   [인계 프롬프트](prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md)를
   사용하고 규범 문서/서비스 정의에서 자동 contract test를 만든다.
3. fresh clone/ROS overlay에서 전체 unit test와 Cortex-M4 Release build를 반복한다.

### B. 오른팔 task parity

1. 동일한 Top-camera routing에서 오른팔을 선택하는 plan-only 결과를 검토한다.
2. 오른팔 place 높이와 접근 자세를 1회 감독 검증한다.
3. 왼팔과 동일한 one-shot flow를 자동 재시도 없이 수행한다.
4. 좌우 각각 10회 pilot 뒤 사전 정의한 반복성 benchmark로 간다.

### C. pretrained policy 상단 앱

1. 데스크탑에서 학습된 model, normalization, joint order, `control_dt`와 SHA를
   deployment bundle로 동결한다.
2. Pi에서 perception/state 입력과 policy 출력을 기록하는 no-motion shadow를
   먼저 수행한다.
3. policy 출력은 MoveIt/collision/operational-limit/freshness supervisor를 지난
   bounded target만 resident interface에 제출한다.
4. deterministic camera Pick/Place reference보다 개선되는지 수치로 비교한다.

### D. 운영 신뢰성

1. camera + YOLO + MoveIt + policy + resident 30분, 이후 8시간 soak
2. serial lease, process crash, USB reconnect, STM32 reset과 status 2/3 startup
   fault의 운영 절차
3. 반복 부팅 STANDBY, journald/systemd, 물리 E-stop과 안전 종료 검증

## 6. 최종 증거 인덱스

- [F2 async host TX](test-results/2026-08-11-f2-async-host-tx-probe.md)
- [양팔 J0 작업자 desired envelope](test-results/2026-08-13-bimanual-j0-desired-envelope.md)
- [J1 unwrap shadow](test-results/2026-08-13-bimanual-j1w-unwrapped-shadow-candidate.md)
- [J1 operational limits](test-results/2026-08-13-bimanual-j1-operational-limit-candidate.md)
- [F7 paired DMA dispatch와 fault stop](test-results/2026-08-14-bimanual-dma-dispatch-f7.md)
- [F8 tracking feedback와 fault stop](test-results/2026-08-14-bimanual-tracking-feedback-f8.md)
- [F8.1 resident measured feedback](test-results/2026-08-14-bimanual-resident-feedback-f81.md)
- [F8.7 resident와 Top-camera Pick/Place](test-results/2026-08-15-f87-resident-top-camera-pick-place.md)
