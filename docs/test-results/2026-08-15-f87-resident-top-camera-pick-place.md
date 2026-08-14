# F8.7 resident 양팔 실행기와 Top 카메라 Pick/Place 수락 결과

- 날짜: 2026-08-15
- 최종 firmware: `0x00024807 / protocol 2 / 12 joints / 0xEFFFFFFF`
- 좌우 calibration hash: `0x2D90167E`
- HEX SHA-256:
  `9a9cd49247428478cae831d948977274d1188e9b0b0756d02de8c7c47fd431aa`
- 결론: **PASS — firmware와 상단 앱의 경계를 고정하고 애플리케이션 개발을
  계속할 수 있다.**

이 판정은 source-agnostic 12축 resident 실행 경로와 왼팔 Top-camera
Pick/Place reference consumer에 대한 것이다. 오른팔 task-level Pick/Place 반복성,
범용 pretrained-policy 앱과 장시간 운용 수락까지 완료했다는 뜻은 아니다.

## 최종 계약

- STM32는 공통 5 ms executor, 좌우 paired TX DMA, route-time tracking,
  heartbeat/watchdog와 coordinated stop을 소유한다.
- Pi resident adapter만 serial backend를 소유하고 ROS
  `/bimanual_stream_adapter/command`를 공개한다.
- 상단 앱은 완전한 finite trajectory를 한 번 제출한다. resident가 내부에서
  최대 9점/400 ms wire window로 나눠 공급하므로 상단 앱이 임의 APPEND로
  finite route를 재분할하지 않는다.
- finite 종료는 마지막 명령 전송이 아니라 12회 연속 fresh measured joint pair와
  완전한 12축 terminal snapshot으로 판정한다.
- arm terminal tolerance는 `46,020 urad`, gripper terminal tolerance는
  접촉 hold를 고려해 `90,000 urad`다. route-time tracking 한계는 완화하지 않았다.
- in-motion position read failure만 3회 연속일 때 정지한다. 성공한 pair는 streak를
  0으로 복구하고 누적 `failed_pairs`는 진단으로 남긴다. DMA, dispatch,
  heartbeat, unwrap, operational-limit와 tracking-error fault는 즉시 coordinated
  stop이다.
- 시작 직전 torque-off fresh anchor를 다시 취득한다. 새 session의 허용 상태는
  `ready`, `owner=null`, `arbiter_epoch=0`, `motion_authorized=true`다.
- 성공 뒤 `ready`는 torque-on HOLD다. 명시적 STOP 뒤의 `stopped` session은
  재사용하지 않고 resident 종료, STM32 reset, resident 재시작으로 새 session을
  만든다.

## 반복 장애의 근본 원인과 수정

### 1. 정상 trajectory 뒤 feedback 한 쌍 실패가 즉시 stop으로 번짐

초기 F8.x는 단발 position-read 실패도 route fault처럼 취급했다. 실제 arm
trajectory가 성공한 뒤 `failed_pairs=1`, 오래된 feedback age와 heartbeat latch로
나타났다. F8.6에서 position-read failure에만 연속 3회 조건을 적용하고 정상 pair가
streak를 지우게 했다. 안전에 직접 관련된 다른 fault의 즉시 정지는 유지했다.

### 2. terminal 완료와 물리 정착을 서로 다른 계층에서 중복 판정

초기 앱은 finite 완료 직후 첫 feedback 또는 firmware보다 좁은 임의 한계로 다시
판정해 정상 동작을 실패로 만들었다. 최종 구조에서는 firmware가 마지막 goal과
torque를 유지한 채 12회 연속 fresh pair를 확인하고, resident가 12축 freshness와
terminal 오차를 확인한 뒤 `ACTIVE -> READY`로 전이한다. 앱은 그 epoch의 measured
terminal anchor와 동일한 tolerance만 사용한다.

### 3. torque-off 뒤 처진 팔과 오래된 anchor를 계획 시작점으로 사용

READY의 주기 feedback은 정지할 수 있으므로 오래된 topic sample을 시작점으로 쓰면
첫 trajectory가 큰 tracking error로 중단될 수 있었다. `/refresh_anchor`를 추가해
ARM 직전 measured 12축 anchor를 갱신하고, 계획과 실제 시작점이 달라지면 stale
계획을 거부한다.

### 4. 검증된 연속 실행을 작은 leg로 다시 쪼개 정착/토크 공백을 늘림

reference 앱은 기존 왼팔 연속 Pick/Place 방식으로 돌아갔다. q0 복귀는 연속 finite
leg 하나, arm task는 세 연속 leg, 그리퍼 open/close/release만 별도 동작으로 둔다.
중간 MoveIt waypoint에서는 정착 판정을 하지 않는다. 자동 재시도는 0회이며 실패
시에는 가능한 경우 현재 pose HOLD를 보존한다.

## 최종 실기 gate

### Resident no-motion과 finite 재사용

| gate | 결과 | artifact SHA-256 |
|---|---|---|
| F8.7 no-motion | 12축, `present_mask=0xFFF`, 명령 거부, output 0 | `8fde19fc93c5d95b2098067501c083d1ccb9e1d6e85b9319807455e5316ccffa` |
| fresh-anchor no-motion | torque-off refresh, sample age 6 ms | `ff3c168d178b165b1dccebf62fa6bf663a4ca2ae7ebccb8e78331989f9cddb84` |
| current-pose hold twice | finite 2회, epoch 1/2, 명시적 STOP | `f565729c93909c23cadc631efd3c3d4ef26d9ef46babe3357545f9d9c71da400` |
| fresh-anchor hold twice | finite 2회, 각 `ACTIVE -> READY`, STOP | `019c84f95207c06cf2ff3c1727510145734fd76fd9f40b839a0423478fec82df` |

원본은 각각 다음 위치에 있다.

- `artifacts/resident_adapter/2026-08-15/no_motion_24807_run01.json`
- `artifacts/resident_adapter/2026-08-15/no_motion_fresh_anchor_24807_run01.json`
- `artifacts/resident_adapter/2026-08-15/current_pose_hold_twice_24807_run01.json`
- `artifacts/resident_adapter/2026-08-15/current_pose_hold_twice_fresh_anchor_24807_run01.json`

### Top-camera 왼팔 Pick/Place 2회

두 실행 모두 카메라 검출 픽셀로 왼팔을 선택했고, fresh anchor, 양팔 연속 q0,
6개 task action, 최종 epoch 7 armed READY/HOLD를 통과했다. 자동 재시도와
coordinated STOP은 발생하지 않았다.

| run | pixel x/width | q0 또는 arm 최대 terminal error | 결과 | artifact SHA-256 |
|---|---:|---:|---|---|
| run20 | 225.8/640 | 35.282 mrad | PASS/HOLD | `67d2d1de5035c937c670a5f23ed0447392479ec81145c607a00ec4ca41aebd1a` |
| run22 | 246.1/640 | 21.476 mrad | PASS/HOLD | `c887c8c723a5b870841cd404ab7673040f7dd0e26c58994ea068c45d0f1edd4c` |

원본은 `artifacts/top_pick_place/2026-08-15/application_run20.json`과
`application_run22.json`이다. 두 실행의 terminal error는 공통 arm 계약
46.020 mrad 안이다. 작업 확인 뒤 작업자가 명시적으로 STOP했고 torque-off를
확인했다.

## 로컬 회귀

- ROS 2 Jazzy와 workspace overlay를 source한 전체 Python suite:
  `1416 passed`
- 상단 앱 계약 표적 suite: `7 passed`
- 마지막 resident/app 관련 표적 suite: `60 passed`
- F8.7 Cortex-M4 image build, flash verify와 reset 통과
- 최종 no-motion/hold-twice gate 뒤 동일 firmware에서 두 번의 실제
  perception-to-motion Pick/Place 통과

## 남은 수락 범위

1. 오른팔 선택 task의 place 높이·접근 자세와 실제 Pick/Place 반복성
2. 좌우 각각 10회 pilot, 이후 사전 정의한 반복성 benchmark
3. generic upper FSM/MoveIt/pretrained-policy adapter의 shadow와 제한 실기
4. 3-camera + perception + MoveIt + policy + resident의 30분/8시간 자원·링크 soak
5. 부팅, reset, serial lease, status 2/3 startup failure를 포함한 운영 runbook

상단 개발자는
[`BIMANUAL_UPPER_APPLICATION_INTERFACE.md`](../BIMANUAL_UPPER_APPLICATION_INTERFACE.md)를
규범 계약으로 사용하고,
[`BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md`](../prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md)를
구현 인계 입력으로 사용한다.
