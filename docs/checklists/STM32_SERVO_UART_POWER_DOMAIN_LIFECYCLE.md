# 0x00022500 servo UART 전원 도메인 수명주기 후보

## 목적

Motion-11 세 번째 물리 시도가 `PLAN_GATE=PASS` 직후
`TimeoutError: joint state timed out`으로 죽었다. 경로나 queue 문제가 아니라
servo UART가 응답을 전혀 반환하지 못한 것이다.

외부 servo adapter는 스위치드 12 V 도메인에 있다. 그 도메인이 꺼진 채 DMA를
armed 상태로 두면 전원 엣지가 부분 응답으로 캡처되어 MCU reset 전까지 USART
receiver를 오염시킨다. 또한 PC5(USART1_RX)가 floating이라 첫 요청 전에
FE/ORE가 발생했다.

이 후보는 servo RX를 blocking `HAL_UARTEx_ReceiveToIdle` burst에서
**transaction 범위의 lazy-arm circular DMA ring**으로 바꾸고, 전원 도메인 엣지를
격리하며, schema v2 진단으로 실패 증거를 남긴다. `motion_authorized=false`를
유지한다.

## 고정 계약

- firmware identity: `0x00022500` (이전 배포 `0x00022100`)
- capabilities: `0x00000FFF` — 신규 bit 없음
- calibration hash: `0x8AD27897`
- servo baud: `1000000`, RX FIFO 비활성
- receive API: `HAL_UARTEx_ReceiveToIdle_DMA`, `DMA_CIRCULAR`, ring `256 B`
- DMA start site: 저장소 전체에서 정확히 1곳 (`ServoBus_StartCircularDma`)
- 부팅 시 armed: `false` — 첫 transaction 전까지 unarmed
- lifecycle: `transaction_scoped_lazy_arm`, 성공 시 disarm
- RX idle bias: internal pull-up (PC5 `GPIO_PULLUP`)
- idle-high 안정: `2 ms`, idle-high timeout: `20 ms`
- receiver hard resync: `USART_CR1_RE` disable/enable + `REACK` 대기
- HAL error IRQ abort: 비활성 (`ServoBus_DisableHalErrorAbort`)
- DMA active gate 4중: software started / `USART_CR3_DMAR` /
  `DMA_CCR_EN` / `RxState == BUSY_RX`
- transaction window: 최대 `64 B`, timeout `50 ms`
- soft error `PE`/`NE`: checksum-gated resynchronize
- hard error `FE`/`ORE`/`RTO`/`DMA`: fail-closed + receiver resync
- 복구 순서: **snapshot 보존 → abort → RE toggle → unarmed 유지**
- failure snapshot: `16 B`, first-fault latched
- 진단 schema version: `2`, DIAGNOSTICS payload `138 B`
- STATE_FEEDBACK position-read-failure payload: `58 B`
- IRQ 우선순위: LPUART1(host heartbeat) `1` > USART1 / DMA1_Channel1 `2`

## Fail-closed 조건

- identity가 `0x00022500`이 아니면 host가 ARM 전에 연결 거부
- schema v2 bus health를 못 얻으면 soak 즉시 실패
- 진단 뒤 receiver가 armed로 남아 있으면 실패
- 성공 진단이 failure snapshot을 보유하면 실패
- cold-start에서 `recovery_count > 1`이면 실패
- `fe_count != recovery_count` 또는 `receiver_resync_count != recovery_count`면 실패
- 12개 오류 counter 중 하나라도 soak 중 증가하면 실패
- 자동 host 재시도 `0회`, motion command `0회`
- soak 실패 시 `deployed`를 올리지 않고 Motion-11도 실행하지 않는다

## 완료 확인

### 데스크탑 (완료)

- [x] 순수 policy 모듈 `servo_rx_window.{h,c}` 분리와 native C 시험 9 시나리오
- [x] transaction 범위 lazy-arm / disarm 구현
- [x] 전원 도메인 엣지 격리와 PC5 pull-up
- [x] HAL error IRQ abort 비활성과 4중 DMA active gate
- [x] snapshot-before-abort 복구 순서
- [x] schema v2 bus health 20 counter와 first-fault latch
- [x] host parser 하위 호환 (구/신 payload 길이 모두 수용)
- [x] 계약 JSON과 validator exact-match dict 동시 갱신
- [x] source-contract 시험 13개
- [x] buffered hot path 불변식 시험 2개 신규
      (`Servo_SyncWritePositions` TX-only, 모든 servo read가
      buffered-execution 게이트 뒤)
- [x] Cortex-M4 Release clean cross-build, warning 0
- [x] **재빌드 HEX가 기존 0x225 artifact와 byte-identical**
- [x] `firmware/stm32_actuator` C 시험 `2/2 passed`
- [x] 6축 DISABLE sweep 예산 재유도 — transaction 준비 `22 ms × 12`를 포함해
      firmware worst case `1223 ms`, host timeout `2500 ms`, 여유 `1277 ms`.
      host timeout 변경 불필요
- [x] 전체 host/ROS 회귀 `607 passed`

### 물리 (2026-08-06 통과)

- [x] HEX Pi 배포와 flash — 2026-08-05 세션에서 완료. backup
      `stm32_before_0x00022500_20260805-044258.bin` 보존됨. 이번 세션은
      재플래시 없이 검증만 수행
- [x] 커밋된 소스의 clean cross-build가 배포된 HEX와 byte-identical
- [x] identity gate `0x00022500 / 0x00000FFF / 0x8AD27897`, protocol 1, joints 6
- [x] MCU reset 후 cold start — counter 13개 전부 0, `schema_version = 2`
- [x] **전원 도메인 엣지** — MCU 유지한 채 12 V OFF→ON.
      `recovery = fe = resync = 1`, `failure_count = 0`, 첫 read 성공.
      `EDGE_VERDICT=BOUNDED`
- [x] 300 s READ_ONLY soak `PASSED=1`, 12개 counter delta 0,
      snapshot 5개 전부 `receiver_armed = False`
- [x] `lazy_arm_count == transaction_count` 불변식 실측 확인
- [x] artifact 3개를 `docs/test-results/evidence/`로 보존
- [x] 계약과 validator에서 `deployed: true` 동시 전환
      (`motion_authorized=false` 유지)
- [x] Motion-11 계획 재생성 — 새 plan SHA
      `630a2873057699f6f93cd98d86c13b52c1d97edbb83c2345041e20ef1e7ce8c7`
- [x] 전체 회귀 `608 passed`
- [x] `VERIFICATION_MATRIX` `MCU-005` 행 추가

`artifacts/`는 gitignore 대상이므로 보존이 필요한 증거는
`docs/test-results/evidence/`로 복사한다.

세부 결과는
[빌드·회귀 결과](../test-results/2026-08-05-stm32-0x00022500-servo-uart-lifecycle-build.md)와
[물리 검증 결과](../test-results/2026-08-06-stm32-0x00022500-servo-uart-power-domain-lifecycle.md)에
기록한다. 선행 servo UART 복구 후보는
[0x00021800 결과](../test-results/2026-07-31-stm32-0x00021800-servo-uart-recovery-candidate.md)에
있다.
