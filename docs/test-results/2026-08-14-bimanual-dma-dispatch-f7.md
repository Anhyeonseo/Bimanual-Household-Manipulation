# 2026-08-14 F7 양팔 DMA dispatch 후보

## 구현 결과

`0x00024604`은 source-agnostic protocol-v2 절대 12축 stream을 실제 좌우
서보 버스로 내보내는 최초 후보다.

- 하나의 5 ms TIM6 event에서 executor를 정확히 한 번 step한다.
- 12축 µrad 전체를 먼저 검증·변환한 뒤 좌/우 6축 raw를 원자적으로 확정한다.
- USART1(왼팔)과 UART4(오른팔)에 각각 26-byte STS3215 SYNC WRITE 패킷을
  TX DMA로 시작한다.
- 이전 쌍이 완료되지 않았거나 tick event가 누락되면 새 출력을 보내지 않고
  양팔 coordinated stop을 요청한다.
- 한쪽 DMA 시작 실패, UART error, heartbeat timeout은 양 버스 전송을 중단하고
  양팔 torque-off를 요청하며 stop latch를 남긴다.
- 정상 operator SAFE_STOP은 idle dispatch를 failure로 기록하지 않되 양팔
  torque-off와 stop latch는 동일하게 수행한다.
- dispatch diagnostics는 launch/completion/failure, 최대 시작 시차, 최대 launch
  lateness, 마지막 tick과 양쪽 시작 timestamp를 protocol-v2로 반환한다.

`0x00024600`은 ARM setup UART 복구를 비활성 dispatch fault로 오인했고,
`0x00024601`은 12축 순차 ARM 설정 중 500 ms heartbeat가 만료됐다. 둘 다
`launch_count=0`에서 정지했다. `0x00024602`는 이 두 문제를 고쳐 첫 DMA 쌍을
`start_skew=1 us`, `launch_lateness=42 us`로 기동했지만 UART4의 최종 TX-complete
IRQ가 없어 `launch_count=1`, `completed_count=0`에서 fail-closed했다.
`0x00024603`은 UART4 TC IRQ를 연결해 `launch_count=14`, `completed_count=14`를
달성했지만, executor를 임의의 `HAL_GetTick()=22681 ms`에서 시작한 반면 실제
TIM6 event는 5 ms의 다른 위상(`...22750, 22755...`)에 있었다. 첫 sample
`22753 ms`를 executor grid는 `22756 ms`에 적용할 예정이라고 판단하면서 실제
`22755 ms`에는 보간 span을 이미 2 ms 넘겨 `INVALID_TIMELINE`으로 정지했다.
`0x00024604`는 executor start를 첫 실제 TIM6 event까지 보류해 control epoch와
step grid를 하나로 만든다. heartbeat 부재/timeout과 실제 DMA 미완료는 계속
양팔 torque-off/stop latch로 수렴한다.

현재 raw의 wrap branch는 외부 테스트 artifact에 의존하지 않는다. 양팔 torque를
먼저 끄고 12축을 읽은 뒤 승인된
`config/bimanual_operational_limits.json` 안에서 유일한 unwrapped 좌표를 자동으로
선택한다. 범위 밖이거나 branch가 유일하지 않으면 arm 전에 거부한다.

## 후보 identity

- firmware: `0x00024604`
- protocol: `2`
- joints: `12`
- host baud: `921600`
- capabilities: `0xEFFFFFFF`
- calibration: left/right `0x2D90167E`

- HEX: `build/stm32_g474_single_arm-bimanual-dma-dispatch-release/stm32_g474_single_arm.hex`
- SHA256: `afc9a9afcd5175c1e32fadb578c0c7035e090de10a378de7bc02de9b9fc4e88f`

## 로컬 검증

- Cortex-M4 Release build: PASS
- actuator-core C: `9/9 PASS`
- F7/protocol-v2 관련 Python: `58 PASS`
- 과거 바이너리 재현:
  - R4 `0x00023B00`: `b677bd97a7097ae8bb3be07ad5d5696dee6df803f33aa4ffe89282ad792acd8a`
  - limits `0x00024400`: `3638b94318342e531eca48713692e57444b02da70ab76f8cfff4fe3dea843990`
  - F7-A `0x00024500`: `ee91b714267a9eda4a8b73332d89c2ba27459750ba0f4043ca9f3d6d4c444d95`

## fault-injection 후보

- firmware: `0x00024605`
- trigger: 정상 paired DMA 8회 완료 뒤 오른쪽 DMA launch rejection 1회
- commanded motion delta: `0`
- 합격 조건: dispatch failure `+1`, stop latch, 좌우 torque-enable register 전체 `0` readback
- HEX SHA256: `dfd2f4b5f728a67ed76ce0cb36bcb0614d2ff1f622400ee44a7c269d40420aed`
- tool SHA256: `4ac7d1a468ffd30ea355dd6b993524c4dcfb5fddc87ea03248d81b33a00165b6`
- 정상 `0x00024604` HEX SHA256는 계속
  `afc9a9afcd5175c1e32fadb578c0c7035e090de10a378de7bc02de9b9fc4e88f`

## fault-injection 실기 결과

`0x00024605` one-shot 후보가 `launches_before_fault=8`,
`completed_before_fault=8`, `failure_delta=1`을 기록했고, 좌우 12개 서보의
torque-enable register가 모두 `0`으로 readback되었다. artifact SHA256은
`55e617941ac75754f9ea44723fa8fa3772500c2e889e7c7e29b1ec693fbfce8e`다.
시험 후 정상 `0x00024604`로 즉시 복구했다. 복구 뒤 no-output 재검증도 `launches=0`으로 통과했으며 artifact SHA256은 `8f135f518095c600c3f056f46a590e26b961be4619ea7e9a8008e2bfcc7dd18e`다.

## 물리 gate

`0x00024604`에서 정상 경로 실기 gate 세 개가 통과했다.

- no-output: `launches=0`, artifact SHA256
  `e24455fb9096e151686159845abefc8c98910e6920dbc7fbe181c6f452774f61`
- current-pose hold: `launches=36`, `completed=36`, 최대 좌우 시작 시차
  `2 us`, 최대 launch lateness `46 us`, 종료 torque-off 확인, artifact SHA256
  `a3cf28c3e209b8fcaa0a64bc8f90b690e2faeed163ac3ac8384cd617ce3a6ae4`
- base small roundtrip: `+0.03 rad` 동일 부호 왕복, `launches=71`,
  `completed=71`, 최대 좌우 시작 시차 `2 us`, 최대 launch lateness `49 us`,
  종료 torque-off 확인, artifact SHA256
  `1364a5a024039c83c78f6153cca37eabac486da81166d5475993d1ef90e68c45`

남은 순서는 다음과 같다.

1. 정상 `0x00024604` 복구 및 identity/no-output smoke
2. route 중 실측 tracking feedback와 tracking-error coordinated stop 연결

따라서 F7 전체 gate는 아직 부분 통과다. 현재 후보의 fail-closed 범위는
tick, DMA/UART, heartbeat, operational-limit이며 실시간 양팔 tracking feedback은
후속 항목으로 남는다.
