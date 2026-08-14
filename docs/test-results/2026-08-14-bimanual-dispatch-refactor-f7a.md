# 2026-08-14 F7-A 양팔 목표 매핑·패킷 리팩터링

## 결과

운용 범위 승인이 끝난 뒤 실제 UART 출력을 연결하기 전의 순수 리팩터링을
완료했다.

- protocol-v2 executor의 절대 12축 µrad 목표를 좌/우 6축 modulo raw 배열로
  원자 변환하는 HAL 독립 `bimanual_goal_map`을 추가했다.
- Shoulder는 unwrapped raw에서 한계를 검사하고 마지막 단계에서만 modulo 4096을
  적용한다.
- 12축 중 하나라도 범위 또는 설정 검사를 실패하면 좌/우 목적 배열을 모두
  변경하지 않는다.
- 기존 왼팔 SYNC WRITE 패킷 조립을 HAL 독립 `sts3215_packet`으로 추출했다.
  실제 UART 호출, torque, PID, heartbeat, stop semantics는 바꾸지 않았다.
- 일반 12축 executor 출력은 여전히 어느 UART에도 연결되지 않았다.

## 바이너리 분리

새 리팩터링 후보는 기존 증거와 섞이지 않도록 별도 identity를 사용한다.

- firmware: `0x00024500`
- capabilities: `0x607FFFFF`
- HEX:
  `build/stm32_g474_single_arm-bimanual-dispatch-refactor-release/stm32_g474_single_arm.hex`
- SHA256:
  `ee91b714267a9eda4a8b73332d89c2ba27459750ba0f4043ca9f3d6d4c444d95`

기존 기준선은 재빌드해 SHA가 그대로임을 확인했다.

- `0x00024400`:
  `3638b94318342e531eca48713692e57444b02da70ab76f8cfff4fe3dea843990`
- R4 `0x00023B00`:
  `b677bd97a7097ae8bb3be07ad5d5696dee6df803f33aa4ffe89282ad792acd8a`

## 로컬 검증

- actuator-core C: `8/8 PASS`
- 관련 Python: `24/24 PASS`
- 전체 Python: `1312/1312 PASS`
- `single_arm_bridge` colcon build: PASS
- Cortex-M4 Release build: PASS
- `git diff --check`: PASS

## 실기 gate

이 단계의 실기 목적은 새 범위를 다시 승인하는 것이 아니다. 공통 패킷
인코더가 기존 왼팔 동작을 바꾸지 않았고, 양팔 read-only feedback도 유지되는지만
확인한다.

1. 백업 후 `0x00024500` flash/verify/reset
2. heartbeat/identity 확인
3. 기존 왼팔 F0 0.03 rad 왕복 probe 1회
4. 양팔 read-only 100 sample soak
5. 실패 시 기존 `0x00024400` 복구

통과 뒤 다음 구현은 USART1/UART4 TX DMA 인스턴스와 하나의 control tick에서
두 패킷을 기동하는 실제 F7 dispatch, 그리고 한 팔 fault의 coordinated stop이다.
