# Motion-4 G474 buffered validation route 로컬 결과

## 결론

G474 binary dispatcher에 multi-sample candidate의 validation-only route를
연결했다. firmware identity는 `0x00021900`, capabilities는
`0x000007FF`이며 bit 10은 물리 실행이 아닌 validation route만 나타낸다.

이번 검증에서 Pi 전송, serial 접근, STM32 flash/reset, 12 V 제어와 로봇
이동은 모두 0회다.

## 검증 결과

- 전체 Python 회귀: `462 passed`
- STM32 공통 C: ctest `2/2 passed`
- ASan/UBSan: ctest `2/2 passed`, 오류 0
- Cortex-M4 Release cross-build: PASS
- ELF text/data/bss: `33400/112/4768`
- ROS `single_arm_bridge` 로컬 rebuild: PASS
- installed identity/contract smoke: PASS
- 생성 HEX SHA-256:
  `ffcc689f82abed08038e9b293b90ae82dceb2a235d54b5d8d498bdeef4cc1ed5`

HEX:
`artifacts/firmware/2026-08-02/stm32_g474_single_arm_0x00021900.hex`

## Fail-closed 확인

- `VALIDATION_ONLY` 없는 candidate는 거부한다.
- validation-only candidate는 servo 출력이 비활성화된 `SAFE_DISABLED/READ_ONLY`에서 무동작 timing 측정에 사용할 수 있다.
- stop latch, `FAULT`, `ESTOPPED`, 진행 중 motion에서는 계속 거부한다.
- candidate handler는 legacy motion start와 servo sync-write를 호출하지 않는다.
- validation 성공 뒤 queue와 diagnostics를 원래 상태로 복원한다.
- 확장 응답의 queued/accepted/applied sample은 모두 0이다.
- route 초기화 실패 시 HELLO에서 bit 10을 제거한다.
- 새 host는 `0x00021800` 또는 bit 10 없는 firmware를 거부한다.
- 기존 flag 0, single-sample 실행 경로는 분리되어 유지된다.
- contract는 `BOARD_VALIDATION_ONLY`, `motion_authorized=false`다.

## 남은 gate

1. 사용자 승인 아래 Pi host/HEX 전송과 기존 flash 전체 backup
2. OpenOCD program/verify/reset 1회와 identity/capability 확인
3. 로봇 무동작 validation route로 Pi–VCP timing 1000회 이상 수집
4. timing 운영값 검토
5. 별도 이슈에서 물리 buffered execution과 ROS Action 연결
