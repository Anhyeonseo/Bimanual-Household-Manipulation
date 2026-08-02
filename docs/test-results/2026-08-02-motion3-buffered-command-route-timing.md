# Motion-3 buffered command route·timing host-only 결과

## 결론

G474에 링크되지만 현재 runtime에서 호출되지 않는 buffered command route 후보,
16/32바이트 status 호환 codec과 offline timing 분석기를 구현했다. 모든 검증은
로컬 host에서 수행했으며 Pi, serial, STM32 reset/flash와 로봇 이동은 0회다.

## 검증 결과

- Python 전체 회귀: `455 passed`
- STM32 공통 C: ctest `2/2 passed`
- ASan/UBSan: ctest `2/2 passed`, 오류 0
- Cortex-M4 Release cross-build: PASS, text/data/bss `31560/112/4176`
- 생성 HEX SHA-256: `4b9ca7c7b3927ce798048258fb1b3deecfb0718d660c6c1bd93308862ef3f317`
- ROS local rebuild와 installed contract/codec smoke: PASS
- timing artifact SHA-256: `1db9ebcb89350209a2174c749607632f61f1370d767f0402d6205e60dc95695e`

## Fail-closed 확인

- firmware identity `0x00021800`, capability `0x000003FF` 유지
- `binary_control.c`의 `sample_count == 1U` 유지
- binary candidate route·Action adapter·motion authority: `false`
- synthetic measurement와 operational timing authorization: `false`
- lead/prime/watermark/refill 운영값: 전부 `null`

이 결과는 physical buffered 실행 승인이 아니다. 다음 이슈에서 firmware route,
identity와 capability를 연결하고 Pi–VCP 실제 timing을 수집해야 한다.
