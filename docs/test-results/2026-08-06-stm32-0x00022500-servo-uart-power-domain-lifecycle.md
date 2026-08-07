# 0x00022500 servo UART 전원 도메인 수명주기 물리 검증

## 결론

**전원 도메인 엣지 가설이 확인됐다.** MCU를 켜둔 채 스위치드 12 V 서보 도메인을
OFF→ON 했을 때 receiver 오염이 발생하지 않았고, framing error 하나를 첫
transaction에서 bounded하게 정리한 뒤 6축 위치 읽기가 곧바로 성공했다.

세 번의 READ_ONLY 시험을 모두 통과했다. 계약의
`servo_uart_receive_candidate`를 `deployed: true`로 전환하며
`motion_authorized=false`는 유지한다. Motion-11 실행 차단이 해제된다.

이 문서에는 motion 명령이 없다. ARM/ENABLE/SETPOINT를 호출하지 않았고
`MOTION_COMMAND_COUNT=0`, `AUTOMATIC_HOST_RETRY_COUNT=0`이다.

## 배경

Motion-11 세 번째 물리 시도(2026-08-05 01:38)는 `PLAN_GATE=PASS` 직후
`TimeoutError: joint state timed out`으로 종료됐다. 경로나 queue가 아니라
servo UART가 응답을 전혀 반환하지 못한 것이다.

가설: 외부 servo adapter가 스위치드 12 V 도메인에 있어, 그 도메인이 꺼진 채
DMA를 armed로 두면 전원 엣지가 부분 응답으로 캡처되어 MCU reset 전까지
USART receiver를 오염시킨다. 추가로 PC5(USART1_RX)가 floating이라 첫 요청
전에 FE/ORE가 발생했다.

수정: blocking burst 수신을 **transaction 범위 lazy-arm circular DMA**로 교체.
부팅·유휴 시 arm하지 않으므로 엣지를 캡처할 수 없고, 첫 transaction은
idle-high 안정 확인 뒤에만 arm한다.

## 검증된 상태

- firmware `0x00022500`, protocol `1`, joints `6`
- capabilities `0x00000FFF` — 신규 bit 없음
- calibration `0x8AD27897`
- 이전 배포 `0x00022100`
- ST-LINK: `usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02`
- 서보 전압 `12.2 ~ 12.5 V`, 온도 `27 ~ 30 °C`, torque 6축 전부 OFF

플래시는 2026-08-05 세션에서 이미 수행됐고
(`/home/pi/firmware_updates/backup/stm32_before_0x00022500_20260805-044258.bin`),
이번 세션은 재플래시 없이 물리 검증만 수행했다. 데스크탑에서 커밋된 소스로
clean cross-build한 HEX가 배포된 artifact와 byte-identical임을 확인했다
(`aae044c0256d5f634c029cd5e6221b00413bd5ba5fe523bee20e978aeeb2f091`,
[빌드 결과](2026-08-05-stm32-0x00022500-servo-uart-lifecycle-build.md)).

## 시험 1 — MCU reset 후 cold start

12 V를 켠 상태에서 `openocd ... -c "reset run"`으로 MCU를 재시작했다.
직전까지 걸려 있던 STOP latch는 12 V가 꺼진 동안의 position read 실패 3회로
설정된 것이며(`HOST_POSITION_READ_FAILURE_LIMIT = 3`,
`binary_control.c:350-366`) 설계된 fail-closed 동작이다.

reset 직후 상태:

- `LATCHED=False`
- 6축 위치 `2058 / 2180 / 1732 / 1839 / 2142 / 2002`
- bus health `schema_version = 2`, `dma_started = False`
- `transaction_count = 42`, `lazy_arm_count = 42`
- **오류 counter 13개 전부 `0`**

70초 soak: heartbeat `700`, position `350`, diagnostics `2`.
12개 counter 전부 delta `0`. 두 snapshot 모두 `receiver_armed = False`.
`lazy_arm` `108 → 1920`.

`PASSED=1`. artifact
[`2026-08-06-servo-uart-dma-coldstart-0x225.json`](evidence/2026-08-06-servo-uart-dma-coldstart-0x225.json),
SHA-256 `f7901ffb4b9fca1dad356037c6685545cbef9814e00c56dd081e8e652210360d`.

## 시험 2 — 전원 도메인 엣지 (핵심 가설 검증)

MCU를 **리셋하지 않은 채** 12 V를 OFF → 약 5초 대기 → ON 했다. 엣지 구간에는
어떤 폴링도 하지 않았다. 폴링하면 read 실패 3회로 latch가 걸려 엣지와 무관한
오염이 생기기 때문이다.

엣지 이후 최초의 servo transaction:

- `FIRST_READ=OK`, 위치 `2059 / 2180 / 1733 / 1840 / 2142 / 2002`
- `recovery_count = 1`, `fe_count = 1`, `receiver_resync_count = 1`
- **`failure_count = 0`** — 읽기 자체는 한 번도 실패하지 않았다
- `discarded_bytes / timeout / overflow / pe / ne / ore / rto / dma_error` 전부 `0`
- `uart_error_code = 0x00000000`, `uart_isr = 0x00000000`
- `transaction_count = 2256 == lazy_arm_count = 2256`
- `dma_started = False`

판정 `EDGE_VERDICT=BOUNDED`.

즉 전원 엣지가 framing error를 **정확히 하나** 남겼고, transaction 범위
수명주기가 이를 감지해 receiver를 **한 번** hard resync한 뒤 읽기가 성공했다.
이전 실패 모드에서는 같은 시나리오가 MCU reset 전까지 버스를 완전히 죽였다.

이어진 70초 soak에서 counter가 `1 / 1 / 1`에 고정된 채 전혀 자라지 않았다.
`lazy_arm` `2322 → 4134`, 양쪽 snapshot `receiver_armed = False`.

`PASSED=1`. artifact
[`2026-08-06-servo-uart-dma-power-edge-0x225.json`](evidence/2026-08-06-servo-uart-dma-power-edge-0x225.json),
SHA-256 `1ef2d2c34e98026bbb09b902dd17c55160c9b324067f5e40f5c7ee536318d17e`.

## 시험 3 — 300초 배포 게이트 soak

엣지를 이미 겪은 상태에서 이어 수행했다. 지속 시험으로는 더 강한 조건이다.

- 관측 시간 `300.0 s`
- heartbeat `3000`, position `1500`, diagnostics `5`
- baseline `recovery = 1 / fe = 1 / resync = 1`에서 **300초 내내 delta `0`**
- snapshot 5개 전부 `receiver_armed = False`
- `lazy_arm` `4494 → 11814` (7320회 arm, 전부 disarm)

`PASSED=1`, `AUTOMATIC_HOST_RETRY_COUNT=0`, `MOTION_COMMAND_COUNT=0`. artifact
[`2026-08-06-servo-uart-dma-soak300-0x225.json`](evidence/2026-08-06-servo-uart-dma-soak300-0x225.json),
SHA-256 `51a5dd82e69b2ab240a741236875d0e5cddb971239758032d04b4fada3bd26e9`.

## latency

세 시험에서 일관됐다. 단위 ms.

| 호출 | n | 평균 | p95 | 최대 |
|---|---:|---:|---:|---:|
| heartbeat | 3000 | 6.062 | 6.134 | 6.186 |
| position (6축) | 1500 | 84.403 | 84.560 | 85.325 |
| diagnostics (6축) | 5 | 209.219 | 209.294 | 209.294 |

position 84 ms는 축당 약 14 ms이며 `/joint_states` 5 Hz 주기 `200 ms` 안에
들어간다. buffered 실행 중에는 servo read 자체가 거부되므로
(`test_servo_reads_are_refused_while_buffered_execution_is_active`)
1 ms executor와 5 ms apply-lateness 예산에 영향을 주지 않는다.

## 관찰: lazy-arm 불변식

세 시험 모두에서 `lazy_arm_count == transaction_count`이고 모든 snapshot에서
`receiver_armed = False`였다. transaction마다 정확히 한 번 arm하고 끝나면
disarm한다는 뜻이며, 유휴 상태에서 DMA가 armed로 남지 않는다는 것이
설계가 아니라 실측으로 확인됐다. 이것이 전원 엣지 면역의 직접적 근거다.

## 계약 전환

`servo_uart_receive_candidate`를 계약 JSON과 validator exact-match dict에서
동시에 전환했다.

- `status`: `LOCAL_SERVO_UART_POWER_DOMAIN_LIFECYCLE_CANDIDATE`
  → `LOCAL_SERVO_UART_POWER_DOMAIN_LIFECYCLE_DEPLOYED`
- `deployed`: `false` → `true`
- `motion_authorized`: `false` **유지**

계약 SHA-256이
`1243f351f07bf32b092c61a77028ca778f3c7f0db9ce760ffa17c85ef7d79807` →
`59362c33adbc2d86cb996755f98a22aa29ea04699ee9d83d135676f57ff18c4e`로 바뀌었다.

계약은 Motion-11 계획 artifact에 임베드되므로 계획을 재생성했다. 새 plan
SHA-256은
`630a2873057699f6f93cd98d86c13b52c1d97edbb83c2345041e20ef1e7ce8c7`이며
duration `47000 ms` / `2351 samples`는 변하지 않았다. **A2 실기는 반드시 이
새 SHA를 사용해야 한다.**

부수 관찰: validator의 exact-match dict가 이제 `deployed: True`를 요구하므로,
undeployed 계약은 실행기의 `require_deployed` 게이트에 도달하기 전에
validator에서 먼저 거부된다. 두 층 모두 fail-closed이며 시험은 실제로
발동하는 바깥층을 검증하도록 고쳤다.

## 로컬 회귀

```
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m pytest -q
```
**`608 passed`** (전환 전 607 + deployed 수락 시험 1)

## 다음 gate

1. Pi의 `tools/actuator_protocol.py`(V2 struct 없는 구버전)와
   `buffered_trajectory.py`를 데스크탑과 동기화한다. soak에는 쓰이지 않았지만
   Motion-11 sender에는 필요하다.
2. 새 계약·계획을 Pi에 배포하고 SHA를 대조한다.
3. fresh anchor로 계획을 재생성하고 새 plan SHA를 기록한다.
4. START 전 precompute 시간과 잔여 lead를 로그로 남긴다
   (2차 시도는 precompute `263.804 ms`가 `140 ms` lead를 `79 ms`로 깎아
   `80 ms` 하한에 걸려 죽었다. 현재 lead는 `160 ms`).
5. Motion-11 Action을 단 1회 실행하고 자동 재시도를 금지한다.
6. startup accounting, firmware terminal, post-settle `≤30 raw`, 최종 pregrasp,
   physical DISABLE을 모두 통과해야 Motion-11을 물리 통과로 승격한다.
