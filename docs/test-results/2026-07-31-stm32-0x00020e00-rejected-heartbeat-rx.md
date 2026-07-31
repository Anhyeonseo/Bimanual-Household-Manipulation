# STM32 0x00020E00 물리 수락 거절 — heartbeat RX starvation

## 판정

`0x00020E00`은 diagnostics 기능과 torque 설정은 검증했지만 첫 Shoulder 이동에서
안전 상태 `status=8`과 stop latch가 재현되어 **거절**한다. 토크 부족을 해결하려고
토크/P gain/허용 오차를 더 올리지 않는다.

## 재현 조건과 결과

- firmware `0x00020E00`, calibration `0x4D62F8D5`, capabilities `0x0000001F`
- READ_ONLY diagnostics: 6축 voltage `12.3..12.5 V`, temperature `31..33 °C`
- MOTION_ENABLED diagnostics:
  - Shoulder torque enabled, P/D/I `16/32/0`, torque limit `780`
  - Elbow torque enabled, P/D/I `24/32/0`, torque limit `650`
- 승인 명령: Shoulder 현재값에서 `-0.08 rad`, duration `2 s`, 단 1회
- 결과: `ARM_EXECUTION_TERMINAL ... status=8 detail=0`, 이어서 STM32 stop latch
- 실제 Shoulder feedback은 목표 `1.696350 rad` 대비 `1.805495 rad`
- 즉시 bridge 종료, 12V OFF, 팔 안전 확보

사후 상태 snapshot:

```text
STOP_LATCHED=1
SAFETY_STATUS=0
HEARTBEAT_COUNT=72
REJECTED_FRAMES=11
MCU_TICK_MS=254586
LAST_HEARTBEAT_MS=19276
HEARTBEAT_AGE_MS=235310
```

`HEARTBEAT_AGE_MS`는 bridge 종료 뒤 읽었으므로 failure 순간의 정확한 age가 아니다.
`REJECTED_FRAMES`도 누적값일 수 있다. 직접 증거는 motion terminal `status=8`과
latched state이며, snapshot은 사후 상태 보조 증거로만 사용한다.

## 근본 원인

코드 감사에서 binary main loop 순서가 다음과 같았다.

1. `BinaryControl_Service()`가 500 ms heartbeat deadline을 검사하고 motion settling
   중 servo telemetry/read를 수행
2. 그 뒤 host UART에서 단 1 byte를 최대 10 ms 기다려 읽음
3. 한 번의 loop에서 heartbeat frame 전체를 decode하지 못함

따라서 UART buffer에 heartbeat가 도착해 있어도 settling 중 loop가 늦어지면 frame이
완전히 decode되기 전에 safety service가 stale heartbeat로 판단할 수 있다. 더구나
20E host heartbeat는 serial write 뒤 응답을 기다리지 않는 fire-and-forget이어서,
host는 MCU가 heartbeat를 실제 decode했는지 알 수 없었다.

## 거절 및 rollback 기준

- 20E HEX는 물리 수락용으로 재사용하지 않는다.
- 현재 장치의 stop latch는 12V OFF 상태에서 유지하며 20F 배포 gate 전 임의 reset,
  clear fault 또는 로봇 이동을 하지 않는다.
- rollback은 검증된 20D HEX
  `c4b564145a32994c6601355cebccc08146bfe5287741ec1423a2b5f5c5012126`와
  20E flash 전 전체 image backup을 사용한다.
