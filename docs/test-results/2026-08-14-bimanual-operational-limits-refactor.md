# 2026-08-14 양팔 운용 범위 통합과 제한 코드 리팩토링

## 결정

작업자가 J0-D에서 직접 천천히 통과시키고 물리·케이블 문제가 없음을 확인한
양팔 12축 범위를 그대로 inclusive firmware operational limit으로 채택했다.
추가 축별 25/50/75% 승인과 J2-R 팔꿈치 복귀 예외는 더 요구하지 않는다.

단일 canonical manifest:

- `config/bimanual_operational_limits.json`
- SHA256 `436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263`

| arm | Base | Shoulder (unwrapped) | Elbow | Wrist Flex | Wrist Roll | Gripper |
|---|---:|---:|---:|---:|---:|---:|
| left | 983..3041 | 1899..4187 | 286..2492 | 170..2384 | 587..2838 | 1872..3257 |
| right | 1108..2996 | 1859..4188 | 297..2523 | 377..2438 | 749..2970 | 1907..3299 |

Shoulder는 raw 4095→0을 연속 통과하므로 unwrapped raw에서 검사하고 마지막
servo dispatch 직전에만 modulo 4096으로 변환한다. Gripper 범위는 servo command
raw 한계이며 jaw-gap 기구학 보정값은 아니다.

## 코드 정리

- 좌우 12축 raw/µrad/방향/zero와 wrap 변환을
  `bimanual_operational_limits.c/.h` 한 모듈로 통합했다.
- 오른팔 one-shot 서비스가 공통 모듈을 사용하며 protocol-v2 executor용 loader도 같은 모듈에 제공했다.
- `right_arm_command_limits.c/.h`와 J2-R 전용 firmware/bridge/validator/test를 제거했다.
- 과거 `0x00024100` no-output J1-L 표는 증거 재현 전용 함수로 격리했다.
- 새 후보 identity는 firmware `0x00024400`, capability `0x20000000`이다. manifest 요구 없이 연결하거나 legacy `allow_motion`을 켜면 bridge가 fail-closed한다.
- heartbeat, stop latch, communication fault, tracking error, maximum control-tick
  step과 verified torque disable은 유지했다.

## 자동 검증

- 공통 범위 C harness: 12축 JSON↔C parity, inclusive endpoint, out-of-range reject,
  오른쪽 Shoulder `4090 + 20 → modulo 14`, 상단 초과 reject 통과.
- 전체 Python/contract tests: `1312 passed in 39.75s`.
- `stm32_actuator` C tests: `6/6 passed`.
- ROS build: `so101_interfaces`, `single_arm_bridge` 성공.
- Cortex-M4 Release build 성공:
  - `0x00024400` candidate HEX SHA256
    `3638b94318342e531eca48713692e57444b02da70ab76f8cfff4fe3dea843990`
  - default R4 regression HEX SHA256
    `b677bd97a7097ae8bb3be07ad5d5696dee6df803f33aa4ffe89282ad792acd8a`
- default R4 SHA가 기존 안정본과 같아 이번 변경이 기본 바이너리에 영향을 주지
  않았음을 확인했다.

## 경계

이 결과는 범위 정책과 관련 리팩토링 완료다. `0x00024400`의 현재 실제 명령 API는
기존 오른팔 단일-servo ±20 raw 서비스이며, 공통 12축 protocol-v2 executor의 실제
양팔 goal dispatch는 아직 연결하지 않았다. 다음 기능 단계는 추가 축별 범위 시험이
아니라 공통 12축 queue → 좌/우 bus dispatch와 coordinated stop 구현이다.
