# 0x00022500 servo UART 전원 도메인 수명주기 — 빌드 재현성과 로컬 회귀

## 결론

작업 트리의 servo UART 수정 세트가 이미 존재하던
`artifacts/firmware/2026-08-05/stm32_g474_single_arm_0x00022500.hex`의 정확한
출처임을 **clean cross-build 재현으로 증명**했다. 재빌드 HEX는 기존 artifact와
byte-identical이고 compiler warning은 0이다.

이 문서는 데스크탑 빌드·회귀 증거만 담는다. 플래시, 물리 실행, soak은
수행하지 않았고 `motion_authorized=false`와 계약의 `deployed=false`를 유지한다.

## 배경

Motion-11 세 번째 물리 시도(2026-08-05 01:38,
`artifacts/motion/2026-08-05/motion11_pick_pregrasp_lead160_anchor2531_BrD5T6.log`)는
`PLAN_GATE=PASS` 직후 `TimeoutError: joint state timed out`으로 종료됐다.
경로나 queue 문제가 아니라 servo UART가 응답을 전혀 반환하지 못한 것이다.

이후 `0x00022200` → `0x00022500` 네 번의 후보가 만들어졌으나, 마지막으로
살아남은 빌드 트리는 `0x00022400`이었고 소스 mtime(04:18–04:34)이 soak 도구
작성 시각(04:59)보다 앞섰다. 따라서 **현재 소스가 0x225 HEX의 출처라는 보증이
없었다.** 플래시 전에 이 연결을 복원하는 것이 이번 작업의 목적이다.

## 재현성 게이트

```
cmake -S firmware/stm32_g474_single_arm \
      -B build/stm32_g474_single_arm-0x225-release-make \
      -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/stm32_g474_single_arm/cmake/arm-none-eabi.cmake
cmake --build build/stm32_g474_single_arm-0x225-release-make -j
```

- 도구: `arm-none-eabi-gcc 13.2.1 20231009`, `cmake 3.28.3`, Unix Makefiles, Release
- compiler warning: `0`, error: `0`
- 재빌드 HEX SHA-256:
  `aae044c0256d5f634c029cd5e6221b00413bd5ba5fe523bee20e978aeeb2f091`
- 기존 artifact SHA-256:
  `aae044c0256d5f634c029cd5e6221b00413bd5ba5fe523bee20e978aeeb2f091`
- 판정: **일치 (byte-identical)**
- ELF size: text `39520`, data `112`, bss `5872` (dec `45504`)

교차 확인으로 기존 `0x00022400` 빌드 트리
(`build/stm32_g474_single_arm-0x224-release-make-v2`)도 자기 artifact와
byte-identical임을 확인했다
(`395e14664133e94bd13d79950c6b7d1cd42712e53bb2af516892c22b687c87b9`).
같은 toolchain에서 이 프로젝트의 cross-build가 재현 가능하다는 뜻이다.

## 고정된 identity

- `HOST_BINARY_FIRMWARE_VERSION`: `0x00022500`
  (`firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h:11`)
- `HOST_BINARY_CAPABILITIES`: `0x00000FFF` — 이전 배포와 동일, 신규 bit 없음
- host `EXPECTED_FIRMWARE_VERSION`: `0x00022500`
  (`ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py:8`)
- calibration hash: `0x8AD27897` (변경 없음)
- 계약 `servo_uart_receive_candidate.deployed`: `false` (유지)

## 새로 고정한 불변식

`ServoBus_Recover → ServoBus_HardResyncReceiver`가 `HAL_Delay`를 포함하므로
buffered 실행 중 servo read가 실패하면 1 ms executor를 막고 5 ms apply-lateness
게이트를 넘길 수 있다는 우려가 있었다. 소스 검사 결과 그 경로는 도달 불가하며,
관찰을 회귀 시험으로 못 박았다
(`tests/test_stm32_servo_uart_circular_dma_contract.py`).

- `test_buffered_setpoint_hot_path_never_opens_an_rx_transaction`
  — `Servo_SyncWritePositions`(5 ms hot path) 본문에
  `ServoBus_PrepareTransaction` / `ServoBus_Recover` / `ServoBus_ArmReceiver` /
  `ServoBus_DisarmReceiver` / `ServoBus_HardResyncReceiver` /
  `ServoBus_WaitForIdleHighStable` / `HAL_Delay`가 하나도 없고
  `HAL_UART_Transmit`만 있음을 단언한다. 또한
  `Host_ServiceBufferedExecution`에 `Servo_ReadData` / `Servo_ReadPosition`
  호출이 없음을 단언한다.
- `test_servo_reads_are_refused_while_buffered_execution_is_active`
  — `binary_control.c`의 모든 `Servo_ReadData` 호출이
  `Host_SendBinaryDiagnostics` 안에 있고, 그 함수의
  `Host_BufferedExecutionIsActive() != 0U` 게이트가 첫 호출보다 앞섬을 단언한다.

**결론: Motion-11 재시도 전 별도 완화가 필요 없다.**

## DISABLE sweep 예산 재유도

`Servo_WriteData`와 `Servo_ReadData`는 이제 전송 전에 매번
`ServoBus_PrepareTransaction`을 지불한다. `ServoBus_ArmReceiver`가 안정된
idle-high를 최대 `SERVO_BUS_IDLE_HIGH_TIMEOUT_MS = 20 ms` 기다리고, 이어지는
preflight loop이 `UART_FLAG_BUSY` 해제를 최대
`SERVO_BUS_PREFLIGHT_IDLE_TIMEOUT_MS = 2 ms` 기다린다. 즉 transaction 하나당
`22 ms`가 추가되고, `Servo_DisableTorqueAll`은 6회 write + 6회 readback이므로
12회분이 붙는다.

`tests/test_stm32_physical_disable_contract.py`의 산술을 가정하지 않고 소스
상수에서 다시 유도하도록 고쳤다.

| 항목 | 값 |
|---|---|
| write × 6 | `6 × (22 + 100 + 2) = 744 ms` |
| settling delay | `5 ms` |
| readback × 6 | `6 × (22 + 5 + 50 + 2) = 474 ms` |
| **firmware worst case** | **`1223 ms`** |
| `DISABLE_RESPONSE_TIMEOUT_S` | `2.5 s` = `2500 ms` |
| **여유** | **`1277 ms`** (요구 `≥500 ms`) |

`docs/test-results/2026-07-31-host-disable-timeout-contract.md`가 기록한
`1817 ms` 봉투는 당시 `READ_TX_TIMEOUT 100 ms` / `READ_TIMEOUT 100 ms`
기준이었다. 이번 변경에서 두 값이 `5 ms` / `50 ms`로 줄었기 때문에, 신규
transaction 준비 비용 `264 ms`를 더해도 총합은 오히려 감소했다.
**host timeout 변경이 필요 없다.**

## 문서 drift 교정

`docs/test-results/2026-08-04-motion11-buffered-pick-pregrasp-plan-only.md`가
`43000 ms / 2151 samples`를 기록했으나 생성기 상수는 `47000 ms / 2351 samples`였다
(`tools/plan_buffered_pick_pregrasp.py:40-42`). 문서를 손으로 고치는 대신
계획을 재생성해 실제 값을 읽어 반영했다.

재생성 명령과 결과:

```
python3 tools/plan_buffered_pick_pregrasp.py --plan-only \
  --anchor-raw 2273 2531 1844 1940 2141 2002 \
  --source-route artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015/01_q0_to_pick_pregrasp.json
```
`SHA256=892d16a871204e1ecf327f450fda903afd98ed8be74ccc2f948abda609eff04b`
— `tests/test_plan_buffered_pick_pregrasp.py`의 기대 artifact SHA와 일치한다.

| 항목 | 이전 문서 | 실제 값 |
|---|---|---|
| anchor→q0 | `8000 ms` | `12000 ms` |
| 총 시간 | `43000 ms` | `47000 ms` |
| waypoint/sample | `2151개` | `2351개` |
| 최대 sample step | `0.002028 rad` | `0.002316 rad` |
| 최대 이산 velocity | `0.101400 rad/s` | `0.115800 rad/s` |
| 최대 이산 acceleration | `0.042500 rad/s²` | `0.032500 rad/s²` |
| firmware 출력 수 | `8601개` | `9401개` |
| 모델 최대 peak error | `79.987 raw` | `86.080 raw` |
| batch 수 | `358` | `392` |
| accepted samples | `2151` | `2351` |
| applied / queued | `2136 / 15` | `2340 / 11` |
| 마지막 apply offset | `43160 ms` | `47160 ms` |

재발 방지를 위해 `tests/test_motion11_plan_documentation_contract.py`(5개)를
추가했다. 문서를 파싱해 duration·sample 수·추종률 계약·firmware identity·
apply offset을 생성기 상수와 직접 비교한다. 총 시간을 `43000 ms`로 되돌리는
음성 시험에서 실제로 실패함을 확인했다.

## 저장소 위생

- `docs/VERIFICATION_MATRIX.md.orig`, `README.md.orig` 삭제. 둘 다
  `.gitignore:41`의 `*.orig`에 걸려 untracked였고, 현재 파일보다 오래된
  스냅샷이며 conflict marker나 고유 내용이 없었다.
- `pytest.ini` 추가. `unittest discover`는 `tests/` 77개 파일 중
  `unittest.TestCase`를 쓰는 39개만 수집하고 나머지 pytest 형식을 놓친다.
  이제 overlay를 source한 뒤 `python3 -m pytest -q` 한 줄로 전체가 돌아간다.

## 로컬 회귀

```
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 -m pytest -q
```
- **`607 passed`** (이전 기준선 600 + DMA 불변식 2 + 문서 계약 5)
- `/opt/ros/jazzy`만 source하면 ament_index가 `single_arm_bridge`를 찾지 못해
  `tests/test_moveit_external_execution.py` 4개가 실패한다. workspace overlay가
  필요하다는 사실을 `pytest.ini` 주석에 기록했다.

```
cmake -S firmware/stm32_actuator -B <build> && cmake --build <build> && ctest
```
- `2/2 passed` (`actuator_core_tests`, `actuator_buffered_command_route_tests`)

## 다음 gate (물리, 미실행)

1. 0x225 HEX를 Pi로 전송하고 Pi에서 SHA-256 재확인.
2. 현재 플래시된 `0x00022100` 전체 백업.
3. OpenOCD program / verify / reset.
4. identity 게이트 — `0x00022500 / 0x00000FFF / 0x8AD27897`.
5. **cold-start 검사** — MCU 동작 중 스위치드 12 V 서보 도메인 전원 순환 후
   `recovery_count ∈ {0,1}` 이고 `fe_count == recovery_count ==
   receiver_resync_count`인지 확인. 근본 원인 가설의 직접 시험이므로 soak보다 먼저.
6. `tools/validate_servo_uart_dma_read_only.py --duration-s 300` READ_ONLY soak.
7. 통과 시에만 계약과 validator에서 `deployed: true`로 동시 전환.
