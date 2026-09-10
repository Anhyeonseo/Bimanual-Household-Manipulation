# Pi–STM32 팔 제어 프로토콜

현재 ROS 실행 경로는 `so101_arm_bridge`의 **protocol v2 양팔 스트림**이다. STM32 펌웨어와 wire 형식은 저장소 재구성 과정에서 변경하지 않았다. 바퀴 제어와 가사 작업 의미는 이 프로토콜에 포함하지 않는다.

## 버전과 소스

| 범위 | 정의 |
|---|---|
| v2 frame·payload·진단 | `ros2_ws/src/so101_arm_bridge/so101_arm_bridge/stream_protocol_v2.py` |
| COBS·CRC-32C | 같은 패키지의 `wire.py` |
| v2 통신 | 같은 패키지의 `stream_transport_v2.py` |
| 명령 소유권·관절 한계·종료 확인 | 같은 패키지의 `bimanual_stream_adapter.py` |
| MCU v2 계약·실행 | `firmware/stm32_actuator/include/actuator_core/`, `src/stream_*_v2.c` |
| 기존 v1 message ID | [message_ids.json](message_ids.json), 생성된 MCU `message_ids.h` |
| v1 검사 codec | `tools/lib/actuator_protocol.py` |

`message_ids.json`은 보존된 v1 manifest다. v2 전체 메시지 목록으로 해석하지 않는다. 이전 v1 ROS 동작 실행기는 제거했으며 v1의 자세한 과거 운용 기록은 [수건 보관본](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80/protocol)에 남아 있다.

## 공통 프레임

ST-LINK VCP를 통해 little-endian 바이트를 전송한다. frame은 COBS 인코딩 후 `0x00`으로 구분하며, header와 payload를 CRC-32C로 검사한다. 현재 양팔 브릿지의 host baud는 921600이다.

| offset | 필드 | 형식 |
|---|---|---|
| 0 | magic `0xA55A` | uint16 |
| 2 | version | uint8 |
| 3 | message type | uint8 |
| 4 | flags | uint16 |
| 6 | payload length | uint16, 최대 512 byte |
| 8 | sequence | uint32 |
| 12 | sender time | uint32, ms |
| 16 | payload | 메시지별 정의 |
| 16 + N | CRC-32C | uint32 |

MCU와 호스트의 절대 시각이 같다고 가정하지 않는다. 적용 시각은 MCU control tick을 기준으로 계산하며 응답의 sequence와 시간 echo를 검사한다.

## 양팔 명령과 피드백

양팔은 왼팔 6축·오른팔 6축의 순서로 하나의 목표 스트림을 사용한다. 한 batch는 최대 9개 표본을 담으며 각 표본에는 12축 목표가 모두 필요하다. ROS 입력은 radian, wire 목표는 부호 있는 micro-radian이다.

세션 준비는 펌웨어 버전 `0x00024809`, capabilities `0xEFFFFFFF`, 양팔 보정 hash `0x2D90167E` 등 기존 식별 계약을 확인한다. 설치된 펌웨어와 맞지 않으면 실행을 거절한다. 이 값의 존재는 새 모바일 플랫폼의 실물 검증을 의미하지 않는다.

`START_FINITE`, `START_OPEN`, `APPEND`, `SPLICE`, `STOP`은 ROS `BimanualStreamCommand`의 연산이다. 브릿지는 이를 wire 세션·batch·stop 명령으로 변환한다. 상위 작업 계층은 UART를 직접 사용하지 않는다.

`BimanualJointFeedback`은 실제 12축 위치, 축별 측정 나이, 유효 축 mask와 MCU 시각을 전달한다. 완료 응답 이후에도 관측의 유효성을 확인해야 하며, 관절 목표 도달만으로 물체의 집기나 배달을 판정하지 않는다.

## 검증과 실행 범위

```bash
python tools/run/validate_protocol_manifest.py
python tools/setup/firmware/generate_protocol_header.py --check
python -m pytest -c config/pytest.ini --rootdir=. -q tests/test_stream_protocol_v2_contract.py tests/test_bimanual_stream_adapter.py
```

위 명령은 통신 계약과 소프트웨어 동작을 검사하며 실제 모터를 실행하지 않는다. 실제 장치 연결·토크·동작은 별도 승인과 초기화·관절 제한·정지 조건 확인이 필요하다.
