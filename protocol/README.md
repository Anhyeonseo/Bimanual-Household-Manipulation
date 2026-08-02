# Pi–STM32 통신 규격 v1 초안

상태: `채택`. 단일 팔용 protocol v1은 실기에 적용했으며, 여러 sample queue와 양팔 payload 실행은 이후 단계에서 추가 검증한다.

## 1. 범위

이 규격은 Raspberry Pi의 ROS 2 제어 bridge와 NUCLEO-G474RE 사이에서 사용한다. STM32와 STS3215 사이에서는 Feetech STS bus protocol을 별도로 사용한다.

~~~text
ROS 2 / Pi
    ↓ ST-LINK VCP, 이 문서에서 정의한 protocol
STM32G474
    ├─ UART → Left Waveshare adapter → STS3215 ID 1~6
    └─ UART → Right Waveshare adapter → STS3215 ID 1~6
~~~

`START`, `SEARCH`, `PLACE`처럼 작업 의미를 나타내는 상태는 ROS 계층에 둔다. MCU protocol은 actuator 활성화, setpoint, 상태와 fault만 다룬다.

## 2. 전송 방식과 frame 구분

- 전송 경로(transport): ST-LINK Virtual COM Port
- Byte 순서: little-endian
- Frame 구분: COBS로 encoding한 frame 뒤에 구분값(delimiter) `0x00` 추가
- 오류 검출: CRC-32C
- Decode 후 payload 최대 크기: 512 byte
- 구조체를 `memcpy`로 바로 보내지 않는다. 메모리 정렬용 여백(padding)과 compiler ABI에 의존하지 않도록 byte 단위로 encode/decode한다.

Decode한 frame 구조:

| 시작 위치 | 자료형 | 필드 | 설명 |
|---:|---|---|---|
| 0 | `uint16` | magic | 고정값 `0xA55A` |
| 2 | `uint8` | version | protocol의 주 version, 초기값 1 |
| 3 | `uint8` | message_type | `message_ids.json` 참조 |
| 4 | `uint16` | flags | ACK 요청, 응답, 오류 등의 표시 |
| 6 | `uint16` | payload_length | payload 크기, 최대 512 byte |
| 8 | `uint32` | sequence | 전송 방향마다 따로 증가하는 번호 |
| 12 | `uint32` | sender_time_ms | 송신 장치에서 계속 증가하는 시간, 최댓값 이후 0으로 돌아감 허용 |
| 16 | bytes | payload | message마다 정의한 실제 데이터 |
| 16+N | `uint32` | crc32c | header와 payload 전체 검사값 |

CRC가 맞더라도 magic, version, type, length 또는 현재 MCU 상태가 올바르지 않으면 packet을 거부한다.

## 3. Sequence와 재전송

- Pi→MCU와 MCU→Pi의 sequence는 서로 독립적으로 증가한다.
- `uint32`가 최댓값을 넘어 0으로 돌아가는 현상은 modular comparison으로 처리한다.
- 상태를 바꾸는 명령이 중복되면 이전과 같은 결과를 반환하되 동작을 다시 실행하지 않는다.
- 이미 지난 setpoint sequence는 거부한다.
- ACK가 필요한 명령만 정해진 횟수와 간격으로 재전송한다. 정확한 값은 VCP 지연 시간을 측정한 뒤 확정한다.

## 4. 시간 기준

Header의 `sender_time_ms`는 데이터가 오래됐는지 확인하고 문제를 진단하는 용도다. Raspberry Pi와 STM32의 절대 시간이 같다고 가정하지 않는다.

Setpoint는 MCU가 알려 준 `control_tick`을 기준으로 `apply_tick`을 지정한다.

~~~text
HELLO/TIME_SYNC
→ MCU의 현재 control_tick 확인
→ Pi가 충분한 여유 시간(lead time)을 둔 apply_tick 생성
→ STM32의 크기가 제한된 queue에 저장
→ 같은 apply_tick에 좌우 setpoint를 한 번에 적용
~~~

다음 값은 양팔 및 장시간 실기 시험 후 최종 확정한다.

- control loop 주기
- 최소 apply lead tick
- heartbeat 제한 시간
- setpoint queue 크기
- queue low-watermark
- 감속 정지 시간

## 5. 관절 단위와 보정(calibration)

- 전송 위치: 부호 있는 micro-radian(`int32`, µrad)
- 전송 속도: 부호 있는 micro-radian/second(`int32`)
- 전송 가속도: 부호 있는 micro-radian/second²(`int32`)
- 전압: millivolt(`uint16`)
- 부하: STS raw feedback와 정규화 값의 관계는 이후 단계에서 확정

Raspberry Pi는 STS3215 raw 위치를 보내지 않는다. STM32가 보정 정보에 기록된 방향 부호, 원점과 안전 raw 범위를 사용해 관절 단위를 서보 raw 단위로 바꾸고 마지막 안전 제한을 적용한다.

Pi와 STM32는 `HELLO` 단계에서 `calibration_hash`를 비교한다. 값이 다르면 `ARMING`을 거부한다.

### 실제 관절 위치 feedback

`HELLO_RESPONSE.capabilities`의 bit 3(`0x00000008`)이 1이면 실제 서보 위치 feedback을 지원한다. Host가 `GET_STATE`에 payload `01`을 넣으면 STM32는 기존 20-byte `STATE_FEEDBACK` 뒤에 `uint16 raw_position[6]`을 추가해 총 32 byte로 응답한다. 빈 payload는 기존 20-byte 응답을 유지하므로 이전 점검 도구와 호환된다.

Raw feedback은 STM32와 hardware bridge 사이의 측정 경계에서만 사용한다. ROS 2 node는 calibration의 원점과 방향을 적용해 radian으로 변환한 뒤 `/joint_states`에 발행한다. ROS·MoveIt·Isaac Sim 바깥 인터페이스에는 raw 값을 노출하지 않는다.

### Background position-read failure diagnostics

`HELLO_RESPONSE.capabilities`의 bit 8(`0x00000100`)이 1이면 위치 포함
`GET_STATE`의 서보 읽기 실패 응답은 최소 24-byte `STATE_FEEDBACK`이다. 기본
20 byte 뒤에 `failed_servo_id`, `consecutive_failure_count`,
`failure_limit`, `reserved`를 각각 `uint8`로 붙인다. 이 24-byte 형식은
firmware `0x00021700` 진단과의 호환을 위해 유지한다.

`HELLO_RESPONSE.capabilities`의 bit 9(`0x00000200`)도 1이면 firmware
`0x00021800`의 UART frame recovery 진단을 지원하며 실패 응답은 40 byte이다.
앞 24 byte는 위 형식과 같고 뒤 16 byte는 다음과 같다.

| offset | type | field |
|---:|---|---|
| 23 | `uint8` | failure reason (`0=none`, `1=TX`, `2=RX timeout`, `3=UART`, `4=header`, `5=servo ID`, `6=length`, `7=servo status`, `8=checksum`, `9=recovery`) |
| 24 | `uint8` | `HAL_StatusTypeDef` 값 |
| 25 | `uint8` | 서보 status/error byte |
| 26 | `uint16` | 누적 UART recovery 횟수 |
| 28 | `uint16` | 이번 응답 전까지 폐기한 byte 수 |
| 30 | `uint16` | reserved (`0`) |
| 32 | `uint32` | `UART_HandleTypeDef.ErrorCode` snapshot |
| 36 | `uint32` | USART ISR snapshot |

모든 다중 byte 값은 little-endian이다. Host는 payload 길이가 24 byte이면
기존 필드만 사용하고, 40 byte이면 확장 원인을 함께 표시한다.

한 위치 sweep은 실패한 축을 내부에서 3회 재시도한다. 배경 feedback에서는
이 sweep 실패가 서로 다른 host feedback 주기에서 3회 연속 발생할 때만 stop을
latch하며, 중간에 한 번이라도 전체 6축 읽기가 성공하면 누적값을 0으로
초기화한다. 반면 trajectory 시작 위치와 종료 정착 검증 sweep 실패는 기존처럼
첫 exhausted sweep에서 즉시 latch한다. 따라서 통신 순간 오류는 축과 누적 횟수를
남기면서 복구할 수 있고, 지속적인 feedback 상실과 동작 중 검증 실패는 fail-closed로
유지된다.

각 서보 READ는 최대 50 ms와 64 byte로 제한된 stream parser를 사용한다. Parser는
stale prefix, 다른 ID의 늦은 응답, 잘못된 길이 또는 checksum frame을 폐기한 뒤
같은 트랜잭션 안에서 기대한 frame을 다시 찾는다. 끝내 성공하지 못하면 UART를
abort하고 ORE/NE/PE/FE/RTO 상태와 RX data를 비운 뒤 quiet interval을 거쳐 다음
트랜잭션을 시작한다. 따라서 단일 손상 frame은 자동 재동기화하고, 완전한 무응답과
UART 하드웨어 오류는 원인을 보존한 채 기존 3-strike fail-closed 정책으로 넘어간다.

### On-demand 서보 diagnostics

`HELLO_RESPONSE.capabilities`의 bit 4(`0x00000010`)가 1이면 message id
`51 (DIAGNOSTICS)`를 지원한다. Host는 `GET_STATE` payload 두 바이트
`02 joint_index`를 보내며 `joint_index`는 `0..joint_count-1`이다. MCU는 한
요청에서 한 서보만 읽는다. Host는 여섯 관절 요청 사이에 heartbeat를 보내
500 ms watchdog을 굶기지 않는다. 동작이 active인 동안 diagnostics는 거부한다.

firmware `0x00021200`부터 `DIAGNOSTICS` payload는 48 byte,
little-endian이다. 기존 30 byte 뒤에 실제 명령 레지스터와 서보 식별·보호 설정을
붙인다. 이 값은 진단 전용이며 읽기만으로 관절 명령을 만들지 않는다.

| offset | type | field |
|---:|---|---|
| 0 | `uint8` | status (`0=정상`, `2=read 실패/동작 중`) |
| 1 | `uint8` | joint_index |
| 2 | `uint8` | joint_count |
| 3 | `uint8` | protocol_version |
| 4 | `uint32` | calibration_hash |
| 8 | `uint32` | sample_time_ms |
| 12 | `uint8` | servo_id |
| 13 | `uint8` | read_status bitmask |
| 14 | `uint8` | torque_enable register 40 |
| 15 | `uint8` | P gain register 21 |
| 16 | `uint8` | D gain register 22 |
| 17 | `uint8` | I gain register 23 |
| 18 | `uint8` | voltage raw (0.1 V) |
| 19 | `uint8` | temperature (°C) |
| 20 | `uint16` | position raw |
| 22 | `uint16` | speed raw |
| 24 | `uint16` | load raw |
| 26 | `uint16` | current raw |
| 28 | `uint16` | runtime torque limit register 48..49 |
| 30 | `uint16` | goal position register 42..43 |
| 32 | `uint16` | model number register 3..4 (`STS3215=777`) |
| 34 | `uint8` | servo firmware major register 0 |
| 35 | `uint8` | servo firmware minor register 1 |
| 36 | `uint16` | EEPROM max torque limit register 16..17 |
| 38 | `uint16` | minimum startup force register 24..25 |
| 40 | `uint8` | CW dead zone register 26 |
| 41 | `uint8` | CCW dead zone register 27 |
| 42 | `uint16` | protection current register 28..29 |
| 44 | `uint8` | operating mode register 33 |
| 45 | `uint8` | protective torque register 34 |
| 46 | `uint8` | protection time register 35 |
| 47 | `uint8` | overload torque register 36 |

`read_status` bit 0은 P/D/I read, bit 1은 runtime register 40..49 read,
bit 2는 telemetry 56..70 read, bit 3은 identity register 0..4 read, bit 4는
EEPROM protection register 13..39 read 실패다. bit 7은 trajectory active라
진단이 거부됐음을 뜻한다. 진단 실패만으로 위치 명령을 만들거나 자동 재시도하지
않는다.

### Acknowledged heartbeat

`HELLO_RESPONSE.capabilities`의 bit 5(`0x00000020`)가 1이면 heartbeat는
확인 응답 방식이다. MCU는 payload가 빈 `HEARTBEAT`를 수락해 watchdog 시각을
갱신한 직후, 요청과 같은 sequence의 20-byte `STATE_FEEDBACK`을 반환한다.
Host는 250 ms 이내에 그 ACK를 받고 `status=0`, `stop_latched=0`을 모두 확인해야
heartbeat 성공으로 인정한다. 단순 UART write 성공은 heartbeat 전달 증거가 아니다.

firmware 0x00021000부터 host LPUART1은 polling이 아니라 RX interrupt와 1024-byte
ring buffer로 수신한다. ISR은 byte 저장과 다음 수신 rearm만 수행하고 protocol parsing은
main loop에서 최대 64 byte씩 처리한다. 이 구조는 servo UART 동기 transaction 중에도
heartbeat frame을 보존한다. Ring overflow, UART error 또는 rearm 실패는 parser reset,
HOLD와 stop latch로 fail-closed 처리한다. capability bit 6(0x00000040)이 이 계약을
나타낸다. ACK 누락·sequence 불일치·latched 응답은 host transport 오류이며 자동 동작
재시도로 이어지지 않는다.

## 6. Setpoint를 한 번에 적용하는 규칙

`SETPOINT_BATCH` frame 하나에는 좌우 각 6개 actuator의 목표가 들어간다. 한 팔만 움직일 때도 반대쪽 목표를 현재 Hold 목표로 채운다.

v1 payload 구조:

~~~text
uint32 apply_tick_ms
uint8  sample_count       # 1~9, 512-byte frame 제한
uint8  arm_mask           # bit0=left, bit1=right
uint16 reserved           # 반드시 0
for each sample:
    uint32 tick_offset_ms
    int32  left_position_urad[6]
    int32  right_position_urad[6]
~~~

현재 단일 팔 초기 구동 펌웨어는 `arm_mask=1`만 허용하고, 존재하지 않는 오른팔 목표 6개가 모두 0인지 검사한다. 양팔 통합 시에는 두 팔의 현재 Hold 목표를 모두 포함하는 규칙으로 확장한다.

초기 검증 단계의 `SETPOINT_STATUS.status` 값:

- `0`: queue가 명령을 정상 접수
- `1`: payload 또는 적용 시각 형식 오류
- `2`: `ACTIVE` 상태가 아니거나 stop latch 상태
- `3`: 관절각 변환 실패 또는 raw limit 위반
- `4`: 지원하지 않는 팔 위치(slot)
- `5`: 전체 검증은 통과했지만 실행하지 않는 validation-only 상태
- `6`: 실행 완료. `detail`은 최대 raw 위치 오차이며 최댓값은 255
- `7`: servo bus 설정, 쓰기 또는 최종 읽기 실패
- `8`: Heartbeat, `HOLD` 또는 `SAFE_STOP`으로 실행 중단
- `9`: 동작 중 load/current 안전 한계 초과 또는 telemetry 읽기 실패. `detail`은 원인이 발생한 servo ID

`flags.bit0=1`이면 packet 전체를 검사만 하고 실행하지 않는다. 현재 단일 팔
실행기는 `flags.bit0=0`과 `sample_count=1`만 실행한다.

Motion-3의 dormant 후보는 bit 1을 candidate 식별자, bit 2/3/4를
BEGIN/START/END로 사용한다. 최대 9개 sample을 원자적으로 queue에 넣고 기존
16바이트 status 뒤에 executor/terminal/queue 진단을 붙인 32바이트 응답을
정의한다. Host는 두 길이를 모두 해석하지만 현재 `0x00021800`은 이를 광고하거나
실행하지 않는다. route·identity·capability 연결과 timing 실측 전에는 물리
buffered 실행 권한이 없다.

firmware `0x00020E00`부터 endpoint는 보간 종료 100 ms 뒤 한 번만 읽지
않는다. 최대 1000 ms 동안 load/current watchdog를 유지하면서 위치를 읽고,
최대 관절 오차가 30 raw 이내인 sample이 2회 연속이면 조기 완료한다. 최대
시간에는 마지막 sample의 최대 오차를 `status=6.detail`로 보고한다.
`0x00021000`부터 시작 위치와 endpoint 위치 읽기는 main-loop당 한 축의
cooperative sweep이고, safety telemetry도 16 ms round-robin slot으로 한 축씩
읽는다. Host Action은 최악 시작 sweep과 settling을 포함하도록 trajectory
시간 뒤 3.5초의 terminal 여유를 둔다.

- Packet 전체가 유효할 때만 queue에 반영한다.
- 일부 관절만 따로 반영하지 않는다.
- Integer 전송 형식에는 NaN이 존재하지 않는다.
- 단위 변환 overflow, limit 위반 또는 불연속 setpoint가 있으면 packet 전체를 거부한다.

## 7. MCU 상태 머신

~~~text
BOOT
  → SAFE_DISABLED
  → ARMED
  → ACTIVE
  → HOLD

어느 상태에서든 조건에 따라:
  → FAULT
  → ESTOPPED
~~~

- `SAFE_DISABLED`: 통신과 상태값 읽기는 가능하지만 actuator 명령은 금지
- `ARMED`: 장치 상태와 설정 검사를 통과했지만 setpoint 실행은 아직 금지
- `ACTIVE`: 제한된 setpoint 실행 허용
- `HOLD`: 감속 정지한 뒤 현재 위치 유지
- `FAULT`: 원인을 제거하고 명시적으로 `CLEAR_FAULT`를 보내기 전까지 잠금 유지
- `ESTOPPED`: 물리 E-stop 입력을 해제하고 정해진 복구 절차를 수행할 때까지 잠금 유지

전원 인가, VCP 재연결 또는 Pi process 재시작만으로 `ACTIVE` 상태가 되지 않는다.

## 8. 정지 명령 구분

- `HOLD`: 계획된 일시 정지 또는 짧은 통신 이상
- `SAFE_STOP`: Pi가 요청하는 감속 정지이며 물리 E-stop이 아님
- `DISABLE`: torque 명령 비활성화 요청
- firmware `0x00021100`부터 `DISABLE`은 멱등적인 물리 안전 연산이다. 이미
  `FAULT` 또는 `ESTOPPED`여도 논리 상태와 stop latch를 지우지 않은 채 6축
  torque OFF write/readback을 수행한다. 물리 readback 성공은 status 0,
  실패는 status 2로 응답한다. 따라서 status 0은 fault 해제나 motion 허용을
  뜻하지 않고 오직 6축 torque OFF 확인을 뜻한다.
- 물리 E-stop: 독립 입력과 전원 계통으로 처리하고 `FAULT_REPORT`로만 상태 보고

Serial message 이름으로 `ESTOP`을 사용하지 않는다. Software packet이 물리 E-stop과 같은 수준의 안전을 보장한다는 오해를 막기 위해서다.

## 9. Fault code 범위

| 범위 | 분류 |
|---|---|
| `0x0000` | fault 없음 |
| `0x0100–0x01FF` | 상위 제어기 연결 또는 heartbeat |
| `0x0200–0x02FF` | framing, CRC 또는 protocol |
| `0x0300–0x03FF` | setpoint queue 또는 적용 시각 |
| `0x0400–0x04FF` | 관절 위치, 속도 또는 가속도 제한 |
| `0x0500–0x05FF` | STS 서보 응답, 과부하 또는 온도 |
| `0x0600–0x06FF` | 전원 또는 전압 |
| `0x0700–0x07FF` | watchdog 또는 물리 E-stop |
| `0xFF00–0xFFFF` | 펌웨어 내부 fault |

세부 fault 번호는 펌웨어에 넣기 전에 기계가 읽을 수 있는 별도 manifest로 고정한다.

## 10. 검증 항목

구현 전 확인:

~~~bash
python3 tools/validate_protocol_manifest.py
~~~

펌웨어 단계 확인:

- 임의 byte stream에서 구분값을 찾아 frame 경계를 다시 맞춤
- 잘린 frame, 제한보다 큰 frame, 알 수 없는 version 거부
- CRC bit가 바뀐 오류 검출
- 중복 명령을 한 번만 실행
- 오래됐거나 순서가 뒤바뀐 setpoint 거부
- queue overflow/underflow fault 처리
- heartbeat가 끊기면 정지
- config hash가 다르면 `ARMING` 거부
- 한 팔에서 서보 fault가 발생하면 양팔 동시 정지
