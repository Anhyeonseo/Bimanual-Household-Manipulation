# STM32 단일 팔 액추에이터 제어기

이 디렉터리에는 특정 보드에 의존하지 않는 C11 공통 core가 들어 있다. 실제 서보에 전원을 넣기 전에 PC에서 통신 규격, 안전 상태 전환과 크기가 제한된 setpoint buffer를 시험할 수 있도록 STM32 HAL 및 RTOS 관련 코드와 분리했다.

## 구현된 공통 core

- COBS frame 구성과 CRC-32C 오류 검출
- protocol v1 frame 검증과 byte stream의 frame 경계 복구
- 이상이 생기면 안전한 쪽으로 닫히는 `BOOT → SAFE_DISABLED → ARMED → ACTIVE` 상태 머신
- heartbeat가 끊기면 `HOLD`로 전환
- 해제 명령 전까지 유지되는 `FAULT`와 물리 정지 상태 `ESTOPPED`
- 6축 setpoint를 제한 범위 안에서 한 번에 반영하는 저장 queue
- `protocol/message_ids.json`에서 생성하는 message ID

이 코드는 CubeIDE 보드 프로젝트에 외부 파일(linked resource)로 연결되며 PC에서도 별도로 시험한다. STM32 시작 코드, HAL, STS3215 bus 접근과 flash 설정은 보드 프로젝트에 남긴다. 관절 단위와 서보 raw 값 사이의 보정(calibration)은 이 공통 core가 담당한다.

## PC에서 빌드

~~~powershell
cmake -S firmware/stm32_actuator -B build/stm32_actuator-host
cmake --build build/stm32_actuator-host --config Debug
ctest --test-dir build/stm32_actuator-host -C Debug --output-on-failure
~~~

Windows에서 CMake가 `PATH`에 없다면 Visual Studio Developer PowerShell에서 실행한다.

## 보드와 공통 core의 경계

- 상위 제어기/Pi 연결: NUCLEO 기본 `LPUART1` 경로(`PA2/PA3`)의 STLINK-V3E VCP
- 단일 서보 bus: 별도 `USART1` 경로(`PC4/PC5`)
- 제어 주기: 현재 비차단식 동작 실행기가 20ms마다 갱신
- UART 수신: frame 검증과 상태 처리는 `binary_control` 모듈에서 수행
- 서보 송수신: 명시적인 timeout을 두고 한 번에 처리량이 제한된 통신 사용

배선하거나 CubeMX 설정을 다시 생성하기 전에는 실제 NUCLEO 보드 revision과 pin 경로를 확인해야 한다.

## 자동으로 생성되는 protocol header

~~~powershell
python tools/setup/firmware/generate_protocol_header.py
python tools/setup/firmware/generate_protocol_header.py --check
~~~

저장소에 포함된 header는 사람이 직접 수정하는 문서가 아니다. `protocol/message_ids.json`에 있는 기계 판독용 정의와 항상 일치해야 한다.
