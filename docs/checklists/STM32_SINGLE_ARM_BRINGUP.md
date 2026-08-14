# STM32 단일 팔 초기 구동(Bring-up) 체크리스트

## A. 도구와 보드 식별

- [ ] STM32CubeIDE 설치
- [ ] STM32CubeG4 package 설치 확인
- [ ] NUCLEO PCB의 MB1367 revision 기록
- [ ] ST-LINK firmware version 기록
- [ ] ST-LINK SWD 연결 확인
- [ ] Virtual COM Port 번호 기록
- [ ] 빈 펌웨어를 build하고 보드에 기록(flash)한 뒤 debug할 수 있는지 확인

## B. 전원 차단 상태 배선

- [ ] NUCLEO USB 분리
- [ ] 서보 12 V 전원 분리
- [ ] Waveshare adapter 점퍼 `A` 확인
- [ ] NUCLEO–adapter 공통 GND 연결
- [ ] Bus Servo Adapter (A) UART는 adapter RX–MCU RX 연결 확인
- [ ] Bus Servo Adapter (A) UART는 adapter TX–MCU TX 연결 확인
- [ ] 12 V가 NUCLEO 3V3/5V/VIN에 연결되지 않았음을 확인
- [ ] 서보 adapter UART의 유휴(idle) 전압 확인
- [ ] 서보 전원의 극성과 정격 확인

## C. 움직임 없는 NUCLEO 시험

- [ ] 서보 adapter를 분리한 상태로 펌웨어 부팅
- [ ] reset 후 상태가 `SAFE_DISABLED`인지 확인
- [ ] VCP 진단 메시지 수신
- [ ] ARM/ENABLE 명령 전에는 actuator 출력이 없는지 확인
- [ ] CRC, version 또는 message 형식이 잘못된 packet 거부
- [ ] VCP 분리·재연결 후 `SAFE_DISABLED` 유지

## D. 쓰기 명령 없는(Read-only) 서보 bus 시험

- [ ] 팔 주변 충돌 물체 제거
- [ ] 즉시 전원 차단 가능한 위치 확보
- [ ] torque 쓰기 경로가 build 시점 또는 실행 중에 비활성인지 확인
- [ ] 서보 전원을 넣었을 때 명령하지 않은 움직임 없음
- [ ] ID 1~6에 ping 전송
- [ ] ID 0, 7 등 없는 ID가 응답하지 않는지 확인
- [ ] position feedback를 100회 반복해서 읽기
- [ ] speed/load/voltage feedback 확인
- [ ] timeout/checksum 오류 횟수 기록
- [ ] UART 통신 한 건의 지연 시간 p50/p95/최댓값 기록

## E. 제한 동작 전 게이트

- [ ] ID와 관절의 연결 관계 기록
- [ ] 관절별 좌표 증가 방향(sign) 기록
- [ ] 현재 raw 위치 기록
- [ ] 보수적 raw min/max 기록
- [ ] 보정 정보(calibration) hash 생성
- [ ] 펌웨어와 상위 제어기 설정의 hash 일치
- [ ] 목표값 제한(clamp)과 범위 밖 명령 거부 시험
- [ ] heartbeat가 끊기는 상황의 시험 계획 준비

이 체크리스트 E까지 완료되기 전에는 서보 6개의 전체 팔 trajectory를 실행하지 않는다.
