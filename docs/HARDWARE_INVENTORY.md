# 하드웨어 인벤토리

상태 표기: `확정`, `측정 필요`, `미정`

## 컴퓨팅

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 주 제어 컴퓨터(SBC) | Raspberry Pi 5 4GB | 확정 | 실제 보유 |
| SBC 운영체제 | Ubuntu Server 24.04 | 확정 | 설치 완료 |
| ROS | ROS 2 Jazzy | 확정 | 설치 완료 |
| Pi 전원 | 공식 27W USB-C | 확정 | 실제 보유 |
| Pi 냉각 | 외부 선풍기 | 확정 | 장시간 시험 전 능동 냉각 재검토 |
| Pi 저장장치 | microSD | 확정 | journald/rosbag 쓰기량 제한 필요 |
| 개발 PC | Windows 11 / Ubuntu 24.04 dual boot | 확정 | RTX 5070 Ti |

## 로봇팔과 서보

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 로봇팔 | 기본형 SO-ARM101 follower × 2 | 확정 | 공식 손목 카메라 mount 적용 |
| Base 설치 방향 | 양팔 동일 방향 | 확정 |  |
| Base 중심 거리 | 약 14 inch / 355.6 mm | 측정 필요 | 최종 mm 측정 필요 |
| 자세 관절 | 팔당 5개 | 확정 | gripper 제외 |
| Gripper 축 | 팔당 1축 | 확정 |  |
| 장착 서보 | STS3215 12V × 12 | 확정 |  |
| 예비 서보 | STS3215 12V × 2 | 확정 |  |
| 서보 ID | 각 버스 1~6 | 확정 | 우팔 R0/R1.1에서 ID 1~6 응답과 관절별 +8 raw 방향 확인 |
| 왼팔 관절 원점/방향 | 프로젝트 q0 raw 2048 기준 | 확정 | 2026-07-26 실기 재정렬 기록 |
| 오른팔 관절 원점/방향 | q0 전 축 raw 2048, 좌팔과 동일 조립 방향 | 확정 | 전원 재인가 후 q0 기록 및 R1.1 전 축 +8 raw 응답 확인 |
| Raw 위치 범위 | 좌팔 실기 범위를 우팔 보수 후보로 채택 | 측정 필요 | 동일 기종·조립 근거로 후보 채택; 우팔 bounded-home 뒤 일반 궤적 전에 범위 gate 필요 |
| Feedback 항목 | position/speed/load/voltage 확인 | 확정 | 단일 시험 팔 ID 1~6 실기 검증 |

## 전원과 서보 버스

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 왼팔 전원 | 12V 10A | 확정 | 실제 출력 전압 측정 필요 |
| 오른팔 전원 | 12V 10A | 확정 | 실제 출력 전압 측정 필요 |
| 서보 전원 분리 | 좌우 독립 | 확정 |  |
| 서보 bus driver | Waveshare Bus Servo Adapter (A) × 2 | 확정 | 왼팔 L ×1, 오른팔 R ×1; UART 논리측↔12 V servo bus 경계 |
| 서보 bus | 좌우 독립 | 확정 | ID 중복 허용, arm namespace 필수 |
| 물리 E-stop | 없음 | 미정 | 자동 작업 전 추가 권장, 양팔 단계 전 필수 검토 |
| 분기 퓨즈 | 확인 안 됨 | 측정 필요 | 배선 사진과 정격 기록 필요 |
| 접지 연결 구조 | 확인 안 됨 | 측정 필요 | Pi/STM32/좌우 adapter 기준 전위 문서화 |

## MCU

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 보드 | NUCLEO-G474RE | 확정 | MB1367-G474RE-D01 |
| 상위 제어기 연결 | On-board ST-LINK VCP | 확정 | 현재 COM3 실기 검증, Pi 연결 시 udev 식별값 확인 필요 |
| 실행 구조 | STM32 HAL 기반 main loop | 확정 | 현재 FreeRTOS 미사용 |
| 현재 서보 연결 | 좌팔 USART1 + 우팔 UART4 독립 bus | 확정 | 양쪽 STS3215 ID 1~6 read-only 동시 점검 통과; 각 팔 Waveshare driver 1개 |
| 양팔 확장 연결 | 우팔 UART4 PC10/CN7-1 TX→driver R TX, PC11/CN7-2 RX←driver R RX | 확정 | Bus Servo Adapter (A)는 같은 이름끼리 연결; 좌우 driver 각 1개; 공통 GND와 3.3 V IO 호환 실측 필요; 12 V bus의 MCU 핀 직접 연결 금지 |
| 현재 배포 펌웨어 | binary protocol v1 / `0x00023B00` (R4 read-only) | 확정 | 양팔 startup verified torque-off 및 12축 ROS read-only soak 100/100 통과; 일반 오른팔 trajectory는 비승인 |
| 우팔 commissioning 상태 | R4 양팔 read-only 통합 통과, 일반 trajectory 비승인 | 확정 | 12축 이름·부호·지속 readback 검증 완료; 다음 gate는 오른팔 물리 joint-zero와 좌우 base transform 실측 |

## 카메라

| 위치 | 장치 | 상태 | 확인할 항목 |
|---|---|---|---|
| 상단(Top) | XPCAM HD 1080p USB webcam | 확정 | VID/PID, serial, UVC format, focus, exposure |
| 왼쪽 손목 | Innomaker 1080p USB 2.0 UVC | 확정 | VID/PID, serial, UVC format, focus, exposure |
| 오른쪽 손목 | Innomaker 1080p USB 2.0 UVC | 확정 | VID/PID, serial, UVC format, focus, exposure |
| USB hub | 전원 공급형 USB 3.0 hub, 5V 3A | 확정 | 연결 구조, 역전원, 카메라 3대 안정성 확인 |

## 작업 환경

| 항목 | 값 | 상태 |
|---|---|---|
| 첫 물체 | 검은색 마커펜 | 확정 |
| 목적지 | 펜꽂이 | 확정 |
| 조명 | 고정 환경 | 확정 |
| 작업대 크기/높이 | 미기록 | 측정 필요 |
| 펜 크기 | 미기록 | 측정 필요 |
| 펜꽂이 입구 | 미기록 | 측정 필요 |
