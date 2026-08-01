# 프로젝트 헌장

## 1. 목표

제한된 ARM Linux 플랫폼에서 멀티카메라 인식, MoveIt 기반 전역 경로,
데스크탑 Isaac Sim/Isaac Lab에서 학습한 ONNX policy의 Edge 추론, ROS 2
프로세스 운영과 STM32 실시간 서보 제어를 통합한다. CPU·메모리·지연·정확도·
장애 복구·장시간 안정성을 정량적으로 검증하고 재부팅·재배치 뒤에도 같은
gate를 통과하는 시연 시스템을 만든다.

## 2. 포트폴리오 포지셔닝

이 프로젝트의 중심은 단순한 AI 데모가 아니다.

- 임베디드 펌웨어: UART/DMA, binary protocol, CRC, trajectory buffer, watchdog, fault injection
- 임베디드 Linux: Headless 부팅, systemd, udev, 크기가 제한된 카메라 pipeline, 진단, 복구, 장시간 시험(soak test)
- 로보틱스: URDF, TF, ros2_control, MoveIt, 보정(calibration), Visual Servo, 양팔 작업 일정 관리
- Edge AI: ONNX Runtime, 추론 일정 관리, Isaac Lab policy 내보내기, 출력 범위를 제한한 policy 실행

## 3. 기능 범위

### 1차 범위

- 검증된 왼팔 5DOF와 gripper의 재현 가능한 생산 기준선
- 정상 복구된 오른팔의 후속 단독 동등성 검증
- 고정 작업대와 시연 배경의 검은색 마커펜 검출
- Top 카메라 기반 평면 `x, y, yaw` 추정
- 넓은 펜꽂이로 Pick and Place
- 정확도와 안정성 우선

### 단계적 확장

1. 왼팔 연속 trajectory, 접촉 Z 보정과 손목 카메라 마지막 정렬
2. Raspberry Pi 5의 3카메라·policy ONNX 실행 기준선과 Headless 운영
3. 오른팔 calibration·MoveIt·STM32·손목 카메라 단독 동등성
4. 독립 작업 영역의 양팔 병렬 작업
5. 공유 영역의 양팔 충돌 검사와 coordinated stop
6. 데스크탑에서 학습한 policy의 Pi shadow mode와 bounded residual
7. 수건 접기

### 초기 비범위

- observation contract와 Pi 실측 없이 여러 카메라 원본을 그대로 연결하는 policy
- Raspberry Pi에서의 policy 학습 또는 Isaac Sim 실행
- 고속 동적 장애물 회피
- 안전 인증이 필요한 산업용 운전
- 재현 가능한 기준 동작(baseline) 없이 policy가 로봇을 직접 제어하는 방식

## 4. 시스템 경계

```text
Desktop Isaac 학습 → versioned policy.onnx
Camera → Observation Adapter → Perception/Policy on Pi
       → MoveIt 전역 경로 또는 bounded Visual/Policy residual
       → Command Arbitration → Continuous Trajectory
       → STM32 → Left Arm → Right Arm → Dual Arm
```

Raspberry Pi는 무엇을 할지 결정하고 검증된 policy를 추론한다. MoveIt은
전역 경로와 충돌 검사를 담당하고 policy/Visual Servo는 제한된 국소 보정을
담당한다. STM32는 승인된 명령을 시간과 안전 조건에 맞게 실행한다.

## 5. 단계 통과 원칙

각 단계는 다음 다섯 항목을 갖는다.

1. 목표
2. 구현 항목
3. 검증 방법
4. 완료 조건
5. 실패 시 조사 항목

완료 조건을 충족하지 못하면 다음 단계의 실제 하드웨어 동작을 활성화하지 않는다.

## 6. 안전 원칙

- 자동 부팅 후 기본 상태는 `STANDBY`다.
- 동작 전 `ARMING` 검사를 통과해야 한다.
- 장애 복구 후 자동 재활성화하지 않는다.
- 통신 단절은 감속 정지 후 제한 시간 Hold를 기본으로 한다.
- 심각한 fault와 E-stop은 Torque Disable 및 해제 전까지 유지되는 fault latch로 처리한다.
- 한 팔에서 심각한 fault가 발생하면 양팔을 함께 정지시킨다.
- 물리 E-stop 설치 전에는 저속 벤치 시험만 허용한다.
