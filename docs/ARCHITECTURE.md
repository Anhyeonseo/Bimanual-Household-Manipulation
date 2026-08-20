# 시스템 구조

## 데이터 흐름

```text
USB cameras
  → manipulation_camera_manager
  → top perception / object observations
  → desk scene inventory
  → task and destination policy
  → MoveIt reachability + collision plan
  → desk-task executor
  → bimanual_stream_adapter
  → protocol v2
  → STM32 12-axis resident executor
```

## 책임 경계

| 계층 | 책임 | 금지 |
|---|---|---|
| Perception | 물체 class, 위치, 장축, freshness, confidence | motion 명령 |
| Scene/task | 물체 목록, 목적지, 우선순위, 팔 선택 | serial 직접 접근 |
| MoveIt | IK, joint limit, self/inter-arm/environment collision | 안전 gate 우회 |
| Desk executor | SHA 고정 plan, 단계 상태기계, 결과 검증 | 자동 실제 재시도 |
| Resident adapter | 12축 owner/epoch, finite stream, feedback | 복수 serial owner |
| STM32 | 동기 출력, tracking, heartbeat, stop/latch | 물체/태스크 판단 |

## 안전 불변식

1. 부팅과 재연결만으로 모터가 움직이지 않는다.
2. 상위 앱은 serial을 직접 열지 않는다.
3. 한 팔 동작도 반대 팔 hold를 포함한 12축 command다.
4. perception, calibration, plan SHA 중 하나라도 stale이면 실행을 거부한다.
5. terminal measured feedback 전에는 성공으로 간주하지 않는다.
6. fault 뒤 session을 재사용하지 않는다.

## 유지하는 공통 기반

펌웨어, protocol, 양팔 URDF, resident adapter, 카메라 manager는 책상 정리
시스템의 실행 기반이므로 유지한다. 과거 데모 전용 계획기·모델·실행기·시험
기록은 제거했다. 일부 디렉터리/package의 `single_arm` 이름은 STM32CubeIDE와
ROS 배포 호환성을 위한 legacy 식별자이며 승인된 task 경로를 뜻하지 않는다.
