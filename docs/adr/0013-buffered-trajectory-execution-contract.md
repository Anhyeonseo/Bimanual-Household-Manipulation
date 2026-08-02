# ADR-0013: 왼팔 buffered trajectory 실행 계약

- 상태: 채택
- 날짜: 2026-08-02

## 상황

감독형 Pick and Place는 여러 single-point Action을 각 구간 정착 뒤 이어
붙여 완주했다. 이 방식은 안전한 초기 시운전에는 유효했지만, 매 구간의
가속·감속과 정착 대기 때문에 생산용 연속 실행 계약으로 사용할 수 없다.

공통 C core에는 16-sample setpoint queue가 있고 protocol v1 payload는 한
frame에 최대 9개 sample을 표현한다. 그러나 현재 보드 펌웨어와 ROS Action은
실제로 `sample_count=1`만 수락한다. C queue의 존재만으로 다중점 실행을
지원한다고 선언해서는 안 된다.

## 결정

1. 전체 trajectory의 `time_from_start` 소유자는 ROS 2
   `FollowJointTrajectory`로 유지한다.
2. STM32는 ROS 시간을 다시 계획하지 않고 변환된 `uint32` millisecond
   `apply_tick`과 검증된 위치 sample 사이를 보간한다.
3. 첫 buffered 구현은 위치 기반 선형 보간이다. velocity, acceleration과
   effort 필드는 조용히 버리지 않고 비어 있지 않으면 거부한다.
4. 모든 점은 물리 위치 제한, MoveIt velocity·acceleration 제한, strictly
   increasing integer-millisecond 시간축을 통과해야 한다.
5. `time_from_start=0` 점은 fresh feedback과 시작 허용치 안에서 일치해야 한다.
6. batch는 전체가 유효할 때만 원자적으로 queue에 들어간다. 부분 수락과
   자동 재전송은 금지한다.
7. operator cancel, queue underflow, missed tick과 connection loss는 남은
   queue를 폐기하고 자동 재개를 금지한다. planned HOLD는 latched cancel과
   별도 상태로 유지한다.
8. uint32 tick wrap은 STM32 C core와 같은 half-range ordering으로 처리한다.
9. 이번 구현 상태는 `HOST_MOCK_ONLY`, `motion_authorized=false`다. 기존
   single-point Action 실행 경로를 변경하지 않는다.
10. minimum/maximum lead, startup prime depth, low watermark, refill target,
    serial RTT와 host jitter는 측정 전 상수로 추측하지 않는다. 다음 firmware
    이슈에서 로컬 fault injection으로 측정·고정한다.

## 영향

- 현재 물리 동작 능력은 늘어나지 않는다.
- 다음 firmware 구현은 기계 판독 계약과 동일한 cancel/HOLD/underflow 의미를
  지켜야 한다.
- MoveIt trajectory의 동적 필드를 실제로 사용할 필요가 생기면 선형 위치
  계약을 몰래 변경하지 않고 새 ADR과 테스트로 보간 방식을 변경한다.
- buffered firmware와 host adapter가 각각 검증된 뒤에만 제한 실기로 간다.

## 다음 gate

1. STM32 queue admission·interpolation·terminal result를 board execution
   path에 연결한다.
2. lead와 watermark 후보를 host-only latency/fault injection으로 측정한다.
3. protocol capability와 firmware identity를 올리고 이전 host의 motion
   연결을 fail-closed로 거부한다.
4. mock·plan-only 뒤 명시적 승인으로 단일 관절 제한 실기를 수행한다.

## 구현 진행

- Motion-1: host validation과 queue mock 완료
- Motion-2: STM32 공통 C core의 원자적 queue, 선형 보간, refill,
  terminal diagnostics와 fault injection 완료
- 현재 G474 binary command route, identity와 capability는 의도적으로
  변경하지 않았다. 따라서 runtime은 계속 single-sample이며 물리 buffered
  motion authority가 없다.
