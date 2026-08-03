# Motion-9 FollowJointTrajectory buffered Action 통합 결과

## 결론

기존 단일 point Action은 recovery 호환 경로로 유지하고, 2개 이상 point를
가진 `FollowJointTrajectory` goal은 `BufferedActionExecutionCore`를 통해
STM32 queue로 연속 전송하도록 로컬 연결했다. Pi 배포와 다중 관절 실기
실행은 아직 승인하지 않았으며 `motion_authorized=false`를 유지한다.

## 실행 경로

`MoveIt FollowJointTrajectory`
→ 다중점·관절 제한·fresh start 검증
→ 20 ms 위치 재표본화
→ 16 sample prime
→ watermark 10 / refill 16
→ one-shot buffered transport
→ extended firmware terminal
→ 6축 post-settle 진단 2회
→ ROS Action terminal

## MoveIt 호환

- 임의 nanosecond waypoint 시간을 허용하고 마지막 시간을 다음 20 ms
  sample 경계로 올림한다.
- velocity·acceleration 배열이 있으면 축별 유한값·개수·MoveIt 제한을
  검증한다.
- effort는 거부한다.
- Shoulder fresh-start 허용치는 `0.055 rad`, 나머지 축은 `0.050 rad`다.
- gripper commanded setpoint는 arm 경로 전체에서 보존한다.

## queue와 성공 판정

- 400 ms horizon보다 이른 prime/refill은 frame을 만들지 않고 대기한다.
- heartbeat tick에서 apply-lateness 5 ms가 지난 sample만 보수적 진행도로
  계산한다.
- clock 진행도는 refill 판단에만 사용하며 성공 상태를 만들 수 없다.
- admission ACK가 전체 sample 적용을 보고해도 firmware extended terminal
  전에는 성공하지 않는다.
- 성공 terminal 뒤 6축 목표 오차 `30 raw` 이내 진단 2회가 모두 통과해야
  ROS Action을 성공 처리한다.
- timeout, ACK/sequence 불일치, underflow, cancel, terminal 불일치와
  post-settle 실패는 무재전송 SAFE_STOP으로 종료한다.

## 로컬 검증

- continuous prime/refill/terminal/post-settle 시뮬레이션: PASS
- post-settle 31 raw fault injection: fail-closed PASS
- uint32 tick wraparound: PASS
- early second-prime wait / late refill fail-closed: PASS
- legacy single-point와 multi-point buffered routing 분리: PASS
- gripper arbitration·commanded setpoint 보존: PASS
- MoveIt nanosecond timestamp 20 ms 올림: PASS
- 전체 Python/ROS regression: `539 passed`
- `single_arm_bridge`, `so101_moveit_config` symlink build: PASS
- package test result: `21 tests, 0 errors, 0 failures`

## 남은 gate

1. Pi host 백업·전송·rebuild·SHA 확인(무동작)
2. READ_ONLY import/identity와 MOTION_ENABLED 무동작 6축 진단
3. plan-only 짧은 다중 관절 trajectory 생성·SHA 고정
4. 작은 가시 다중 관절 연속경로 1회, terminal·post-settle·DISABLE 확인
5. q0 왕복과 Pick pregrasp 연속경로를 단계적으로 확대

Git 작업은 사용자가 직접 수행한다.
