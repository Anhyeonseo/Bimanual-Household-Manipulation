# Motion-5 buffered host Action adapter 후보

## 목적

검증된 MoveIt 다중점 경로를 20 ms sample로 바꾸고, 실측 정책
`prime 16 → watermark 10 → refill 16`에 따라 frame을 만드는 host-only
스케줄러를 고정한다. ROS Action 서버·serial execution route·로봇에는 아직
연결하지 않는다.

## 구현 계약

- 첫 sample lead `100 ms`, 허용 lead `60..400 ms`
- 5축 arm 경로를 20 ms 선형 재샘플링하고 현재 gripper 위치 보존
- 시작 queue는 `9 + 7` sample 두 frame으로 16개를 채운 뒤 START
- queue가 10 이하가 되면 최대 9개/frame으로 16까지 계속 refill
- ACK의 accepted/applied/queued 합계가 정확히 일치해야 진행
- pending frame 재호출, 거부 ACK, 늦은 lead, underflow, cancel은 자동
  재전송·자동 재개 없이 fail-closed
- uint32 millisecond tick wrap 보존

## 안전 상태

- firmware physical buffered execution: `false`
- ROS Action server connection: `false`
- transport execution connection: `false`
- motion authorized: `false`
- 기존 single-point runtime 변경: `없음`

## 완료 확인

- [x] 20 ms 재샘플링과 gripper 보존
- [x] 100 ms 초기 lead와 60/400 ms admission
- [x] 9+7 startup prime 후 START
- [x] watermark 10에서 target 16 refill
- [x] 80 ms outage 모형의 9+2 refill
- [x] ACK 불일치·재전송·late refill·underflow·cancel fail-closed
- [x] END·input complete·success 및 uint32 wrap
- [x] 전체 Python 회귀와 package rebuild
- [ ] firmware physical execution route 연결
- [ ] ROS `FollowJointTrajectory` multi-point runtime 연결
- [ ] 명시적 승인 후 제한 실기

세부 결과는
[Motion-5 host adapter 결과](../test-results/2026-08-03-motion5-buffered-host-adapter.md)에
기록한다.
