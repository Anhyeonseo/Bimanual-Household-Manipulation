# Motion-6 buffered extended status mapping

## 목적

Motion-5 scheduler와 firmware 공통 C core의 32-byte extended
`SETPOINT_STATUS` 계약을 host-only로 연결한다. 실제 serial send, firmware
physical route와 ROS Action runtime은 계속 연결하지 않는다.

## 수용 조건

- admission ACK: status 0, sample count, first apply tick, calibration hash 일치
- PRIMING/START/RUNNING executor state 전이 일치
- accepted/applied/queued 합계와 queue result 일치
- terminal status: status 6과 C enum의 state/reason 조합 일치
- SUCCEEDED는 전체 sample accepted/applied, queue 0, safe-stop false
- planned HOLD만 safe-stop false, underflow/missed tick/cancel/abort는 true
- timeout·disconnect·legacy/missing extended field는 pending 폐기 후 abort
- 자동 재전송·자동 resume 없음

## 완료 확인

- [x] extended admission ACK 정상 전이
- [x] status/sample/apply tick/hash/state/safe-stop mismatch 거부
- [x] timeout 시 pending batch 폐기
- [x] success terminal 전체 적용 확인
- [x] underflow terminal queue clear와 safe-stop 확인
- [x] terminal state/reason/safe-stop 불일치 거부
- [x] 전체 Python 회귀와 package rebuild
- [ ] 실제 transport execution method 연결
- [ ] firmware physical route 연결
- [ ] ROS Action runtime 연결

세부 결과는
[Motion-6 결과](../test-results/2026-08-03-motion6-buffered-status-mapping.md)에
기록한다.
