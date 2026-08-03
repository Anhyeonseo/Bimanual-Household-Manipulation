# Motion-7 buffered mock transport driver

## 목적

Motion-5 scheduler와 Motion-6 extended status mapping 사이에 one-shot
frame/response 교환 순서를 고정한다. 실제 serial transport method는 연결하지
않으며 mock port만 사용한다.

## 계약

- scheduler batch를 기존 binary payload codec으로 encode
- `CANDIDATE|BEGIN/START/END`, `VALIDATION_ONLY` 미포함
- 각 batch identity는 정확히 한 번만 exchange
- outer frame sequence와 payload request sequence 일치 필수
- 32-byte extended result만 admission ACK로 수용
- timeout·exchange exception·sequence mismatch·legacy result 즉시 abort
- terminal-before-ACK는 pending 폐기 후 abort
- terminal 상태에서 service 재호출 시 port를 다시 호출하지 않음

## 완료 확인

- [x] 9+7 prime frame encode와 순서
- [x] watermark refill frame이 prime frame을 재사용하지 않음
- [x] timeout 후 exchange 횟수 1 고정
- [x] response sequence mismatch 거부
- [x] legacy 16-byte result 거부
- [x] terminal-before-ACK 거부
- [x] binary header first tick/count/arm-mask/reserved 확인
- [ ] 실제 `ActuatorTransport` execution method 연결
- [ ] firmware physical route 연결
- [ ] ROS Action runtime 연결

세부 결과는
[Motion-7 결과](../test-results/2026-08-03-motion7-buffered-mock-transport.md)에
기록한다.
