# Motion-7 buffered mock transport driver 결과

## 결론

buffered scheduler가 만든 batch를 기존 wire codec으로 encode하고, mock port에
단 한 번 전달한 뒤 matching extended response만 수용하는 driver를 구현했다.
timeout과 malformed/out-of-order 응답은 재전송 없이 fail-closed다.

실제 `ActuatorTransport`, serial, firmware physical route, ROS Action에는 연결하지
않았고 `motion_authorized=false`다.

## 검증

- scheduler+mock transport 집중 회귀: `29 passed`
- ROS Jazzy overlay 포함 전체 Python 회귀: `490 passed`
- `single_arm_bridge` symlink-install rebuild: `1 package finished`
- timeout 후 port exchange count: `1`
- sequence mismatch/legacy response/terminal-before-ACK 추가 exchange: `0`
- Pi 전송, serial 접근, firmware 변경, reset, robot motion: `0`

## 다음 gate

1. firmware physical route source를 별도 identity/capability로 구현
2. host `ActuatorTransport` 실행 method는 새 capability 없으면 fail-closed
3. C fault injection과 cross-build
4. Pi 배포·flash 전 별도 명시 승인
