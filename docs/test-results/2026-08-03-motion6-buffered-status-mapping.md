# Motion-6 buffered extended status mapping 결과

## 결론

host scheduler가 protocol `MotionResult`의 extended admission ACK와 terminal
diagnostics를 fail-closed로 해석하도록 연결했다. calibration, sample 수,
apply tick, executor state, terminal reason, safe-stop, queue accounting 중 하나라도
다르면 pending frame을 폐기하고 자동 재전송 없이 abort한다.

firmware 0x00021900은 여전히 validation-only이며 실제 serial buffered send와
ROS Action runtime은 미연결이다. `motion_authorized=false`를 유지한다.

## 검증

- buffered host adapter·status mock: `22 passed`
- ROS Jazzy overlay 포함 전체 Python 회귀: `483 passed`
- `single_arm_bridge` symlink-install rebuild: `1 package finished`
- Pi 전송, serial 접근, firmware 변경, reset, robot motion: `0`

## 다음 gate

1. 실행 기능 없이 mock transport driver로 frame/response 순서 fault injection
2. 별도 firmware identity에서 physical execution route 연결
3. READ_ONLY identity·capability와 무동작 terminal 검증
4. ROS Action 연결 후 명시적 승인 하의 단일 관절 제한 실기
