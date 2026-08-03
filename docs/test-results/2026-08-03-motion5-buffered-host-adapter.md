# Motion-5 buffered host Action adapter 후보 결과

## 결론

실측 timing 정책 `20 ms / 60..400 ms / 16→10→16`을 사용하는 순수 host
스케줄러를 구현했다. MoveIt 다중점 경로를 20 ms로 재샘플링하고 초기 queue를
9+7 sample로 prime한 뒤, watermark 도달 시 여러 frame을 사용해 target 16까지
보충한다.

첫 wire sample은 fresh feedback와 start tolerance를 통과한 `t=0` 자세다.
firmware는 이를 첫 apply tick보다 20 ms 앞선 anchor에도 동일하게 사용하므로,
실행 직전 blocking 6축 read sweep 없이 시작 자세를 유지한 채 연속 보간할 수
있다.

이 모듈은 ROS, serial, firmware execution과 연결되지 않았고 기존
single-point runtime도 변경하지 않았다. 따라서 `motion_authorized=false`다.

## 검증

- adapter 집중 mock: `12 passed`
- adapter·trajectory contract·timing·wire protocol 집중 회귀: `54 passed`
- ROS Jazzy overlay 포함 전체 Python 회귀: `473 passed`
- `single_arm_bridge` symlink-install rebuild: `1 package finished`
- 설치 산출물 import: `PASS`
- Pi 전송, serial 접근, firmware 변경, reset, robot motion: `0`

## 장애 계약

- pending frame은 다시 반환하지 않는다.
- rejected/mismatched ACK는 terminal ABORTED다.
- 다음 sample lead가 60 ms 미만이면 frame 생성 전에 중단한다.
- 80 ms outage 모형에서 queue 5부터 9+2 sample로 target 16까지 복구한다.
- input이 남았는데 applied가 accepted를 따라잡으면 underflow로 중단한다.
- cancel은 safe-stop-required terminal이며 자동 resume이 없다.

## 다음 gate

1. 별도 firmware 변경으로 validation-only와 physical execution route 분리
2. mock transport에서 실제 extended ACK/terminal mapping 연결
3. ROS `FollowJointTrajectory` multi-point runtime 연결
4. 무동작 fault injection 뒤 명시적 승인 하의 단일 관절 제한 실기
