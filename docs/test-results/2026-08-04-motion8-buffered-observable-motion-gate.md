# Motion-8 0x221 observable buffered motion gate 결과

## 결론

`0x00022100` bounded apply-lateness firmware의 단일 관절 실기에서
16개 sample이 모두 적용됐고 실제 Wrist Roll 위치 변화도 독립 READ_ONLY
진단으로 확인했다. 이어진 5 raw 복귀 시험에서는 기존 30 raw 정착
허용치가 무동작도 PASS할 수 있음을 확인하여 commissioning 전용
observable-motion gate를 로컬 구현했다.

## 0x221 실기 증거

- firmware: `0x00022100`, calibration: `0x8AD27897`
- queue: `peak=16`, `accepted=16`, `applied=16`
- terminal: `SUCCEEDED`, reason `0`, safe stop `false`
- maximum apply lateness: `1 ms`
- 최초 Wrist Roll 기준: `2043 raw`
- terminal 후 정착 위치: `2048 raw`
- 독립 READ_ONLY 위치: `2048 raw`
- 독립 측정 drift: `0 raw`
- physical DISABLE 6축 readback: PASS
- 실제 미세 움직임 관찰, 비정상 소음·진동 없음

## 기존 판정의 false-positive

- 복귀 명령: `-0.00767 rad`, 계획 변위 약 `-5 raw`
- terminal: `accepted=16`, `applied=16`, maximum lateness `1 ms`
- 정착 위치: `2048 raw`로 변화 없음
- 목표 오차 `5 raw`가 기존 허용치 `30 raw` 안이어서 도구는 PASS
- 결론: firmware setpoint 적용 성공과 실제 물리 추종 성공을 분리해야 함

## 로컬 observable-motion gate

- 계획 변위가 `16 raw` 미만이면 ARM/ENABLE 전에 거부
- 선택 축이 명령 방향으로 실제 `10 raw` 이상 이동해야 함
- 선택 축 최종 목표 오차 `8 raw` 이하
- 나머지 축 목표 오차 `30 raw` 이하
- 위 조건을 6축 진단 2회 연속 만족해야 physical PASS
- firmware terminal PASS와 physical motion PASS를 별도 출력
- 조건 미충족은 SAFE_STOP·physical DISABLE, 자동 재시도 없음

## 로컬 검증

- 정상 방향 추종 PASS
- 무동작 거부
- 역방향 이동 거부
- 최소 계획 변위 `15/16 raw` 경계 검증
- 선택 축 목표 오차 `8/9 raw` 경계 검증
- 나머지 축 `31 raw` 이탈 거부
- focused regression: `44 passed`
- Python/ROS 전체 regression: `533 passed`
- `single_arm_bridge` symlink-install rebuild: PASS

## 상태

- firmware 재변경: 불필요
- 새 commissioning host 도구 Pi 배포: 완료
- observable `+0.03 rad` 실기: PASS
  - 계획: `2048 -> 2068 raw`, `20 raw`
  - 실측: `2048 -> 2064 raw`, 명령 방향 `16 raw`
  - 목표 오차: `4 raw`, 기타 축 최대 오차: `0 raw`
  - `accepted=16 / applied=16 / maximum apply lateness=2 ms`
  - physical DISABLE: PASS
  - 실제 움직임 명확, 비정상 소음·진동 없음
- ROS Action runtime: 로컬 연결 완료, Pi 미배포
- 자동 재시도: 금지
