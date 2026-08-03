# Motion-9 FollowJointTrajectory buffered Action 물리 검증 결과

## 결론

`0x00022100`의 다중점 `FollowJointTrajectory`를 20 ms buffered queue로
전송하는 Motion-9 경로를 Pi에 배포하고, 소형 다중 관절 왕복 동작 1회를
실물에서 통과했다. Action 전송은 1회였고 자동 재시도는 없었다. firmware
terminal, post-settle, 독립 READ_ONLY 위치 확인과 6축 physical DISABLE까지
모두 통과했다.

이 결과는 **buffered Action 물리 경로가 배포·검증됐음**을 뜻한다. 임의
Pick/Place나 자율 정책 실행 권한을 뜻하지 않으므로 host 계약의
`motion_authorized=false`는 유지한다.

## 검증 대상

- firmware: `0x00022100`
- calibration: `0x8AD27897`
- capabilities: `0x00000FFF`
- plan SHA-256:
  `d5378b6c0eb5eb4069e79e609ee12efb14750d228b61b009d29555fb573f47f8`
- one-shot sender SHA-256:
  `d66f26f7b3907fda1988895a01e657bafa902ea901396a2c38f8524f16e93671`
- execution log SHA-256:
  `80f14845bab532de3217fcee7a9c4c2b0b5cf4241b65023844d6ba7d615de087`
- anchor raw: `2070 / 2253 / 1785 / 1981 / 2064 / 2002`
- 왕복 변위: Base `+0.015 rad`, Shoulder `+0.015 rad`,
  Wrist Roll `+0.030 rad`
- duration/sample: `1200 ms / 61 samples`
- admission batch: `9 / 7 / 6 / 6 / 6 / 6 / 6 / 6 / 6 / 3`

## 실행 경로

`FollowJointTrajectory`
→ 다중점·관절 제한·fresh-start 검증
→ 20 ms 위치 재표본화
→ 16 sample prime
→ watermark 10 / refill 16
→ one-shot buffered transport
→ extended firmware terminal
→ 6축 post-settle 진단 2회
→ ROS Action terminal
→ bridge 정상 종료와 physical DISABLE
→ 별도 READ_ONLY 6축 위치 확인

## 물리 실행 결과

- plan gate: PASS
- fresh-start 최대 오차: `0.009204 rad`
- Action goal 전송: `1회`
- Action terminal: `status=4`, `error_code=0`
- firmware terminal: `state=succeeded`, `status=6`
- accepted/applied: `61 / 61` 전체 경로 완료
- 최대 apply lateness: `4 ms` (`0..5 ms` 계약 이내)
- firmware post-settle 최대 오차: `6 raw`
- 자동 재시도: `0회`
- 비정상 소음·진동: 없음

독립 READ_ONLY 확인에서 전 축 torque OFF였고 anchor 대비 최종 raw 오차는
Base `+5`, Shoulder `+2`, Elbow `0`, Wrist Flex `0`, Wrist Roll `+6`,
Gripper `0`이었다. 최대 오차는 `6 raw`로 독립 허용치 `30 raw` 이내였으며
6축 physical DISABLE도 통과했다.

## 안전·계약 판정

- `physical_execution_deployed=true`
- `physical_execution_candidate.deployed=true`
- `motion_authorized=false`
- terminal만으로 성공하지 않고 post-settle과 독립 readback을 함께 요구
- timeout, ACK/sequence 불일치, underflow, cancel과 settle 실패는
  무재전송 fail-closed 유지
- gripper는 arm 경로에서 보존하며 이번 검증에서 별도 동작하지 않음

계약 상태가 물리 검증 전 후보에서 `PHYSICAL_ACTION_COMMISSIONED`로
승격됐기 때문에, 이전 계약 SHA를 담은 plan artifact는 증거로만 보존한다.
후속 실행은 현재 계약으로 새 plan을 생성하고 새 SHA 승인을 받아야 한다.

## 로컬 정합성 검증

- contract/plan/sender 관련 집중 회귀: `61 passed, 1 skipped`
- ROS Jazzy 환경 전체 Python 회귀: `539 passed`
- `single_arm_bridge` symlink rebuild: PASS
- `single_arm_bridge` package identity test: PASS
- install 계약 readback:
  `PHYSICAL_ACTION_COMMISSIONED / deployed=true / motion_authorized=false`

## 다음 gate

1. 현재 계약으로 q0 소형 왕복 연속경로 검증
2. Pick pregrasp 연속경로 검증
3. grasp/lift/place/retreat/q0를 정착 대기 없는 연속 구간으로 확대
4. 10회 pilot 뒤 50회 반복성 benchmark

Git 작업은 사용자가 직접 수행한다.
