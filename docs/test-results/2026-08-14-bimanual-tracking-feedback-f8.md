# 2026-08-14 F8 양팔 실시간 tracking feedback 후보

## 목적

`0x00024700`은 이미 검증된 F7 양팔 5 ms DMA dispatch 위에 실행 중 위치
피드백을 연결한다. 경로 생성원이 MoveIt, task FSM, 학습 정책 중 무엇인지는
관계없으며 firmware는 동일한 protocol-v2 12축 stream 계약만 처리한다.

이 단계는 새로운 관절 범위 검증이 아니다. 작업자가 승인한
`config/bimanual_operational_limits.json`을 그대로 사용하면서, 실제 서보가
명령을 따라오지 못하는 경우 다음 출력을 막고 양팔을 함께 torque-off하는
실행 무결성 단계다.

## 구현

- 각 5 ms paired DMA dispatch 완료 후 한 관절 번호를 선택한다.
- 왼팔 USART1과 오른팔 UART4에 같은 관절의 present-position READ를 동시에
  비동기로 시작한다.
- 오른팔 UART4 RX는 DMA1 Channel5의 256-byte circular ring을 사용한다.
- 응답을 기다리는 동안 blocking receive를 사용하지 않는다.
- 요청 시점의 좌우 명령값을 함께 보존하므로, 늦게 도착한 측정값을 더 새로운
  명령과 잘못 비교하지 않는다.
- 6개 관절을 순환하므로 정상 200 Hz dispatch에서는 팔별 전체 sweep가 약
  30 ms다.
- 어느 한쪽 응답 실패, 4 ms timeout, unwrap/변환 실패, 관절별 tracking-error
  초과는 다음 5 ms 출력을 보내기 전에 executor를 abort하고 양팔 torque-off와
  stop latch로 수렴한다.
- 마지막 planned-horizon 출력의 지연 피드백도 판정한다.
- 별도 tracking diagnostics는 pair 요청/완료/실패, 최대 응답 지연과 12축 최대
  tracking error를 반환한다.

## 후보 identity

- firmware: `0x00024700`
- protocol: `2`
- joints: `12`
- host baud: `921600`
- capabilities: `0xEFFFFFFF`
- calibration: left/right `0x2D90167E`
- HEX: `build/stm32_g474_single_arm-bimanual-tracking-release/stm32_g474_single_arm.hex`
- HEX SHA256: `c7e22af34c19d5643ef51838c94f91d59d476656fc1a9bcfd2fcf85f87eca80e`
- no-output tool SHA256: `8d34395293d11b570a64100a9082af5cb29b77bac37035fef10f4c90eeb9543a`
- hold tool SHA256: `b1b26f8f807cb758f72eb0cde077235f6f89faa21fb9e3cf73bfd9d00d1b4d3f`

## 로컬 검증

- Cortex-M4 Release build: PASS
- actuator-core CTest: `9/9 PASS`
- F8/F7/protocol 선택 Python: `40 PASS`
- 전체 Python: `1328 PASS`
  - ROS overlay 미소싱 상태에서는 MoveIt package lookup 4개가 실패했으나,
    `ros2_ws/install/setup.bash` 소싱 후 해당 4개도 통과했다.
- F7 실기 기준 HEX 재현 SHA256:
  `afc9a9afcd5175c1e32fadb578c0c7035e090de10a378de7bc02de9b9fc4e88f`
  (기존 검증본과 byte-identical)

## 정상 F8 실기 결과

- no-output: firmware `0x00024700`, `launches=0`, 양팔 12축 anchor와
  tracking diagnostics inactive/zero 확인, artifact SHA256
  `bb26f675365f82f7a5a180b163d798ab214acdca7780fa23b259c4688766972c`
- zero-delta current-pose hold: dispatch `35/35`, feedback pair `35/35`,
  최대 reply latency `2 ms`, 최대 tracking error `0 urad`, 최대 좌우 시작
  시차 `6 us`, 최대 launch lateness `49 us`, 종료 verified torque-off,
  artifact SHA256
  `ad1c6aad9e887742435331c641b7582d9a4e8f221c399fa3c30cec96f8e6f21e`

## tracking-error fault 후보

- firmware: `0x00024701`
- trigger: 정상 paired feedback 8회 완료 뒤 오른팔 joint 7의 측정값을
  요청 시점 command보다 `100000 urad` 크게 한 번 주입
- commanded motion delta: `0`
- HEX SHA256:
  `21bd7e796d834178f948b89b5c1a6d500c817ffac57da9053d775703158878e0`
- tool SHA256:
  `f6008a6b1b6bf34bba86b9e5c11f985b691403b04e2b70002d7c8e4c0d56de78`
- 로컬 CTest `9/9`, 전체 Python `1329 PASS`
- 정상 `0x00024700` HEX SHA는 계속
  `c7e22af34c19d5643ef51838c94f91d59d476656fc1a9bcfd2fcf85f87eca80e`

## tracking-error fault 실기 결과

`0x00024701`에서 정확히 8개의 paired dispatch와 8개의 paired feedback 뒤
오른팔 global joint 7에 `100000 urad` tracking error가 주입되었다. executor는
`ABORTED/TRACKING_ERROR`로 종료했고 다음 출력은 시작되지 않았다. 좌우 12개
서보의 torque-enable register readback이 모두 0이었고 최대 feedback reply
latency는 `2 ms`였다. artifact SHA256은
`365c7e66b796c736757f5b560f65437f9c7fe778cb052caadefe99629c860937`다.

시험 뒤 정상 `0x00024700`으로 복구했고 no-output에서 동일한 12축
raw/unwrapped anchor와 `launches=0`을 확인했다. 복구 artifact SHA256은
`e400b2f1b1e7c7e2ee51ca4dc01fcd2e00e7f1eda8e9ae8c54baa76c7cfafb47`다.

따라서 F8 route-time tracking feedback gate는 정상·fault·복구 경로 모두
실기 통과다. 장시간 open-stream 정책 공급은 상위 Pi 실행기 통합 단계의 별도
gate로 남긴다.
