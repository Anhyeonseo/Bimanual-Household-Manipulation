# F8.1 resident 양팔 실측 feedback 수락 결과

- 날짜: 2026-08-14
- firmware: `0x00024800 / protocol 2 / 12 joints / 0xEFFFFFFF`
- HEX SHA-256:
  `594bbcf8bb4ce69a29ec08cbfb07f0ec15b6cc36e7b871b6a29e23db2ea1e08b`
- 결론: **PASS — 상단 애플리케이션 ROS 경계 고정 가능**

## 변경 범위

F8 tracking safety read를 바꾸지 않고, 같은 실측 관절 pair를 별도 12축 cache에
보존했다. protocol-v2에 `GET_FEEDBACK_SNAPSHOT=61`,
`FEEDBACK_SNAPSHOT=62`를 추가하고 Pi resident node가 다음을 발행한다.

- `/bimanual_stream_adapter/joint_states`
- `/bimanual_stream_adapter/feedback`
- 위치 12개, 관절별 sample age, full present mask, firmware tick,
  completed pair count

기존 `0x00024703` HEX는 byte-identical SHA
`a4b5bb23a257f86a57ed724366a0bb61107260cba65f9f175ca68cfe4b672e87`를
유지했다.

## 로컬 gate

- 전체 변경 직후 Python suite: `1356 passed`
- 최종 문서/adapter/F8.1 표적 회귀: `31 passed`
- host-native feedback cache test와 Cortex-M4 Release build 통과
- command/feedback ROS interface SHA를 상단 계약 문서에 고정

## 실기 gate

### Direct no-output

```text
F81_BIMANUAL_FEEDBACK_NO_OUTPUT_PASS
firmware=0x00024800
present_mask=0xFFF
sample_age_ms=[57,57,57,57,57,57,57,57,57,57,57,57]
launches=0
```

- artifact:
  `artifacts/f81/2026-08-14/feedback_no_output_run01.json`
- SHA-256:
  `18c520f293835195a948ea63524873287006ab1e6be74ecb99306bb051bae6e6`

### ROS resident no-motion

```text
RESIDENT_BIMANUAL_ADAPTER_NO_MOTION_PASS joints=12 state=ready
```

- artifact:
  `artifacts/resident_adapter/2026-08-14/no_motion_24800_run01.json`
- SHA-256:
  `971d4876dda7443ff32fe82b3bda9d11055a48ee6067468210f1d250101785a8`

첫 실행은 모든 검증 뒤 evidence JSON을 저장할 때 ROS `uint32` 객체를 Python
JSON encoder가 처리하지 못해 실패했다. feedback position/count/age/mask/tick을
명시적으로 내장 `float`/`int`로 변환했고, 다음 actual rolling evidence 경로의
동일 지점도 함께 수정했다. firmware나 motion failure가 아니다.

### ROS resident actual rolling

```text
RESIDENT_BIMANUAL_ROLLING_BASE_SMALL_ROUNDTRIP_ONCE_PASS
delta_rad=0.030000
feedback_samples=8
feedback_age_max_ms=27
commands=7
epochs=1,1,2,2,2,2,2
state=stopped
```

- artifact:
  `artifacts/resident_adapter/2026-08-14/rolling_base_feedback_24800_run01.json`
- SHA-256:
  `055d79fef4b0590f439fdd5943e2a024ab7e85e839d7e9ddc4221d14257ba6ec`

양팔 base `+0.03 rad` 왕복에서 START_OPEN, APPEND, SPLICE, keepalive APPEND와
STOP이 실제 실측 feedback과 함께 통과했다. 최대 sample age는 상단 계약의
active freshness gate `150 ms`보다 충분히 작았다.

## 남은 범위

Firmware/ROS stream 경계는 완료했다. 다음은 firmware source와 무관한 상단
작업이다.

- MoveIt/FSM/pretrained-policy command arbiter 및 adapter
- 양팔 URDF/base transform 정밀화와 inter-arm collision
- policy deployment bundle 및 camera/observation freshness
- task-level 반복성, fault injection과 운영 runbook

상단 구현 계약은
[`docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md`](../BIMANUAL_UPPER_APPLICATION_INTERFACE.md),
개발 인계 프롬프트는
[`docs/prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md`](../prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md)에
고정한다.
