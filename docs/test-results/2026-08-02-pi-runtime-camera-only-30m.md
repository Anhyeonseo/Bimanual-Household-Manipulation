# Pi 5 3카메라·STM32 READ_ONLY 30분 자원 기준선

- 날짜: 2026-08-02 KST
- 대상: Raspberry Pi 5 4GB, USB MJPEG 카메라 3대, STM32G474 왼팔
- firmware: `0x00021800`
- camera phase: `RUNTIME_BASELINE`
- bridge: `READ_ONLY`, joint feedback 5 Hz
- 측정: warm-up 10초 + 본시험 1,800초
- policy/detector/MoveIt: 미실행
- 원본 artifact SHA-256:
  `88bfe6b7f6cdf6b01b677c1ba18bbd73da807a3430afca0cea484534b16484a7`

## 목적과 안전 경계

Top·양 손목 카메라의 선택 decode와 RGB DDS 전달이 30분 동안 Pi 자원과
STM32 heartbeat/feedback을 방해하지 않는지 확인했다. 측정 도구는
`/camera_phase`만 발행했고 팔·그리퍼 명령 topic을 만들지 않았다.
`motion_authorized=false`, robot command publisher 수는 0이다.

## 카메라 결과

| 카메라 | 목표/실측 decode | RGB DDS payload | decode 실패 | reconnect | 진단 frame age p95 | decode p95 |
|---|---:|---:|---:|---:|---:|---:|
| Top | 6.00/5.907 Hz | 43.55 Mbps | 0 | 0 | 21.20 ms | 3.09 ms |
| Wrist A | 5.00/4.937 Hz | 36.40 Mbps | 0 | 0 | 33.39 ms | 2.90 ms |
| Wrist B | 5.00/4.787 Hz | 35.29 Mbps | 0 | 0 | 28.08 ms | 4.83 ms |

세 RGB topic의 application payload 합계는 약 115.25 Mbps다. 세 카메라 모두
`STREAMING`, diagnostic level 0을 유지했다. subscriber frame age p95의
최댓값은 Wrist A의 38.95 ms, 단일 frame 최댓값은 46.48 ms였다.

## STM32 제어 격리

| 지표 | 결과 |
|---|---:|
| `/joint_states` 수신 | 9,001개 |
| 실측 rate | 5.00049 Hz |
| feedback interval p95 | 202.85 ms |
| feedback interval 최대 | 206.41 ms |
| heartbeat 오류·지연 로그 | 0회 |
| feedback 오류·지연 로그 | 0회 |
| safety-latch 오류 로그 | 0회 |

`/rosout`의 `single_arm_bridge` transport 이벤트와
`/joint_states` 연속성을 함께 관측했다.

## Raspberry Pi 자원

| 지표 | 결과 | 기준 | 판정 |
|---|---:|---:|---|
| CPU 평균/p95 | 7.94% / 10.34% | 평균 70% 이하 | 통과 |
| CPU 1초 최대 | 34.98% | 90% 미만 | 통과 |
| memory 사용 최대 | 644.21 MB | 3,000 MB 이하 | 통과 |
| memory 가용 최소 | 3,339.75 MB | 700 MB 이상 | 통과 |
| CPU 온도 평균/최대 | 33.40°C / 40.80°C | 80°C 미만 | 통과 |
| swap in/out delta | 0/0 | 0/0 | 통과 |
| throttling flags OR | `0x00000000` | 0 | 통과 |
| camera manager RSS 평균/최대 | 26.17/26.60 MB | 기록 | 통과 |
| single arm bridge RSS 평균/최대 | 60.84/61.27 MB | 기록 | 통과 |

## 판정과 남은 범위

- 3카메라 decode·DDS + STM32 READ_ONLY 30분 하위 gate: **통과**
- robot command publication, camera reconnect, decode 실패, bridge
  heartbeat/feedback 오류, swap, throttling: 모두 **0**
- 실제 detector, MoveIt planning, ONNX policy shadow를 포함하지 않았으므로
  `RES-001`과 `RES-002` 전체 상태는 **부분 통과**
- 다음 gate는 고정된 deployment manifest의 ONNX policy와 detector를
  `command_publications=0` shadow mode로 추가해 같은 측정을 반복하는 것이다.
- 이후 8시간 soak와 headless 재부팅 반복 시험을 수행한다.

## 증거

- `artifacts/stage9/2026-08-02/pi_runtime_camera_only_30m.json`
- `tools/pi_runtime_resource_baseline.py`
- `config/camera_schedule.json`
- `config/policy_shadow_diagnostics_contract.json`

