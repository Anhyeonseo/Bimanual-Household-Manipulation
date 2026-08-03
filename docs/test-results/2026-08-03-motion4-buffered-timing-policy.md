# Motion-4 Pi–VCP buffered timing 실측·운영 입력 결과

## 결론

STM32 `0x00021900`의 validation-only route를 `SAFE_DISABLED/READ_ONLY`,
6축 torque OFF 상태에서 측정했다. 100/80/60 ms 첫 sample lead와 400 ms
maximum horizon은 각각 1000회 오류 없이 통과했다. 40 ms 첫 lead는
`status=1`, `detail=9(queue rejected)`로 즉시 거부됐고 자동 재시도는 없었다.

별도 reviewed derivation으로 다음 배포 입력을 채택했다.

- sample period: `20 ms`
- minimum/maximum lead: `60/400 ms`
- startup prime depth: `16 samples`
- low watermark: `10 samples`
- refill target: `16 samples`
- motion authority: `false`

## 실측 envelope

- 성공 capture: 4개, 각 series `1000 samples`
- serial RTT worst p95/p99: `17.428593/17.533277 ms`
- host dispatch jitter worst p95: `0.062925 ms`
- delivery lateness worst p95: `0.0 ms`
- 주입 host outage 최대: `80.064074 ms`
- transport error: `0`

정책 SHA-256:
`362e09c91e696ca587963c664f26cc49e06a44d205e2b5d6daf186c63e1fd8f2`

정책 파일:
`artifacts/motion/2026-08-03/buffered_timing_policy_reviewed.json`

## Watermark 보정

초기 `9/7/16` 후보는 outage와 RTT만 반영하고, 복구 뒤 새 sample도 최소
60 ms 앞서 있어야 한다는 admission 조건을 누락했다. 계약 반영 전에 로컬
queue 모델이 이를 검출했다.

최종 복구 예산은 `80.064074 ms outage + 17.533277 ms RTT p99 + 20 ms host
scheduler guard + 60 ms minimum lead = 177.597351 ms`다. 20 ms sample 9개가
필요하며 한 개 guard를 더해 low watermark를 10으로 정했다. 시작 시 queue
16개를 prime하고 refill target도 16으로 유지한다.

## Fail-closed 확인

- 각 analyzer 결과는 `MEASURED_DEPLOYMENT_INPUT`이다.
- 개별 analyzer는 운영값을 자동 승인하지 않는다.
- 별도 derivation 도구가 capture와 analysis의 완전 일치 및 SHA를 확인한다.
- 40 ms 거부는 재전송하지 않고 sweep 전체를 중단했다.
- outage 뒤 유효 tick gap 없이 2-batch refill하는 host queue 모델이 PASS했다.
- refill이 없으면 underflow가 HOLD·safe-stop-required로 끝난다.
- contract와 policy 모두 `motion_authorized=false`다.

## 로컬 검증

- timing·capture·queue 관련 집중 회귀: `38 passed`
- ROS Jazzy와 workspace overlay를 포함한 전체 Python 회귀: `473 passed`
- `single_arm_bridge` symlink-install rebuild: `1 package finished`
- serial 접근, Pi 전송, firmware 변경, reset과 로봇 이동: `0`

## 남은 gate

1. 별도 firmware 변경에서 validation-only route와 물리 execution route를 분리
2. 구현된 host-only prime/refill/cancel 스케줄러를 ROS Action에 연결
3. mock 완료 뒤 plan-only 및 무동작 terminal fault injection
4. 명시적 승인 후 단일 관절 제한 실기
