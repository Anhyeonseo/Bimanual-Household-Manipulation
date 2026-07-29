# 제한적 안쪽 방향 복구 정책

- 날짜: 2026-07-28
- 상태: 제한적 복구 safety path 실기 PASS / trajectory 완료 기준은 final error 25 raw로 미통과
- 대상: 왼팔 `FollowJointTrajectory` Action adapter

## 발생 조건

Top–base 등록점 2 자세에서 `left_elbow_joint` 피드백이 strict 최대값
`2048 raw`보다 `21 raw` 바깥인 `2069 raw`로 측정됐다. 기존 host 정책은
`21..40 raw` 잔차에서 all-zero q0, 정확히 `2000 ms` 목표만 허용하므로,
등록점 3 목표는 Action server에서 전송 전에 거절됐다. 두 거절 시도 모두
실제 servo 전송과 관절 이동은 없었다.

## 변경한 안전 계약

피드백이 firmware clear-stop recovery envelope 안이지만 strict 범위를
`20 raw`보다 많이 벗어난 경우, 다음 조건을 모두 만족하는 단일 목표만
허용한다.

- 실행 시간은 정확히 `2000 ms`
- 각 팔 관절의 현재값 대비 목표 이동은 최대 `64 raw`
- 모든 팔 목표는 strict 양쪽 경계에서 최소 `20 raw` 안쪽
- 범위를 벗어난 관절은 반드시 strict 안쪽 방향으로 이동
- gripper가 `20 raw`보다 많이 벗어나면 arm recovery로 우회하지 않고 거절
- 다중 waypoint, 자동 재시도, clamp와 임의 목표는 계속 금지

## 검증

- 현재 사례 raw `(2279, 2051, 2069, 2048, 2054, 1959)`에서
  `[0.45, 0.10, 0.05, 0.05, 0.05] rad`, `2000 ms` 목표 수락
- 경계에 너무 가까운 목표 거절
- `2000 ms`가 아닌 목표 거절
- `64 raw` 초과 이동 거절
- out-of-range gripper 상태의 arm recovery 거절
- 기존 cancel, SAFE_STOP, backend exclusivity와 gripper 회귀 포함
  `81 passed`

## 단 1회 실기 결과

- 등록점 3 목표는 새 bounded inward recovery gate에서 수락됐다.
- 명령은 정확히 1회 전송됐고 자동 재시도하지 않았다.
- MoveIt은 `2.7 s` upper bound에서 timeout 후 cancel을 요청했다.
- bridge는 `final error 25 raw`를 보고하고 SAFE_STOP을 latched했다.
- 사용자 물리 점검 뒤 `/clear_fault`를 정확히 1회 호출했다.
- elbow feedback은 `-0.0322135965 rad`에서 `+0.0122718463 rad`로
  이동해 strict command range 안으로 복귀했다.
- 실제 피드백과 Top marker를 사용한 등록점 V3는 검출 PASS다.
- 따라서 안쪽 복구와 fail-closed 정지는 실기 PASS지만, 일반 trajectory
  완료 기준 `20 raw`는 별도 보강이 필요하다.

## 남은 게이트

복구 목표를 반복하지 않는다. 다음 등록 자세는 현재 strict 안의 실측값을
기준으로 final error가 큰 elbow 목표를 무리하게 요구하지 않도록 선정한다.
MoveIt execution upper bound와 firmware/servo의 `20 raw` 완료 기준 조정은
등록용 저위험 자세 수집과 분리해 검토한다.
