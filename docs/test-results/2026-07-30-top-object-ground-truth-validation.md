# Top 작업대 물체 실제 좌표 검증

## 판정

- 날짜: 2026-07-30
- 물체: 단단한 검은 펜
- 판정: **PASS — board-relative coarse perception**
- 합격 기준: 위치 최대 `10 mm`, 장축 yaw 최대 `5 deg`
- motion authorization: **계속 false**

이 검증은 출력한 20 mm tabletop validation grid와 검은 펜을 사용해
`top_board` 기준 `x/y/yaw`를 실제 배치와 대조했다. 로봇은 12 V OFF였고
이동 명령은 없었다.

## 시험 방법

1. 빈 작업대에서 보정 영역 내부 물체 후보가 0개인지 확인했다.
2. validation grid 원점을 raw pixel `(293.0, 310.5)`로 등록했다.
3. 등록 원점은 `top_board`의 `(103.053823, 103.560962) mm`에 해당했다.
4. 세 위치·각도에 펜을 수동 배치하고 각 위치에서 5프레임을 독립 검출했다.
5. 인쇄선은 threshold 후보가 되지 않았고 펜 footprint는 모두 보정 영역 내부였다.

검증지 배치와 펜 중심 맞춤 오차가 결과에 포함되므로, 아래 값은 검출기만의
오차가 아니라 사람이 배치하는 과정까지 포함한 보수적인 end-to-end 오차다.

## 결과

| 위치 | 검증지 기준 | 프레임 | 평균 위치 오차 | 최대 위치 오차 | 평균 yaw 오차 | 최대 yaw 오차 |
|---|---|---:|---:|---:|---:|---:|
| P1 | `(0, 0) mm`, `0 deg` | 5 | `5.016 mm` | `5.065 mm` | `0.447 deg` | `0.452 deg` |
| P2 | `(+20, 0) mm`, `45 deg` | 5 | `6.154 mm` | `6.196 mm` | `1.473 deg` | `1.478 deg` |
| P3 | `(0, -20) mm`, `90 deg` | 5 | `7.585 mm` | `7.603 mm` | `2.906 deg` | `2.911 deg` |

전체 15프레임 집계:

- 검출 성공: `15/15`
- 위치 평균/RMSE/최대: `6.251 / 6.340 / 7.603 mm`
- yaw 평균/RMSE/최대: `1.609 / 1.899 / 2.911 deg`
- 위치 반복 span 최대: `0.385 mm`
- yaw 반복 span 최대: `0.022 deg`

## 안전 경계와 다음 단계

`VIS-001`의 board-relative coarse perception gate는 통과했다. 그러나 현재
출력은 여전히 `top_board` 기준이며 base-frame robot target 계약은 열지 않았다.

- `motion_authorized: false`
- `robot_target_available: false`
- `base_registration_status: PROVISIONAL_RULER_MEASUREMENT`

다음 단계는 이 검출값을 바로 실행하지 않고, 고정된 Top-to-base 변환으로
base-frame shadow target을 만들고 MoveIt 계획과 비교하는 것이다. 실제 Pick
명령은 shadow target의 좌표·workspace·freshness gate를 통과한 뒤에만 별도
승인한다.

## 증거

- 수치 원본: `evidence/2026-07-30-top-object-ground-truth-summary.json`
- registration image SHA-256: `d483ab466d419e9905437a1df72b16bb4e8739329523627470d489b5ac7f6ab6`
- P1 representative SHA-256: `dd53b94e7416d9e14974ae1d7611579df344ead859cd94a9967e98d89308a2b8`
- P2 representative SHA-256: `08f36c2427380df9eafd2728eef51d5d0bbbb005f2d589e46ebb1aa32f721a86`
- P3 representative SHA-256: `6a041f572a1989557d7fcd1aac930f9a58dafb892ccd643ca9c7c795447428cf`
