# Top–base registration session04 재감사

- 날짜: 2026-07-28
- 입력: `evidence/2026-07-28-top-base-session04-rejected.yaml`
- 마커: 강체 고정 노란 정사각형, 실측 `11.57 x 11.57 mm`
- 상태: REJECTED / motion authorization 유지 금지

## 기존 중심점 등록

다섯 PASS 프레임을 높이 보정된 단일 중심점으로 계산한 결과는 다음과 같다.

- robot FK 최대 XY span: `37.660 mm`
- camera board-plane 최대 span: `17.814 mm`
- span ratio: `0.4730`
- rigid fit RMS: `10.972 mm`
- rigid fit max: `17.208 mm`

따라서 점 개수는 충족했지만 강체 기하 조건을 통과하지 못했다.

## 11.57 mm 사각형 4-코너 PnP 재감사

저장된 다섯 원본 프레임에서 노란 윤곽은 모두 완전한 4각형으로 검출됐다.
정사각형 PnP의 프레임별 재투영 오차는 `0.134..0.495 px`여서 영상 검출
자체는 정상이다. 하지만 PnP 중심과 URDF FK 중심을 3D 강체 정합한 결과:

- rigid fit RMS: `14.240 mm`
- rigid fit max: `17.954 mm`
- pairwise distance ratio: `0.291..2.098`

카메라 중심 검출을 4-코너 방식으로 바꿔도 URDF FK와 하나의 강체 변환으로
설명되지 않는다. 같은 제한 자세를 추가 수집해 평균내는 방식은 사용하지 않는다.

## 판정과 다음 게이트

현재 저장소의 공식 차단 항목인 `actual mechanical range UNKNOWN`과 사진 기반
근사 q0/FK를 먼저 해결해야 한다. 자동 motion 없이 torque-off raw 범위만
관측하는 `tools/stm32_raw_range_observer.py`를 추가했다. 이 도구는 보정 파일을
자동 변경하지 않으며 결과도 `apply_to_calibration: false`로 저장한다.

실기는 다음 조건을 모두 만족할 때만 별도 승인 후 진행한다.

- 팔 전체를 사람이 물리적으로 받침
- 주변 충돌물과 케이블 장력 점검
- bridge/MoveIt/Isaac 종료
- observer가 `DISABLE`을 보낸 뒤 수동으로만 관절을 움직임
- 관측 extrema를 그대로 limit으로 사용하지 않고 보수적 margin을 별도 검토
- host와 firmware calibration을 함께 변경하고 새 hash를 검증

위 게이트 전까지 `motion_authorized: false`,
`robot_target_available: false`를 유지한다.
