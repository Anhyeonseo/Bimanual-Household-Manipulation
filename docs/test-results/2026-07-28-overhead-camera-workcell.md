# 오버헤드 카메라 작업셀 형상 검증 결과

- 날짜: 2026-07-28
- 대상: 정상 왼팔 SO-101 작업셀
- 환경: Ubuntu 24.04.4, ROS 2 Jazzy, Isaac Sim 6.0.1
- 상태: PASS

## 확정한 실물 조립

- `arm_base`는 로봇 바닥 홈에 맞춰 결합한다.
- `cam_mount_bottom`과 `cam_mount_top` 조립체는 180도 반전한다.
- 조립체는 `arm_base` 앞면이 아니라 물리적 오른쪽 측면의 지정 홈에 넣는다.
- 측면 홈 삽입 깊이는 10.0 mm다.
- 파손되어 없는 부분은 `cam_mount_top` 끝의 작은 중앙 돌기뿐이다.
- 다른 끝단 구조와 카메라 마운트 본체는 유지한다.
- 모든 작업셀 연결은 고정 조인트다.

## 검증 결과

| 항목 | 결과 |
| --- | --- |
| RViz 실물 조립 방향 육안 정합 | PASS |
| Xacro/URDF 파싱 | PASS |
| `so101_description` 빌드 | PASS |
| 관련 계약 테스트 18개 | PASS |
| Isaac Sim 6.0.1 USD 합성 | PASS |
| Isaac Sim 6.0.1 GUI 형상 육안 확인 | PASS |
| 생성 USD 재현성 SHA-256 | PASS |

생성된 `overhead_workcell.usd`는 표시 전용 정적 레이어다. 기존 로봇
articulation, joint drive, robotLinks, 질량·관성 및 충돌에는 참여하지
않으며 실제 카메라 센서도 생성하지 않는다.

## 남은 보정 경계

`top_camera_link`와 `top_camera_optical_frame`은 아직 임시 기준 프레임이다.
실제 카메라–로봇 베이스 외부 파라미터를 측정하기 전에는 인식 결과로
실물 모션을 승인하지 않는다. 다음 작업은 `top_camera ↔ workcell_base`
외부 보정과 독립 검증이다.
