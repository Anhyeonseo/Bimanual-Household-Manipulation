# ADR-0011: Top-base 등록은 TCP GridBoard eye-to-hand 방식으로 전환

- 상태: 채택
- 날짜: 2026-07-28

## 상황

단계 6에서 Top 카메라의 작업대 좌표와 왼팔 base 좌표를 연결하기 위해 손목
카메라 장착판에 `11.57 x 11.57 mm` 노란 사각형을 붙이고 다섯 자세를
수집했다. 영상 검출과 정사각형 PnP 재투영 오차는 작았지만 등록 결과는
강체 기하 조건을 통과하지 못했다.

- 중심점 기반 SE(2) 등록 RMS: `10.972 mm`
- 정사각형 PnP 기반 3D 등록 RMS: `14.240 mm`
- 최대 잔차: 약 `18 mm`
- robot FK 최대 XY span: `37.660 mm`
- camera 관측 최대 span: `17.814 mm`

재감사 결과 다섯 마커 중심은 모두 원래 작업대 homography 보정판 영역 밖이며
영상 왼쪽 가장자리 `x=21..51 px`에 있었다. 마커 프레임은 실제
`left_gripper_frame_link`에서 약 `73 mm` 옆에 있는 손목 카메라 장착판에
위치했다. 자세 분포도 작고 한 방향으로 편향됐다.

따라서 기존 결과는 카메라가 물체를 못 찾는 증거가 아니다. 작은 단일
사각형의 깊이 모호성, 영상 가장자리, task TCP와 떨어진 위치, 부족한 자세
분포가 결합된 등록 설계 문제다. actual mechanical raw range 측정은 별도의
안전-limit 작업이며 이 좌표 불일치의 직접 보정 방법으로 사용하지 않는다.

## 결정

Top 카메라와 왼팔 base 등록은 실제 grasp TCP 근처에서 그리퍼가 잡는
**2x2 ArUco GridBoard eye-to-hand** 방식으로 전환한다.

- dictionary: `DICT_4X4_50`
- IDs: `0, 1, 2, 3`
- marker side: `20 mm`
- separation: `5 mm`
- grid outer side: `45 mm`
- 마커 네 개를 가리지 않는 별도 grip tab 사용
- 평평한 강성 카드에 부착하고 capture session 동안 장착을 바꾸지 않음

각 capture에서 다음 두 변환을 기록한다.

```text
T_base_gripper  : 실제 /joint_states와 URDF FK
T_camera_target : 고정 intrinsic과 GridBoard PnP
```

정확히 재기 어려운 `T_gripper_target`은 상수 미지수로 두고
`T_base_camera`와 동시에 푼다.

```text
T_base_gripper * T_gripper_target
    = T_base_camera * T_camera_target
```

최소 여덟 training 자세와 training에 사용하지 않은 최소 두 validation
자세를 사용한다. 자세는 최대 translation span `40 mm` 이상, rotation span
`15 deg` 이상이어야 하며 마커는 영상 경계에서 최소 `10 px` 떨어져야 한다.

초기 합격 기준은 다음과 같다.

- PnP reprojection RMS 최대 `1.5 px`
- training translation RMS 최대 `3 mm`
- training translation 최대 잔차 `5 mm`
- training rotation RMS 최대 `1 deg`
- training rotation 최대 잔차 `2 deg`
- held-out validation translation 최대 잔차 `5 mm`
- held-out validation rotation 최대 잔차 `2 deg`

`1.5 px` 한도는 2026-07-29 강체 보드와 고정 초점 Top 카메라로 수집한
20-frame 묶음의 실측 분포를 반영한다. training 01은 `0.986..1.029 px`,
training 02는 `0.777..0.807 px`였으며 intrinsic calibration의 per-view 최대
오차도 약 `0.920 px`였다. 따라서 `1 px`는 정상 영상 노이즈에 대한 여유가
없었다. 이 변경은 영상 측정의 정상 변동만 허용하며 아래의 강체 SE(3)
training 잔차, 자세 다양성, held-out validation 기준은 완화하지 않는다.
종이만 사용해 휘었던 이전 session01의 비강체 불일치는 이 기준을 바꿔도
합격할 수 없다.

solver 결과는 합격해도 `motion_authorized: false`와
`robot_target_available: false`를 유지한다. 이후 실제 작업대의 독립
`x, y, yaw` 계측 검증을 별도 통과해야 단계 6 perception 결과로 사용한다.

## 이유

- 실제 Pick and Place가 사용하는 grasp TCP 부근을 직접 보정한다.
- 여러 ID와 16개 코너가 단일 색상 사각형보다 검출과 PnP에 강하다.
- 미지의 보드 장착 오프셋을 손으로 재서 생기는 오차를 제거한다.
- 영상 가장자리 대신 실제 task FOV 안에서 데이터를 수집한다.
- training과 held-out validation을 분리해 같은 데이터에 맞춘 결과를 PASS로
  오인하지 않는다.
- raw 범위 확대나 URDF 상수 변경 전에 현재 FK가 외부 관측과 일관적인지
  먼저 판별할 수 있다.

## 영향

- 기존 session04와 audit 문서는 실패 증거로 보존하며 삭제하지 않는다.
- 손목 카메라 장착판의 노란 마커는 공식 Top-base 등록 입력으로 더 이상
  사용하지 않는다.
- 새로운 보정판, 읽기 전용 capture 도구와 fail-closed solver를 사용한다.
- 새 방식도 validation에서 실패하면 그때 joint zero, 축, link 길이와
  backlash를 분리 측정한다. 실패 전에 raw extrema나 URDF를 추정으로
  변경하지 않는다.
- STM32 `DISABLE`이 실제 서보 torque를 끄지 않던 별도 결함은 보정 방식과
  독립적으로 수정하고, 실기 재개 전에 firmware readback으로 검증한다.
