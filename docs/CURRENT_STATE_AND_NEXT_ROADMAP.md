# 현재 상태와 다음 로드맵

- 기준일: 2026-08-16
- 기준 firmware: F8.9 `0x00024809`, protocol v2, 12축 resident
- 현재 수락점: Top-camera 기반 왼팔→오른팔 펜 전달 1회 완주

## 완료

- STM32 한 대가 좌 USART1/우 UART4의 SO-ARM101 6축씩을 제어한다.
- 공통 5 ms executor, paired DMA dispatch, operational limits, shoulder unwrap,
  measured tracking과 coordinated stop을 사용한다.
- Pi resident adapter만 serial backend를 소유하고 12축 finite command,
  fresh anchor, terminal feedback과 owner/epoch 상태를 ROS로 공개한다.
- F8.9는 팔 tracking 한계를 유지하면서 그리퍼 접촉에만
  150,000 µrad terminal/route 한계와 160,000 µrad firmware hard cap을 적용한다.
- motion-disabled gate와 current-pose hold 2회를 통과했다.
- 상단 카메라/작업대 보정과 오른팔 data-fit URDF를 적용했다.
- 계획 스키마 12에서 화면축 보정을 plan SHA에 고정한다.
  - 왼팔: 화면 오른쪽 13.72 mm
  - 오른팔: 화면 왼쪽 29.47 mm
- fresh left plan→검증→실행→fresh right plan→검증→실행을 자동 재시도 없이
  완주했고 최종 READY/HOLD를 유지했다.

최종 증거는
[F8.9 resident와 양팔 펜 전달 수락 결과](test-results/2026-08-16-f89-bimanual-pen-transfer.md)에
모았다.

## 운영 불변식

1. motion은 `ready`, `owner=null`, `arbiter_epoch=0`,
   `motion_authorized=true`인 새 session에서 시작한다.
2. STOP/FAULTED session은 재사용하지 않고 STM32 reset과 resident 재시작 후
   새 anchor를 얻는다.
3. 한 팔 동작도 반대 팔 hold를 포함한 12축 command로 제출한다.
4. firmware와 resident의 measured terminal 판정 전에는 성공으로 보지 않는다.
5. dispatch, heartbeat, unwrap, operational-limit 및 arm tracking fault는
   자동 재시도하지 않는다.
6. plan-only/validate-only와 실제 실행을 분리하고 plan SHA와 최대 age를 검사한다.

## 다음 PR: 캔 한 개 수직 접근 파지

범위를 캔 집기 한 동작으로 제한한다.

1. 현재 Top YOLO-OBB 데이터셋과 holdout을 정리하고 캔 한 개의 장축 yaw를
   `modulo π`로 안정화한다.
2. 실측 캔 치수와 gripper open/close/contact 값을 bounded probe로 확정한다.
3. 그리퍼 접근축은 작업대에 수직으로 유지하고, 닫힘 축은 캔 장축에
   90도로 맞춘다.
4. wrist-roll 등가 해를 operational limit 안에서 열거하고 현재 anchor에서
   회전량이 가장 작은 분기만 고정한다. 불필요한 180°/360° 회전은 금지한다.
5. open-gripper 자세 확인→plan-only→validate-only→감독 1회 파지 순으로 승격한다.

현재 wrist-roll 한계:

| arm | lower | upper | span |
|---|---:|---:|---:|
| left | -128.41° | +69.43° | 197.84° |
| right | -114.17° | +81.04° | 195.21° |

양팔 모두 180°보다 넓어 무방향 장축의 모든 yaw를 표현할 수 있지만,
수직 접근·충돌·관절 결합 가능성은 각 plan에서 별도 검증한다.

## 이후로 미룬 범위

- 캔→쓰레기통
- 왼팔→오른팔 캔 handover
- wrist-camera close-up refinement
- pretrained policy와 범용 상단 arbiter
- 반복성 benchmark 및 8시간/24시간 soak
- systemd/부팅 복구와 운영 E-stop runbook

위 항목은 캔 단일 파지가 통과한 뒤 각각 별도 PR로 다룬다.

## 관련 문서

- [양팔 상단 애플리케이션 인터페이스](BIMANUAL_UPPER_APPLICATION_INTERFACE.md)
- [Top-camera resident Pick/Place](TOP_CAMERA_RESIDENT_PICK_PLACE_APPLICATION.md)
- [F8.7 이전 수락 결과](test-results/2026-08-15-f87-resident-top-camera-pick-place.md)
- [F8.9 최종 수락 결과](test-results/2026-08-16-f89-bimanual-pen-transfer.md)
