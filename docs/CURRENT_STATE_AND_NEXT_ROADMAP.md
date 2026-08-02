# 현재 분기점과 남은 로드맵

- 기준일: 2026-08-02
- 목적: 지금까지 검증된 결과를 보존하면서 단일 팔 완성, 오른팔 동등성 검증,
  양팔 통합과 Raspberry Pi 5 정책 배포의 순서를 명확히 한다.

## 1. 현재 분기점

현재 프로젝트는 “왼팔로 한 번 Pick and Place에 성공한 단계”를 넘어,
검증된 단일 팔 기준선을 생산형 계약으로 바꾸기 직전이다.

### 검증된 사실

- 왼팔 STM32 firmware 0x00021800, protocol 1, calibration
  0x8AD27897, capabilities 0x000003FF 조합을 물리 수락했다.
- Shoulder P32, Elbow P28 설정으로 grasp, 약 20 mm lift, Place, release,
  retreat와 q0 복귀를 감독하에 1회 완주했다.
- 서보 UART 재동기화·완전 복구, 5분 무동작 heartbeat/feedback,
  fault injection과 reset 없는 6축 복구를 통과했다.
- 단계 7 감독형 시운전 체크리스트는 100%다. 그러나 정식 완료 조건인
  50회 중 90% 이상 반복 시험은 아직 수행하지 않아 단계 7은 ‘부분 통과’다.
- Top 카메라의 기존 작업대 좌표 검증은 통과했지만, 2026-08-01 재배치
  영상에서는 대리석 무늬·반사 환경에서 기존 임계값 검출기가
  “detected 2 (ignored 2 fully outside)”로 fail-closed 동작했다.
  영상 수집 자체는 640x480, rgb8, sharpness 87.93으로 정상이다.
- 오른팔은 사용자가 현재 정상 작동한다고 확인했다. 이는 하드웨어 복구
  사실이며, 오른팔의 calibration, 모델, MoveIt, STM32와 반복 동작의 공식
  동등성 검증을 대신하지 않는다.

### 아직 채택하지 않은 항목

- single-point Action을 이어 붙인 정착형 실행은 생산용 연속 trajectory가 아니다.
- Place 높이는 실제 안착에서 총 10 mm 추가 하강이 필요해 Pick/Place 접촉
  offset을 분리해 다시 계측해야 한다.
- 반사·무늬 배경의 검은 펜 holdout에서 legacy 명도 임계값 검출기는
  miss 100%, false positive 66.7%로 실패했다. 이후 별도 학습 데이터로
  경량 YOLO-OBB를 학습·ONNX export했고, Pi 5에서 Top OBB 4 Hz와 3카메라
  동시 30분 자원 gate를 통과했다.
- 손목 카메라 eye-in-hand와 최종 visual correction은 미완료다.
- 오른팔과 양팔 동작은 formal gate를 아직 통과하지 않았다.
- 실제 Isaac 정책의 ONNX 입력·출력, control_dt와 Pi 5 실행시간은 아직
  deployment contract로 동결하지 않았다.

## 2. 현재 이후의 결정

1. 왼팔을 재현 가능한 단일 팔 기준선으로 먼저 완성한다.
2. 정상 복구된 오른팔에 왼팔의 검증 절차를 그대로 적용해 단독 동등성을 만든다.
3. 두 팔이 각각 단독 기준선을 통과한 뒤에만 양팔 동시·공유 영역을 통합한다.
4. Isaac Sim/Isaac Lab 학습은 데스크탑에서 수행하고, 검증된 정책만 ONNX
   deployment bundle로 Raspberry Pi 5에 배포한다.
5. MoveIt은 전역 경로와 충돌 검사를 담당하고, 정책은 관절 보정값 또는
   제한된 Cartesian residual을 출력한다. 정책은 STM32나 servo raw 명령을
   직접 우회하지 않는다.
6. Top 카메라는 전역 탐색, 각 손목 카메라는 접근 직전 상대 정렬을 담당한다.
7. 카메라·인식·정책 부하는 STM32 bridge와 분리하고, stale observation이나
   deadline miss가 있으면 이전 action을 반복하지 않고 fail-closed 한다.

상세 결정은 [ADR-0012](adr/0012-arm-integration-and-pi-policy-deployment.md)에
기록한다.

## 3. 목표 실행 구조

~~~text
Desktop
└─ Isaac Sim/Isaac Lab 학습·평가
   └─ policy.onnx + manifest + calibration/normalization hash

Raspberry Pi 5
├─ 3-camera capture와 phase scheduler
├─ 공통 observation adapter
├─ Top/손목 perception
├─ policy.onnx inference
├─ MoveIt 전역 경로 또는 검증된 trajectory
├─ action safety supervisor와 command arbiter
└─ STM32 bridge

STM32
├─ servo bus 시간축과 bounded interpolation
├─ heartbeat/watchdog
├─ position/read failure 진단
└─ HOLD, physical DISABLE과 latched stop
~~~

정책이 구조화 상태를 입력받으면 Pi의 연산 부담과 sim-to-real 차이가 가장
작다. 이미 학습된 정책이 3개 RGB tensor를 요구한다면 camera order, crop,
해상도, 색 순서, normalization, timestamp와 frame-valid 규칙을 학습 때와
동일하게 재현하고 실제 ONNX로 Pi 자원 gate를 통과해야 한다.

## 4. 남은 로드맵

### A. 왼팔 생산 기준선 완성

1. single-point 정착 체인을 multi-point/buffered trajectory로 교체한다.
2. 시간축, queue, cancel, HOLD, continuous diagnostics와 tracking error 계약을
   단위 시험·mock·plan-only·제한 실기로 검증한다.
3. Pick/Place TCP-to-contact offset을 분리하고 Place 후보 0.015 m를 다시 계측한다.
4. 반사·무늬 배경 holdout과 legacy 실패 기준선을 고정하고 별도 학습
   데이터로 경량 YOLO-OBB를 학습·ONNX export했다. 같은 holdout과 Pi 5
   3카메라 동시 30분 runtime gate를 통과했다.
5. 왼쪽 손목 카메라 eye-in-hand와 마지막 수 cm의 bounded visual correction을
   검증한다.
6. 10회 pilot 뒤 50회 benchmark에서 Pick/Place 각각 90% 이상,
   비명령 동작·충돌 0회를 확인한다.

### B. Pi 5 3카메라·정책 실행 기준선

1. 세 카메라의 stable identity, USB topology, mode, FPS와 phase별 필요도를
   기록했다.
2. 3카메라+STM32 READ_ONLY 30분 시험에서 frame age, decode 시간, CPU,
   memory, 온도, throttling을 machine-readable artifact로 남겼다.
3. policy ONNX의 입력·출력, joint order, action scale, control_dt, stale/deadline
   규칙을 manifest로 동결한다.
4. Pi에서 실제 모델 warm-up과 반복 inference의 p50/p95/max를 측정한다.
5. camera-only 부하에서 STM32 heartbeat/feedback 오류 0회를 확인했다.
   detector/policy 동시 부하는 후속 gate다.
6. camera-only 30분 시험은 통과했다. 다음은 policy shadow 30분, 8시간 soak,
   headless 재부팅 반복 gate다.

2026-08-02 분기점에서 Top YOLO-OBB와 3카메라 30분 동시 부하는 통과했다.
실제 policy ONNX/체크포인트는 아직 로컬에 없으므로 값을 추측하지 않고
`config/policy_deployment_contract.json`과
`tools/validate_policy_deployment_bundle.py`로 model·observation·action·
runtime·provenance 계약을 먼저 fail-closed로 고정한다. 실제 모델을 확보한
뒤에만 bundle artifact를 만들고 Pi shadow inference로 진행한다.

### C. 오른팔 단독 동등성 검증

1. servo ID, 방향, raw range, q0, torque/PID와 전원을 실측한다.
2. 오른팔 URDF/Xacro, collision, SRDF, MoveIt group과 Isaac articulation을
   왼팔 계약과 독립적으로 검증한다.
3. READ_ONLY physical disable, identity, heartbeat, diagnostics를 통과한다.
4. 단일 관절 → 전체 팔 → gripper → home → cancel/fault 순서로 제한 실기를 한다.
5. 오른쪽 손목 카메라 eye-in-hand와 단독 Pick/Place 기준선을 만든다.
6. 왼팔과 같은 pilot·반복 기준을 적용한다.

### D. 양팔 통합

1. 좌우 namespace, joint order, controller와 camera identity를 분리한다.
2. 한 팔 fault 시 양팔 coordinated stop을 먼저 검증한다.
3. 개별 작업 영역에서 병렬 실행과 시작 시각 차이를 측정한다.
4. 공유 영역은 두 독립 계획이 아니라 하나의 충돌 검사 계약으로 실행한다.
5. Top 전역 관측과 두 손목 residual을 하나의 timestamped observation으로 묶는다.
6. 양팔 10회 pilot와 정식 반복 시험을 수행한다.

### E. 학습 정책의 Pi 배포

1. 데스크탑에서 학습된 policy와 기준 baseline을 저장 데이터로 비교한다.
2. Pi에서 motion 없는 shadow mode로 실제 관측과 정책 출력을 기록한다.
3. action limit, workspace, collision, freshness와 deadline supervisor를 통과한
   bounded residual만 허용한다.
4. 왼팔 제한 동작 → 오른팔 제한 동작 → 양팔 순서로 실제 권한을 확장한다.
5. 정책 미사용 baseline보다 재현성 또는 성공률이 수치상 개선될 때만 채택한다.

### F. 시연 재현성과 Headless 운영

여기서 재현성은 카메라 각도·높이, 작업대–base transform과 물체 Z가 같은
기하 조건을 유지한 채 집과 시연 장소 사이에서 **배경과 조명만 달라져도**
동일한 인식·Pick/Place 성능을 내는 것을 뜻한다.

1. 카메라 serial, mount hash, exposure/focus, calibration과 model SHA를 동결한다.
2. 집/시연 장소를 대표하는 배경·조명·반사 조건의 이미지 세트를 만들고
   위치/yaw 오차, miss와 false positive를 분리 측정한다.
3. 자동 노출·white balance가 검출을 흔들지 않도록 고정값 또는 허용 범위를 기록한다.
4. 저장 이미지/rosbag과 Isaac synthetic observation으로 회귀 시험을 수행한다.
5. systemd, udev, journald, watchdog, 안전 종료와 재부팅 STANDBY를 검증한다.
6. 카메라 mount나 물체 Z가 실제로 바뀐 경우는 이 재현성 범위가 아니며,
   기존 calibration을 재사용하지 않고 별도 재보정 gate로 전환한다.

### G. 최종 확장

- 양팔 policy 개선
- segmentation/keypoint 기반 수건 상태 인식
- 수건 grasp와 fold 상태 머신
- 전체 benchmark, 영상과 장애 복구 보고서

## 5. 바로 다음 작업

Top YOLO-OBB와 3카메라 동시 30분 Pi 자원 gate까지 완료했다. 현재 이슈는
실제 Isaac 정책 값을 추측하지 않고 policy ONNX bundle의 model·observation·
action·runtime·provenance 계약과 fail-closed 검증기를 고정한다. 다음 작은
이슈는 실제 학습 체크포인트와 export 설정을 확보해 ONNX bundle 및 validation
artifact를 생성하는 것이다. 그 뒤 Pi 단일 policy shadow smoke로 넘어간다.
“연속 trajectory 계약”과 “Place Z offset”은 별도 이슈로 유지한다. Git issue,
branch, commit과 PR 조작은 사용자가 직접 수행한다.
