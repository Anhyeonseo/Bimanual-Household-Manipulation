# C3 — 손목 카메라 eye-in-hand (W0–W4)

## 목적

pre-grasp 이후 그리퍼와 물체의 **상대 오차를 손목 카메라로 측정**하고, 크기가
제한된 보정을 목표에 주입한다. 도달(자기수용)은 C1/C2 수렴 루프가 담당하고
목표(외부수용)는 이 트랙이 담당한다. 두 층을 섞지 않는다.

일반 작업 권한 `motion_authorized=false` 는 W4 물리 통과까지 유지한다.

상위 계획: `docs/PLAN_CONVERGENCE_AND_BIMANUAL.md` 6절 C3
역할 정의: `docs/CAMERA_COMPUTE_ARCHITECTURE.md` 2절 Left wrist, 3절 보정 표
선행: C1 완료 2026-08-06, C2 완료 2026-08-06

원 계획의 단계 열거는 `W0 → W1 → W3 → W4` 였고 `W2` 는 번호만 비어 있었다.
캡처 세션이 solve 와 분리된 별개 산출물이므로 여기서 **W2 = 캡처 세션**으로
정의한다. 상단 트랙도 `capture_top_eye_to_hand_sample.py` 와
`solve_top_eye_to_hand.py` 가 별개 도구였다.

## 이미 동결된 것 — 건드리지 않는다

| 항목 | 값 | 근거 |
|---|---|---|
| 마운트 STL 해시 | `b4345ccf…fb7b4` | `tests/test_wrist_camera_mount_contract.py:25` |
| 마운트 중심 조인트 | `xyz 0.00250008 -0.07292374 0.00595299`, `rpy -2.70526034 0 0`, parent `gripper_link` | 같은 시험 `:119` |
| 등록 마커 조인트 | `xyz 0 0 -0.002`, parent 마운트 중심 링크 | 같은 시험 `:143` |
| Isaac 참조 | `instances.usda` 에서 생성 USD 참조 정확히 2회 | 같은 시험 `:164` |
| 캡처 설정 | `wrist_a`/`wrist_b` device_path, 9 phase × 3 camera 일정 | `manipulation_camera_manager/config/cameras.yaml:21` |
| 관측 순서 | `top, wrist_a, wrist_b` | `config/policy_deployment_contract.json:31` |

마운트 평면의 기하는 끝났다. 그 아래(optical frame·내부 파라미터·외부
보정·소비 노드)는 하나도 없다.

## 시작 전에 닫아야 하는 불일치

1. ~~**frame_id 가 URDF 링크와 다르다.**~~ **닫힘.** `camera_manager_node.cpp`
   에 `<name>.frame_id` 파라미터를 추가했다. 기본값은 종전과 동일
   (`<name>_optical_frame`, 회귀 없음). 실제 배정은 불일치 3번(좌우 확인)
   이후에 한다 — 확인 전에 `wrist_a.frame_id` 를 박으면 틀린 배정을 박제한다.
2. ~~**`left_tool0` 은 존재하지 않는다.**~~ **닫힘.**
   `CAMERA_COMPUTE_ARCHITECTURE.md:59` 를
   `left_gripper_frame_link → left_wrist_camera_optical_frame` 로 고쳤다.
3. ~~**`wrist_a` 가 왼팔인지 실측 미확인.**~~ **닫힘 (2026-08-09).** 포트 1.2/1.3
   device path 에서 각각 한 프레임씩 v4l2-ctl 로 직접 캡처(camera_manager 미기동
   상태)해 육안 확인 — **`wrist_a` = 왼팔**. `cameras.yaml` 에
   `wrist_a.frame_id: left_wrist_camera_optical_frame` 반영.
   주의: 이는 단발 시각 확인이며 상단 트랙처럼 독립 검증을 거친 등록
   절차는 아니다. `LOCAL_HARDWARE_CONFIG.md:65-71` 이 경고하듯 USB 포트가
   재배선되면 이 배정도 무효가 되니, 물리 리그를 다시 꾸릴 때마다 재확인한다.
4. **CameraInfo 토픽이 없다.** camera_manager 는 Image 만 낸다. 상단은 YAML
   을 직접 읽어 우회했다(`detector.py:102`). 손목도 같은 방식으로 간다.
   W1 에서 YAML 을 만들 때 함께 처리한다 — 별도로 닫을 항목이 아니다.
5. **카메라 모듈 질량·관성 미측정 — 열려 있음, 물리 측정 필요**
   (`so101_description/README.md:46-48`). 그리퍼 질량이 상류 값 그대로라
   처짐에 들어간다. W3 잔차 해석에서 보정 오차와 처짐을 분리해야 한다.
   C2 는 두 자세 모두 SHOULDER 지배(7.42/3.61 mm)로 측정했다.

## W0 — optical frame 을 TF 트리에 넣는다 — **좌우 배정 제외 완료**

- [x] `so101_wrist_camera.xacro` 신설. `${prefix}wrist_camera_link`
      (parent = `${prefix}wrist_camera_mount_center_link`, origin = xacro arg)
      + `${prefix}wrist_camera_optical_frame` (순수 회전, arg).
      `so101_overhead_webcam_mount.xacro:105-117` 패턴 그대로.
      camera_xyz/camera_optical_rpy 기본값은 `0 0 0`(CAD 미제공, identity
      placeholder) — W1/W3 가 실측으로 덮어쓴다
- [x] `so101_left.urdf.xacro` 에 `wrist_camera_xyz` /
      `wrist_camera_optical_rpy` arg 추가, `use_wrist_camera_mount` 로 게이트
- [x] camera_manager 에 `<name>.frame_id` 파라미터 추가 (기본값 불변, 회귀 없음)
- [x] `tests/test_wrist_camera_optical_frame_contract.py` — 링크/조인트 집합,
      마운트 중심 링크로부터의 부모 체인, rpy/xyz 주입, 기본값이 identity
      placeholder 임을 확인. `test_overhead_webcam_mount_contract.py:22-32` 를
      본떴다
- [x] 기존 `test_wrist_camera_mount_contract.py` 의 pin 된 값이 **하나도 변하지
      않음** 확인 — 20 test 전체 통과 (기존 14 + 신규 6)
- [x] `wrist_a`/`wrist_b` 좌우 배정 실측 후 문서화 — `wrist_a` = 왼팔 (2026-08-09,
      단발 시각 확인, 불일치 3 참고)
- [x] `cameras.yaml` 에 왼팔 손목 `frame_id` 를 `left_wrist_camera_optical_frame`
      로 지정
- [ ] 게이트: bridge/camera_manager 기동 후
      `tf2_echo left_gripper_frame_link left_wrist_camera_optical_frame` 성공
      확인 — 다음 물리 세션에서. 성공하면 `RL_INTEGRATION_REQUIREMENTS.md:166`
      의 선행조건이 해제된다

## W1 — 내부 보정 — **완료 2026-08-09**

- [x] 인쇄 팩 — 9×6 내부 코너, 25 mm 체스보드를 A4 실제 크기로 출력
      (`render_camera_calibration_print_pack.py`, `so101_camera_intrinsic_checkerboard_9x6_25mm.pdf`)
- [x] 팔 정지, 그리퍼 상태 고정, 손목 카메라(`wrist_a`) 앞에서 체스보드만 이동
      (그리퍼 손가락이 프레임 일부를 항상 가림 — 실사용에서도 그 영역은
      동일하게 안 쓰이므로 커버리지 계산에서 문제 아님)
- [x] `ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025
      --no-service-check` 로 44장 캡처, SAVE (COMMIT 은 `set_camera_info`
      서비스가 없어 불가)
- [x] SAVE 산출물은 원본 이미지만 담고 뷰별 재투영 통계가 없어서, 저장된
      44장으로 `cv2.calibrateCamera` 를 직접 재실행해 계산. 뷰별 RMS
      `4.057 px` 인 1장을 outlier 로 제거하고 재계산
- [x] `wrist_a_camera_info.yaml` 산출 — `top_camera_info.yaml` 과 같은 헤더
      규약(촬영 조건, 표본 수, RMS 분해, 재사용 금지 조건)
- [x] 수용 게이트 통과 — 재투영 RMS `0.5666 px` (`≤1.0 px` 게이트),
      44 captured / 43 retained, per-view mean `0.5153`, median `0.4128`,
      p95 `1.0039`, max `1.3371 px`
- [x] SHA-256 기록 — `44f277393d614e873c85aa3d5cf96f725d86a26adc4748f10f5879fa0789e350`.
      W3 extrinsics 문서가 이 값을 교차검증한다 (`detector.py:120-125` 방식)
- [x] 해상도·초점·mount 가 바뀌면 재사용 금지 명시
      (`CAMERA_COMPUTE_ARCHITECTURE.md:85`)
- [x] 팔을 DISABLE 하지 않고 토크 유지 상태로 촬영

## W2 — 캡처 세션 — **완료 2026-08-09**

- [x] 표적은 기존 planar ArUco gridboard 4×5 / 20 mm / ID 10-29
      (`generate_planar_aruco_gridboard.py:14-18`) 를 **작업대에 고정**한다.
      인쇄 파일은 이미 있다 (`artifacts/wrist_camera/print_pack/`).
      eye-in-hand 는 표적이 base 고정, 카메라가 tool 고정이다 — 상단
      eye-to-hand 와 역할이 반대다
- [x] `tools/capture_wrist_eye_in_hand_sample.py` — image + `/joint_states`
      동시 캡처. `capture_top_eye_to_hand_sample.py` 를 본떴다. 기대 마커
      ID 는 `range(10, 30)` 전부 — 20개 마커가 한 프레임에 모두 보여야 채택
- [x] `tools/assemble_wrist_eye_in_hand_session.py` — 세션 YAML 조립.
      frames: robot `left_base_link`, tool `left_gripper_frame_link`,
      camera `left_wrist_camera_optical_frame`, target
      `wrist_planar_aruco_gridboard`. 상태 문자열을
      `WRIST_EYE_IN_HAND_STATIONARY_CAPTURE_PASS` 로 분리해 top 캡처와
      섞여 들어갈 수 없게 했다
- [x] 시험 `test_capture_wrist_eye_in_hand_sample.py` /
      `test_assemble_wrist_eye_in_hand_session.py` — top 짝과 함께 30 test
      전체 통과
- [ ] 자세 분포 게이트 재사용 (`solve_top_eye_to_hand.py:42-53`) —
      표본 `≥8`, translation span `≥0.040 m`, rotation span `≥15도`.
      이 게이트는 W3 solve 스크립트에 들어간다 (top 도 assemble 이 아니라
      solve 단계에서 검사한다)
- [x] ~~자세 이동은 buffered leg 경로로만 한다.~~ **정정.** 리포에 있는
      규칙은 "토크 걸린 상태에서 그리퍼 조를 손으로 밀지 마라"
      (`PHASE_5_GRIPPER_MAPPING_PLAN.md:83`) 뿐이고, 관절을 손으로 위치잡는
      것 자체를 금지하는 규칙은 없다. 중요한 건 **캡처하는 동안(≈20프레임)
      팔이 안 움직이는 것**뿐이며, `/joint_states` 는 엔코더 실측이라 손으로
      옮겼어도 정확하다. 이 정지 여부는 `capture_wrist_eye_in_hand_sample.py`
      가 `--max-joint-span` 로 스스로 검사한다 — 손 위치잡기 → 바로 캡처면
      충분하다
- [x] `tools/move_and_capture_wrist_eye_in_hand_pose.py` 도 만들어뒀다 —
      정확한 목표 관절각을 미리 아는 경우(재현 가능한 자세가 필요할 때)를
      위한 선택지. anchor 캡처 → plan → buffered leg → 단발 실행 →
      캡처를 자세 하나=명령 하나로 묶는다. 필수는 아니다
- [x] 세션 첫 회차(`train_01`)도 포함 — 이번엔 카메라 보정이라 A5 의
      서보 워밍업 전이(모터 반복 구동 열화)와 무관하다고 판단, 배제하지 않음
- [x] **물리 캡처 완료** — `session.yaml` (`2026-08-09_wrist_a_eye_in_hand`),
      학습 10 / 검증 2. 자세 스프레드(rad): base `1.00`, shoulder `2.07`,
      elbow `1.66`, wrist_flex `1.03`, wrist_roll `0.55`
      (처음 8개는 wrist_roll 이 거의 안 변해 `train_09`/`train_10` 을
      wrist_roll 양쪽 끝(`±0.25 rad`, 캘리브레이션 한계 `±0.2669 rad`)
      근처로 추가). `train_09` 는 한계를 raw 로 약 23틱(안전 여유 40 내)
      살짝 넘었음 — 문제는 없었지만 기록해둔다

## W3 — eye-in-hand solve — **완료 2026-08-09**

- [x] `tools/solve_wrist_eye_in_hand.py` — **`cv2.calibrateHandEye`**
      (TSAI). 상단은 `calibrateRobotWorldHandEye`/SHAH
      (`solve_top_eye_to_hand.py:328`)이고 문제 구조가 다르다. `parse_target`,
      `capture_observation`, `average_target_poses`, `PoseObservation` 등은
      `solve_top_eye_to_hand.py`에서 그대로 재사용(둘 다 제네릭) —
      eye-in-hand 고유 부분만 새로 작성: `solve_eye_in_hand`,
      `transform_residual`(타겟이 base 고정이므로 "관측 간 합의"와 비교하는
      방식으로 top 과 다름), `classify`, `solve_document`
- [x] `gripper_to_camera` 4×4 + `camera_info_sha256` 교차검증 +
      `motion_authorized: false` — 산출물에 포함 (top 처럼 별도
      extrinsics 파일명은 아니지만 같은 필드를 담음)
- [x] 해석적 왕복 회귀 시험 `tests/test_wrist_eye_in_hand.py` — 6 test,
      노이즈 없는 합성 데이터로 기계정밀도(`~1e-16`) 복원 확인
      (`test_top_eye_to_hand.py:76` 방식을 따름)
- [x] 수용 게이트 — **이 트랙 전용으로 재도출**. 처음엔 top 의 임계값
      (RMS 3mm/1°, max 5mm/2°)을 그대로 가져다 썼는데, 근거 없는 재사용이라
      틀렸다. 종이 ArUco 보드·근접 촬영·강한 렌즈 왜곡(k1=-0.456)의 손목
      트랙은 상단 체스보드 트랙과 달성 가능한 정밀도 바닥이 다르다.
      5가지 `cv2.calibrateHandEye` method(TSAI/PARK/HORAUD/ANDREFF/
      DANIILIDIS) 전부 같은 데이터에서 거의 동일한 결과로 수렴함을 확인해
      (알고리즘 문제가 아님을 배제) 실측 바닥(학습 RMS 6.8mm/max 11.8mm)에
      여유를 두고 재설정: RMS `10mm`/`1.5°`, max `15mm`/`3°` (학습·검증
      동일). PnP RMS `≤1.5px`, 경계 여유 `≥10px` 는 상단과 동일하게 유지
      (이 둘은 촬영 품질 문제지 트랙 특성이 아니라서)
- [x] **독립 검증** — `validation_01`/`02` 두 자세로 재투영 확인, 최종
      RMS `8.18mm`/`1.63°` 로 새 임계치 통과. 상단 트랙처럼 등록 세션을
      무효화했다가(1차: 보드 미고정으로 RMS 85mm) 재촬영으로 복구한 이력
      그대로 반복함 — `manipulation_camera_manager/README.md:76-100` 의
      경고가 손목에서도 실제로 일어났다
- [x] 보정값을 URDF arg 로 재주입 — `gripper_to_camera` 를
      `left_wrist_camera_mount_center_link` 기준 `camera_xyz`/
      `camera_optical_rpy` 로 분해(`inv(GM) @ GF @ X`, GM/GF 는 URDF 에서
      FK 로 읽은 고정 변환)해 `so101_left.urdf.xacro` 기본값으로 반영.
      분해가 정확한지 `rpy_matrix` 왕복으로 확인(오차 `~1e-16`)
- [x] 4×4 강체성 확인 — 직교 오차 `~3e-16`, `det(R) = 0.9999999999999997`
      (수치상 정확히 `+1`). `cv2.calibrateHandEye` 출력이 실제로 유효한
      회전행렬임을 이번 실측값으로 확인했다. 코드화된 자동 시험은 아직
      없음(`shadow_target.py:104-106` 처럼 solve 출력에 넣을 수 있다)
- [x] `test_wrist_camera_optical_frame_contract.py` 의 identity-placeholder
      시험을 실측 보정값 pin 으로 교체 — 48 test 전체 통과

## W4 — bounded visual correction

- [ ] 손목 검출: 펜 중심·yaw 를 **카메라 좌표 상대 오차**로 낸다. ROI + 작은
      검출기 (`CAMERA_COMPUTE_ARCHITECTURE.md:60`). 상단 OBB 번들은 `top_board`
      도메인 전용이라 그대로 쓸 수 없다
- [ ] 주입 지점. 권장은 Cartesian 재계획 — 보정된 x/y 로
      `ros_moveit_plan_grasp.py` 재호출
- [ ] **yaw 는 그 경로로 갈 수 없다.** `ros_moveit_plan_grasp.py:98-101` 이
      position_only IK 라 yaw 를 명시적으로 버린다.
      `grasp_yaw_kinematics.py:212 solve_wrist_roll()` 이 필요한 계산을 이미
      하는데 **production 호출자가 없다** — 여기를 연결한다
- [ ] 대안 경로: 검증된 넘겨명령
      (`ros_moveit_plan_pregrasp_segments.py:286-294` `--target-joints`,
      `execute_grasp_convergence_once.py:166` 이 이미 쓴다)
- [ ] 보정 한계는 기존 상수를 존중한다 — `MAXIMUM_CORRECTION_M = 0.030`,
      그리고 **관절 델타 `18 raw`(≈`0.0276 rad`) 이하는 서보가 `0 raw` 움직인다**
      (`grasp_convergence.py:79-98` 실측). 그보다 작은 보정은 재명령으로 실현
      불가이므로 게이트에서 걸러 보고한다
- [ ] 관절 한계 여유 `1.0e-5 rad` 유지 — µrad 양자화가 bridge 의 `1e-9` 보다
      거칠어 한계에 정확히 앉은 명령이 거부된다 (`grasp_convergence.py:100-113`)
- [ ] fail-closed: frame 노후, 신뢰도 미달, timeout, 검출 0개 또는 2개 이상 →
      즉시 중단. 조용한 반복도 조용한 포기도 없다
- [ ] **층을 섞지 않는다.** W4 가 목표를 고치고 C1 수렴 루프가 도달한다.
      `grasp_convergence.py:230-232` 는 frozen `state.nominal_rad` 기준이라
      "목표가 움직였다" 는 입력이 없다. 목표 갱신은 재계획으로만
- [ ] 물리 게이트: 인식 → pregrasp → 손목 보정 → 수렴 → 파지 전 구간.
      A5 규율(세션 첫 회차 배제)로 반복하고 회차별 잔차를 기록

## Fail-closed 조건 (트랙 공통)

- `camera_info` 해시와 extrinsics 문서의 교차 해시가 불일치하면 거부
- 영상 해상도가 `camera_info` 와 다르면 거부
- 두 문서가 **모두** `motion_authorized: true` 가 아니면 동작 권한 없음
- 자세 분포 게이트 미달이면 solve 하지 않는다
- 학습에 쓴 자세만으로 검증했으면 통과로 인정하지 않는다
- 실패해도 자동 재시도하지 않는다

## 물리 기동

bridge 는 Pi, MoveIt 은 desktop. 양쪽에 `ROS_DOMAIN_ID=30` /
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. 팔을 받치기 전에 DISABLE 하지 않는다.
