# 검증 매트릭스

상태: `미실행`, `부분 통과`, `통과`, `실패`, `차단`

| ID | 단계 | 검증 | 초기 합격 기준 | 상태 | 증거 |
|---|---|---|---|---|---|
| HW-001 | 단계 0 | 서보 12개 ping | 12/12 응답 | 부분 통과 | 단일 시험 팔 6/6 응답 |
| HW-002 | 단계 0 | ID와 관절 연결 확인 | 좌우 각 1~6 기록 | 부분 통과 | 단일 시험 팔 ID 1~6 확인 |
| HW-003 | 단계 0 | 상태값 읽기 | position/speed/load/voltage 기록 | 부분 통과 | [단일 팔 실기 결과](test-results/2026-07-20-stm32-binary-control-plane.md) |
| HW-004 | 단계 0 | 전원 재인가 | 명령하지 않은 움직임 0회 | 부분 통과 | 단일 시험 팔 확인 |
| HW-005 | 단계 0 | 전원 출력 | 양팔 각각 무부하/동작 전압 기록 | 부분 통과 | 단일 시험 팔 12.3~12.5V 확인 |
| ROS-001 | 단계 1 | 새 환경 build | 오류 0 | 미실행 |  |
| ROS-002 | 단계 1 | Mock 실행 | STANDBY 진입 | 미실행 |  |
| ROS-003 | 단계 1 | 잘못된 명령 | NaN/범위 초과/오래된 명령 100% 거부 | 미실행 |  |
| MODEL-001 | 단계 4 | 왼팔 URDF/Xacro | visual, TF, q0, 축과 limit 일치 | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| MOVEIT-001 | 단계 4 | 왼팔 MoveIt config | SRDF, collision, IK와 q0 validity 정상 | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| MOVEIT-002 | 단계 4 | mock 대표 trajectory | arm과 gripper Plan/Execute 성공 | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| ISAAC-001 | 단계 4 | Isaac articulation | 6 joint 안정 유지, state/command round trip | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| ISAAC-002 | 단계 4 | MoveIt → Isaac trajectory | arm random pose, gripper open/closed, home 성공 | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| ISAAC-003 | 단계 4 | 실제 hardware 격리 | serial/STM32 접근과 실제 servo 동작 0 | 통과 | [단계 4 통합 결과](test-results/2026-07-24-isaac-moveit-left-arm-integration.md) |
| MOVEIT-003 | 단계 5 | MoveIt → STM32 single-point | home, arm, gripper와 feedback 성공 | 통과 | [단계 5 실기 결과](test-results/2026-07-25-phase5-stm32-read-only.md) |
| MCU-001 | 단계 2 | packet 해석기 | 절단/CRC/길이 오류 거부 | 통과 | [바이너리 제어 경로 결과](test-results/2026-07-20-stm32-binary-control-plane.md) |
| MCU-002 | 단계 2 | heartbeat 단절 | 정의된 시간 안에 안전 정지 | 통과 | [바이너리 제어 경로 결과](test-results/2026-07-20-stm32-binary-control-plane.md) |
| MCU-003 | 단계 2 | 제어 loop | overrun/underflow 0 | 미실행 | 여러 sample queue 구현 후 시험 |
| MCU-004 | 단계 2 | 단일 팔 6축 동시 적용 | 같은 명령에서 함께 시작 | 통과 | [바이너리 제어 경로 결과](test-results/2026-07-20-stm32-binary-control-plane.md) |
| CAM-001 | 단계 3 | 카메라 3대 capture | 장치별 목표 FPS 기록 | 통과 | [카메라 대역폭·제어 격리 결과](test-results/2026-07-21-camera-bandwidth-control-isolation.md) |
| CAM-002 | 단계 3 | 재연결 | 자동 복구 시간 기록 | 통과 | [카메라 manager·hot-plug 결과](test-results/2026-07-21-camera-manager-hotplug.md) |
| CAM-003 | 단계 3 | 제어 격리 | 카메라 부하 중 heartbeat 위반 0 | 통과 | [카메라 decode·DDS 제어 격리 결과](test-results/2026-07-21-camera-decode-control-load.md) |
| CAM-004 | 단계 3 | 추론 일정 | 모든 작업 상태 합계 12Hz 이하 | 통과 | `config/camera_schedule.json` 정적 검증 |
| CAM-005 | 단계 3 | frame 최신성 | 상태별 p95/max 기록 | 통과 | [phase scheduler·선택적 decode 결과](test-results/2026-07-21-camera-phase-decode-latency.md) |
| RES-001 | 단계 3/9 | Pi 자원 한도 | CPU/memory/temperature 기준 충족 | 부분 통과 | [3카메라·bridge 30분 기준선](test-results/2026-08-02-pi-runtime-camera-only-30m.md)과 [3카메라·Top OBB 30분](test-results/2026-08-02-pi-runtime-top-obb-30m.md) 통과. Top OBB 동시 부하에서 CPU 평균 35.07%, 온도 최대 50.15°C, swap·throttling 0. policy·MoveIt·8시간 시험은 미실행 |
| RES-002 | 단계 9 | Pi 실제 통합 부하 | 3카메라+검출기+배포 policy+MoveIt+bridge에서 heartbeat 위반·throttling·swap 0 | 부분 통과 | [3카메라·Top OBB 30분](test-results/2026-08-02-pi-runtime-top-obb-30m.md)에서 Top OBB 3.989 Hz, 처리 오류·명령 발행·재연결 0. MoveIt·실제 ONNX policy shadow 미포함 |
| POL-001 | 단계 9/11 | policy observation 계약 | 학습과 배포의 camera order·전처리·shape·normalization 또는 structured schema 일치, deadline 기록 | 부분 통과 | [배포 번들 계약](checklists/STAGE9_POLICY_DEPLOYMENT_BUNDLE.md)과 fail-closed 검증기 구현. 실제 policy ONNX bundle 미확보 |
| POL-002 | 단계 9/11 | Pi policy shadow mode | stale/deadline/범위 초과 출력 100% 차단, 실제 명령 0 | 미실행 |  |
| MOT-001 | 단계 5 | 단일 시험 왼팔 trajectory | 반복 실행 성공 | 통과 | [단계 5 실기 결과](test-results/2026-07-25-phase5-stm32-read-only.md) |
| MOT-002 | 단계 5 | 취소/정지 | 정해진 안전 상태 진입 | 통과 | [단계 5 실기 결과](test-results/2026-07-25-phase5-stm32-read-only.md) |
| MOT-003 | 단계 8 | 왼팔 multi-point/buffered trajectory | 시간축·queue·cancel·soft-abort·SAFE_STOP 실기 통과, 불필요한 정착 정지 제거 | 부분 통과 | [Motion-1 host 계약](checklists/MOTION_BUFFERED_TRAJECTORY_CONTRACT.md)과 [Motion-2 STM32 core 후보](checklists/MOTION_STM32_BUFFERED_QUEUE.md): 다중점 검증, 원자적 queue, 선형 보간/refill, cancel/HOLD/underflow/missed-tick fault injection과 Cortex-M4 cross-build 통과. 현재 0x218 binary route와 Action은 여전히 `sample_count=1`; timing 실측·제한 실기 미실행 |
| VIS-001 | 단계 6 | 작업대 위치 추정 | 위치 최대 10 mm, yaw 최대 5 deg | 통과 | [Top 물체 실제 좌표 검증](test-results/2026-07-30-top-object-ground-truth-validation.md) |
| VIS-002 | 단계 7 준비 | base-frame shadow target 및 table–base 등록 | 두 위치 물리 검증, freshness/workspace 검사, 실행 가능 flag false | 통과 | [현재 작업대–왼팔 base 등록](test-results/2026-07-30-current-table-base-registration.md), [Top-base shadow target](test-results/2026-07-30-top-base-shadow-target.md) |
| VIS-003 | 단계 8 | 시연 환경 강건 펜 검출 | 카메라 각도·높이·물체 Z 고정, 배경·조명·반사만 다른 조건에서 물체 1개와 위치/yaw·miss·false positive 기준 충족 | 통과 | [기준선 계약](test-results/2026-08-02-top-pen-detection-baseline-contract.md), [holdout·legacy 결과](test-results/2026-08-02-top-pen-holdout-legacy-baseline.md), [YOLO-OBB 후보](checklists/STAGE8_TOP_PEN_YOLO_OBB.md). 고정 holdout에서 miss 0%, false positive 0%, 중심 p95 5.29 px, yaw p95 2.79° |
| TASK-001 | 단계 7 | Pick | 50회 중 90% 이상 | 부분 통과 | [감독형 실제 Pick/Place 1회 완주](test-results/2026-07-31-stage7-supervised-pick-place-complete.md): grasp와 약 20 mm loaded lift 성공, 자동 재시도 0회. 50회 반복과 무인 perception-to-task 실행은 미실행 |
| TASK-002 | 단계 7 | Place | 50회 중 90% 이상 | 부분 통과 | [전체 Pick/Place plan-only](test-results/2026-07-31-stage7-full-pick-place-plan-only.md) 및 [감독형 실제 Pick/Place 1회 완주](test-results/2026-07-31-stage7-supervised-pick-place-complete.md): Place·release·retreat·q0 복귀 성공. 수동 Z 보정이 있었고 50회 반복은 미실행 |
| SYS-001 | 단계 9 | 부팅 | 반복 부팅 모두 무동작 STANDBY | 미실행 |  |
| SYS-002 | 단계 9 | 장시간 시험 | 8시간 후 24시간 | 미실행 |  |
| RIGHT-001 | 단계 10 | 오른팔 단독 하드웨어·모델·안전 동등성 | 6축 identity/calibration, q0/FK, READ_ONLY, 무동작, 격리 이동과 fault gate 통과 | 부분 통과 | 사용자가 오른팔 정상 동작을 확인했으나 저장소의 정식 parity 증거는 미수집 |
| RIGHT-002 | 단계 10 | 오른팔 단독 Pick/Place | 왼팔과 동일한 반복성·비명령 동작·충돌 기준 통과 | 미실행 |  |
| DUAL-001 | 단계 11 | 실제 시작 시각 차이 | 측정값과 기준 기록 | 미실행 |  |
| DUAL-002 | 단계 11 | 연동 정지 | 한 팔 fault 시 양팔 정지 | 미실행 |  |
| DUAL-003 | 단계 11 | 양팔 통합 진입 gate | LEFT production baseline과 RIGHT parity가 모두 통과 | 차단 | 단일 팔 gate 완료 전 양팔 실제 명령 금지 |
| AI-001 | 단계 11 | policy 비교 | baseline 대비 개선 | 미실행 |  |

`부분 통과`는 일부 실기 증거는 있으나 해당 행의 전체 합격 기준을 아직
충족하지 않았다는 뜻이다. 특히 오른팔의 정상 동작 보고는 중요한 출발점이지만
정식 parity gate를 대신하지 않는다. 기준을 바꾸면 ADR 또는 변경 사유를 남긴다.
