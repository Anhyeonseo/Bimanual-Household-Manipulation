# RL 통합 요구사항 — 양팔 Pick/Place 와 수건 접기

RL 팀이 이 목록을 전부 만족시키면 **추가 협의 없이 통합**된다.
반대로 하나라도 빠지면 `tools/validate_policy_deployment_bundle.py` 가
거부하거나, 통과하더라도 실기에서 전이되지 않는다.

2026-08-06 기준. 실기 실측이 반영돼 있다.

---

## 0. 먼저 — 우리가 줘야 할 것 (RL 팀 책임 아님)

이것들이 없으면 RL 팀이 아무리 잘 만들어도 통합할 곳이 없다. 요구사항을
읽기 전에 어디까지가 우리 몫인지 분명히 한다.

| 항목 | 상태 | 없으면 생기는 일 |
|---|---|---|
| 오른팔 슬롯 활성화 | 미구현 (프로토콜만 예약) | 양팔 정책을 실행할 대상이 없다 |
| 비동기 펌웨어 (host TX / servo I/O / 타이머 ISR) | 미구현, **양팔 진입 전 필수** | 양팔 트래픽에서 apply lateness 예산이 무너진다 |
| 조율된 중단 (한 팔 fault → 양팔 정지) | 미구현 | 한 팔이 멈춰도 다른 팔이 계속 움직인다 |
| **연속 명령 경로** | **미해결** | 아래 §3 참조. 이것이 가장 큰 미결 항목이다 |
| 자세별 처짐 지도 | 미측정 | 아래 §2 참조. 시뮬레이터가 이걸 모를 수 없다 |
| `so101_isaac_bridge/mapping.py` 관절 한계 3건 | **결함 있음** | shoulder/wrist_roll 구간 뒤집힘, wrist_flex stale. Isaac backend 한정이지만 RL 이 그 경로를 쓴다 |

---

## 1. 통합 표면 — 반드시 맞춰야 하는 계약

`config/policy_deployment_contract.json` 과
`tools/validate_policy_deployment_bundle.py` 가 강제한다. 협상 대상이 아니다.

### 1.1 ONNX 형식

- [ ] `format: onnx`
- [ ] **batch = 1 고정.** 동적 batch 축 금지
- [ ] **모든 non-batch 차원이 정적 양수.** `dim_param` 금지, `-1` 금지
- [ ] 선언한 입출력의 **이름 / dtype / shape 가 그래프와 정확히 일치**
- [ ] dtype 은 `bool, float16, float32, float64, int32, int64, uint8` 중에서만
- [ ] opset 을 manifest 에 적고 그래프와 일치시킬 것

### 1.2 제공 형식 (provenance)

- [ ] `checkpoint_sha256`
- [ ] `training_config_sha256`
- [ ] `export_config_sha256`
- [ ] `deployment_contract_sha256` = 계약 파일을 **다시 계산한** SHA-256

전부 64자리 hex. 하나라도 다르면 `refuse_start`.

### 1.3 런타임 예산 (Pi 에서 도는 값이다)

- [ ] `runtime.mode = SHADOW`
- [ ] `backend = onnxruntime_cpu` (GPU 없음)
- [ ] `command_publications_allowed = false` — **정책은 아직 로봇을 움직이지
      않는다.** 이 값을 true 로 바꾸는 것은 별도 게이트다
- [ ] 추론 p95 **≤ 80 ms**
- [ ] 실제 관측 rate ≥ 목표의 **90%**
- [ ] `control_dt_s × target_inference_hz == 1.0` (현재 20 Hz → `deadline_ms ≤ 50`)

**CPU 예산이 실제 제약이다.** 2026-08-02 실측: bridge + 3카메라 +
YOLO-OBB 4 Hz = CPU `30.5%`, 여유 한계 `70%`. 정책이 쓸 수 있는 것은 그
차이이며, 양팔이면 bridge 트래픽이 두 배가 된다. **모델 크기를 그 안에서
정할 것.**

### 1.4 안전 반응 (전부 거부이며 완화 불가)

- [ ] deadline 초과 → reject
- [ ] 관측 stale → reject
- [ ] source skew → reject
- [ ] 출력 non-finite → reject
- [ ] 출력 범위 밖 → reject
- [ ] manifest 불일치 → refuse_start

---

## 2. 시뮬레이터가 반드시 모델링해야 하는 것

**여기가 sim2real 이 갈리는 곳이다.** 아래는 전부 2026-08-06 실기 실측이며,
모델링하지 않으면 정책이 전이되지 않는다.

### 2.1 중력 하 정상상태 오차 — 가장 중요

관절이 **명령한 위치에 도달하지 못하고 평형에서 멈춘다.** 추종 지연이
아니다. 14회 관측이 2685 ms 동안 1 raw 도 변하지 않았다 — 더 기다려도
가지 않는다.

| 상황 | SHOULDER 오차 |
|---|---|
| 접는 방향 상승 (1536 raw 이동) | 6 raw |
| 자세 유지 | 5~15 raw |
| **펼친 채 드는 방향 (259 raw 이동)** | **32 raw** |

- [ ] actuator 모델이 **P 제어의 정상상태 오차**를 낸다 (완전 추종 금지)
- [ ] 그 오차가 **자세와 중력 토크의 함수**다
- [ ] `randomize_actuator_gains` 로 stiffness/damping 을 랜덤화한다
      (기록된 실측: stiffness `17.8`, damping `0.60`, ±20%)

`1 raw = 2π/4096 = 0.001534 rad`. `32 raw ≈ 0.049 rad`, 반경 0.4 m 에서
약 `20 mm` 다.

### 2.2 작은 명령은 서보를 움직이지 못한다 — **정책 설계에 직결**

보정 leg 가 관절별로 `+4, -4, -18, -9, +8 raw` 를 명령했는데 **최대 이동이
6 raw** 였고, 문턱을 넘긴 `18 raw` 조차 **0 raw** 움직였다.

저장소의 독립 실측(`tools/execute_buffered_joint_delta_once.py`)도
`MINIMUM_OBSERVABLE_COMMAND_RAW = 16` 을 기록하고 있다.

**20 Hz 에서 `joint_position_delta` 를 내는 정책에 이것이 무슨 뜻인지 보라.**
관절을 `0.5 rad/s` 로 움직이려면 step 당 `0.025 rad = 16 raw` 다 —
정확히 사각지대다.

- [ ] actuator 모델에 **정지 마찰 / 사각지대**를 넣는다
- [ ] 정책의 action scale 이 사각지대를 넘도록 설계한다
- [ ] 또는 delta 를 누적해 내보내는 구조를 쓴다

이 항목을 무시하면 **시뮬레이터에서 완벽한 정책이 실기에서 아무것도 하지
않는다.**

### 2.3 관측 잡음 — 실측 예산 그대로

- [ ] `joint_pos`: ±`0.0015 rad` (raw 1카운트)
- [ ] Top 인식 위치: ±`0.010 m`, yaw: ±`5°` (VIS-001 실측 상한)
      - 실제 흔들림은 `0.77~1.65 mm` 로 훨씬 작지만, **정확도**는 위 값이다
- [ ] `enable_corruption = True` (현재 `False`)

### 2.4 관절 한계 — 실기 보정값을 쓸 것

- [ ] `config/single_arm_calibration.json` 의 raw 범위에서 유도한다
- [ ] **`wrist_roll` 이 raw `1874..2219` ≈ `±15°` 뿐이다.**
      Gate-3 `Q_GRASP` 는 `72.79°` 를 쓰는데 **실기에서 도달 불가**다.
      이 자세를 쓰는 정책은 통합할 수 없다
- [ ] `wrist_flex` 는 파지 자세에서 **상시 한계 근처**에 선다
      (nominal 1197, 하한 1194). 정책이 그 관절로 보정하려 하면 못 한다

---

## 3. 연속 명령 경로 — **가장 큰 미결 항목**

현재 물리 경로는 **buffered trajectory** 다. 전체 궤적을 미리 만들어
`FollowJointTrajectory` Action 으로 한 번 보내고, 펌웨어가 20 ms 간격으로
적용한다. **정책이 매 step 명령을 내는 구조가 아니다.**

정책은 20 Hz 로 `joint_position_delta` 를 낸다. 이 둘을 잇는 방법이 아직
정해지지 않았다.

- [ ] **RL 팀이 정해야 할 것**: 정책 출력이
      (a) step 당 delta 인가, (b) 짧은 궤적 조각인가?
- [ ] **우리가 정해야 할 것**: 스트리밍 setpoint 경로를 만들 것인가,
      아니면 정책 출력을 짧은 buffered leg 로 묶을 것인가?

이것이 정해지기 전에는 정책이 `SHADOW` 를 벗어날 수 없다. **양팔 진입 전에
반드시 합의해야 한다.**

---

## 4. 관측·행동 공간

### 4.1 관측

- [ ] 모드는 `structured_state` / `rgb_tensor` / `hybrid` 중 하나
- [ ] 카메라를 쓰면 순서는 `top, wrist_a, wrist_b` 고정
- [ ] **현재 env 는 `privileged_state`** 다 — 물체의 참 위치를 본다.
      실기에는 그런 것이 없다. 배포 과제와 다르다는 것이
      `rl_task_contract.json` 에 기록되어야 한다
- [ ] 손목 카메라를 관측에 넣을 계획이면 **C3 완료가 선행**이다
      (지금 optical frame 이 TF 트리에 없다)

### 4.2 행동

- [ ] 표현은 `joint_position_residual_rad` / `cartesian_residual_m_rad` /
      `arm_selection` 중에서만
- [ ] `order`, `scale`, `lower`, `upper` 의 **길이가 같아야** 한다
- [ ] `lower[i] < upper[i]`
- [ ] **`order` 는 project 이름 공간**이어야 한다
      (`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`).
      Isaac 이름으로 내면 거부된다

---

## 5. 과제별 추가

### 5.1 양팔 Pick/Place

- [ ] `arm_selection` 행동을 쓸 것인지, 두 팔을 동시에 낼 것인지 정할 것
- [ ] **두 팔의 처짐이 다르다.** 팔별 actuator 파라미터를 랜덤화할 것
- [ ] 한 팔 실패 시 정책이 어떻게 행동하는지 정의할 것
      (우리 쪽 조율된 중단이 개입하기 전 단계)
- [ ] 두 팔의 작업영역이 겹치는 구간에서 자기충돌을 학습으로 피할 것인지,
      계획기가 막을 것인지 정할 것 — **현재 planning scene 이 비어 있어
      MoveIt 은 자기충돌만 본다**

### 5.2 수건 접기

- [ ] **변형체(deformable) 시뮬레이션**이 필요하다. 현재 env 는 강체
      상자다. Isaac 의 deformable/particle 을 쓸 것인지, 근사할 것인지 결정
- [ ] 성공 판정을 정의할 것. 펜은 `lift_height + hold_time + distance` 로
      되지만 접힘은 그렇게 안 된다
- [ ] **테이블 위에서 접는다** (공중에서 양팔로 드는 것이 아니다).
      → 적재 하중은 작지만 **뻗은 낮은 자세가 상시 조건**이다.
      §2.1 의 `32 raw` 가 나온 자세가 정확히 그 영역이다
- [ ] 과제 허용치가 펜보다 **훨씬 헐거울 것**이다 (천은 유연하고 모서리는
      길다). 그러면 제약은 정밀도가 아니라 **안전 게이트**가 된다 —
      `POST_SETTLE_TOLERANCE_RAW = 30` 은 자세와 무관한 상수라
      정상 동작을 중단시킬 수 있다. 우리 쪽에서 자세별 게이트로 바꿔야 한다

---

## 6. 산출물 목록 (이대로 주면 바로 검증한다)

- [ ] `policy.onnx` — §1.1 만족
- [ ] `manifest.json` — 관측/행동 spec, opset, provenance 3종 SHA,
      `deployment_contract_sha256`, `safety` 객체(계약과 **byte-equal**)
- [ ] 학습 설정 파일 + 그 SHA
- [ ] export 스크립트 + 그 SHA
- [ ] 시뮬레이터 환경 버전 고정 (IsaacLab commit SHA, isaacsim / torch /
      warp-lang / rsl-rl-lib 해석된 버전)
- [ ] **§2 항목별로 "무엇을 어떻게 모델링했는가"** 를 적은 문서.
      값이 아니라 근거를 적을 것

검증:

```bash
python3 tools/validate_policy_deployment_bundle.py --bundle <경로>
```

`POLICY_DEPLOYMENT_BUNDLE_PASS` 가 나와야 한다.

---

## 7. 지금 당장 시작할 수 있는 것

우리 쪽 미결 항목(§0)을 기다리지 않고 할 수 있는 순서다.

1. Isaac Lab 환경 고정 + `config/isaac_lab_environment_contract.json`
2. `mapping.py` 관절 한계 3건 교정 + URDF/보정/PhysX 3자 교차 검증 시험
3. `wrist_roll ±15°` 제약 안에서 grasp 자세 재설계 (Gate-3 `Q_GRASP` 는 폐기)
4. §2 의 actuator 모델 — 정상상태 오차와 사각지대. **이것이 최우선이다**
5. 보상/성공/종료 항 (`mdp/success.py` 를 단일 출처로)
6. `EventCfg` 채우기 — 특히 `randomize_actuator_gains`
7. ONNX export 경로를 **더미 정책으로** 관통시켜 §1 검증 통과

**4번을 먼저 하라고 권한다.** 나머지가 아무리 잘 돼도 그것이 틀리면
전이되지 않고, 그 사실은 실기에 올려야만 드러난다.
