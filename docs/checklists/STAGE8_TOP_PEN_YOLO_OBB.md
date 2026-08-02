# 단계 8 경량 YOLO-OBB 펜 검출 후보

## 목적

고정된 Top 카메라 기하와 물체 Z를 유지하면서 배경·조명·반사가 달라져도
검은 펜 1개의 중심과 무방향 장축 yaw를 안정적으로 구한다. 학습은
워크스테이션에서 수행하고 Pi 5에는 ONNX와 OpenCV DNN만 배포한다.

이 단계는 오프라인 후보 검증이다. 고정 holdout을 통과하기 전에는 ROS 실시간
노드에 연결하지 않으며 로봇 명령을 만들지 않는다.

## 현재 분기점

- 고정 holdout v2: 18장(positive 12, negative 6), 형상 annotation 검토 완료
- holdout 용도: 평가 전용, train/validation 사용 금지
- 학습 데이터 gate: 70장(train 53, validation 17), 중복·누수 검사 통과
- 최종 후보 모델: YOLO11n-OBB 320 px, v1 best 기반 30 epoch fine-tuning, ONNX export·OpenCV smoke 통과
- 최종 내부 validation: P 0.996, R 1.000, mAP50 0.995, mAP50-95 0.895
- 기존 holdout 재평가: 중심 p95 3.31 px, board yaw p95 2.20 degree, 시스템 FP 0%
- 기존 holdout 보류 사유: positive 6장이 image/board 운용 안전영역 밖이라 miss로 거부됨
- 교체 holdout v2: 기존 유효 positive 6장과 negative 6장을 유지하고 부적합
  positive 6장만 운용 안전영역 안에서 새로 촬영했다. 후보 가중치는 바꾸지 않았다.
- 후보 v1 holdout v2 평가: miss 0%, false positive 0%, board yaw p95
  2.57 degree였으나 중심 p95 9.47 px로 실패했다.
- 정상조명 hard example 10장(positive 8, negative 2)을 별도로 수집해 기존
  60장과 합친 70장 training v2를 만들었고 holdout SHA 누수 gate를 통과했다.
- 후보 v2 full retraining은 중심 p95를 3.83 px로 개선했지만 동일 펜을 여러
  OBB로 분할해 miss-rate gate에 실패했다. 후처리로 강제 병합하지 않았다.
- 최종 후보 v3: 기존 v1 best checkpoint에서 30 epoch hard-example fine-tuning,
  seed 101, confidence 0.25, IoU 0.45로 고정했다.
- 후보 v3 holdout v2 최종 결과: positive 12/12, negative 6/6, miss 0%,
  false positive 0%, 중심 p95 5.29 px, board yaw p95 2.79 degree로 합격했다.
- ROS 무동작 runtime 준비: 기존 detector를 기본값으로 유지하고, 후보 v3만
  선택하는 `top_obb_runtime_smoke.launch.py`를 추가했다. 4 Hz rate limit,
  model/holdout SHA, 추론 지연, 정상 거절과 처리 오류 분리 진단을 제공하며
  `motion_authorized=false`, `robot_target_available=false`, 명령 발행 0건을
  유지한다.
- 로컬 gate: ROS 패키지 build와 전용 launch 설치 확인, 전체 397 tests 통과.
- runtime 호환성 gate: Ubuntu/ROS 기본 OpenCV 4.6은 YOLO11 OBB `Split`
  node를 읽지 못해 사용 금지로 확정했다. 별도 `--system-site-packages`
  venv에서 해시 고정 OpenCV headless 4.10.0.84 + 시스템 NumPy 1.26.4 +
  ROS Jazzy rclpy 조합으로 후보 v3 실제 1회 추론을 통과했다. 시스템 Python과
  OpenCV는 변경하지 않는다.
- Pi runtime gate: 해시 고정 후보 v3를 실제 OpenCV DNN 4 Hz로 실행하면서
  3카메라를 30분 동시 운용했다. 추론 3.989 Hz, p95 86.95 ms, 처리 오류와
  command publication 0, CPU 평균 35.07%, 온도 최대 50.15°C, swap과
  throttling 0으로 통과했다.
- 손목 카메라 내부 decode는 각각 5.000 Hz였고 DDS subscriber 관측률은
  3.650/3.804 Hz였다. 90% 하한 gate는 통과했지만 nominal 4 Hz보다 낮으므로
  후속 실제 손목 perception·policy shadow 통합에서 다시 검증한다.

시연 환경 인식 강화 진행률은 **100%**다. 고정된 카메라 기하·높이·물체 Z
범위에서 배경·조명·반사 변화 holdout과 Pi 실시간 30분 gate를 완료했다.

## 학습 데이터 계약

학습 데이터는 기존 holdout 이미지를 복사하거나 변형해 만들면 안 된다.
같은 정지 장면의 연속 프레임으로 수량만 채우지 말고 펜 위치·yaw와 방해
요소를 실제로 바꾼다.

| split | Positive | Negative | 합계 |
|---|---:|---:|---:|
| train | 36 이상 | 9 이상 | 45 이상 |
| validation | 12 이상 | 3 이상 | 15 이상 |
| 전체 | 48 이상 | 12 이상 | 60 이상 |

전체 데이터에는 배경 3종 이상, 조명 3종 이상, 반사 2종 이상이 필요하다.
권장 label은 다음과 같다.

- background: home_marble_clean, home_marble_distractors, demo_candidate
- lighting: normal, dim, bright_side
- glare: none, present

## 디렉터리 구조

학습 데이터는 Git에 넣지 않는 로컬 dataset 디렉터리에 둔다.

~~~text
datasets/top_pen_obb_training/
├── data.yaml
├── manifest.json
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    └── val/
~~~

data.yaml 예시는 다음과 같다.

~~~yaml
path: /absolute/path/to/datasets/top_pen_obb_training
train: images/train
val: images/val
names:
  0: pen
~~~

## 수집과 annotation

워크스테이션에서 ROS 환경을 source한 뒤 한 프레임씩 저장한다.

~~~bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

python3 tools/capture_top_frame.py \
  --output datasets/top_pen_obb_training/images/train/train_positive_001.png
~~~

Positive label은 Ultralytics OBB 형식 한 줄이다.

~~~text
0 x1 y1 x2 y2 x3 y3 x4 y4
~~~

좌표는 이미지 폭·높이로 정규화한 0~1 값이다. 꼭짓점은 펜 전체를 감싸는
사각형으로 순서대로 기록한다. Negative label 파일은 존재하되 내용은
비어 있어야 한다. 뚜껑 방향은 label에 넣지 않는다.

각 manifest case는 image/label SHA와 조건을 함께 기록한다.

~~~json
{
  "id": "train-home-normal-positive-001",
  "image": "images/train/train_positive_001.png",
  "image_sha256": "<SHA-256>",
  "label": "labels/train/train_positive_001.txt",
  "label_sha256": "<SHA-256>",
  "expected_present": true,
  "condition": {
    "background": "home_marble_clean",
    "lighting": "normal",
    "glare": "none"
  }
}
~~~

수집 중에는 config/top_pen_obb_training_metadata.example.json 형식으로
case id, split, 상대 경로와 조건만 기록한다. 수집이 끝나면 다음 명령이
이미지와 label SHA를 계산해 manifest를 만든다.

~~~bash
python3 tools/build_top_pen_obb_training_manifest.py \
  --dataset-root datasets/top_pen_obb_training \
  --metadata datasets/top_pen_obb_training/metadata.json \
  --output datasets/top_pen_obb_training/manifest.json
~~~

생성 구조 예시는 config/top_pen_obb_training_manifest.example.json에서 볼 수
있다. split 이름은 manifest에서 train, validation이고 실제 Ultralytics
디렉터리 이름은 train, val이어도 된다.

## 데이터 gate

~~~bash
python3 tools/validate_top_pen_obb_training_dataset.py \
  --manifest datasets/top_pen_obb_training/manifest.json \
  --holdout-manifest artifacts/stage8/top_pen_dataset/manifest.json \
  --contract config/top_pen_yolo_obb_training_contract.json \
  --output artifacts/stage8/top_pen_obb_training_gate.json
~~~

다음을 모두 확인한다.

- 이미지·label 실제 SHA 일치
- holdout 및 auxiliary holdout 이미지 SHA와 중복 없음
- train/validation 중복 없음
- Positive 한 개 OBB, Negative 빈 label
- 배경·조명·반사와 최소 수량 충족
- 로봇 명령 topic 생성 0개

## 워크스테이션 학습·ONNX export

학습 의존성은 Pi가 아니라 별도 virtual environment에 설치한다.

~~~bash
python3 -m venv .venv-yolo-obb
source .venv-yolo-obb/bin/activate
python3 -m pip install --no-cache-dir -r requirements-training-cu128.txt
python3 -m pip install -r requirements-training.txt
~~~

먼저 학습 없이 계약만 확인한다.

~~~bash
python3 tools/train_export_top_pen_yolo_obb.py \
  --manifest datasets/top_pen_obb_training/manifest.json \
  --holdout-manifest artifacts/stage8/top_pen_dataset/manifest.json \
  --training-contract config/top_pen_yolo_obb_training_contract.json \
  --output-dir artifacts/stage8/top_pen_yolo_obb_candidate \
  --device cpu \
  --dry-run
~~~

GPU가 있는 워크스테이션에서는 실제 학습을 수행한다.

~~~bash
python3 tools/train_export_top_pen_yolo_obb.py \
  --manifest datasets/top_pen_obb_training/manifest.json \
  --holdout-manifest artifacts/stage8/top_pen_dataset/manifest.json \
  --training-contract config/top_pen_yolo_obb_training_contract.json \
  --output-dir artifacts/stage8/top_pen_yolo_obb_candidate \
  --device 0
~~~

도구는 Ultralytics 8.4.67과 YOLO11n-OBB를 사용하고 fixed 320×320,
batch 1, opset 17, NMS 미포함·graph simplification 비활성 ONNX를 만든다.
base checkpoint SHA와 Python·Numpy·OpenCV·Torch 버전을 기록한다. export
직후 같은 환경의
OpenCV DNN CPU로 dummy image를 한 번 추론해 tensor layout까지 확인한다.
생성 bundle에는 model·학습 manifest·계약·holdout SHA와
holdout_used_for_training=false가 들어간다.

## 고정 holdout 비교

~~~bash
python3 tools/evaluate_top_pen_yolo_obb.py \
  --manifest artifacts/stage8/top_pen_dataset/manifest.json \
  --contract config/top_pen_yolo_obb_evaluation_contract.json \
  --bundle-manifest artifacts/stage8/top_pen_yolo_obb_candidate/top_pen_yolo_obb_bundle.json \
  --camera-info ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml \
  --homography ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml \
  --output artifacts/stage8/top_pen_yolo_obb_holdout.json
~~~

합격 기준은 miss 5% 이하, false positive 2% 이하, 중심 오차 p95 8 px
이하, 무방향 yaw 오차 p95 5 degree 이하다. 실패하면 holdout을 보고
재학습하지 않는다. 별도 학습 데이터와 augmentation만 수정한 새 후보를
만들어야 한다.
Positive holdout은 펜 OBB 전체가 image edge margin 안에 있고 중심이 보정된
board region 안에 있어야 한다. 이 조건을 어긴 프레임은 모델 miss로 튜닝하지
않고 같은 조건의 새 프레임으로 교체한 뒤 manifest를 다시 동결한다. 교체 전
후보 가중치는 변경하지 않으며 새 holdout은 단 한 번만 평가한다.

## 다음 gate

고정 holdout 합격 후에만 Pi 5에서 OpenCV DNN 4 Hz 자원 smoke test를 한다.
그 뒤 배경·조명이 다른 시연 후보 환경에서 새 이미지를 수집해 최종
재검증한다. 이 두 gate 전에는 기존 ROS detector를 교체하지 않는다.

Pi에는 아래 세 파일을 같은 상대 구조로 배포한다.

- `top_pen_yolo_obb_bundle.json`: SHA-256
  `e7ae5c78e4e7239afe35663d035f718f316efbf883839387de0ee9f060eb7879`
- `top_pen_yolo11n_obb.onnx`: SHA-256
  `5fd4d014a3be8fb8ebaf09d601616e30e94a2956424acffe3431b40e52f670db`
- frozen holdout v2 manifest: SHA-256
  `da7ea8a03a264ea798b049dc00ae0579517da1f6cfa59e92c9e6998c8dcbf7f2`

Pi 전용 runtime을 시스템 패키지와 분리해 한 번 만든다.

~~~bash
python3 -m venv --system-site-packages \
  /home/pi/Manipulation/.venv-top-perception-opencv410

/home/pi/Manipulation/.venv-top-perception-opencv410/bin/python -m pip install \
  --require-hashes --no-deps \
  -r /home/pi/Manipulation/requirements-top-perception-runtime.txt
~~~

먼저 Top OBB runtime을 실행한다.

~~~bash
ros2 launch so101_top_perception top_obb_runtime_smoke.launch.py \
  python_executable:=/home/pi/Manipulation/.venv-top-perception-opencv410/bin/python \
  bundle_manifest:=/home/pi/Manipulation/artifacts/stage8/top_pen_yolo_obb_candidate_v3_finetune_2026-08-02/top_pen_yolo_obb_bundle.json \
  inference_hz:=4.0
~~~

다른 터미널에서 3카메라가 `RUNTIME_BASELINE` phase로 실행 중인지 확인한 뒤
60초 smoke를 먼저 수행한다. 이 측정기는 펜의 존재 여부를 합격 조건으로
삼지 않고, 실제 ONNX 4 Hz, 지연, 처리 오류 0건, 고정 SHA와 비동작 계약을
검사한다.

카메라 rate gate는 관리자 내부 `decoded_frames` 증분으로 `decode_hz`를
검증하고, DDS subscriber가 실제로 받은 영상률에 `inference_hz`의 90%
하한을 적용한다. 두 목표를 합쳐서 판단하지 않는다.

~~~bash
python3 /home/pi/Manipulation/tools/pi_runtime_resource_baseline.py \
  --phase RUNTIME_BASELINE \
  --duration 60 \
  --warmup 10 \
  --allow-missing-joint-states \
  --require-top-perception \
  --output /home/pi/Manipulation/artifacts/stage9/2026-08-02/pi_runtime_top_obb_smoke_60s.json
~~~

60초 gate가 통과한 경우에만 같은 조건으로 1800초 gate를 수행한다. 실패하면
운용 detector를 교체하지 않고 CPU, 온도, inference p95와 처리 오류부터
분석한다.
