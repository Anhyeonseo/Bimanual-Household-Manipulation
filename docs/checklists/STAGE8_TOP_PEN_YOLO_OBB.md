# 단계 8 경량 YOLO-OBB 펜 검출 후보

## 목적

고정된 Top 카메라 기하와 물체 Z를 유지하면서 배경·조명·반사가 달라져도
검은 펜 1개의 중심과 무방향 장축 yaw를 안정적으로 구한다. 학습은
워크스테이션에서 수행하고 Pi 5에는 ONNX와 OpenCV DNN만 배포한다.

이 단계는 오프라인 후보 검증이다. 고정 holdout을 통과하기 전에는 ROS 실시간
노드에 연결하지 않으며 로봇 명령을 만들지 않는다.

## 현재 분기점

- 고정 holdout: 18장(positive 12, negative 6), annotation 승인 완료
- holdout 용도: 평가 전용, train/validation 사용 금지
- legacy 결과: miss 100%, false positive 66.7%
- 현재 작업: 별도 학습 데이터 수집 → YOLO11n-OBB 학습 → ONNX export
- 방향 정의: 펜 뚜껑/촉은 구분하지 않고 장축 yaw를 modulo pi로 평가

시연 환경 인식 강화 항목은 학습·export 전 기준 약 65%다. 이 문서의
학습 데이터 gate와 ONNX export가 통과하면 약 75%, 고정 holdout 합격 후
약 85%, Pi 자원 및 새 환경 재검증까지 끝나면 100%로 본다.

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

## 다음 gate

고정 holdout 합격 후에만 Pi 5에서 OpenCV DNN 4 Hz 자원 smoke test를 한다.
그 뒤 배경·조명이 다른 시연 후보 환경에서 새 이미지를 수집해 최종
재검증한다. 이 두 gate 전에는 기존 ROS detector를 교체하지 않는다.
