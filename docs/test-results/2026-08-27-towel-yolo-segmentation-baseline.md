# Towel YOLO segmentation baseline — 2026-08-27

## 범위

사람이 검수한 수건 polygon만 YOLO segmentation 형식으로 내보내고, RTX 5070 Ti에서
YOLO26n-seg baseline을 학습한 뒤 고정 confidence로 validation inference를 재실행했다.
robot-occluded 3장은 training label로 사용하지 않았고 실제 모터 명령은 없었다.

## 환경과 입력

- GPU: NVIDIA GeForce RTX 5070 Ti 16 GB
- NVIDIA driver: 595.84
- PyTorch: 2.11.0+cu128
- Ultralytics: 8.4.130
- model: `yolo26n-seg.pt`
- train/validation: 103/35장
- train/validation empty negative: 13/5장
- export item SHA: `7094cc0db5493889fca3d3a40c10cb1067c651a5ca16089e8f1c1508accb9b36`
- image size: 640
- batch: auto (`-1`)
- seed: 42, deterministic: true
- epochs: 100

학습 결과는 로컬 `tmp/towel_yolo_runs/yolo26n_seg_r0/`에 생성됐고 선택 weight는
`artifacts/models/towel_yolo26n_seg_r0/best.pt`에도 보존했다. SHA-256은
`b4ad43b567146ef9f0e52cc61c1742166b602881ce319ad900dd8a60987d4beb`다.
`artifacts/`와 `*.pt`는 Git ignore 대상이므로 이 문서는 weight 파일 자체를
Git에 보존하지 않는다. 팀 공유에는 별도 artifact storage, GitHub Release 또는
Git LFS가 필요하다.

## 학습 결과

`results.csv`에서 mask mAP50-95 최고값은 epoch 46의 `0.995`다. 100 epoch 마지막
값은 precision `0.99936`, recall `1.0`, mAP50 `0.995`, mAP50-95 `0.99077`이다.
학습 로그 기준 누적 시간은 약 83.8초였다.

## 고정-threshold validation inference

`best.pt`, `imgsz=640`, detection confidence `0.25`, mask threshold `0.5`로 35장을
다시 추론했다.

| 측정값 | 결과 |
|---|---:|
| 수건 영상 검출 | 30/30 |
| empty 영상 거절 | 5/5 |
| false negative | 0 |
| empty false positive | 0 |
| non-empty mask IoU 평균 | 0.979250436 |
| non-empty mask IoU 최저 | 0.942681973 |

재현 명령:

```bash
PYTHONPATH= .venv-yolo/bin/python \
  tools/run/evaluate_towel_yolo_segmentation.py \
  tmp/towel_yolo_runs/yolo26n_seg_r0/weights/best.pt \
  --output tmp/towel_yolo_segmentation_evaluation.json
```

## 판정

학습과 validation inference baseline 생성은 완료됐다. 다만 기존 고정-camera
segmentation backend의 같은 35장 IoU 평균/최저 `0.980284/0.965564`보다 YOLO가
각각 `0.001034/0.022882` 낮아 현재 결과만으로 기존 backend를 교체하지 않는다.
또한 이 validation은 매 epoch model 선택에 사용됐으므로, hyperparameter와 weight를
고정한 뒤 새 물리 재배치 test session으로 일반화 성능을 확인해야 runtime 후보로
승격할 수 있다.
