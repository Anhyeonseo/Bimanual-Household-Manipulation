# Towel YOLO segmentation expanded training — 2026-08-27

## 입력과 조건

- model: `yolo26n-seg.pt`
- reviewed train: 540장(기존 103 + assisted 승인 437)
- independent validation: 35장(수건 30 + empty 5)
- excluded assisted review: 상태 불량 7장
- robot-occluded OOD: 학습 label로 사용하지 않음
- export item SHA-256: `3ec69ca4f729c1f768671690ee8fd7ffd15ba28bc998a88b1f27ad6dca5c10ac`
- `epochs=100`, `imgsz=640`, `batch=-1`, `seed=42`, `deterministic=true`
- GPU: NVIDIA GeForce RTX 5070 Ti

학습은 302.157초에 완료됐다. 마지막 epoch의 mask precision/recall은
`0.99837/1.0`, mask mAP50/mAP50-95는 `0.995/0.995`다. 선택 weight는
`artifacts/models/towel_yolo26n_seg_expanded_r1/best.pt`에 보존했으며 SHA-256은
`834db2d3a76c7261b8847e9260d9cf20495a5f2209ec3470b8ea1f02d40e7aa9`다. 팀원이
동일 weight로 바로 검증할 수 있도록 이 canonical `best.pt`만 Git에 포함하고,
나머지 epoch weight와 전체 학습 run은 제외한다.

## 고정-threshold validation

`best.pt`, `imgsz=640`, detection confidence `0.25`, mask threshold `0.5`로 기존
baseline과 동일한 35장을 평가했다.

| 측정값 | 103장 baseline | 540장 expanded |
|---|---:|---:|
| 수건 영상 검출 | 30/30 | 30/30 |
| empty 영상 거절 | 5/5 | 5/5 |
| non-empty mask IoU 평균 | 0.979250436 | 0.980166482 |
| non-empty mask IoU 최저 | 0.942681973 | 0.966108314 |

평균 IoU는 `+0.000916046`, 최저 IoU는 `+0.023426341` 개선됐다. 가장 큰 개선은
`curled_or_overlapped_0004`의 `0.942681973 → 0.968820735`다. 30개 non-empty
frame 중 14개는 개선되고 16개는 소폭 낮아졌으므로 모든 frame이 일괄 개선된 것은
아니다.

기존 OpenCV backend의 평균/최저 `0.980284/0.965564`와 비교하면 expanded YOLO는
평균이 `0.000118` 낮고 최저가 `0.000544` 높아 현재 validation에서는 사실상
동률이다. 새 물리 재배치 test session과 실시간 카메라 검증 전에는 runtime
backend를 교체하지 않는다.
