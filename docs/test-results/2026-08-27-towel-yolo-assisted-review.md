# Towel YOLO-assisted mask expansion — 2026-08-27

## 목적

개발 원본 595장 중 기존 사람 검수 train 103장을 제외한 모든 이미지를 현재
YOLO26n-seg baseline으로 pre-label하되, 자동 proposal을 학습 label로 승격하지
않고 사람 검수 workspace로 만든다.

## 입력과 분리

- source: `datasets/towel_yolo_source/20260826_top_01`
- 기존 reviewed train: 103장
- model SHA-256: `b4ad43b567146ef9f0e52cc61c1742166b602881ce319ad900dd8a60987d4beb`
- configuration: `imgsz=640`, detection confidence `0.25`, mask threshold `0.5`
- 기존 validation 35장: 입력에 포함하지 않음

## 결과

| 구분 | 수량 |
|---|---:|
| 선택된 미검수 원본 | 492 |
| LabelMe 학습 후보 | 444 |
| robot-occluded OOD overlay | 48 |
| YOLO miss 후 OpenCV fallback | 15 |
| high-priority review | 142 |
| medium-priority review | 60 |
| low-priority review | 242 |

출력은 `tmp/towel_yolo_assisted_review_r1/`에 있으며 annotation proposal 444개는
모두 schema 검사를 통과했다. `pilot_manifest.json`은
`training_labels_authorized=false`, `state_labels_authorized=false`,
`robot_occluded_training_labels_authorized=false`를 유지한다.

YOLO miss에는 손이 수건을 가리는 개발 frame이 포함돼 있다. 이 15장은 OpenCV
fallback polygon을 제공하지만 자동 승인하지 않고 high-priority review 또는
`review_rejected=true`가 필요하다.

## 사람 검수 closeout

LabelMe에서 444개 proposal을 전수 검수했다. 수건 상태 또는 가림이 심해 학습에
부적합한 7장은 `review_rejected=true`로 제외했고, 나머지 437장을 segmentation
학습 label로 승인했다.

```text
TOWEL_SEGMENTATION_LABELME_REVIEW_PASS reviewed=444 accepted=437 rejected=7 segmentation_labels_authorized=true state_labels_authorized=false robot_occluded_training_labels_authorized=false
```

기존 승인 train 103장과 합친 train annotation은 총 540장이다. 48개
robot-occluded frame은 계속 OOD overlay로만 유지하며 학습 label로 승인하지 않았다.

## 다음 gate

승인 train 540장과 기존 독립 validation 35장을 다시 export해 YOLO segmentation을
재학습한다. 기존 103장 baseline과 동일한 validation 및 이후 새 독립 test에서
성능을 비교하기 전에는 runtime backend로 승격하지 않는다.
