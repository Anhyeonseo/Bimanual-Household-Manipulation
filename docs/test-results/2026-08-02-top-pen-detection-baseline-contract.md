# Top 카메라 펜 검출 데이터 기준선 계약

- 날짜: 2026-08-02 KST
- 범위: offline dataset 평가, 로봇 이동 없음
- 대상 backend: `legacy_dark_threshold`
- 입력 해상도: 640x480
- motion authorization: false

## 구현

- `config/top_pen_detection_baseline_contract.json`에 current backend와
  coverage·acceptance 기준을 고정했다.
- `tools/evaluate_top_pen_detection_baseline.py`가 dataset manifest, 이미지,
  camera-info와 homography SHA-256을 검증한다.
- positive의 miss·중심 pixel·무방향 장축 yaw 오차와 hard-negative의
  false positive를 분리해 machine-readable JSON으로 출력한다.
- 절대 이미지 경로나 dataset 밖 경로를 허용하지 않고 robot command topic을
  생성하지 않는다.

## 재배치 프레임 재현

사용자가 제공한 `/tmp/top_relocated_check.png`를 ROS 노드와 같은 threshold,
면적, solidity, exclusion rectangle과 partial-footprint 설정으로 재평가했다.

```text
expected exactly 1 object intersecting the calibrated region,
detected 3 (ignored 1 fully outside)
```

영상 입력 실패가 아니라 대리석 무늬·그림자·검은 구조물이 기존 임계값
backend의 후보로 남는 문제를 재현했다. fail-closed이므로 pose나 로봇 target은
발행되지 않는다.

## 자동 시험

- 기준선 evaluator 단위 시험: 5/5 통과
- 기존 Top detector와 frame-age 회귀 포함: 20/20 통과
- 검증 항목: 합격 dataset, hard-negative false positive, 환경 coverage 부족,
  이미지 SHA mismatch, 180-degree 장축 yaw 정규화

## 상태와 다음 gate

평가 계약과 고정 holdout 18장 수집·annotation·legacy 실패 artifact 생성을
완료했다. Legacy 결과는 miss 100%, false positive 66.7%다. `VIS-003`은 새
backend가 아직 기준을 통과하지 않아 부분 통과다. 다음 gate는 holdout을
학습에 섞지 않고 별도 학습 데이터로 경량 YOLO-OBB를 학습·ONNX export한
뒤 같은 manifest로 비교하는 것이다. 상세 결과는
`2026-08-02-top-pen-holdout-legacy-baseline.md`에 기록했다.
