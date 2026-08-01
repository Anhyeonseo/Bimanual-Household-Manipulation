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

평가 계약 구현은 통과했지만 실제 최소 18장 dataset은 아직 수집하지 않았다.
따라서 `VIS-003`은 부분 통과다. 다음 gate는 고정 기하에서 positive 12장,
hard-negative 6장을 배경·조명·반사 조건별로 수집해 legacy 실패 artifact를
생성하는 것이다. 이후 같은 manifest로 명도+형상 backend와 필요 시 경량
ONNX backend를 비교한다.
