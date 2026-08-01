# Top 펜 holdout 수집·legacy 기준선 결과

- 날짜: 2026-08-02 KST
- 범위: 고정 Top 카메라의 offline 인식 평가
- 로봇 명령·이동: 없음
- 대상 backend: `legacy_dark_threshold`
- 입력 해상도: 640x480

## 데이터 구성

카메라 각도·높이와 작업대–base 기하는 고정하고, 배경·조명·반사만 바꿨다.

- 배경 2종: 깨끗한 대리석 무늬, 방해물이 있는 대리석 무늬
- 조명 3종: 정상, 어두움, 측면광
- positive 12장: 각 배경·조명 조합마다 펜 위치 2개
- hard-negative 6장: 각 배경·조명 조합마다 펜 제거 1장
- 측면광 조건은 반사 있음으로 기록
- 추가 극저조도 1장은 acceptance에서 제외한 stress sample로 보존

핵심 18장은 모두 서로 다른 SHA-256이며, 학습 데이터가 아닌 고정
holdout이다. 이후 detector를 학습할 때 이 이미지들을 train/validation에
섞지 않는다.

## 정답 annotation

positive 정답은 같은 조건의 펜 제거 영상과의 차영상으로 후보를 만든 뒤
12장 모두 수동 검토했다. 사용자가 green box와 red center가 모두 정상임을
확인했다.

- 위치 정답: 펜 중심 pixel
- 회전 정답: 펜의 **무방향 장축**, `yaw modulo pi`
- 파란 표시는 장축의 양방향을 뜻하며 뚜껑 방향을 뜻하지 않음
- 뚜껑/촉 방향 label: 없음 (`cap_direction_labeled=false`)

현재 Pick & Place는 펜 중심과 장축만 필요하며 180도 반대 접근도 같은
파지이므로 앞뒤를 구분하지 않는다. 향후 뚜껑 방향이 task에 필요하면 현재
yaw를 재해석하지 않고, 별도의 tip/cap keypoint 또는 방향성 head와 별도
평가 계약을 추가한다.

## Legacy 결과

기존 명도 임계값 backend는 고정 holdout에서 예상대로 실패했다.

| 지표 | 결과 | 계약 기준 |
|---|---:|---:|
| positive miss | 12/12, 100% | 최대 5% |
| hard-negative false positive | 4/6, 66.7% | 최대 2% |
| processing error | 2 | 0 |
| center error p95 | 산출 불가 | 최대 8 px |
| yaw error p95 | 산출 불가 | 최대 5 deg |

positive마다 후보가 여러 개 남거나 이미지 안전 여백에 닿는 큰 contour가
선택돼, 노드는 fail-closed로 pose를 발행하지 않았다. 이는 영상 수집이나
카메라 연결 실패가 아니라 대리석 무늬·반사·방해물에 대한 legacy detector의
일반화 부족이다.

## 무결성

- dataset manifest SHA-256:
  `d967e7d2a9a271fab5b02a9a2733c047bad82b2b4983485644214600efdf9511`
- annotation review SHA-256:
  `36d284f2a8187c75d0ac539cdd3385807d0bd7e9814e68f97c584b56b90348a1`
- legacy 결과 SHA-256:
  `e2209dcc0826ca30e3b99861d322906c9a1da9294439a086f818dd963d75eb13`
- `SHA256SUMS` SHA-256:
  `e85f3ba90f7d2e26cad5cfd50dcc25975ee2e44663224d9bdd793aa346ac2d18`

## 판정과 다음 gate

holdout 수집·annotation·legacy 특성화는 통과했다. `VIS-003` 전체는 강건한
backend가 아직 계약을 통과하지 않았으므로 부분 통과다. 다음 이슈에서는
별도 학습 데이터로 경량 YOLO-OBB를 학습하고 ONNX로 export한 뒤, 이 고정
holdout 18장에 한해서 legacy와 miss·false positive·center·yaw를 비교한다.

