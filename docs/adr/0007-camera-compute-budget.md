# ADR-0007: Raspberry Pi 카메라와 연산 자원 한도

- 상태: 채택(실제 policy로 재측정 필요)
- 날짜: 2026-07-12
- 갱신: 2026-08-01

## 결정

- 카메라 3대는 모두 Pi 5에 연결하되 카메라마다 압축된 최신 frame 한 장만
  유지하고 queue backlog를 만들지 않는다.
- Top은 전역 탐색, 왼쪽·오른쪽 손목은 각 팔의 마지막 상대 정렬을 담당한다.
- policy가 구조화 관측을 사용하면 phase scheduler가 현재 필요한 영상만
  decode·추론한다.
- 이미 학습된 policy가 세 RGB 입력을 요구하면 학습과 동일한 해상도,
  camera order, crop, normalization과 시간 정렬을 유지하고 실제 ONNX로
  자원 gate를 다시 통과한다.
- Isaac Sim/Isaac Lab 학습은 데스크탑에서 수행하고 Pi는 policy.onnx
  inference만 담당한다.
- debug raw image의 지속 DDS 전송과 전체 rosbag 기록은 기본 경로에서
  제외하고, 요청 시 저율 영상 또는 fault 전후 ring buffer만 기록한다.
- camera/perception/policy process는 control bridge와 분리한다.
- 오래된 frame, inference deadline miss, camera skew 또는 manifest mismatch가
  있으면 action을 폐기하고 motion을 차단한다.
- MoveIt은 task 경계의 전역 경로에 사용하고 policy 주기마다 전체 planning을
  반복하지 않는다.

## 현재 근거

- 세 UVC 카메라 MJPEG capture, hot-plug recovery와 phase별 decode가 구현됐다.
- RGB 3개 topic과 STM32 bridge 동시 부하에서 CPU 평균 6.38%,
  joint state 5.008 Hz, heartbeat 위반 0회를 기록했다.
- 위 결과에는 실제 detector ONNX, 실제 policy ONNX와 장시간 부하가 포함되지
  않으므로 최종 자원 수락 근거는 아니다.

## 초기 합격 기준

- 전체 CPU 평균 70% 이하, p95 85% 이하를 초기 목표로 측정
- swap-in/swap-out과 thermal throttling 0회
- 카메라 queue depth 1, stale observation action 0회
- policy inference p95는 policy period의 40% 이하
- 전체 observation-to-action p95는 policy period의 70% 이하
- camera/perception/policy 부하 중 STM32 heartbeat/feedback 오류 0회
- 카메라 한 대 장애 시 motion은 fail-closed하고 control bridge는 유지
- 30분 부하 시험 후 8시간 soak와 재부팅 반복 통과

## 채택 후 남은 측정

- 실제 UVC mode, stable identity와 USB topology
- 실제 policy 입력·출력·control_dt와 execution provider
- 실제 detector/policy ONNX p50/p95/max
- 세 카메라 observation age와 최대 timestamp skew
- CPU, memory, 온도, throttling과 fault recovery artifact
