# Pi 5 3카메라·Top YOLO-OBB 30분 자원 기준선

- 날짜: 2026-08-02 KST
- 대상: Raspberry Pi 5 4GB, USB MJPEG 카메라 3대
- camera phase: `RUNTIME_BASELINE`
- Top detector: YOLO11n-OBB 320 px, OpenCV DNN 4.10.0.84
- 측정: warm-up 10초 + 본시험 1,800초
- robot/bridge/policy/MoveIt: 미실행
- 원본 artifact SHA-256:
  `d42884b0dd89e67fc9279da03c45cad32cbf5b515a4b21d1ef585311e9fce0c7`

## 목적과 안전 경계

배경·조명·반사 holdout을 통과한 Top OBB 후보를 Pi에 배포하고, Top과 양
손목 카메라를 함께 디코딩하면서 실제 ONNX 추론 4 Hz와 Pi 자원 안정성을
확인했다. 측정 도구는 `/camera_phase`만 발행했다.
`motion_authorized=false`, `robot_target_available=false`, robot command
publisher와 Top perception command publication은 모두 0이다.

## 모델 고정값과 인식 결과

| 항목 | 결과 |
|---|---:|
| model SHA-256 | `5fd4d014a3be8fb8ebaf09d601616e30e94a2956424acffe3431b40e52f670db` |
| holdout manifest SHA-256 | `da7ea8a03a264ea798b049dc00ae0579517da1f6cfa59e92c9e6998c8dcbf7f2` |
| 실측 추론률 | 3.9888 Hz |
| 추론 p50/p95/max | 80.02/86.95/95.20 ms |
| 성공 observation | 7,180개 |
| detection rejection | 0개 |
| processing/input 오류 | 0개 |
| command publication | 0개 |

시험 장면에서 진단은 30분 동안
`TRACKING_CENTER_CALIBRATED_FULLY_VISIBLE`를 유지했다.

## 카메라 결과

| 카메라 | 내부 decode 목표/실측 | inference 목표 | DDS subscriber 실측 | frame age p95 | decode p95 | 실패/reconnect |
|---|---:|---:|---:|---:|---:|---:|
| Top | 6.000/6.000 Hz | 4.000 Hz | 6.000 Hz | 32.53 ms | 4.73 ms | 0/0 |
| Wrist A | 5.000/5.000 Hz | 4.000 Hz | 3.650 Hz | 27.67 ms | 8.70 ms | 0/0 |
| Wrist B | 5.000/5.000 Hz | 4.000 Hz | 3.804 Hz | 23.41 ms | 7.61 ms | 0/0 |

내부 decode는 세 카메라 모두 목표를 정확히 유지했다. DDS subscriber
관측률은 inference 목표의 90% 하한을 적용하며 Wrist A는 하한 3.600 Hz보다
0.050 Hz 높은 3.650 Hz로 통과했다. nominal 4 Hz보다 낮고 여유가 작으므로,
향후 실제 손목 perception과 policy shadow를 함께 실행하는 단계에서는 실제
소비 노드별 rate를 다시 측정한다. 이 수치를 4 Hz 달성으로 표현하지 않는다.

## Raspberry Pi 자원

| 지표 | 결과 | 기준 | 판정 |
|---|---:|---:|---|
| CPU 평균/p95 | 35.07% / 37.38% | 평균 70% 이하 | 통과 |
| CPU 1초 최대 | 43.14% | 90% 미만 | 통과 |
| memory 사용 최대 | 565.01 MB | 3,000 MB 이하 | 통과 |
| memory 가용 최소 | 3,418.95 MB | 700 MB 이상 | 통과 |
| CPU 온도 평균/최대 | 46.46°C / 50.15°C | 80°C 미만 | 통과 |
| swap in/out delta | 0/0 | 0/0 | 통과 |
| throttling flags OR | `0x00000000` | 0 | 통과 |
| camera manager RSS 평균/최대 | 25.11/25.62 MB | 기록 | 통과 |
| Top OBB RSS 평균/최대 | 124.93/125.52 MB | 기록 | 통과 |

## 판정과 남은 범위

- 3카메라 + Top YOLO-OBB 4 Hz 무동작 30분 gate: **통과**
- 고정 holdout v2의 miss 0%, false positive 0%와 이 실시간 gate를 합쳐
  시연 환경 인식 강화 구간: **100%**
- 카메라 기하, 높이와 물체 Z는 고정 범위다. 배경·조명·반사 변화만 현재
  재현성 계약에 포함한다.
- Wrist A DDS rate의 nominal 4 Hz 대비 부족은 후속 policy shadow 통합
  gate의 명시적 재검증 항목이다.
- 실제 정책 ONNX, 손목 perception, STM32/양팔 동시 운용과 8시간 soak는
  이 결과에 포함되지 않는다.

## 증거

- `artifacts/stage9/2026-08-02/pi_runtime_top_obb_30m.json`
- `tools/pi_runtime_resource_baseline.py`
- `config/camera_schedule.json`
- `artifacts/stage8/top_pen_yolo_obb_candidate_v3_finetune_2026-08-02/top_pen_yolo11n_obb.onnx`
- `artifacts/stage8/top_pen_dataset_v2/manifest.json`
