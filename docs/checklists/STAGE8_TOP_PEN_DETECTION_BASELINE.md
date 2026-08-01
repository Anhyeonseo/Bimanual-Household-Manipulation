# 단계 8 Top 카메라 펜 검출 데이터 기준선

## 목적

카메라 각도·높이, 작업대–base 기하와 물체 Z는 고정하고 배경·조명·반사만
달라지는 집/시연 환경에서 펜 검출 성능을 같은 데이터 계약으로 비교한다.
이 체크리스트는 로봇 명령을 발행하지 않으며 detector 선택과 튜닝 전에
기존 `legacy_dark_threshold` backend의 실패 양상을 수치로 고정한다.

## 안전 및 고정 조건

- Bridge와 MoveIt 실행은 필요하지 않다. 12V는 OFF로 유지해도 된다.
- Top 카메라 mount, 높이, 해상도, focus와 exposure 설정을 바꾸지 않는다.
- 펜이 놓이는 평면의 Z와 기존 camera-info/homography 파일을 바꾸지 않는다.
- mount나 물체 Z가 바뀌면 기존 dataset에 추가하지 않고 재보정한다.
- 평가기는 ROS publisher를 만들지 않으며 `motion_authorized=false`만 기록한다.

## 데이터 구성

최소 18장을 수집한다.

| 종류 | 최소 수 | 내용 |
|---|---:|---|
| Positive | 12 | 펜 1개, 위치·yaw를 바꾼 영상 |
| Hard negative | 6 | 펜 없음, 검은 물체·무늬·그림자·반사는 유지 |

전체 dataset에는 배경 label 2개 이상, 조명 label 3개 이상,
반사 label 2개 이상이 있어야 한다. 같은 장면의 연속 프레임만 늘려 수량을
채우지 말고 펜 위치·방향과 방해 요소를 실제로 바꾼다.

권장 label 예시는 다음과 같다.

- `background`: `home_marble`, `wood`, `demo_candidate`
- `lighting`: `normal`, `dim`, `bright_side`
- `glare`: `none`, `present`

## 프레임 수집

워크스테이션에서 camera manager와 ROS overlay를 source한 뒤 실행한다.

```bash
python3 tools/capture_top_frame.py \
  --output artifacts/stage8/top_pen_dataset/images/positive_001.png
```

각 이미지의 SHA-256을 기록한다.

```bash
sha256sum artifacts/stage8/top_pen_dataset/images/*.png
```

manifest는 dataset 디렉터리 안에 두며 image 경로는 manifest 기준 상대 경로만
허용한다. Positive case는 수동 기준 중심 pixel과 펜의 무방향 장축 yaw를
기록한다. yaw는 `-90 <= yaw < 90` degree 범위다. 이 yaw는 180도 대칭인
`modulo pi`이며, 뚜껑/촉 방향은 현재 Pick & Place 계약에 포함하지 않는다.

```json
{
  "protocol_version": 1,
  "geometry": {
    "camera_info_sha256": "<top_camera_info.yaml SHA-256>",
    "homography_sha256": "<top_worktable_homography.yaml SHA-256>"
  },
  "cases": [
    {
      "id": "home-marble-normal-positive-001",
      "image": "images/positive_001.png",
      "image_sha256": "<PNG SHA-256>",
      "expected_present": true,
      "expected_center_px": [301.5, 352.0],
      "expected_yaw_deg": 88.0,
      "condition": {
        "background": "home_marble",
        "lighting": "normal",
        "glare": "present"
      }
    },
    {
      "id": "home-marble-normal-negative-001",
      "image": "images/negative_001.png",
      "image_sha256": "<PNG SHA-256>",
      "expected_present": false,
      "condition": {
        "background": "home_marble",
        "lighting": "normal",
        "glare": "present"
      }
    }
  ]
}
```

## 평가

```bash
python3 tools/evaluate_top_pen_detection_baseline.py \
  --manifest artifacts/stage8/top_pen_dataset/manifest.json \
  --contract config/top_pen_detection_baseline_contract.json \
  --camera-info ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml \
  --homography ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml \
  --output artifacts/stage8/top_pen_detection_legacy_baseline.json
```

합격 기준은 다음과 같다.

- miss rate 5% 이하
- false-positive rate 2% 이하
- 검출된 positive의 중심 오차 p95 8 px 이하
- 무방향 장축 yaw 오차 p95 5 degree 이하
- 입력 이미지, camera-info, homography SHA 불일치 0건
- 평가 artifact의 `robot_command_topics_created=0`

현재 임계값 backend가 실패해도 기준선 작업 자체의 실패는 아니다. 실패
artifact가 다음 명도+형상 또는 경량 ONNX backend를 같은 입력으로 비교하는
근거가 된다. `VIS-003`은 새 backend가 이 dataset과 시연 장소 재검증을 모두
통과하기 전까지 완료로 바꾸지 않는다.
