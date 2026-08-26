# 2026-08-26 Top-camera 수건 YOLO Segmentation 데이터셋

## 목적과 상태

이 데이터셋은 상단 카메라에서 수건을 하나의 `towel` 클래스로 분할하기 위한
원본 수집본이다. 수건의 펼침·구김·말림·접힘·로봇 가림과 수건이 없는 작업대를
포함한다. 폴더 이름은 수집 조건을 나타낼 뿐 YOLO 클래스가 아니다.

현재는 **원본 JPEG만 수집된 상태**다. polygon segmentation 라벨, train/validation/
test episode split, YOLO dataset YAML 및 학습 모델은 아직 만들지 않았다. 수건이
없는 사진은 라벨링 때 빈 annotation으로 처리한다.

## 저장 위치와 보존

| 위치 | 경로 | 용도 |
|---|---|---|
| Raspberry Pi 원본 | `/home/pi/SO101-Bimanual-Manipulation/datasets/towel_yolo_source/20260826_top_01/` | 카메라에 직접 연결된 Pi의 1차 원본 보관 |
| 노트북·Git 버전 | `datasets/towel_yolo_source/20260826_top_01/` | 라벨링·학습 준비 및 GitHub 보관본 |

Pi 원본은 Git push 후에도 삭제하지 않는다. 노트북 복사는 아래 명령으로
추가 촬영분만 안전하게 동기화할 수 있다.

```bash
rsync -av --progress --partial \
  pi@192.168.35.237:/home/pi/SO101-Bimanual-Manipulation/datasets/towel_yolo_source/20260826_top_01/ \
  datasets/towel_yolo_source/20260826_top_01/
```

## 수집 조건

- 카메라: Pi `pi5-chess`의 Top camera
- 캡처 경로: ROS 2 `/camera/top/image_raw` → Pi 현지 capture tool
- 해상도: JPEG `1280×960`, RGB 3-channel
- 카메라 manager 설정: MJPEG `1280×960 @ 30 fps`; 촬영 때 각 저장 프레임은
  settle-frame gate를 통과한다.
- 로봇 motion: 사진 수집 중 로봇 자동 동작은 사용하지 않았다.

## 세션 구성

세션 루트: `datasets/towel_yolo_source/20260826_top_01/`

| 폴더 | 의미 | JPEG 수 |
|---|---|---:|
| `00_empty_table` | 수건 없는 작업대, 그림자·비수건 물체 negative | 16 |
| `01_flat` | 전체가 보이는 펼친 수건 | 100 |
| `02_light_wrinkle` | 낮은 주름·가벼운 변형 | 120 |
| `03_heavy_wrinkle` | 큰 구김·불규칙 외곽 | 107 |
| `04_curled_or_overlapped` | 모서리 말림·국소 겹침 | 73 |
| `05_first_fold` | 1차 반 접기와 정렬 오차 | 81 |
| `06_second_fold` | 2차 접기와 다층 정렬 오차 | 50 |
| `07_robot_occluded` | 로봇/그리퍼 가림 | 48 |
| **합계** |  | **595** |

세션 크기는 약 98 MB다. `images/` 디렉터리는 이전 도구 호환을 위해 남아
있지만 이 세션에는 이미지가 없다.

## 다음 단계

1. 같은 수건 배치에서 나온 연속 사진이 서로 다른 split에 섞이지 않도록
   episode 단위 manifest를 만든다.
2. 모든 non-empty 이미지에 보이는 수건 외곽만 polygon으로 라벨링한다.
   프레임 밖으로 잘린 수건은 보이는 영역만 라벨링하며, 가려짐·겹침으로
   경계가 확정되지 않으면 `UNKNOWN` 검토 대상으로 남긴다.
3. empty-table 이미지는 빈 label 파일을 만들고, YOLO segmentation 형식과
   schema validation을 통과시킨다.
4. held-out session으로 mask quality, border-touch false positive와 robot
   occlusion 성능을 확인한 뒤에만 runtime perception에 연결한다.
