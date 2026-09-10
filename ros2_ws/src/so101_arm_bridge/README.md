# SO101 arm bridge

STM32가 제어하는 SO101 양팔의 protocol-v2 스트림을 ROS 2에 연결한다. 기존 resident 양팔 경로를 보존하고, 단일팔 v1 실행기와 고정 작업대용 명령 경로를 제거했다.

## 빌드

저장소 루트에서 ROS 2 Jazzy 환경을 불러온 뒤 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`rclpy`, `ament_index_python`, `launch_ros`, `sensor_msgs`, `std_srvs`, `trajectory_msgs`, ROS interface generators와 `python3-serial`이 필요하다. 팔 모델에는 `robot_state_publisher`와 `xacro`가 필요하다.

## 실행

```bash
ros2 launch so101_arm_bridge bimanual_stream.launch.py motion_authorized:=false
```

이 명령은 실제 장치와 통신하므로 여기서 자동 실행하지 않는다. 기본 설정에서는 새 위치 명령과 토크 활성화를 허용하지 않지만, 초기화는 연결된 펌웨어의 준비·토크 비활성화·상태 읽기 경로를 사용한다. 이미 다른 제어기가 동작 중인 장치에 연결하지 않는다.

기존 패키지 `single_arm_bridge`의 실행 명령은 사용할 수 없다. 배포할 때 오래된 overlay와 동시에 실행하지 않도록 한다.

## 인터페이스

- `~/command`: `BimanualStreamCommand`로 유한 경로, 연속 스트림, append·splice·stop 요청.
- `~/joint_states`, `/joint_states`: 실제 관절 위치.
- `~/feedback`: `BimanualJointFeedback`, 축별 측정 나이와 유효 축 표시.
- `~/status`: 상태 조회 서비스. 준비 상태·소유자·오류 정보는 구현의 응답을 따른다.

팔 브릿지는 모터 UART를 단독 소유한다. 베이스 이동, 카메라 인식, 작업 성공 판정은 상위 기능의 책임이다. 관절 피드백만으로 물체의 집기·놓기를 확인할 수 없다.

현재 어댑터는 펌웨어 `0x00024809`와 보존된 관절 한계를 요구한다. 이는 기존 펌웨어 계약이며 새 모바일 플랫폼의 실물 동작 승인이 아니다. 하드웨어 변경 시 명령·충돌·정지·관측 조건을 다시 확인한다.
