# sjtu_drone for IICC26

이 저장소의 현재 기준 실행 경로는 ROS2 + Gazebo 시뮬레이션과 MATLAB `AutoSim.m` 자동 실험 파이프라인이다. 예전 MATLAB 단일 노드/수동 파이프라인 스크립트는 제거했고, 문서도 AutoSim 중심으로 정리했다.

## 현재 사용 구성

- `sjtu_drone_bringup`: Gazebo/AprilTag/bridge/teleop launch
- `sjtu_drone_description`: 드론 모델, world, Gazebo 플러그인, wind plugin
- `sjtu_drone_control`: teleop 노드
- `sjtu_drone_interfaces`: `SetWind` 서비스 정의
- `matlab/AutoSim.m`: 현재 실험 자동화 진입점

## 요구 환경

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- MATLAB R2024b 이상 권장

## 빌드

```bash
cd /home/j/INCSL/IICC26_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/j/INCSL/IICC26_ws/install/setup.bash
```

주의:

- launch는 `install/setup.bash` 기준 install-space 리소스를 사용한다.
- `src/` 아래 launch, xacro, Python 패키지 코드를 수정했으면 해당 패키지를 다시 빌드해야 반영된다.

## ROS 실행

헤드리스 기본 실행 예시는 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source /home/j/INCSL/IICC26_ws/install/setup.bash

ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
  use_gui:=false \
  use_rviz:=false \
  use_teleop:=false \
  use_apriltag:=true \
  apriltag_camera:=/drone/bottom \
  apriltag_image:=image_raw \
  apriltag_tags:=tags \
  apriltag_type:=umich \
  apriltag_bridge_topic:=/landing_tag_state
```

## 핵심 토픽

- `/wind_command`: 입력 풍속/풍향 명령
- `/wind_condition`: 풍속/풍향 관측 출력
- `/set_wind`: 풍속/풍향 서비스 인터페이스
- `/landing_tag_state`: MATLAB 호환 AprilTag bridge 출력
- `/drone/gt_pose`, `/drone/gt_vel`, `/drone/state`: AutoSim 주요 입력

## MATLAB 실행

현재 지원하는 MATLAB 진입점은 하나다.

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')
```

AutoSim은 다음을 수행한다.

- 시나리오별 launch/cleanup
- ROS 센서 수집
- 온톨로지 추론과 안정성 판단
- 모델 학습 및 스냅샷 저장
- 결과/추적 데이터 저장

상세 내용은 [matlab/README.md](matlab/README.md)를 본다.
연동 설계와 연구실 공유용 운영 기준은 [matlab/ROS2_Gazebo_MATLAB_Validation_Guideline.md](matlab/ROS2_Gazebo_MATLAB_Validation_Guideline.md)를 본다.

## 정리 원칙

- `build/`, `install/`, `log/`는 워크스페이스 산출물이다.
- `matlab/data`, `matlab/logs`, 학습 모델 스냅샷, 결과 plot은 생성 산출물로 취급한다.
- 현재 저장소는 AutoSim 기반 워크플로만 문서화한다.
