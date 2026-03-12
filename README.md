# IICC26 Workspace

이 워크스페이스의 현재 기준 실행 흐름은 `src/sjtu_drone-ros2` 아래 ROS 패키지들과 MATLAB `AutoSim.m` 조합이다.

## 구조

- `src/sjtu_drone-ros2/`: 실제 소스 저장소
- `build/`, `install/`, `log/`: colcon 산출물
- `src/Diagram/`: 문서/도식 자료

`src/sjtu_drone-ros2` 내부 활성 패키지:

- `sjtu_drone_bringup`
- `sjtu_drone_description`
- `sjtu_drone_control`
- `sjtu_drone_interfaces`
- `matlab/AutoSim.m`

## 빠른 시작

```bash
cd /home/j/INCSL/IICC26_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/j/INCSL/IICC26_ws/install/setup.bash
```

MATLAB 실행:

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')
```

## 정리 기준

- 현재 워크스페이스 문서는 AutoSim 중심으로 유지한다.
- install-space를 직접 가리키는 우회 경로보다 `install/setup.bash` 기반 실행을 우선한다.
- MATLAB 학습 모델, 로그, trace, plot은 생성 산출물로 본다.

## 참고 문서

- [src/sjtu_drone-ros2/README.md](src/sjtu_drone-ros2/README.md)
- [src/sjtu_drone-ros2/matlab/README.md](src/sjtu_drone-ros2/matlab/README.md)