# sjtu_drone for IICC26

이 저장소는 드론 착륙 안전성 연구를 위한 ROS2/Gazebo 실행 계층과 MATLAB AutoSim 분석 계층을 함께 제공한다.

## 아키텍처 개요

- 실행 계층(ROS2/Gazebo): 물리 시뮬레이션, 센서/토픽, 풍속 외란 플러그인
- 판단 계층(MATLAB): 온톨로지+AI 융합 판단, 정책 평가, 학습/검증 루프
- 데이터 계층(CSV/MAT): 시나리오 결과, trace, 모델 스냅샷, 논문용 요약 산출물

## 패키지 구성

- `sjtu_drone_bringup`: Gazebo, AprilTag, bridge, 런치 인자 관리
- `sjtu_drone_description`: URDF/SDF, world, wind plugin
- `sjtu_drone_control`: 제어 노드
- `sjtu_drone_interfaces`: `SetWind` 서비스
- `matlab/`: `AutoSimMain.m`, `AutoSim.m`, 검증/플롯 스크립트

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

- `install/setup.bash` 기준으로 실행 경로를 통일한다.
- `src/` 하위 launch/xacro/python/cpp를 수정하면 관련 패키지를 재빌드해야 반영된다.

## ROS 실행 예시

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

## 병렬 시뮬레이션(멀티 Gazebo)

학습/데이터 수집 처리량을 높이기 위해 인스턴스별 `ROS_DOMAIN_ID`와 `GAZEBO_MASTER_URI`를 분리해 여러 Gazebo를 동시에 실행할 수 있다.

기대 처리량 근사는 다음으로 볼 수 있다.

$$
T_{parallel} \approx N \cdot T_{single} \cdot \eta
$$

- $N$: 병렬 인스턴스 수
- $\eta$: 병렬 효율(리소스 경합 포함, 보통 $0<\eta\le1$)

### 1) 실행 전 백업

```bash
cd /home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2
chmod +x scripts/backup_before_parallel.sh
./scripts/backup_before_parallel.sh /home/j/INCSL/IICC26_ws
```

### 2) 병렬 실행(스크립트 방식)

```bash
cd /home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2
chmod +x scripts/run_parallel_gazebo.sh scripts/stop_parallel_gazebo.sh

# 예: 4개 인스턴스, domain 40부터, Gazebo 포트 12045부터
./scripts/run_parallel_gazebo.sh 4
```

주요 환경 변수:

- `DOMAIN_BASE` (기본 `40`)
- `GAZEBO_PORT_BASE` (기본 `12045`)
- `USE_RVIZ` (기본 `false`)
- `USE_APRILTAG` (기본 `true`)
- `USE_GUI_FIRST_ONLY` (기본 `false`)
- `STARTUP_STAGGER_SEC` (기본 `2`)

예시:

```bash
DOMAIN_BASE=60 GAZEBO_PORT_BASE=12145 USE_GUI_FIRST_ONLY=true ./scripts/run_parallel_gazebo.sh 3
```

중지:

```bash
./scripts/stop_parallel_gazebo.sh
```

### 3) 병렬 실행(ROS2 launch 방식)

```bash
source /opt/ros/humble/setup.bash
source /home/j/INCSL/IICC26_ws/install/setup.bash

ros2 launch sjtu_drone_bringup sjtu_drone_parallel.launch.py \
  instance_count:=4 \
  domain_id_base:=40 \
  gazebo_port_base:=12045 \
  startup_stagger_sec:=2.0 \
  use_gui_first_only:=false \
  use_rviz:=false \
  use_teleop:=false \
  use_apriltag:=true
```

주의:

- AutoSim 멀티 워커는 워커별 `ROS_DOMAIN_ID`, `GAZEBO_MASTER_URI`, 출력 디렉터리, lock 파일을 분리하도록 업데이트되었다.
- 병렬 워커에서는 `cleanup_scope=instance`를 사용해 워커가 띄운 프로세스 트리만 정리하고, 전체 `pkill` 동작은 피한다.

### 4) AutoSim 멀티 워커 실행 (도메인/출력 분리 + 자동 튜닝)

```bash
cd /home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2
chmod +x matlab/scripts/run_autosim_parallel.sh matlab/scripts/stop_autosim_parallel.sh

# 워커 수 자동 계산(CPU/메모리 기반)
matlab/scripts/run_autosim_parallel.sh auto
```

수동 워커 수 지정:

```bash
DOMAIN_BASE=70 GAZEBO_PORT_BASE=14045 SCENARIO_COUNT=300 matlab/scripts/run_autosim_parallel.sh 4
```

자동 튜닝 규칙:

$$
N_{cpu} = \max(1, C_{total} - C_{reserve})
$$

$$
N_{mem} = \max\left(1, \left\lfloor \frac{M_{available} - M_{reserve}}{M_{per\_worker}} \right\rfloor \right)
$$

$$
N_{auto} = \min(N_{cpu}, N_{mem})
$$

- `C_total`: 전체 CPU 코어 수
- `M_available`: 가용 메모리(GB)
- 기본값: `CPU_RESERVE=2`, `MEM_RESERVE_GB=4`, `MEM_PER_WORKER_GB=3`

중지:

```bash
matlab/scripts/stop_autosim_parallel.sh
```

### 5) 병렬 결과 병합 후처리

```bash
# session_root 예: matlab/parallel_runs/20260320_123000
python3 matlab/scripts/merge_autosim_results.py matlab/parallel_runs/<session_root>
```

산출물:

- `merged/autosim_dataset_merged.csv`
- `merged/autosim_trace_merged.csv`
- `merged/autosim_learning_merged.csv`

각 행에는 `worker_id`, `run_id`, `source_file`가 추가되어 출처를 추적할 수 있다.

### 6) 실시간 모니터링(필수 그래프만)

복잡한 온톨로지 라이브 뷰는 기본 비활성화(`AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ=false`)되며, 병렬 실행 모니터링은 경량 그래프로 분리했다.

```matlab
monitor_autosim_parallel('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/parallel_runs/<session_root>', 2.0)
```

- 상단: 워커별 진행 시나리오 수 + unsafe rate
- 하단: 전체 워커 aggregate unsafe landing rate 추이

## 핵심 인터페이스

- `/wind_command`: 풍속/풍향 명령 입력
- `/wind_condition`: 풍속/풍향 상태 출력
- `/set_wind`: 풍속 설정 서비스
- `/landing_tag_state`: MATLAB 호환 AprilTag bridge
- `/drone/gt_pose`, `/drone/gt_vel`, `/drone/state`: AutoSim 주요 입력

## MATLAB 실행

```matlab
AutoSimMain
```

또는

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')
```

AutoSim 주요 기능:

- 시나리오 생성/실행/정리 자동화
- 온톨로지+AI 융합 판단
- 누적 데이터셋 기반 모델 학습/검증
- 결과 CSV, 추적 로그, 논문용 figure/table 생성

상세 문서:

- [matlab/README.md](matlab/README.md)
- [matlab/ROS2_Gazebo_MATLAB_Validation_Guideline.md](matlab/ROS2_Gazebo_MATLAB_Validation_Guideline.md)

## 산출물 취급 원칙

- `build/`, `install/`, `log/`는 빌드/실행 산출물
- `matlab/data`, `matlab/logs`, `matlab/models`, `matlab/plots`는 실험 산출물
- 버전 관리 대상과 산출물 경로를 분리해 재현성을 유지한다
