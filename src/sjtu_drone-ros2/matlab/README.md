# MATLAB AutoSim Guide

현재 이 폴더의 실사용 MATLAB 진입점은 AutoSim.m 하나다. 예전 수동 노드, 수동 wind publisher, 별도 학습 파이프라인 스크립트는 제거했다.

공유용 연동 설계/운영 가이드는 `ROS2_Gazebo_MATLAB_Validation_Guideline.md`를 본다.

## 실행 목표

AutoSim은 다음을 한 번에 수행한다.

- ROS2/Gazebo launch 시작 및 정리
- 시나리오별 풍속/풍향 외란 주입
- 센서 수집과 AprilTag 기반 호버 구간 분석
- 온톨로지 추론과 모델 기반 착륙 판단
- 결과 라벨링, 데이터셋 누적, 모델 재학습
- plot, trace, checkpoint 저장

## 실행 방법

터미널에서 ROS 환경을 먼저 불러온 뒤 MATLAB에서 아래만 실행한다.

```bash
cd /home/j/INCSL/IICC26_ws
source /opt/ros/humble/setup.bash
source /home/j/INCSL/IICC26_ws/install/setup.bash
```

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')
```

## 현재 AutoSim 동작 범위

유효 분석 구간은 다음으로 제한된다.

- 시작: 목표 지점 도달 후 xy_hold 진입 시점
- 종료: 착륙 명령이 발행되는 시점

다음 구간은 분석, 저장, 모델 입력, plot에서 제외된다.

- 이륙 전 준비 구간
- 목표점 도달 전 이동과 상승 구간
- 착륙 이후 관측 구간

## 주요 입력 토픽

- /drone/gt_pose
- /drone/gt_vel
- /drone/state
- /wind_condition
- /landing_tag_state
- 선택적 IMU 및 contact 토픽

## 주요 출력 토픽

- /wind_command
- /drone/takeoff
- /drone/cmd_vel

## launch 정책

AutoSim은 내부에서 아래 방향으로 launch를 단순화해서 사용한다.

- use_gui:=false
- use_rviz:=false
- use_teleop:=false
- use_apriltag:=true

즉 현재 AutoSim 경로에서는 RViz와 teleop가 기본 실행 경로가 아니다.

## 판단 파이프라인

1. ROS 센서 수집
2. AprilTag 시계열 안정성 계산
3. 풍속, 자세, 속도 기반 온톨로지 상태 생성
4. 온톨로지 추론으로 semantic state 계산
5. 모델 입력 feature 생성
6. semantic과 model 융합으로 land 가능성 계산
7. hover 구간 품질 기준으로 최종 라벨링

## 산출물

AutoSim은 아래 위치에 실행 산출물을 남긴다.

- matlab/data/<run_id>/
- matlab/logs/<run_id>/
- matlab/models/autosim_model_*.mat
- matlab/plots/...

이 중 데이터, 로그, 모델 스냅샷, 결과 plot은 생성 산출물이다.

## 유지보수 메모

- launch는 install/setup.bash 이후 install-space 기준으로 동작한다.
- src 아래 launch, xacro, Python 패키지 코드를 수정했으면 관련 패키지를 다시 빌드해야 한다.
- AutoSim은 별도 PYTHONPATH 우회 없이 워크스페이스 환경만 사용한다.
- AprilTag bridge 형식은 [detected, tag_id, center_x_px, center_y_px, area_px2, margin, num_tags] 이다.
