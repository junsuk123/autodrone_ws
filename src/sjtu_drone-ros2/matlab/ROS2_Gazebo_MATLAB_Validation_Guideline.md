# ROS 2, Gazebo, MATLAB 연동 알고리즘 검증 가이드

이 문서는 현재 워크스페이스의 AutoSim 기반 구조를 바탕으로, ROS 2와 Gazebo 시뮬레이션 그리고 MATLAB을 연동해 알고리즘을 검증할 때의 공통 설계 원칙과 운영 절차를 정리한 문서다. 특정 드론 과제에 바로 쓰는 것뿐 아니라, 다른 로봇/센서/알고리즘에도 재사용할 수 있도록 범용성을 우선해 정리한다.

## 1. 문서 목적

이 문서의 목표는 다음 네 가지다.

- MATLAB이 알고리즘 검증 오케스트레이터 역할을 맡고, ROS 2와 Gazebo는 실행 환경과 데이터 소스를 제공하는 구조를 표준화한다.
- 실험 재현성, 토픽 계약, 빌드 반영 규칙, 장애 대응 기준을 공유한다.
- 현재 AutoSim 구현에서 이미 검증된 실행 흐름을 문서화한다.
- 향후 다른 모델 기반 제어기, 강화학습, 추정기, 비전 알고리즘으로 바뀌어도 그대로 적용 가능한 최소 공통 구조를 제안한다.

## 2. 현재 기준 아키텍처

현재 기준 실행 경로는 MATLAB의 AutoSim이 실험 전체를 관리하고, ROS 2 launch가 Gazebo 및 드론 관련 노드를 올린 뒤, MATLAB이 ROS 2 토픽을 통해 상태를 읽고 명령을 내리는 구조다.

구성 역할은 아래처럼 나누는 것이 가장 안정적이다.

- Gazebo: 물리 시뮬레이션, 센서/접촉/풍장 환경 생성
- ROS 2 launch: 시뮬레이터, 브리지, 제어 노드, AprilTag 관련 노드 실행
- ROS 2 토픽/서비스: MATLAB과 시뮬레이터 사이의 인터페이스 계층
- MATLAB: 시나리오 관리, 데이터 수집, 상태 해석, 알고리즘 판단, 모델 업데이트, 로그 저장

현재 코드 기준 핵심 진입점은 아래 파일들이다.

- MATLAB 실험 진입점: matlab/AutoSim.m
- ROS 2 bringup launch: sjtu_drone_bringup/launch/sjtu_drone_bringup.launch.py
- 드론 제어 토픽 정의 참고: sjtu_drone_control/sjtu_drone_control/drone_utils/drone_object.py

## 3. 권장 역할 분리 원칙

범용성을 확보하려면 MATLAB, ROS 2, Gazebo의 책임을 섞지 않는 것이 중요하다.

### 3.1 MATLAB이 가져야 할 책임

- 실험 시나리오 생성과 반복 실행
- 알고리즘 추론 또는 검증 로직 실행
- ROS 2 토픽 구독 결과를 시간축 데이터로 정리
- 시나리오 단위 성공/실패 판정
- 데이터셋 저장, 모델 저장, 플롯 저장
- 실험 중단 요청 처리와 자원 정리

### 3.2 ROS 2가 가져야 할 책임

- 노드 실행과 생명주기 관리
- 센서와 액추에이터 인터페이스 노출
- 시뮬레이터와 외부 알고리즘 간 메시지 교환
- 토픽 타입과 네임스페이스 일관성 유지

### 3.3 Gazebo가 가져야 할 책임

- 동역학, 센서, 외란, 충돌, 월드 조건 생성
- 알고리즘 내부 판단에는 직접 관여하지 않음

이 분리가 흐려지면 MATLAB 스크립트가 launch 세부 구현에 과하게 종속되거나, 반대로 ROS 노드가 실험 정책까지 떠안게 되어 유지보수가 어려워진다.

## 4. 현재 워크스페이스의 실제 인터페이스

현재 AutoSim은 아래 토픽들을 핵심 인터페이스로 사용한다.

### 4.1 MATLAB 주요 입력

- /drone/state
- /drone/gt_pose
- /drone/gt_vel
- /landing_tag_state
- /wind_condition
- 선택적 입력: /drone/imu, /drone/bumper_states

### 4.2 MATLAB 주요 출력

- /wind_command
- /drone/takeoff
- /drone/land
- /drone/cmd_vel

### 4.3 현재 공유되는 의미 계약

- /drone/state: 드론 상태 정수값
- /drone/gt_pose: 위치와 자세
- /drone/gt_vel: 선속도 기반 상태
- /landing_tag_state: 착륙 타깃 인식 결과를 Float32MultiArray로 단순화한 브리지 토픽
- /wind_condition: 현재 적용 중인 풍장 상태
- /wind_command: 외란 명령 입력

중요한 점은 MATLAB이 토픽 이름 자체보다 토픽이 제공하는 의미 계약에 의존해야 한다는 점이다. 다른 프로젝트로 옮길 때는 알고리즘 코어를 고치기보다, 의미 계약에 맞는 ROS 어댑터를 만드는 것이 훨씬 안전하다.

## 5. 현재 AutoSim 실행 흐름

현재 AutoSim의 시나리오 단위 흐름은 아래와 같다.

1. 실행 환경과 저장 디렉터리를 초기화한다.
2. 최신 학습 모델이 있으면 로드하고, 없으면 placeholder 모델로 시작한다.
3. MATLAB에서 ROS 2 node, publisher, subscriber를 생성한다.
4. 시나리오 정책에 따라 hover height, wind 조건 등을 설정한다.
5. MATLAB이 ROS 2 launch를 시작한다.
6. warmup 이후 상태 토픽을 수집하면서 hover 안정성, 시각 추적 품질, 풍장 영향 등을 계산한다.
7. 규칙 기반 semantic 판단과 모델 기반 판단을 결합해 land 또는 abort를 결정한다.
8. 결과를 시나리오 단위로 저장하고, 누적 데이터셋으로 모델을 다시 학습한다.
9. plot, trace, checkpoint를 저장하고 다음 시나리오로 넘어간다.
10. 종료 시 launch 프로세스와 ROS 핸들을 정리한다.

이 구조는 실험 자동화에 적합하다. 연구실에서 다른 알고리즘을 넣더라도 위 흐름 자체는 유지하고, 6~8 단계만 바꾸는 것을 권장한다.

## 6. 범용성 있게 설계하는 방법

다른 과제에도 재사용하려면 아래 구조를 권장한다.

### 6.1 어댑터 계층을 먼저 고정한다

알고리즘 코드가 아래 추상 입력만 받게 설계한다.

- pose
- velocity
- mission state
- target observation
- disturbance estimate
- optional safety signal

즉 MATLAB 내부 알고리즘은 드론 전용 토픽 이름을 직접 참조하지 말고, ROS 수신 결과를 정규화한 내부 상태 구조체만 사용해야 한다.

### 6.2 실험 정책과 알고리즘 정책을 분리한다

- 실험 정책: 시나리오 수, 외란 크기, 초기 위치, 종료 조건
- 알고리즘 정책: 착륙/정지/회피/재시도 판단 로직

이 둘을 분리하면 동일 알고리즘을 여러 시나리오에 반복 적용하기 쉽다.

### 6.3 토픽보다 메시지 의미를 우선한다

예를 들어 현재는 /landing_tag_state를 사용하지만, 다른 비전 시스템에서는 bounding box나 pose estimate를 직접 줄 수 있다. 이 경우에도 MATLAB 내부에서는 다음 같은 공통 필드로 변환해 쓰는 것이 좋다.

- target_detected
- target_error_x
- target_error_y
- target_quality
- target_id

### 6.4 판단기와 학습기를 느슨하게 결합한다

현재 AutoSim은 semantic reasoning과 모델 기반 판단을 함께 사용한다. 다른 연구 주제로 확장할 때도 아래처럼 느슨하게 묶는 것이 좋다.

- safety filter
- rule-based evaluator
- learned policy or classifier
- final decision gate

이렇게 하면 모델이 바뀌어도 전체 실험 프레임은 유지된다.

## 7. 표준 실행 절차

### 7.1 워크스페이스 준비

기본적으로 아래 순서를 지킨다.

1. ROS 2 환경을 로드한다.
2. colcon build로 패키지를 빌드한다.
3. install/setup.bash를 다시 source 한다.
4. MATLAB을 연다.
5. AutoSim 또는 후속 실험 스크립트를 실행한다.

현재 워크스페이스는 install space 기준으로 동작하므로, src 아래 코드를 수정했으면 재빌드 없이 반영된다고 가정하면 안 된다.

### 7.2 launch와 MATLAB의 시작 순서

현재 구조에서는 MATLAB이 launch까지 직접 시작한다. 이 방식은 반복 실험 자동화에는 유리하지만, 디버깅 시에는 두 모드로 나눠 운용하는 것이 좋다.

- 운영 모드: MATLAB이 launch를 시작하고 종료까지 관리
- 디버그 모드: 터미널에서 launch를 먼저 올리고, MATLAB은 토픽 입출력만 검증

연구실 공유 기준 문서는 두 모드를 모두 허용하되, 자동 실험 재현은 운영 모드를 기본으로 두는 것이 좋다.

### 7.3 실험 산출물 관리

실험 산출물은 반드시 실행 ID 단위로 저장한다.

- raw trace
- scenario summary
- 모델 스냅샷
- plot
- launch log
- checkpoint

현재 AutoSim도 data, logs, plots, models 디렉터리를 분리해 이 원칙을 따른다.

## 8. 연구실 공용 검증 체크리스트

실험을 돌리기 전에는 아래를 확인한다.

### 8.1 빌드/환경 체크

- ROS 2 Humble 환경이 잡혀 있는가
- install/setup.bash를 다시 source 했는가
- Gazebo plugin 경로가 유효한가
- MATLAB ROS 2 Toolbox가 현재 ROS 배포판과 호환되는가

### 8.2 인터페이스 체크

- 제어 입력 토픽과 상태 토픽 이름이 일치하는가
- 메시지 타입이 MATLAB에서 기대하는 타입과 일치하는가
- AprilTag bridge나 센서 브리지의 출력 형식이 바뀌지 않았는가

### 8.3 실험 정책 체크

- 시나리오 수와 외란 범위가 연구 목적과 맞는가
- 종료 조건이 무한 대기 상태를 만들지 않는가
- 실패 시 cleanup 로직이 다음 시나리오에 영향을 주지 않는가

### 8.4 데이터 체크

- 저장 디렉터리가 생성되는가
- trace와 summary가 시나리오별로 구분되는가
- 모델 feature schema가 이전 스냅샷과 호환되는가

## 9. 자주 발생하는 문제와 대응

### 9.1 src를 수정했는데 launch 반영이 안 되는 경우

원인:

- install space 기준으로 실행 중인데 재빌드를 하지 않음

대응:

- 관련 패키지를 다시 colcon build
- install/setup.bash 재적용

### 9.2 MATLAB에서 ROS 객체 정리 시 불안정한 경우

MATLAB ROS 2 객체는 종료 순서에 민감할 수 있다. publisher, subscriber, message, node를 암묵적으로 정리하게 두지 말고, 명시적으로 정리 순서를 두는 것이 안전하다. 현재 워크스페이스도 이 방향으로 보완되어 있다.

권장 사항:

- subscriber/publisher/message/node를 명시적으로 해제
- node를 가장 마지막에 정리
- 문제 재현 시 optional subscription을 하나씩 끄고 최소 구성으로 분리 테스트

### 9.3 launch가 중복 실행되는 경우

원인:

- 이전 scenario의 프로세스 정리가 끝나기 전에 새 launch를 시작함

대응:

- launch 시작 전 stale process snapshot 검사
- kill settle 시간을 두고 재시도
- 자동 실험에서는 lock file 또는 PID 파일 사용

### 9.4 비전 토픽이 일시적으로 끊기는 경우

대응:

- target_detected와 최근 수신 시각을 분리 관리
- 제어 루프에서는 마지막 정상 샘플을 짧은 시간만 예측 보간
- 장시간 dropout은 즉시 unsafe 또는 hold 상태로 해석

## 10. 다른 프로젝트로 확장할 때의 최소 변경점

현재 구조를 유지한 채 다른 과제로 옮길 때는 아래만 바꾸는 것을 권장한다.

### 10.1 보통 바꿔야 하는 항목

- launch 파일 이름과 인자
- 토픽 이름과 메시지 타입
- 센서 어댑터
- 알고리즘 코어
- 평가 지표와 라벨링 기준

### 10.2 가능하면 그대로 유지할 항목

- 시나리오 반복 프레임
- 실행 로그와 산출물 저장 구조
- 체크포인트 저장 방식
- cleanup 절차
- 실패 시 중단 및 재시작 규칙

즉 실험 플랫폼은 바꿔도, 실험 운영 프레임은 유지하는 방향이 가장 생산적이다.

## 11. 권장 폴더 구성 예시

범용 프로젝트에서는 아래처럼 두는 것이 관리에 유리하다.

- matlab/
- matlab/adapters/
- matlab/algorithms/
- matlab/evaluators/
- matlab/config/
- matlab/logs/
- matlab/data/
- matlab/models/
- ros2_bringup/
- ros2_interfaces/
- ros2_description/

현재 AutoSim은 단일 스크립트 비중이 크지만, 연구실 공용 자산으로 키울 계획이면 adapter, algorithm, evaluator 분리를 점진적으로 진행하는 것이 좋다.

## 12. 이 워크스페이스 기준 권장 운영 방침

- 실험 자동화의 기준 진입점은 AutoSim으로 유지한다.
- MATLAB은 토픽 계약을 소비하는 상위 오케스트레이터 역할에 집중한다.
- ROS 2 launch, 토픽 브리지, Gazebo 센서 모델은 MATLAB 밖에서 독립적으로 검증 가능해야 한다.
- 새 알고리즘을 넣을 때는 AutoSim 전체를 복제하기보다 판단 모듈과 feature 구성만 교체한다.
- 연구실 공유 문서는 실행 명령보다 인터페이스 계약과 디버깅 기준을 우선해 유지한다.

## 13. 요약

현재 워크스페이스는 MATLAB이 실험을 통합 관리하는 구조로 이미 잘 정리되어 있다. 범용성을 높이려면 토픽 이름에 종속된 스크립트를 늘리는 대신, ROS 2 어댑터 계층과 MATLAB 내부 공통 상태 표현을 중심으로 재구성해야 한다. 연구실 내 공유 기준은 다음 한 줄로 정리할 수 있다.

MATLAB은 실험을 조정하고 판단하며 저장하고, ROS 2는 인터페이스를 제공하며, Gazebo는 물리 환경을 제공한다.