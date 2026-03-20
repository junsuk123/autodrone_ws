# IICC26 Workspace

IICC26 워크스페이스는 ROS2+Gazebo 시뮬레이션과 MATLAB `AutoSim`을 결합해, 바람 외란 환경에서 드론 착륙 의사결정(AttemptLanding vs HoldLanding)을 연구하는 통합 실험 환경이다.

## 연구 목표

- 바람/시각 정렬 오차/자세 안정성의 결합 위험도를 정량화한다.
- 온톨로지 규칙 기반 의미론 판단과 데이터 기반 모델 추론을 결합한다.
- 위험 착륙(unsafe landing)을 낮추면서도 안전 착륙 기회를 과도하게 버리지 않는 정책을 설계한다.

## 이론 배경

### 1) 바람 하중 기반 물리 한계

드론이 횡풍을 버틸 수 있는 최대 조건을, 추력 여유와 항력 평형으로 근사한다.

$$
F_d = \frac{1}{2}\rho C_d A v^2
$$

$$
F_{margin} = T_{max} - mg
$$

$$
v_{hover\_{limit}} = \sqrt{\frac{2(T_{max}-mg)}{\rho C_d A}}
$$

착륙 구간은 호버보다 보수적으로 제한한다.

$$
v_{landing\_limit} = \alpha \cdot v_{hover\_limit}, \quad \alpha \approx 0.5
$$

### 2) 온톨로지+AI 결합 판단

최종 착륙 점수는 모델 확률과 의미론적 안전 점수의 가중 결합으로 구성한다.

$$
s_{fusion} = w_m \cdot p_{model}(safe) + (1-w_m) \cdot s_{semantic}
$$

- `s_semantic`: 온톨로지 규칙(풍속/가속도 위험, 시각 신뢰도, 관계 일관성) 기반 안전도
- `p_model(safe)`: 학습 모델의 안전 착륙 확률
- `w_m`: 의미론 충돌/주의 상태에서 자동 축소되는 적응 가중치

최종 정책은 임계값 비교로 이진화한다.

$$
\hat{y} =
\begin{cases}
	ext{AttemptLanding}, & s_{fusion} \ge \tau \\
	ext{HoldLanding}, & s_{fusion} < \tau
\end{cases}
$$

### 3) 평가 지표

혼동행렬 원소를 $TP, FP, FN, TN$으로 둘 때,

$$
	ext{Accuracy} = \frac{TP+TN}{TP+FP+FN+TN}
$$

$$
	ext{Precision} = \frac{TP}{TP+FP}, \quad
	ext{Recall} = \frac{TP}{TP+FN}
$$

$$
	ext{Specificity} = \frac{TN}{TN+FP}
$$

$$
	ext{Balanced Accuracy} = \frac{\text{Recall}+\text{Specificity}}{2}
$$

$$
	ext{Unsafe Landing Rate} = \frac{FP}{FP+TN}
$$

본 연구에서는 단순 정확도보다 `Unsafe Landing Rate`, `Specificity`, `Balanced Accuracy`를 핵심 안전 지표로 본다.

## 워크스페이스 구조

- `src/sjtu_drone-ros2/`: 실제 연구 코드 저장소
- `build/`, `install/`, `log/`: `colcon` 산출물
- `src/Diagram/`: 논문/도식/정리 문서
- `matlab_msg_ws/`: MATLAB 메시지 연동 산출물

핵심 실행 단위:

- `sjtu_drone_bringup`: 시뮬레이터/브리지/런치
- `sjtu_drone_description`: 모델/월드/풍속 플러그인
- `sjtu_drone_control`: 기본 제어 노드
- `sjtu_drone_interfaces`: `SetWind` 인터페이스
- `matlab/AutoSimMain.m`, `matlab/AutoSim.m`: 자동 실험 파이프라인

## 빠른 시작

```bash
cd /home/j/INCSL/IICC26_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/j/INCSL/IICC26_ws/install/setup.bash
```

```matlab
AutoSimMain
```

또는

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')
```

## 운영 원칙

- 문서는 AutoSim 기반 현재 파이프라인 기준으로 유지한다.
- 실행은 `install/setup.bash` 환경 기준으로 통일한다.
- 데이터/로그/모델 스냅샷/플롯은 생성 산출물로 간주한다.

## 문서 맵

- [src/sjtu_drone-ros2/README.md](src/sjtu_drone-ros2/README.md)
- [src/sjtu_drone-ros2/matlab/README.md](src/sjtu_drone-ros2/matlab/README.md)
- [src/sjtu_drone-ros2/sjtu_drone_description/README.md](src/sjtu_drone-ros2/sjtu_drone_description/README.md)