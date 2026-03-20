# ROS I/O Modules

ROS2 토픽/서비스 입출력과 메시지 파싱을 담당한다.

## 기능 설명

- ROS context 생성/해제
- subscriber/publisher 구성
- AprilTag, wind, IMU, contact 메시지 파싱
- callback cache 및 receive fallback 관리

## 이론 포인트

- 실험 신뢰성은 입력 지연/누락 처리에 크게 좌우됨
- 파싱 실패를 NaN-safe 흐름으로 흡수해 루프 중단을 방지

입력 신선도는 수신 시간 기반으로 본다.

$$
\Delta t_{rx}=t_{now}-t_{last\_rx}
$$

태그 중심 오차는 정규화 좌표에서 계산한다.

$$
e_{tag}=\sqrt{u^2+v^2}
$$

NaN-safe 처리는 기본값 치환 형태로 적용한다.

$$
x'=
\begin{cases}
x, & \mathrm{isfinite}(x) \\
x_{fallback}, & \text{otherwise}
\end{cases}
$$

## 대표 파일

- `autosimCreateRosContext.m`
- `autosimTryReceive.m`
- `autosimParseTag.m`
- `autosimParseWindConditionMsg.m`
- `autosimParseContactForces.m`

## 확장 가이드

- 새로운 센서 토픽 추가 시 parser + context + trace 반영을 함께 적용한다.
