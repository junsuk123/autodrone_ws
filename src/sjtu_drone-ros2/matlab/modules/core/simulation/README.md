# Simulation Modules

Gazebo/ROS 상에서 시나리오를 실제 실행하고 리셋/착륙 흐름을 제어한다.

## 기능 설명

- launch, reset, takeoff, control-loop 실행
- soft reset(`/reset_world` -> `/reset_simulation`) 우선 시도
- 착륙 후 안정화 구간에서 최종 라벨 계산용 통계 추출

## 이론 포인트

- 제어 관점: 착륙 직전 추종 안정성과 touchdown 동역학 분리 평가
- 환경 관점: 풍속/풍향 시계열 및 가속도 기반 난류 성분 반영
- 안전 판정: 상태/자세/속도/접촉 지표를 조합해 stable/unstable 라벨링

## 대표 파일

- `autosimRunScenario.m`
- `autosimResetSimulationForScenario.m`
- `autosimSoftReset.m`
- `autosimComputeTagTrackingCommand.m`
- `autosimSummarizeAndLabel.m`

## 확장 가이드

- reset/takeoff 안정성, hover/landing phase 타이밍, 바람 모델 변경은 이 폴더에서 관리한다.
