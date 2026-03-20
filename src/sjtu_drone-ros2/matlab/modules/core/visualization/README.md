# Visualization Modules

실험 진행과 결과를 해석 가능한 그래프로 표현한다.

## 기능 설명

- 진행 중 decision/performance 추세 그래프 업데이트
- 시나리오 실시간 온톨로지 플로우 시각화
- 실험 종료 후 GT vs Prediction, 성능 요약 그래프 생성

## 이론 포인트

- 단일 정확도보다 FP/FN 분해가 안전 연구에서 핵심
- 풍속 밴드/문맥 상태별 시각화가 정책 편향 탐지에 유효

## 대표 파일

- `autosimInitPlots.m`
- `autosimUpdatePlots.m`
- `autosimInitScenarioRealtimePlot.m`
- `autosimUpdateScenarioRealtimePlot.m`
- `autosimPlotGtVsPrediction.m`

## 확장 가이드

- 논문 figure 변경은 이 폴더에서 데이터 집계 방식과 함께 수정한다.
