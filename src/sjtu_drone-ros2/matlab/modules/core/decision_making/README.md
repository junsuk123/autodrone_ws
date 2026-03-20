# Decision Making Modules

정책 선택, feature 조합, 판단 결과 해석을 담당한다.

## 기능 설명

- 시나리오별 정책 모드 선택(exploit/boundary/hard-negative)
- 온라인 관측값을 모델 입력 feature로 변환
- 정책 판단과 GT 관계를 confusion 관점으로 정리

## 이론 포인트

- 이진 의사결정: `AttemptLanding` vs `HoldLanding`
- 안전 중심 평가: FP(unsafe landing)를 강하게 억제
- 운영 관점 균형: FN(과도한 abort) 감소가 후속 최적화 과제

## 대표 파일

- `autosimChooseScenarioPolicy.m`
- `autosimBuildAdaptiveScenarioConfig.m`
- `autosimBuildOnlineFeatureVector.m`
- `autosimBuildDecisionTable.m`
- `autosimEvaluateDecisionMetrics.m`

## 확장 가이드

- 밴드별 threshold, uncertainty handling, probe policy 조정은 이 폴더에서 우선 수정한다.
