# Decision Making Modules

정책 선택, feature 조합, 판단 결과 해석을 담당한다.

## 최근 업데이트 (2026-03-23)

의사결정 입력으로 들어가는 온톨로지 변환에서 바람 벡터 성분 반영을 강화했다.

$$
	ext{onto\_wind\_condition} \propto
w_1\,\text{wind\_risk\_enc} + w_2\,\|\mathbf{v}_w\|_2 + w_3\max(|v_x|,|v_y|)
$$

$$
	ext{onto\_gust} \propto
u_1\,\|\mathbf{a}_w\|_2 + u_2\max(|a_x|,|a_y|) + u_3\,\Delta v_{gust}
$$

여기서 `wind_risk_enc`은 온톨로지에서 항력 하중비를 기반으로 계산된 위험도 인코딩이다.

## 기능 설명

- 시나리오별 정책 모드 선택(exploit/boundary/hard-negative)
- 온라인 관측값을 모델 입력 feature로 변환
- 정책 판단과 GT 관계를 confusion 관점으로 정리

## 이론 포인트

- 이진 의사결정: `AttemptLanding` vs `HoldLanding`
- 안전 중심 평가: FP(unsafe landing)를 강하게 억제
- 운영 관점 균형: FN(과도한 abort) 감소가 후속 최적화 과제

의사결정 함수는 임계값 기반으로 표현할 수 있다.

$$
\hat{y}=
\begin{cases}
\mathrm{AttemptLanding}, & s_{fusion}\ge \tau \\
\mathrm{HoldLanding}, & s_{fusion}<\tau
\end{cases}
$$

안전/기회 균형은 혼동행렬 지표로 본다.

$$
\mathrm{Precision}=\frac{TP}{TP+FP},\quad
\mathrm{Recall}=\frac{TP}{TP+FN},\quad
\mathrm{UnsafeLandingRate}=\frac{FP}{FP+TN}
$$

## 핵심 변수/용어 표

| 항목 | 의미 | 단위/범위 | 비고 |
|---|---|---|---|
| s_fusion | 최종 의사결정 점수 | 0~1 | 모델+의미 점수 결합 |
| tau | 착륙 임계값 | 0~1 | s_fusion >= tau 이면 AttemptLanding |
| TP | 안전 착륙을 착륙으로 판단 | count | 정답 착륙 |
| FP | 위험 착륙을 착륙으로 판단 | count | 가장 위험한 오판 |
| FN | 안전 착륙을 중단으로 판단 | count | 기회 손실 |
| TN | 위험 착륙을 중단으로 판단 | count | 정답 거절 |
| scenario_policy | 시나리오 정책 모드 | exploit/boundary/hard_negative | curriculum/probe와 연동 |

## 대표 파일

- `autosimChooseScenarioPolicy.m`
- `autosimBuildAdaptiveScenarioConfig.m`
- `autosimBuildOnlineFeatureVector.m`
- `autosimBuildDecisionTable.m`
- `autosimEvaluateDecisionMetrics.m`

## 확장 가이드

- 밴드별 threshold, uncertainty handling, probe policy 조정은 이 폴더에서 우선 수정한다.
