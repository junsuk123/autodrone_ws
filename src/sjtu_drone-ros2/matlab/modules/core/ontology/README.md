# Ontology Modules

온톨로지 상태 생성과 의미론 기반 위험/안전 추론을 담당한다.

## 최근 업데이트 (2026-03-23)

바람 위험도 계산이 항력 하중 기반으로 확장되었다.

$$
\mathbf{v}_w=[v_x,v_y]^\top,\ \mathbf{a}_w=[a_x,a_y]^\top,
v=\max\left(\|\mathbf{v}_w\|_2,\max(|v_x|,|v_y|)\right)
$$

$$
F_d=\frac{1}{2}\rho C_d A v^2
$$

$$
r_d=\frac{F_d}{F_{cap}},\quad
r_{wind}=\min\left(1,\sqrt{r_d}\right)
$$

$$
c_{tilt}=\cos(|roll|)\cos(|pitch|),\quad
T_{req}=\frac{mg}{\max(c_{tilt},c_{min})},\quad
F_{cap}=\max(T_{max}-T_{req},F_{min})
$$

semantic 출력에도 `wind_velocity_x/y`, `wind_acceleration_x/y`를 유지해 차원 누락을 방지한다.

## 기능 설명

- Wind/Drone/Vision 관측을 객체 상태로 구조화
- 관계(정렬, 시각 신뢰, 문맥 안전)를 점수화
- 규칙 기반/경량 AI 결합으로 semantic encoding 산출

## 이론 포인트

- 위험도 결합: 항력 하중비 + 변동성/방향 변화량
- 시각 안정도: 태그 검출 연속성 + 중심 오차 + jitter
- 최종 의미 점수는 decision feature의 `*_enc`로 전달

항력 기반 위험도 예시는 다음과 같다.

$$
F_d = \frac{1}{2}\rho C_d A v^2
$$

$$
r_w = \min\left(1,\sqrt{\frac{F_d}{F_{cap}}}\right)
$$

여기서 $F_{cap}$은 현재 자세 기울기(roll/pitch)에 따라 감소하는 유효 추력 여유를 사용한다.

시각 정렬 신뢰도는 중심 오차 정규화로 계산한다.

$$
c_v = \min\left(1,\max\left(0,1-\frac{e_{tag}}{e_{thr}}\right)\right)
$$

자세 안정도는 roll/pitch 크기에 대한 지수 감쇠로 정의한다.

$$
s_a = \exp\left(-\beta_r\frac{|roll|}{roll_{thr}}-\beta_p\frac{|pitch|}{pitch_{thr}}\right)
$$

문맥 안전 점수는 임무 단계 일치도와 관계 일관성을 포함한 위험 항의 보완값으로 둔다.

$$
m_{ctx}=\mathrm{clamp}\left(1-\left(\lambda_1 r_w+\lambda_2(1-c_v)+\lambda_3\,\mathrm{control\_load}+\lambda_4\,\mathrm{visual\_dropout}+\lambda_5\,\mathrm{relation\_conflict}\right),0,1\right)
$$

여기서 `relation_conflict`는 의미 관계 충돌 정도(예: supportive=0, conditional=0.5, conflicting=1)이며, $\lambda_i$는 가중치다.

최종 의미 점수는 가중 결합 형태다.

$$
s_{sem}=w_w(1-r_w)+w_v c_v+w_a s_a+w_m m_{ctx}
$$

## 핵심 변수/용어 표

| 항목 | 의미 | 단위/범위 | 비고 |
|---|---|---|---|
| r_w | 풍 위험도 인코딩 | 0~1 | 높을수록 위험 |
| c_v | 시각 정렬 신뢰도 | 0~1 | 높을수록 안정 |
| s_a | 자세 안정도 | 0~1 | roll/pitch 기반 |
| m_ctx | 문맥 안전 점수 | 0~1 | 임무 단계/관계 일관성 |
| wind_risk_enc | 풍 위험 feature | 0~1 | decision 입력 |
| alignment_enc | 정렬 feature | 0~1 | decision 입력 |
| visual_enc | 시각 안정 feature | 0~1 | decision 입력 |
| context_enc | 문맥 안전 feature | 0~1 | decision 입력 |

## 대표 파일

- `autosimBuildOntologyState.m`
- `autosimBuildTemporalSemanticState.m`
- `autosimOntologyReasoning.m`
- `autosimBuildSemanticFeatures.m`

## 확장 가이드

- 규칙 임계치, 개념 정의, relation 라벨 체계 변경은 이 폴더를 기준으로 수행한다.
