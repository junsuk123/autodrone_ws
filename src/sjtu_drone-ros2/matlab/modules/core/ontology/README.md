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

## 수식 변수 정의 (상세)

아래 변수 정의는 본 문서의 풍하중/자세/문맥 수식 전체에 공통으로 적용한다.

### 1) 바람 벡터와 항력 식 변수

- $\mathbf{v}_w=[v_x,v_y]^\top$: 수평면 바람 속도 벡터 (m/s)
- $v_x, v_y$: 바람 속도의 x/y 성분 (m/s)
- $v=\max(\|\mathbf{v}_w\|_2,\max(|v_x|,|v_y|))$: 위험도 평가에 사용하는 대표 풍속 (m/s)
- $F_d$: 바람으로 인한 등가 항력 (N)
- $\rho$: 공기 밀도 (kg/m^3)
- $C_d$: 항력 계수 (무차원)
- $A$: 기준 정면 면적 (m^2)

### 2) 기울기 보정 추력 여유 변수

- $roll, pitch$: 드론 자세 각도 (rad, 절대값 사용)
- $c_{tilt}=\cos(|roll|)\cos(|pitch|)$: 수직 성분 보정 계수 (무차원)
- $c_{min}$: 극단 자세에서 분모 발산 방지를 위한 최소 코사인 클램프 (무차원)
- $m$: 드론 질량 (kg)
- $g$: 중력가속도 (m/s^2)
- $T_{req}$: 현재 기울기에서 고도 유지를 위해 필요한 총 추력 (N)
- $T_{max}$: 기체 총 최대 추력 (N)
- $F_{min}$: 최소 추력 여유 하한 (N)
- $F_{cap}$: 횡풍 대응에 사용 가능한 유효 추력 여유(항력 허용치) (N)

### 3) 풍위험 인코딩 변수

- $r_d=F_d/F_{cap}$: 항력 하중비 (무차원)
- $r_w$: 풍위험 인코딩 (0~1)
- $\min(1,\cdot)$: 위험도 상한 1로 포화

### 4) 시각/자세/문맥 변수

- $c_v$: 시각 정렬 신뢰도 (0~1)
- $e_{tag}$: 태그 중심 정렬 오차 (정규화 값)
- $e_{thr}$: 허용 태그 오차 임계값
- $s_a$: 자세 안정도 인코딩 (0~1)
- $\beta_r, \beta_p$: roll/pitch 감쇠 계수 (무차원)
- $roll_{thr}, pitch_{thr}$: 자세 허용 임계각 (rad)
- $m_{ctx}$: 문맥 안전 점수 (0~1)
- $\mathrm{control\_load}$: 제어 부담도 인코딩 (0~1)
- $\mathrm{visual\_dropout}$: 시각 신호 손실률 인코딩 (0~1)
- $\mathrm{relation\_conflict}$: 관계 충돌 정도 (supportive=0, conditional=0.5, conflicting=1)
- $\lambda_1\sim\lambda_5$: 문맥 위험 항 가중치 (무차원)
- $\mathrm{clamp}(x,0,1)$: 값을 0~1 범위로 제한하는 연산

### 5) 최종 의미 점수 결합 변수

- $s_{sem}$: 온톨로지 기반 최종 의미 안전 점수 (0~1)
- $w_w, w_v, w_a, w_m$: 풍위험/시각정렬/자세안정/문맥안전 가중치
- 가중치 합 제약: $w_w+w_v+w_a+w_m=1$

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
