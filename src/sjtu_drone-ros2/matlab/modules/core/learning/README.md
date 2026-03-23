# Learning Modules

모델 생명주기(로드/검증/학습/예측)를 담당한다.

## 최근 업데이트 (2026-03-23)

학습 모듈 인터페이스/스키마는 유지하되, 입력으로 들어오는 온톨로지 파생 특성의 물리 차원 보존이 강화되었다.

특히 바람 관련 의미 인코딩은 벡터 성분 기반 위험도에서 계산되어, 클래스 분리에 필요한 방향성 정보를 더 잘 반영한다.

요약식:

$$
x_{wind\_sem}=f(\|\mathbf{v}_w\|_2,\max(|v_x|,|v_y|),\|\mathbf{a}_w\|_2,\max(|a_x|,|a_y|))
$$

## 기능 설명

- 모델 스키마 호환성 검증
- GaussianNB 학습 및 추론
- 데이터 누적 기반 incremental update

## 이론 포인트

- 현재 기본 모델은 Gaussian Naive Bayes
- feature schema 불일치 시 안전하게 placeholder 모델로 폴백
- class imbalance를 고려한 업데이트 조건 포함

클래스별 조건부 분포는 가우시안으로 둔다.

$$
p(x\mid y=c)=\prod_{j=1}^{d}\mathcal{N}(x_j;\mu_{c,j},\sigma_{c,j}^2)
$$

예측은 사후확률 최대 클래스를 선택한다.

$$
\hat{y}=\arg\max_c\;\log p(y=c)+\sum_{j=1}^{d}\log \mathcal{N}(x_j;\mu_{c,j},\sigma_{c,j}^2)
$$

불균형 완화를 위해 prior를 균등 prior와 혼합한다.

$$
\pi'_c=(1-\lambda)\pi_c+\lambda\frac{1}{K}
$$

## GaussianNB와 Sigmoid의 역할 분리

Learning 모듈은 의사결정 파이프라인 내에서 **특징 인코딩(Sigmoid)과 확률 분류(GaussianNB)** 두 단계를 보완적으로 사용한다.

### 아키텍처 플로우

센서 신호 → Sigmoid 정규화 → 특징 벡터 → GaussianNB → 착륙 판정

### 역할 구분

**1단계: Sigmoid 인코딩**
- 담당 모듈: `autosim_ontology_engine.m`
- 입력: Raw 센서 (풍속, 오차, 자세)
- 출력: [wind_risk_enc, align_enc, visual_enc]
- 목적: 비선형 정규화 + 물리 해석

**2단계: GaussianNB 분류**
- 담당 모듈: `autosimPredictGaussianNB.m`
- 입력: 인코딩된 특징 (13차원)
- 출력: P(AttemptLanding | X)
- 목적: 확률 기반 분류 + 신뢰도

### 왜 두 기법을 함께 사용하나?

**Sigmoid의 역할 (특징 변환):**
- 각 센서 그룹을 독립적으로 [0,1] 범위로 정규화  
- 비선형 활성화로 복잡한 특징 공간 매핑
- **장점:** 경량(O(1)), 물리 의미 명확, 온톨로지 규칙과 자연스러운 통합

**GaussianNB의 역할 (확률 분류):**
- Sigmoid로 변환된 특징 공간에서 **클래스 조건부 분포를 학습**
- 사후확률 계산으로 클래스별 신뢰도 수량화
- **장점:** 해석 가능, 데이터 효율성, 불균형 자동 처리, 실시간 추론

### 선택 근거

| 평가 항목 | Sigmoid 단독 | Sigmoid+GaussianNB |
|-----------|------------|-------------------|
| 특징 정규화 | ✓ | ✓ |
| 클래스 경계 학습 | ✗ | ✓ |
| 신뢰도 수량화 | 고정 화율 | 동적 확률 |
| 데이터 필요량 | 중간 | 적음 |
| 해석성 | 낮음 | 높음 |
| 불균형 처리 | 수동 | 자동 |
| 실시간 성능 | 매우 빠름 | 빠름 |

### 구체적 예시

```matlab
% 1) Sigmoid 인코딩 (ontology engine)
wind_risk_enc = sigmoid(0.5 * wind_speed + (-0.3) * wind_accel + 0.2);  % = 0.62
align_enc = sigmoid(2.0 * (-tag_error) + 0.1);                          % = 0.78
visual_enc = sigmoid(1.5 * attitude_stability + (-0.5));                % = 0.85

% 2) GaussianNB 분류 (learning module)
X = [0.62, 0.78, 0.85, other_features];  % 13차원 입력

% 각 클래스의 log 우도 계산
log_prob_attempt = sum(log_likelihood_attempt);  % mean, variance로 계산
log_prob_hold = sum(log_likelihood_hold);

% 클래스 선택
post_prob_attempt = softmax(log_prob_attempt, log_prob_hold);
% post_prob_attempt ≈ 0.91 → AttemptLanding (신뢰도 91%)
```

## 핵심 변수/용어 표

| 항목 | 의미 | 단위/범위 | 비고 |
|---|---|---|---|
| X | 입력 feature 행렬 | N x d | d=feature 수 |
| y | 클래스 라벨 | AttemptLanding/HoldLanding | 학습 타깃 |
| mu_c,j | 클래스별 평균 | 실수 | GaussianNB 파라미터 |
| sigma2_c,j | 클래스별 분산 | 양수 실수 | 너무 작으면 floor 적용 |
| pi_c | 클래스 prior | 0~1 | 합=1 |
| lambda | prior uniform blend | 0~1 | 불균형 완화 |
| schema_version | feature 스키마 버전 | 문자열 | 불일치 시 폴백 |
| placeholder model | 임시 모델 | struct | cold start/불일치 안전 처리 |

## 대표 파일

- `autosimLoadOrInitModel.m`
- `autosimTrainGaussianNB.m`
- `autosimPredictGaussianNB.m`
- `autosimIncrementalTrainAndSave.m`
- `autosimModelFeatureSchemaMatches.m`

## 확장 가이드

- 다른 분류기 추가 시 `predict/train` 인터페이스를 유지해 교체 가능하게 구현한다.
