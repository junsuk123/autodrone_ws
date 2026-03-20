# Learning Modules

모델 생명주기(로드/검증/학습/예측)를 담당한다.

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

## 대표 파일

- `autosimLoadOrInitModel.m`
- `autosimTrainGaussianNB.m`
- `autosimPredictGaussianNB.m`
- `autosimIncrementalTrainAndSave.m`
- `autosimModelFeatureSchemaMatches.m`

## 확장 가이드

- 다른 분류기 추가 시 `predict/train` 인터페이스를 유지해 교체 가능하게 구현한다.
