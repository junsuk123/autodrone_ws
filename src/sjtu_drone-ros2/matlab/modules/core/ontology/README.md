# Ontology Modules

온톨로지 상태 생성과 의미론 기반 위험/안전 추론을 담당한다.

## 기능 설명

- Wind/Drone/Vision 관측을 객체 상태로 구조화
- 관계(정렬, 시각 신뢰, 문맥 안전)를 점수화
- 규칙 기반/경량 AI 결합으로 semantic encoding 산출

## 이론 포인트

- 위험도 결합: 풍속/가속도/변동성/방향 변화량
- 시각 안정도: 태그 검출 연속성 + 중심 오차 + jitter
- 최종 의미 점수는 decision feature의 `*_enc`로 전달

풍속/가속도 기반 위험도 예시는 다음과 같다.

$$
r_w = \min\left(1,\max\left(0,\alpha_v\frac{v}{v_{thr}}+\alpha_a\frac{|a_w|}{a_{thr}}\right)\right)
$$

시각 정렬 신뢰도는 중심 오차 정규화로 계산한다.

$$
c_v = \min\left(1,\max\left(0,1-\frac{e_{tag}}{e_{thr}}\right)\right)
$$

최종 의미 점수는 가중 결합 형태다.

$$
s_{sem}=w_w(1-r_w)+w_v c_v+w_a s_a+w_m m_{ctx}
$$

## 대표 파일

- `autosimBuildOntologyState.m`
- `autosimBuildTemporalSemanticState.m`
- `autosimOntologyReasoning.m`
- `autosimBuildSemanticFeatures.m`

## 확장 가이드

- 규칙 임계치, 개념 정의, relation 라벨 체계 변경은 이 폴더를 기준으로 수행한다.
