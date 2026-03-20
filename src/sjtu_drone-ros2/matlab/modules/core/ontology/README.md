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

## 대표 파일

- `autosimBuildOntologyState.m`
- `autosimBuildTemporalSemanticState.m`
- `autosimOntologyReasoning.m`
- `autosimBuildSemanticFeatures.m`

## 확장 가이드

- 규칙 임계치, 개념 정의, relation 라벨 체계 변경은 이 폴더를 기준으로 수행한다.
