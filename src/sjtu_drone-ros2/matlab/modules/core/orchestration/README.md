# Orchestration Modules

실행 진입, 설정, 상태 전환, 저장/종료 절차를 담당한다.

## 기능 설명

- 기본 설정/외부 오버라이드 적용
- 실행 중 stop request/interrupt 처리
- 결과 요약, 체크포인트, 최종 산출물 정리

## 이론 포인트

- 재현성은 설정 고정과 저장 일관성에 의존
- 장시간 실험에서 안전 종료/복구 가능성이 중요

실험 단위 재현성은 설정 함수 고정으로 표현할 수 있다.

$$
\mathcal{D}_{run}=F(\theta, s)
$$

- $\theta$: 고정된 설정 집합
- $s$: 시나리오/샘플링 정책

체크포인트 기반 복구는 단계별 상태 저장으로 모델링된다.

$$
S_k = \{R_k, T_k, M_k, H_k\}
$$

- $R_k$: results
- $T_k$: trace
- $M_k$: model
- $H_k$: learning history

## 대표 파일

- `autosimDefaultConfig.m`
- `autosimApplyExternalOverride.m`
- `autosimFinalize.m`
- `autosimSaveCheckpoint.m`
- `autosimSummaryTable.m`

## 확장 가이드

- 실험 정책의 전역 스위치/기본값은 이 폴더에서 관리한다.
