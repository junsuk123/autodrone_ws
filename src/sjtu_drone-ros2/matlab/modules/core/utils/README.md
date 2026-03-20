# Utility Modules

다른 도메인 모듈에서 공통으로 사용하는 보조 함수 모음이다.

## 기능 설명

- 수치 처리: clamp, nan-safe 통계, 정규화
- 시스템 처리: process/lock 관리, 문자열/시간 유틸
- 물리/좌표 보조: quaternion 변환, wind vector 변환

## 이론 포인트

- nan-safe 유틸은 센서 결측 상황에서 실험 중단을 방지
- lock/process 유틸은 반복 실험의 안정성 보장

## 대표 파일

- `autosimClamp.m`, `autosimNanMean.m`, `autosimSafeDivide.m`
- `autosimAcquireLock.m`, `autosimCleanupProcesses.m`
- `autosimQuat2Eul.m`, `autosimWindVectorFromSpeedDir.m`

## 확장 가이드

- 특정 도메인에 종속된 로직은 이 폴더에 두지 않고 해당 도메인 폴더로 이동한다.
