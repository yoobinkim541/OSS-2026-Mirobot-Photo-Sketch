# 작업 로그

이 폴더는 Mirobot 사진-스케치 프로젝트의 진행, 실험 조건, 관찰 결과, 미해결 항목을 날짜별로 기록합니다.

## 기록 원칙

- 실제 실행 결과, 사용자가 관찰한 결과, 원인에 대한 추정을 구분합니다.
- 측정하지 않은 좌표·압력·보정값은 확정값처럼 기록하지 않습니다.
- 로봇 동작을 바꾼 뒤에는 조건과 결과를 함께 남기고, 재사용 도형은 trajectories/shape-library.json에도 반영합니다.
- JSON 변경 전 스냅샷은 snapshots/에 보관합니다.
- 근거 이미지는 assets/<날짜-주제>/에 보관합니다.

## 세션 기록

- [2026-09-24 — 하트 그리기 결과와 종이 면/X축 접촉 보정 이슈](./2026-09-24-session.md)
- [2026-09-24 — CV 파이프라인, 모델 및 오픈소스 조사](./2026-09-24-cv-model-research.md)
- [2026-09-24 — OpenCV 학습 방향 결정 및 1강 실습 시작](./2026-09-24-opencv-study-kickoff.md)
- [2026-09-24 — 프로젝트 정리, 문제 정의와 해결 (획 추적·종이 변환·실행기)](./2026-09-24-cleanup-and-fixes.md)
- [2026-09-24 — 3D 시뮬레이션으로 드로잉 경로 검증 (IK·관절 한계·도달 지도·RViz)](./2026-09-24-simulation.md)
- [2026-09-24 — 입력 이미지 6장: 획 추출 비교와 3D 시뮬레이션 (스크린톤·순서 속도 개선)](./2026-09-24-image-set-simulation.md)
- [2026-09-25 — CV 레이어·GUI 보강 (획 이어붙이기·이중선 제거·시간 추정·GUI)](./2026-09-25-cv-gui.md)
- [2026-09-25 — 패키지화, Windows .exe, GitHub Actions](./2026-09-25-packaging.md)
- [2026-09-25 — LLM 에이전트 사이드 패널 (OpenRouter / Claude Code / Codex)](./2026-09-25-agent-panel.md)
- [2026-09-25 — GUI에서 RViz 3D 재생 바로 열기](./2026-09-25-gui-rviz.md)
- [2026-09-26 — 입력 이미지를 컬러 원본으로 표시](./2026-09-26-color-original.md)
- [2026-09-26 — 앱 아이콘 교체](./2026-09-26-app-icon.md)
- [2026-09-26 — 단계별 선 추출 파이프라인과 편집 제안](./2026-09-26-stage-pipeline.md)
- [2026-09-26 — GUI에서 로봇으로 그리기 + RViz 실시간 따라가기](./2026-09-26-live-draw.md)
- [2026-09-26 — v0.2.0 릴리스](./2026-09-26-release-v0.2.0.md)
- [2026-09-26 — 선 스무딩 (계단·톱니 제거)](./2026-09-26-smoothing.md)
- [2026-09-26 — 얼굴 섬세하게 그리기 (자동 구도 + 얼굴 세밀 처리)](./2026-09-26-face-detail.md)
- [2026-09-26 — RViz 3D 환경 설치 도우미](./2026-09-26-rviz-setup.md)
- [2026-09-26 — 더 크게 그리기 (도달 범위에 맞춘 넓은 범위)](./2026-09-26-bigger-drawing.md)
- [2026-09-26 — v0.3.0 릴리스](./2026-09-26-release-v0.3.0.md)
- [2026-09-28 — GUI 로봇 USB·컨트롤러 상태 표시](./2026-09-28-robot-status-gui.md)
- [2026-09-28 — 전체 과정 확인 및 실물 연결 점검](./2026-09-28-overall-process-check.md)
- [2026-09-28 — 5점 접촉 캘리브레이션과 사각형 범위 탐색](./2026-09-28-calibration-implementation.md)
- [2026-09-29 — 사진 → 앱 → 로봇 좌표 전송 경로 확인](./2026-09-29-app-drawing-pipeline.md)
- [2026-09-29 — 파이프라인 검증 문제 수정, 캘리브레이션 선택 사항으로 변경](./2026-09-29-pipeline-verification-fixes.md)
- [2026-09-29 — 앱 파이프라인으로 첫 실물 그리기 (보정 없이)](./2026-09-29-first-real-run.md)
- [2026-09-29 — ③ 시작 위치 확인에서 막히던 문제 두 가지 수정](./2026-09-29-start-position-fixes.md)
- [2026-09-29 — 펜으로 그릴 수 있는 최대 정사각형을 120 mm 이상으로 (보정 영역 확장)](./2026-09-29-120mm-calibration.md)
- [2026-09-29 — 펜 올림 높이 10 mm, 접촉 직전만 느리게](./2026-09-29-pen-up-10mm.md)

## 주요 프로젝트 파일

- [OpenCV 학습 계획과 첫 실습](../docs/opencv-study/lesson-01-image-basics.md)
- [도형 및 실행 이력](../trajectories/shape-library.json)
- [프로젝트 설계 문서](../docs/superpowers/specs/2026-09-23-mirobot-photo-sketch-design.md)
- [드로잉 실행기 설정](../robot/drawing_config.json)
- 실물 실행 기록: runs/ (robot/draw_executor.py --execute 가 자동 생성)
