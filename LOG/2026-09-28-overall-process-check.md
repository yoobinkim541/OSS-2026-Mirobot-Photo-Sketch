# 2026-09-28 — 전체 과정 확인 및 실물 연결 점검

## 확인한 것

- 설치된 Python 3.13에서 현재 저장소 테스트 전체 실행: **276개 통과, 1개 건너뜀**. 건너뛴 테스트는 Windows에서 bash가 필요한 스크립트 문법 검사다.
- 실제 사진 `input/photo1_mic.webp`를 현재 CV 소스로 처리했다. 70 × 41.559 mm, 75획, 438점, 펜다운 경로 412.46 mm, 펜업 이동 261.39 mm로 변환됐고 예상 시간은 약 4.7분, G-code는 590줄이다.
- 생성 경로의 로봇 좌표 사전검사와 관절 시뮬레이션이 통과했다. 1 mm 간격 표본 1,357개, 최소 관절 여유 22.79°였다.
- USB 포트 목록 조회에서는 COM9가 보였지만, 실제 시리얼 열기 시 `FileNotFoundError`가 발생했다. 연결/호밍/그리기는 시작되지 않았고 로봇에 명령을 보내지 않았다.
- WSL 배포판 목록은 세션에서 WSL 서비스 `E_ACCESSDENIED`로 확인하지 못했다.

## 생성 산출물

- `verification-2026-09-28/photo1_mic_70_low.json`: 사진에서 변환한 종이 mm 획 데이터
- `verification-2026-09-28/photo1_mic_70_low_preview.png`: CV 단계와 그리기 순서 미리보기
- `verification-2026-09-28/photo1_mic_70_low.gcode`: 로봇을 움직이지 않는 dry-run G-code
- `verification-2026-09-28/photo1_mic_70_low_joint_plot.png`: 관절 궤적 그래프
- `verification-2026-09-28/current_illust1_low.*`: 일러스트 샘플의 현재 소스 파이프라인 결과

GUI 스모크 테스트는 통과했으나 창을 닫은 뒤 Tk `after` 예약 콜백의 `invalid command` 진단이 출력됐다. 닫을 때 반복 USB 폴링 타이머를 정리하는 점검이 남아 있다.

## 캘리브레이션 미해결 항목

`robot/drawing_config.json`에서 `paper_x_to_robot_y_sign`은 `unverified`, `plane_compensation.status`는 `pending_measurement`다. `pen_tip_offset_mm.x`는 약 100 mm로 추정한 값이다. 연결 후 TCP가 설정된 종이 중심에서 5 mm 넘게 벗어나면 실행기가 중단하지만, 현재 코드는 종이 평면 기울기나 펜 접촉을 자동 측정하지 않는다. 컨트롤러 상태와 TCP만으로 물리적 종이 면/접촉을 추론할 수 없으며, 해당 측정에는 사용자 접촉 확인이나 별도 센서가 필요하다.

실물 드로잉은 COM9가 다시 열리고 설정의 방향/접촉 보정을 확인한 뒤 진행한다.
