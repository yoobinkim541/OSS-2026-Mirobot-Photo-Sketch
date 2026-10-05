# 2026-10-05 — 시작 자세 T4: 종이 가운데에 맞춘 자세 검사·저장·불러오기

## 만든 것

- `start_pose.check_ready(cfg, status, reference)`: 펜을 올린 직후 자세(reference)와 비교해
  - Z가 0.5 mm 넘게 바뀌었거나 A·B·C가 0.5° 넘게 바뀌면 거부("화살표로 X·Y만 움직이세요").
  - 그 자리(플랜지 X·Y·Z)에서 그리기 범위(limits) 전체를 시뮬레이터로 검사: 닿지 않는 점 0개, 관절 여유 5° 이상, 특이점까지 15° 이상. 부족하면 5 mm 간격으로 X를 늘려 보며 "로봇을 약 N mm 뒤로" 안내(100 mm까지).
- `start_pose.save_ready_pose(config_path, status)`: 설정 파일을 다시 읽어 `start_pose.ready_pose`(tcp_mm, abc_deg, joints_deg, saved_local)만 바꿔 저장. 다른 값·메모는 그대로(임시 파일에 쓴 뒤 교체, 보정과 같은 방식).
- `start_pose.settings(cfg)["ready_pose"]`: 저장한 값(없으면 None). 앱·명령줄이 설정을 읽을 때 같이 불러옴.
- `start_pose.jog_session(...)`: 펜을 올린 뒤 Enter = 현재 좌표와 대기 자세 대비 X·Y·Z·자세 변화, s = 검사 후 저장, q = 저장 안 하고 끝.
- `python -m mirobot_sketch.start_pose --execute`: 호밍 확인 → 펜 올리기 → 화살표로 맞추기 → 저장까지 한 번에.
- `pen_pose.study(..., center_y=)`: 종이 중심이 Y로 옮겨진 경우도 검사.
- 테스트 `tests/test_ready_pose.py` 9개(새 코드 없이 실패 확인 후 구현). 전체 419개 통과(1개 건너뜀).

## 확인

- 가상 로봇으로 펜 올리기 → Enter → s 저장까지 실행: 임시 설정 파일에 ready_pose가 저장됨(관절 여유 5.2°, 특이점까지 36.8°). 저장소의 `robot/drawing_config.json`은 건드리지 않았다.

## 확인하지 못한 것 (실물)

- 앱이 USB로 연결된 동안 컨트롤러 화살표가 동작하는지, 화살표가 직교(X·Y) 이동인지(T3).
- 실물 컨트롤러가 보고하는 A·B·C가 화살표 이동 중에도 그대로인지(T3).
- GUI(그리기 창)의 [이 위치 저장] 버튼은 T5에서 연결 흐름과 함께 만든다.
