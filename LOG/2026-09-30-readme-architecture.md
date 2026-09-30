# 2026-09-30 — README 한·영 분리와 설계 문서(ARCHITECTURE) 추가

## 정의
- 처음 보는 사람이 설치 방법(exe·셸 스크립트), Ubuntu 22.04 + ROS 2 Humble을 쓰는 이유, 패키징, 단계별 기술 선택을 한곳에서 알 수 없었습니다.
- 오픈소스로서 영어 문서가 없었습니다.

## 해결
- `README.md`(한국어)와 `README.en.md`(영어)로 나누고 맨 위에 언어 전환 링크를 두었습니다. 설치는 exe·zip·pip 세 가지와 RViz 환경 설치(설치 도우미, `packaging/wsl/setup_ros_env.sh`, 예비 경로)를 정리했습니다.
- 설계와 기술 선택은 `docs/ARCHITECTURE.md`(영어판 `ARCHITECTURE.en.md`)로 분리했습니다: 11단계별 기술과 선택 이유, 사용한 OpenCV 함수, 종이 좌표 검증, 로봇 실행, Humble을 쓰는 이유, ROS와 로봇 제어의 관계, WSL2↔Windows 브리지, 패키징·CI/CD, UI, 에이전트, 한계, 오픈소스 목록.
- 앱 화면 스크린샷 10장(`assets/screenshots/`)은 NASA 공개 도메인 사진(`assets/sample/`)으로 만들었습니다. 출처와 라이선스는 `THIRD_PARTY_NOTICES.md`에 적었습니다. `input/`의 사진은 저작권 미확인이라 쓰지 않았습니다.
- 사실과 다른 문구를 고쳤습니다: "MoveIt 2를 경로 검토에 사용"(실제로는 쓰지 않음) → "ROS는 3D 보기에만 사용", 없는 `CV/presets.py` → `mirobot_sketch/presets.py`.
- 선택의 trade-off는 Notion 페이지에 따로 적었습니다.

## 확인
- 문서의 상대 링크와 이미지 경로가 모두 실제 파일을 가리키는지 검사했습니다.
- 스크린샷은 앱을 실제로 띄워 캡처했습니다(RViz 창 제목의 사용자 폴더 경로는 잘라냄).
- 코드는 바꾸지 않아 테스트는 다시 돌리지 않았습니다.

## 한계
- ROS 1과 ROS 2를 연결한 작업은 저장소에 없습니다. 문서에는 확인되는 사실(실물 제어는 ROS 없이 시리얼 G-code, ROS 2는 3D 보기 전용)만 적었습니다.
- Ubuntu 22.04 + Humble 이외의 조합(예: Ubuntu 24.04 + Jazzy)은 확인하지 않았습니다.
- GitHub에서 README가 화면에 어떻게 보이는지(표 안 이미지 크기, 앵커)는 PR 미리보기로 확인해야 합니다.
