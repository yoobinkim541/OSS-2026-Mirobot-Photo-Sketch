# 2026-09-29 — 전체 파이프라인 검증에서 나온 문제 수정, 캘리브레이션 선택 사항으로 변경

## 검증에서 확인한 것 (수정 전)

- `python -m unittest discover -s tests`: 300개 중 실패 2건, 건너뜀 1건.
- 사진 3장(`photo1_mic`, `illust1_color`, `photo3_chair`, 100 mm)을 가상 로봇 공중 모드로 끝까지 실행: 명령 2,871 / 3,347 / 3,809줄 전부 전송·응답, 전송된 G-code를 종이 mm로 되돌리면 원래 획과 최대 0.0005 mm 차이, 실행 기록·저장 획·진행 파일 일치.
- 오류 경로: 30번째 명령 컨트롤러 오류 → 그 뒤 전송 없음, 실행 중 멈춤 → `stopped_by_user`, 최종 확인에서 취소 → 이동 명령 0개, 시작 TCP가 20 mm 벗어남 → ③에서 거부, ±40 mm 밖 그림 펜다운 → 사전 검사에서 거부.
- 실제 로봇 연결은 하지 않았다(로봇 없음, COM9 열리지 않음).

## 문제와 수정

1. **`test_default_box_fits_executor_limits_exactly` 실패** — 테스트가 실제 `robot/drawing_config.json`을 읽는데, 9/28 캘리브레이션이 `limits`를 ±40 mm로 바꿔 가정(±50 mm)이 깨졌다. 실물 보정이 저장소 설정을 바꿀 때마다 테스트가 흔들리는 구조가 원인.
   - 수정: `tests/golden.py`에 `default_cfg()`(패키지 기본 설정 사본)를 추가하고 `test_agent`, `test_draw_job`, `test_draw_executor`, `test_sim`, `test_virtual_robot`, `test_face_session`, `test_limits`가 이를 쓰게 했다.
2. **`test_packaged_default_config_has_same_fields_as_repo_config` 실패** — 저장소 설정에만 `calibration` 항목이 있었다.
   - 수정: `mirobot_sketch/data/drawing_config.json`에 `calibration`(`pending_measurement`, 값은 null)과 설명 항목을 추가.
3. **설치판 기본 설정이 저장소 설정과 값이 다름**(종이 중심 TCP, `limits`) — 그대로 둠. 실물 값은 설치마다 보정으로 정해지는 값이고 구조만 같으면 된다.

## 사용자 결정: 캘리브레이션 없이 바로 그리기

사용자가 "캘리브레이션 없이 바로 그리게 해도 될듯"이라고 결정했다. 이전에는 `plane_compensation.status`가 `verified`가 아니면 실물 연결 때마다 캘리브레이션이 자동으로 시작됐다.

- 앱: 캘리브레이션은 "종이·펜 위치 변경: 다시 보정"을 골랐을 때만 실행한다. 고르지 않으면 호밍 → 시작 위치 확인 → 최종 확인 → 그리기로 바로 진행한다. 넓은 범위 사진의 접촉 영역 측정도 이 선택을 했을 때만 한다.
- 명령줄: `mirobot-draw --execute --calibrate`일 때만 캘리브레이션. 없으면 바로 진행한다.
- 최종 확인 화면에 보정이 안 된 실물 연결이면 "종이 보정 없음(설정에 저장된 값 사용)"을 표시한다.
- 사용하지 않게 된 `calibration.calibration_needed()`와 그 테스트를 제거했다.
- **그대로 유지한 안전장치**: 종이 허용 범위(`limits`) 검사(밖이면 펜다운 거부, 공중 모드만 가능), 시작 TCP가 종이 중심에서 5 mm 이내인지 검사, 관절 시뮬레이션 FAIL 거부, 사람의 최종 확인 체크, 오류 시 자동 복구 없이 멈춤.
- 거부 메시지를 "설정된 펜 접촉 영역 밖…"으로 바꾸고 다시 보정 방법을 안내한다.

### 알아 둘 점 (사실 vs 미확인)

- 보정 없이 그리면 `robot/drawing_config.json`에 저장된 평면 보정값(9/28 측정, 상태 `needs_recalibration`)과 `limits ±40 mm`를 그대로 쓴다. 그 뒤 종이·펜 위치를 조정했으므로 **펜 접촉과 선질은 실물로 확인된 적 없다.**
- 펜다운으로 ±40 mm보다 큰 그림은 그릴 수 없다. 더 크게 그리려면 `limits`를 직접 넓히거나 다시 보정해야 한다.
- 실물 실행은 아직 한 번도 이 경로로 하지 않았다(로봇 없음). 실제 종이에서 확인이 필요하다: F 방향 시험, 새 설정에서 짧은 시험선의 접촉·선질, 그 다음 사진 전체.

## 테스트

- 추가: 앱 실물 실행이 캘리브레이션 없이 펜다운 그리기까지 가는 테스트, CLI가 `--calibrate` 없이는 캘리브레이션을 하지 않는 테스트.
- 변경: 다시 보정을 고른 경우와 `--calibrate`가 있는 경우로 기존 테스트를 바꿈.
- 전체 실행 결과: 301개 중 실패 1건 — `test_thousands_of_proposals_draw_fast_with_capped_labels`(제한 5초, 7.6초). 이번 실행이 644초로 평소(약 290초)의 두 배 이상 걸린 부하 상태에서만 발생했고, 단독 실행에서는 통과(17.7초). 시간에 민감한 GUI 테스트라 부하에 약하다(별도 개선 후보).
