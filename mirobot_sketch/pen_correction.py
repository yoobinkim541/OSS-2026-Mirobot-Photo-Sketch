"""
끝점 세로 줄 보정 — 카메라 확인 결과만으로 정하는 규칙 (LLM·네트워크 없이 로컬에서 동작)
=========================================================================================
펜을 세게 누른 채 내리거나 떼면 펜 끝이 미끄러져 획의 시작·끝점에 세로 줄이 생깁니다.
vision.LineCheck.end_ticks()가 잰 끝점마다의 세로 줄로, 시작점/끝점을 종이에서 뒤로 물리는 양
(drawing_config.json pen.start_backoff_mm / pen.end_backoff_mm)을 한 단계씩 고칩니다.

- 세로 줄이 난 끝점 비율이 TICK_RATE 이상이면 그쪽(시작/끝)을 STEP_MM 더 물림 (최대 END_BACKOFF_MAX_MM)
- 선이 안 보인 획 비율이 앞 구간(바꾸기 전 값으로 그린 획)보다 MISSING_RISE 넘게 늘면 너무 물린 것으로 보고 둘 다
  STEP_MM 되돌림 (0 아래로는 안 감). 앞 구간이 없으면 MISSING_RATE 이상일 때만. 종이 위치 오차·연한 선 때문에 원래
  안 보이는 획이 있어도(2026-11-10: 80획 중 28) 그것만으로 되돌리지 않게 비교로 판단함.
- 지금 값으로 그린 획만 셈: 값을 바꾼 뒤 새로 그린 끝점이 MIN_ENDPOINTS개 모일 때까지는 다시 바꾸지 않음

물리는 방향은 종이에서 멀어지는 쪽뿐이고 범위가 0~1 mm라, 시뮬레이션한 접촉 자세보다 종이를 더 누르지 않습니다.
"""

import json

from . import paths, vision
from .draw_executor import END_BACKOFF_MAX_MM

STEP_MM = 0.2
TICK_RATE = 0.2           # 잰 끝점 중 이 비율 이상에 세로 줄이 있으면 더 물림 (2026-11-10 실제 종이: 시작 12/52, 끝 10/52)
MISSING_RATE = 0.5        # 앞 구간이 없을 때: 잰 획 중 이 비율 이상에서 선이 안 보이면 덜 물림
MISSING_RISE = 0.15       # 앞 구간보다 선이 안 보인 획 비율이 이만큼 늘면 덜 물림
MIN_ENDPOINTS = 8         # 이만큼 잰 끝점이 모여야 판단


def _round(v):
    return round(min(END_BACKOFF_MAX_MM, max(0.0, v)), 2)


def missing_rate(coverage, strokes):
    """strokes(획 번호들) 중 잰 획에서 선이 안 보인 비율, 잰 획이 없으면 None."""
    measured = [coverage[i] for i in strokes if coverage.get(i) is not None]
    return sum(1 for v in measured if v < vision.LINE_SEEN) / len(measured) if measured else None


def next_backoff(current, ticks, coverage, baseline_missing=None):
    """current: {"start": mm, "end": mm}. ticks: {획: (시작 mm|None, 끝 mm|None)} (지금 값으로 그린 획만).
    coverage: {획: 선이 보인 비율 또는 None}. baseline_missing: 바꾸기 전 값으로 그린 획의 안 보인 비율 (없으면 None).
    반환 (새 값 dict, 이유 문자열) — 바꿀 것이 없으면 (current, "")."""
    measured = [i for i in ticks if coverage.get(i) is not None]
    if len(measured) >= MIN_ENDPOINTS and (current["start"] > 0 or current["end"] > 0):
        missing = missing_rate(coverage, measured)
        limit = MISSING_RATE if baseline_missing is None else baseline_missing + MISSING_RISE
        if missing >= limit:
            new = {k: _round(v - STEP_MM) for k, v in current.items()}
            return new, f"선이 안 보인 획 {missing:.0%}: 끝점을 {STEP_MM:g} mm 덜 물림"
    summary = vision.tick_summary(ticks, coverage)
    new, why = dict(current), []
    for kind, name in (("start", "시작점"), ("end", "끝점")):
        hits, n = summary[kind]
        if n >= MIN_ENDPOINTS and hits / n >= TICK_RATE and current[kind] < END_BACKOFF_MAX_MM:
            new[kind] = _round(current[kind] + STEP_MM)
            why.append(f"{name} 세로 줄 {hits}/{n}")
    if not why:
        return current, ""
    return new, " · ".join(why) + f": {STEP_MM:g} mm 더 물림"


def current_backoff(cfg):
    pen = cfg["pen"]
    return {"start": float(pen.get("start_backoff_mm", 0.0)), "end": float(pen.get("end_backoff_mm", 0.0))}


def save_backoff(values, config_path=None):
    """설정 파일의 pen.start_backoff_mm / end_backoff_mm만 바꿔 저장 (다른 값은 파일 그대로)."""
    from .calibration import save_config
    path = config_path or paths.config_path()
    data = json.loads(path.read_text(encoding="utf-8"))
    data["pen"]["start_backoff_mm"] = values["start"]
    data["pen"]["end_backoff_mm"] = values["end"]
    save_config(path, data)
