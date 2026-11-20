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
- 비스듬히 대고 떼기(pen.edge_rise_mm)를 켜 두었으면(0보다 크면) 세로 줄의 방향으로 그 값도 고침: 위·아래를 가를 수 있는
  끝점 중 아래로 뻗은 비율이 RISE_RATE 이상이고 위보다 확실히 많으면(아래 참고) RISE_STEP_MM 더 위로 비스듬히,
  반대면 덜 (RISE_STEP_MM ~ EDGE_RISE_MAX_MM). 0이면 꺼 둔 것으로 보고 건드리지 않음 (명령 수가 바뀌지 않게)
- 뗄 때 비스듬히가 켜져 있으면 내릴 때 값(pen.edge_land_mm)도 시작점 세로 줄의 방향으로 고침: 아래 줄이 많으면 RISE_STEP_MM 더
  위에서 내려오며(−) 대고, 위 줄이 많으면 덜(0까지, next_edge_land)
- 비스듬히 값(뗄 때·댈 때·아래쪽 추가)은 방향을 가른 끝점이 DIR_MIN_ENDPOINTS개 넘게 모이고, 한쪽 줄이 반대쪽보다 우연으로
  보기 어려울 만큼(부호 검정 한쪽 p ≤ DIR_P) 많을 때만 한 단계 고친다. 11-18 두 그림: 구간마다 끝점 10~24개(예: 아래 5·위 2)로
  판단해, 같은 −1 mm를 15:53에는 "아래", 20:44에는 "위"로 보고 반대로 움직였다. 끝난 뒤 사진으로 다시 재면 −0.75 구간도
  101개 중 아래 21·위 10이었다.
- 한 그림 안에서 어떤 값이 바꾼 방향과 반대로 다시 바뀌려 하면(앞뒤 판단이 그 사이를 가리킴), 지금 값과 새 값의 가운데로 두고
  그 값은 그 그림에서 더 바꾸지 않음(settle_changes). 다른 값은 계속 고친다. 11-18 15:53 그림: 내릴 때 값이 −2(위 줄) → −1.5 →
  −1(아래 줄)로 움직인 뒤 −1.5로 돌아가려 했는데, 전체를 멈추는 예전 규칙 때문에 나머지 획을 −1로 그렸고 사진에서 그 구간에
  아래 줄이 많았다.

물리는 방향은 종이에서 멀어지는 쪽뿐이고 범위가 0~1 mm라, 시뮬레이션한 접촉 자세보다 종이를 더 누르지 않습니다.
비스듬히 대고 떼기도 접촉점은 그대로이고 종이에서 떨어진 중간점만 위·아래로 옮깁니다.
"""

import json
from math import comb

from . import paths, vision
from .draw_executor import EDGE_RISE_LOW_FROM_MM, EDGE_RISE_LOW_SPAN_MM, EDGE_RISE_MAX_MM, END_BACKOFF_MAX_MM

STEP_MM = 0.2
TICK_RATE = 0.2           # 잰 끝점 중 이 비율 이상에 세로 줄이 있으면 더 물림 (2026-11-10 실제 종이: 시작 12/52, 끝 10/52)
MISSING_RATE = 0.5        # 앞 구간이 없을 때: 잰 획 중 이 비율 이상에서 선이 안 보이면 덜 물림
MISSING_RISE = 0.15       # 앞 구간보다 선이 안 보인 획 비율이 이만큼 늘면 덜 물림
MIN_ENDPOINTS = 8         # 이만큼 잰 끝점이 모여야 판단
RISE_STEP_MM = 0.5
RISE_RATE = 0.1           # 위·아래를 가른 끝점 중 한쪽 세로 줄이 이 비율 이상이면 (2026-11-16: 아래 약 16%, 위 약 6%)
DIR_MIN_ENDPOINTS = 30    # 비스듬히 값: 위·아래를 가른 끝점이 이만큼 모여야 판단 (11-18: 10~24개로는 같은 값을 반대로 봄)
DIR_P = 0.05              # 그리고 한쪽 줄이 많은 것이 부호 검정(한쪽) 이 값 이하일 때만 (양쪽 다 많으면 방향 문제가 아님)
RISE_LOW_MARGIN = 0.1     # 종이 아래쪽 추가: 아래쪽 비율이 위쪽보다 이만큼 넘게 클 때만


def _round(v):
    return round(min(END_BACKOFF_MAX_MM, max(0.0, v)), 2)


def reversed_keys(last_dirs, old, new):
    """old·new: {"start", "end", "rise", "rise_low": mm}. last_dirs: {키: +1|−1} 이번 그림에서 마지막으로 바꾼 방향.
    반환 (앞 방향과 반대로 바뀐 키 목록, 바꾼 방향을 더한 새 last_dirs)."""
    dirs, flipped = dict(last_dirs), []
    for k, v in new.items():
        if v == old.get(k):
            continue
        d = 1 if v > old[k] else -1
        if dirs.get(k, d) != d:
            flipped.append(k)
        dirs[k] = d
    return flipped, dirs


def settle_changes(locked, last_dirs, old, new):
    """규칙들이 낸 새 값(new)을 정리. locked(잠근 키)는 old 그대로. 앞 방향과 반대로 바뀌려는 키는 old와 new의 가운데로 두고
    잠근다(앞뒤 판단이 그 사이를 가리키므로, 0.5 mm 단계로 오가지 않게). 반환 (정리한 값, 새 locked, 새 last_dirs, 가운데로 둔 키)."""
    new = {k: (old[k] if k in locked else v) for k, v in new.items()}
    flipped, dirs = reversed_keys(last_dirs, old, new)
    for k in flipped:
        new[k] = round((old[k] + new[k]) / 2, 3)
    return new, set(locked) | set(flipped), dirs, flipped


def missing_rate(coverage, strokes):
    """strokes(획 번호들) 중 잰 획에서 선이 안 보인 비율, 잰 획이 없으면 None."""
    measured = [coverage[i] for i in strokes if coverage.get(i) is not None]
    return sum(1 for v in measured if v < vision.LINE_SEEN) / len(measured) if measured else None


def next_backoff(current, ticks, coverage, baseline_missing=None, kinds=("start", "end")):
    """current: {"start": mm, "end": mm}. ticks: {획: (시작 mm|None, 끝 mm|None)} (지금 값으로 그린 획만).
    coverage: {획: 선이 보인 비율 또는 None}. baseline_missing: 바꾸기 전 값으로 그린 획의 안 보인 비율 (없으면 None).
    kinds: 세로 줄로 더 물릴 쪽. 비스듬히 값(next_edge_rise·next_edge_land)이 끝점·시작점 줄을 맡을 때는 ()로 넘긴다. 그러면 물림은
    자동으로 움직이지 않는다: 같은 줄로 두 값이 한꺼번에 움직이지 않게 하고, "선이 안 보이면 덜 물림"도 쉰다. 2026-11-19 접촉 누름을
    1 mm 줄인 뒤로는 선이 옅어 '안 보임'이 실제보다 많이 잡혀(사용자 확인: 빠진 선 없음), 교수님 사진(109획)에서 앞 40획의 54%로 시작점
    물림이 0.4 → 0.2로 내려가 저장됐다. 반환 (새 값 dict, 이유 문자열) — 바꿀 것이 없으면 (current, "")."""
    if not kinds:
        return current, ""
    measured = [i for i in ticks if coverage.get(i) is not None]
    if len(measured) >= MIN_ENDPOINTS and (current["start"] > 0 or current["end"] > 0):
        missing = missing_rate(coverage, measured)
        limit = MISSING_RATE if baseline_missing is None else baseline_missing + MISSING_RISE
        if missing >= limit:
            new = {k: _round(v - STEP_MM) for k, v in current.items()}
            return new, f"선이 안 보인 획 {missing:.0%}: 끝점을 {STEP_MM:g} mm 덜 물림"
    summary = vision.tick_summary(ticks, coverage)
    new, why = dict(current), []
    for kind, name in (k for k in (("start", "시작점"), ("end", "끝점")) if k[0] in kinds):
        hits, n = summary[kind]
        if n >= MIN_ENDPOINTS and hits / n >= TICK_RATE and current[kind] < END_BACKOFF_MAX_MM:
            new[kind] = _round(current[kind] + STEP_MM)
            why.append(f"{name} 세로 줄 {hits}/{n}")
    if not why:
        return current, ""
    return new, " · ".join(why) + f": {STEP_MM:g} mm 더 물림"


def _endpoint_dirs(dirs, coverage, strokes=None, keep=None, end=1):
    """선이 보인 획의 끝점(end=1, 펜을 뗀 자리) 또는 시작점(end=0, 펜을 댄 자리) 중 위·아래를 가른 것의 (위 mm, 아래 mm) 목록.
    strokes·keep을 주면 그 점의 종이 y가 keep(y)인 것만. 뗄 때 값(edge_rise)은 끝점으로만 고친다: 시험 종이(2026-11-16)에서
    시작점 아래 줄은 뗄 때 비스듬히 값과 상관없이 약 2.3 mm였다(내릴 때 짐이 걸리며 생기는 줄). 시작점은 내릴 때 값(edge_land)으로 고친다."""
    out = []
    for i, pair in dirs.items():
        if coverage.get(i) is None or coverage[i] < vision.LINE_SEEN:
            continue
        d = pair[end]
        if d is None:
            continue
        if keep is not None and not keep(float(strokes[i][-1 if end else 0][1])):
            continue
        out.append(d)
    return out


def _rates(vals):
    n = len(vals)
    up = sum(1 for u, _ in vals if u >= vision.TICK_MIN_MM)
    down = sum(1 for _, d in vals if d >= vision.TICK_MIN_MM)
    return n, up, down


def _clearly_more(a, b):
    """줄 a개 대 b개: 위·아래가 반반이라면 a개 이상 나올 확률(한쪽 부호 검정)이 DIR_P 이하인가."""
    n = a + b
    return a > b and sum(comb(n, k) for k in range(a, n + 1)) / 2 ** n <= DIR_P


def next_edge_rise(current, dirs, coverage, strokes=None):
    """current: pen.edge_rise_mm (뗄 때). dirs: {획: (시작 (위 mm, 아래 mm)|None, 끝 ...)} (지금 값으로 그린 획만,
    LineCheck.end_tick_dirs). 선이 보인 획의 끝점만 셈. strokes(종이 mm 획)를 주면 아래쪽 추가(edge_rise_low_mm)가 걸리지 않는
    높이(y ≥ EDGE_RISE_LOW_FROM_MM)의 끝점만 본다. 반환 (새 값, 이유) — 바꿀 것이 없거나 꺼져 있으면 (current, "")."""
    if current <= 0:
        return current, ""
    keep = (lambda y: y >= EDGE_RISE_LOW_FROM_MM) if strokes is not None else None
    n, up, down = _rates(_endpoint_dirs(dirs, coverage, strokes, keep))
    if n < DIR_MIN_ENDPOINTS:
        return current, ""
    if down / n >= RISE_RATE and _clearly_more(down, up) and current < EDGE_RISE_MAX_MM:
        new = round(min(EDGE_RISE_MAX_MM, current + RISE_STEP_MM), 2)
        return new, f"아래로 끌린 끝점 {down}/{n} (위 {up}): 비스듬히 {new:g} mm로"
    if up / n >= RISE_RATE and _clearly_more(up, down) and current > RISE_STEP_MM:
        new = round(max(RISE_STEP_MM, current - RISE_STEP_MM), 2)
        return new, f"위로 뻗은 끝점 {up}/{n} (아래 {down}): 비스듬히 {new:g} mm로"
    return current, ""


def next_edge_rise_low(current, base, dirs, coverage, strokes):
    """종이 아래쪽 추가(pen.edge_rise_low_mm). 아래쪽(y < EDGE_RISE_LOW_FROM_MM − 폭/2, 추가가 절반 넘게 걸리는 곳) 끝점과
    위쪽(y ≥ EDGE_RISE_LOW_FROM_MM) 끝점을 비교해, 아래쪽만 아래 줄이 RISE_RATE 이상이고 위쪽보다 RISE_LOW_MARGIN 넘게 많으면
    RISE_STEP_MM 더(합이 EDGE_RISE_MAX_MM까지), 아래쪽만 위 줄이 그렇게 많으면 덜(0까지). base(edge_rise_mm)가 0이면 그대로."""
    if base <= 0:
        return current, ""
    low_y = EDGE_RISE_LOW_FROM_MM - EDGE_RISE_LOW_SPAN_MM / 2
    n_lo, up_lo, down_lo = _rates(_endpoint_dirs(dirs, coverage, strokes, lambda y: y < low_y))
    n_hi, up_hi, down_hi = _rates(_endpoint_dirs(dirs, coverage, strokes, lambda y: y >= EDGE_RISE_LOW_FROM_MM))
    if n_lo < DIR_MIN_ENDPOINTS or n_hi < DIR_MIN_ENDPOINTS:
        return current, ""
    room = EDGE_RISE_MAX_MM - base
    if down_lo / n_lo >= RISE_RATE and down_lo / n_lo - down_hi / n_hi > RISE_LOW_MARGIN and current < room:
        new = round(min(room, current + RISE_STEP_MM), 2)
        return new, f"종이 아래쪽 아래 줄 {down_lo}/{n_lo} (위쪽 {down_hi}/{n_hi}): 아래쪽 +{new:g} mm로"
    if up_lo / n_lo >= RISE_RATE and up_lo / n_lo - up_hi / n_hi > RISE_LOW_MARGIN and current > 0:
        new = round(max(0.0, current - RISE_STEP_MM), 2)
        return new, f"종이 아래쪽 위 줄 {up_lo}/{n_lo} (위쪽 {up_hi}/{n_hi}): 아래쪽 +{new:g} mm로"
    return current, ""


def next_edge_land(current, rise, dirs, coverage):
    """current: pen.edge_land_mm (내릴 때, −이면 위에서 내려오며 댐). 선이 보인 획의 시작점 중 위·아래를 가른 것을 세어,
    아래 줄이 RISE_RATE 이상이고 위보다 확실히(_clearly_more) 많으면 RISE_STEP_MM 더 위에서(−), 위 줄이 그렇게 많으면 덜(0까지).
    시험 종이(2026-11-18, 카메라): 종이 왼쪽 시작점 아래 줄이 0/−1/−2/−3 mm에서 4.2/3.5/1.3/1.2 mm, −3에서 위로 1.9 mm.
    rise(edge_rise_mm)가 0이면 그대로: 비스듬히 대고 떼기 명령이 없거나 이 값이 0이 되며 명령 수가 바뀔 수 있음."""
    if rise <= 0:
        return current, ""
    n, up, down = _rates(_endpoint_dirs(dirs, coverage, end=0))
    if n < DIR_MIN_ENDPOINTS:
        return current, ""
    if down / n >= RISE_RATE and _clearly_more(down, up) and current > -EDGE_RISE_MAX_MM:
        new = round(max(-EDGE_RISE_MAX_MM, current - RISE_STEP_MM), 2)
        return new, f"아래로 끌린 시작점 {down}/{n} (위 {up}): {abs(new):g} mm 위에서 내려오며 대기로"
    if up / n >= RISE_RATE and _clearly_more(up, down) and current < 0:
        new = round(min(0.0, current + RISE_STEP_MM), 2)
        return new, f"위로 뻗은 시작점 {up}/{n} (아래 {down}): {abs(new):g} mm 위에서 내려오며 대기로"
    return current, ""


def current_rise_low(cfg):
    return float(cfg["pen"].get("edge_rise_low_mm", 0.0))


def current_land(cfg):
    return float(cfg["pen"].get("edge_land_mm", 0.0))


def current_backoff(cfg):
    pen = cfg["pen"]
    return {"start": float(pen.get("start_backoff_mm", 0.0)), "end": float(pen.get("end_backoff_mm", 0.0))}


def current_rise(cfg):
    return float(cfg["pen"].get("edge_rise_mm", 0.0))


def save_pen(values, config_path=None):
    """설정 파일의 pen.start_backoff_mm / end_backoff_mm와, values에 있으면 edge_rise_mm("rise")·edge_rise_low_mm("rise_low")·
    edge_land_mm("land")만 바꿔 저장 (다른 값은 파일 그대로)."""
    from .calibration import save_config
    path = config_path or paths.config_path()
    data = json.loads(path.read_text(encoding="utf-8"))
    data["pen"]["start_backoff_mm"] = values["start"]
    data["pen"]["end_backoff_mm"] = values["end"]
    for key, name in (("rise", "edge_rise_mm"), ("rise_low", "edge_rise_low_mm"), ("land", "edge_land_mm")):
        if key in values:
            data["pen"][name] = values[key]
    save_config(path, data)
