"""자동 세팅: 사진마다 목표 시간 안에서 인물이 가장 닮아 보이는 선 정리·명암 설정을 찾는다.

후보(잔선 정리 강도 × 명암 빗금)를 실제 파이프라인으로 돌려 보고, 예상 시간(앱 시간 카드와 같은 계산)이
목표 안에 드는 후보 중 닮음 점수가 가장 높은 것을 고른다. 어느 후보도 시간 안에 못 들면 얼굴 밖에서 덜 중요한
선부터 빼는 선 개수 상한(clutter_max)을 이분 탐색해 맞춘다.

닮음 점수 (0~1, 클수록 닮음)
- face      : 눈(눈썹)·코·입 영역과 얼굴 윤곽 띠에서 사진의 뚜렷한 윤곽 중 그려진 비율의 평균 (가장 큰 비중)
- silhouette: 배경을 지운 인물 외곽선 중 그려진 비율 (머리 모양·어깨선)
- detail    : 사진 전체의 뚜렷한 윤곽 중 그려진 비율 (머리 결·옷 무늬)
- tone      : 큰 덩어리의 밝고 어두움이 사진과 닮은 정도 (compare의 tone_match)
- precision : 그린 윤곽 중 사진 윤곽 근처인 비율 (낮으면 잡음)
"""

import sys
import time

import cv2
import numpy as np

from . import declutter
from . import tone as tone_mod

TARGET_MIN = 45.0
# 잔선 정리 강도 사다리 (느슨 → 엄격): (얼굴 밖 짧은 선 기준 mm, 흐린 선 기준 백분위). 첫 칸은 기본값보다 느슨해
# 시간이 남는 사진은 잔선을 더 살려 본다.
# 백분위는 75까지만 올린다: 그 위로는 글자·로고 획(대비 100~130)까지 흐린 선으로 지워짐. 더 줄일 때는 선 개수 상한으로
STRICTNESS = ((2.0, 50), (3.0, 60), (4.0, 60), (5.0, 65), (6.0, 70), (8.0, 75), (10.0, 75), (12.0, 75))
# 명암 빗금 후보: (단계, 간격 mm). 0단계는 빗금 없음.
# 스케치는 채색 전 첫 단계라 면을 빗금으로 메우지 않는다(사용자 결정): 머리카락은 결 선(hair_flow, 실사 프리셋)으로
# 그리고, 남는 시간은 잔선을 덜 지우는 데 쓴다. 빗금이 필요하면 tone_levels를 직접 올린다.
TONES = ((0, 2.0),)
# 상한으로도 시간을 못 맞출 때 결 선을 끄기 전에 넓혀 볼 머리 결 간격(mm). 넓히면 결 선 수가 준다.
HAIR_GAPS = (3.0, 4.0)
WEIGHTS_FACE = {"face": 0.40, "silhouette": 0.15, "detail": 0.15, "tone": 0.15, "precision": 0.15}
WEIGHTS_NO_FACE = {"face": 0.0, "silhouette": 0.30, "detail": 0.25, "tone": 0.20, "precision": 0.25}
MIN_EDGE_PX = 12          # 영역 안 사진 윤곽이 이보다 적으면 그 영역은 점수에서 뺌 (가려진 눈 등)


def _ellipse_mask(shape, center, axes):
    m = np.zeros(shape, np.uint8)
    cv2.ellipse(m, (int(round(center[0])), int(round(center[1]))),
                (max(1, int(round(axes[0]))), max(1, int(round(axes[1])))), 0, 0, 360, 1, -1)
    return m.astype(bool)


def _recall(drawn, edges, region=None):
    e = edges if region is None else edges & region
    n = int(e.sum())
    return (float(drawn[e].mean()), n) if n >= MIN_EDGE_PX else (None, n)


def likeness(session):
    """지금 결과의 닮음 점수 dict (score와 항목별 값). session.result가 있어야 함."""
    with session.lock:
        r = session.result
        gray = r["base"]
        outline = [np.asarray(e["poly"], np.float64) for e in session.table.values() if e["kind"] == "stroke"]
        pen_px = session.params["pen_mm"] / session._xf()[0]
        faces = list(r["stages"]["face"].get("faces") or [])
    shape = gray.shape[:2]
    ink = (tone_mod.ink_image(outline, shape, pen_px) < 128).astype(np.uint8)
    drawn = cv2.dilate(ink, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))) > 0
    edges = tone_mod.photo_edges(gray) > 0

    face_scores = []
    for f in faces:
        x, y, w, h = (float(v) for v in f.box)
        zones = declutter.feature_zones(f)
        # 입 속(이 사이 경계)은 일부러 그리지 않고 입 윤곽 하나로 다시 그리므로(declutter.tidy_mouth) 입 영역 안쪽 55%의
        # 사진 윤곽은 세지 않고 입술 윤곽 띠만 재현을 본다. 눈·코는 영역 전체.
        parts = [_ellipse_mask(shape, c, a) for c, a in zones[:3]]
        (mc, ma) = zones[3]
        parts.append(_ellipse_mask(shape, mc, ma) & ~_ellipse_mask(shape, mc, (0.55 * ma[0], 0.55 * ma[1])))
        center = (x + w / 2, y + h / 2)
        ring = _ellipse_mask(shape, center, (0.75 * w, 0.8 * h)) & ~_ellipse_mask(shape, center, (0.42 * w, 0.45 * h))
        vals = [v for v, _ in (_recall(drawn, edges, m) for m in parts + [ring]) if v is not None]
        if vals:
            face_scores.append(float(np.mean(vals)))
    face = float(np.mean(face_scores)) if face_scores else None

    subject = gray < 245                                   # 배경 제거(흰 바탕)한 인물
    silhouette = None
    if 0.02 < subject.mean() < 0.95:
        subject = cv2.morphologyEx(subject.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        rim = cv2.morphologyEx(subject, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        silhouette, _ = _recall(drawn, rim)
    metrics, _ = session.compare()
    parts = {"face": face, "silhouette": silhouette, "detail": metrics.get("edge_recall"),
             "tone": metrics["tone_match"], "precision": metrics.get("edge_precision")}
    weights = dict(WEIGHTS_FACE if face is not None else WEIGHTS_NO_FACE)
    if silhouette is None:                                 # 외곽선을 못 재면 그 비중을 윤곽 정확도로
        weights["precision"] += weights.pop("silhouette")
    total = sum(weights.values())
    score = sum(weights[k] * (parts[k] or 0.0) for k in weights) / total
    return {"score": round(score, 4), **{k: (None if v is None else round(float(v), 3)) for k, v in parts.items()}}


def _minutes(result):
    return result["timing"]["total_s"] / 60.0


def tune(session, target_min=TARGET_MIN, progress=None):
    """목표 시간 안에서 닮음 점수가 가장 높은 설정을 찾아 세션에 적용. 반환: 보고 dict.
    progress(i, text)로 진행을 알림. 잔선 정리는 실사 선화 경로에서만 효과가 있으므로, 지금 결과에서 얼굴을 찾았는데
    이미지 종류가 실사 사진이 아니면 실사 사진 프리셋으로 바꾼 뒤 찾는다. 도중에 실패하면 원래 설정으로 되돌리고 다시 올림."""
    before = (getattr(session, "image_type", None), getattr(session, "detail", None), dict(session.params))
    try:
        return _search(session, target_min, progress)
    except Exception:
        image_type, detail, params = before
        if image_type is not None and image_type != getattr(session, "image_type", None):
            session.apply_preset(image_type, detail)
        session.update_params(params)
        raise


def _search(session, target_min, progress):
    started = time.monotonic()
    report = {"target_min": target_min, "tried": []}
    faces = (((getattr(session, "result", None) or {}).get("stages") or {}).get("face") or {}).get("faces")
    if faces and getattr(session, "image_type", "photo") != "photo":
        session.apply_preset("photo")
        report["preset_switched"] = "photo"
    step = [0]

    def run(changes):
        session.update_params(changes)
        r = session.run_current()
        step[0] += 1
        entry = {"params": dict(changes), "minutes": round(_minutes(r), 1),
                 "strokes": len(r["stages"]["simplify"]["strokes"]),
                 "hatch": len(r["stages"]["tone"].get("hatch_strokes", []))}
        report["tried"].append(entry)
        if progress:
            progress(step[0], f"후보 {step[0]}: {entry['minutes']}분")
        return r, entry

    def scored(entry):
        entry["likeness"] = likeness(session)
        return entry

    def budget(common):
        """가장 엄격한 강도로도 넘으면 선 개수 상한을 이분 탐색: 목표 안에서 가장 많이 남기는 상한."""
        r, entry = run(common)
        hi, lo, best = len(r["stages"]["clutter"]["strokes"]) - 1, 20, None
        while lo <= hi:
            mid = (lo + hi) // 2
            _, entry = run({**common, "clutter_max": mid})
            if entry["minutes"] <= target_min:
                best, lo = scored(entry), mid + 1
            else:
                hi = mid - 1
        return best

    flow = bool(session.params.get("hair_flow"))
    gap0 = float(session.params.get("hair_spacing_mm", 2.0))
    base = {"declutter": True, "clutter_max": 0, "hair_flow": flow, "hair_spacing_mm": gap0}
    feasible = []
    for levels, spacing in TONES:
        tone_p = {"tone_levels": levels, "tone_spacing_mm": spacing}
        hit = None
        for mm, pct in STRICTNESS:                         # 가장 느슨한(선이 많은) 것부터: 시간에 처음 드는 강도
            _, entry = run({**base, **tone_p, "clutter_mm": mm, "clutter_pct": pct})
            if entry["minutes"] <= target_min:
                hit = scored(entry)
                break
        strict = {**base, **tone_p, "clutter_mm": STRICTNESS[-1][0], "clutter_pct": STRICTNESS[-1][1]}
        if hit is None and levels <= 1:                    # 강도만으로 안 되면 덜 중요한 선부터 빼서 맞춰 봄
            hit = budget(strict)
        # 상한은 결 선을 줄이지 못한다(빗금처럼 따로 그어지고, 상한으로 뺀 머리 획 자리를 다시 채우는 것으로 봄). 긴 생머리 사진은
        # 상한 20개로도 47.4분(결 선 39획)이었다. 스케치는 머리를 결 선으로 묘사하므로(사용자 결정) 먼저 결 간격을 넓히고,
        # 그래도 안 될 때만 결 선을 끈다(같은 사진 결 없이 상한 20개면 39.7분).
        for gap in (g for g in HAIR_GAPS if flow and g > gap0):
            if hit is None and levels <= 1:
                hit = budget({**strict, "hair_spacing_mm": gap})
        if hit is None and levels <= 1 and flow:
            hit = budget({**strict, "hair_flow": False})
        if hit is not None:
            feasible.append(hit)
    if feasible:
        chosen = max(feasible, key=lambda e: e["likeness"]["score"])
        report["within_target"] = True
    else:                                                  # 상한 20개로도 못 맞춤: 가장 빠른 후보
        chosen = min(report["tried"], key=lambda e: e["minutes"])
        report["within_target"] = False
    r, _ = run(chosen["params"])
    if "likeness" not in chosen:
        scored(chosen)                                     # 지금 세션이 이 후보의 결과일 때 채점
    report["chosen"] = {**chosen, "minutes": round(_minutes(r), 1)}
    report["candidates"] = len(report["tried"])
    report["hair_flow_dropped"] = flow and chosen["params"].get("hair_flow") is False
    gap = chosen["params"].get("hair_spacing_mm", gap0)
    report["hair_gap_widened"] = gap if flow and not report["hair_flow_dropped"] and gap > gap0 else None
    report["elapsed_s"] = round(time.monotonic() - started, 1)
    return report


def summary(report):
    """사람이 읽는 한 줄."""
    c = report["chosen"]
    p = c["params"]
    like = c.get("likeness") or {}
    tone_txt = f"빗금 {p['tone_levels']}단계 {p['tone_spacing_mm']}mm" if p.get("tone_levels") else "빗금 없음"
    cap = f", 선 상한 {p['clutter_max']}" if p.get("clutter_max") else ""
    cap += ", 머리 결 선 끔(시간)" if report.get("hair_flow_dropped") else ""
    cap += f", 결 간격 {report['hair_gap_widened']:g}mm(시간)" if report.get("hair_gap_widened") else ""
    head = "자동 세팅" if report["within_target"] else f"자동 세팅(목표 {report['target_min']:g}분 초과)"
    if report.get("preset_switched"):
        head += "(얼굴이 있어 실사 사진 프리셋으로)"
    return (f"{head}: 예상 {c['minutes']}분 · 잔선 기준 {p['clutter_mm']:g}mm/{p['clutter_pct']} · {tone_txt}{cap}"
            f" · 닮음 {like.get('score', 0):.2f} (후보 {report['candidates']}개, {report['elapsed_s']:.0f}초)")


def side_by_side(session, height=800):
    """원본 사진 | 그려질 획을 실제 펜 굵기로 같은 크기에 나란히 (1px 선으로 그리면 빗금이 실제보다 훨씬 옅어 보임)."""
    with session.lock:
        r = session.result
        photo, o = r["color"], r["stages"]
        strokes = [np.asarray(e["poly"], np.float64) for e in session.table.values() if e["kind"] == "stroke"]
        strokes += [np.asarray(q, np.float64) for q in o["tone"].get("hatch_strokes", [])]
        pen_px = session.params["pen_mm"] / session._xf()[0]
    k = height / photo.shape[0]
    size = (int(round(photo.shape[1] * k)), height)
    ink = tone_mod.ink_image([q * k for q in strokes], (height, size[0]), max(1, int(round(pen_px * k))))
    return np.hstack([cv2.resize(photo, size, interpolation=cv2.INTER_AREA), cv2.cvtColor(ink, cv2.COLOR_GRAY2BGR)])


def _write_png(path, img):
    """cv2.imwrite는 Windows 한글 경로에서 조용히 실패하므로 인코딩 후 직접 씀."""
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise ValueError(f"PNG 인코딩 실패: {path}")
    buf.tofile(str(path))


def _out_name(out, path, taken):
    """결과 파일 이름: 보통 사진 이름 그대로. 이번 실행에서 이미 쓴 이름이거나, 폴더에 다른 사진의 결과(<이름>_strokes.json의
    source.image가 다른 파일)가 있으면 확장자를, 그래도 겹치면 번호를 붙인다 (a.webp를 따로 돌린 뒤 a.jfif가 덮어썼음)."""
    import json
    import os
    from pathlib import Path

    p = Path(path)
    me = os.path.normcase(os.path.abspath(path))
    ext = p.suffix.lstrip(".")
    candidates = [p.stem, f"{p.stem}_{ext}"] + [f"{p.stem}_{ext}_{i}" for i in range(2, 100)]
    for name in candidates:
        if name in taken:
            continue
        prev = Path(out) / f"{name}_strokes.json"
        try:
            src = json.loads(prev.read_text(encoding="utf-8"))["source"]["image"]
        except (OSError, ValueError, KeyError, TypeError):
            return name                                     # 결과가 없거나 읽을 수 없음: 새로 씀
        if os.path.normcase(os.path.abspath(src)) == me:
            return name                                     # 같은 사진의 예전 결과: 덮어씀
    raise ValueError(f"결과 이름을 정하지 못했습니다: {path}")



def main(argv=None):
    """사진 여러 장에 자동 세팅을 돌려 미리보기·보고·획 파일을 저장 (앱과 같은 세션·계산).
    python -m mirobot_sketch.autotune 사진1 사진2 ... [--target 45] [--out out/auto]
    획 파일(<이름>_strokes.json)은 앱의 '획 저장'과 같은 형식이라 mirobot-draw로 바로 그릴 수 있다."""
    import argparse
    import json
    from pathlib import Path

    from . import paper_mapping as pm
    from . import sketch_pipeline as sp
    from .session import SketchSession

    ap = argparse.ArgumentParser(description="사진마다 목표 시간 안에서 인물이 가장 닮아 보이는 설정 찾기")
    ap.add_argument("images", nargs="+")
    ap.add_argument("--target", type=float, default=TARGET_MIN, help="목표 그리기 시간(분)")
    ap.add_argument("--out", default="out/auto", help="미리보기·보고·획 파일을 저장할 폴더")
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    taken = set()
    for path in args.images:
        name = _out_name(out, path, taken)
        taken.add(name)
        s = SketchSession()
        s.set_image(path)
        s.apply_preset("photo")
        s.run_current()
        show = (lambda i, text: print(f"  {name}: {text}", end="\r", flush=True)) if sys.stdout.isatty() else None
        rep = tune(s, args.target, progress=show)
        print(f"{name}: {summary(rep)}" + " " * 10)
        _write_png(out / f"{name}_auto.png", side_by_side(s))
        _write_png(out / f"{name}_paper.png", s.render("paper"))
        r = s.result
        metrics = sp.stroke_metrics(r["strokes_mm"], start=(0.0, 0.0))
        metrics["estimated_time"] = r["timing"]
        source = {"image": r["path"], "image_size_px": [int(r["base"].shape[1]), int(r["base"].shape[0])],
                  "params": {**r["params"], "type": s.image_type, "detail": s.detail, "tool": "autotune"}}
        pm.save_strokes_json(out / f"{name}_strokes.json",
                             pm.build_strokes_document(r["strokes_mm"], r["placement"], metrics, source))
        rep["params"] = dict(s.params)
        (out / f"{name}_auto.json").write_text(json.dumps(rep, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
