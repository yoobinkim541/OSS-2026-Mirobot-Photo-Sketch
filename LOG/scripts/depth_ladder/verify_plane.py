"""평면 보정 확인: 새 설정(앱 설정 그대로: 평면 보정 + requested_pen_down_extra_mm)과 예전 설정(평면 0, 깊이 0)을
4줄 × 4열 칸(가로 10 mm 선 + 그 오른쪽 5 mm에 세운 세로 10 mm 선)에 줄마다 번갈아 긋고, 칸마다 선이 생겼는지와
가로선의 세로 퍼짐(끌림)을 잰다.

  실행:  python verify_plane.py
  분석만: python verify_plane.py --analyze"""
import copy
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plane_ladder as pl  # noqa: E402

tl, dex, spm = pl.tl, pl.dex, pl.spm
OUT = os.path.join(os.environ["TEMP"], next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--out=")), "verifyplane"))
os.makedirs(OUT, exist_ok=True)
tl.OUT = OUT
ROWS = (40.0, 10.0, -20.0, -50.0)
COLS = (-45.0, -20.0, 5.0, 30.0)
P = 8.0


def cells():
    return [("new" if (r + c) % 2 else "old", x, y) for r, y in enumerate(ROWS) for c, x in enumerate(COLS)]


def strokes(x, y):
    return [[(x, y), (x + 10.0, y)], [(x + 15.0, y), (x + 15.0, y + 10.0)]]


def old_cfg(c):
    c = copy.deepcopy(c)
    c["plane_compensation"]["a_per_mm_y"] = 0.0
    c["plane_compensation"]["b_per_mm_z"] = 0.0
    c["pen"]["down_extra_mm"] = 0.0
    return c


def run():
    cfg = tl.cfg
    check = spm.drawing_config_from_recorded_contact(cfg)
    bad = dex.check_limits([s for _, x, y in cells() for s in strokes(x, y)], check)
    if bad:
        raise SystemExit(f"범위 밖 점이 있어 멈춤: {bad[:3]}")
    cap = tl.open_cam()
    before = tl.shoot(cap, "before")
    marks = pl.find_marks(before)
    if marks is None:
        raise SystemExit("기준 위치 근처에서 귀퉁이 표시 네 개를 찾지 못해 멈춤(가림·카메라 이동)")
    print("귀퉁이 표시", np.round(marks, 1).tolist(), flush=True)
    link = dex.open_link(cfg)
    try:
        pl.check_config(link)
        print("호밍 자세 확인", tl.wait_home(link)["joints_deg"], flush=True)
        spm.execute_requested_start(link, cfg)
        new = spm.prepare_requested_drawing(link, cfg)
        print("새 설정 깊이", new["pen"]["down_extra_mm"], "평면", new["plane_compensation"], flush=True)
        for n, (kind, x, y) in enumerate(cells()):
            c = new if kind == "new" else old_cfg(new)
            for line, _ in dex.Planner(c).plan(strokes(x, y))[1:-1]:
                link.send_and_ack(line, cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            print(f"{n + 1}/{len(cells())} {kind} x {x:+.0f} y {y:+.0f}", flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    after, marks_after = pl.shoot_until_marks(cap, "after", marks, before=before)
    cap.release()
    json.dump({"before": marks.tolist(), "after": marks_after.tolist(), "cells": cells()},
              open(os.path.join(OUT, "meta.json"), "w"), indent=1)


def reshoot():
    """로봇 없이 그린 뒤 사진만 다시 찍는다(뒤 사진이 가려졌을 때)."""
    meta = json.load(open(os.path.join(OUT, "meta.json")))
    cap = tl.open_cam()
    before = cv2.imread(os.path.join(OUT, "before.jpg"))
    _, marks_after = pl.shoot_until_marks(cap, "after", np.array(meta["before"]), before=before)
    cap.release()
    meta["after"] = marks_after.tolist()
    json.dump(meta, open(os.path.join(OUT, "meta.json"), "w"), indent=1)


def to_px(p, off):
    return int((148.5 + p[0] + off[0]) * P), int((105.0 - (p[1] + off[1])) * P)


def analyze():
    meta = json.load(open(os.path.join(OUT, "meta.json")))
    W, H = int(297 * P), int(210 * P)
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)

    def warp(name, m):
        im = cv2.imread(os.path.join(OUT, name + ".jpg"))
        return cv2.warpPerspective(im, cv2.getPerspectiveTransform(np.float32(m), dst), (W, H))

    g = lambda im: cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    B, A = warp("before", meta["before"]), warp("after", meta["after"])
    d = g(B) - g(A)
    d = d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 41).astype(np.float32) - 128)
    ink = cv2.morphologyEx((d > 4.0).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    inkd = cv2.dilate(ink, np.ones((int(3 * P), int(3 * P)), np.uint8))

    def score(off):
        t = np.zeros_like(ink)
        for _, x, y in meta["cells"]:
            for s in strokes(x, y):
                cv2.line(t, to_px(s[0], off), to_px(s[1], off), 1, int(1.5 * P))
        return float((t * inkd).sum())

    # 칸 간격(25·30 mm)의 절반보다 좁게 찾고, 점수가 같으면 이동이 작은 쪽
    off = max(((score((ox, oy)), -abs(ox) - abs(oy), (ox, oy)) for ox in np.arange(-12, 12.1, 1.0)
               for oy in np.arange(-12, 12.1, 1.0)))[2]
    print("계획 → 종이 장 이동 mm", off)
    rows = {}
    for kind, x, y in meta["cells"]:
        x0, ya = to_px((x - 1, y + 15), off)
        x1, yb = to_px((x + 11, y - 5), off)
        w = ink[ya:yb, x0:x1]
        cols = (w.sum(0) > 0).sum() / P
        r = np.nonzero(w.sum(1))[0]
        span = (r.max() - r.min()) / P if len(r) else None
        vx0, vy0 = to_px((x + 12, y + 16), off)
        vx1, vy1 = to_px((x + 18, y - 6), off)
        rv = np.nonzero(ink[vy0:vy1, vx0:vx1].sum(1))[0]
        vlen = (rv.max() - rv.min()) / P if len(rv) else 0.0
        drawn = cols >= 5.0
        rows.setdefault(kind, []).append((x, y, drawn, cols, span, vlen))
        print(f"{kind} ({x:+.0f},{y:+.0f}): 가로선 잉크 폭 {cols:.1f} mm{' (그려짐)' if drawn else ' (없음/짧음)'}, "
              f"세로 퍼짐 {span} mm, 세로선 {vlen:.1f} mm")
    for kind, rs in sorted(rows.items()):
        sp = [r[4] for r in rs if r[2] and r[4] is not None]
        print(f"== {kind}: 그려짐 {sum(r[2] for r in rs)}/{len(rs)}, 가로선 세로 퍼짐 평균 "
              f"{np.mean(sp) if sp else float('nan'):.1f} mm (최대 {max(sp) if sp else float('nan'):.1f})")
    vis = cv2.cvtColor(np.uint8(np.clip(255 - np.clip(d, 0, None) * 20, 0, 255)), cv2.COLOR_GRAY2BGR)
    for kind, x, y in meta["cells"]:
        col = (0, 160, 0) if kind == "new" else (60, 60, 220)
        for s in strokes(x, y):
            cv2.line(vis, to_px(s[0], off), to_px(s[1], off), col, 1)
        cv2.putText(vis, kind, to_px((x, y - 4), off), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    a, b = to_px((-52, 58), off), to_px((52, -60), off)
    cv2.imwrite(os.path.join(OUT, "verify.jpg"), vis[a[1]:b[1], a[0]:b[0]])
    cv2.imwrite(os.path.join(OUT, "verify_raw.jpg"), A[a[1]:b[1], a[0]:b[0]])
    print("저장", os.path.join(OUT, "verify.jpg"))


if __name__ == "__main__":
    if "--reshoot" in sys.argv:
        reshoot()
    elif "--analyze" not in sys.argv:
        run()
    analyze()
