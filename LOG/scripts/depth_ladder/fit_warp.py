"""scale_rows 결과로 '그림 좌표 미리 비틀기(drawing_warp)' 계수를 맞춘다. 로봇은 움직이지 않는다.

세로선 20개 각각의 아래 끝(펜을 내린 점, 계획 (x, y))을 사진에서 찾고,
실제 위치 → 계획 위치의 2차 다항식 G(x, y) = c1·x + c2·y + c3·x² + c4·xy + c5·y² (상수항 0, 종이 중심 고정)을 맞춘다.
실제 위치는 종이를 비뚤게 붙인 회전만 빼고(강체 회전) 쓴다. 계획 좌표를 G에 넣어 보내면 실제로는 계획대로 그려진다.

  python fit_warp.py [결과 폴더 이름(기본 scalerows)] [--write]   (--write면 robot/drawing_config.json에 기록)"""
import json
import os
import sys

import cv2
import numpy as np
from scipy.signal import find_peaks

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
P = 8.0
ROWS = (40.0, 10.0, -20.0, -50.0)
XS = (-40.0, -20.0, 0.0, 20.0, 40.0)


def line_ends(d_dir):
    """선마다 (아래 끝 x, 아래 끝 y, 위 끝 x, 위 끝 y) 종이 장 mm. 줄은 잉크가 몰린 띠 4개로 찾는다."""
    meta = json.load(open(os.path.join(d_dir, "meta.json")))
    W, H = int(297 * P), int(210 * P)
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
    warp = lambda n, m: cv2.warpPerspective(cv2.imread(os.path.join(d_dir, n + ".jpg")),
                                            cv2.getPerspectiveTransform(np.float32(m), dst), (W, H))
    g = lambda im: cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    d = g(warp("before", meta["before"])) - g(warp("after", meta["after"]))
    d = np.clip(d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 41).astype(np.float32) - 128), 0, None)
    d[d < 6] = 0
    x0, x1 = int((148.5 - 75) * P), int((148.5 + 90) * P)
    rp = np.convolve(d[:, x0:x1].mean(1), np.ones(int(6 * P)) / (6 * P), "same")
    rows, _ = find_peaks(rp, distance=int(15 * P), prominence=0.05)
    rows = sorted(sorted(rows, key=lambda i: -rp[i])[:4])
    out = []
    for yi in rows:
        y0, y1 = yi - int(12 * P), yi + int(12 * P)
        band = d[y0:y1]
        prof = np.convolve(band.mean(0), np.ones(9) / 9, "same")
        pk, _ = find_peaks(prof[x0:x1], distance=int(12 * P))
        pk = sorted(sorted(pk, key=lambda i: -prof[i + x0])[:5])
        for i in pk:
            cx = i + x0
            win = band[:, cx - int(6 * P):cx + int(6 * P)]
            ys, xs_ = [], []
            for r in range(win.shape[0]):
                c = win[r]
                if c.sum() > 0:
                    ys.append(r)
                    xs_.append((np.arange(len(c)) * c).sum() / c.sum())
            ys, xs_ = np.array(ys, float), np.array(xs_, float)
            k, b = np.polyfit(ys, xs_, 1)
            yb, yt = ys.max(), ys.min()                      # 그림 아래 = 행 번호가 큰 쪽
            to_mm = lambda r, c: ((cx - int(6 * P) + c) / P - 148.5, 105 - (y0 + r) / P)
            out.append(to_mm(yb, k * yb + b) + to_mm(yt, k * yt + b))
    return np.array(out)


def terms(x, y):
    return np.stack([x, y, x * x, x * y, y * y], 1)


def main():
    name = next((a for a in sys.argv[1:] if not a.startswith("--")), "scalerows")
    ends = line_ends(os.path.join(os.environ["TEMP"], name))
    plan = np.array([(x, y) for y in ROWS for x in XS])
    if len(ends) != len(plan):
        raise SystemExit(f"선 {len(ends)}개를 찾았습니다(20개여야 함)")
    drawn = ends[:, :2]
    # 종이를 비뚤게 붙인 회전과 이동은 뺀다: 계획 → 실제의 강체 맞춤
    cp, cd = plan.mean(0), drawn.mean(0)
    u, _, vt = np.linalg.svd((plan - cp).T @ (drawn - cd))
    rot = (u @ vt).T
    # 계획 원점이 그려진 곳(앞쪽 근사): 원점 가까운 점들로 1차 맞춘 값
    a = np.c_[np.ones(len(plan)), plan, plan ** 2, plan[:, :1] * plan[:, 1:]]
    origin = np.array([np.linalg.lstsq(a, drawn[:, k], rcond=None)[0][0] for k in range(2)])
    rel = (drawn - origin) @ rot          # 실제 위치(종이 회전 뺌, 원점 기준)
    t = terms(rel[:, 0], rel[:, 1])
    cx_, *_ = np.linalg.lstsq(t, plan[:, 0], rcond=None)
    cy_, *_ = np.linalg.lstsq(t, plan[:, 1], rcond=None)
    pred = np.c_[t @ cx_, t @ cy_]
    print(f"종이 회전 {np.degrees(np.arctan2(rot[1, 0], rot[0, 0])):+.2f}°, 계획 원점이 그려진 곳 {origin.round(2)}")
    print("x 계수 [x, y, x², xy, y²]:", np.round(cx_, 6).tolist())
    print("y 계수 [x, y, x², xy, y²]:", np.round(cy_, 6).tolist())
    print(f"역사상 잔차 RMS {np.sqrt(((pred - plan) ** 2).sum(1).mean()):.2f} mm, 최대 {np.abs(pred - plan).max():.2f} mm")
    # 계획 점을 G에 넣었을 때 보낼 좌표(눈으로 확인)
    for (x, y) in ((-40, 40), (40, 40), (-40, -50), (40, -50), (0, 0)):
        tt = terms(np.array([x], float), np.array([y], float))
        print(f"계획 ({x:+d},{y:+d}) → 보낼 좌표 ({(tt @ cx_)[0]:+.2f}, {(tt @ cy_)[0]:+.2f})")
    if "--write" in sys.argv:
        path = os.path.join(REPO, "robot", "drawing_config.json")
        cfg = json.load(open(path, encoding="utf-8"))
        cfg["drawing_warp"] = {"x": [round(float(v), 7) for v in cx_], "y": [round(float(v), 7) for v in cy_],
                               "terms": ["x", "y", "x^2", "xy", "y^2"], "status": f"fitted_{name}"}
        cfg["_drawing_warp_note"] = (
            "그림 좌표 미리 비틀기: 계획한 종이 좌표 (x, y)를 이 2차식으로 바꿔 보내면 실제로는 계획대로 그려진다"
            "(종이 중심 고정, 상수항 없음). 2026-11-13 scale_rows(설정 하나로 그은 세로선 20개)의 아래 끝 위치로 맞춤. "
            "위로 갈수록 가로·세로가 크게 그려지고 세로선이 부채꼴로 기우는 현상을 보정한다. 원인(J4·J6 추정)은 미확인. "
            "없거나 null이면 끔.")
        json.dump(cfg, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("기록함:", path)


if __name__ == "__main__":
    main()
