"""compare_depth 결과 분석: 전후 사진을 각자의 귀퉁이 표시로 펴고, 계획 그림을 잉크에 맞춰(이동만 찾음) 칸마다
가로선의 세로 퍼짐·가로선 높이 차이·세로선 길이를 깊이별로 비교한다."""
import json
import os

import cv2
import numpy as np

import sys

D = os.path.join(os.environ["TEMP"], sys.argv[1] if len(sys.argv) > 1 else "comparedepth")
P = 8.0
W, H = int(297 * P), int(210 * P)
DST = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
meta = json.load(open(os.path.join(D, "meta.json")))


def warp(name, m):
    im = cv2.imread(os.path.join(D, name + ".jpg"))
    return cv2.warpPerspective(im, cv2.getPerspectiveTransform(np.float32(m), DST), (W, H))


g = lambda im: cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
B, A = warp("before", meta["before"]), warp("after", meta["after"])
d = g(B) - g(A)
d = d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 41).astype(np.float32) - 128)
ink = cv2.morphologyEx((d > 4.0).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def to_px(p, off):
    return int((148.5 + p[0] + off[0]) * P), int((105.0 - (p[1] + off[1])) * P)


def template(off):
    t = np.zeros_like(ink)
    for depth, x, y in meta["cells"]:
        cv2.line(t, to_px((x, y), off), to_px((x + 10, y), off), 1, int(1.5 * P))
        cv2.line(t, to_px((x + 15, y), off), to_px((x + 15, y + 10), off), 1, int(1.5 * P))
    return t


if len(sys.argv) > 3:  # 같은 종이의 앞 실행에서 찾은 이동을 그대로 쓴다
    off = (float(sys.argv[2]), float(sys.argv[3]))
else:
    inkd = cv2.dilate(ink, np.ones((int(3 * P), int(3 * P)), np.uint8))
    best = max(((float((template((ox, oy)) * inkd).sum()), (ox, oy))
                for ox in np.arange(-15, 15.1, 1.0) for oy in np.arange(-15, 15.1, 1.0)))
    off = best[1]
print("계획→종이 장 이동 mm", off)

rows = {}
for depth, x, y in meta["cells"]:
    # 가로선: 좌우 끝 1 mm 안쪽, 위로 15 mm·아래로 5 mm 창
    x0, ya = to_px((x - 1, y + 15), off)
    x1, yb = to_px((x + 11, y - 5), off)
    w = ink[ya:yb, x0:x1]
    r = np.nonzero(w.sum(1))[0]
    if len(r):
        top, bot = (ya + r.min()), (ya + r.max())
        base = to_px((x, y), off)[1]
        span = (bot - top) / P
        mid_up = (base - (top + bot) / 2) / P
        # 가로선 몸통 높이: 행마다 잉크가 가장 많은 행
        body = (base - (ya + int(np.argmax(w.sum(1))))) / P
    else:
        span = mid_up = body = None
    # 세로선: 위아래 5 mm 여유
    vx0, vy0 = to_px((x + 12, y + 16), off)
    vx1, vy1 = to_px((x + 18, y - 6), off)
    v = ink[vy0:vy1, vx0:vx1]
    rv = np.nonzero(v.sum(1))[0]
    vlen = (rv.max() - rv.min()) / P if len(rv) else None
    rows.setdefault(depth, []).append((x, y, span, body, vlen))
    print(f"깊이 {depth:+.1f} ({x:+.0f},{y:+.0f}): 가로선 세로 퍼짐 {span} mm, 몸통 높이(계획 대비 위+) {body} mm, 세로선 길이 {vlen} mm")

for depth, rs in sorted(rows.items()):
    sp = [r[2] for r in rs if r[2] is not None]
    bd = [r[3] for r in rs if r[3] is not None]
    vl = [r[4] for r in rs if r[4] is not None]
    if not sp:
        print(f"== 깊이 {depth:+.1f}: 선 보임 0/{len(rs)}")
        continue
    print(f"== 깊이 {depth:+.1f}: 가로선 세로 퍼짐 평균 {np.mean(sp):.1f} mm (최대 {np.max(sp):.1f}), "
          f"몸통 높이 {np.mean(bd):+.1f}±{np.std(bd):.1f} mm, 세로선 길이 {np.mean(vl) if vl else float('nan'):.1f} mm (계획 10), "
          f"선 보임 {len(sp)}/{len(rs)}")

vis = cv2.cvtColor(np.uint8(np.clip(255 - np.clip(d, 0, None) * 20, 0, 255)), cv2.COLOR_GRAY2BGR)
for depth, x, y in meta["cells"]:
    col = (0, 160, 0) if depth == 0 else (200, 80, 0)
    cv2.line(vis, to_px((x, y), off), to_px((x + 10, y), off), col, 1)
    cv2.line(vis, to_px((x + 15, y), off), to_px((x + 15, y + 10), off), col, 1)
    cv2.putText(vis, f"{depth:+.1f}", to_px((x, y - 4), off), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
a, b = to_px((-52, 58), off), to_px((52, -60), off)
cv2.imwrite(os.path.join(D, "compare.jpg"), vis[a[1]:b[1], a[0]:b[0]])
print("저장", os.path.join(D, "compare.jpg"))
