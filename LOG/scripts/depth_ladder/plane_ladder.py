"""종이 면 기울기 재기: 4줄 × 2열 = 8곳에서 깊이를 바꾼 4 mm 짧은 선 5개를 긋고(9 mm 간격, 왼쪽 얕음 → 오른쪽 깊음),
곳마다 처음 닿는 깊이를 찾아 평면(깊이 = a + b·x + c·y)을 맞춘다.

  실행:  python plane_ladder.py            (로봇으로 긋고 전후 사진 저장 후 분석)
  분석만: python plane_ladder.py --analyze

선마다 판정 창을 따로 둔다(선 x−1.5 ~ x+5.5 mm, y−4 ~ y+15 mm). 그리는 순서는 곳 안에서 섞어
앞 선의 누름이 다음 선 깊이에 겹치지 않게 한다."""
import copy
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
import touch_ladder as tl  # noqa: E402
from mirobot_sketch import camera_marks as cmk, draw_executor as dex, start_pose as spm  # noqa: E402

OUT = os.path.join(os.environ["TEMP"], "planeladder")
os.makedirs(OUT, exist_ok=True)
tl.OUT = OUT
ROWS = (40.0, 15.0, -15.0, -45.0)
COLS = (-25.0, 25.0)
DASH, STEP = 4.0, 9.0
ORDER = (2, 0, 4, 1, 3)  # 곳 안에서 긋는 순서(위치 번호)
P = 8.0


def depths_for(y):
    return (-3.0, -2.0, -1.0, 0.0, 1.0) if y > 30 else (-4.0, -3.0, -2.0, -1.0, 0.0)


def dashes():
    """[(곳 번호, 깊이, 시작 x, y)] 긋는 순서."""
    out, g = [], 0
    for y in ROWS:
        for c in COLS:
            ds = depths_for(y)
            for i in ORDER:
                out.append((g, ds[i], c - 20.0 + STEP * i, y))
            g += 1
    return out

# 마지막으로 확인한 종이 귀퉁이 표시(4K px, 2026-11-13). 카메라를 고정했으므로 새 종이도 이 근처에 있다.
REF_MARKS = np.array([[1100.1, 796.8], [2938.4, 379.2], [2818.3, 1351.9], [1434.1, 1790.9]])


def find_marks(img, ref=REF_MARKS, radius=250):
    """기준 위치 근처에서만 귀퉁이 표시를 찾는다(손·옷을 표시로 잘못 잡는 것 방지)."""
    marks, _ = cmk.find_corner_marks(img, expected_px=ref, radius_px=radius)
    return marks


def occluded_fraction(before, before_marks, img, marks, p=2.0):
    """두 사진을 각자의 귀퉁이로 편 뒤 밝기가 크게 바뀐 면적 비율. 선 16칸은 1% 아래, 손·팔은 훨씬 크다."""
    W, H = int(297 * p), int(210 * p)
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
    g = [cv2.GaussianBlur(cv2.cvtColor(cv2.warpPerspective(im, cv2.getPerspectiveTransform(np.float32(m), dst), (W, H)),
                                       cv2.COLOR_BGR2GRAY), (7, 7), 0).astype(np.float32)
         for im, m in ((before, before_marks), (img, marks))]
    return float((np.abs(g[0] - g[1]) > 25).mean())


def shoot_until_marks(cap, name, ref, radius=120, tries=20, wait_s=3.0, before=None, max_changed=0.05):
    """귀퉁이 표시 네 개가 기준 근처에 모두 보이고, (before를 주면) 종이 안쪽이 가려지지 않을 때까지 다시 찍는다."""
    for i in range(tries):
        img = tl.shoot(cap, name)
        marks = find_marks(img, ref=ref, radius=radius)
        if marks is not None and before is None:
            return img, marks
        if marks is not None:
            changed = occluded_fraction(before, ref, img, marks)
            if changed <= max_changed:
                return img, marks
            why = f"종이 안쪽의 {changed:.0%}가 바뀌어(손·팔로 보임)"
        else:
            why = "귀퉁이가 가려져"
        print(f"'{name}' 사진에서 {why} {wait_s:.0f}초 뒤 다시 찍습니다 ({i + 1}/{tries})", flush=True)
        time.sleep(wait_s)
    raise SystemExit(f"'{name}' 사진이 계속 가려졌습니다(종이를 가리지 않게 비켜 주세요)")



def check_config(link):
    time.sleep(2.5)
    link.ser.read(link.ser.in_waiting or 1)
    link.ser.reset_input_buffer()
    link.ser.write(b"$$\r\n")
    t, buf = time.time(), b""
    while time.time() - t < 10 and b"\nok" not in buf:
        buf += link.ser.read(link.ser.in_waiting or 1)
    got = {ln.split("=")[0]: ln.split("=")[1].split(" ")[0] for ln in buf.decode("utf-8", "replace").splitlines()
           if ln.startswith(("$3=", "$23="))}
    if got.get("$3") != "123" or got.get("$23") != "60":
        raise SystemExit(f"설정이 바뀌어 있어 멈춤: {got}")


def run():
    cfg = tl.cfg
    draw_check = spm.drawing_config_from_recorded_contact(cfg)
    bad = dex.check_limits([[(x, y), (x + DASH, y)] for _, _, x, y in dashes()], draw_check)
    if bad:
        raise SystemExit(f"범위 밖 점이 있어 멈춤: {bad[:3]}")
    cap = tl.open_cam()
    before = tl.shoot(cap, "before")
    marks = find_marks(before)
    if marks is None:
        raise SystemExit("기준 위치 근처에서 귀퉁이 표시 네 개를 찾지 못해 멈춤(가림·카메라 이동)")
    print("귀퉁이 표시", np.round(marks, 1).tolist(), flush=True)
    link = dex.open_link(cfg)
    try:
        check_config(link)
        print("호밍 자세 확인", tl.wait_home(link)["joints_deg"], flush=True)
        spm.execute_requested_start(link, cfg)
        draw_cfg = spm.prepare_requested_drawing(link, cfg)
        print("앱 설정 깊이", draw_cfg["pen"]["down_extra_mm"], flush=True)
        for n, (g, depth, x, y) in enumerate(dashes()):
            c = copy.deepcopy(draw_cfg)
            c["pen"]["down_extra_mm"] = depth
            for line, _ in dex.Planner(c).plan([[(x, y), (x + DASH, y)]])[1:-1]:
                link.send_and_ack(line, cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            print(f"{n + 1}/{len(dashes())} 곳 {g} 깊이 {depth:+.0f} x {x:+.0f} y {y:+.0f}", flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    after, marks_after = shoot_until_marks(cap, "after", marks, before=before)
    cap.release()
    json.dump({"before": marks.tolist(), "after": marks_after.tolist(), "dashes": dashes()},
              open(os.path.join(OUT, "meta.json"), "w"), indent=1)


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
        for _, depth, x, y in meta["dashes"]:
            cv2.line(t, to_px((x, y), off), to_px((x + DASH, y), off), 1, int(1.5 * P))
        return float((t * inkd).sum())

    off = max(((score((ox, oy)), (ox, oy)) for ox in np.arange(-15, 15.1, 1.0)
               for oy in np.arange(-15, 15.1, 1.0)))[1]
    print("계획 → 종이 장 이동 mm", off)
    res = {}
    for gi, depth, x, y in meta["dashes"]:
        x0, y0 = to_px((x - 1.5, y + 15), off)
        x1, y1 = to_px((x + DASH + 1.5, y - 4), off)
        w = ink[y0:y1, x0:x1]
        cols = int((w.sum(0) > 0).sum())
        rows = np.nonzero(w.sum(1))[0]
        span = (rows.max() - rows.min()) / P if len(rows) else 0.0
        res.setdefault(gi, []).append((depth, x, y, cols / P, span))
    groups = []
    for gi in sorted(res):
        rs = sorted(res[gi])
        touched = [r[3] >= 1.5 for r in rs]
        first = next((r[0] for r, t in zip(rs, touched) if t), None)
        last_no = max((r[0] for r, t in zip(rs, touched) if not t and (first is None or r[0] < first)), default=None)
        thr = None if first is None else (first - 0.5 if last_no is not None else None)
        cx, cy = rs[0][1] + 20.0, rs[0][2]
        mono = all(touched[i] <= touched[i + 1] for i in range(len(touched) - 1))
        groups.append((gi, cx, cy, thr, first, mono))
        print(f"곳 {gi} (x {cx:+.0f}, y {cy:+.0f}): " + ", ".join(
            f"{r[0]:+.0f}:{'O' if t else '.'}({r[3]:.1f}/{r[4]:.1f})" for r, t in zip(rs, touched))
            + f"  → 처음 닿음 {first}, 문턱 {thr}{'' if mono else '  (단조 아님)'}")
    pts = [(cx, cy, thr) for _, cx, cy, thr, _, _ in groups if thr is not None]
    if len(pts) >= 3:
        M = np.array([[1.0, p[0], p[1]] for p in pts])
        v = np.array([p[2] for p in pts])
        coef, *_ = np.linalg.lstsq(M, v, rcond=None)
        resid = v - M @ coef
        print(f"평면: 문턱 = {coef[0]:+.2f} {coef[1]:+.4f}·x {coef[2]:+.4f}·y (mm), 잔차 최대 {np.abs(resid).max():.2f} mm")
        json.dump({"offset_mm": off, "groups": groups, "plane": coef.tolist(),
                   "resid": resid.tolist()}, open(os.path.join(OUT, "result.json"), "w"), indent=1)
    vis = cv2.cvtColor(np.uint8(np.clip(255 - np.clip(d, 0, None) * 20, 0, 255)), cv2.COLOR_GRAY2BGR)
    for gi, depth, x, y in meta["dashes"]:
        cv2.line(vis, to_px((x, y), off), to_px((x + DASH, y), off), (220, 60, 0), 1)
        cv2.putText(vis, f"{depth:+.0f}", to_px((x - 0.5, y - 4), off), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 60, 0), 1)
    a, b = to_px((-52, 58), off), to_px((52, -55), off)
    cv2.imwrite(os.path.join(OUT, "plane.jpg"), vis[a[1]:b[1], a[0]:b[0]])
    print("저장", os.path.join(OUT, "plane.jpg"))


if __name__ == "__main__":
    if "--analyze" not in sys.argv:
        run()
    analyze()
