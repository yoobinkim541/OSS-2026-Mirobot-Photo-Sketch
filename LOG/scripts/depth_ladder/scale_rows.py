"""높이별 가로 배율 재기: 앱 설정 하나로만(평면 보정·깊이 그대로) 4줄(y +40/+10/−20/−50)에
세로 8 mm 짧은 선 5개를 20 mm 간격(x −40…+40)으로 긋고, 줄마다 선 사이 간격을 사진에서 잰다.
깊이를 칸마다 바꾸지 않으므로, 간격 차이는 위치에 따른 배율만 반영한다.

  실행:  python scale_rows.py
  분석만: python scale_rows.py --analyze"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plane_ladder as pl  # noqa: E402

tl, dex, spm = pl.tl, pl.dex, pl.spm
OUT = os.path.join(os.environ["TEMP"], next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--out=")), "scalerows"))
os.makedirs(OUT, exist_ok=True)
tl.OUT = OUT
ROWS = (40.0, 10.0, -20.0, -50.0)
XS = (-40.0, -20.0, 0.0, 20.0, 40.0)
LEN = 8.0
P = 8.0


def strokes():
    return [[(x, y), (x, y + LEN)] for y in ROWS for x in XS]


def run():
    cfg = tl.cfg
    check = spm.drawing_config_from_recorded_contact(cfg)
    bad = dex.check_limits(strokes(), check)
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
        c = spm.prepare_requested_drawing(link, cfg)
        print("설정 깊이", c["pen"]["down_extra_mm"], "평면", c["plane_compensation"], flush=True)
        for n, s in enumerate(strokes()):
            for line, _ in dex.Planner(c).plan([s])[1:-1]:
                link.send_and_ack(line, cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            print(f"{n + 1}/{len(strokes())} x {s[0][0]:+.0f} y {s[0][1]:+.0f}", flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    _, marks_after = pl.shoot_until_marks(cap, "after", marks, before=before)
    cap.release()
    json.dump({"before": marks.tolist(), "after": marks_after.tolist()},
              open(os.path.join(OUT, "meta.json"), "w"), indent=1)


def analyze():
    """줄마다 세로선 x 위치(종이 장 mm, 카메라 값은 자 측정과 맞음)를 찾아 간격을 낸다."""
    from scipy.signal import find_peaks
    meta = json.load(open(os.path.join(OUT, "meta.json")))
    W, H = int(297 * P), int(210 * P)
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)

    def warp(name, m):
        im = cv2.imread(os.path.join(OUT, name + ".jpg"))
        return cv2.warpPerspective(im, cv2.getPerspectiveTransform(np.float32(m), dst), (W, H))

    g = lambda im: cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    d = g(warp("before", meta["before"])) - g(warp("after", meta["after"]))
    d = np.clip(d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 41).astype(np.float32) - 128), 0, None)
    # 줄 띠는 세로선이 실제로 있는 높이에서 찾는다: 가로 투영으로 잉크가 몰린 띠 4개
    rowprof = np.convolve(d.mean(1), np.ones(int(4 * P)) / (4 * P), "same")
    rpk, _ = find_peaks(rowprof, distance=int(18 * P))
    rpk = sorted(sorted(rpk, key=lambda i: -rowprof[i])[:4])
    out = []
    for yi, plan_y in zip(rpk, ROWS):
        band = d[max(0, yi - int(5 * P)):yi + int(5 * P)]
        prof = np.convolve(band.mean(0), np.ones(5) / 5, "same")
        pk, _ = find_peaks(prof, distance=int(12 * P))
        pk = sorted(sorted(pk, key=lambda i: -prof[i])[:5])
        xs = [i / P - 148.5 for i in pk]
        gaps = np.diff(xs).round(1).tolist()
        out.append({"plan_y": plan_y, "sheet_y": round(105 - yi / P, 1), "xs": [round(v, 1) for v in xs], "gaps": gaps,
                    "scale": round(float(np.mean(gaps)) / 20.0, 3) if gaps else None})
        print(f"줄 y {plan_y:+.0f} (종이 {105 - yi / P:+.1f}): 선 x {[round(v, 1) for v in xs]}, 간격 {gaps} mm, "
              f"평균 배율 {out[-1]['scale']}")
    json.dump(out, open(os.path.join(OUT, "result.json"), "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    if "--analyze" not in sys.argv:
        run()
    analyze()
