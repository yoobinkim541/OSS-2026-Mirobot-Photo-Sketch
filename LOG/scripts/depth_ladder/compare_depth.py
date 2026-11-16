"""누르는 깊이 비교: 깊이 0(예전)과 −5.5 mm(새 설정)로 같은 칸(가로 10 mm 선 + 세로 10 mm 선)을 번갈아 긋고,
가로선의 세로 퍼짐(위로 미끄러짐·끝 꼬리)과 세로선의 길이를 종이 귀퉁이 표시로 편 사진에서 잰다."""
import copy
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")))
import touch_ladder as tl  # noqa: E402
from mirobot_sketch import camera_marks as cmk, draw_executor as dex, start_pose as spm  # noqa: E402

OUT = os.path.join(os.environ["TEMP"], "comparedepth")
os.makedirs(OUT, exist_ok=True)
tl.OUT = OUT
ROWS = (40.0, 10.0, -20.0, -50.0)
COLS = (-45.0, -20.0, 5.0, 30.0)
DEPTHS = (0.0, -5.5)


def cells():
    """[(깊이, x, y)] 그리는 순서. 줄마다 깊이 배치를 바꿔 위치와 깊이가 섞이지 않게."""
    out = []
    for r, y in enumerate(ROWS):
        for c, x in enumerate(COLS):
            out.append((DEPTHS[(r + c) % 2], x, y))
    return out


def strokes(x, y):
    return [[(x, y), (x + 10.0, y)], [(x + 15.0, y), (x + 15.0, y + 10.0)]]


def main():
    cfg = tl.cfg
    cap = tl.open_cam()
    before = tl.shoot(cap, "before")
    marks, _ = cmk.find_corner_marks(before)
    if marks is None:
        raise SystemExit("귀퉁이 표시를 찾지 못해 멈춤")
    print("손 표시", np.round(marks, 1).tolist(), flush=True)
    link = dex.open_link(cfg)
    try:
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
        print("호밍 자세 확인", tl.wait_home(link)["joints_deg"], flush=True)
        spm.execute_requested_start(link, cfg)
        draw_cfg = spm.prepare_requested_drawing(link, cfg)
        print("앱 설정 깊이", draw_cfg["pen"]["down_extra_mm"], flush=True)
        for n, (depth, x, y) in enumerate(cells()):
            c = copy.deepcopy(draw_cfg)
            c["pen"]["down_extra_mm"] = depth
            for line, _ in dex.Planner(c).plan(strokes(x, y))[1:-1]:
                link.send_and_ack(line, cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            print(f"{n + 1}/{len(cells())} 깊이 {depth:+.1f} x {x:+.0f} y {y:+.0f}", flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    after = tl.shoot(cap, "after")
    cap.release()
    marks_after, _ = cmk.find_corner_marks(after, expected_px=marks, radius_px=120)
    if marks_after is None:
        marks_after = marks
    json.dump({"before": marks.tolist(), "after": marks_after.tolist(), "cells": cells()},
              open(os.path.join(OUT, "meta.json"), "w"), indent=1)
    print("저장", OUT, flush=True)


if __name__ == "__main__":
    main()
