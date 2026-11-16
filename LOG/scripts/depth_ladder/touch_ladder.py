"""높이별로 펜이 막 닿는 깊이 찾기: 네 줄(종이 y +45/+15/−20/−55 mm)에 5 mm 가로 짧은 선 10개씩,
접촉 깊이를 기준(2번 접촉 자세의 평면)보다 −7 ~ +2 mm로 바꿔 긋고, 전후 사진 차이로 잉크가 생긴 가장 얕은 깊이를 찾는다.
줄마다 얕은→깊은 방향을 번갈아(좌→우, 우→좌) 가로 위치와 깊이가 섞이지 않게 한다."""
import copy
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, ROOT)
from mirobot_sketch import (camera as camm, camera_marks as cmk, draw_executor as dex,  # noqa: E402
                            start_pose as spm, vision)

OUT = os.path.join(os.environ["TEMP"], "touchladder")
os.makedirs(OUT, exist_ok=True)
cfg = json.load(open(os.path.join(ROOT, "robot", "drawing_config.json"), encoding="utf-8"))
HM_PREV = np.array([[1168.9, 794.5], [3003.0, 407.1], [2864.3, 1378.4], [1473.2, 1786.9]])
PPM, FRAME = 4.0, (3840, 2160)
ROWS = (45.0, 15.0, -20.0, -55.0)
OFFSETS = tuple(float(v) for v in range(-7, 3))           # 기준보다 덜(−)·더(+) 들어가는 mm (막 닿는 깊이 찾기)
SHEET_T = (3.97, 0.15)        # 로봇 종이 mm → 종이 장 중심 기준 mm 어림 이동(등록값). 창을 넓게 잡아 오차를 흡수
XS = tuple(float(v) for v in range(-45, 46, 10))
DASH = 5.0


def dashes():
    """[(줄 y, 깊이, 시작 x)] 그리는 순서."""
    out = []
    for r, y in enumerate(ROWS):
        xs = XS if r % 2 == 0 else XS[::-1]
        out += [(y, off, x) for off, x in zip(OFFSETS, xs)]
    return out


def open_cam():
    settings = camm.capture_settings("Logitech BRIO")
    cap = camm.open_capture(0)[0]
    camm.apply_size(cap, settings)
    camm.settle_size(cap, settings, threading.Event(), timeout=8)
    camm.apply_focus(cap, settings)
    return cap


def shoot(cap, name, n=25):
    time.sleep(1.5)
    for _ in range(n):
        cap.read()
    ok, f = cap.read()
    cv2.imwrite(os.path.join(OUT, name + ".jpg"), f)
    return f


def wait_home(link):
    deadline, steady = time.time() + 240, 0
    while True:
        st = dex.parse_status(link.query_status()[2])
        j = st.get("joints_deg")
        steady = steady + 1 if st["state"] == "Idle" and j and max(abs(v) for v in j) <= spm.HOME_JOINT_TOL_DEG else 0
        if steady >= 3:
            return st
        if time.time() > deadline:
            raise SystemExit(f"4분 안에 호밍 자세가 되지 않아 멈춤: {st}")
        time.sleep(1.0)


def main():
    reg = vision.load("Logitech BRIO")
    cap = open_cam()
    before = shoot(cap, "before")
    marks, _ = cmk.find_corner_marks(before, expected_px=HM_PREV, radius_px=200)
    if marks is None:
        marks, _ = cmk.find_corner_marks(before)
    if marks is None:
        raise SystemExit("네 귀퉁이 표시를 찾지 못해 멈춤")
    print("손 표시", np.round(marks, 1).tolist(), "이전과 최대 차이 px", round(float(np.abs(marks - HM_PREV).max()), 1),
          flush=True)
    link = dex.open_link(cfg)
    print("포트", link.port, flush=True)
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
        print("설정", got, flush=True)
        if got.get("$3") != "123" or got.get("$23") != "60":
            raise SystemExit(f"설정이 바뀌어 있어 멈춤: {got}")
        print("호밍 자세 확인", wait_home(link)["joints_deg"], flush=True)
        spm.execute_requested_start(link, cfg)
        draw_cfg = spm.prepare_requested_drawing(link, cfg)
        for n, (y, off, x) in enumerate(dashes()):
            c = copy.deepcopy(draw_cfg)
            c["pen"]["down_extra_mm"] = off
            # 느린 구간 시작점은 기준 평면에서 slow_zone만큼 앞이라, 덜 들어가는 깊이가 그보다 크면 지나쳐 버림:
            # 목표 접촉점 3 mm 앞에서 느려지게 맞춤 (펜업 높이 20 mm 안)
            c["pen"]["slow_zone_mm"] = min(max(float(c["pen"]["slow_zone_mm"]), 3.0 - off), 19.0)
            cmds = dex.Planner(c).plan([[(x, y), (x + DASH, y)]])
            for line, _ in cmds[1:-1]:          # 처음·끝의 종이 중심 펜업은 빼고 그 획만
                link.send_and_ack(line, cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            print(f"{n + 1}/{len(dashes())} y {y:+.0f} 깊이 {off:+.0f} x {x:+.0f}", flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    after = shoot(cap, "after")
    cap.release()
    marks_after, _ = cmk.find_corner_marks(after, expected_px=marks, radius_px=80)
    drift = None if marks_after is None else float(np.abs(marks_after - marks).max())
    print("시험 전후 손 표시 이동 px", drift, "(2 px 넘으면 결과 무효)", flush=True)
    if marks_after is None:
        marks_after = marks

    wm, hm = int(297 * PPM), int(210 * PPM)
    dst = np.array([[0, 0], [wm, 0], [wm, hm], [0, hm]], np.float32)
    gray = lambda im, m: cv2.GaussianBlur(cv2.cvtColor(cv2.warpPerspective(
        im, cv2.getPerspectiveTransform(m.astype(np.float32), dst), (wm, hm)), cv2.COLOR_BGR2GRAY), (7, 7), 0).astype(np.float32)
    d = gray(before, marks) - gray(after, marks_after)      # 각자 귀퉁이 표시로 펴서 보드 움직임을 뺌
    d = d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 31).astype(np.float32) - 128)
    ink_all = cv2.morphologyEx((d > 3.5).astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

    def flat(p):      # 로봇 종이 mm → 편 사진 px (종이 장 좌상단 원점, 아래로 +)
        return np.array([(148.5 + p[0] + SHEET_T[0]) * PPM, (105.0 - (p[1] + SHEET_T[1])) * PPM])

    rows = []
    for y, off, x in dashes():
        a, b = flat((x - 4, y + 22)), flat((x + DASH + 4, y - 8))     # 위로 미끄러진 자국까지 보는 넓은 창
        x0, x1 = int(min(a[0], b[0])), int(max(a[0], b[0]))
        y0, y1 = int(min(a[1], b[1])), int(max(a[1], b[1]))
        ink = ink_all[y0:y1, x0:x1] > 0
        area = float(ink.sum()) / PPM ** 2                          # mm²
        if area > 0.8:
            ys = np.nonzero(ink)[0] + y0
            top = flat((x, y))[1]
            span = ((top - ys.min()) / PPM, (top - ys.max()) / PPM)  # 계획 높이 기준 위(+) mm
        else:
            span = None
        rows.append({"y": y, "offset": off, "x": x, "ink_mm2": round(area, 2),
                     "ink_up_mm": None if span is None else [round(v, 1) for v in span]})
        print(f"y {y:+.0f} 깊이 {off:+.0f} x {x:+.0f}: 잉크 {area:5.2f} mm²"
              + ("" if span is None else f", 계획보다 위 {span[1]:+.1f}~{span[0]:+.1f} mm"), flush=True)
    json.dump(rows, open(os.path.join(OUT, "result.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    for y in ROWS:
        hit = sorted(r["offset"] for r in rows if r["y"] == y and r["ink_mm2"] > 0.5)
        print(f"줄 y {y:+.0f}: 잉크가 생긴 가장 얕은 깊이 {hit[0] if hit else '없음'}", flush=True)
    vis = cv2.cvtColor(np.uint8(np.clip(255 - d * 12, 0, 255)), cv2.COLOR_GRAY2BGR)
    for y, off, x in dashes():
        p, q = flat((x, y)).astype(int), flat((x + DASH, y)).astype(int)
        cv2.line(vis, tuple(p), tuple(q), (0, 170, 0), 1)
        cv2.putText(vis, f"{off:+.0f}", tuple(flat((x, y - 4)).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 220), 1)
    a, b = flat((-55, 70)), flat((55, -65))
    crop = vis[int(a[1]):int(b[1]), int(a[0]):int(b[0])]
    cv2.imwrite(os.path.join(OUT, "ladder.jpg"), cv2.resize(crop, (crop.shape[1] * 2, crop.shape[0] * 2)))
    print("저장", OUT, flush=True)


if __name__ == "__main__":
    main()
