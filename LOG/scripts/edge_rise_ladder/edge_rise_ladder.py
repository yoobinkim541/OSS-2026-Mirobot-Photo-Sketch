"""끝점 세로 줄 시험 종이: 네 줄(종이 y +40/+10/−25/−55 mm)에 5 mm 가로 짧은 선 12개씩, 선마다 값 하나를 바꿔 긋고,
끝점마다 종이 위·아래로 뻗은 세로 줄을 본다. 바꾸는 값(--knob):
  rise    (기본) 비스듬히 양 pen.edge_rise_mm = edge_land_mm = 0·1·2·3 mm (2026-11-16 20:31에 그은 시험)
  backoff 시작점 물림 pen.start_backoff_mm = 0·0.5·1.0 mm (뗄 때·댈 때 비스듬히는 설정값. 2026-11-18까지는 댈 때 0으로 그었음)
  land    내릴 때 비스듬히 pen.edge_land_mm = 0·−1·−2·−3 mm (−: 위에서 내려오며 댐, 뗄 때·물림은 설정값)
  speed   닿는 속도: 펜을 댈 때의 land·pen-down 명령만 F = 50·100·200·400 mm/min (설정 approach 200, 그 밖의 값은 설정값)
  depth   선 전체를 누르는 깊이: pen.down_extra_mm에 0·−0.5·−1.0·−1.5 mm를 더함(−는 덜 누름, 대기·접촉 자세 기록의 값 기준)
같은 값이 줄마다 다른 가로 위치에 오도록 돌려 놓고, 줄마다 긋는 방향(좌→우, 우→좌)을 번갈아 위치·방향과 값이 섞이지 않게 한다.

    python LOG/scripts/edge_rise_ladder/edge_rise_ladder.py plan [--knob backoff|land|speed|depth]      # 가상 로봇·범위 검사·관절 시뮬레이션만
    python LOG/scripts/edge_rise_ladder/edge_rise_ladder.py run --yes [--knob backoff|land|speed|depth] # 실물로 긋기 (호밍 자세에서, 빈 종이)
    python LOG/scripts/edge_rise_ladder/edge_rise_ladder.py measure [--knob backoff|land|speed|depth]   # 지금 카메라 사진으로 다시 재기

카메라는 앱 그리기 창이 공유하는 out/camera_live/full.jpg를 읽는다(카메라를 직접 열지 않음). 4K JPEG라 연한 샤프 선은
잘 안 잡히므로(20:31 시험은 겹침 0.29), 결과는 휴대폰 사진으로도 본다.
기록·사진은 %TEMP%/edgeriseladder/(rise), %TEMP%/edgeriseladder_<knob>/(그 밖)에 남긴다.
"""
import argparse
import copy
import json
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, ROOT)
from mirobot_sketch import draw_executor as dex, mirobot_sim as ms, paths, start_pose as spm, vision  # noqa: E402

OUT = os.path.join(os.environ.get("TEMP", "."), "edgeriseladder")
SHARE = os.path.join(ROOT, "out", "camera_live")
ROWS = (40.0, 10.0, -25.0, -55.0)
XS = tuple(float(v) for v in range(-48, 41, 8))      # 12곳, 선은 x → x+5 (실물 확인 범위가 종이 x −50~+55)
KNOBS = {"rise": (0.0, 1.0, 2.0, 3.0), "backoff": (0.0, 0.5, 1.0), "land": (0.0, -1.0, -2.0, -3.0),
         "speed": (50.0, 100.0, 200.0, 400.0), "depth": (0.0, -0.5, -1.0, -1.5)}
TOUCH_LABELS = (" land", " pen-down")    # speed 시험에서 속도를 바꾸는 명령 (펜을 대는 구간만)
KNOB = "rise"
RISES = KNOBS[KNOB]          # 지금 시험의 값들 (main에서 --knob에 따라 바꿈)
DASH = 5.0


def dashes():
    """[(줄 y, 비스듬히 mm, 시작 x, 끝 x)] 그리는 순서."""
    out = []
    for r, y in enumerate(ROWS):
        cells = [(RISES[(k + r) % len(RISES)], x) for k, x in enumerate(XS)]
        if r % 2:
            cells = cells[::-1]
            out += [(y, rise, x + DASH, x) for rise, x in cells]      # 우→좌
        else:
            out += [(y, rise, x, x + DASH) for rise, x in cells]      # 좌→우
    return out


def variant(cfg, value):
    c = copy.deepcopy(cfg)
    pen = c["pen"]
    if KNOB == "rise":
        pen["edge_rise_mm"], pen["edge_land_mm"], pen["edge_rise_low_mm"] = value, value, 0.0
    elif KNOB == "land":
        pen["edge_land_mm"], pen["edge_rise_low_mm"] = value, 0.0
    elif KNOB == "backoff":
        pen["start_backoff_mm"], pen["edge_rise_low_mm"] = value, 0.0
    elif KNOB == "depth":
        pen["down_extra_mm"], pen["edge_rise_low_mm"] = float(pen.get("down_extra_mm", 0.0)) + value, 0.0
    else:                    # speed: 펜 값은 설정 그대로, 속도는 commands()에서 대는 명령만 바꿈
        pen["edge_rise_low_mm"] = 0.0
    return c


def touch_feed(plan, value):
    """speed 시험: 펜을 대는 명령(land·pen-down)의 F만 value로."""
    if KNOB != "speed":
        return plan
    out = []
    for gcode, label in plan:
        if label.endswith(TOUCH_LABELS):
            head, f = gcode.rsplit(" F", 1)
            gcode = f"{head} F{value:g}"
        out.append((gcode, label))
    return out


def commands(draw_cfg):
    """선마다 그 비스듬히 값으로 계획한 명령을 이어 붙임: 처음 종이 중심 펜업 한 번, 선마다 그 선 명령만, 끝에 종이 중심 펜업."""
    out = []
    plans = [touch_feed(dex.Planner(variant(draw_cfg, rise)).plan([[(x0, y), (x1, y)]]), rise) for y, rise, x0, x1 in dashes()]
    out.append(plans[0][0])
    for p in plans:
        out += p[1:-1]
    out.append(plans[-1][-1])
    return out


def check(draw_cfg):
    """범위 밖 선 (선마다 그 비스듬히 값으로 옮긴 점까지)."""
    bad = [d for d in dashes() if dex.check_limits([[(d[2], d[0]), (d[3], d[0])]], variant(draw_cfg, d[1]))]
    print("범위 밖 선", len(bad), bad[:3])
    return bad


def share_frame(after_time=None, timeout=60):
    """공유 사진(full.jpg) 한 장 (시각, 파일 바이트). after_time을 주면 그 뒤에 쓴 사진이 올 때까지 기다림.
    바이트를 그대로 저장해 다시 압축하지 않는다."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            st = json.load(open(os.path.join(SHARE, "status.json"), encoding="utf-8"))
        except (OSError, ValueError):
            st = None
        if st and (after_time is None or st["time"] > after_time):
            try:
                data = open(os.path.join(SHARE, "full.jpg"), "rb").read()
            except OSError:
                data = b""
            if cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR) is not None:
                return st["time"], data
        time.sleep(1.0)
    raise SystemExit("카메라 공유 사진이 갱신되지 않습니다. 앱 그리기 창에서 카메라를 켜 두세요.")


def save_frames(prefix, count, after_time=None, timeout=90):
    """공유 사진 count장을 OUT/prefix_N.jpg로 저장. 마지막 사진의 시각을 돌려줌."""
    t = after_time
    for n in range(count):
        t, data = share_frame(t, timeout)
        open(os.path.join(OUT, f"{prefix}_{n + 1}.jpg"), "wb").write(data)
    return t


# ---- 재기 (검증 안 됨): 4K 공유 사진에서 종이는 약 5 px/mm라 샤프 선이 배경보다 4/255쯤만 어둡다(2026-11-16 20:31).
# 선 확인(LineCheck)처럼 계획 자리만 보면 등록 오차(줄 안에서 약 3 mm 기울어짐)로 놓치므로, 선마다 실제 잉크 자리를 찾아 잰다.
# 20:31 사진 1장으로는 "찾음" 44/48이었지만 문턱(LINE_MIN 1.5)이 낮아 잡음을 선으로 본 것이 섞였을 수 있다. 끝점 줄은 방향 경향
# (아래 줄이 줄고 위 줄이 늘어남)만 휴대폰과 같고 길이는 절반쯤, 시작점 아래 줄은 비스듬히 값에 따른 경향도 나오지 않았다.
# 그래서 세로 줄 길이는 휴대폰 사진으로 정하고, 이 값은 참고만 한다. 사진 여러 장 평균으로 나아지는지는 아직 모름.
FLAT_PPM = 10.0
FLAT_BOX = (-60.0, 55.0, -75.0, 55.0)          # 종이 x0, x1, y0, y1 (mm)
BEFORE_FRAMES, AFTER_FRAMES = 3, 6
INK_SIGMA = 3.0                                 # 잡음의 이 배수보다 어두워진 곳을 잉크로 봄
LINE_MIN = 1.5                                  # 가로선 세기(잡음 배수 평균)가 이보다 작으면 못 찾은 것으로 봄


def _flat_mean(names, reg):
    x0, x1, y0, y1 = FLAT_BOX
    w, h = int((x1 - x0) * FLAT_PPM), int((y1 - y0) * FLAT_PPM)
    acc = None
    for name in names:
        img = cv2.imdecode(np.fromfile(os.path.join(OUT, name), np.uint8), cv2.IMREAD_COLOR)
        m = np.array([[FLAT_PPM, 0, -x0 * FLAT_PPM], [0, -FLAT_PPM, y1 * FLAT_PPM], [0, 0, 1]]) @ np.linalg.inv(
            reg.matrix((img.shape[1], img.shape[0])))
        g = cv2.cvtColor(cv2.warpPerspective(img, m, (w, h), flags=cv2.INTER_CUBIC), cv2.COLOR_BGR2GRAY).astype(np.float32)
        acc = g if acc is None else acc + g
    return acc / len(names)


def _to_px(x, y):
    x0, _x1, _y0, y1 = FLAT_BOX
    return int(round((x - x0) * FLAT_PPM)), int(round((y1 - y) * FLAT_PPM))


def _leg(mask, c0, c1, row, step):
    """row에서 step(+1 종이 아래, −1 위) 방향으로 c0~c1 띠에 잉크가 이어지는 길이(mm). 0.5 mm 빈틈까지 이어 봄."""
    gap = n = last = 0
    r = row
    while 0 <= r + step < mask.shape[0] and n < 12 * FLAT_PPM:
        r += step
        n += 1
        if mask[r, c0:c1].any():
            gap, last = 0, n
        else:
            gap += 1
            if gap > 0.5 * FLAT_PPM:
                break
    return last / FLAT_PPM


def measure(before, after, tag):
    reg = vision.load("Logitech BRIO")
    d = _flat_mean(before, reg) - _flat_mean(after, reg)         # 어두워진 만큼 +
    d = cv2.GaussianBlur(d, (0, 0), 1.0)
    d -= cv2.GaussianBlur(d, (0, 0), 15)
    z = d / (1.4826 * float(np.median(np.abs(d - np.median(d)))))
    mask = cv2.morphologyEx((z > INK_SIGMA).astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    rows = []
    for y, val, sx, ex in dashes():
        lo, hi = min(sx, ex), max(sx, ex)
        best = None
        for dy in np.arange(-6.0, 6.01, 0.2):      # 선마다 등록 오차를 따라감 (가로는 이웃 선과 섞이지 않게 ±2.5 mm)
            for dx in np.arange(-2.5, 2.51, 0.2):
                c0, r = _to_px(lo + dx, y + dy)
                c1, _ = _to_px(hi + dx, y + dy)
                s = float(np.clip(z[r - 2:r + 3, c0:c1], 0, 8).max(0).mean())
                if best is None or s > best[0]:
                    best = (s, float(dx), float(dy))
        s, dx, dy = best
        c0, r = _to_px(lo + dx, y + dy)
        c1, _ = _to_px(hi + dx, y + dy)
        r = r - 4 + int(np.argmax(np.clip(z[r - 4:r + 5, c0:c1], 0, None).sum(1)))   # 가로선 행을 다시 잡음
        band = int(1.2 * FLAT_PPM)
        sides = {"left": (c0 - band // 2, c0 + band), "right": (c1 - band, c1 + band // 2)}
        legs = {k: (_leg(mask, a, b, r, -1), _leg(mask, a, b, r, +1)) for k, (a, b) in sides.items()}
        start, end = ("left", "right") if sx < ex else ("right", "left")
        rows.append({"y": y, "value": val, "x0": sx, "x1": ex, "line": round(s, 2), "found": s >= LINE_MIN,
                     "shift_mm": [round(dx, 1), round(dy, 1)],
                     "start_up": legs[start][0], "start_down": legs[start][1],
                     "end_up": legs[end][0], "end_down": legs[end][1]})
    json.dump({"knob": KNOB, "before": before, "after": after, "rows": rows},
              open(os.path.join(OUT, f"result_{tag}.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"줄 y | {KNOB} | 찾음 | 시작 아래 중앙값 | 끝 아래 / 위 중앙값 (mm)")
    med = lambda rs, k: float(np.median([r[k] for r in rs])) if rs else float("nan")  # noqa: E731
    for y in ROWS + (None,):
        for v in RISES:
            rs = [r for r in rows if (y is None or r["y"] == y) and r["value"] == v]
            ok = [r for r in rs if r["found"]]
            label = "전체" if y is None else f"{y:+5.0f}"
            print(f"{label} | {v:g} mm | {len(ok)}/{len(rs)} | {med(ok, 'start_down'):.1f} | "
                  f"{med(ok, 'end_down'):.1f} / {med(ok, 'end_up'):.1f}")
    # 그림: 잉크(어두워진 곳) + 찾은 선 자리(초록) + 값
    img = cv2.cvtColor(np.clip(255 - z * 25, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for r in rows:
        lo, hi = min(r["x0"], r["x1"]), max(r["x0"], r["x1"])
        p = _to_px(lo + r["shift_mm"][0], r["y"] + r["shift_mm"][1])
        q = _to_px(hi + r["shift_mm"][0], r["y"] + r["shift_mm"][1])
        cv2.line(img, (p[0], p[1] - 25), (q[0], q[1] - 25), (0, 170, 0) if r["found"] else (0, 0, 220), 1)
        cv2.putText(img, f"{r['value']:g}", (p[0], p[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 220), 1)
    cv2.imencode(".png", img)[1].tofile(os.path.join(OUT, f"ladder_{tag}.png"))
    print("저장", OUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("plan", "run", "measure"))
    ap.add_argument("--yes", action="store_true", help="run: 실물로 움직임에 동의")
    ap.add_argument("--knob", choices=tuple(KNOBS), default="rise")
    args = ap.parse_args()
    global KNOB, RISES, OUT
    KNOB, RISES = args.knob, KNOBS[args.knob]
    if KNOB != "rise":
        OUT = OUT + "_" + KNOB
    os.makedirs(OUT, exist_ok=True)
    cfg = dex.load_config(paths.config_path())
    if args.mode == "measure":   # 실행 때 저장한 before_N.jpg와 지금 사진으로 다시 잼
        before = sorted(n for n in os.listdir(OUT) if n.startswith("before_"))
        if not before:
            raise SystemExit(f"{OUT}에 실행 전 사진(before_N.jpg)이 없습니다")
        save_frames("after_m", AFTER_FRAMES)
        measure(before, [f"after_m_{n + 1}.jpg" for n in range(AFTER_FRAMES)], "measure")
        return
    draw_cfg = spm.drawing_config_from_recorded_contact(cfg)
    cmds = commands(draw_cfg)
    print(f"선 {len(dashes())}개, 명령 {len(cmds)}개")
    if check(draw_cfg):
        raise SystemExit("범위 밖 선이 있어 멈춤")
    # 관절 시뮬레이션: 비스듬히 값마다 그 값의 선만 모아 앱과 같은 방식으로
    rot, q0 = ms.simulation_pose(cfg)
    for rise in RISES:
        pts = [[(x0, y), (x1, y)] for y, r, x0, x1 in dashes() if r == rise]
        res = ms.simulate(ms.plan_targets(pts, variant(cfg, rise)), r_goal=rot, q0=q0)
        print(f"시뮬레이션 {KNOB} {rise:g} mm: {ms.verdict(res)}")
        if not ms.verdict(res).startswith(("PASS", "WARN")):
            raise SystemExit("시뮬레이션 실패로 멈춤")
    if args.mode == "plan":
        from mirobot_sketch.virtual_robot import VirtualMirobotLink
        link = VirtualMirobotLink(draw_cfg, speed=500, homing_s=0.0)
        link.wait_for_homing(1, lambda *_: None)
        r = dex.execute(link, cmds, draw_cfg, progress=lambda *_: None)
        print("가상 실행:", r["result"], r.get("error", ""))
        t = dex.estimate_time([[(x0, y), (x1, y)] for y, _r, x0, x1 in dashes()], variant(draw_cfg, RISES[-1]))
        print(f"예상 시간 약 {t['total_s'] / 60:.1f}분 ({KNOB} {RISES[-1]:g} mm 기준)")
        return
    if not args.yes:
        raise SystemExit("실물 실행은 --yes가 필요합니다")
    t0 = save_frames("before", BEFORE_FRAMES)
    link = dex.open_link(cfg)
    print("포트", link.port, flush=True)
    try:
        print("컨트롤러 설정", dex.require_controller_settings(link, cfg), flush=True)
        st = dex.parse_status(link.query_status()[2])
        j = st.get("joints_deg")
        if st["state"] != "Idle" or j is None or max(abs(v) for v in j) > spm.HOME_JOINT_TOL_DEG:
            raise SystemExit(f"호밍 자세(Idle, 관절 0)가 아니어서 멈춤: {st['state']} {j}")
        spm.execute_requested_start(link, cfg)
        draw_cfg = spm.prepare_requested_drawing(link, cfg)
        cmds = commands(draw_cfg)
        r = dex.execute(link, cmds, draw_cfg, progress=lambda m: print(m, flush=True))
        print("실행:", r["result"], r.get("error", ""), flush=True)
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)
    save_frames("after", AFTER_FRAMES, t0)
    measure([f"before_{n + 1}.jpg" for n in range(BEFORE_FRAMES)], [f"after_{n + 1}.jpg" for n in range(AFTER_FRAMES)], "run")


if __name__ == "__main__":
    main()
