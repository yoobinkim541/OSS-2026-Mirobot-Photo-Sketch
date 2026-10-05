"""
시작 자세: 펜 끝을 위로 올린 대기 자세 (시작 자세 설계 T2)
==========================================================
호밍 자세(모든 관절 0)는 플랜지가 아래를 향해 펜이 수직입니다. 그리려면 손목(J5)을 돌려 펜이
벽(+X) 쪽을 보게 해야 하는데,

  - 펜 축이 팔뚝 축(J4)과 나란하면 손목 특이점이라(T1, LOG/2026-10-05-start-pose-t1.md), 펜을 수평으로 쓰려면
    종이 중심을 낮게 두어 그리는 동안 팔뚝이 아래로 기울어 있게 하고(LOG/2026-10-05-start-pose-level.md),
    필요하면 펜 끝을 pen_up_deg만큼 위로 올리며,
  - 손목을 벽 앞에서 돌리면 펜 끝이 펜 길이만큼 벽 쪽으로 쓸려 가므로, 베이스(J1)를 옆으로 돌린 채
    손목을 돌린 뒤 다시 벽 쪽으로 돌아옵니다. J1만 돌 때 펜 끝은 베이스 축을 중심으로 원을 그리므로
    벽 쪽으로 더 다가가지 않습니다.

순서 (모두 관절 이동 M21 — 특이점을 지나도 컨트롤러가 손목을 휘두르지 않음):
  1. J1 -> side_j1_deg (펜은 아래를 향한 채 옆으로)
  2. 옆을 본 채 J2~J6 -> 대기 자세 값 (팔을 들고 펜 끝을 위로)
  3. J1 -> 대기 자세 값(≈0) (벽 쪽으로 돌아옴)
대기 자세: 플랜지 (종이면 X - 여유 - 펜 길이·cos(올림), 0, standby_z_mm), 펜 끝은 종이에서 clearance_mm 앞.
컨트롤러 보고 기준(실물 2026-10-05): 펜 끝 35° 위 = B -125°, 수평 = B -90°.

계산은 시뮬레이터(URDF 기구학) 기준입니다. 옆으로 도는 쪽에 물건이 없는지는 사람이 확인해야 합니다.
"""

import queue

import numpy as np

from . import mirobot_sim as ms
from . import pen_pose as pp

MOUNTS = ("right_angle", "along_flange")
DEFAULTS = {
    "pen_mount": "right_angle",  # right_angle: 펜이 플랜지 축과 직각(호밍 자세에서 팔뚝·펜 모두 수평으로 벽을 향함,
                                 #   손목을 돌리지 않음). along_flange: 펜이 플랜지 축과 나란함(아래 순서로 손목을 돌림)
    "pen_up_deg": 0.0,           # 펜 끝을 수평에서 위로 올리는 각도. 0 = 펜이 종이에 수직(수평). 종이 중심이 낮으면
                                 # (플랜지 Z ≤ 약 145 mm) 팔뚝이 아래로 기울어 수평 펜도 특이점에서 20° 이상 떨어짐
    "pen_len_max_mm": 60.0,      # 플랜지 면 -> 펜 끝 (펜 축 방향). 안전 계산은 이 최댓값 기준
    "paper_x_mm": 260.0,         # 로봇 중심(J1 축) -> 종이 면 (로봇 X)
    "clearance_mm": 15.0,        # 대기 자세와 펜 올리는 동안 펜 끝과 종이 면 사이 최소 거리
    "standby_z_mm": 130.0,       # 대기 자세 플랜지 높이 (= 화살표로 맞출 때 고정되는 높이, 종이 중심 높이)
    "side_j1_deg": -90.0,        # 손목을 돌리는 동안 베이스 방향 (-: 로봇 오른쪽)
    "joint_feed": 600,           # 관절 이동 속도 F (실물 단위 미확인: 느린 값으로 시작)
    "min_tip_z_mm": 30.0,        # 펜 끝이 책상 위로 이만큼은 떠 있어야 함
    "min_joint_margin_deg": 5.0,
    "min_singularity_deg": 15.0,
    "ready_tol_deg": 0.5,        # 조그는 펜 자세를 바꾸지 않으므로: 대기 자세와 A·B·C 차이 허용
    "ready_pose": None,          # 펜 끝을 종이 가운데에 맞춘 자세 (save_ready_pose가 저장)
    "jog_steps_mm": [0.5, 2.0, 10.0],   # 키보드 조그 한 번 이동량 (1/2/3 키로 고름)
    "jog_default_step": 1,               # 처음 이동량: jog_steps_mm의 번호 (0부터)
    "jog_max_forward_mm": 60.0,  # 대기 자세에서 벽 쪽(+X)으로 갈 수 있는 최대
    "jog_max_back_mm": 30.0,     # 대기 자세에서 뒤(-X)로 갈 수 있는 최대
    "jog_max_y_mm": 60.0,        # 대기 자세에서 좌우(Y)로 갈 수 있는 최대
    "jog_max_z_mm": 60.0,        # 대기 자세에서 위아래(Z)로 갈 수 있는 최대
}
# (X, Y, Z 방향). w 벽 쪽 / x 뒤로 / a 왼쪽(종이 왼쪽 = 로봇 +Y) / d 오른쪽 / r 위 / f 아래
JOG_KEYS = {"w": (1, 0, 0), "x": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "r": (0, 0, 1), "f": (0, 0, -1)}
AT_POSE_TOL_MM = 1.0       # 저장된 시작 자세에 이미 있다고 볼 거리
SAMPLES = 60    # 관절 이동 한 번을 나눠 검사하는 수
GAP_TOL_MM = 0.1   # IK 위치 허용 오차(0.05 mm) 때문에 대기 자세 끝점이 여유보다 조금 모자라게 계산되는 것을 허용
_UP_SEED = np.radians([0, -10, 20, 0, -130, 0])


class StartPoseError(Exception):
    """대기 자세로 안전하게 갈 수 없음 (설정·배치를 바꿔야 함)."""


def settings(cfg):
    """설정의 start_pose 블록 + 기본값."""
    return {**DEFAULTS, **cfg.get("start_pose", {})}


def joint_line(q_deg, feed):
    j = [f"{v:.3f}" for v in q_deg]
    return f"M21 G90 G01 X{j[0]} Y{j[1]} Z{j[2]} A{j[3]} B{j[4]} C{j[5]} F{feed:.0f}"


def _tip(q, pen_len):
    flange, _ = ms.fk(q)
    return flange[:3, 3] + pen_len * flange[:3, 2]


def _check_segments(poses, s):
    """관절 이동을 잘게 나눠 관절 여유·펜 끝과 종이 거리·펜 끝 높이를 검사. 반환: (최소 여유°, 최소 거리 mm)."""
    lo, hi = ms.JOINT_LIMITS_RAD[:, 0], ms.JOINT_LIMITS_RAD[:, 1]
    worst_margin, worst_gap = np.inf, np.inf
    for k, (a, b) in enumerate(zip(poses, poses[1:]), start=1):
        for t in np.linspace(0.0, 1.0, SAMPLES + 1):
            q = a + (b - a) * t
            margin = float(np.degrees(np.minimum(q - lo, hi - q)).min())
            tip = _tip(q, s["pen_len_max_mm"])
            gap = s["paper_x_mm"] - float(tip[0])
            if margin < s["min_joint_margin_deg"]:
                raise StartPoseError(f"{k}번째 이동에서 관절 한계 여유가 {margin:.1f}°입니다 "
                                     f"(필요 {s['min_joint_margin_deg']:g}°). side_j1_deg·standby_z_mm를 바꾸세요.")
            if gap < s["clearance_mm"] - GAP_TOL_MM:
                raise StartPoseError(f"{k}번째 이동에서 펜 끝이 종이 면에서 {gap:.1f} mm까지 다가갑니다 "
                                     f"(필요 {s['clearance_mm']:g} mm). 로봇을 벽에서 더 떨어뜨리거나 펜을 짧게 물리고 "
                                     f"paper_x_mm·pen_len_max_mm를 고치세요.")
            if tip[2] < s["min_tip_z_mm"]:
                raise StartPoseError(f"{k}번째 이동에서 펜 끝 높이가 {tip[2]:.1f} mm입니다 "
                                     f"(필요 {s['min_tip_z_mm']:g} mm 이상). standby_z_mm를 높이세요.")
            worst_margin, worst_gap = min(worst_margin, margin), min(worst_gap, gap)
    return worst_margin, worst_gap


def _mount(s):
    if s["pen_mount"] not in MOUNTS:
        raise StartPoseError(f"pen_mount는 {' / '.join(MOUNTS)} 중 하나여야 합니다 (지금: {s['pen_mount']}).")
    return s["pen_mount"]


def pen_frame(s):
    """그리는 동안 고정할 플랜지 자세와 IK 시작 관절각."""
    if _mount(s) == "right_angle":
        return ms.HOME_ROT, np.zeros(6)
    return pp.pen_rotation(-s["pen_up_deg"]), pp.LEVEL_SEED


def plan_raise(cfg):
    """호밍 자세 -> 펜 끝을 올린 대기 자세. 반환 dict: commands [(gcode, 설명)], joints_deg(각 이동 끝),
    standby_tcp_mm, min_joint_margin_deg, min_tip_clearance_mm, standby_singularity_deg.
    직각 홀더(right_angle)는 호밍 자세가 곧 대기 자세라 보낼 명령이 없음.
    안전 조건을 못 맞추면 StartPoseError (명령을 보내기 전에)."""
    s = settings(cfg)
    if _mount(s) == "right_angle":
        return {"commands": [], "joints_deg": [], "standby_tcp_mm": ms.HOME_TCP_MM.round(3).tolist(),
                "min_joint_margin_deg": None, "min_tip_clearance_mm": None,
                "standby_singularity_deg": pp.singularity_deg(np.zeros(6))}
    up = np.radians(s["pen_up_deg"])
    x = s["paper_x_mm"] - s["clearance_mm"] - s["pen_len_max_mm"] * np.cos(up)
    target = np.array([x, 0.0, s["standby_z_mm"]])
    q_up, err, ok = ms.ik(target, _UP_SEED, r_goal=pp.pen_rotation(-s["pen_up_deg"]), iters=400)
    if not ok:
        raise StartPoseError(f"대기 자세 (X {x:.1f}, Z {target[2]:.1f} mm)에 도달할 수 없습니다 (오차 {err:.2f} mm).")
    q_up = pp._wrap(q_up)
    sing = pp.singularity_deg(q_up)
    if sing < s["min_singularity_deg"]:
        raise StartPoseError(f"대기 자세가 손목 특이점에서 {sing:.1f}°밖에 떨어져 있지 않습니다 "
                             f"(필요 {s['min_singularity_deg']:g}°). pen_up_deg를 바꾸세요.")
    side = np.radians(s["side_j1_deg"])
    poses = [np.zeros(6), np.array([side, 0, 0, 0, 0, 0]), np.array([side, *q_up[1:]]), q_up]
    margin, gap = _check_segments(poses, s)
    labels = ["base to side (pen down)", "raise pen tip (facing side)", "turn back to the wall"]
    deg = [np.degrees(q) for q in poses[1:]]
    return {
        "commands": [(joint_line(q, s["joint_feed"]), label) for q, label in zip(deg, labels)],
        "joints_deg": [q.round(3).tolist() for q in deg],
        "standby_tcp_mm": ms.fk(q_up)[0][:3, 3].round(3).tolist(),
        "min_joint_margin_deg": margin,
        "min_tip_clearance_mm": gap,
        "standby_singularity_deg": sing,
    }


MOVE_BACK_SCAN_MM = (10.0, 100.0)   # 그리기 범위가 안 닿을 때 로봇을 얼마나 뒤로 빼야 하는지 찾는 간격·최대


def check_ready(cfg, status, reference):
    """펜 끝을 종이 가운데에 맞춘 자세(status)를 저장해도 되는지. reference: 조그를 시작한 자세(호밍 직후 또는
    펜을 올린 직후). 조그는 X·Y·Z만 움직이므로 펜 자세(A·B·C)가 그대로인지, 그 자리에서 그리기 범위(limits)
    전체가 닿고 특이점에서 떨어져 있는지 시뮬레이터로 검사.
    반환: ok, problems, min_margin_deg, min_singularity_deg, move_back_mm."""
    s = settings(cfg)
    problems = []
    x, y, z = status["tcp"]
    turned = max(abs(a - b) for a, b in zip(status["abc_deg"], reference["abc_deg"]))
    if turned > s["ready_tol_deg"]:
        problems.append(f"펜 자세(A·B·C)가 {turned:.2f}° 바뀌었습니다 (허용 {s['ready_tol_deg']:g}°). "
                        "호밍(또는 펜 올리기)부터 다시 하세요.")

    rot, seed = pen_frame(s)

    def fits(cx):
        r = pp.study(cfg, cx, z, 0.0, center_y=y, rot=rot, seed=seed)
        good = (r["unreachable"] == 0 and r["min_margin_deg"] >= s["min_joint_margin_deg"]
                and r["min_singularity_deg"] >= s["min_singularity_deg"])
        return good, r

    good, r = fits(x)
    move_back = 0.0
    if not good:
        step, limit = MOVE_BACK_SCAN_MM
        move_back = None
        for d in np.arange(step, limit + 1e-9, step):
            if fits(x + d)[0]:
                move_back = float(d)
                break
        need = (f"로봇을 벽에서 약 {move_back:g} mm 뒤로 옮긴 뒤 다시 맞추세요" if move_back is not None
                else f"로봇을 {limit:g} mm 뒤로 옮겨도 부족합니다. 펜 길이·종이 높이를 확인하세요")
        problems.append(f"이 자리에서는 그리기 범위 끝까지 닿지 않거나 여유가 부족합니다 "
                        f"(관절 여유 {r['min_margin_deg']:.1f}°, 특이점까지 {r['min_singularity_deg']:.1f}°, "
                        f"닿지 않는 점 {r['unreachable']}개). {need}.")
    return {"ok": not problems, "problems": problems, "min_margin_deg": r["min_margin_deg"],
            "min_singularity_deg": r["min_singularity_deg"], "move_back_mm": move_back}


def save_ready_pose(config_path, status, now=None):
    """설정 파일의 start_pose.ready_pose만 바꿔 저장 (다른 값·메모는 그대로). 반환: 저장한 dict."""
    import json
    from datetime import datetime

    from .calibration import save_config

    with open(config_path, encoding="utf-8") as f:
        raw = json.load(f)
    ready = {"tcp_mm": [float(v) for v in status["tcp"]], "abc_deg": [float(v) for v in status["abc_deg"]],
             "joints_deg": [float(v) for v in status["joints_deg"]] if status.get("joints_deg") else None,
             "saved_local": (now or datetime.now()).isoformat(timespec="seconds")}
    raw.setdefault("start_pose", {})["ready_pose"] = ready
    save_config(config_path, raw)
    return ready


def in_jog_box(s, target, reference):
    """조그 목표가 기준 자세(호밍 직후 등) 둘레의 허용 상자 안인지."""
    (x, y, z), (rx, ry, rz) = target, reference
    return (-s["jog_max_back_mm"] <= x - rx <= s["jog_max_forward_mm"] and abs(y - ry) <= s["jog_max_y_mm"]
            and abs(z - rz) <= s["jog_max_z_mm"])


def jog_box_text(s):
    return (f"기준 자세에서 X -{s['jog_max_back_mm']:g}~+{s['jog_max_forward_mm']:g}, "
            f"Y ±{s['jog_max_y_mm']:g}, Z ±{s['jog_max_z_mm']:g} mm")


def jog_line(xyz, abc, feed):
    (x, y, z), (a, b, c) = xyz, abc
    return f"M20 G90 G01 X{x:.3f} Y{y:.3f} Z{z:.3f} A{a:.3f} B{b:.3f} C{c:.3f} F{feed:.0f}"


class StartPoseSetup:
    """앱(그리기 창)의 시작 자세 맞추기 — 작업 스레드에서 run(), 화면 스레드에서 jog()/accept()/cancel().
    직각 홀더(right_angle) 전용: 호밍 자세가 곧 그리는 자세.

    1. 호밍 자세인지 확인. 저장된 시작 자세가 있으면 펜을 뒤로 뺀 채 그 높이·좌우로 옮긴 뒤 앞으로 다가가 자동으로
       그 자리로 (이미 그 자리면 움직이지 않음, 호밍 자세도 저장 자세도 아니면 거부).
    2. 조그: 누르고 있는 동안 화면이 jog(dx, dy, dz)를 계속 넣음. 한 번에 하나만 받고 바쁘면 버려서 손을 떼면 곧 멈춤.
       펜 자세(A·B·C)는 기준 자세 값으로 고정, 허용 상자(jog_box_text) 밖으로는 보내지 않음.
    3. accept(): 검사(check_ready)를 통과하면 설정 파일에 저장하고 그 TCP를 반환. 못 맞추면 이유를 보내고 계속 기다림.
    events("start_pose", state=moving|jog|saved, tcp, message, problems)."""

    def __init__(self, link, cfg, config_path, events):
        self.link, self.cfg, self.config_path, self.events = link, cfg, config_path, events
        self._jogs = queue.Queue(maxsize=1)
        self._decision = queue.Queue()

    def jog(self, dx, dy, dz):
        try:
            self._jogs.put_nowait((float(dx), float(dy), float(dz)))
        except queue.Full:
            pass

    def accept(self):
        self._decision.put("accept")

    def cancel(self):
        self._decision.put("cancel")

    def _status(self):
        from .draw_executor import parse_status
        return parse_status(self.link.query_status()[2])

    def _move(self, xyz, abc, feed):
        self.link.send_and_ack(jog_line(xyz, abc, feed), self.cfg["ack_timeout_s"])
        self.link.wait_idle(self.cfg["idle_timeout_s"])

    def _emit(self, state, **data):
        self.events("start_pose", state=state, **data)

    def run(self, should_stop):
        s = settings(self.cfg)
        if _mount(s) != "right_angle":
            raise StartPoseError("앱 안의 시작 자세 맞추기는 직각 홀더(pen_mount right_angle)만 지원합니다. "
                                 "손목을 돌리는 방식은 python -m mirobot_sketch.start_pose --execute를 쓰세요.")
        st = self._status()
        j = st["joints_deg"]
        homed = j is not None and max(abs(v) for v in j) <= HOME_JOINT_TOL_DEG
        ready = s["ready_pose"]
        at_ready = bool(ready) and float(np.linalg.norm(np.subtract(st["tcp"], ready["tcp_mm"]))) <= AT_POSE_TOL_MM
        if homed:
            reference = st
        elif at_ready:
            reference = {"tcp": tuple(float(v) for v in ms.HOME_TCP_MM), "abc_deg": tuple(ready["abc_deg"])}
        else:
            raise StartPoseError(f"호밍 자세도, 저장된 시작 자세도 아닙니다 (TCP {st['tcp']}, 관절 {j}). "
                                 "가운데 버튼 2초로 호밍한 뒤 다시 연결하세요.")
        abc = tuple(reference["abc_deg"])
        feeds = self.cfg["feeds_mm_per_min"]
        if ready and homed:
            rx, ry, rz = ready["tcp_mm"]
            back = min(st["tcp"][0], rx) - float(self.cfg["pen"]["up_clearance_mm"])
            self._emit("moving", tcp=tuple(st["tcp"]), message="저장된 시작 자세로 옮기는 중입니다")
            for xyz, feed in (((back, st["tcp"][1], st["tcp"][2]), feeds["travel"]),
                              ((back, ry, rz), feeds["travel"]), ((rx, ry, rz), feeds["approach"])):
                if should_stop():
                    raise StartPoseError("취소했습니다.")
                self._move(xyz, abc, feed)
            st = self._status()
        target = tuple(st["tcp"])
        self._emit("jog", tcp=target, message="저장된 시작 자세입니다. 필요하면 조그로 고치세요." if ready else
                   "펜 끝을 종이 가운데에 살짝 대세요.")
        while True:
            if should_stop():
                raise StartPoseError("취소했습니다.")
            try:
                decision = self._decision.get_nowait()
            except queue.Empty:
                decision = None
            if decision == "cancel":
                raise StartPoseError("취소했습니다.")
            if decision == "accept":
                st = self._status()
                check = check_ready(self.cfg, st, reference)
                if not check["ok"]:
                    self._emit("jog", tcp=tuple(st["tcp"]), message="기억하지 않았습니다.", problems=check["problems"])
                    continue
                saved = save_ready_pose(self.config_path, st)
                self.cfg.setdefault("start_pose", {})["ready_pose"] = saved
                self._emit("saved", tcp=tuple(st["tcp"]), message=f"시작 자세를 기억했습니다 (관절 여유 "
                           f"{check['min_margin_deg']:.1f}°, 특이점까지 {check['min_singularity_deg']:.1f}°)")
                return tuple(st["tcp"])
            try:
                dx, dy, dz = self._jogs.get(timeout=0.05)
            except queue.Empty:
                continue
            nxt = (target[0] + dx, target[1] + dy, target[2] + dz)
            if not in_jog_box(s, nxt, reference["tcp"]):
                self._emit("jog", tcp=target, message=f"조그 범위를 넘어 움직이지 않았습니다 ({jog_box_text(s)})")
                continue
            self._move(nxt, abc, feeds["approach"])
            target = nxt
            self._emit("jog", tcp=tuple(self._status()["tcp"]), message="")


def jog_session(link, cfg, reference, config_path, input_fn=input, output_fn=print):
    """펜을 올린 뒤(직각 홀더는 호밍 직후) 키보드로 X·Y·Z를 움직여 펜 끝을 종이 가운데에 대고 저장. 자세(A·B·C)는
    reference 값으로 고정해 보냄. 키: w 벽 쪽 / x 뒤로 / a 왼쪽 / d 오른쪽 / r 위 / f 아래 (여러 글자면 차례로),
    1·2·3 이동량, Enter 현재 좌표, s 검사 후 저장, q 저장하지 않고 끝. 반환: result(saved|quit), ready."""
    from .draw_executor import ControllerError, parse_status

    s = settings(cfg)
    (rx, ry, rz), (ra, rb, rc) = reference["tcp"], reference["abc_deg"]
    tx, ty, tz = rx, ry, rz
    steps = s["jog_steps_mm"]
    step = steps[s["jog_default_step"]]
    feed = cfg["feeds_mm_per_min"]["approach"]
    output_fn("키보드로 펜 끝을 종이 가운데에 살짝 대세요. w 벽 쪽 / x 뒤로 / a 왼쪽 / d 오른쪽 / r 위 / f 아래 (예: www), "
              f"1·2·3 이동량 {'/'.join(f'{v:g}' for v in steps)} mm (지금 {step:g} mm). 펜 자세는 고정됩니다.")
    while True:
        key = input_fn(f"[{step:g} mm] w/x/a/d/r/f 이동, 1/2/3 이동량, Enter 좌표, s 저장, q 끝 > ").strip().lower()
        if key == "q":
            return {"result": "quit", "ready": None}
        if key in ("1", "2", "3"):
            step = steps[int(key) - 1]
            output_fn(f"  이동량 {step:g} mm")
            continue
        if key and all(ch in JOG_KEYS for ch in key):
            for ch in key:
                ux, uy, uz = JOG_KEYS[ch]
                nx, ny, nz = tx + ux * step, ty + uy * step, tz + uz * step
                if not in_jog_box(s, (nx, ny, nz), (rx, ry, rz)):
                    output_fn(f"  조그 범위를 넘어 움직이지 않음 ({jog_box_text(s)})")
                    break
                line = jog_line((nx, ny, nz), (ra, rb, rc), feed)
                try:
                    link.send_and_ack(line, cfg["ack_timeout_s"])
                    link.wait_idle(cfg["idle_timeout_s"])
                except ControllerError as e:
                    output_fn(f"  컨트롤러 오류로 멈춤: {e}")
                    return {"result": "stopped_on_error", "ready": None, "error": str(e)}
                tx, ty, tz = nx, ny, nz
            output_fn(f"  목표 X {tx:.1f}, Y {ty:.1f}, Z {tz:.1f} mm (대기 자세에서 X {tx - rx:+.1f}, "
                      f"Y {ty - ry:+.1f}, Z {tz - rz:+.1f})")
            continue
        if key not in ("", "s"):
            output_fn(f"  모르는 키: {key}")
            continue
        st = parse_status(link.query_status()[2])
        (x, y, z), abc = st["tcp"], st["abc_deg"]
        output_fn(f"  TCP ({x:.1f}, {y:.1f}, {z:.1f})  대기 자세에서 X {x - rx:+.1f}, Y {y - ry:+.1f}, Z {z - rz:+.2f} mm, "
                  f"A·B·C 변화 {max(abs(a - b) for a, b in zip(abc, (ra, rb, rc))):.2f}°")
        if key != "s":
            continue
        check = check_ready(cfg, st, reference)
        if not check["ok"]:
            for p in check["problems"]:
                output_fn(f"  저장하지 않음: {p}")
            continue
        ready = save_ready_pose(config_path, st)
        output_fn(f"  저장했습니다: {config_path} (관절 여유 {check['min_margin_deg']:.1f}°, "
                  f"특이점까지 {check['min_singularity_deg']:.1f}°)")
        return {"result": "saved", "ready": ready}


HOME_JOINT_TOL_DEG = 0.5   # 호밍 직후로 볼 관절 각도 허용 오차


def run_raise(link, cfg, confirm, progress=print):
    """호밍 자세인지 확인 -> 계획을 보여 주고 confirm(plan)이 True일 때만 전송 -> 컨트롤러가 보고한 자세를 반환.
    오류 응답이면 바로 멈추고 자동 복구를 하지 않습니다(사람이 확인). 반환 dict: result, plan, reported, (error)."""
    from .draw_executor import ControllerError, parse_status

    s = settings(cfg)
    before = parse_status(link.query_status()[2])
    j = before["joints_deg"]
    if j is None or max(abs(v) for v in j) > HOME_JOINT_TOL_DEG:
        raise StartPoseError(f"호밍 자세(모든 관절 0°)가 아닙니다 (관절 {j}). 가운데 버튼 2초로 호밍한 뒤 다시 하세요.")
    plan = plan_raise(cfg)
    if not plan["commands"]:            # 직각 홀더: 호밍 자세 그대로가 대기 자세
        progress("  펜이 이미 그리는 자세입니다(직각 홀더). 움직이지 않습니다.")
        return {"result": "completed", "plan": plan, "reported": before}
    if not confirm(plan):
        return {"result": "cancelled", "plan": plan, "reported": before}
    for i, (line, label) in enumerate(plan["commands"], start=1):
        progress(f"  [{i}/{len(plan['commands'])}] {label}")
        try:
            link.send_and_ack(line, cfg["ack_timeout_s"])
            link.wait_idle(cfg["idle_timeout_s"])
        except ControllerError as e:
            return {"result": "stopped_on_error", "error": str(e), "plan": plan,
                    "failed_command": label, "reported": parse_status(link.query_status()[2])}
    after = parse_status(link.query_status()[2])
    progress(f"  끝: 관절 {after['joints_deg']}, TCP {after['tcp']}, A·B·C {after['abc_deg']} "
             f"(예상 TCP {plan['standby_tcp_mm']}, 펜 끝 {s['pen_up_deg']:g}° 위)")
    return {"result": "completed", "plan": plan, "reported": after}


def main():
    """펜 올리기 -> 화살표로 종이 가운데 맞추기 -> 시작 자세 저장: python -m mirobot_sketch.start_pose [--execute | --virtual]"""
    import argparse
    import json
    from datetime import datetime

    from . import draw_executor as de
    from . import paths

    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--execute", action="store_true", help="실제 로봇으로 전송 (없으면 계획과 검사 결과만)")
    ap.add_argument("--virtual", action="store_true", help="가상 로봇으로 실행")
    args = ap.parse_args()
    cfg = de.load_config(args.config)
    s = settings(cfg)
    try:
        plan = plan_raise(cfg)
    except StartPoseError as e:
        print(f"계획 실패: {e}")
        return 2
    if plan["commands"]:
        print(f"설정: 펜이 플랜지 축과 나란함, 펜 끝 {s['pen_up_deg']:g}° 위, 펜 길이 최대 {s['pen_len_max_mm']:g} mm, "
              f"종이 면 X {s['paper_x_mm']:g} mm, 옆으로 J1 {s['side_j1_deg']:g}°")
        for line, label in plan["commands"]:
            print(f"  {line}    ; {label}")
        print(f"대기 자세 TCP {plan['standby_tcp_mm']}, 관절 여유 최소 {plan['min_joint_margin_deg']:.1f}°, "
              f"펜 끝-종이 최소 {plan['min_tip_clearance_mm']:.1f} mm, 특이점까지 {plan['standby_singularity_deg']:.1f}°")
    else:
        print("설정: 직각 홀더 — 호밍 자세(팔뚝·펜 수평)가 그리는 자세라 손목을 돌리지 않습니다. "
              "연결 뒤 키보드로 X·Y만 맞춥니다(Z·자세는 호밍 값 고정).")
    if not (args.execute or args.virtual):
        print("dry-run: 로봇을 움직이지 않았습니다. --execute로 실물 실행.")
        return 0
    link = de.open_link(cfg, virtual=args.virtual, virtual_speed=20.0)
    try:
        de.connect_and_home(link, cfg)

        def confirm(_plan):
            print(f"로봇 오른쪽(J1 {s['side_j1_deg']:g}°로 도는 쪽)과 위쪽에 물건이 없고, 펜 길이가 "
                  f"{s['pen_len_max_mm']:g} mm 이하이며, 종이 면이 로봇 중심에서 {s['paper_x_mm']:g} mm 이상 떨어져 있습니까?")
            return args.virtual or input("yes 입력 시 시작: ").strip().lower() == "yes"

        result = run_raise(link, cfg, confirm)
        if result["result"] == "completed":
            result["jog"] = jog_session(link, cfg, result["reported"], args.config or paths.config_path())
    except (de.DrawError, StartPoseError) as e:
        print(f"멈춤: {getattr(e, 'message', e)}")
        return 1
    finally:
        link.close()
    out = paths.runs_dir().parent / "start_pose"
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = out / f"raise-{stamp}.json"
    record = paths.scrub_paths({"started_local": stamp, "virtual": args.virtual, "settings": s, **result})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=1, default=float)
    print(f"결과 {result['result']}, 기록: {path}")
    return 0 if result["result"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
