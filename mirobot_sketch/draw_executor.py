"""
획 JSON -> Mirobot 펜 드로잉 실행기 (Windows 전용)
==================================================
CV/make_strokes.py 또는 GUI가 만든 sketch_strokes JSON(종이 중심 기준 mm)을
읽어 Cartesian 직선 이동 G-code로 바꾸고, 시리얼로 한 줄씩 보냅니다.

기본은 dry-run입니다: 로봇을 움직이지 않고 G-code 파일과 요약만 만듭니다.
실제로 그리려면 --execute를 붙이세요.

    mirobot-draw trajectories/orientation-test-F.json
    mirobot-draw out/photo_strokes.json --execute
    (저장소에서는 python robot/draw_executor.py ... 도 같음)

실행 순서 (설계 문서 / mirobot_control.py와 같은 규칙):
  1. 다른 프로그램이 COM 포트를 열고 있지 않은지 확인
  2. --execute로 실행 -> 포트를 열면 보드가 리셋돼 Alarm 상태가 됨
  3. 안내가 나오면 중앙 버튼 2초로 호밍 -> Idle이 되면 자동으로 다음 단계
  4. 홈 자세 TCP가 종이 중심(drawing_config.json의 paper_center_tcp_mm)과 같아야
     하므로, 홈 자세에서 펜 끝이 종이 중앙에 오도록 종이를 고정해 둔다
  5. 확인 질문에 yes 입력

안전 규칙:
  - 자동 호밍을 하지 않습니다. Idle이 아니면 시작하지 않습니다.
  - 현재 TCP가 설정된 종이 중심에서 max_start_offset_mm 이상 떨어져 있으면 시작하지 않습니다.
  - 종이 범위(limits)를 벗어나는 점이 하나라도 있으면 전송 전에 거부합니다.
    실물 확인 전의 넓은 범위(limits_pending_verification)는 --pending-limits를
    붙였을 때만 쓰며, 그 범위를 확인하는 시험 경로용입니다.
  - --air: 펜을 종이에 대지 않고(펜업 높이) 같은 경로를 따라갑니다. 도달 범위·충돌을
    먼저 확인할 때 씁니다.
  - 명령마다 ok 응답을 기다립니다. Alarm / limit / error 응답이나 타임아웃이 오면
    즉시 전송을 멈추고, 복구 동작을 자동으로 하지 않습니다 (사용자 확인 대상).
"""

import argparse
import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from . import paths, robot_status

DEFAULT_CONFIG = None  # None이면 paths.config_path() (저장소 / 사용자 폴더 / MIROBOT_CONFIG)

ERROR_TOKENS = ("alarm", "error", "limit", "emergency")
CARTESIAN_RE = re.compile(r"Cartesian coordinate\(XYZ RxRyRz\):\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)")
_NUM = r"\s*([-\d.]+)"
POSE_RE = re.compile(r"Cartesian coordinate\(XYZ RxRyRz\):" + ",".join([_NUM] * 6))
ANGLE_RE = re.compile(r"Angle\(ABCDXYZ\):" + ",".join([_NUM] * 7))


def parse_status(raw):
    """컨트롤러 상태 응답 '<상태,Angle(ABCDXYZ):…,Cartesian coordinate(XYZ RxRyRz):…>' ->
    {state, joints_deg [J1..J6] 또는 None, tcp (x,y,z) 또는 None, abc_deg (A,B,C) 또는 None}.
    Angle 값은 J4,J5,J6,D(레일),J1,J2,J3 순서로 옵니다."""
    state = raw[raw.index("<") + 1:].split(",")[0] if "<" in raw else "unknown"
    a, p = ANGLE_RE.search(raw), POSE_RE.search(raw)
    joints = None
    if a:
        v = [float(x) for x in a.groups()]
        joints = [v[4], v[5], v[6], v[0], v[1], v[2]]
    pose = [float(x) for x in p.groups()] if p else None
    return {"state": state, "joints_deg": joints,
            "tcp": tuple(pose[:3]) if pose else None, "abc_deg": tuple(pose[3:]) if pose else None}


# ---------------------------------------------------------------------------
# 입력
# ---------------------------------------------------------------------------

def load_config(path=None):
    from .limits import migrate
    path = path or paths.config_path()
    with open(path, "r", encoding="utf-8") as f:
        return migrate(json.load(f))   # 설치판 사용자 폴더에 남은 예전 기본 넓은 범위(±60)는 새 영역으로


def load_strokes(path):
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("kind") != "sketch_strokes" or doc.get("units") != "mm":
        raise ValueError(f"sketch_strokes(mm) 형식이 아닙니다: {path}")
    strokes = [[(float(x), float(y)) for x, y in s["points_xy_mm"]] for s in doc["strokes"]]
    return doc, [s for s in strokes if len(s) >= 2]


# ---------------------------------------------------------------------------
# 계획: 종이 mm -> 로봇 TCP -> G-code
# ---------------------------------------------------------------------------

def active_limits(cfg, pending=False):
    """평소에는 limits, --pending-limits일 때만 실물 미확인 범위."""
    return cfg["limits_pending_verification"] if pending else cfg["limits"]


def drawing_warp(cfg):
    """그림 좌표 미리 비틀기 계수 {"x": [5], "y": [5]} 또는 None(끔).
    계획한 종이 좌표 (x, y)를 c1·x + c2·y + c3·x² + c4·xy + c5·y²로 바꿔 보내면 실제로는 계획대로 그려진다
    (종이 중심 고정). 높이에 따라 가로·세로 배율이 달라지는 팔을 사진 측정으로 맞춘 값이다."""
    w = cfg.get("drawing_warp")
    if not w:
        return None
    try:
        cx, cy = [float(v) for v in w["x"]], [float(v) for v in w["y"]]
    except (KeyError, TypeError, ValueError):
        raise ValueError("drawing_warp는 x·y 계수 5개씩이어야 합니다.") from None
    if len(cx) != 5 or len(cy) != 5 or not all(math.isfinite(v) for v in cx + cy):
        raise ValueError("drawing_warp는 x·y 계수 5개씩(유한한 숫자)이어야 합니다.")
    if not (0.5 <= cx[0] <= 1.5 and 0.5 <= cy[1] <= 1.5):
        raise ValueError(f"drawing_warp의 1차 배율이 0.5~1.5 밖입니다 (x {cx[0]}, y {cy[1]}): 측정 실수로 보고 멈춥니다.")
    return {"x": cx, "y": cy}


def warp_xy(warp, x, y):
    """계획 종이 좌표 → 로봇에 보낼 종이 좌표 (warp가 None이면 그대로)."""
    if warp is None:
        return x, y
    t = (x, y, x * x, x * y, y * y)
    return sum(c * v for c, v in zip(warp["x"], t)), sum(c * v for c, v in zip(warp["y"], t))


def check_limits(strokes, cfg, pending=False):
    """범위 검사는 실제로 보낼 좌표(미리 비틀기 적용 후)로 하고, 벗어난 점은 계획 좌표로 알린다.
    비스듬히 대고 떼기(pen.edge_rise_mm·edge_land_mm)를 켜면 획 시작 아래·끝 위로 옮긴 점도 본다 (종이 가장자리에서 범위를 넘지 않게)."""
    from .limits import Region
    region = Region.from_cfg(active_limits(cfg, pending))
    warp = drawing_warp(cfg)
    pen = cfg.get("pen", {})
    land = float(pen.get("edge_land_mm", 0.0))

    def points(s):
        s = [tuple(p) for p in s]
        if not s or not edge_slanted(pen):
            return s
        return s + [(s[0][0], s[0][1] - land), (s[-1][0], s[-1][1] + edge_rise_at(pen, s[-1][1]))]
    return [(i, x, y) for i, s in enumerate(strokes) for x, y in points(s) if not region.contains(*warp_xy(warp, x, y))]


END_BACKOFF_MAX_MM = 1.0   # 끝점을 뒤로 물리는 최대량 (이보다 크면 끝점에서 선이 끊김)
J1_OFFSET_MAX_DEG = 3.0   # J1 방향 보정 상한 (측정값 약 0.66°: 이보다 크면 설정 실수로 봄)
EDGE_RISE_MAX_MM = 3.0    # 비스듬히 대고 떼는 세로 이동 상한 (끝점 세로 줄 길이 중앙값 약 2.4 mm)
J1_TURN_EPS_RAD = math.radians(0.01)   # 이보다 작은 J1 변화는 방향이 바뀐 것으로 보지 않음
J1_APPROACH_MARGIN_RAD = math.radians(0.2)   # 획 시작 전 공중에서 미리 돌리는 양 = 보정값 + 이만큼
# 종이 아래쪽일수록 아래로 끌린 세로 줄이 많았다(2026-11-16 두 그림: y −40 mm 아래 33%·25%, 그 위 3~16%).
# pen.edge_rise_low_mm를 y가 이 높이보다 낮아질수록 더해, 이 폭만큼 내려가면 다 더한다.
EDGE_RISE_LOW_FROM_MM = -20.0
EDGE_RISE_LOW_SPAN_MM = 40.0


def edge_slanted(pen):
    """비스듬히 대고 떼기를 켰는지 (켜면 획마다 land·lift-out 명령이 생김)."""
    return float(pen.get("edge_rise_mm", 0.0)) > 0 or float(pen.get("edge_land_mm", 0.0)) != 0


def edge_rise_at(pen, py):
    """종이 높이 py(mm)의 끝점에서 뗄 때 위로 비스듬히 빼는 양: edge_rise_mm + 아래쪽 추가(edge_rise_low_mm × 0~1),
    최대 EDGE_RISE_MAX_MM. edge_rise_mm가 0이면 0."""
    base = float(pen.get("edge_rise_mm", 0.0))
    if base <= 0:
        return 0.0
    k = min(max((EDGE_RISE_LOW_FROM_MM - float(py)) / EDGE_RISE_LOW_SPAN_MM, 0.0), 1.0)
    return min(EDGE_RISE_MAX_MM, base + float(pen.get("edge_rise_low_mm", 0.0)) * k)


class Planner:
    def __init__(self, cfg, air=False):
        self.cfg = cfg
        self.air = air  # True면 펜다운 자세도 펜업 높이로 (종이에 닿지 않음)
        c = cfg["paper_center_tcp_mm"]
        self.cx, self.cy, self.cz = c["x"], c["y"], c["z"]
        o = cfg["orientation_deg"]
        self.abc = (o["a"], o["b"], o["c"])
        self.sy = cfg["paper_x_to_robot_y_sign"]
        self.sz = cfg["paper_y_to_robot_z_sign"]
        pen = cfg["pen"]
        press = float(pen["down_extra_mm"])
        # 공중 경로는 미측정 영역도 지날 수 있으므로 실물 탐색에서 쓴 여유(8 mm)를 유지한다.
        clearance = max(float(pen["up_clearance_mm"]), float(pen.get("air_clearance_mm", 8.0))) if air else float(pen["up_clearance_mm"])
        # 펜업 높이와 느린 구간은 종이 면(접촉 기준)에서 잰다: 더 누르는 양(down_extra)만큼 누른 점에서 더 떨어짐
        self.up_dx = pen["retract_x_sign"] * (clearance + press)
        # 펜업 높이가 slow_zone_mm보다 높으면 종이에 닿기 직전 slow_zone_mm만 느리게(approach) 내리고 그 위는 빠르게(travel).
        # 공중 모드는 종이에 닿지 않으므로 해당 없음. slow_zone_mm 이하로 띄우면 예전처럼 한 번에 approach 속도.
        self.slow_zone = float(pen.get("slow_zone_mm", 2.5))
        self.two_stage = (not air) and clearance > self.slow_zone + 1e-9
        self.slow_dx = pen["retract_x_sign"] * (self.slow_zone + press)
        self.down_extra = -pen["retract_x_sign"] * press  # 누르는 방향 = 후퇴의 반대
        # 끝점 덜 누르기: 펜을 내린 첫 점과 떼기 직전 마지막 점만 종이에서 이만큼 뒤로(후퇴 방향) 둠.
        # 세게 누른 채 내리거나 떼면 펜 끝이 미끄러져 끝점에 세로 줄이 생김 (카메라 확인: pen_correction).
        self.start_backoff = float(pen.get("start_backoff_mm", 0.0))
        self.end_backoff = float(pen.get("end_backoff_mm", 0.0))
        for v in (self.start_backoff, self.end_backoff):
            if not (math.isfinite(v) and 0.0 <= v <= END_BACKOFF_MAX_MM):
                raise ValueError(f"끝점 뒤로 물리는 양은 0~{END_BACKOFF_MAX_MM:g} mm여야 합니다 (지금 {v}).")
        # 비스듬히 대고 떼기. 뗄 때(edge_rise_mm): 접촉점에서 edge_rise_depth_mm 떨어진 점을 종이 위로 옮겨 위쪽으로 비스듬히 뺀다.
        # 끝점마다 아래로 끌린 줄(뗄 때 짐이 빠지며 펜 끝이 내려감)을 상쇄한다. 시험 종이(2026-11-16, 휴대폰 사진): 끝 줄이 0 mm에서
        # 아래 약 2 mm, 1 mm에서 거의 없음, 2~3 mm에서 위로 0.5~1.5 mm.
        # 내릴 때(edge_land_mm): 중간점을 종이 아래(+)·위(−)로 옮겨 비스듬히 댄다. 같은 시험에서 시작점 아래 줄이 0 mm에서
        # 약 2.3 mm였고, 아래에서 올라오며 대면 1 mm마다 약 1 mm씩 길어졌다(펜이 예상보다 일찍 닿아 그 길을 그림). 닿아서 짐이
        # 걸리면 펜 끝이 약 2.3 mm 올라가는 것으로 보고(추정), 위에서 내려오며 대면(−) 그만큼 상쇄될지 시험한다. 기본 0.
        self.edge_rise = float(pen.get("edge_rise_mm", 0.0))
        self.edge_low = float(pen.get("edge_rise_low_mm", 0.0))
        self.edge_land = float(pen.get("edge_land_mm", 0.0))
        self.edge_depth = float(pen.get("edge_rise_depth_mm", 1.5))
        for v in (self.edge_rise, self.edge_low):
            if not (math.isfinite(v) and 0.0 <= v <= EDGE_RISE_MAX_MM):
                raise ValueError(f"비스듬히 떼는 세로 이동은 0~{EDGE_RISE_MAX_MM:g} mm여야 합니다 (지금 {v}).")
        if not (math.isfinite(self.edge_land) and abs(self.edge_land) <= EDGE_RISE_MAX_MM):
            raise ValueError(f"비스듬히 대는 세로 이동은 ±{EDGE_RISE_MAX_MM:g} mm 안이어야 합니다 (지금 {self.edge_land}).")
        room = self.slow_zone if self.two_stage else clearance
        if edge_slanted(pen) and not (math.isfinite(self.edge_depth)
                                      and max(self.start_backoff, self.end_backoff) < self.edge_depth < room):
            raise ValueError(f"비스듬히 대고 떼는 깊이는 끝점 물림({max(self.start_backoff, self.end_backoff):g} mm)보다 크고 "
                             f"{room:g} mm보다 작아야 합니다 (지금 {self.edge_depth}).")
        self.pen = pen
        pc = cfg["plane_compensation"]
        self.pa, self.pb = pc["a_per_mm_y"], pc["b_per_mm_z"]
        self.radial_max = float(pc.get("radial_retract_max_mm", 0.0))
        self.radial_radius = float(pc.get("radial_retract_radius_mm", 60.0))
        if not math.isfinite(self.radial_max) or self.radial_max < 0 or not math.isfinite(self.radial_radius) or self.radial_radius <= 0:
            raise ValueError("방사형 후퇴량은 0 이상, 기준 거리는 0보다 커야 합니다.")
        self.retract_sign = float(pen["retract_x_sign"])
        # J1 방향 보정: 같은 J1 목표라도 J1이 늘어나며 도착할 때와 줄어들며 도착할 때 실제 각이 이만큼 다르다
        # (늘어나며 도착 - 줄어들며 도착, 2026-11-12 대기 자세 사진 비교로 약 +0.66°). 0이면 끔.
        # 목표점마다 도착 방향을 보고 반대쪽으로 절반씩 돌려 보내고, 방향이 바뀌는 점에서는 같은 점으로 한 번 더
        # 보내 유격을 먼저 메운다 (그리는 도중 방향이 바뀌어도 선이 옆으로 밀리지 않게).
        self.j1_offset = math.radians(float(pen.get("j1_direction_offset_deg", 0.0)))
        if not (math.isfinite(self.j1_offset) and abs(self.j1_offset) <= math.radians(J1_OFFSET_MAX_DEG)):
            raise ValueError(f"J1 방향 보정은 ±{J1_OFFSET_MAX_DEG:g}° 안이어야 합니다.")
        self._j1_dir = 1            # 대기 자세에는 J1 -90° → 0°로, 늘어나며 도착한다
        self._j1_prev = None        # 마지막으로 보낸 목표점(보정 전)
        self.feeds = cfg["feeds_mm_per_min"]
        self.warp = drawing_warp(cfg)

    def robot_yz(self, px, py):
        px, py = warp_xy(self.warp, px, py)
        return self.cy + self.sy * px, self.cz + self.sz * py

    def contact_x(self, ry, rz):
        distance = math.hypot(ry - self.cy, rz - self.cz)
        radial = self.retract_sign * self.radial_max * min(float(distance) / self.radial_radius, 1.0)
        return self.cx + self.pa * (ry - self.cy) + self.pb * (rz - self.cz) + self.down_extra + radial

    def _off_paper(self, contact, x):
        """접촉점에서 종이 수직 방향(X)으로 x까지 뺀 점. J1 방향 보정이 켜져 있으면 베이스 축에서 접촉점으로 뻗은
        선(같은 J1) 위로 빼서, 펜을 내리고 올리는 동안 J1이 반대로 돌아 유격 상태가 바뀌지 않게 한다.
        공중 모드는 종이에 닿지 않으므로 그대로 X만 뺀다 (공중 경로가 그림 모양을 그대로 따라가게)."""
        cx, ry, rz = contact
        return x, (ry * x / cx if self.j1_offset and not self.air else ry), rz

    def pose(self, px, py, pen_down):
        ry, rz = self.robot_yz(px, py)
        x = self.contact_x(ry, rz)
        if not pen_down:
            return self._off_paper((x, ry, rz), x + self.up_dx)
        if self.air:     # 공중 모드의 그리는 점: 펜업 높이에서 같은 Y·Z (그림 모양을 그대로 따라감)
            return x + self.up_dx, ry, rz
        return x, ry, rz

    def pose_slow_zone(self, px, py):
        """종이에 닿기 직전 느린 구간이 시작되는 높이(접촉면에서 slow_zone_mm 위)."""
        ry, rz = self.robot_yz(px, py)
        x = self.contact_x(ry, rz)
        return self._off_paper((x, ry, rz), x + self.slow_dx)

    def gcode(self, pose, feed):
        x, y, z = pose
        a, b, c = self.abc
        return f"M20 G90 G01 X{x:.3f} Y{y:.3f} Z{z:.3f} A{a:.3f} B{b:.3f} C{c:.3f} F{feed:.0f}"

    def _farther_x(self, current, other):
        """둘 중 종이에서 더 떨어진 X. 보정 이동은 이 쪽으로만 갱신한다."""
        sign = self.cfg["pen"]["retract_x_sign"]
        return current if sign * (current - other) >= 0 else other

    def _rotated(self, pose, angle):
        """베이스 축 둘레로 angle(rad)만큼 돌린 목표점 (자세 A·B·C는 그대로)."""
        x, y, z = pose
        c, s = math.cos(angle), math.sin(angle)
        return x * c - y * s, x * s + y * c, z

    def _j1_turned(self, pose):
        """J1 도착 방향의 반대쪽으로 보정의 절반만큼 돌린 목표점."""
        return self._rotated(pose, -0.5 * self.j1_offset * self._j1_dir)

    def _first_j1_dir(self, stroke):
        """획의 첫 그리기 이동에서 J1이 도는 방향(+1/-1). 보정이 꺼져 있거나 J1이 거의 안 돌면 0."""
        if not self.j1_offset or len(stroke) < 2:
            return 0
        a, b = self.pose(*stroke[0], True), self.pose(*stroke[1], True)
        turn = math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])
        return 0 if abs(turn) <= J1_TURN_EPS_RAD else (1 if turn > 0 else -1)

    def _emit(self, items):
        """[(목표점, 속도, 설명)] -> [(gcode, 설명)]. J1 방향 보정이 켜져 있으면 방향이 바뀌는 점마다
        직전 목표점으로 새 보정값을 먼저 보내는 'j1-takeup' 명령을 넣는다 (설명 끝말은 원래 명령과 같게)."""
        out = []
        for pose, feed, label in items:
            if self.j1_offset and self._j1_prev is not None:
                turn = math.atan2(pose[1], pose[0]) - math.atan2(self._j1_prev[1], self._j1_prev[0])
                if abs(turn) > J1_TURN_EPS_RAD and (turn > 0) != (self._j1_dir > 0):
                    self._j1_dir = 1 if turn > 0 else -1
                    head, _, tail = label.rpartition(" ")
                    out.append((self.gcode(self._j1_turned(self._j1_prev), feed), f"{head} j1-takeup {tail}"))
            out.append((self.gcode(self._j1_turned(pose) if self.j1_offset else pose, feed), label))
            self._j1_prev = pose
        return out

    def return_to_origin(self):
        """그림을 다 그린 뒤 펜을 처음 상태(종이 중심, 펜 끝이 종이에 살짝 닿음)로 되돌리는 명령들.
        plan()이 종이 중심 위에서 펜업으로 끝난 뒤 이어 보낸다(J1 방향 보정 상태도 이어받음). 공중 모드는 해당 없음."""
        if self.air:
            return []
        f = self.feeds
        items = []
        if self.two_stage:
            items.append((self.pose_slow_zone(0, 0), f["travel"], "return origin descend"))
        items.append((self.pose(0, 0, True), f["approach"], "return origin"))
        return self._emit(items)

    def _backed(self, pose, mm):
        """접촉 자세를 종이에서 mm만큼 뒤로 (공중 모드는 이미 떠 있으므로 그대로)."""
        if self.air or not mm:
            return pose
        return self._off_paper(pose, pose[0] + self.retract_sign * mm)

    def _edge(self, px, py):
        """비스듬히 대고 떼기의 중간점: (px, py) 접촉점에서 종이 반대쪽으로 edge_rise_depth_mm."""
        contact = self.pose(px, py, True)
        return self._off_paper(contact, contact[0] + self.retract_sign * self.edge_depth)

    def plan(self, strokes):
        """[(gcode, 설명)] 리스트. 획이 끝나면 그 자리에서 펜업하고, 그 X를 유지한 채 다음 위치로 간 뒤에만 내린다.
        시작은 종이 중심 펜다운(사람이 맞춘 시작 자세)이고, J1 방향 보정 상태는 매번 처음부터 다시 센다."""
        f = self.feeds
        self._j1_dir, self._j1_prev = 1, self.pose(0, 0, True)
        home = self.pose(0, 0, False)
        items = [(home, f["approach"], "center pen-up")]
        held_x = home[0]
        slanted = not self.air and edge_slanted(self.pen)   # 켜면 획마다 land·lift-out 명령이 생김 (높이에 따라 양만 다름)
        for i, s in enumerate(strokes):
            x0, y0 = s[0]
            ya = y0 - (self.edge_land if slanted else 0.0)    # 내릴 때 +면 아래에서 올라오며, −면 위에서 내려오며 댐 (기본 0: 곧게)
            up = self.pose(x0, ya, False)
            travel_x = self._farther_x(held_x, up[0])
            travel = self._off_paper(self.pose(x0, ya, True), travel_x)
            first = self._first_j1_dir(s)
            if first:   # 첫 그리기 방향으로 공중에서 미리 J1을 움직여, 종이 위 첫 이동에서 유격을 메우지 않게
                lead = -first * (abs(self.j1_offset) + J1_APPROACH_MARGIN_RAD)
                items.append((self._rotated(travel, lead), f["travel"], f"stroke {i} j1-approach travel"))
            items.append((travel, f["travel"], f"stroke {i} travel"))
            if self.two_stage:   # 높이 띄운 곳에서 빠르게 내려오다 접촉 직전부터 천천히
                items.append((self.pose_slow_zone(x0, ya), f["travel"], f"stroke {i} descend"))
            if slanted:
                items.append((self._edge(x0, ya), f["approach"], f"stroke {i} land"))
            items.append((self._backed(self.pose(x0, y0, True), self.start_backoff), f["approach"], f"stroke {i} pen-down"))
            last = len(s) - 1
            for j, (x, y) in enumerate(s[1:], 1):
                pose = self.pose(x, y, True)
                if j == last:
                    pose = self._backed(pose, self.end_backoff)
                items.append((pose, f["draw"], f"stroke {i} draw"))
            x1, y1 = s[-1]
            yb = y1 + (edge_rise_at(self.pen, y1) if slanted else 0.0)   # 뗄 때는 위로 비스듬히 빼고
            # 그 자리에서 뒤로 뺀다(켜져 있으면 종이 위쪽으로 조금 비스듬히). 옆으로 가는 명령은 이 다음이다.
            if slanted:
                items.append((self._edge(x1, yb), f["approach"], f"stroke {i} lift"))
                if self.two_stage:
                    items.append((self.pose_slow_zone(x1, yb), f["approach"], f"stroke {i} lift-out"))
            elif self.two_stage:
                items.append((self.pose_slow_zone(x1, y1), f["approach"], f"stroke {i} lift"))
            end_up = self.pose(x1, yb, False)
            items.append((end_up, f["travel"] if self.two_stage else f["approach"], f"stroke {i} pen-up"))
            held_x = end_up[0]
        back = self.pose(0, 0, False)
        items.append(((self._farther_x(held_x, back[0]), back[1], back[2]), f["travel"], "return center pen-up"))
        return self._emit(items)


def estimate_time(strokes, cfg, air=False):
    """그리기 시간 추정. CV(make_strokes, GUI)와 실행기가 같은 함수를 씁니다.

    반환 dict:
      draw_s       펜다운 이동 (길이 / draw 속도)
      travel_s     펜업 이동 (획 사이 거리 / travel 속도, 종이 중심 출발·복귀 포함)
      pen_lift_s   일반: 획마다 펜 내림+올림. 공중: 시작점에서 후퇴하는 시간
      command_count, latency_s  명령마다 ok 응답을 기다리는 지연 (timing 설정의 가정값)
      motion_s     = draw + travel + pen_lift (가감속 제외 이론값)
      total_s      = motion + latency
    길이는 로봇에 실제로 보낼(drawing_warp로 미리 비튼) 좌표로 잽니다.
    """
    f = cfg["feeds_mm_per_min"]
    clear = cfg["pen"]["up_clearance_mm"]
    warp = drawing_warp(cfg)                              # 종이 중심(0, 0)은 비틀어도 그대로
    if warp is not None:
        strokes = [[warp_xy(warp, float(x), float(y)) for x, y in s] for s in strokes]
    down = up = 0.0
    pos = (0.0, 0.0)
    for s in strokes:
        up += _dist(pos, s[0])
        down += sum(_dist(a, b) for a, b in zip(s, s[1:]))
        pos = s[-1]
    up += _dist(pos, (0.0, 0.0))
    n = len(strokes)
    draw_s = down / f["draw"] * 60.0
    travel_s = up / f["travel"] * 60.0
    slow = float(cfg["pen"].get("slow_zone_mm", 2.5))
    press = float(cfg["pen"].get("down_extra_mm", 0.0))
    lift = clear + press                                   # 누른 점에서 펜업 높이까지 (Planner.up_dx)
    two_stage = (not air) and clear > slow + 1e-9        # Planner.two_stage와 같은 조건
    slow_lift = two_stage  # 접촉점부터 느린 구간을 먼저 수직으로 벗어난 뒤 빠르게 올림
    if air:
        pen_lift_s = max(clear, float(cfg["pen"].get("air_clearance_mm", 8.0))) / f["approach"] * 60.0
    elif slow_lift:
        pen_lift_s = (lift / f["approach"] + n * (2 * (clear - slow) / f["travel"]
                                                  + 2 * (slow + press) / f["approach"])) * 60.0
    elif two_stage:   # 처음 중심 펜업(approach) + 획마다 [빠른 하강 + 느린 접촉·누름 구간 + 빠른 올림(travel)]
        pen_lift_s = (lift / f["approach"] + n * ((clear - slow) / f["travel"] + (slow + press) / f["approach"]
                                                  + lift / f["travel"])) * 60.0
    else:
        pen_lift_s = lift * (2 * n + 1) / f["approach"] * 60.0
    command_count = 2 + sum(len(s) + 2 for s in strokes) + (n if two_stage else 0) + (n if slow_lift else 0)
    if not air and edge_slanted(cfg["pen"]):   # 비스듬히 대고 떼기: 획마다 중간점 2개, 그 두 구간은 곧게 가는 대신 비스듬히(느린 속도)
        pen = cfg["pen"]
        depth = float(pen.get("edge_rise_depth_mm", 1.5))
        command_count += 2 * n
        slant = math.hypot(depth, float(pen.get("edge_rise_mm", 0.0))) + math.hypot(depth, float(pen.get("edge_land_mm", 0.0)))
        pen_lift_s += n * (slant - 2 * depth) / f["approach"] * 60.0
    if float(cfg["pen"].get("j1_direction_offset_deg", 0.0)):   # J1 방향 보정은 획마다 접근점·유격 메우기를 더함
        command_count = len(Planner(cfg, air=air).plan(strokes))
    latency = cfg.get("timing", {}).get("assumed_command_latency_s", 0.0)
    motion_s = draw_s + travel_s + pen_lift_s
    return {
        "pen_down_mm": round(down, 1), "pen_up_mm": round(up, 1), "stroke_count": n,
        "draw_s": round(draw_s, 1), "travel_s": round(travel_s, 1), "pen_lift_s": round(pen_lift_s, 1),
        "command_count": command_count, "latency_s": round(command_count * latency, 1),
        "motion_s": round(motion_s, 1), "total_s": round(motion_s + command_count * latency, 1),
        "up_clearance_mm": float(clear),
    }


def format_time(t):
    """사람이 읽는 한 줄 요약."""
    return (f"약 {t['total_s'] / 60:.1f}분 (그리기 {t['draw_s'] / 60:.1f} + 이동 {t['travel_s'] / 60:.1f}"
            f" + 펜 올림·내림 {t['pen_lift_s'] / 60:.1f} + 명령 지연 {t['latency_s'] / 60:.1f}분, "
            f"명령 {t['command_count']}개)")


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


# ---------------------------------------------------------------------------
# 시리얼
# ---------------------------------------------------------------------------

class ControllerError(RuntimeError):
    pass


class StopRequested(Exception):
    """사용자가 [멈춤]을 눌러 다음 명령을 보내지 않음."""


class MirobotLink:
    """시리얼 연결을 한 번만 열고 재사용합니다. ser는 테스트용 가짜 객체로 바꿀 수 있습니다."""

    def __init__(self, ser, verbose=False):
        self.ser = ser
        self.verbose = verbose
        self.status_observer = None

    @classmethod
    def open(cls, port, baud, verbose=False):
        import serial

        ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2)  # 포트를 열면 보드가 리셋됨 -> 부팅 대기
        ser.read(ser.in_waiting or 1)
        return cls(ser, verbose)

    def _readline(self):
        return self.ser.readline().decode(errors="replace").strip()

    def query_status(self, timeout=2.0):
        """(상태 문자열, (x, y, z) 또는 None, 원문)."""
        self.ser.reset_input_buffer()
        self.ser.write(b"?\r\n")
        deadline = time.monotonic() + timeout
        buf = ""
        while time.monotonic() < deadline:
            buf += self._readline()
            if "<" in buf and ">" in buf:
                break
        if "<" not in buf:
            if self.status_observer is not None:
                self.status_observer("unknown", None, buf)
            return "unknown", None, buf
        body = buf[buf.index("<") + 1:]
        state = body.split(",")[0]
        m = CARTESIAN_RE.search(buf)
        tcp = tuple(float(v) for v in m.groups()) if m else None
        if self.status_observer is not None:
            self.status_observer(state, tcp, buf)
        return state, tcp, buf

    def read_settings(self, timeout=10.0):
        """$$ 설정 덤프. send_and_ack는 쓰지 않음 ($20 설명의 limit가 오류로 잡힘).
        실물에서 쓰던 방식(LOG/scripts/depth_ladder/plane_ladder.py)과 같다: 입력 버퍼를 비우고 ok까지 읽는다."""
        self.ser.reset_input_buffer()
        self.ser.write(b"$$\r\n")
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline and b"\nok" not in buf:
            chunk = self.ser.read(getattr(self.ser, "in_waiting", 0) or 1)
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk if isinstance(chunk, (bytes, bytearray)) else str(chunk).encode()
        got = {}
        for ln in buf.decode("utf-8", "replace").splitlines():
            if not ln.startswith("$") or "=" not in ln:
                continue
            key, rest = ln.split("=", 1)
            got[key] = rest.split(" ")[0]
        return got

    def wait_idle(self, timeout, should_stop=None):
        return self._wait_settled(timeout, None, None, should_stop=should_stop)

    def wait_until_x(self, target_x, timeout, tol=1.5):
        """뺀 X가 보고될 때까지. 좌표가 없는 Idle은 통과하지 않는다."""
        return self._wait_settled(timeout, float(target_x), float(tol))

    def wait_until_pose(self, target, timeout, tol=0.6, should_stop=None):
        """컨트롤러가 목표 XYZ에 도착해 Idle이 될 때까지 기다린다."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if should_stop and should_stop():
                raise StopRequested()
            state, tcp, raw = self.query_status()
            if any(t in raw.lower() for t in ERROR_TOKENS):
                raise ControllerError(f"위치 대기 중 오류 상태: {raw}")
            if state == "Idle" and tcp is not None and all(abs(a-b) <= tol for a,b in zip(tcp, target)):
                return tcp
            time.sleep(0.05)
        raise ControllerError("목표 위치 도착을 확인하지 못했습니다")

    def wait_until_left_contact(self, contact_x, timeout, retract_sign=-1.0, min_mm=5.0):
        """이번 획의 접촉 X를 본 뒤, 종이 반대쪽으로 min_mm 이상 움직인 보고가 올 때까지.
        목표 X나 직전 빼기 X만 맞은 샘플은 통과하지 않는다."""
        deadline = time.monotonic() + timeout
        saw_contact = False
        while time.monotonic() < deadline:
            state, tcp, raw = self.query_status()
            if any(t in raw.lower() for t in ERROR_TOKENS):
                raise ControllerError(f"Idle 대기 중 오류 상태: {raw}")
            if tcp is not None:
                if abs(tcp[0] - contact_x) <= 2.0:
                    saw_contact = True
                away = (contact_x - tcp[0]) * (-float(retract_sign))
                if saw_contact and state == "Idle" and away >= min_mm:
                    return tcp
            time.sleep(0.05)
        raise ControllerError("펜이 종이에서 떨어진 위치를 확인하지 못했습니다")

    def _wait_settled(self, timeout, target_x, tol, target_yz=None, yz_tol=1.0, contact_sign=1.0,
                      should_stop=None):
        deadline = time.monotonic() + timeout
        first = True
        saw_move = False
        saw_contact = False
        while time.monotonic() < deadline:
            if should_stop and should_stop():
                raise StopRequested()
            state, tcp, raw = self.query_status()
            if any(t in raw.lower() for t in ERROR_TOKENS):
                raise ControllerError(f"Idle 대기 중 오류 상태: {raw}")
            if target_x is None:
                if state == "Idle":
                    return tcp
            else:
                x_ok = tcp is not None and abs(tcp[0] - target_x) <= tol
                pose_ok = x_ok and target_yz is not None and (
                    abs(tcp[1] - target_yz[0]) <= yz_tol and abs(tcp[2] - target_yz[1]) <= yz_tol)
                if tcp is not None and (tcp[0] - target_x) * contact_sign >= 5.0:
                    saw_contact = True
                if target_yz is None:
                    if state != "Idle" or (tcp is not None and not x_ok):
                        saw_move = True
                    if state == "Idle" and x_ok and (first or saw_move):
                        return tcp
                elif state == "Idle" and ((first and pose_ok) or (saw_contact and x_ok)):
                    return tcp
            first = False
            time.sleep(0.05)
        raise ControllerError("펜을 뒤로 뺀 위치를 확인하지 못했습니다" if target_x is not None else "Idle 대기 시간 초과")

    def send_and_ack(self, line, timeout, should_stop=None):
        """한 줄을 보내고 ok를 기다립니다. 오류 응답이나 타임아웃이면 ControllerError.
        should_stop은 받기만 함: 실제 로봇은 이미 보낸 명령을 도중에 멈추지 않음 (다음 명령부터 안 보냄)."""
        self.ser.write((line + "\r\n").encode("ascii"))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            resp = self._readline()
            if not resp:
                continue
            if self.verbose:
                print(f"    <- {resp}")
            low = resp.lower()
            if any(t in low for t in ERROR_TOKENS):
                raise ControllerError(f"컨트롤러 오류 응답: {resp}")
            if low.startswith("ok"):
                return
        raise ControllerError(f"ok 응답 시간 초과: {line}")

    def wait_for_homing(self, timeout, progress=print, should_cancel=None):
        """포트를 열면 보드가 리셋돼 Alarm으로 시작합니다. 사용자가 물리 버튼으로
        호밍해 Idle이 될 때까지 기다립니다 (자동 호밍 명령은 보내지 않음).
        반환: (상태, TCP). 시간 안에 Idle이 안 되면 마지막 상태를 그대로 반환. 취소하면 ("cancelled", None)."""
        deadline = time.monotonic() + timeout
        last = None
        state, tcp = "unknown", None
        while time.monotonic() < deadline:
            if should_cancel and should_cancel():
                return "cancelled", None
            state, tcp, _ = self.query_status()
            if state != last:
                progress(f"  컨트롤러 상태: {state}")
                if state == "Alarm":
                    progress("  -> 중앙 네비게이션 버튼을 2초 눌러 호밍하세요. Idle이 되면 자동으로 계속합니다.")
                last = state
            if state == "Idle":
                break
            time.sleep(1.0)
        return state, tcp

    def close(self):
        self.ser.close()


# 실행 단계 (GUI 단계 표시줄과 명령줄 출력이 같은 이름을 씀): (id, 이름, 누가)
DRAW_STEPS = (("preflight", "사전 검사", "auto"), ("connect", "연결·호밍", "person"),
              ("start", "시작 위치 확인", "auto"), ("confirm", "최종 확인", "person"),
              ("drawing", "그리는 중", "auto"), ("done", "끝", "auto"))


class DrawError(Exception):
    """단계에서 멈춤: 어느 단계(step), 무엇이(message), 어떻게 하면 되는지(hint)."""

    def __init__(self, step, message, hint=""):
        super().__init__(message)
        self.step, self.message, self.hint = step, message, hint


def preflight(strokes, cfg, pending=False, air=False):
    """① 범위 검사 + 명령 계획 + 시간 추정. 로봇에 연결하기 전."""
    if not strokes:
        raise DrawError("preflight", "그릴 획이 없습니다.")
    bad = check_limits(strokes, cfg, pending)
    if bad:
        i, x, y = bad[0]
        hint = ("그림 크기를 줄이세요." if pending else
                "그림 크기를 줄이거나, 실물 확인 전 넓은 범위를 쓰려면 '넓은 범위 허용'을 켜세요.")
        raise DrawError("preflight", f"종이 허용 범위를 벗어난 점 {len(bad)}개 (예: 획 {i}, x={x:.1f}, y={y:.1f} mm)", hint)
    planner = Planner(cfg, air=air)
    return {"cmds": planner.plan(strokes), "timing": estimate_time(strokes, cfg, air=air), "planner": planner}


def open_link(cfg, virtual=False, virtual_speed=20.0, verbose=False):
    if virtual:
        from .virtual_robot import VirtualMirobotLink
        return VirtualMirobotLink(cfg, speed=virtual_speed)
    try:
        choice = robot_status.detect_robot_port(cfg.get("port"))
    except Exception as e:  # pyserial 포트 목록을 읽지 못함
        raise DrawError("connect", f"USB 포트 목록을 읽지 못했습니다: {e}") from None
    if choice.port is None:
        hint = ("사용할 포트를 drawing_config.json의 port에 적어 주세요." if choice.source == "ambiguous" else
                "로봇 전원과 USB 케이블을 확인하세요. 다른 USB-시리얼 어댑터라면 drawing_config.json의 port에 "
                "포트 이름을 적어 주세요.")
        raise DrawError("connect", robot_status.port_choice_problem(choice) + ".", hint)
    try:
        link = MirobotLink.open(choice.port, cfg["baud"], verbose)
    except Exception as e:  # 포트 사용 중·권한 없음 (serial.SerialException 등)
        raise DrawError("connect", f"포트 {choice.port}를 열 수 없습니다: {e}",
                        "USB 연결을 확인하고, 다른 프로그램이 같은 포트를 쓰고 있지 않은지 보세요.") from None
    link.port = choice.port
    return link


def connect_and_home(link, cfg, progress=print, should_cancel=None):
    """② 호밍 대기 (자동 호밍 명령은 보내지 않음). 반환: TCP."""
    state, tcp = link.wait_for_homing(cfg["idle_timeout_s"], progress=progress, should_cancel=should_cancel)
    if state == "cancelled":
        raise DrawError("connect", "취소했습니다.")
    if state != "Idle":
        raise DrawError("connect", f"Idle이 되지 않았습니다 (상태: {state}).",
                        "로봇 가운데 네비게이션 버튼을 2초 눌러 호밍한 뒤 다시 시작하세요.")
    if tcp is None:
        raise DrawError("connect", "TCP 좌표를 읽지 못했습니다.", "연결을 다시 해 보세요.")
    return tcp


DEFAULT_CONTROLLER_SETTINGS = {"$3": 123, "$23": 60}


def require_controller_settings(link, cfg):
    """방향 마스크가 기대와 같아야 움직인다. $$는 send_and_ack로 읽지 않는다.
    반환: 읽은 값 dict, 또는 설정에서 controller_settings_expected를 null/{}로 둬 끈 경우 None
    (실물 컨트롤러에서 읽기가 실패해 그리기가 막힐 때의 탈출구. 끈 사실은 실행 기록에 남긴다)."""
    expected = (cfg or {}).get("controller_settings_expected", DEFAULT_CONTROLLER_SETTINGS)
    if not expected:
        return None
    try:
        got = link.read_settings()
    except OSError as e:
        raise DrawError("start", f"컨트롤러 설정을 읽지 못했습니다: {e}",
                        "USB 연결을 확인하세요.") from None
    bad = []
    for key, want in expected.items():
        have = got.get(str(key))
        if have is None or str(have) != str(want):
            shown = "없음" if have is None else have
            bad.append(f"{key}={shown} (필요 {want})")
    if bad:
        raise DrawError(
            "start",
            "컨트롤러 방향 설정이 기대와 다릅니다: " + ", ".join(bad) + ". 움직이지 않았습니다.",
            "방향 마스크가 펌웨어 기본값이면 팔이 반대로 움직입니다. 설정을 확인한 뒤 다시 시작하세요. "
            "값이 맞는데 읽기만 실패한다면 설정 파일의 controller_settings_expected를 null로 두면 이 확인을 건너뜁니다.")
    return {str(k): got.get(str(k)) for k in expected}


RETRACTED_START_MAX_MM = 20.0   # 캘리브레이션 직후 펜을 벽에서 뺀 자세로 허용하는 최대 후퇴 거리


def check_start(tcp, cfg, retracted=False):
    """③ 현재 펜 끝이 설정된 종이 중심 근처인지. 반환: 떨어진 거리(mm).

    retracted=True: 캘리브레이션이 끝난 직후. 그 절차는 펜을 벽에서 뺀(펜업) 자세로 끝나므로, Y·Z는 종이 중심과
    max_start_offset_mm 안에서 같아야 하고 X는 벽 반대 방향으로 RETRACTED_START_MAX_MM까지만 물러난 것을 허용한다
    (벽 쪽으로 들어가 있으면 허용하지 않음)."""
    c = cfg["paper_center_tcp_mm"]
    off = ((tcp[0] - c["x"]) ** 2 + (tcp[1] - c["y"]) ** 2 + (tcp[2] - c["z"]) ** 2) ** 0.5
    if retracted:
        back = (tcp[0] - c["x"]) * float(cfg["pen"]["retract_x_sign"])     # +: 벽에서 물러난 양
        lateral = ((tcp[1] - c["y"]) ** 2 + (tcp[2] - c["z"]) ** 2) ** 0.5
        if lateral > cfg["max_start_offset_mm"] or not -1.0 <= back <= RETRACTED_START_MAX_MM:
            raise DrawError("start", f"보정 뒤 펜 끝이 종이 중심에서 벗어났습니다 (옆으로 {lateral:.1f} mm, "
                                     f"벽에서 물러난 양 {back:.1f} mm; 허용 옆 {cfg['max_start_offset_mm']} mm, "
                                     f"뒤로 {RETRACTED_START_MAX_MM:g} mm).",
                            "종이와 로봇을 확인하고 다시 보정하세요.")
        return off
    if off > cfg["max_start_offset_mm"]:
        raise DrawError("start", f"펜 끝이 종이 중심 설정에서 {off:.1f} mm 떨어져 있습니다 "
                                 f"(허용 {cfg['max_start_offset_mm']} mm).",
                        "호밍 뒤 펜 끝이 종이 가운데에 오도록 종이 위치를 맞추세요.")
    return off


def write_run_record(meta, result, cfg):
    """⑥ runs/run-<끝난 시각>.json. meta: strokes_json, stroke_count, command_count, air_mode, pending_limits,
    estimated_time, source, virtual, virtual_speed 등. meta에 started_local(시작 시각)이 없으면 끝난 시각을 넣는다.
    config_snapshot은 끝났을 때 설정이다(그리는 중 바뀐 펜 값은 meta의 pen_changes)."""
    run_dir = paths.runs_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    # 공개 저장소에 올려도 사용자 폴더 이름이 남지 않게 경로를 정리해서 기록
    run = paths.scrub_paths({"started_local": stamp, "finished_local": stamp, **meta, "config_snapshot": cfg,
                             **result, "visual_verification": "pending"})
    path = run_dir / f"run-{stamp}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(run, f, ensure_ascii=False, indent=1)
    return path


_XYZ = re.compile(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)")


def resume_commands(planner, cmds, start):
    """[멈춤]으로 펜을 종이 반대쪽(같은 Y·Z)으로 뗀 뒤 cmds[start:]를 이어 보내기 전에 보낼 명령.
    반환: (먼저 보낼 [(gcode, 설명)], 이어 보낼 첫 명령 번호).
    - 펜이 종이에 닿은 채 멈췄으면(마지막으로 보낸 명령이 pen-down·draw): 그 접촉점 바로 위 느린 구간까지 빠르게,
      같은 접촉점까지 천천히 다시 펜을 대고 남은 획을 잇는다 (이미 그은 부분은 다시 긋지 않음).
      다만 그 획을 다 그은 뒤라면(다음이 그 획의 lift·pen-up) 이미 뗐으므로 그 둘을 건너뛴다.
    - 다음 명령이 펜 내림이면 그 접촉점 위 느린 구간까지 먼저 빠르게 내려온다 (뗀 높이에서 끝까지 느리게 내리지 않게).
    - 공중 모드는 종이에 닿지 않으므로 그대로 잇는다."""
    if planner.air or start <= 0 or start >= len(cmds):
        return [], start
    f = planner.feeds
    prev_line, prev = cmds[start - 1]
    if prev.startswith("stroke ") and (prev.endswith(" pen-down") or prev.endswith(" draw")):
        stroke = prev.split(" ")[1]
        i = start
        while (i < len(cmds) and cmds[i][1].startswith(f"stroke {stroke} ")
               and cmds[i][1].endswith((" lift", " lift-out", " pen-up"))):
            i += 1
        if i > start:
            return [], i
        x, y, z = (float(v) for v in _XYZ.search(prev_line).groups())
        out = []
        if planner.two_stage:
            out.append((planner.gcode((planner.contact_x(y, z) + planner.slow_dx, y, z), f["travel"]),
                        f"stroke {stroke} resume descend"))
        out.append((planner.gcode((x, y, z), f["approach"]), f"stroke {stroke} resume pen-down"))
        return out, start
    line, label = cmds[start]
    if planner.two_stage and label.startswith("stroke ") and label.endswith((" pen-down", " land")):
        _, y, z = (float(v) for v in _XYZ.search(line).groups())
        return [(planner.gcode((planner.contact_x(y, z) + planner.slow_dx, y, z), f["travel"]),
                 f"stroke {label.split(' ')[1]} resume descend")], start
    return [], start


def execute(link, cmds, cfg, progress=print, on_ack=None, should_stop=None, start=0):
    """계획된 명령을 순서대로 보냅니다. 반환: 실행 결과 dict. 오류·멈춤 시 즉시 멈춥니다(자동 복구 없음).
    on_ack(보낸 수, 전체 수): ok마다 호출 (GUI 진행 표시·RViz 따라가기).
    should_stop(): True면 다음 명령부터 보내지 않음 (가상 시뮬레이션은 기다리는 중에도 바로 멈춤).
    start: 이 번호의 명령부터 보냄(멈춘 뒤 이어 그리기). 보낸 수(commands_sent)는 cmds 처음부터 센 번호."""
    started = time.monotonic()
    sent = start
    stop = should_stop or (lambda: False)
    contact_line = None
    trace = []
    context = {"command": "", "target": ""}
    previous_observer = getattr(link, "status_observer", None)

    def observe(state, tcp, raw):
        if len(trace) < 20000:
            parsed = parse_status(raw)
            trace.append({"time_s": round(time.monotonic() - started, 3),
                          "command": context["command"], "target": context["target"],
                          "state": state, "tcp_mm": tcp, "abc_deg": parsed["abc_deg"],
                          "joints_deg": parsed["joints_deg"]})
        if previous_observer is not None:
            previous_observer(state, tcp, raw)

    if hasattr(link, "status_observer"):
        link.status_observer = observe

    def outcome(**values):
        return {**values, "controller_trace": trace}

    try:
        for i in range(start, len(cmds)):
            line, label = cmds[i]   # 그 자리 항목을 보낼 때 읽음 (끝점 물림을 바꾸면 남은 항목이 교체됨)
            if stop():
                raise StopRequested()
            # 수직 후퇴 이전에 마지막 그리기 위치에 실제로 도착해야 한다.
            if (contact_line and hasattr(link, "wait_until_pose") and label.startswith("stroke ")
                    and (label.endswith(" lift") or (label.endswith(" pen-up") and
                         (i == start or not cmds[i-1][1].endswith((" lift", " lift-out")))))):
                context.update(command="stroke endpoint", target=contact_line)
                endpoint = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", contact_line).groups())
                link.wait_until_pose(endpoint, cfg["idle_timeout_s"], should_stop=stop)
            if should_stop:
                link.send_and_ack(line, cfg["ack_timeout_s"], should_stop=stop)
            else:   # 예전 방식의 연결 객체(테스트의 가짜 시리얼 등)와 호환
                link.send_and_ack(line, cfg["ack_timeout_s"])
            sent = i + 1
            if on_ack:
                on_ack(sent, len(cmds))
            if label.endswith("pen-down") or label.endswith("draw"):
                contact_line = line
            # 후퇴·시작점 이동·접근 구간이 각각 끝난 뒤에 다음 구간을 보낸다.
            if (hasattr(link, "wait_until_pose") and
                    (label == "center pen-up" or
                     (label.startswith("stroke ") and
                      (label.endswith(" travel") or label.endswith(" descend"))))):
                context.update(command=label, target=line)
                target = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
                link.wait_until_pose(target, cfg["idle_timeout_s"], should_stop=stop)
            if label.startswith("stroke ") and label.endswith("pen-down") and hasattr(link, "wait_until_pose"):
                context.update(command=label, target=line)
                target = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
                link.wait_until_pose(target, cfg["idle_timeout_s"], should_stop=stop)
            if label.endswith("pen-down") or i == len(cmds) - 1:
                progress(f"  [{sent}/{len(cmds)}] {label}")
            # 고정 대기는 없다. 이번 접촉에서 종이 반대쪽으로 움직인 보고 직후에 다음 위치로 간다.
            if label.startswith("stroke ") and label.endswith("pen-up") and contact_line and hasattr(link, "wait_until_pose"):
                context.update(command=label, target=line)
                target = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
                link.wait_until_pose(target, cfg["idle_timeout_s"], should_stop=stop)
        link.wait_idle(cfg["idle_timeout_s"])
        return outcome(result="completed", commands_sent=sent, elapsed_s=round(time.monotonic() - started, 1))
    except StopRequested:
        return outcome(result="stopped_by_user", commands_sent=sent, elapsed_s=round(time.monotonic() - started, 1))
    except ControllerError as e:
        return outcome(result="stopped_on_error", error=str(e), commands_sent=sent,
                       failed_command=cmds[sent][1] if sent < len(cmds) else None,
                       elapsed_s=round(time.monotonic() - started, 1))
    except OSError as e:     # USB 시리얼이 끊김 (serial.SerialException도 OSError): 실행 기록·카메라 기록을 남기도록 결과로 돌려줌
        return outcome(result="stopped_on_error", error=f"로봇과 USB 통신이 끊겼습니다: {e}", commands_sent=sent,
                       failed_command=cmds[sent][1] if sent < len(cmds) else None,
                       elapsed_s=round(time.monotonic() - started, 1))
    except KeyboardInterrupt:
        return outcome(result="interrupted_by_user", commands_sent=sent, elapsed_s=round(time.monotonic() - started, 1))
    finally:
        if hasattr(link, "status_observer"):
            link.status_observer = previous_observer


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("strokes_json", type=Path)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--execute", action="store_true", help="실제 로봇으로 전송 (없으면 dry-run)")
    ap.add_argument("--gcode-out", type=Path, help="dry-run G-code 저장 경로 (기본: 입력 옆 .gcode)")
    ap.add_argument("--verbose", action="store_true", help="컨트롤러 응답을 모두 출력")
    ap.add_argument("--air", action="store_true", help="펜을 대지 않고 펜업 높이로 경로만 따라감")
    ap.add_argument("--pending-limits", action="store_true",
                    help="실물 미확인 확장 범위(limits_pending_verification)로 검사 — 범위 확인 시험 전용")
    ap.add_argument("--calibrate", action="store_true",
                    help="그리기 전에 종이 캘리브레이션 실행 (종이·펜 위치를 바꿨을 때; 없으면 설정에 저장된 값으로 바로 그림)")
    ap.add_argument("--virtual", action="store_true", help="로봇 없이 가상 시뮬레이션으로 실행 (--execute와 함께)")
    ap.add_argument("--virtual-speed", type=float, default=20.0, help="가상 시뮬레이션 배속")
    args = ap.parse_args()

    cfg = load_config(args.config)
    doc, strokes = load_strokes(args.strokes_json)
    if not strokes:
        print("그릴 획이 없습니다.")
        return 1

    if args.pending_limits:
        from .limits import pending_region
        pl, r = cfg["limits_pending_verification"], pending_region(cfg)
        print(f"주의: 실물 미확인 넓은 범위(좌우 ±{r.x_max:.0f}, 아래 {r.y_min:.0f} ~ 위 최대 {r.top(0):.0f}mm)로 검사합니다 "
              f"({pl.get('status', '')}). 범위 확인 시험에만 쓰세요.")
    try:
        pre = preflight(strokes, cfg, args.pending_limits, args.air)
    except DrawError as e:
        print(f"거부: {e.message}.")
        print("CV 단계에서 --box를 줄이거나 drawing_config.json의 limits를 확인하세요.")
        return 2
    planner, cmds, timing = pre["planner"], pre["cmds"], pre["timing"]
    if args.air:
        print(f"공중 모드(--air): 펜을 종이에 대지 않습니다 (접촉면에서 {cfg['pen']['up_clearance_mm']} mm 떨어져 이동).")
    ys = [planner.robot_yz(x, y)[0] for s in strokes for x, y in s]
    zs = [planner.robot_yz(x, y)[1] for s in strokes for x, y in s]

    print(f"입력: {args.strokes_json}")
    print(f"획 {len(strokes)}개, G-code {len(cmds)}줄")
    print(f"펜다운 {timing['pen_down_mm']:.0f} mm, 펜업 이동 {timing['pen_up_mm']:.0f} mm")
    print(f"예상 시간: {format_time(timing)}")
    print(f"로봇 Y 범위 {min(ys):.1f} ~ {max(ys):.1f}, Z 범위 {min(zs):.1f} ~ {max(zs):.1f} mm")
    if cfg.get("_axis_sign_status") != "verified":
        print("주의: 좌우 부호(paper_x_to_robot_y_sign)가 아직 미검증입니다. orientation-test-F로 먼저 확인하세요.")
    if cfg["plane_compensation"]["status"] != "verified":
        print("주의: 종이 평면 보정이 아직 없습니다. 위치에 따라 펜 접촉이 달라질 수 있습니다.")

    if not args.execute:
        out = args.gcode_out or args.strokes_json.with_suffix(".gcode")
        with open(out, "w", encoding="ascii") as f:
            for line, label in cmds:
                f.write(f"{line} ; {label}\n")
        print(f"dry-run: 로봇을 움직이지 않았습니다. G-code 저장: {out}")
        return 0

    if args.virtual:
        print(f"가상 시뮬레이션 {args.virtual_speed:g}배속: 로봇 없이 같은 절차로 실행합니다.")
    try:
        link = open_link(cfg, args.virtual, args.virtual_speed, args.verbose)
    except DrawError as e:
        print(e.message)
        return 3
    try:
        try:
            from . import calibration
            if args.calibrate and not args.virtual:
                print("--calibrate: 종이 캘리브레이션을 시작합니다.")
                try:
                    measured = calibration.run_calibration(link, cfg)
                except (calibration.CalibrationError, ValueError, KeyboardInterrupt) as e:
                    record = {"result": "aborted", "reason": str(e), "samples": []}
                    print(f"캘리브레이션 실패 기록: {calibration.save_report(record, args.config or paths.config_path())}")
                    print(f"{e}. 설정은 갱신하지 않았습니다.")
                    return 4
                calibration_path = calibration.save_report(measured, args.config or paths.config_path())
                if measured.get("result") != "ready":
                    print(f"캘리브레이션을 완료하지 못했습니다. 설정은 유지했습니다. 기록: {calibration_path}")
                    return 4
                cfg = calibration.apply_calibration(
                    cfg, measured["center_tcp_mm"], measured["plane_fit"],
                    measured["selected_half_size_mm"], measured["max_residual_mm"])
                calibration.save_config(args.config or paths.config_path(), cfg)
                pre = preflight(strokes, cfg, args.pending_limits, args.air)
                planner, cmds, timing = pre["planner"], pre["cmds"], pre["timing"]
                tcp = tuple(measured["ready_tcp_mm"][axis] for axis in ("x", "y", "z"))
                print(f"캘리브레이션 완료 ({2*measured['selected_half_size_mm']:g}×"
                      f"{2*measured['selected_half_size_mm']:g} mm). 기록: {calibration_path}")
            else:
                tcp = connect_and_home(link, cfg)
            print(f"컨트롤러 상태: Idle, TCP: {tcp}")
            check_start(tcp, cfg, retracted=bool(args.calibrate and not args.virtual))
        except DrawError as e:
            print(f"{e.message} 중단합니다." + (f" ({e.hint})" if e.hint else ""))
            return 3
        except (calibration.CalibrationError, ValueError, OSError) as e:
            print(f"캘리브레이션/설정 오류로 중단했습니다: {e}. 자동 후퇴 없이 기존 설정을 유지합니다.")
            return 3
        if input("종이·펜·주변을 확인했으면 yes 입력: ").strip().lower() != "yes":
            print("취소했습니다.")
            return 0
        result = execute(link, cmds, cfg)
    finally:
        link.close()

    print(f"결과: {result}")
    if result["result"] != "completed":
        print("자동 복구를 하지 않았습니다. 펜과 로봇 상태를 확인한 뒤 수동으로 조치하세요.")
    path = write_run_record({"strokes_json": str(args.strokes_json), "stroke_count": len(strokes),
                             "command_count": len(cmds), "air_mode": args.air, "pending_limits": args.pending_limits,
                             "estimated_time": timing, "source": doc.get("source", {}),
                             "virtual": bool(args.virtual),
                             "virtual_speed": args.virtual_speed if args.virtual else None}, result, cfg)
    print(f"실행 기록: {path} (종이 사진 확인 결과를 visual_verification에 적어두세요)")
    return 0 if result["result"] == "completed" else 4


if __name__ == "__main__":
    sys.exit(main())
