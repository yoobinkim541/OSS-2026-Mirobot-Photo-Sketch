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
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from . import paths

DEFAULT_CONFIG = None  # None이면 paths.config_path() (저장소 / 사용자 폴더 / MIROBOT_CONFIG)

ERROR_TOKENS = ("alarm", "error", "limit", "emergency")
CARTESIAN_RE = re.compile(r"Cartesian coordinate\(XYZ RxRyRz\):\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)")


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


def check_limits(strokes, cfg, pending=False):
    from .limits import Region
    region = Region.from_cfg(active_limits(cfg, pending))
    return [(i, x, y) for i, s in enumerate(strokes) for x, y in s if not region.contains(x, y)]


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
        # 공중 경로는 미측정 영역도 지날 수 있으므로 실물 탐색에서 쓴 여유(8 mm)를 유지한다.
        clearance = max(float(pen["up_clearance_mm"]), float(pen.get("air_clearance_mm", 8.0))) if air else float(pen["up_clearance_mm"])
        self.up_dx = pen["retract_x_sign"] * clearance
        self.down_extra = -pen["retract_x_sign"] * pen["down_extra_mm"]  # 누르는 방향 = 후퇴의 반대
        pc = cfg["plane_compensation"]
        self.pa, self.pb = pc["a_per_mm_y"], pc["b_per_mm_z"]
        self.feeds = cfg["feeds_mm_per_min"]

    def robot_yz(self, px, py):
        return self.cy + self.sy * px, self.cz + self.sz * py

    def contact_x(self, ry, rz):
        return self.cx + self.pa * (ry - self.cy) + self.pb * (rz - self.cz) + self.down_extra

    def pose(self, px, py, pen_down):
        ry, rz = self.robot_yz(px, py)
        x = self.contact_x(ry, rz)
        if not pen_down or self.air:
            x += self.up_dx
        return x, ry, rz

    def gcode(self, pose, feed):
        x, y, z = pose
        a, b, c = self.abc
        return f"M20 G90 G01 X{x:.3f} Y{y:.3f} Z{z:.3f} A{a:.3f} B{b:.3f} C{c:.3f} F{feed:.0f}"

    def plan(self, strokes):
        """[(gcode, 설명)] 리스트. 종이 중심 펜업에서 시작해 종이 중심 펜업으로 끝납니다."""
        f = self.feeds
        cmds = [(self.gcode(self.pose(0, 0, False), f["approach"]), "center pen-up")]
        for i, s in enumerate(strokes):
            x0, y0 = s[0]
            cmds.append((self.gcode(self.pose(x0, y0, False), f["travel"]), f"stroke {i} travel"))
            cmds.append((self.gcode(self.pose(x0, y0, True), f["approach"]), f"stroke {i} pen-down"))
            for x, y in s[1:]:
                cmds.append((self.gcode(self.pose(x, y, True), f["draw"]), f"stroke {i} draw"))
            x1, y1 = s[-1]
            cmds.append((self.gcode(self.pose(x1, y1, False), f["approach"]), f"stroke {i} pen-up"))
        cmds.append((self.gcode(self.pose(0, 0, False), f["travel"]), "return center pen-up"))
        return cmds


def estimate_time(strokes, cfg, air=False):
    """그리기 시간 추정. CV(make_strokes, GUI)와 실행기가 같은 함수를 씁니다.

    반환 dict:
      draw_s       펜다운 이동 (길이 / draw 속도)
      travel_s     펜업 이동 (획 사이 거리 / travel 속도, 종이 중심 출발·복귀 포함)
      pen_lift_s   일반: 획마다 펜 내림+올림. 공중: 시작점에서 후퇴하는 시간
      command_count, latency_s  명령마다 ok 응답을 기다리는 지연 (timing 설정의 가정값)
      motion_s     = draw + travel + pen_lift (가감속 제외 이론값)
      total_s      = motion + latency
    """
    f = cfg["feeds_mm_per_min"]
    clear = cfg["pen"]["up_clearance_mm"]
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
    lift_mm = max(clear, float(cfg["pen"].get("air_clearance_mm", 8.0))) if air else clear * (2 * n + 1)
    pen_lift_s = lift_mm / f["approach"] * 60.0
    command_count = 2 + sum(len(s) + 2 for s in strokes)  # Planner.plan과 같은 수
    latency = cfg.get("timing", {}).get("assumed_command_latency_s", 0.0)
    motion_s = draw_s + travel_s + pen_lift_s
    return {
        "pen_down_mm": round(down, 1), "pen_up_mm": round(up, 1), "stroke_count": n,
        "draw_s": round(draw_s, 1), "travel_s": round(travel_s, 1), "pen_lift_s": round(pen_lift_s, 1),
        "command_count": command_count, "latency_s": round(command_count * latency, 1),
        "motion_s": round(motion_s, 1), "total_s": round(motion_s + command_count * latency, 1),
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
            return "unknown", None, buf
        body = buf[buf.index("<") + 1:]
        state = body.split(",")[0]
        m = CARTESIAN_RE.search(buf)
        tcp = tuple(float(v) for v in m.groups()) if m else None
        return state, tcp, buf

    def wait_idle(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state, tcp, raw = self.query_status()
            if state == "Idle":
                return tcp
            if any(t in raw.lower() for t in ERROR_TOKENS):
                raise ControllerError(f"Idle 대기 중 오류 상태: {raw}")
            time.sleep(0.5)
        raise ControllerError("Idle 대기 시간 초과")

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
        return MirobotLink.open(cfg["port"], cfg["baud"], verbose)
    except Exception as e:  # 포트 없음·사용 중 (serial.SerialException 등)
        raise DrawError("connect", f"포트 {cfg['port']}를 열 수 없습니다: {e}",
                        "USB 연결과 포트 번호(drawing_config.json의 port)를 확인하고, "
                        "다른 프로그램이 쓰고 있지 않은지 보세요.") from None


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


def check_start(tcp, cfg):
    """③ 현재 펜 끝이 설정된 종이 중심 근처인지. 반환: 떨어진 거리(mm)."""
    c = cfg["paper_center_tcp_mm"]
    off = ((tcp[0] - c["x"]) ** 2 + (tcp[1] - c["y"]) ** 2 + (tcp[2] - c["z"]) ** 2) ** 0.5
    if off > cfg["max_start_offset_mm"]:
        raise DrawError("start", f"펜 끝이 종이 중심 설정에서 {off:.1f} mm 떨어져 있습니다 "
                                 f"(허용 {cfg['max_start_offset_mm']} mm).",
                        "호밍 뒤 펜 끝이 종이 가운데에 오도록 종이 위치를 맞추세요.")
    return off


def write_run_record(meta, result, cfg):
    """⑥ runs/run-<시각>.json. meta: strokes_json, stroke_count, command_count, air_mode, pending_limits,
    estimated_time, source, virtual, virtual_speed 등."""
    run_dir = paths.runs_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run = {"started_local": stamp, **meta, "config_snapshot": cfg, **result, "visual_verification": "pending"}
    path = run_dir / f"run-{stamp}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(run, f, ensure_ascii=False, indent=1)
    return path


def execute(link, cmds, cfg, progress=print, on_ack=None, should_stop=None):
    """계획된 명령을 순서대로 보냅니다. 반환: 실행 결과 dict. 오류·멈춤 시 즉시 멈춥니다(자동 복구 없음).
    on_ack(보낸 수, 전체 수): ok마다 호출 (GUI 진행 표시·RViz 따라가기).
    should_stop(): True면 다음 명령부터 보내지 않음 (가상 시뮬레이션은 기다리는 중에도 바로 멈춤)."""
    started = time.monotonic()
    sent = 0
    stop = should_stop or (lambda: False)
    try:
        for i, (line, label) in enumerate(cmds):
            if stop():
                raise StopRequested()
            if should_stop:
                link.send_and_ack(line, cfg["ack_timeout_s"], should_stop=stop)
            else:   # 예전 방식의 연결 객체(테스트의 가짜 시리얼 등)와 호환
                link.send_and_ack(line, cfg["ack_timeout_s"])
            sent = i + 1
            if on_ack:
                on_ack(sent, len(cmds))
            if label.endswith("pen-down") or i == len(cmds) - 1:
                progress(f"  [{sent}/{len(cmds)}] {label}")
        link.wait_idle(cfg["idle_timeout_s"])
        return {"result": "completed", "commands_sent": sent, "elapsed_s": round(time.monotonic() - started, 1)}
    except StopRequested:
        return {"result": "stopped_by_user", "commands_sent": sent,
                "elapsed_s": round(time.monotonic() - started, 1)}
    except ControllerError as e:
        return {"result": "stopped_on_error", "error": str(e), "commands_sent": sent,
                "failed_command": cmds[sent][1] if sent < len(cmds) else None,
                "elapsed_s": round(time.monotonic() - started, 1)}
    except KeyboardInterrupt:
        return {"result": "interrupted_by_user", "commands_sent": sent,
                "elapsed_s": round(time.monotonic() - started, 1)}


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
            check_start(tcp, cfg)
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
