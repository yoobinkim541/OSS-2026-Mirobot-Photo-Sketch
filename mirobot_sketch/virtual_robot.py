"""
가상 시뮬레이션 — 로봇 없이 실행 흐름(호밍 → 명령·ok → 멈춤·오류)을 그대로 시험
===================================================================================
MirobotLink와 같은 메서드를 가집니다. 명령마다 실제로 걸릴 시간(거리 / 속도 + 명령 지연 가정값)만큼
기다렸다가 ok를 돌려주며, speed(배속)로 줄일 수 있습니다. 오류 상황도 흉내 냅니다.
"""

import math
import re
import time

from .draw_executor import ControllerError, StopRequested

_XYZ = re.compile(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)")
_F = re.compile(r"F([\d.]+)")
_JOINTS = re.compile(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+) A([-\d.]+) B([-\d.]+) C([-\d.]+)")
POLL_S = 0.02     # 기다리는 동안 멈춤 확인 간격


class VirtualMirobotLink:
    virtual = True

    def __init__(self, cfg, speed=1.0, fail_at=None, timeout_at=None, start_offset_mm=0.0,
                 homing_fails=False, homing_s=1.0):
        self.cfg, self.speed = cfg, max(float(speed), 1e-6)
        self.fail_at, self.timeout_at = fail_at, timeout_at
        self.homing_fails, self.homing_s = homing_fails, homing_s
        c = cfg["paper_center_tcp_mm"]
        self.pos = (c["x"], c["y"], c["z"] + float(start_offset_mm))
        self.state = "Alarm"
        self.joints = [0.0] * 6     # 관절 이동(M21) 기준. 직교 이동은 이 값을 바꾸지 않음
        self.sent = []
        self.latency = cfg.get("timing", {}).get("assumed_command_latency_s", 0.1)

    def _sleep(self, seconds, should_stop=None):
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return
            if should_stop and should_stop():
                raise StopRequested()
            time.sleep(min(POLL_S, left))

    def wait_for_homing(self, timeout, progress=print, should_cancel=None):
        progress("  컨트롤러 상태: Alarm (가상)")
        progress("  -> 가상 시뮬레이션: 잠시 뒤 자동으로 호밍됩니다.")
        deadline = time.monotonic() + timeout
        ready = time.monotonic() + self.homing_s
        while time.monotonic() < deadline:
            if should_cancel and should_cancel():
                return "cancelled", None
            if time.monotonic() >= ready and not self.homing_fails:
                self.state = "Idle"
                progress("  컨트롤러 상태: Idle (가상)")
                return "Idle", self.pos
            time.sleep(POLL_S)
        return self.state, None

    def query_status(self, timeout=2.0):
        j, (x, y, z) = self.joints, self.pos     # 실제 컨트롤러와 같은 형식 (Angle은 J4,J5,J6,D,J1,J2,J3 순서)
        raw = (f"<{self.state},Angle(ABCDXYZ):{j[3]:.3f},{j[4]:.3f},{j[5]:.3f},0.000,{j[0]:.3f},{j[1]:.3f},{j[2]:.3f},"
               f"Cartesian coordinate(XYZ RxRyRz):{x:.3f},{y:.3f},{z:.3f},0.000,0.000,0.000,virtual>")
        return self.state, self.pos, raw

    def send_and_ack(self, line, timeout, should_stop=None):
        self.sent.append(line)
        n = len(self.sent)
        if self.fail_at == n:
            raise ControllerError("컨트롤러 오류 응답: Alarm (가상 시뮬레이션 오류 흉내)")
        if self.timeout_at == n:
            self._sleep(timeout, should_stop)
            raise ControllerError(f"ok 응답 시간 초과: {line}")
        m, f = _XYZ.search(line), _F.search(line)
        if m and line.startswith("M21"):     # 관절 이동: X~C = J1~J6 각도(°). 걸리는 시간은 가장 많이 도는 관절 기준
            q = [float(v) for v in _JOINTS.search(line).groups()]
            turn = max(abs(a - b) for a, b in zip(q, self.joints))
            feed = float(f.group(1)) if f else 1000.0
            self._sleep((turn / (feed / 60.0) + self.latency) / self.speed, should_stop)
            from .mirobot_sim import fk
            self.joints = q
            self.pos = tuple(float(v) for v in fk([math.radians(v) for v in q])[0][:3, 3])
        elif m:
            target = tuple(float(v) for v in m.groups())
            dist = sum((a - b) ** 2 for a, b in zip(target, self.pos)) ** 0.5
            feed = float(f.group(1)) if f else 1000.0
            self._sleep((dist / (feed / 60.0) + self.latency) / self.speed, should_stop)
            self.pos = target

    def wait_idle(self, timeout):
        return self.pos

    def close(self):
        self.state = "closed"
