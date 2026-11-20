"""
로봇으로 그리기 작업 — 안전 절차 ①~⑥을 작업 스레드에서 진행 (화면과 무관, GUI 실행 창이 events로 표시)
======================================================================================================
①사전 검사 → ②연결·호밍 → ③시작 위치 → ④최종 확인(사람이 confirm 할 때까지 대기) → ⑤그리는 중 → ⑥끝.
ok 응답마다 진행 파일을 써서 RViz 따라가기가 같은 위치를 보여 줍니다. 자동 복구는 하지 않습니다.

events(kind, **data):
  step      id, status(active|done|failed), message, hint
  progress  acked, total, stroke, strokes, drawn, elapsed_s, remaining_s   (drawn: plan의 앞에서부터 다 그린 획 수)
  plan      strokes (그리는 순서의 획 mm 목록, ⑤ 시작 때 한 번)
  rviz      ok, message
  paused    acked, total   (⑤에서 [멈춤]: 펜을 뗀 뒤 일시정지. job.resume()으로 이어 그리거나 job.finish()로 끝냄)
  resumed   (이어서를 골라 멈춘 자리부터 다시 그림)
  finished  result, record_path
"""

import bisect
import copy
import json
import math
import queue
import threading
import time
from datetime import datetime

from . import draw_executor as de, robot_status
from . import live_progress as lp
from . import paths

CANCELLED = "취소했습니다."
CALIBRATION_RECT_MM = (-60.0, -65.0, 60.0, 55.0)
PROGRESS_SETTLE_S = 60.0   # 이만큼 그린 뒤부터 남은 시간을 실제 속도로 고쳐 셈 (처음 몇 명령은 빨라 남은 시간을 반으로 셌음)


def _floor_tenth(v):
    return math.floor(v * 10 + 1e-9) / 10


def _ceil_tenth(v):
    return math.ceil(v * 10 - 1e-9) / 10


def target_contact_rect(strokes, cfg):
    """확장 그림을 둘러싼 접촉 직사각형 (x0, y0, x1, y1), 종이 중심 기준 mm. 좌우는 대칭.

    위쪽 한계 때문에 큰 그림은 종이 중심보다 아래로 배치되므로 세로 범위는 그림의 실제 범위를 따른다.
    가장자리 여유는 1 mm를 먼저 시도하고 허용 영역 밖이면 0.5 mm, 그래도 밖이면 그림 범위 그대로(여유 없음).
    그것도 영역 밖이면 None."""
    from . import limits

    region = limits.pending_region(cfg)
    xs = [x for st in strokes for x, _ in st]
    ys = [y for st in strokes for _, y in st]
    half_x = max(abs(min(xs)), abs(max(xs)))
    for margin in (1.0, 0.5):
        hx = _ceil_tenth(half_x + margin)
        y0, y1 = _floor_tenth(min(ys) - margin), _ceil_tenth(max(ys) + margin)
        if all(region.contains(x, y) for x in (-hx, hx) for y in (y0, y1)):
            return (float(-hx), float(y0), float(hx), float(y1))
    # 여유 없이 그림 범위 그대로 (앱이 위쪽 한계에 딱 맞춰 배치한 큰 그림은 반올림만으로도 영역을 넘음)
    exact = tuple(float(v) for v in (-half_x, min(ys), half_x, max(ys)))
    if all(region.contains(x, y) for x in (exact[0], exact[2]) for y in (exact[1], exact[3])):
        return exact
    return None


class DrawJob:
    def __init__(self, session, cfg, events, progress_path=None, launch_rviz=None):
        self.session, self.cfg, self.events = session, cfg, events
        self.progress_path = progress_path or (paths.output_dir() / "live_progress.json")
        self.launch_rviz = launch_rviz
        self.state = "idle"
        self.summary = {}
        self.link = None
        self._traj_path = None
        self._snap = None
        self._air_sim = None
        self._air_verdict = None
        self._air_thread = None
        self._air_error = None
        self._air_cancel = threading.Event()      # 작업이 끝나면 백그라운드 시뮬레이션도 멈춤
        self._return_origin = True
        self._confirm = threading.Event()
        self._stop = threading.Event()
        self._stop_presses = 0            # [멈춤]을 누른 횟수: 펜을 떼는 중·호밍 자세로 돌아가는 중에 한 번 더 누르면 그 자리에서 멈춤
        self._decision = queue.Queue()    # ⑤ 일시정지 중 사람이 고른 것: resume(이어서) / finish(끝내기) / end(그 자리에서 끝)
        self._end_requested = False       # 창을 닫음: 일시정지에서 기다리지 않고 그 자리에서 끝냄
        self._choice = {}
        self._thread = None
        self._calibration_reply = None
        self._calibration_reply_lock = threading.Lock()
        self._start_pose = False
        self.start_pose_setup = None      # ③ 시작 자세 맞추기 중일 때만 (화면이 jog·accept를 보냄)
        self._live = None                 # ⑤ 그리는 중: 보내는 명령 목록·획 (끝점 물림을 남은 명령에 적용할 때)

    # ---------------------------------------------------------------- 사람이 누르는 것
    def start(self, virtual=True, virtual_speed=20.0, pending=False, recalibrate=False, return_to_origin=True,
              start_pose=False):
        """pending: 실물 확인 전 넓은 범위(limits_pending_verification) 허용 — ① 사전 검사부터 적용 (④에서 바꿀 수도 있음).
        return_to_origin: 그림이 끝까지 그려지면 펜을 처음 상태로 되돌림 (공중 모드는 해당 없음). 저장된 대기 자세로 시작했으면
        (start_pose와 requested_start_pose) 호밍 자세(관절 0)로 돌아가고, [멈춤]을 눌렀을 때도 펜을 뗀 뒤 같은 곳으로 돌아감.
        start_pose: ②호밍 뒤 ③에서 저장된 시작 자세로 자동 이동하고, 사람이 조그로 맞춘 뒤 [기억하고 계속]을 누르면
        그 자세를 저장하고 이번 실행의 종이 중심으로 씀 (start_pose.StartPoseSetup, 다시 보정을 고르면 쓰지 않음)."""
        self._virtual, self._speed, self._pending = bool(virtual), float(virtual_speed), bool(pending)
        self._recalibrate = bool(recalibrate)
        self._return_origin = bool(return_to_origin)
        self._start_pose = bool(start_pose)
        self.state = "preflight"      # 첫 이벤트 전에 창을 닫아도 '진행 중'으로 보이게 (스레드 시작 전에)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def confirm(self, air=True, pending=False, checked=False):
        if not checked:
            raise ValueError("종이·펜·주변 확인 체크가 필요합니다")
        if self.state != "confirm":
            raise ValueError("최종 확인 단계가 아닙니다")
        self._choice = {"air": bool(air), "pending": bool(pending)}
        self._confirm.set()

    def stop(self):
        """⑤에서는 다음 명령부터 보내지 않고 펜을 종이에서 떼어 일시정지 (resume으로 이어 그리거나 finish로 끝냄).
        떼는 중이나 끝내고 돌아가는 중에 한 번 더 누르면 그 자리에서 멈춤. 일시정지 중에 누르면 그 자리에서 끝냄
        (창을 닫을 때). 그 전 단계에서는 취소."""
        if self.state == "paused":
            self._decision.put("end")
        self._stop_presses += 1
        self._stop.set()
        self._confirm.set()
        with self._calibration_reply_lock:
            reply = self._calibration_reply
        if reply is not None:
            try:
                reply.put_nowait("q")
            except queue.Full:
                pass

    cancel = stop

    def resume(self):
        """⑤ 일시정지 중일 때만: 멈춘 자리부터 이어 그림."""
        if self.state == "paused":
            self._decision.put("resume")

    def finish(self):
        """⑤ 일시정지 중일 때만: 그리기를 끝냄. 저장된 대기 자세로 시작했고 '처음 위치로 되돌리기'를 켰으면 호밍 자세로 돌아감."""
        if self.state == "paused":
            self._decision.put("finish")

    def end(self):
        """창을 닫을 때: 어느 단계든 그 자리에서 끝냄.
        그리는 중이면 펜을 떼지 않고 바로 끝낸다. 이미 일시정지면 기다리지 않고 끝냄."""
        self._end_requested = True
        self._decision.put("end")
        self.stop()

    def jog(self, dx, dy, dz):
        """③ 시작 자세 맞추기 중일 때만: 로봇 좌표로 (dx, dy, dz) mm 움직이기 요청 (바쁘면 버려짐)."""
        setup = self.start_pose_setup
        if setup is not None:
            setup.jog(dx, dy, dz)

    def accept_start_pose(self):
        """③ 시작 자세 맞추기: 지금 자세를 검사해 기억하고 다음 단계로."""
        setup = self.start_pose_setup
        if setup is not None:
            setup.accept()

    def set_pen_correction(self, values):
        """⑤ 그리는 중: 아직 보내지 않은 명령의 끝점 물림(pen.start/end_backoff_mm)과, values에 "rise"가 있으면
        비스듬히 대고 떼기(pen.edge_rise_mm, 켜져 있을 때만 바뀜)를 바꿈. 바뀌는 것은 접촉 자세의 X(종이에서 0~1 mm 뒤)와
        종이에서 떨어진 중간점의 위·아래뿐이라 명령 수·순서는 그대로입니다.
        반환: 시작점부터 새 값으로 그리는 첫 획 번호. 그리는 중이 아니거나 공중 모드면 None."""
        live = self._live
        if live is None or live["air"] or self.state != "drawing":
            return None
        cfg = copy.deepcopy(self.cfg)
        cfg["pen"]["start_backoff_mm"], cfg["pen"]["end_backoff_mm"] = values["start"], values["end"]
        if "rise" in values:
            cfg["pen"]["edge_rise_mm"] = values["rise"]
        if "rise_low" in values:
            cfg["pen"]["edge_rise_low_mm"] = values["rise_low"]
        if "land" in values:
            cfg["pen"]["edge_land_mm"] = values["land"]
        new = de.Planner(cfg).plan(live["strokes"])
        cmds = live["cmds"]
        if len(new) != len(cmds):
            return None
        first = live["acked"] + 2          # 실행기가 이미 꺼냈을 수 있는 바로 다음 명령은 그대로 둠
        for i in range(first, len(cmds)):
            if new[i] != cmds[i]:
                cmds[i] = new[i]           # 목록 항목 교체는 원자적: 실행기는 보낼 때 그 자리 항목을 읽음
        self.cfg["pen"].update(cfg["pen"])
        applied = next((int(cmds[i][1].split(" ")[1]) for i in range(first, len(cmds))
                        if cmds[i][1].endswith(" pen-down")), len(live["strokes"]))
        live["pen_changes"].append({"from_stroke": applied,
                                    "elapsed_s": round(time.monotonic() - live["started"], 1),
                                    **{k: cfg["pen"].get(k, 0.0) for k in live["pen_keys"]}})
        return applied

    def join(self, timeout=None):
        if self._thread:
            self._thread.join(timeout)

    def is_alive(self):
        return bool(self._thread and self._thread.is_alive())

    # ---------------------------------------------------------------- 작업 스레드
    def _step(self, sid, status, message="", hint=""):
        if status == "active":
            self.state = sid
        self.events("step", id=sid, status=status, message=message, hint=hint)

    def timing_for(self, air):
        """확인 화면에 보여줄 예상 시간. 공중 체크와 같은 air로 다시 계산한다."""
        strokes = (self._snap or {}).get("strokes") or []
        return de.estimate_time(strokes, self.cfg, air=bool(air))

    def answer_calibration_input(self, value):
        """GUI에서 받은 사람 응답을 캘리브레이션 작업 스레드에 전달."""
        with self._calibration_reply_lock:
            reply = self._calibration_reply
        if reply is not None:
            try:
                reply.put_nowait("q" if value is None else str(value))
            except queue.Full:
                pass

    def _use_start_pose_as_paper_center(self, tcp):
        """실물 연결(보정 안 함)의 ③: 사람이 펜 끝을 종이 가운데에 닿게 세팅하고 시작하므로, 시작 자세가 곧 종이 중심.
        설정 파일 값은 바꾸지 않고 이번 실행에서만 종이 중심으로 쓴다(실행 기록의 설정 사본에 남음).
        반환: 설정 파일의 종이 중심과 시작 자세의 거리(mm)."""
        s = self.session
        previous = self.cfg["paper_center_tcp_mm"]
        moved = math.dist((tcp[0], tcp[1], tcp[2]), (previous["x"], previous["y"], previous["z"]))
        if moved < 1e-6:
            return 0.0
        updated = copy.deepcopy(self.cfg)
        updated["paper_center_tcp_mm"] = {"x": float(tcp[0]), "y": float(tcp[1]), "z": float(tcp[2])}
        for c in {id(self.cfg): self.cfg, id(s.cfg): s.cfg}.values():
            c.clear()
            c.update(updated)
        # 종이 중심이 바뀌면 로봇 좌표가 바뀌므로 관절 시뮬레이션을 다시 검사
        sim = s.simulate()
        if sim["verdict"].startswith("FAIL"):
            raise de.DrawError("start", f"시작 자세를 종이 중심으로 쓰면 로봇 시뮬레이션이 FAIL입니다: {sim['verdict']}",
                               "종이와 펜 위치를 확인하거나 그림 크기를 줄인 뒤 다시 시작하세요.")
        with s.lock:
            self._snap["sim_raw"] = s.sim["raw"]
        return moved

    def _calibration_input(self, prompt):
        reply = queue.Queue(maxsize=1)
        with self._calibration_reply_lock:
            self._calibration_reply = reply
        self.events("calibration_input", prompt=prompt)
        try:
            while not self._stop.is_set():
                try:
                    return reply.get(timeout=0.1)
                except queue.Empty:
                    continue
            return "q"
        finally:
            with self._calibration_reply_lock:
                if self._calibration_reply is reply:
                    self._calibration_reply = None

    def _run(self):
        result, record = {"result": "cancelled"}, None
        writer = lp.ProgressWriter(self.progress_path)
        run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        s = self.session
        controller = robot_status.ControllerStatusReporter(
            lambda **event: self.events("robot_status", **event))
        try:
            # ① 사전 검사
            self._step("preflight", "active", "범위와 로봇 시뮬레이션을 확인합니다")
            requested_mode = bool(self._start_pose and self.cfg.get("requested_start_pose"))
            if requested_mode:
                from . import start_pose as sp
                self.cfg = sp.drawing_config_from_recorded_contact(self.cfg)
                s.cfg = self.cfg
                s.sim = None
            if not self._virtual and self.cfg.get("requested_start_pose") and not requested_mode:
                raise de.DrawError("preflight", "요청한 관절 시작 자세는 아직 실행 경로와 시뮬레이션에 연결되지 않았습니다.",
                                   "시작 자세의 실제 관절 상태와 펜·종이 보정을 확인한 뒤 사용하세요.")
            if requested_mode and not self._virtual:
                if self._recalibrate:
                    raise de.DrawError("preflight", "120×120 mm 다시 보정은 저장된 대기·접촉 자세와 함께 쓸 수 없습니다.",
                                       "'다시 보정'을 끄거나 '호밍 뒤 시작 자세 맞추기'를 끄고 다시 시작하세요.")
                blocker = sp.requested_motion_blocker(self.cfg)
                if blocker:
                    raise de.DrawError("preflight", f"실물 검증 전이라 로봇을 움직이지 않았습니다. {blocker}",
                                       "펜 홀더 위치와 종이 접촉 간격을 실측해 설정을 맞춘 뒤 다시 시작하세요. "
                                       "같은 순서는 가상 시뮬레이션으로 확인할 수 있습니다.")
            if s.sim is None:
                s.simulate()
            # ①에서 그릴 획·시뮬레이션·경로를 한 번에 찍어 둠: 이후 세션이 바뀌어도(편집·이미지 열기)
            # 로봇이 그리는 것, 확인 요약, RViz 궤적, 실행 기록이 모두 같은 그림을 가리키게
            with s.lock:
                if s.result is None or s.sim is None:
                    raise de.DrawError("preflight", "처리 결과나 시뮬레이션이 없습니다.", "이미지를 다시 처리하세요.")
                snap = {"strokes": [[tuple(pt) for pt in st] for st in s.robot_strokes_mm()],   # 종이 맞춤 적용
                        "sim_raw": s.sim["raw"], "verdict": s.sim["summary"]["verdict"], "path": s.result["path"],
                        "params": dict(s.result["params"]), "placement": dict(s.result["placement"]),
                        "paper_fit": dict(s.paper_fit) if s.paper_fit else None}
            self._snap = snap
            verdict = snap["verdict"]
            if verdict.startswith("FAIL"):
                raise de.DrawError("preflight", f"로봇 시뮬레이션이 FAIL입니다: {verdict}",
                                   "그림 크기를 줄이거나 설정을 바꾼 뒤 다시 시뮬레이션하세요.")
            strokes = snap["strokes"]
            pre = de.preflight(strokes, self.cfg, pending=self._pending, air=True)
            pl = snap["placement"]
            self.summary = {"stroke_count": len(strokes), "command_count": len(pre["cmds"]),
                            "estimated_s": pre["timing"]["total_s"],
                            "drawing_mm": [pl["drawing_width_mm"], pl["drawing_height_mm"]],
                            "port": "가상 시뮬레이션" if self._virtual else "USB 자동 감지",
                            "virtual": self._virtual, "virtual_speed": self._speed,
                            "plane_verified": self.cfg.get("plane_compensation", {}).get("status") == "verified"}
            self._step("preflight", "done", f"획 {len(strokes)}개 · 명령 {len(pre['cmds'])}줄 · 시뮬레이션 {verdict.split(':')[0]}")
            # ② 연결·호밍
            self._step("connect", "active", "가상 시뮬레이션에 연결합니다" if self._virtual else
                       "로봇 가운데 버튼을 2초 눌러 호밍하세요. Idle이 되면 자동으로 넘어갑니다")
            if not self._virtual:
                controller.connection_started()
            self.link = de.open_link(self.cfg, self._virtual, self._speed)
            if not self._virtual:
                self.summary["port"] = self.link.port
            writer.write(run_id=run_id, state="homing", acked=0, total=len(pre["cmds"]),
                         speed=self._speed if self._virtual else 1.0, trajectory=None, message="호밍 대기")
            from . import calibration
            calibration_path = None
            calibrated = False
            # 한 번의 120×120 mm 영역 보정. 위쪽 도달 한계를 고려해 중심보다 5 mm 아래에 둔다.
            # run_calibration이 현재 확장 영역 안인지 확인한 후 이동한다.
            if self._recalibrate and not self._virtual:
                self._step("connect", "active", "종이 캘리브레이션을 시작합니다")
                def calibration_progress(message):
                    self.events("calibration_status", message=message)
                    if robot_status.controller_state_from_progress(message) is not None:
                        controller.progress(message)

                try:
                    measured = calibration.run_calibration(
                        self.link, self.cfg, input_fn=self._calibration_input,
                        output_fn=calibration_progress, target_rect_mm=CALIBRATION_RECT_MM)
                except (calibration.CalibrationError, ValueError) as e:
                    calibration_path = calibration.save_report(
                        {"result": "aborted", "reason": str(e), "samples": []}, paths.config_path())
                    raise de.DrawError("connect", f"캘리브레이션을 완료하지 못했습니다: {e}",
                                       "종이 위치와 로봇 주변을 확인하고 다시 시도하세요.") from e
                calibration_path = calibration.save_report(measured, paths.config_path())
                if measured.get("result") != "ready":
                    raise de.DrawError("connect", "캘리브레이션이 취소되었거나 평면 편차가 허용값을 넘었습니다.",
                                       "종이를 다시 평평하게 고정한 뒤 실제 로봇 연결로 다시 시작하세요.")
                updated_cfg = calibration.apply_calibration(
                    self.cfg, measured["center_tcp_mm"], measured["plane_fit"],
                    measured["selected_half_size_mm"], measured["max_residual_mm"])
                calibration.save_config(paths.config_path(), updated_cfg)
                self.cfg.clear()
                self.cfg.update(updated_cfg)
                # 측정된 접촉면과 한계로 경로/관절 시뮬레이션을 다시 검사한 뒤 계속합니다.
                pre = de.preflight(strokes, self.cfg, pending=self._pending, air=True)
                sim_summary = s.simulate()
                if sim_summary["verdict"].startswith("FAIL"):
                    raise de.DrawError("connect", f"보정된 경로의 로봇 시뮬레이션이 FAIL입니다: {sim_summary['verdict']}",
                                       "그림 크기를 줄인 뒤 다시 시뮬레이션하세요.")
                with s.lock:
                    self._snap["sim_raw"] = s.sim["raw"]
                tcp = tuple(measured["ready_tcp_mm"][axis] for axis in ("x", "y", "z"))
                calibrated = True
                self.summary.update(command_count=len(pre["cmds"]), estimated_s=pre["timing"]["total_s"])
            else:
                tcp = de.connect_and_home(self.link, self.cfg,
                                          progress=controller.progress if not self._virtual else lambda *_: None,
                                          should_cancel=self._stop.is_set)
            self._step("connect", "done", f"캘리브레이션 완료 · {calibration_path.name}"
                       if calibration_path is not None and measured.get("result") == "ready" else "Idle")
            if not self._virtual:
                read = de.require_controller_settings(self.link, self.cfg)
                self.summary["controller_settings"] = read if read is not None else "확인 꺼짐(설정 null)"
            # ③ 시작 위치
            self._step("start", "active", "펜 끝 위치를 확인합니다")
            used_start_pose = self._start_pose and not calibrated
            if requested_mode:
                try:
                    sp.execute_requested_start(self.link, self.cfg, should_stop=self._stop.is_set)
                except sp.StartPoseError as e:
                    raise de.DrawError("start", str(e), "자동 복구를 하지 않았습니다. 로봇 자세를 확인하세요.") from None
                start_note = "저장된 대기 자세"
            else:
                if used_start_pose:
                    tcp = self._setup_start_pose()
                if (self._virtual and not used_start_pose) or calibrated:
                    # 방금 캘리브레이션했다면 로봇은 펜을 벽에서 뺀 자세로 끝났으므로 그 자세를 허용해서 검사
                    off = de.check_start(tcp, self.cfg, retracted=calibrated)
                    start_note = f"종이 중심에서 {off:.1f} mm"
                else:
                    moved = self._use_start_pose_as_paper_center(tcp)
                    self.summary["start_center_shift_mm"] = round(moved, 1)
                    start_note = ("시작 자세를 종이 중심으로 사용" +
                                  (f" (설정 파일 값과 {moved:.1f} mm 차이)" if moved >= 0.05 else ""))
            # 공중 경로의 관절 시뮬레이션은 사람이 ④에서 확인하는 동안 백그라운드에서 돌리고, 공중 모드를 고른 때만 기다린다
            self._start_air_simulation(strokes)
            self._step("start", "done", start_note)
            # ④ 최종 확인 (사람)
            self._step("confirm", "active", "종이·펜·주변을 확인하고 시작하세요")
            self._confirm.wait()
            if self._stop.is_set():
                raise de.DrawError("confirm", CANCELLED)
            ch = self._choice
            # 범위 검사는 접촉 자세로 움직이기 전에: 종이에 닿은 채로 멈추지 않게
            if not self._virtual and not ch["air"]:
                outside = de.check_limits(strokes, self.cfg)
                if outside:
                    raise de.DrawError(
                        "confirm", "설정된 펜 접촉 영역 밖에서는 그림을 시작할 수 없습니다.",
                        "공중 모드로 확인하거나, '종이·펜 위치 변경: 다시 보정'을 켜고 다시 시작해 접촉 영역을 측정하세요.")
            if requested_mode and not ch["air"]:
                # 공중 모드는 종이에 닿지 않으므로 접촉 자세로 가지 않고 기록된 접촉 샘플 기준으로 그린다
                try:
                    updated = sp.prepare_requested_drawing(self.link, self.cfg, should_stop=self._stop.is_set)
                except sp.StartPoseError as e:
                    raise de.DrawError("confirm", str(e), "자동 복구를 하지 않았습니다. 로봇 자세를 확인하세요.") from None
                self.cfg = updated
                s.cfg = updated
                sim_summary = s.simulate()
                if sim_summary["verdict"].startswith("FAIL"):
                    raise de.DrawError("confirm", f"접촉 자세 경로 시뮬레이션 실패: {sim_summary['verdict']}")
                with s.lock:
                    self._snap["sim_raw"] = s.sim["raw"]
            pre = de.preflight(strokes, self.cfg, pending=ch["pending"], air=ch["air"])
            if ch["air"]:
                self._wait_air_simulation()
                if self._air_verdict.startswith("FAIL"):
                    raise de.DrawError("confirm", f"공중 경로 시뮬레이션이 FAIL입니다: {self._air_verdict}",
                                       "그림 크기를 줄인 뒤 다시 시도하세요.")
                self._snap["sim_raw"] = self._air_sim
            self._step("confirm", "done", "공중 모드" if ch["air"] else "펜으로 그림")
            # ⑤ 그리는 중
            from . import paper_mapping as pm
            strokes_path = paths.output_dir() / f"run_strokes_{run_id}.json"
            doc = pm.build_strokes_document(strokes, snap["placement"],
                                            source={"image": snap["path"], "params": snap["params"],
                                                    "tool": "draw_job"})
            # 실행기는 메모리의 좌표를 사용한다. 파일도 같은 값을 보존해 실행 기록을 재현 가능하게 한다.
            for item, points in zip(doc["strokes"], strokes):
                item["points_xy_mm"] = [[float(x), float(y)] for x, y in points]
            pm.save_strokes_json(strokes_path, doc)
            traj = self._write_trajectory(run_id)
            total = len(pre["cmds"])
            writer.write(state="running", acked=0, total=total, trajectory=str(traj), message="그리는 중")
            self._open_rviz(traj)
            self._step("drawing", "active", "그리는 중")
            if not self._virtual:
                controller.drawing_started()
            started, est = time.monotonic(), pre["timing"]["total_s"]
            starts = [i for i, (_, label) in enumerate(pre["cmds"]) if label.endswith("pen-down")]
            # 획마다 마지막 명령 번호 (그리는 순서): 그 명령까지 ok가 오면 그 획은 다 그린 것
            ends = {}
            for i, (_, label) in enumerate(pre["cmds"]):
                parts = label.split(" ")
                if parts[0] == "stroke" and len(parts) > 1 and parts[1].isdigit():
                    ends[int(parts[1])] = i
            order = list(ends)
            last_cmds = [ends[k] for k in order]
            self.events("plan", strokes=[strokes[k] for k in order])

            def on_ack(acked, n):
                self._live["acked"] = acked
                writer.write(state="running", acked=acked)
                el = time.monotonic() - started
                # 처음 PROGRESS_SETTLE_S 동안은 미리 잰 값. 첫 명령 직후 0.3초 남짓으로 비를 재면 크게 틀림(22:41: 34분 표시, 실제 74분)
                ratio = el / max(est * acked / n, 1e-6) if acked and el >= PROGRESS_SETTLE_S else 1.0
                remaining = est * (1 - acked / n) * ratio
                self.events("progress", acked=acked, total=n, stroke=sum(1 for i in starts if i < acked),
                            strokes=len(strokes), drawn=bisect.bisect_left(last_cmds, acked),
                            elapsed_s=el, remaining_s=remaining)

            pen_keys = ("start_backoff_mm", "end_backoff_mm", "edge_rise_mm", "edge_rise_low_mm", "edge_land_mm")
            self._live = {"cmds": pre["cmds"], "strokes": strokes, "air": ch["air"], "acked": 0, "started": started,
                          "pen_keys": pen_keys,
                          "pen_changes": [{"from_stroke": 0, "elapsed_s": 0.0,
                                           **{k: self.cfg["pen"].get(k, 0.0) for k in pen_keys}}]}
            result, decision = self._draw_with_pauses(pre["cmds"], pre["planner"], on_ack)
            if result["result"] == "completed" and self._return_origin and (not ch["air"] or self.cfg.get("requested_start_pose")):
                ok, why = self._return_to_origin(pre["planner"])
                result = {**result, "returned_to_origin": ok, **({"return_error": why} if why else {})}
            elif (result["result"] == "stopped_by_user" and decision == "finish" and self._return_origin
                  and requested_mode):
                ok, why = self._return_after_stop(pre["planner"])
                result = {**result, "returned_to_origin": ok, **({"return_error": why} if why else {})}
            final = {"completed": "done", "stopped_by_user": "stopped"}.get(result["result"], "error")
            writer.write(state=final, message=result.get("error", result["result"]))
            if final == "done":
                if result.get("returned_to_origin") is False:
                    self._step("drawing", "done", "완료 (펜이 시작 위치로 돌아가지 못했습니다)",
                               f"{result.get('return_error', '')} 다음 그림 전에 호밍하거나 펜 위치를 확인하세요.")
                else:
                    self._step("drawing", "done", "완료")
            elif final == "stopped" and result.get("returned_to_origin"):
                self._step("drawing", "failed", "사용자 멈춤", "펜을 종이에서 떼고 호밍 자세(관절 0)로 돌아왔습니다.")
            elif final == "stopped" and result.get("returned_to_origin") is False:
                self._step("drawing", "failed", "사용자 멈춤 (호밍 자세로 돌아가지 못했습니다)",
                           f"{result.get('return_error', '')} 펜과 로봇 상태를 확인하세요.")
            elif final == "stopped" and result.get("pen_lifted"):
                self._step("drawing", "failed", "사용자 멈춤", "펜을 종이에서 떼고 그 자리에 두었습니다.")
            else:
                self._step("drawing", "failed", "사용자 멈춤" if final == "stopped" else result.get("error", ""),
                           (f"{result['lift_error']} " if result.get("lift_error") else "") +
                           "자동 복구를 하지 않았습니다. 펜과 로봇 상태를 확인하세요.")
            record = de.write_run_record({
                "strokes_json": str(strokes_path), "stroke_count": len(strokes), "command_count": total,
                "air_mode": ch["air"], "pending_limits": ch["pending"], "estimated_time": pre["timing"],
                "return_to_origin": self._return_origin, "paper_fit": snap["paper_fit"],
                "source": {"image": snap["path"], "params": snap["params"]},
                "virtual": self._virtual, "virtual_speed": self._speed if self._virtual else None,
                "port": self.summary["port"], "controller_settings": self.summary.get("controller_settings"),
                "run_id": run_id, "started_local": run_id, "trajectory": str(traj),
                "pen_changes": self._live["pen_changes"]}, result, self.cfg)
        except de.DrawError as e:
            self._step(e.step, "failed", e.message, e.hint)
            result = {"result": "cancelled" if e.message == CANCELLED else "not_started", "error": e.message}
            writer.write(state="stopped", message=e.message)
        except Exception as e:  # 예상 못 한 오류도 화면에 보이게 (포트는 아래에서 닫음)
            self._step(self.state if self.state not in ("idle", "done") else "preflight", "failed",
                       f"{type(e).__name__}: {e}")
            result = {"result": "error", "error": str(e)}
            writer.write(state="error", message=str(e))
        finally:
            self._live = None               # 끝점 물림은 보내는 중일 때만 바꿀 수 있음
            self._air_cancel.set()          # 남은 공중 경로 계산이 CPU를 쓰지 않게
            if self.link is not None:
                try:
                    self.link.close()
                except Exception as e:  # 닫기 실패해도 finished는 보내야 GUI 잠금이 풀림
                    self.events("step", id="done", status="failed", message=f"포트 닫기 실패: {e}",
                                hint="USB를 뺐다 꽂고 앱을 다시 시작하세요.")
            controller.finished(result.get("result"), link_open=self.link is not None,
                                returned_to_origin=result.get("returned_to_origin"))
            self._step("done", "active", "")
            self._step("done", "done", result["result"])
            self.state = "done"
            self.events("finished", result=result, record_path=str(record) if record else None)

    def _setup_start_pose(self):
        """③ 시작 자세: 저장된 자세로 자동 이동 → 사람이 조그로 맞추고 [기억하고 계속] → 저장하고 그 TCP를 반환."""
        from . import start_pose as sp

        self._step("start", "active", "시작 자세를 맞춥니다: 조그로 펜 끝을 종이 가운데에 살짝 대고 [이 위치 기억하고 계속]")
        setup = sp.StartPoseSetup(self.link, self.cfg, paths.config_path(), self.events)
        self.start_pose_setup = setup
        try:
            return setup.run(should_stop=self._stop.is_set)
        except sp.StartPoseError as e:
            raise de.DrawError("start", str(e), "" if str(e) == CANCELLED else
                               "호밍 상태와 펜 위치를 확인하고 다시 연결하세요.") from None
        except de.ControllerError as e:
            raise de.DrawError("start", f"조그 중 컨트롤러 오류: {e}",
                               "자동 복구를 하지 않았습니다. 로봇 상태를 확인하고 호밍부터 다시 하세요.") from None
        finally:
            self.start_pose_setup = None

    def _draw_with_pauses(self, cmds, planner, on_ack):
        """⑤ 명령을 보내다 [멈춤]이 눌리면 펜을 종이에서 떼고 일시정지한다. 사람이 [이어서]를 고르면 멈춘 자리에 다시 펜을
        대고 다음 명령부터 이어 보낸다(이미 그은 부분은 다시 긋지 않음). 같은 연결·같은 명령 목록을 그대로 쓴다.
        반환: (실행 결과(구간들을 합침), 마지막 선택: None(멈춤 없이 끝남·오류) / "finish"(끝내기) / "end"(그 자리에서 끝))."""
        start, segments, pauses = 0, [], 0
        while True:
            r = de.execute(self.link, cmds, self.cfg, progress=lambda *_: None, on_ack=on_ack,
                           should_stop=self._stop.is_set, start=start)
            segments.append(r)
            if r["result"] != "stopped_by_user":
                return self._merged(segments, r, pauses), None
            sent = r["commands_sent"]
            while True:     # 일시정지: 펜 떼기 → 사람의 선택 → (이어서) 멈춘 자리에 다시 펜 대기
                pauses += 1
                lifted, lift_error = self._lift_for_pause(planner)
                if lift_error:
                    return self._merged(segments, {**r, "lift_error": lift_error}, pauses), "end"
                decision = self._wait_pause_decision(sent, len(cmds), lifted)
                if decision != "resume":
                    return self._merged(segments, {**r, "pen_lifted": lifted}, pauses), decision
                try:
                    start = self._approach_for_resume(planner, cmds, sent)
                except (de.ControllerError, OSError) as e:
                    failed = {**r, "result": "stopped_on_error", "error": f"이어 그리려고 펜을 대는 중 오류: {e}"}
                    return self._merged(segments, failed, pauses), None
                if start is not None:
                    break

    @staticmethod
    def _merged(segments, last, pauses):
        """이어 그린 구간들의 실행 결과를 하나로: 마지막 구간의 결과·보낸 수에 걸린 시간과 컨트롤러 기록을 합침."""
        merged = {**last, "elapsed_s": round(sum(s.get("elapsed_s", 0.0) for s in segments), 1),
                  "controller_trace": [t for s in segments for t in s.get("controller_trace", [])]}
        if pauses:
            merged["pauses"] = pauses
        return merged

    def _lift_for_pause(self, planner):
        """[멈춤] 직후: 멈춰 선 자리에서 펜을 종이 반대쪽(X)으로 펜업 높이까지 곧게 뗀다(옆으로 가지 않음).
        공중 모드는 이미 종이에서 떨어져 있어 움직이지 않는다. 떼는 중에 한 번 더 누르면 그 자리에서 멈춤.
        창을 닫는 중(end)이면 움직이지 않고 그 자리에서 바로 끝낸다 (창이 오래 기다리지 않게).
        반환: (뗐는지, 오류 문구 또는 "")."""
        if planner.air or self._end_requested:
            return False, ""
        base = self._stop_presses     # 멈춤은 보내던 명령의 ok 뒤에야 걸리므로, 떼기 시작한 뒤 누른 것만 셈

        def again():
            return self._stop_presses > base

        self._step("drawing", "active", "멈춤: 펜을 종이에서 떼는 중입니다. 다시 누르면 그 자리에서 멈춥니다")
        try:
            self._lift_straight(planner, again)
        except de.StopRequested:
            return False, ""
        except de.ControllerError as e:
            return False, f"펜을 떼는 중 오류: {e}"
        except OSError as e:
            return False, f"로봇과 USB 통신이 끊겼습니다: {e}"
        return True, ""

    def _lift_straight(self, planner, should_stop):
        """멈춰 선 자리(Y·Z 그대로)에서 X만 펜업 높이까지 뺀다. 이미 그 높이 이상 떨어져 있으면 보내지 않는다."""
        x, y, z = self._settled_tcp(should_stop)
        target = (planner._farther_x(x, planner.contact_x(y, z) + planner.up_dx), y, z)
        if should_stop():
            raise de.StopRequested()
        if abs(target[0] - x) > 0.05:
            self.link.send_and_ack(planner.gcode(target, planner.feeds["approach"]), self.cfg["ack_timeout_s"],
                                   should_stop=should_stop)
            self.link.wait_until_pose(target, self.cfg["idle_timeout_s"], should_stop=should_stop)

    def _pending_pause_decision(self):
        """펜을 떼는 동안 들어온 선택. finish가 end보다 이기고, 아니면 가장 최근 것을 남긴다."""
        pending = []
        while True:
            try:
                pending.append(self._decision.get_nowait())
            except queue.Empty:
                break
        if "finish" in pending:
            return "finish"
        if "end" in pending:
            return "end"
        return pending[-1] if pending else None

    def _wait_pause_decision(self, sent, total, lifted):
        """일시정지: 사람이 resume / finish / end 중 하나를 고를 때까지 기다린다 (포트는 연 채로, 로봇은 그 자리에).
        펜을 떼는 동안 이미 고른 finish/end는 기다리지 않고 쓴다. 창을 닫는 중이면(end) 기다리지 않는다."""
        early = self._pending_pause_decision()
        if early == "finish":
            return "finish"
        if early == "end" or self._end_requested:
            return "end"
        if early is not None:
            self._decision.put(early)
        where = "펜을 종이에서 뗐습니다" if lifted else "로봇이 그 자리에 있습니다"
        self._step("drawing", "active", f"일시정지 (명령 {sent:,}/{total:,}): {where}. "
                                        "[▶ 이어서 그리기]를 누르면 멈춘 자리부터 다시 그립니다")
        self.state = "paused"
        self.events("paused", acked=sent, total=total)
        decision = self._decision.get()
        if decision == "resume":
            self._stop.clear()
            self.events("resumed")
        return decision

    def _approach_for_resume(self, planner, cmds, sent):
        """이어서: 멈춘 자리에 펜을 다시 댄다(de.resume_commands). 반환: 이어 보낼 첫 명령 번호. 펜을 대는 중에
        [멈춤]을 누르면 None(다시 일시정지)."""
        prefix, start = de.resume_commands(planner, cmds, sent)
        self._step("drawing", "active", "이어 그리기: 멈춘 자리에 펜을 다시 대는 중입니다" if prefix else "그리는 중 (이어서)")
        try:
            for line, _label in prefix:
                self.link.send_and_ack(line, self.cfg["ack_timeout_s"], should_stop=self._stop.is_set)
                target = tuple(float(v) for v in de._XYZ.search(line).groups())
                self.link.wait_until_pose(target, self.cfg["idle_timeout_s"], should_stop=self._stop.is_set)
        except de.StopRequested:
            return None
        if prefix:
            self._step("drawing", "active", "그리는 중 (이어서)")
        return start

    def _return_to_origin(self, planner):
        """완료 뒤 펜을 처음 상태로 되돌림. 반환: (성공 여부 또는 None(공중 모드라 해당 없음), 오류 문구).
        자동 복구는 하지 않는다: 멈춤·오류가 나면 그 자리에서 멈추고 사람에게 알린다."""
        if self.cfg.get("requested_start_pose"):
            from . import start_pose as sp
            self._step("drawing", "active", "대기 자세를 거쳐 호밍 자세로 돌아가는 중입니다")
            try:
                sp.execute_requested_return(self.link, self.cfg, should_stop=self._stop.is_set)
                return True, ""
            except (sp.StartPoseError, de.ControllerError, OSError) as e:
                return False, f"호밍 자세 복귀 실패: {e}"
        cmds = planner.return_to_origin()
        if not cmds:
            return None, ""
        self._step("drawing", "active", "그림 완료! 펜을 처음 위치(종이 중심)로 되돌리는 중입니다")
        try:
            for line, _label in cmds:
                if self._stop.is_set():
                    return False, "멈춤 요청으로 시작 위치로 돌아가지 않았습니다."
                self.link.send_and_ack(line, self.cfg["ack_timeout_s"], should_stop=self._stop.is_set)
            self.link.wait_idle(self.cfg["idle_timeout_s"])
        except de.StopRequested:
            return False, "멈춤 요청으로 시작 위치로 돌아가지 않았습니다."
        except de.ControllerError as e:
            return False, f"시작 위치로 돌아가는 중 오류: {e}"
        except Exception as e:   # 이미 끝까지 그린 그림이 되돌리기의 예상 못 한 오류 때문에 실패로 바뀌지 않게
            return False, f"시작 위치로 돌아가는 중 오류: {type(e).__name__}: {e}"
        return True, ""

    def _return_after_stop(self, planner):
        """일시정지에서 [끝내기]: 펜을 종이 반대쪽으로 펜업 높이까지 곧게 뗀 다음(일시정지에서 이미 뗐으면 보내지 않음),
        1번 대기 자세를 거쳐 호밍 자세(관절 0)로. 펜이 종이를 누른 채일 수 있어 먼저 떼야 관절 이동이 종이를 긁지 않는다.
        대기 자세로 가는 관절 경로는 start_pose가 펜 끝과 종이 거리를 검사하고, 통과하지 못하면 움직이지 않는다.
        돌아가는 중에 [멈춤]을 누르면 그 자리에서 멈춤. 반환: (성공 여부, 오류 문구)."""
        from . import start_pose as sp

        base = self._stop_presses     # 돌아가기 시작한 뒤 누른 것만 셈

        def again():
            return self._stop_presses > base

        self._step("drawing", "active", "끝내기: 펜을 종이에서 떼고 호밍 자세(관절 0)로 돌아가는 중입니다. "
                                        "다시 누르면 그 자리에서 멈춥니다")
        try:
            self._lift_straight(planner, again)
            sp.execute_requested_return(self.link, self.cfg, should_stop=again)
        except de.StopRequested:
            return False, "다시 [멈춤]을 눌러 그 자리에 두었습니다."
        except sp.StartPoseError as e:
            if again():               # 관절 이동 사이에 [멈춤]: start_pose는 '취소했습니다'로 멈춤
                return False, "다시 [멈춤]을 눌러 그 자리에 두었습니다."
            return False, f"호밍 자세 복귀 실패: {e}"
        except de.ControllerError as e:
            return False, f"펜을 떼고 돌아가는 중 오류: {e}"
        except OSError as e:
            return False, f"로봇과 USB 통신이 끊겼습니다: {e}"
        return True, ""

    def _settled_tcp(self, should_stop, samples=3, tol_mm=0.05):
        """멈춘 직전에 보낸 명령이 끝나 Idle에서 같은 좌표가 samples번 이어 보고될 때까지 기다린 TCP.
        ok는 명령을 받았다는 뜻일 뿐이라, 첫 Idle만 보고 다음 명령을 계산하면 아직 움직일 자리를 놓칠 수 있다."""
        deadline = time.monotonic() + self.cfg["idle_timeout_s"]
        last, same = None, 0
        while time.monotonic() < deadline:
            if should_stop():
                raise de.StopRequested()
            state, tcp, raw = self.link.query_status()
            if any(t in raw.lower() for t in de.ERROR_TOKENS):
                raise de.ControllerError(f"멈춘 뒤 오류 상태: {raw}")
            if state == "Idle" and tcp is not None:
                same = same + 1 if last is not None and max(abs(a - b) for a, b in zip(tcp, last)) <= tol_mm else 1
                last = tcp
                if same >= samples:
                    return tuple(float(v) for v in tcp)
            else:
                last, same = None, 0
            time.sleep(0.1)
        raise de.ControllerError("멈춘 뒤 로봇이 멈춰 선 위치를 확인하지 못했습니다")

    def _start_air_simulation(self, strokes):
        """공중 경로 시뮬레이션을 백그라운드 스레드로 시작 (그림·설정은 이 시점의 사본을 씀)."""
        from . import mirobot_sim as ms
        cfg = copy.deepcopy(self.cfg)
        pts = [[tuple(pt) for pt in st] for st in strokes]

        def check_cancel(done, total):
            if self._air_cancel.is_set():
                raise ms.SimulationCancelled()

        def work():
            try:
                rotation, q0 = ms.simulation_pose(cfg)
                self._air_sim = ms.simulate(ms.plan_targets(pts, cfg, air=True), progress=check_cancel,
                                            r_goal=rotation, q0=q0)
                self._air_verdict = ms.verdict(self._air_sim)
            except ms.SimulationCancelled:
                pass
            except Exception as e:  # 공중 모드를 골랐을 때 오류로 보여 줌
                self._air_error = e

        self._air_thread = threading.Thread(target=work, daemon=True)
        self._air_thread.start()

    def _wait_air_simulation(self):
        """공중 모드를 골랐을 때만: 백그라운드 시뮬레이션이 끝날 때까지 기다림 (멈춤·취소는 바로 반영)."""
        thread = self._air_thread
        if thread is not None and thread.is_alive():
            self._step("confirm", "active", "공중 경로의 관절 움직임을 확인하는 중입니다 (잠시만요)")
            while thread.is_alive():
                if self._stop.is_set():
                    raise de.DrawError("confirm", CANCELLED)
                thread.join(0.1)
        if self._air_error is not None or self._air_verdict is None:
            raise de.DrawError("confirm", f"공중 경로 시뮬레이션에 실패했습니다: {self._air_error or '결과 없음'}",
                               "그림을 다시 처리한 뒤 시도하세요.")

    def _write_trajectory(self, run_id):
        from . import mirobot_sim as ms
        path = paths.output_dir() / f"live_traj_{run_id}.json"
        doc = ms.trajectory_doc(self._snap["sim_raw"], self.cfg, self._snap["path"])
        path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
        self._traj_path = path
        return path

    def _open_rviz(self, traj):
        """RViz 따라가기를 백그라운드로 엶. WSL이 깨어나는 데 10초 넘게 걸릴 수 있어 드로잉은 기다리지 않음
        (RViz는 진행 파일로 현재 위치를 따라잡음)."""
        launch = self.launch_rviz
        if launch is None:
            from .rviz_launch import launch

        def work():
            try:
                launch(traj, follow=self.progress_path)
                self.events("rviz", ok=True, message="RViz 따라가기를 열었습니다")
            except Exception as e:  # RvizUnavailable 포함: 드로잉은 계속
                self.events("rviz", ok=False, message=str(e))

        threading.Thread(target=work, daemon=True).start()
