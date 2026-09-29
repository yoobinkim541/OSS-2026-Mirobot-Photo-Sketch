"""
로봇으로 그리기 작업 — 안전 절차 ①~⑥을 작업 스레드에서 진행 (화면과 무관, GUI 실행 창이 events로 표시)
======================================================================================================
①사전 검사 → ②연결·호밍 → ③시작 위치 → ④최종 확인(사람이 confirm 할 때까지 대기) → ⑤그리는 중 → ⑥끝.
ok 응답마다 진행 파일을 써서 RViz 따라가기가 같은 위치를 보여 줍니다. 자동 복구는 하지 않습니다.

events(kind, **data):
  step      id, status(active|done|failed), message, hint
  progress  acked, total, stroke, strokes, elapsed_s, remaining_s
  rviz      ok, message
  finished  result, record_path
"""

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


def target_contact_extents(strokes, cfg):
    """확장 그림을 둘러싼 중심 정렬 접촉 직사각형의 반폭. 불가능하면 None."""
    from . import limits

    hx = math.ceil(max(abs(x) for st in strokes for x, _ in st)) + 1
    hy = math.ceil(max(abs(y) for st in strokes for _, y in st)) + 1
    if all(limits.pending_region(cfg).contains(x, y)
           for x in (-hx, hx) for y in (-hy, hy)):
        return hx, hy
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
        self._confirm = threading.Event()
        self._stop = threading.Event()
        self._choice = {}
        self._thread = None
        self._calibration_reply = None
        self._calibration_reply_lock = threading.Lock()

    # ---------------------------------------------------------------- 사람이 누르는 것
    def start(self, virtual=True, virtual_speed=20.0, pending=False, recalibrate=False):
        """pending: 실물 확인 전 넓은 범위(limits_pending_verification) 허용 — ① 사전 검사부터 적용 (④에서 바꿀 수도 있음)."""
        self._virtual, self._speed, self._pending = bool(virtual), float(virtual_speed), bool(pending)
        self._recalibrate = bool(recalibrate)
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
        """⑤에서는 다음 명령부터 보내지 않음(멈춤), 그 전 단계에서는 취소."""
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

    def answer_calibration_input(self, value):
        """GUI에서 받은 사람 응답을 캘리브레이션 작업 스레드에 전달."""
        with self._calibration_reply_lock:
            reply = self._calibration_reply
        if reply is not None:
            try:
                reply.put_nowait("q" if value is None else str(value))
            except queue.Full:
                pass

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
            if s.sim is None:
                s.simulate()
            # ①에서 그릴 획·시뮬레이션·경로를 한 번에 찍어 둠: 이후 세션이 바뀌어도(편집·이미지 열기)
            # 로봇이 그리는 것, 확인 요약, RViz 궤적, 실행 기록이 모두 같은 그림을 가리키게
            with s.lock:
                if s.result is None or s.sim is None:
                    raise de.DrawError("preflight", "처리 결과나 시뮬레이션이 없습니다.", "이미지를 다시 처리하세요.")
                snap = {"strokes": [[tuple(pt) for pt in st] for st in s.result["strokes_mm"]],
                        "sim_raw": s.sim["raw"], "verdict": s.sim["summary"]["verdict"], "path": s.result["path"],
                        "params": dict(s.result["params"]), "placement": dict(s.result["placement"])}
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
                            "port": "가상 시뮬레이션" if self._virtual else self.cfg["port"],
                            "virtual": self._virtual, "virtual_speed": self._speed,
                            "plane_verified": self.cfg.get("plane_compensation", {}).get("status") == "verified"}
            self._step("preflight", "done", f"획 {len(strokes)}개 · 명령 {len(pre['cmds'])}줄 · 시뮬레이션 {verdict.split(':')[0]}")
            # ② 연결·호밍
            self._step("connect", "active", "가상 시뮬레이션에 연결합니다" if self._virtual else
                       "로봇 가운데 버튼을 2초 눌러 호밍하세요. Idle이 되면 자동으로 넘어갑니다")
            if not self._virtual:
                controller.connection_started()
            self.link = de.open_link(self.cfg, self._virtual, self._speed)
            writer.write(run_id=run_id, state="homing", acked=0, total=len(pre["cmds"]),
                         speed=self._speed if self._virtual else 1.0, trajectory=None, message="호밍 대기")
            from . import calibration
            calibration_path = None
            target_extents = None
            # 캘리브레이션은 사람이 '다시 보정'을 골랐을 때만: 아니면 바로 호밍 → 그리기 (범위·시작 위치 검사는 그대로)
            if self._recalibrate and not self._virtual and self._pending and de.check_limits(strokes, self.cfg):
                target_extents = target_contact_extents(strokes, self.cfg)
            if self._recalibrate and not self._virtual:
                self._step("connect", "active", "종이 캘리브레이션을 시작합니다")
                def calibration_progress(message):
                    self.events("calibration_status", message=message)
                    if robot_status.controller_state_from_progress(message) is not None:
                        controller.progress(message)

                try:
                    kw = {"target_half_extents_mm": target_extents} if target_extents else {}
                    measured = calibration.run_calibration(
                        self.link, self.cfg, input_fn=self._calibration_input,
                        output_fn=calibration_progress, **kw)
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
                self.summary.update(command_count=len(pre["cmds"]), estimated_s=pre["timing"]["total_s"])
            else:
                tcp = de.connect_and_home(self.link, self.cfg,
                                          progress=controller.progress if not self._virtual else lambda *_: None,
                                          should_cancel=self._stop.is_set)
            self._step("connect", "done", f"캘리브레이션 완료 · {calibration_path.name}"
                       if calibration_path is not None and measured.get("result") == "ready" else "Idle")
            # ③ 시작 위치
            self._step("start", "active", "펜 끝 위치를 확인합니다")
            off = de.check_start(tcp, self.cfg)
            from . import mirobot_sim as ms
            self._step("start", "active", "공중 경로의 관절 움직임을 확인합니다")
            self._air_sim = ms.simulate(ms.plan_targets(strokes, self.cfg, air=True))
            self._air_verdict = ms.verdict(self._air_sim)
            self._step("start", "done", f"종이 중심에서 {off:.1f} mm")
            # ④ 최종 확인 (사람)
            self._step("confirm", "active", "종이·펜·주변을 확인하고 시작하세요")
            self._confirm.wait()
            if self._stop.is_set():
                raise de.DrawError("confirm", CANCELLED)
            ch = self._choice
            pre = de.preflight(strokes, self.cfg, pending=ch["pending"], air=ch["air"])
            if not self._virtual and not ch["air"]:
                outside = de.check_limits(strokes, self.cfg)
                if outside:
                    raise de.DrawError(
                        "confirm", "설정된 펜 접촉 영역 밖에서는 그림을 시작할 수 없습니다.",
                        "공중 모드로 확인하거나, '종이·펜 위치 변경: 다시 보정'을 켜고 다시 시작해 접촉 영역을 측정하세요.")
            if ch["air"]:
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

            def on_ack(acked, n):
                writer.write(state="running", acked=acked)
                el = time.monotonic() - started
                ratio = el / max(est * acked / n, 1e-6) if acked else 1.0   # 실제/예상 속도 비로 보정
                self.events("progress", acked=acked, total=n, stroke=sum(1 for i in starts if i < acked),
                            strokes=len(strokes), elapsed_s=el, remaining_s=est * (1 - acked / n) * ratio)

            result = de.execute(self.link, pre["cmds"], self.cfg, progress=lambda *_: None,
                                on_ack=on_ack, should_stop=self._stop.is_set)
            final = {"completed": "done", "stopped_by_user": "stopped"}.get(result["result"], "error")
            writer.write(state=final, message=result.get("error", result["result"]))
            if final == "done":
                self._step("drawing", "done", "완료")
            else:
                self._step("drawing", "failed", "사용자 멈춤" if final == "stopped" else result.get("error", ""),
                           "자동 복구를 하지 않았습니다. 펜과 로봇 상태를 확인하세요.")
            record = de.write_run_record({
                "strokes_json": str(strokes_path), "stroke_count": len(strokes), "command_count": total,
                "air_mode": ch["air"], "pending_limits": ch["pending"], "estimated_time": pre["timing"],
                "source": {"image": snap["path"], "params": snap["params"]},
                "virtual": self._virtual, "virtual_speed": self._speed if self._virtual else None,
                "run_id": run_id, "trajectory": str(traj)}, result, self.cfg)
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
            if self.link is not None:
                try:
                    self.link.close()
                except Exception as e:  # 닫기 실패해도 finished는 보내야 GUI 잠금이 풀림
                    self.events("step", id="done", status="failed", message=f"포트 닫기 실패: {e}",
                                hint="USB를 뺐다 꽂고 앱을 다시 시작하세요.")
            controller.finished(result.get("result"), link_open=self.link is not None)
            self._step("done", "active", "")
            self._step("done", "done", result["result"])
            self.state = "done"
            self.events("finished", result=result, record_path=str(record) if record else None)

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
