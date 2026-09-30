"""실행 작업(가상 시뮬레이션): 단계 순서, 확인 전 대기, 멈춤, 기록, 진행 파일."""

import re
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import live_progress as lp  # noqa: E402
from mirobot_sketch.draw_job import DrawJob, target_contact_rect  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

import golden  # noqa: E402


class Recorder:
    def __init__(self):
        self.events, self.lock = [], threading.Lock()
        self.ready = threading.Event()
        self.done = threading.Event()

    def __call__(self, kind, **data):
        with self.lock:
            self.events.append((kind, data))
        if kind == "step" and data["id"] == "confirm" and data["status"] == "active":
            self.ready.set()
        if kind == "finished":
            self.done.set()

    def steps(self):
        return [(d["id"], d["status"]) for k, d in self.events if k == "step"]


class PhysicalLinkStub:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class DrawJobTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        p = Path(cls.tmp.name) / "line.png"
        cv2.imwrite(str(p), golden.synthetic_images()["line"])
        cls.session = SketchSession(golden.default_cfg())
        cls.session.set_image(p)
        cls.session.update_params({"box_mm": 60, "epsilon_px": 4.0})
        cls.session.run_current()
        cls.session.simulate()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def job(self, rec, launch_rviz=lambda *a, **k: None):
        d = Path(self.tmp.name)
        for name, fn in (("runs_dir", lambda: d / "runs"), ("output_dir", lambda: d)):
            patcher = mock.patch.object(de.paths, name, fn)
            patcher.start()
            self.addCleanup(patcher.stop)
        return DrawJob(self.session, self.session.cfg, rec, progress_path=d / "live.json", launch_rviz=launch_rviz)

    def test_waits_for_confirmation_then_runs_all_steps(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        self.assertEqual(job.state, "confirm")
        self.assertIn("command_count", job.summary)
        with self.assertRaises(ValueError):
            job.confirm(checked=False)                         # 확인 체크 없이는 시작 안 함
        job.confirm(air=True, checked=True)
        self.assertTrue(rec.done.wait(60))
        done_ids = [i for i, st in rec.steps() if st == "done"]
        self.assertEqual(done_ids, ["preflight", "connect", "start", "confirm", "drawing", "done"])
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertTrue(Path(fin["record_path"]).exists())
        import json
        run = json.loads(Path(fin["record_path"]).read_text(encoding="utf-8"))
        strokes_file = Path(self.tmp.name) / run["strokes_json"]           # 기록에는 파일 이름만 (저장소 밖 경로)
        self.assertTrue(strokes_file.exists())
        doc, saved_strokes = de.load_strokes(strokes_file)
        self.assertEqual(doc["source"]["image"], str(self.session.image_path))
        self.assertEqual(saved_strokes[0], [tuple(pt) for pt in self.session.result["strokes_mm"][0]])
        prog = [d for k, d in rec.events if k == "progress"]
        self.assertEqual(prog[-1]["acked"], prog[-1]["total"])
        live = lp.read_progress(Path(self.tmp.name) / "live.json")
        self.assertEqual((live["state"], live["acked"]), ("done", live["total"]))
        self.assertTrue(Path(lp.to_local_path(live["trajectory"])).exists())

    def test_recalibrate_choice_runs_guided_calibration_before_drawing(self):
        from mirobot_sketch import calibration

        cfg = golden.default_cfg()
        session = SketchSession(cfg)
        session.set_image(Path(self.tmp.name) / "line.png")
        session.update_params({"box_mm": 40, "epsilon_px": 4.0})
        session.run_current()
        session.simulate()
        rec = Recorder()
        job = self.job(rec)
        job.session, job.cfg = session, cfg
        center = dict(cfg["paper_center_tcp_mm"])
        ready = dict(center)
        ready["x"] -= cfg["pen"]["up_clearance_mm"]
        measurement = {"result": "ready", "center_tcp_mm": center, "ready_tcp_mm": ready,
                       "samples": [], "selected_half_size_mm": 30.0,
                       "max_residual_mm": 1.0,
                       "plane_fit": {"a_per_mm_y": 0.0, "b_per_mm_z": 0.0,
                                     "max_abs_residual_mm": 0.0}}
        link = PhysicalLinkStub()
        with mock.patch.object(de, "open_link", return_value=link), \
             mock.patch.object(calibration, "run_calibration", return_value=measurement) as run, \
             mock.patch.object(calibration, "save_report", return_value=Path("calibration.json")), \
             mock.patch.object(calibration, "save_config") as save:
            job.start(virtual=False, recalibrate=True)
            self.assertTrue(rec.ready.wait(15), rec.events[-5:])
            run.assert_called_once()
            save.assert_called_once()
            self.assertEqual(cfg["plane_compensation"]["status"], "verified")
            job.cancel()
            self.assertTrue(rec.done.wait(5))
        self.assertTrue(link.closed)

    def test_stop_during_drawing(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=1)
        self.assertTrue(rec.ready.wait(10))
        job.confirm(checked=True)
        deadline = time.monotonic() + 30                      # 공중 경로 확인이 끝나고 그리기가 시작된 뒤에 멈춤
        while not any(k == "progress" for k, _ in rec.events) and time.monotonic() < deadline:
            time.sleep(0.02)
        job.stop()
        self.assertTrue(rec.done.wait(10))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "stopped_by_user")
        self.assertEqual(lp.read_progress(Path(self.tmp.name) / "live.json")["state"], "stopped")
        self.assertEqual(job.link.state, "closed")

    def test_cancel_at_confirmation_closes_without_record(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.cancel()
        self.assertTrue(rec.done.wait(5))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "cancelled")
        self.assertIsNone(fin["record_path"])
        self.assertEqual(job.link.state, "closed")

    def test_wide_range_choice_applies_to_preflight(self):
        big = SketchSession(golden.default_cfg())
        big.set_image(Path(self.tmp.name) / "line.png")
        big.update_params({"box_mm": 110, "epsilon_px": 4.0})       # ±55mm: 실행기 허용(±50) 밖, 넓은 범위(±60) 안
        big.run_current()
        big.simulate()
        for pending, expect in ((False, "failed"), (True, "confirm")):
            rec = Recorder()
            job = self.job(rec)
            job.session = big
            job.start(virtual=True, virtual_speed=500, pending=pending)
            if expect == "confirm":
                self.assertTrue(rec.ready.wait(20))
                job.cancel()
            self.assertTrue(rec.done.wait(20))
            pre = [st for i, st in rec.steps() if i == "preflight"]
            self.assertEqual(pre[-1], "failed" if expect == "failed" else "done", pending)

    def test_real_pen_down_rejects_unmeasured_area_after_confirmation(self):
        cfg = golden.default_cfg()
        cfg["plane_compensation"]["status"] = "verified"
        big = SketchSession(cfg)
        big.set_image(Path(self.tmp.name) / "line.png")
        big.update_params({"box_mm": 110, "epsilon_px": 4.0})
        big.run_current()
        big.simulate()
        self.assertTrue(de.check_limits(big.result["strokes_mm"], big.cfg))
        rec = Recorder()
        job = self.job(rec)
        job.session, job.cfg = big, big.cfg
        link = PhysicalLinkStub()
        center = dict(cfg["paper_center_tcp_mm"])
        with mock.patch.object(de, "open_link", return_value=link), \
             mock.patch.object(de, "connect_and_home", return_value=tuple(center[k]
                                                                         for k in ("x", "y", "z"))):
            job.start(virtual=False, pending=True)
            self.assertTrue(rec.ready.wait(15), rec.events[-5:])
            job.confirm(air=False, pending=True, checked=True)
            self.assertTrue(rec.done.wait(5))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "not_started")
        self.assertIn("설정된 펜 접촉 영역", fin["result"]["error"])
        self.assertTrue(link.closed)

    def test_physical_run_without_recalibrate_skips_calibration_and_draws_pen_down(self):
        from mirobot_sketch import calibration

        cfg = golden.default_cfg()
        self.assertNotEqual(cfg["plane_compensation"]["status"], "verified")   # 보정 안 된 설정
        session = SketchSession(cfg)
        session.set_image(Path(self.tmp.name) / "line.png")
        session.update_params({"box_mm": 40, "epsilon_px": 4.0})
        session.run_current()
        session.simulate()
        rec = Recorder()
        job = self.job(rec)
        job.session, job.cfg = session, cfg
        c = cfg["paper_center_tcp_mm"]
        link = PhysicalLinkStub()
        with mock.patch.object(de, "open_link", return_value=link), \
             mock.patch.object(de, "connect_and_home", return_value=(c["x"], c["y"], c["z"])), \
             mock.patch.object(calibration, "run_calibration") as run, \
             mock.patch.object(calibration, "save_config") as save, \
             mock.patch.object(de, "execute", return_value={"result": "completed", "commands_sent": 1,
                                                            "elapsed_s": 0.1}) as execute:
            job.start(virtual=False, return_to_origin=False)          # 가짜 연결이라 되돌리기는 이 테스트와 무관
            self.assertTrue(rec.ready.wait(15), rec.events[-5:])
            self.assertFalse(job.summary["plane_verified"])
            job.confirm(air=False, pending=False, checked=True)
            self.assertTrue(rec.done.wait(10), rec.events[-5:])
            run.assert_not_called()
            save.assert_not_called()
            execute.assert_called_once()
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertTrue(link.closed)

    def _start_pose_run(self, dx, air=True):
        """실물 연결(보정 안 함)에서 호밍 자세가 설정 파일의 종이 중심에서 X로 dx mm 벗어난 상황."""
        from mirobot_sketch import calibration

        cfg = golden.default_cfg()
        session = SketchSession(cfg)
        session.set_image(Path(self.tmp.name) / "line.png")
        session.update_params({"box_mm": 40, "epsilon_px": 4.0})
        session.run_current()
        session.simulate()
        rec = Recorder()
        job = self.job(rec)
        job.session, job.cfg = session, cfg
        c = dict(cfg["paper_center_tcp_mm"])
        tcp = (c["x"] + dx, c["y"], c["z"])
        link = PhysicalLinkStub()
        with mock.patch.object(de, "open_link", return_value=link),              mock.patch.object(de, "connect_and_home", return_value=tcp),              mock.patch.object(calibration, "save_config") as save,              mock.patch.object(de, "execute", return_value={"result": "completed", "commands_sent": 1,
                                                            "elapsed_s": 0.1}) as execute:
            job.start(virtual=False)
            self.assertTrue(rec.ready.wait(20), rec.events[-5:])
            job.confirm(air=air, pending=False, checked=True)
            self.assertTrue(rec.done.wait(10), rec.events[-5:])
        return cfg, c, tcp, save, execute, job, rec, link

    def test_real_start_pose_is_used_as_paper_center_without_asking(self):
        cfg, before, tcp, save, execute, job, rec, link = self._start_pose_run(-18.0)
        self.assertEqual(cfg["paper_center_tcp_mm"], {"x": tcp[0], "y": tcp[1], "z": tcp[2]})
        self.assertAlmostEqual(job.summary["start_center_shift_mm"], 18.0, places=1)
        self.assertFalse(any(k == "start_offset" for k, _ in rec.events))       # 확인 대화상자는 없음
        save.assert_not_called()                                                # 설정 파일은 바꾸지 않음
        execute.assert_called_once()
        sent_first = execute.call_args.args[1][0][0]                            # 첫 명령이 새 종이 중심(펜업)을 향함
        x = float(re.search(r"X([-\d.]+)", sent_first).group(1))
        self.assertAlmostEqual(x, tcp[0] + cfg["pen"]["retract_x_sign"] * cfg["pen"]["air_clearance_mm"], places=2)
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertTrue(link.closed)

    def test_real_start_pose_matching_config_reports_no_shift(self):
        cfg, before, tcp, save, execute, job, rec, link = self._start_pose_run(0.0)
        self.assertEqual(job.summary["start_center_shift_mm"], 0.0)
        self.assertEqual(cfg["paper_center_tcp_mm"], before)

    def test_virtual_run_keeps_configured_paper_center(self):
        rec = Recorder()
        job = self.job(rec)
        before = dict(job.cfg["paper_center_tcp_mm"])
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.cancel()
        self.assertTrue(rec.done.wait(5))
        self.assertEqual(job.cfg["paper_center_tcp_mm"], before)
        self.assertNotIn("start_center_shift_mm", job.summary)

    def test_photo_sized_rectangle_can_be_selected_for_contact_calibration(self):
        cfg = golden.default_cfg()
        outline = [[(-60, -35.4), (60, -35.4), (60, 35.4), (-60, 35.4)]]
        self.assertEqual(target_contact_rect(outline, cfg), (-61.0, -36.4, 61.0, 36.4))
        centered_square = [[(-60, -60), (60, -60), (60, 60), (-60, 60)]]
        self.assertIsNone(target_contact_rect(centered_square, cfg))      # 위쪽 한계(55~57.5) 때문에 중심에 둘 수 없음

    def test_120mm_square_is_placed_lower_and_can_be_selected_for_contact_calibration(self):
        from mirobot_sketch import limits
        cfg = golden.default_cfg()
        self.assertEqual(limits.pending_region(cfg).fit(120, 120), (0.0, -5.0))   # 앱이 그림을 5 mm 아래로 배치
        placed = [[(-60, -65), (60, -65), (60, 55), (-60, 55)]]
        self.assertEqual(target_contact_rect(placed, cfg), (-60.0, -65.0, 60.0, 55.0))   # 가장자리 여유 없이 정확히 그림 범위
        self.assertEqual(target_contact_rect([[(-30, -40), (30, -40), (30, 20), (-30, 20)]], cfg),
                         (-31.0, -41.0, 31.0, 21.0))

    def test_rectangular_calibration_reaches_app_command_execution(self):
        from mirobot_sketch import calibration

        cfg = golden.default_cfg()
        session = SketchSession(cfg)
        session.set_image(Path(self.tmp.name) / "line.png")
        session.update_params({"box_mm": 60, "epsilon_px": 4.0})
        session.run_current()
        session.result["strokes_mm"] = [[(-60.0, -35.4), (60.0, -35.4),
                                         (60.0, 35.4), (-60.0, 35.4)]]
        session.simulate()
        rec = Recorder()
        job = self.job(rec)
        job.session, job.cfg = session, cfg
        center = dict(cfg["paper_center_tcp_mm"])
        center["x"] -= 3.0                       # 보정한 종이 중심이 기존 설정과 다름
        ready = dict(center)
        ready["x"] -= 9.0                        # 사각형 보정은 펜을 벽에서 8 mm 이상 뺀 자세로 끝남 (기존 5 mm 검사에 걸리던 값)
        measurement = {"result": "ready", "center_tcp_mm": center, "ready_tcp_mm": ready,
                       "samples": [], "selected_half_size_mm": [-61.0, -36.4, 61.0, 36.4],
                       "max_residual_mm": 1.0,
                       "plane_fit": {"a_per_mm_y": cfg["plane_compensation"]["a_per_mm_y"],
                                     "b_per_mm_z": cfg["plane_compensation"]["b_per_mm_z"],
                                     "max_abs_residual_mm": 0.0}}
        link = PhysicalLinkStub()
        with mock.patch.object(de, "open_link", return_value=link), \
             mock.patch.object(calibration, "run_calibration", return_value=measurement) as run, \
             mock.patch.object(calibration, "save_report", return_value=Path("calibration.json")), \
             mock.patch.object(calibration, "save_config"), \
             mock.patch.object(de, "execute", return_value={"result": "completed", "commands_sent": 7,
                                                            "elapsed_s": 0.1}) as execute:
            job.start(virtual=False, pending=True, recalibrate=True, return_to_origin=False)
            self.assertTrue(rec.ready.wait(25), rec.events[-5:])
            self.assertEqual(run.call_args.kwargs["target_rect_mm"], (-61.0, -36.4, 61.0, 36.4))
            self.assertEqual(cfg["limits"], {"x_max_mm": 61.0, "y_min_mm": -36.4,
                                              "roof_mm": [[0.0, 36.4], [61.0, 36.4]]})
            job.confirm(air=False, pending=True, checked=True)
            self.assertTrue(rec.done.wait(10), rec.events[-5:])
            execute.assert_called_once()
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertTrue(link.closed)

    def test_slow_rviz_launch_does_not_delay_drawing(self):
        import time
        rec = Recorder()
        job = self.job(rec, launch_rviz=lambda *a, **k: time.sleep(3))   # WSL이 깨어나는 데 오래 걸리는 경우
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job._air_thread.join(60)                          # 공중 경로 확인이 끝난 뒤부터 재서 RViz 대기만 측정
        t0 = time.monotonic()
        job.confirm(checked=True)
        first = threading.Event()
        orig = job.events

        def watch(kind, **d):
            if kind == "progress":
                first.set()
            orig(kind, **d)

        job.events = watch
        self.assertTrue(first.wait(5))
        self.assertLess(time.monotonic() - t0, 1.0)       # RViz를 기다리지 않고 바로 첫 명령
        self.assertTrue(rec.done.wait(60))

    def test_air_simulation_runs_while_waiting_and_is_awaited_only_for_air_mode(self):
        for air in (True, False):
            rec = Recorder()
            job = self.job(rec)
            with mock.patch.object(job, "_wait_air_simulation", wraps=job._wait_air_simulation) as waited:
                job.start(virtual=True, virtual_speed=2000)
                self.assertTrue(rec.ready.wait(20))
                self.assertIsNotNone(job._air_thread)         # 확인을 기다리는 동안 이미 시작됨
                job.confirm(air=air, checked=True)
                self.assertTrue(rec.done.wait(60))
            self.assertEqual(waited.call_count, 1 if air else 0, air)
            fin = next(d for k, d in rec.events if k == "finished")
            self.assertEqual(fin["result"]["result"], "completed", air)
            self.assertTrue(job._air_verdict.startswith(("PASS", "WARN")), air)

    def test_stop_while_waiting_for_air_simulation_cancels_without_moving(self):
        rec = Recorder()
        job = self.job(rec)
        release = threading.Event()
        job.start(virtual=True, virtual_speed=2000)
        self.assertTrue(rec.ready.wait(20))
        job._air_thread.join(60)
        job._air_thread = threading.Thread(target=release.wait, args=(30,), daemon=True)   # 아직 도는 중인 것처럼
        job._air_thread.start()
        job.confirm(air=True, checked=True)
        time.sleep(0.5)
        self.assertEqual(job.state, "confirm")                # 시뮬레이션을 기다리는 중
        job.stop()
        self.assertTrue(rec.done.wait(10))
        release.set()
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "cancelled")
        self.assertEqual(job.link.sent, [])                   # 로봇 명령은 하나도 안 감

    def test_background_air_simulation_stops_when_the_job_ends(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=2000)
        self.assertTrue(rec.ready.wait(20))
        job._air_cancel.clear()
        started = threading.Event()
        from mirobot_sketch import mirobot_sim as ms
        real = ms.simulate

        def endless(targets, step_mm=1.0, progress=None):          # 오래 걸리는 계산을 흉내: 취소만 기다림
            started.set()
            while True:
                progress(0, 1)
                time.sleep(0.01)

        job._air_thread.join(60)
        with mock.patch.object(ms, "simulate", endless):
            job._start_air_simulation(self.session.result["strokes_mm"])
            self.assertTrue(started.wait(5))
            job.cancel()                                                # 확인 단계에서 취소 -> 작업 끝
            self.assertTrue(rec.done.wait(10))
            job._air_thread.join(5)
            self.assertFalse(job._air_thread.is_alive(), "남은 공중 경로 계산이 계속 돌고 있음")

    def _finish_run(self, air=False, return_to_origin=True, link_kw=None, stop_after=None, link_cls=None):
        """가상 로봇으로 끝까지(또는 중간까지) 그린다. 반환: (job, link, finished 결과, 이벤트 기록)."""
        rec = Recorder()
        job = self.job(rec)
        holder = {}

        def open_link(cfg, virtual=False, speed=20.0, verbose=False):
            from mirobot_sketch.virtual_robot import VirtualMirobotLink
            holder["link"] = (link_cls or VirtualMirobotLink)(cfg, speed=speed, **(link_kw or {}))
            return holder["link"]

        with mock.patch.object(de, "open_link", open_link):
            job.start(virtual=True, virtual_speed=2000, return_to_origin=return_to_origin)
            self.assertTrue(rec.ready.wait(30))
            job.confirm(air=air, checked=True)
            if stop_after:
                deadline = time.monotonic() + 30
                while len(holder["link"].sent) < stop_after and not rec.done.is_set() and time.monotonic() < deadline:
                    time.sleep(0.005)
                job.stop()
            self.assertTrue(rec.done.wait(90))
        fin = next(d for k, d in rec.events if k == "finished")
        return job, holder["link"], fin, rec

    def test_pen_returns_to_the_initial_origin_after_a_completed_drawing(self):
        job, link, fin, rec = self._finish_run()
        res = fin["result"]
        self.assertEqual((res["result"], res["returned_to_origin"]), ("completed", True))
        c = job.cfg["paper_center_tcp_mm"]
        self.assertEqual(tuple(round(v, 3) for v in link.pos), (c["x"], c["y"], c["z"]))      # 처음 자세 = 종이 중심, 닿음
        self.assertIn(f"X{c['x']:.3f}", link.sent[-1])
        import json
        run = json.loads(Path(fin["record_path"]).read_text(encoding="utf-8"))
        self.assertTrue(run["return_to_origin"])
        self.assertTrue(run["returned_to_origin"])

    def test_option_off_leaves_the_pen_lifted_at_the_paper_center(self):
        job, link, fin, rec = self._finish_run(return_to_origin=False)
        c = job.cfg["paper_center_tcp_mm"]
        self.assertNotIn("returned_to_origin", fin["result"])
        self.assertNotEqual(round(link.pos[0], 3), c["x"])                                    # 펜업 위치에서 끝남
        self.assertEqual((round(link.pos[1], 3), round(link.pos[2], 3)), (c["y"], c["z"]))

    def test_air_mode_has_nothing_to_return(self):
        job, link, fin, rec = self._finish_run(air=True)
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertIsNone(fin["result"].get("returned_to_origin"))

    def test_no_automatic_movement_after_stop_or_error(self):
        job, link, fin, rec = self._finish_run(stop_after=20)
        self.assertEqual(fin["result"]["result"], "stopped_by_user")
        self.assertNotIn("returned_to_origin", fin["result"])
        job2, link2, fin2, rec2 = self._finish_run(link_kw={"fail_at": 30})
        self.assertEqual(fin2["result"]["result"], "stopped_on_error")
        self.assertNotIn("returned_to_origin", fin2["result"])
        self.assertEqual(len(link2.sent), 30)

    def test_failure_while_returning_is_reported_without_recovery(self):
        _probe_job, _probe_link, probe, _ = self._finish_run()
        n_planned = probe["result"]["commands_sent"]                                          # 그림 명령 수
        job, link, fin, rec = self._finish_run(link_kw={"fail_at": n_planned + 1})            # 되돌리기 첫 명령에서 오류
        res = fin["result"]
        self.assertEqual((res["result"], res["returned_to_origin"]), ("completed", False))
        self.assertIn("시작 위치로 돌아가는 중 오류", res["return_error"])
        self.assertEqual(len(link.sent), n_planned + 1)                                       # 그 뒤로는 보내지 않음
        done_msgs = [d["message"] for k, d in rec.events
                     if k == "step" and d["id"] == "drawing" and d["status"] == "done"]
        self.assertTrue(any("돌아가지 못했습니다" in m for m in done_msgs), done_msgs)

    def test_unexpected_error_while_returning_does_not_turn_a_finished_drawing_into_a_failure(self):
        from mirobot_sketch.virtual_robot import VirtualMirobotLink
        _job, _link, probe, _ = self._finish_run()
        n_planned = probe["result"]["commands_sent"]

        class BrokenAfterDrawing(VirtualMirobotLink):
            def send_and_ack(self, line, timeout, should_stop=None):
                if len(self.sent) >= n_planned:
                    raise RuntimeError("예상 못 한 연결 오류")
                return super().send_and_ack(line, timeout, should_stop)

        job, link, fin, rec = self._finish_run(link_cls=BrokenAfterDrawing)
        res = fin["result"]
        self.assertEqual((res["result"], res["returned_to_origin"]), ("completed", False))    # 그림은 끝난 것으로 남김
        self.assertIn("RuntimeError", res["return_error"])

    def test_state_is_active_as_soon_as_started(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertNotEqual(job.state, "idle")             # 첫 이벤트 전에 닫아도 '진행 중'으로 보이게
        self.assertTrue(rec.ready.wait(10))
        job.cancel()
        self.assertTrue(rec.done.wait(5))

    def test_session_changes_after_step1_do_not_affect_the_run(self):
        rec = Recorder()
        job = self.job(rec)
        path = self.session.result["path"]
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        saved = (self.session.result, self.session.sim)
        self.session.result, self.session.sim = None, None     # 호밍 중에 이미지를 바꾸거나 편집한 상황
        try:
            job.confirm(checked=True)
            self.assertTrue(rec.done.wait(60))
        finally:
            self.session.result, self.session.sim = saved
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")   # ①에서 찍어 둔 획으로 끝까지
        self.assertTrue(Path(fin["record_path"]).exists())
        import json
        run = json.loads(Path(fin["record_path"]).read_text(encoding="utf-8"))
        self.assertEqual(run["source"]["image"], Path(path).name)          # 기록에는 사진 파일 이름만 (폴더 경로는 뺌)
        self.assertTrue((Path(self.tmp.name) / run["strokes_json"]).exists())

    def test_progress_file_failures_never_abort_drawing(self):
        rec = Recorder()
        job = self.job(rec)
        with mock.patch.object(lp.os, "replace", side_effect=OSError("locked by antivirus")):
            job.start(virtual=True, virtual_speed=500)
            self.assertTrue(rec.ready.wait(10))
            job.confirm(checked=True)
            self.assertTrue(rec.done.wait(60))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")

    def test_port_close_error_still_finishes(self):
        rec = Recorder()
        job = self.job(rec)
        from mirobot_sketch import virtual_robot
        with mock.patch.object(virtual_robot.VirtualMirobotLink, "close", side_effect=OSError("close failed")):
            job.start(virtual=True, virtual_speed=500)
            self.assertTrue(rec.ready.wait(10))
            job.cancel()
            self.assertTrue(rec.done.wait(5))                 # finished가 와야 GUI 잠금이 풀림

    def test_rviz_unavailable_does_not_block_drawing(self):
        rec = Recorder()

        def no_rviz(*a, **k):
            from mirobot_sketch.rviz_launch import RvizUnavailable
            raise RvizUnavailable("ROS 없음")

        job = self.job(rec, launch_rviz=no_rviz)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.confirm(checked=True)
        self.assertTrue(rec.done.wait(60))
        self.assertTrue(any(k == "rviz" and not d["ok"] for k, d in rec.events))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")


if __name__ == "__main__":
    unittest.main()
