"""실행 작업(가상 시뮬레이션): 단계 순서, 확인 전 대기, 멈춤, 기록, 진행 파일."""

import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import live_progress as lp  # noqa: E402
from mirobot_sketch.draw_job import DrawJob, target_contact_extents  # noqa: E402
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
        strokes_file = Path(run["strokes_json"])
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
        threading.Timer(0.5, job.stop).start()
        self.assertTrue(rec.done.wait(5))
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
            job.start(virtual=False)
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

    def _offset_run(self, answer, dx=-18.0):
        """실물 연결에서 호밍 자세가 종이 중심 설정에서 dx mm 벗어난 상황에서 사람이 answer로 답한다."""
        import time
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
        with mock.patch.object(de, "open_link", return_value=link), \
             mock.patch.object(de, "connect_and_home", return_value=tcp), \
             mock.patch.object(calibration, "save_config") as save:
            job.start(virtual=False)
            deadline = time.monotonic() + 15
            while not any(k == "start_offset" for k, _ in rec.events) and time.monotonic() < deadline:
                time.sleep(0.02)
            offer = next(d for k, d in rec.events if k == "start_offset")
            self.assertEqual(job.state, "start")                 # 답하기 전에는 다음 단계로 가지 않음
            job.answer_start_offset(answer)
            if answer:
                self.assertTrue(rec.ready.wait(20), rec.events[-5:])
                job.cancel()
            self.assertTrue(rec.done.wait(10), rec.events[-5:])
        return cfg, c, tcp, save, offer, rec, link

    def test_start_offset_can_become_paper_center_after_human_confirms_pen_contact(self):
        cfg, before, tcp, save, offer, rec, link = self._offset_run(True)
        self.assertEqual(offer["tcp"], {"x": tcp[0], "y": tcp[1], "z": tcp[2]})
        self.assertEqual(offer["previous"], before)
        save.assert_called_once()
        self.assertEqual(save.call_args.args[1]["paper_center_tcp_mm"], offer["tcp"])
        self.assertEqual(cfg["paper_center_tcp_mm"], offer["tcp"])       # 이어지는 계획도 새 중심을 씀
        done = [i for i, st in rec.steps() if st == "done"]
        self.assertIn("start", done)
        self.assertIn("confirm", [i for i, st in rec.steps() if st == "active"])
        self.assertTrue(link.closed)

    def test_declining_start_offset_keeps_config_and_does_not_start(self):
        cfg, before, tcp, save, offer, rec, link = self._offset_run(False)
        save.assert_not_called()
        self.assertEqual(cfg["paper_center_tcp_mm"], before)
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "not_started")
        self.assertIn("떨어져 있습니다", fin["result"]["error"])
        self.assertTrue(link.closed)

    def test_virtual_run_never_offers_to_move_paper_center(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.cancel()
        self.assertTrue(rec.done.wait(5))
        self.assertFalse(any(k == "start_offset" for k, _ in rec.events))

    def test_photo_sized_rectangle_can_be_selected_for_contact_calibration(self):
        outline = [[(-60, -35.4), (60, -35.4), (60, 35.4), (-60, 35.4)]]
        self.assertEqual(target_contact_extents(outline, golden.default_cfg()), (61, 37))
        square = [[(-60, -60), (60, -60), (60, 60), (-60, 60)]]
        self.assertIsNone(target_contact_extents(square, golden.default_cfg()))

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
                       "samples": [], "selected_half_size_mm": [61.0, 37.0],
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
            job.start(virtual=False, pending=True, recalibrate=True)
            self.assertTrue(rec.ready.wait(25), rec.events[-5:])
            self.assertEqual(run.call_args.kwargs["target_half_extents_mm"], (61, 37))
            self.assertEqual(cfg["limits"], {"max_abs_paper_x_mm": 61.0,
                                              "max_abs_paper_y_mm": 37.0})
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
        self.assertEqual(run["source"]["image"], path)
        self.assertTrue(Path(run["strokes_json"]).exists())

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
