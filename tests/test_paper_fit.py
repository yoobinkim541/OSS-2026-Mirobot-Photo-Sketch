"""종이 맞춤: 모서리 표시로 잰 종이 자리·회전·크기에 맞춰 로봇이 그릴 좌표만 바꾸고, 종이 좌표(편집·미리보기)는 그대로."""

import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import limits  # noqa: E402
from mirobot_sketch.draw_job import DrawJob  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

import golden  # noqa: E402


class PaperFitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        cls.pic = cls.root / "line.png"
        cv2.imwrite(str(cls.pic), golden.synthetic_images()["line"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def session(self, box_mm=40):
        s = SketchSession(golden.default_cfg())
        s.set_image(self.pic)
        s.update_params({"box_mm": box_mm})
        s.run_current()
        return s

    def test_fit_moves_rotates_and_scales_only_what_the_robot_draws(self):
        s = self.session()
        paper = [np.array(p) for p in s.result["strokes_mm"]]
        s.sim = {"summary": {}, "raw": None}
        used = s.set_paper_fit({"cx": 6.0, "cy": -4.0, "deg": 90.0, "scale": 0.5})
        self.assertEqual(used["scale"], 0.5)
        x, y = paper[0][0]
        np.testing.assert_allclose(s.robot_strokes_mm()[0][0], [6.0 - 0.5 * y, -4.0 + 0.5 * x], atol=1e-9)
        for a, b in zip(paper, s.result["strokes_mm"]):
            np.testing.assert_array_equal(a, b)                  # 편집·에이전트·종이 미리보기 좌표는 그대로
        self.assertIsNone(s.sim)                                 # 로봇이 그릴 자리가 바뀌었으니 다시 시뮬레이션
        self.assertIs(s.robot_strokes_mm(), s.robot_strokes_mm())  # 화면 겹침이 매번 다시 계산하지 않게

    def test_time_estimate_follows_the_fitted_strokes_and_the_pen_lift_actually_used(self):
        s = self.session()
        robot = lambda: [[tuple(p) for p in q] for q in s.robot_strokes_mm()]
        self.assertEqual(s.result["timing"], de.estimate_time(robot(), s.cfg))       # 저장된 시작 자세가 없으면 설정값
        s.set_paper_fit({"cx": 0.0, "cy": 0.0, "deg": 0.0, "scale": 0.5})
        self.assertEqual(s.result["timing"], de.estimate_time(robot(), s.cfg))       # 절반 크기로 그리면 시간도 그 획으로
        s.cfg = {**s.cfg, "requested_start_pose": {"joints_deg": [0] * 6}}
        s.set_paper_fit(None)
        self.assertEqual(s.result["timing"]["up_clearance_mm"], 20.0)               # 그리기 창이 쓰는 펜 올림 높이
        self.assertGreater(s.result["timing"]["pen_lift_s"], de.estimate_time(robot(), s.cfg)["pen_lift_s"])

    def test_placement_shrinks_until_the_points_sent_after_the_warp_fit_the_range(self):
        from mirobot_sketch import paper_mapping as pm
        region = limits.pending_region(golden.default_cfg())
        rect = [np.array([[0, 0], [400, 0], [400, 200], [0, 200], [0, 0]], float)]
        warp = lambda x, y: (1.3 * x, y)                                   # 가로를 넓혀 보내는 보정
        _, plain = pm.pixels_to_paper(rect, box_mm=(250, 250), region=region)
        st, fitted = pm.pixels_to_paper(rect, box_mm=(250, 250), region=region, warp=warp)
        self.assertTrue(all(region.contains(*warp(x, y)) for x, y in np.vstack(st)))
        self.assertLess(fitted["drawing_width_mm"], plain["drawing_width_mm"])
        st2, same = pm.pixels_to_paper(rect, box_mm=(60, 60), region=region, warp=warp)
        self.assertEqual(same["drawing_width_mm"], 60.0)                   # 이미 들어가면 그대로

    def test_placement_with_a_bowing_warp_passes_the_executor_check_on_every_point(self):
        from mirobot_sketch import paper_mapping as pm
        cfg = golden.default_cfg()
        cfg["drawing_warp"] = {"x": [1.0, 0.0, 0.0, 0.0, 0.0], "y": [0.0, 1.0, -0.002, 0.0, 0.0]}   # 양옆이 내려가 가운데가 가장 높음
        warp = de.drawing_warp(cfg)
        region = limits.pending_region(cfg)
        t = np.linspace(0, 1, 200)[:, None]
        corners = np.array([[0, 0], [400, 0], [400, 200], [0, 200], [0, 0]], float)
        rect = [np.vstack([a + (b - a) * t for a, b in zip(corners[:-1], corners[1:])])]   # 촘촘한 테두리 획
        st, _ = pm.pixels_to_paper(rect, box_mm=(250, 250), region=region,
                                   warp=lambda x, y: de.warp_xy(warp, x, y))
        self.assertEqual(de.check_limits([[tuple(p) for p in s] for s in st], cfg, pending=True), [])

    def test_no_fit_means_the_original_position(self):
        s = self.session()
        s.set_paper_fit({"cx": 6.0, "cy": 0.0, "deg": 0.0, "scale": 1.0})
        s.set_paper_fit(None)
        self.assertIs(s.robot_strokes_mm(), s.result["strokes_mm"])

    def test_fit_outside_the_pen_range_is_shrunk_around_the_paper_center(self):
        s = self.session(box_mm=60)
        region = limits.executor_region(s.cfg)
        used = s.set_paper_fit({"cx": region.x_max - 15.0, "cy": 0.0, "deg": 0.0, "scale": 1.0})
        self.assertLess(used["range_scale"], 1.0)
        self.assertGreaterEqual(used["range_scale"], 0.3)
        pts = np.vstack(s.robot_strokes_mm())
        self.assertTrue(all(region.contains(x, y) for x, y in pts))
        self.assertEqual(s.result["out_of_limits"], 0)

    def test_paper_far_outside_the_range_falls_back_to_the_original_position(self):
        s = self.session()
        self.assertIsNone(s.set_paper_fit({"cx": 400.0, "cy": 0.0, "deg": 0.0, "scale": 1.0}))
        self.assertIn("원래 위치", s.paper_fit_error)
        self.assertIs(s.robot_strokes_mm(), s.result["strokes_mm"])

    def test_virtual_drawing_sends_the_fitted_strokes_and_records_the_fit(self):
        s = self.session()
        s.set_paper_fit({"cx": 5.0, "cy": -3.0, "deg": 2.0, "scale": 0.8})
        expected = [np.array(p) for p in s.robot_strokes_mm()]
        ready, finished, events = threading.Event(), threading.Event(), []

        def callback(kind, **data):
            events.append((kind, data))
            if kind == "step" and data["id"] == "confirm" and data["status"] == "active":
                ready.set()
            if kind == "finished":
                finished.set()

        job = DrawJob(s, s.cfg, callback, progress_path=self.root / "progress.json")
        with mock.patch.object(de.paths, "runs_dir", return_value=self.root / "runs"), \
                mock.patch.object(de.paths, "output_dir", return_value=self.root), \
                mock.patch.object(job, "_open_rviz"):
            job.start(virtual=True, virtual_speed=5000, return_to_origin=False)
            self.assertTrue(ready.wait(60), events[-3:])
            job.confirm(air=True, checked=True)
            self.assertTrue(finished.wait(60))
        fin = next(d for k, d in events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        record = json.loads(Path(fin["record_path"]).read_text(encoding="utf-8"))
        self.assertAlmostEqual(record["paper_fit"]["scale"], 0.8)
        sent = json.loads((self.root / Path(record["strokes_json"]).name).read_text(encoding="utf-8"))["strokes"]
        got = [np.array(st["points_xy_mm"]) for st in sent]
        self.assertEqual(len(got), len(expected))
        for a, b in zip(got, expected):
            np.testing.assert_allclose(a, b, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
