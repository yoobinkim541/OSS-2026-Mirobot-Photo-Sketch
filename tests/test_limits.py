"""드로잉 허용 영역: 지붕 모양 넓은 범위, 예전 대칭 형식, 배치 맞추기, 시뮬레이터로 도달 확인."""

import copy
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import limits  # noqa: E402

ROOF = {"x_max_mm": 125, "y_min_mm": -85,
        "roof_mm": [[0, 57.5], [50, 57.5], [60, 55], [80, 55], [90, 52.5], [100, 50], [110, 47.5], [120, 45],
                    [125, 42.5]]}


class RegionTest(unittest.TestCase):
    def test_roof_region(self):
        r = limits.Region.from_cfg(ROOF)
        self.assertTrue(r.contains(0, 57.5))
        self.assertFalse(r.contains(0, 58))
        self.assertTrue(r.contains(125, 42.5))
        self.assertFalse(r.contains(125, 43))
        self.assertTrue(r.contains(-55, 56.2))                 # 50~60 사이는 직선: 56.25
        self.assertFalse(r.contains(-55, 56.4))
        self.assertTrue(r.contains(0, -85))
        self.assertFalse(r.contains(0, -85.1))
        self.assertFalse(r.contains(125.1, 0))
        self.assertAlmostEqual(r.top(55), 56.25)

    def test_legacy_symmetric_limits(self):
        r = limits.Region.from_cfg({"max_abs_paper_x_mm": 50.0, "max_abs_paper_y_mm": 50.0})
        self.assertTrue(r.contains(50, -50))
        self.assertFalse(r.contains(50.1, 0))
        self.assertFalse(r.contains(0, 50.1))
        self.assertEqual((r.x_max, r.y_min, r.top(0)), (50, -50, 50))
        pts = np.array(r.outline())
        np.testing.assert_allclose(pts.min(0), [-50, -50])
        np.testing.assert_allclose(pts.max(0), [50, 50])

    def test_outline_is_closed_polygon_on_the_boundary(self):
        r = limits.Region.from_cfg(ROOF)
        pts = r.outline()
        self.assertEqual(tuple(pts[0]), tuple(pts[-1]))
        for x, y in pts:
            self.assertTrue(r.contains(x, y))

    def test_fit_keeps_center_moves_down_or_refuses(self):
        r = limits.Region.from_cfg(ROOF)
        self.assertEqual(r.fit(100, 100), (0.0, 0.0))           # 가운데에 들어감 → 그대로
        cx, cy = r.fit(250, 120)                                # 위가 +42.5까지라 아래로 내림
        self.assertEqual(cx, 0.0)
        self.assertAlmostEqual(cy + 60, 42.5)                   # 위쪽 끝이 지붕에 닿을 만큼만 내림
        self.assertIsNone(r.fit(250, 130))                      # 높이 127.5를 넘음
        self.assertIsNone(r.fit(260, 50))                       # 폭 250을 넘음

    def test_effective_long_side(self):
        r = limits.Region.from_cfg(ROOF)
        self.assertAlmostEqual(limits.effective_long_mm(r, 100, 1156, 1440), 100)      # 요청대로 들어감
        self.assertAlmostEqual(limits.effective_long_mm(r, 250, 1156, 1440), 140.86, delta=0.05)  # 세로: 폭 114에서 지붕 55.75
        self.assertAlmostEqual(limits.effective_long_mm(r, 250, 2000, 1000), 250, delta=0.1)     # 가로: 폭까지

    def test_max_scale(self):
        r = limits.Region.from_cfg(ROOF)
        s = r.max_scale(2.0, 1.0)                               # 2:1 가로 그림
        self.assertIsNotNone(r.fit(2 * s, s))
        self.assertIsNone(r.fit(2 * s * 1.01, s * 1.01))
        self.assertAlmostEqual(2 * s, 250, delta=0.5)           # 폭이 먼저 닿음


class ConfigTest(unittest.TestCase):
    def test_bundled_config_uses_roof_region(self):
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        r = limits.pending_region(cfg)
        self.assertEqual(r.x_max, 125)
        self.assertEqual(limits.executor_region(cfg).x_max, 50)          # 실물 확인 범위는 그대로
        repo = de.load_config(ROOT / "robot" / "drawing_config.json")
        self.assertEqual(repo["limits_pending_verification"], cfg["limits_pending_verification"])

    def test_untouched_old_default_is_migrated(self):
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        old = copy.deepcopy(cfg)
        old["limits_pending_verification"] = {"max_abs_paper_x_mm": 60.0, "max_abs_paper_y_mm": 60.0,
                                              "status": "x"}
        self.assertEqual(limits.pending_region(limits.migrate(old)).x_max, 125)
        custom = copy.deepcopy(old)
        custom["limits_pending_verification"]["max_abs_paper_x_mm"] = 55.0      # 사용자가 바꾼 값은 그대로
        self.assertEqual(limits.pending_region(limits.migrate(custom)).x_max, 55)

    def test_executor_checks_roof(self):
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        inside = [[(-120, -80), (120, -80), (120, 44), (0, 57)]]
        outside = [[(0, 0), (0, 58)]]
        self.assertEqual(de.check_limits(inside, cfg, pending=True), [])
        self.assertEqual(len(de.check_limits(outside, cfg, pending=True)), 1)
        self.assertEqual(len(de.check_limits(inside, cfg)), 4)                  # 실물 확인 범위(±50) 밖


def rect_px(w, h):
    return [np.array([[0, 0], [w, 0], [w, h], [0, h], [0, 0]], float)]


class PlacementTest(unittest.TestCase):
    def setUp(self):
        from mirobot_sketch import paper_mapping as pm
        self.pm = pm
        self.region = limits.Region.from_cfg(ROOF)

    def bbox(self, strokes_mm):
        p = np.vstack(strokes_mm)
        return p.min(0), p.max(0)

    def test_100mm_unchanged(self):
        a, pa = self.pm.pixels_to_paper(rect_px(400, 300), box_mm=(100, 100))
        b, pb = self.pm.pixels_to_paper(rect_px(400, 300), box_mm=(100, 100), region=self.region)
        np.testing.assert_allclose(np.vstack(a), np.vstack(b))
        self.assertEqual(pb["offset_mm"], [0.0, 0.0])

    def test_wide_drawing_moves_down_into_region(self):
        s, pl = self.pm.pixels_to_paper(rect_px(800, 400), box_mm=(250, 250), region=self.region)
        lo, hi = self.bbox(s)
        self.assertAlmostEqual(hi[0] - lo[0], 250, delta=0.01)
        self.assertLess(pl["offset_mm"][1], 0)                     # 아래로 내림
        for x, y in np.vstack(s):
            self.assertTrue(self.region.contains(x, y), (x, y))
        self.assertEqual(pl["transform"]["oy"], pl["offset_mm"][1])

    def test_tall_drawing_is_shrunk_to_fit(self):
        s, pl = self.pm.pixels_to_paper(rect_px(300, 600), box_mm=(250, 250), region=self.region)
        lo, hi = self.bbox(s)
        self.assertAlmostEqual(hi[1] - lo[1], 57.5 + 85, delta=0.5)    # 가운데 지붕 높이까지
        for x, y in np.vstack(s):
            self.assertTrue(self.region.contains(x, y))

    def test_session_big_drawing_round_trip(self):
        import tempfile
        import cv2
        from mirobot_sketch.session import SketchSession
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "wide.png"
            img = np.full((300, 800), 255, np.uint8)
            cv2.rectangle(img, (20, 20), (780, 280), 0, 3)
            cv2.line(img, (20, 150), (780, 150), 0, 3)
            cv2.imwrite(str(p), img)
            s = SketchSession(de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json"))
            s.set_image(p)
            s.update_params({"box_mm": 250})
            self.assertEqual(s.params["box_mm"], 250)                  # 상한 = 영역 폭
            s.run_current()
            r = limits.pending_region(s.cfg)
            pts = np.vstack(s.result["strokes_mm"])
            self.assertGreater(pts[:, 0].max() - pts[:, 0].min(), 200)
            self.assertTrue(all(r.contains(x, y) for x, y in pts))
            self.assertEqual(s.result["out_of_pending"], 0)
            px = s.result["strokes_px"][0][0]
            mm = s.px_to_mm([px])[0]
            np.testing.assert_allclose(s.mm_to_px(mm.tolist()), px, atol=1e-6)
            self.assertIn("pending_limit_mm", s.state())


class ReachSimulationTest(unittest.TestCase):
    """넓은 범위 전체가 실행기와 같은 계획으로 도달 가능하고 관절 여유가 남는지 (실물 확인 전 안전장치)."""

    def test_region_border_and_grid_are_reachable(self):
        from mirobot_sketch import mirobot_sim as ms
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        r = limits.pending_region(cfg)
        strokes = [[tuple(p) for p in r.outline()]]
        for y in np.arange(r.y_min, r.top(0) + 0.01, 20):                    # 가로 줄
            xs = [x for x in np.arange(-r.x_max, r.x_max + 0.01, 5) if r.contains(x, y)]
            strokes.append([(xs[0], y), (xs[-1], y)])
        res = ms.simulate(ms.plan_targets(strokes, cfg), step_mm=2.0)
        self.assertEqual(res["failures"], [])
        self.assertEqual(res["violations"], [])
        self.assertGreaterEqual(float(res["min_margin_deg"].min()), 5.0)


if __name__ == "__main__":
    unittest.main()


class ReviewFixTest(unittest.TestCase):
    def test_bundled_roof_is_concave_so_segments_stay_inside(self):
        # 실행기는 꼭짓점만 검사 → 영역이 볼록해야 "꼭짓점이 안이면 선분도 안"이 성립
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        r = limits.pending_region(cfg)
        roof = cfg["limits_pending_verification"]["roof_mm"]
        slopes = [(y1 - y0) / (x1 - x0) for (x0, y0), (x1, y1) in zip(roof, roof[1:])]
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(slopes, slopes[1:])), slopes)
        pts = r.outline()
        for a in pts:
            for b in pts:
                for t in np.linspace(0, 1, 21):
                    x, y = np.add(a, np.subtract(b, a) * t)
                    self.assertTrue(r.contains(x, y, eps=1e-6), (a, b, t))

    def test_agent_schema_allows_big_drawings(self):
        from mirobot_sketch.agent import tools
        self.assertEqual(tools.param_schema()["box_mm"]["maximum"], 250)

    def test_face_pen_width_uses_actual_drawing_size(self):
        from mirobot_sketch import stages
        cfg = de.load_config(ROOT / "mirobot_sketch" / "data" / "drawing_config.json")
        prev = {"gray": np.zeros((800, 642), np.uint8), "region": limits.pending_region(cfg)}
        p = {"pen_mm": 0.5, "box_mm": 250}
        eff = limits.effective_long_mm(prev["region"], 250, 642, 800)          # 세로 그림: 약 141mm로 줄어듦
        self.assertAlmostEqual(stages._face_pen_px(prev, p), 0.5 / (eff / 800))
        self.assertAlmostEqual(stages._face_pen_px({"gray": prev["gray"]}, p), 0.5 / (250 / 800))   # 영역 모름
