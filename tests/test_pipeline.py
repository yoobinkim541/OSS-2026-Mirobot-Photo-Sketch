"""CV 파이프라인과 종이 좌표 변환 테스트.

    python -m unittest discover -s tests -v
"""

import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mirobot_sketch import paper_mapping as pm  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402


def blank(h=200, w=300):
    return np.zeros((h, w), np.uint8)


class TraceStrokesTest(unittest.TestCase):
    def test_straight_line_is_traced_once(self):
        # 문제 1 재현: findContours는 1px 선을 왕복해서 길이가 두 배가 됨
        e = blank(50, 300)
        cv2.line(e, (10, 25), (290, 25), 255, 1)
        contour = sp.extract_strokes_contour(e, 5)
        self.assertGreater(sp.polyline_length(contour[0]), 500)

        strokes = sp.trace_strokes(e, 5)
        self.assertEqual(len(strokes), 1)
        self.assertAlmostEqual(sp.polyline_length(strokes[0]), 280, delta=2)

    def test_circle_becomes_one_closed_stroke(self):
        e = blank(200, 200)
        cv2.circle(e, (100, 100), 60, 255, 1)
        strokes = sp.trace_strokes(e, 5)
        self.assertEqual(len(strokes), 1)
        self.assertTrue(sp.is_closed(strokes[0]))

    def test_t_junction_splits_into_three(self):
        e = blank(200, 200)
        cv2.line(e, (20, 50), (180, 50), 255, 1)
        cv2.line(e, (100, 50), (100, 180), 255, 1)
        strokes = sp.trace_strokes(e, 5)
        self.assertEqual(len(strokes), 3)
        total = sum(sp.polyline_length(s) for s in strokes)
        self.assertAlmostEqual(total, 160 + 130, delta=6)

    def test_small_components_removed_but_letters_kept(self):
        # 문제 3 재현: 조각 단위 길이 필터가 분기점 많은 글자를 지웠음
        img = np.full((120, 400), 255, np.uint8)
        cv2.putText(img, "MIROBOT", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
        cv2.circle(img, (380, 20), 1, 0, -1)  # 점 잡음
        mask = sp.compute_dark_mask(img)
        letters = 255 - img

        def coverage(strokes):
            drawn = sp.draw_strokes_image(strokes, img.shape, thickness=5)
            return np.count_nonzero(drawn & letters) / np.count_nonzero(letters), drawn

        ceiling, _ = coverage(sp.trace_skeleton(sp.skeletonize_edges(mask)))  # 필터 없음
        got, drawn = coverage(sp.trace_strokes(mask, min_length_px=15))
        # 수정 전(조각 단위 필터)은 0.66 / 필터 없음 0.92 수준이었음
        self.assertGreater(got, 0.95 * ceiling)
        self.assertEqual(drawn[20, 380], 0)  # 잡음 점은 그리지 않음

    def test_dark_mode_gives_single_centerline_for_thick_line(self):
        # 문제 4 재현: Canny는 굵은 선 하나를 두 줄로 만듦
        img = np.full((100, 300), 255, np.uint8)
        cv2.line(img, (20, 50), (280, 50), 0, 6)
        _, canny_strokes = sp.run_pipeline(img, line_source="canny")
        _, dark_strokes = sp.run_pipeline(img, line_source="dark")
        canny_len = sum(sp.polyline_length(s) for s in canny_strokes)
        dark_len = sum(sp.polyline_length(s) for s in dark_strokes)
        self.assertGreater(canny_len, 450)
        self.assertLess(dark_len, 290)


class MergeStrokesTest(unittest.TestCase):
    def test_t_junction_three_strokes_become_two(self):
        # 끝점 3개(홀수) + 분기점(차수 3, 홀수) -> 홀수 노드 4개 -> 최소 2획
        e = blank(200, 200)
        cv2.line(e, (20, 50), (180, 50), 255, 1)
        cv2.line(e, (100, 50), (100, 180), 255, 1)
        raw = sp.trace_strokes(e, 5)
        merged = sp.merge_strokes(raw)
        self.assertEqual(len(raw), 3)
        self.assertEqual(len(merged), 2)

    def test_cross_continues_straight(self):
        # "+" 모양: 교차점에서 가장 덜 꺾이는 쪽으로 이어야 가로 1획 + 세로 1획
        e = blank(200, 200)
        cv2.line(e, (20, 100), (180, 100), 255, 1)
        cv2.line(e, (100, 20), (100, 180), 255, 1)
        merged = sp.merge_strokes(sp.trace_strokes(e, 5))
        self.assertEqual(len(merged), 2)
        for s in merged:
            span = s.max(axis=0) - s.min(axis=0)
            self.assertLess(min(span), 4)  # 각 획이 한 방향으로 곧음

    def test_merging_draws_the_same_pixels(self):
        img = np.full((300, 400), 255, np.uint8)
        cv2.putText(img, "MIROBOT", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 0, 5)
        raw = sp.trace_strokes(sp.compute_dark_mask(img), 15)
        merged = sp.merge_strokes(raw)
        a = sp.draw_strokes_image(raw, img.shape)
        b = sp.draw_strokes_image(merged, img.shape)
        self.assertLess(len(merged), len(raw))
        self.assertLess(np.count_nonzero(a != b), 0.01 * np.count_nonzero(a))


class DedupeTest(unittest.TestCase):
    def test_close_parallel_lines_become_one(self):
        e = blank(100, 300)
        cv2.line(e, (20, 50), (280, 50), 255, 1)
        cv2.line(e, (20, 53), (280, 53), 255, 1)   # 3px 옆 = 굵은 선의 반대쪽 경계
        out = sp.dedupe_strokes(sp.trace_strokes(e, 5), e.shape, dist_px=3)
        self.assertAlmostEqual(sum(sp.polyline_length(s) for s in out), 260, delta=10)

    def test_separate_lines_are_kept(self):
        e = blank(100, 300)
        cv2.line(e, (20, 40), (280, 40), 255, 1)
        cv2.line(e, (20, 60), (280, 60), 255, 1)   # 20px 떨어진 별개의 선
        out = sp.dedupe_strokes(sp.trace_strokes(e, 5), e.shape, dist_px=3)
        self.assertAlmostEqual(sum(sp.polyline_length(s) for s in out), 520, delta=10)

    def test_thick_line_drawn_once_by_default_pipeline(self):
        img = np.full((100, 300), 255, np.uint8)
        cv2.line(img, (20, 50), (280, 50), 0, 2)   # Canny 경계 두 줄이 4px 떨어져 잡힘
        _, st = sp.run_pipeline(img)
        self.assertLess(sum(sp.polyline_length(s) for s in st), 330)
        _, st_off = sp.run_pipeline(img, dedupe_px=0)
        self.assertGreater(sum(sp.polyline_length(s) for s in st_off), 450)

    def test_ink_between_parallel_edges_become_one_but_a_light_gap_stays(self):
        gray = np.full((80, 300), 240, np.uint8)
        gray[46:54, 20:280] = 20                          # 8px 굵은 먹. 양쪽 경계 사이는 어둡다
        edges = np.zeros_like(gray)
        edges[46, 20:280] = 255
        edges[53, 20:280] = 255
        twins = sp.drop_ink_twins(sp.trace_strokes(edges, 5), gray, gap_px=12)
        self.assertLess(sum(sp.polyline_length(s) for s in twins), 320)
        light = np.full((80, 300), 240, np.uint8)       # 사이가 밝은 두 선 (머리카락)
        edges2 = np.zeros_like(light)
        edges2[40, 20:280] = 255
        edges2[48, 20:280] = 255
        kept = sp.drop_ink_twins(sp.trace_strokes(edges2, 5), light, gap_px=12)
        self.assertGreater(sum(sp.polyline_length(s) for s in kept), 480)
class ResizeTest(unittest.TestCase):
    def test_small_image_is_upscaled_to_max_side(self):
        # 저해상도 원본도 긴 변 800px로 맞춰야 px 파라미터 의미가 같고 선이 매끄러움
        small = np.zeros((267, 189), np.uint8)
        self.assertEqual(sp.resize_max_side(small).shape, (800, 566))
        self.assertEqual(sp.resize_max_side(small, upscale=False).shape, (267, 189))

    def test_large_image_is_downscaled(self):
        big = np.zeros((1200, 2000), np.uint8)
        self.assertEqual(sp.resize_max_side(big).shape, (480, 800))


class MedianPrefilterTest(unittest.TestCase):
    def test_median_removes_screentone_but_keeps_line(self):
        # 망점(스크린톤) 영역 + 굵은 선 하나
        img = np.full((200, 300), 255, np.uint8)
        for y in range(20, 180, 6):          # 지름 5px, 6px 간격 망점
            for x in range(20, 140, 6):
                cv2.circle(img, (x, y), 2, 0, -1)
        cv2.line(img, (170, 30), (280, 170), 0, 5)
        _, plain = sp.run_pipeline(img, 80, 200, 5, 40, 3.0)
        _, filt = sp.run_pipeline(img, 80, 200, 5, 40, 3.0, median_ksize=7)
        self.assertGreater(len(plain), len(filt))
        self.assertTrue(any(sp.polyline_length(s) > 150 for s in filt))  # 선은 남음


class SimplifyAndOrderTest(unittest.TestCase):
    def test_simplify_keeps_closed_loop_closed(self):
        e = blank(200, 200)
        cv2.circle(e, (100, 100), 60, 255, 1)
        s = sp.simplify_strokes(sp.trace_strokes(e, 5), 2.0)[0]
        self.assertTrue(sp.is_closed(s))
        self.assertLess(len(s), 40)

    def test_ordering_reduces_pen_up_travel(self):
        rng = np.random.default_rng(0)
        strokes = []
        for _ in range(30):
            p = rng.uniform(0, 500, 2)
            strokes.append(np.array([p, p + rng.uniform(-20, 20, 2)]))
        before = sp.stroke_metrics(strokes, start=(250, 250))["pen_up_length"]
        after = sp.stroke_metrics(sp.order_strokes(strokes, start=(250, 250)), start=(250, 250))["pen_up_length"]
        self.assertLess(after, before * 0.6)

    def test_ordering_can_reverse_open_stroke(self):
        s = np.array([[100.0, 0.0], [0.0, 0.0]])
        ordered = sp.order_strokes([s], start=(0, 0))
        self.assertEqual(tuple(ordered[0][0]), (0.0, 0.0))


class PaperMappingTest(unittest.TestCase):
    def test_center_aspect_and_y_flip(self):
        # 200x100 px 사각형, 위쪽 변이 이미지 y=10
        strokes = [np.array([[0, 10], [200, 10], [200, 110], [0, 110], [0, 10]])]
        mm, placement = pm.pixels_to_paper(strokes, box_mm=(100, 100))
        pts = mm[0]
        self.assertAlmostEqual(placement["drawing_width_mm"], 100)
        self.assertAlmostEqual(placement["drawing_height_mm"], 50)
        self.assertAlmostEqual(pts[:, 0].min() + pts[:, 0].max(), 0)  # 가운데 정렬
        self.assertAlmostEqual(pts[:, 1].min() + pts[:, 1].max(), 0)
        self.assertAlmostEqual(pts[0][1], 25)  # 이미지 위쪽 -> 종이 위쪽(+)
        self.assertEqual(pm.check_within_paper(mm), 0)

    def test_paper_preview_size_and_ink(self):
        line = [np.array([[-40.0, 0.0], [40.0, 0.0]])]
        img = pm.render_paper_preview(line, line_width_mm=0.5, px_per_mm=4.0)
        self.assertEqual(img.shape, (840, 1188, 3))            # A4 297x210mm x 4px/mm
        column = img[:, 594, 0]                                 # 가운데 세로줄
        dark = np.nonzero(column < 128)[0]
        self.assertTrue(abs(dark.mean() - 420) < 2)             # 선이 종이 중심 높이에
        self.assertLessEqual(len(dark), 3)                      # 0.5mm = 2px 굵기 (+안티앨리어싱)

    def test_box_larger_than_paper_rejected(self):
        with self.assertRaises(ValueError):
            pm.pixels_to_paper([np.array([[0, 0], [10, 10]])], box_mm=(300, 100))

    def test_document_roundtrip(self):
        import tempfile

        mm, placement = pm.pixels_to_paper([np.array([[0, 0], [10, 0], [10, 10]])])
        doc = pm.build_strokes_document(mm, placement)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "s.json"
            pm.save_strokes_json(p, doc)
            loaded, strokes = pm.load_strokes_json(p)
        self.assertEqual(loaded["kind"], "sketch_strokes")
        self.assertTrue(np.allclose(strokes[0], mm[0], atol=1e-3))
        self.assertTrue(math.isclose(abs(strokes[0][0][0]), 50, abs_tol=1e-3))


class LabEdgesTest(unittest.TestCase):
    def test_lab_finds_boundary_between_colors_of_equal_brightness(self):
        # 흑백으로 바꾸면 밝기가 같은 두 색(주황빛 / 청록빛): 흑백 Canny는 경계를 못 찾음
        img = np.zeros((200, 200, 3), np.uint8)
        img[:, :100] = (0, 100, 200)     # BGR, 흑백 ≈ 118.5
        img[:, 100:] = (255, 152, 0)     # BGR, 흑백 ≈ 118.3
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        luma = sp.compute_edges(gray, 30, 100, 5)
        lab = sp.compute_edges_lab(img, 30, 100, 5)
        self.assertLess(int((luma > 0).sum()), 20)
        self.assertGreater(int((lab[:, 90:110] > 0).sum()), 150)   # 세로 경계 200px 대부분

    def test_lab_on_gray_image_matches_brightness_edges_roughly(self):
        img = np.full((200, 200), 255, np.uint8)
        cv2.circle(img, (100, 100), 50, 0, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        a = (sp.compute_edges(img, 30, 100, 5) > 0).sum()
        b = (sp.compute_edges_lab(bgr, 30, 100, 5) > 0).sum()
        self.assertLess(abs(int(a) - int(b)), 0.3 * a)


class DiscardedTest(unittest.TestCase):
    def test_trace_records_small_blobs_and_spurs(self):
        img = np.zeros((200, 200), np.uint8)
        cv2.circle(img, (100, 100), 50, 255, 1)
        cv2.line(img, (10, 10), (13, 10), 255, 1)          # 4px 덩어리 -> small
        cv2.line(img, (150, 100), (154, 100), 255, 1)      # 원에 붙은 5px 잔가지 -> spur
        disc = []
        kept = sp.trace_strokes(img, min_length_px=15, spur_px=6, discarded=disc)
        reasons = {r for _, r in disc}
        self.assertIn("small", reasons)
        self.assertIn("spur", reasons)
        self.assertEqual([np.asarray(s).tolist() for s in sp.trace_strokes(img, 15, 6)],
                         [np.asarray(s).tolist() for s in kept])            # 기록해도 결과는 같음

    def test_dedupe_records_overlap_pieces(self):
        a = np.array([[10, 50], [190, 50]])
        b = np.array([[10, 52], [190, 52]])               # 2px 옆 이중선
        disc = []
        out = sp.dedupe_strokes([a, b], (100, 200), 4, discarded=disc)
        self.assertTrue(disc)
        self.assertEqual({r for _, r in disc}, {"overlap"})
        plain = sp.dedupe_strokes([a, b], (100, 200), 4)
        self.assertEqual([p.tolist() for p in out], [p.tolist() for p in plain])


if __name__ == "__main__":
    unittest.main()
