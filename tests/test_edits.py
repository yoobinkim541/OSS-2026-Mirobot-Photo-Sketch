"""획 편집 함수와 모양 기반 재적용."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import edits  # noqa: E402

LINE = np.array([[0, 0], [10, 0], [20, 0], [30, 0]], float)


class PointOpsTest(unittest.TestCase):
    def test_move_delete_insert(self):
        m = edits.move_point(LINE, 1, (10, 5))
        self.assertEqual(m[1].tolist(), [10, 5])
        self.assertEqual(LINE[1].tolist(), [10, 0])            # 원본은 그대로
        self.assertEqual(len(edits.delete_points(LINE, [1, 2])), 2)
        self.assertEqual(edits.insert_point(LINE, 0, (5, 1))[1].tolist(), [5, 1])
        with self.assertRaises(edits.EditError):
            edits.delete_points(LINE, [0, 1, 2])               # 2개 미만이 남음
        with self.assertRaises(edits.EditError):
            edits.move_point(LINE, 9, (0, 0))

    def test_smooth_keeps_endpoints_and_reduces_zigzag(self):
        zig = np.array([[x, 3 * (x % 2)] for x in range(0, 40)], float)
        sm = edits.smooth(zig, 3)
        self.assertEqual(sm[0].tolist(), zig[0].tolist())
        self.assertEqual(sm[-1].tolist(), zig[-1].tolist())
        self.assertLess(np.abs(np.diff(edits.densify(sm)[:, 1])).max(), 1.5)
        with self.assertRaises(edits.EditError):
            edits.smooth(zig, 9)

    def test_closed_circle_stays_even_after_smoothing(self):
        t = np.linspace(0, 2 * np.pi, 80, endpoint=False)
        circ = np.column_stack([100 + 20 * np.cos(t), 100 + 20 * np.sin(t)])
        circ = np.vstack([circ, circ[:1]])
        sm = edits.smooth(circ, 3)
        self.assertTrue(np.allclose(sm[0], sm[-1]))
        edges = np.linalg.norm(np.diff(sm, axis=0), axis=1)
        self.assertLess(float(edges.max()), 1.35 * float(np.median(edges)))
        radii = np.linalg.norm(sm[:-1] - (100, 100), axis=1)
        self.assertLess(float(np.abs(radii - 20).max()), 1.5)

    def test_split_and_join_roundtrip(self):
        a, b = edits.split(LINE, 2)
        self.assertEqual(a[-1].tolist(), b[0].tolist())
        j = edits.join(b[::-1], a)                            # 방향이 달라도 가까운 끝끼리
        self.assertAlmostEqual(float(np.hypot(*np.diff(j, axis=0).T).sum()), 30.0)   # 원래 길이 그대로
        with self.assertRaises(edits.EditError):
            edits.split(LINE, 0)


class RemoveMatchingTest(unittest.TestCase):
    def test_removed_shape_matches_nearby_recomputed_stroke(self):
        old = np.array([[10, 10], [90, 10]], float)
        new_same = np.array([[11, 11], [89, 11]], float)       # 다시 계산해 1px 어긋난 같은 선
        other = np.array([[10, 50], [90, 50]], float)
        keep, hit = edits.remove_matching([new_same, other], [old, np.array([[0, 90], [5, 95]], float)],
                                          (100, 100))
        self.assertEqual(keep, [1])
        self.assertEqual(hit, [True, False])                    # 두 번째 지운 모양은 대상 없음 -> 미적용

    def test_stroke_left_of_the_image_does_not_match_a_border_erasure(self):
        # 화면 밖 점을 테두리로 잘라 읽으면, 왼쪽 선이 x=0 삭제선과 겹친 것으로 빠진다
        removed = np.array([[0, 40], [0, 60]], float)
        outside = np.array([[-30, 45], [-10, 55]], float)
        inside = np.array([[10, 50], [90, 50]], float)
        border = np.array([[0, 42], [0, 58]], float)
        keep, hit = edits.remove_matching([outside, inside], [removed], (100, 100))
        self.assertEqual(keep, [0, 1])
        self.assertEqual(hit, [False])
        keep_b, hit_b = edits.remove_matching([border, inside], [removed], (100, 100))
        self.assertEqual(keep_b, [1])                 # 테두리 위의 선은 그대로 지워짐
        self.assertEqual(hit_b, [True])

    def test_no_removed_keeps_all(self):
        self.assertEqual(edits.remove_matching([LINE], [], (10, 40)), ([0], []))


class RenderTest(unittest.TestCase):
    def test_proposals_are_green_and_red(self):
        table = {1: {"kind": "stroke", "poly": np.array([[10, 50], [190, 50]], float), "reason": ""},
                 2: {"kind": "candidate", "poly": np.array([[10, 150], [190, 150]], float), "reason": "small"}}
        views = [{"id": 1, "before": table[1]["poly"], "after": None},
                 {"id": 2, "before": None, "after": table[2]["poly"]}]
        img = edits.render_edit_view((200, 200), table, views, max_px=200)
        self.assertEqual(tuple(int(v) for v in img[50, 100]), edits.RED)
        self.assertEqual(tuple(int(v) for v in img[150, 100]), edits.GREEN)

    def test_candidates_hidden_unless_asked(self):
        table = {1: {"kind": "candidate", "poly": np.array([[10, 100], [190, 100]], float), "reason": "small"}}
        hidden = edits.render_edit_view((200, 200), table, [], max_px=200)
        shown = edits.render_edit_view((200, 200), table, [], show_candidates=True, max_px=200)
        self.assertTrue((hidden == 255).all())
        self.assertFalse((shown == 255).all())

    def test_segment_crossing_a_zoomed_window_is_drawn(self):
        table = {1: {"kind": "stroke", "poly": np.array([[0, 50], [200, 50]], float), "reason": ""}}
        zoom = edits.render_edit_view((100, 220), table, [], region_px=(80, 40, 120, 70), max_px=200)
        self.assertEqual(zoom.shape[:2], (150, 200))
        self.assertTrue((zoom[48:53] != 255).any())          # y=50 선이 창을 가로지름


if __name__ == "__main__":
    unittest.main()
