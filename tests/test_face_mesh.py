"""얼굴 점으로 눈·다문 입 다시 그리기: 합성 얼굴 점(478개 중 쓰는 번호만 배치)으로 검사."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import face_mesh as fm  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402

PEN = 4.0


def _arc(idx, x0, x1, y, bulge, pts):
    t = np.linspace(0, 1, len(idx))
    for i, tt in zip(idx, t):
        pts[i] = (x0 + (x1 - x0) * tt, y + bulge * np.sin(np.pi * tt))


def mesh(mouth_gap=0.0, right_eye_w=40.0):
    """눈 폭 40px·높이 16px, 홍채 반지름 7px, 입 폭 60px인 정면 얼굴. 오른눈(그림 왼쪽)은 폭을 바꿀 수 있음."""
    pts = np.zeros((478, 2))
    for eye, iris, (cx, w) in zip(fm.EYES, fm.IRISES, ((100.0, right_eye_w), (200.0, 40.0))):
        outer, inner = (cx - w / 2, cx + w / 2) if eye is fm.EYES[0] else (cx + w / 2, cx - w / 2)
        _arc(eye["upper"], outer, inner, 100.0, -8.0, pts)
        _arc(eye["lower"], outer, inner, 100.0, 8.0, pts)
        t = np.linspace(0, 2 * np.pi, 4, endpoint=False)
        pts[list(iris)] = np.vstack([[cx, 98.0], np.column_stack([cx + 7 * np.cos(t), 98.0 + 7 * np.sin(t)])])
    _arc(fm.LIP_OUTER_UP, 120, 180, 200.0, -10.0, pts)
    _arc(fm.LIP_OUTER_LOW, 120, 180, 200.0, 12.0, pts)
    _arc(fm.LIP_INNER_UP, 120, 180, 200.0, -mouth_gap / 2, pts)
    _arc(fm.LIP_INNER_LOW, 120, 180, 200.0, mouth_gap / 2, pts)
    pts[61], pts[291] = (120.0, 200.0), (180.0, 200.0)
    return pts


def line(x0, y0, x1, y1, n=20):
    return np.column_stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)])


class FaceMeshRedrawTest(unittest.TestCase):
    def setUp(self):
        self.lips = [line(120, 195, 180, 195), line(120, 206, 180, 206)]   # 다문 입술의 위·아래 윤곽 (선화)
        self.eye_noise = [line(95, 97, 105, 97, 6)]                          # 눈 속 반사광 토막
        self.cheek = [line(20, 150, 60, 150)]                                # 얼굴의 다른 선

    def test_closed_mouth_becomes_a_seam_and_eye_noise_goes(self):
        strokes = self.lips + self.eye_noise + self.cheek
        out, gone, mouths = fm.redraw(strokes, [mesh(0.0)], PEN, (300, 300))
        self.assertEqual(mouths, {0})
        self.assertTrue(any(np.allclose(q, self.cheek[0]) for q in out))       # 얼굴 다른 선은 그대로
        self.assertGreaterEqual(len(gone), 3)                                   # 입술 두 줄·눈 속 토막을 지움
        seam = [q for q in out if abs(q[0, 0] - 120) < 1 and abs(q[-1, 0] - 180) < 1]
        self.assertEqual(len(seam), 1)                                          # 입꼬리에서 입꼬리까지 한 줄
        self.assertLess(float(np.abs(seam[0][:, 1] - 200).max()), 1.0)
        for q in out:                                                           # 입술 바깥선은 입꼬리에 닿지 않음
            if q is not seam[0] and 190 < q[:, 1].mean() < 215:
                self.assertGreater(q[:, 0].min(), 125)
                self.assertLess(q[:, 0].max(), 175)

    def test_open_mouth_is_left_to_the_old_mouth_tidy(self):
        out, gone, mouths = fm.redraw(self.lips, [mesh(14.0)], PEN, (300, 300))
        self.assertEqual(mouths, set())
        self.assertTrue(all(any(np.allclose(q, p) for q in out) for p in self.lips))

    def test_iris_is_clipped_by_the_eyelids_and_gets_a_pupil_dot(self):
        out, _, _ = fm.redraw([], [mesh(0.0)], PEN, (300, 300))
        eye = mesh(0.0)
        poly = np.vstack([eye[list(fm.EYES[0]["upper"])], eye[list(fm.EYES[0]["lower"])][::-1][1:-1]])
        near = [q for q in out if np.linalg.norm(q.mean(axis=0) - (100, 98)) < 8]
        self.assertTrue(near)
        for q in near:
            for p in q:
                self.assertGreaterEqual(cv2.pointPolygonTest(poly.astype(np.float32), (float(p[0]), float(p[1])), True),
                                        -1.5)
        dots = [q for q in near if np.ptp(q[:, 0]) < PEN]
        self.assertEqual(len(dots), 1)                                          # 눈동자 점 하나

    def test_turned_face_is_not_redrawn(self):
        strokes = self.lips + self.eye_noise
        out, gone, mouths = fm.redraw(strokes, [mesh(0.0, right_eye_w=16.0)], PEN, (300, 300))
        self.assertEqual((len(out), gone, mouths), (len(strokes), [], set()))
        out, gone, mouths = fm.redraw(strokes, [None], PEN, (300, 300))         # 얼굴 점을 못 찾은 얼굴
        self.assertEqual((len(out), gone, mouths), (len(strokes), [], set()))


class RoiMappingTest(unittest.TestCase):
    def test_model_points_map_back_through_the_rotated_crop(self):
        class FakeSession:                       # 모델 입력 가운데와 오른쪽 가장자리 가운데에 점을 내는 가짜
            def get_inputs(self):
                return [type("I", (), {"name": "x"})()]

            def run(self, _, feed):
                self.seen = feed["x"]
                pts = np.zeros((478, 3), np.float32)
                pts[:, :2] = 128
                pts[1, :2] = (256, 128)
                return [pts.reshape(1, 1, 1, -1), np.array([[[[3.0]]]], np.float32)]

        fake = FakeSession()
        old = fm._session
        fm._session = fake
        try:
            img = np.zeros((400, 400, 3), np.uint8)
            pts, score = fm._run(img, (200.0, 150.0), 120.0, np.radians(30))
        finally:
            fm._session = old
        self.assertEqual(fake.seen.shape, (1, 256, 256, 3))
        np.testing.assert_allclose(pts[0], (200, 150), atol=1e-6)
        np.testing.assert_allclose(pts[1], (200 + 60 * np.cos(np.radians(30)), 150 + 60 * np.sin(np.radians(30))),
                                   atol=1e-6)
        self.assertEqual(score, 3.0)
        c, size, ang = fm._roi(np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [0.0, 4.0]]), (0, 0), (1, 0))
        np.testing.assert_allclose(c, (5, 2))
        self.assertAlmostEqual(size, 15.0)                                  # 긴 변 10 × 1.5
        self.assertAlmostEqual(ang, 0.0)


class JawFillTest(unittest.TestCase):
    def test_only_uncovered_visible_jaw_runs_are_filled(self):
        pts = mesh(0.0)
        t = np.linspace(0, np.pi, len(fm.JAW))
        pts[list(fm.JAW)] = np.column_stack([150 - 80 * np.cos(t), 150 + 100 * np.sin(t)])   # 귀 → 턱 끝 → 귀
        contrast = np.zeros((300, 300), np.float32)
        contrast[:, :200] = 100.0                         # 오른쪽 끝(x ≥ 200)은 사진에 경계가 안 보임
        left = np.column_stack([150 - 80 * np.cos(t[:6]), 150 + 100 * np.sin(t[:6])])       # 선화가 그린 왼쪽 턱
        out, added = fm.fill_jaw([left], [pts], contrast, PEN)
        self.assertTrue(added)
        for q in added:
            self.assertTrue((q[:, 0] < 201).all())                                      # 안 보이는 곳은 안 채움
            gap = np.min(np.linalg.norm(q[:, None, :] - left[None, :, :], axis=2), axis=1)
            self.assertTrue((gap > fm.JAW_COVER_PEN * PEN - 1.5).all())                # 이미 그린 곳은 안 채움
        self.assertEqual(len(out), 1 + len(added))


class TextMaskTest(unittest.TestCase):
    def test_text_boxes_are_masked_except_inside_the_face(self):
        from mirobot_sketch import faces as fc
        from mirobot_sketch import stages
        face = fc.Face(np.array([100.0, 100.0, 80.0, 80.0]), np.zeros((5, 2)), 1.0)
        caption = np.array([[10, 250], [90, 250], [90, 270], [10, 270]], float)       # 얼굴 밖 자막
        on_face = np.array([[130, 130], [150, 130], [150, 140], [130, 140]], float)    # 얼굴 안 (오검출로 봄)
        prev = {"gray": np.zeros((300, 300), np.uint8), "faces": [face], "text_boxes": [caption, on_face]}
        mask = stages._text_mask(prev, pen_px=2.0)
        self.assertTrue(mask[260, 50] and mask[249, 50])                               # 펜 굵기만큼 넓힘
        self.assertFalse(mask[135, 140])
        kept, gone = sp.cut_by_mask([np.array([[0, 260], [150, 260]], float)], mask)
        self.assertEqual(len(gone), 1)
        self.assertTrue(all(q[:, 0].max() < 9 or q[:, 0].min() > 91 for q in kept))


if __name__ == "__main__":
    unittest.main()
