"""얼굴 세밀 단계: 타원 안 획 교체, 경계 자르기, 펜 굵기 밀도 제한, 캐시 (합성 이미지, 검출기 없음)."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import faces  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402
from mirobot_sketch import stages  # noqa: E402

N = 400                                   # 작업 이미지 400×400
FACE = faces.Face(np.array([150.0, 150, 100, 120]),
                  np.array([[180, 198], [220, 198], [200, 222], [185, 246], [215, 246]], float), 0.9)
CENTER, AXES = faces.ellipse_of(FACE)     # (200, 210), (65, 84)


def params(**kw):
    return {**stages.default_params(), "edge_mode": "dark", **kw}


def crop_of(draw):
    """작업 이미지 전체를 2배로 키운 조각 (scale 2, origin 0) 위에 draw(img)로 그림."""
    img = np.full((2 * N, 2 * N, 3), 255, np.uint8)
    draw(img)
    return {"img": img, "scale": 2.0, "origin": np.array([0.0, 0.0])}


def prev(strokes, crop, faces_=(FACE,)):
    gray = np.full((N, N), 255, np.uint8)
    return {"gray": gray, "color": cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), "strokes": strokes,
            "faces": list(faces_), "face_crops": [crop] * len(faces_)}


def inside(pt, tol=1.0):
    (cx, cy), (ax, ay) = CENTER, AXES
    return ((pt[0] - cx) / (ax + tol)) ** 2 + ((pt[1] - cy) / (ay + tol)) ** 2 <= 1


FAR = np.array([[10, 10], [60, 10], [60, 50]], np.int32)
CROSS = np.array([[0, 210], [400, 210]], np.int32)                    # 타원을 가로지름
INNER = np.array([[190, 200], [210, 200], [210, 220]], np.int32)       # 타원 안


class FaceStageTest(unittest.TestCase):
    def test_no_face_or_off_passes_strokes_through(self):
        base = [FAR, CROSS]
        for pv, p in ((prev(base, crop_of(lambda i: None), faces_=()), params()),
                      (prev(base, crop_of(lambda i: None)), params(face_detail=False))):
            out = stages._run_face(pv, p)
            self.assertEqual(len(out["strokes"]), 2)
            for a, b in zip(out["strokes"], base):
                self.assertIs(a, b)
            self.assertEqual(out["discarded_face"], [])

    def test_inside_replaced_crossing_line_clipped(self):
        out = stages._run_face(prev([FAR, CROSS, INNER], crop_of(lambda i: None)), params())   # 빈 조각
        st = out["strokes"]
        self.assertTrue(any(s is FAR for s in st))                                  # 먼 획은 그대로
        pieces = [s for s in st if s is not FAR]
        self.assertEqual(len(pieces), 2)                                            # 가로선: 바깥 두 조각
        for s in pieces:
            self.assertTrue(all(not inside(p, tol=-1.0) for p in s))
        self.assertAlmostEqual(sum(sp.polyline_length(s) for s in pieces), 400 - 2 * 65, delta=3)
        gone = [q for q, why in out["discarded_face"]]
        self.assertEqual(len(gone), 2)                                              # 가로선 안쪽 + INNER
        self.assertTrue(all(why == "얼굴 세밀 처리로 교체" for _, why in out["discarded_face"]))

    def test_face_strokes_added_inside_ellipse_only(self):
        def draw(img):
            cv2.ellipse(img, (400, 396), (40, 16), 0, 0, 360, (0, 0, 0), 3)       # 작업 좌표 (200,198) 부근 눈
            cv2.line(img, (20, 400), (780, 400), (0, 0, 0), 3)                  # 조각 전체를 가로지름
        out = stages._run_face(prev([FAR], crop_of(draw)), params())
        added = [s for s in out["strokes"] if s is not FAR]
        self.assertTrue(added)
        for s in added:
            self.assertTrue(all(inside(p) for p in s))                             # 타원 밖은 잘림
        self.assertEqual(out["faces_used"], 1)

    def parallel(self, img):
        for y in range(330, 520, 7):                                            # 조각 7px(작업 3.5px) 간격 선
            cv2.line(img, (330, y), (470, y), (0, 0, 0), 2)

    def total(self, **kw):
        out = stages._run_face(prev([], crop_of(self.parallel)), params(**kw))
        return sum(sp.polyline_length(s) for s in out["strokes"])

    def test_density_follows_pen_width_and_paper_size(self):
        self.assertGreater(self.total(pen_mm=0.5), self.total(pen_mm=1.5) * 1.5)
        self.assertGreater(self.total(box_mm=120), self.total(box_mm=50) * 1.5)

    def test_short_strokes_kept_only_near_landmarks(self):
        def draw(img):
            cv2.line(img, (356, 396), (368, 396), (0, 0, 0), 3)                 # 눈(180,198) 옆 짧은 선
            cv2.line(img, (388, 280), (400, 280), (0, 0, 0), 3)                 # 이마(197,140): 먼 짧은 선
        out = stages._run_face(prev([], crop_of(draw)), params(pen_mm=1.0, box_mm=100))
        ys = [np.mean(np.asarray(s)[:, 1]) for s in out["strokes"]]
        self.assertTrue(any(abs(y - 198) < 3 for y in ys))
        self.assertFalse(any(abs(y - 140) < 3 for y in ys))


class FaceStageWiringTest(unittest.TestCase):
    def test_order_titles_and_candidates(self):
        ids = list(stages.PIPELINE_IDS)
        self.assertEqual(ids.index("face"), ids.index("merge") + 1)
        self.assertEqual(ids.index("simplify"), ids.index("face") + 1)
        self.assertEqual(ids.index("tone"), ids.index("simplify") + 1)
        self.assertTrue(stages.stage_title("paper").startswith("⑪"))
        outs = {"simplify": {"discarded_trace": [], "discarded_dedupe": [],
                             "discarded_face": [(FAR, "얼굴 세밀 처리로 교체")]}}
        self.assertEqual(len(stages.candidates_of(outs)), 1)

    def test_paper_size_reruns_face_stage_only(self):
        def draw(img):
            cv2.circle(img, (400, 420), 60, (0, 0, 0), 3)
        work = np.full((N, N, 3), 255, np.uint8)
        cv2.circle(work, (200, 210), 30, (0, 0, 0), 2)
        cv2.line(work, (20, 350), (380, 350), (0, 0, 0), 2)
        inputs = {"gray": cv2.cvtColor(work, cv2.COLOR_BGR2GRAY), "color": work,
                  "faces": [FACE], "face_crops": [crop_of(draw)]}
        pl, p = stages.Pipeline(), params()
        pl.run(inputs, "k", p)
        before = dict(pl.run_counts)
        pl.run(inputs, "k", {**p, "box_mm": 60})
        self.assertEqual(pl.run_counts["face"], before["face"] + 1)
        self.assertEqual(pl.run_counts["trace"], before["trace"])


    def test_paper_size_does_not_rerun_without_faces(self):
        # 얼굴이 없으면 종이 크기는 얼굴 단계에 쓰이지 않음 → 번호·제안이 유지되게 뒤 단계도 다시 계산하지 않음
        work = np.full((N, N, 3), 255, np.uint8)
        cv2.line(work, (20, 350), (380, 350), (0, 0, 0), 2)
        for found, key in (([], "a"), ([FACE], "b")):
            inputs = {"gray": cv2.cvtColor(work, cv2.COLOR_BGR2GRAY), "color": work,
                      "faces": found, "face_crops": [crop_of(lambda i: None)] * len(found)}
            pl, p = stages.Pipeline(), params()
            first = pl.run(inputs, key, p)
            again = pl.run(inputs, key, {**p, "box_mm": 60})
            self.assertEqual(again["simplify"] is first["simplify"], not found)
            self.assertEqual(pl.dirty_from(key, {**p, "box_mm": 70}) is None, not found)
        pl, p = stages.Pipeline(), params(face_detail=False)                  # 꺼도 마찬가지
        inputs = {"gray": cv2.cvtColor(work, cv2.COLOR_BGR2GRAY), "color": work,
                  "faces": [FACE], "face_crops": [crop_of(lambda i: None)]}
        first = pl.run(inputs, "c", p)
        self.assertIs(pl.run(inputs, "c", {**p, "box_mm": 60})["simplify"], first["simplify"])


if __name__ == "__main__":
    unittest.main()
