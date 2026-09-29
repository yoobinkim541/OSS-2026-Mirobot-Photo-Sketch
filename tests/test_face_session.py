"""세션 입력: 원본 해상도 구도 자르기, 얼굴 좌표 변환, 구도가 바뀔 때만 편집 초기화 (검출기는 가짜)."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import faces  # noqa: E402
from mirobot_sketch import session as session_mod  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

import golden  # noqa: E402

W, H = 1156, 1440


def photo(path):
    img = np.full((H, W, 3), 235, np.uint8)
    cv2.ellipse(img, (575, 465), (125, 165), 0, 0, 360, (60, 60, 60), 6)      # 얼굴 윤곽
    cv2.circle(img, (530, 430), 14, (30, 30, 30), -1)
    cv2.circle(img, (620, 430), 14, (30, 30, 30), -1)
    cv2.rectangle(img, (300, 700), (850, 1350), (90, 90, 90), 8)               # 몸
    cv2.line(img, (100, 100), (1000, 200), (40, 40, 40), 5)
    cv2.imwrite(str(path), img)


def face(x, y, w, h):
    lm = np.array([[x + .3 * w, y + .4 * h], [x + .7 * w, y + .4 * h], [x + .5 * w, y + .6 * h],
                   [x + .35 * w, y + .8 * h], [x + .65 * w, y + .8 * h]])
    return faces.Face(np.array([x, y, w, h], float), lm, 0.9)


SMALL = face(450, 300, 250, 330)     # 100mm: 22.9mm → 상반신, 120mm: 27.5mm → 전체
BIG = face(375, 265, 400, 400)       # 100mm: 27.8mm → 전체


class FaceSessionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "p.png"
        photo(self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def session(self, found):
        p = mock.patch.object(session_mod.faces, "detect_faces", lambda img, **kw: list(found))
        p.start()
        self.addCleanup(p.stop)
        s = SketchSession(golden.default_cfg())
        s.set_image(self.path)
        return s

    def test_auto_bust_crops_at_original_resolution(self):
        s = self.session([SMALL])
        s.run_current()
        h, w = s.result["base"].shape
        self.assertEqual(max(h, w), 800)
        self.assertAlmostEqual(w / h, 0.8, delta=0.01)                     # 세로 4:5
        self.assertEqual(s.result["frame"]["kind"], "bust")
        self.assertIn("상반신", s.result["frame"]["notice"])
        self.assertEqual(s.state()["result"]["frame"], "bust")
        self.assertEqual(s.state()["result"]["faces"], 1)

    def test_faces_and_crops_map_to_work_coordinates(self):
        s = self.session([SMALL])
        inputs, key = s._inputs(False, "auto", 100)
        x0, y0, x1, y1 = key[2]
        k = 800 / max(x1 - x0, y1 - y0)
        f = inputs["faces"][0]
        np.testing.assert_allclose(f.box[:2], [(450 - x0) * k, (300 - y0) * k], atol=1e-6)
        c = inputs["face_crops"][0]
        ch, cw = c["img"].shape[:2]
        self.assertEqual(max(ch, cw), 500)
        (ex, ey), _ = faces.ellipse_of(f)
        center = np.array([cw / 2, ch / 2]) / c["scale"] + c["origin"]
        np.testing.assert_allclose(center, [ex, ey], atol=1.5)             # 조각 중심 = 얼굴 타원 중심

    def test_full_frame_keeps_whole_image_and_faces(self):
        s = self.session([SMALL])
        s.update_params({"frame": "full"})
        s.run_current()
        self.assertEqual(s.result["base"].shape, (800, 642))
        self.assertEqual(s.result["frame"]["kind"], "full")
        inputs, key = s._inputs(False, "full", 100)
        self.assertIsNone(key[2])
        self.assertEqual(len(inputs["faces"]), 1)

    def test_no_face_is_unchanged(self):
        s = self.session([])
        s.run_current()
        np.testing.assert_array_equal(s.result["base"], sp.load_gray(self.path))
        self.assertEqual(s.result["frame"], {"kind": "full", "notice": ""})

    def delete_one(self, s):
        sid = next(i for i, e in s.table.items() if e["kind"] == "stroke")
        s.propose_edits([{"op": "delete", "ids": [sid]}], apply_now=True)
        self.assertTrue(s.book["removed"])

    def test_edits_kept_when_frame_box_unchanged(self):
        s = self.session([BIG])
        s.run_current()
        self.delete_one(s)
        s.update_params({"box_mm": 110})
        s.run_current()
        self.assertTrue(s.book["removed"])                                 # 틀 그대로(전체) → 편집 유지

    def test_edits_cleared_when_frame_box_changes(self):
        s = self.session([SMALL])
        s.run_current()
        self.assertEqual(s.result["frame"]["kind"], "bust")
        self.delete_one(s)
        s.update_params({"box_mm": 120})
        s.run_current()
        self.assertEqual(s.result["frame"]["kind"], "full")
        self.assertEqual(s.book, {"removed": [], "added": [], "trash": []})
        self.assertEqual(s.history, [])
        self.assertIn("구도", s.notice)

    def test_failed_run_restores_frame_box(self):
        # 새 구도 계산이 실패하면 틀도 되돌려야 함 (아니면 다음 계산이 멀쩡한 편집을 지우거나 옛 좌표로 다시 씀)
        s = self.session([SMALL])
        s.run_current()
        self.delete_one(s)
        s.update_params({"frame": "full"})
        with mock.patch.object(s, "_refresh_drawing", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                s.run_current()
        self.assertEqual(s.result["frame"]["kind"], "bust")                # 결과는 이전(상반신) 그대로
        s.update_params({"frame": "auto"})
        s.run_current()
        self.assertTrue(s.book["removed"])                                 # 같은 틀 → 편집 유지
        self.assertNotIn("초기화", s.notice)

    def test_dirty_stages_sees_frame_change_from_paper_size(self):
        s = self.session([SMALL])
        s.run_current()
        self.assertEqual(s.dirty_stages()[0], "edit")                      # 다 계산됨
        s.update_params({"box_mm": 105})                                   # 24.1mm: 여전히 상반신 → 종이만
        self.assertNotIn("source", s.dirty_stages())
        s.update_params({"box_mm": 120})                                   # 27.5mm: 전체로 바뀜 → 처음부터
        self.assertEqual(s.dirty_stages()[0], "source")

    def test_rembg_and_plain_inputs_do_not_mix(self):
        s = self.session([SMALL])
        white = np.full((H, W, 3), 255, np.uint8)
        with mock.patch.object(session_mod.sp, "remove_background_bgr", return_value=white) as rb:
            a, ka = s._inputs(True, "auto", 100)
            b, kb = s._inputs(False, "auto", 100)
        self.assertEqual(rb.call_args.kwargs.get("max_side", "missing"), None)  # 원본 크기로 받음
        self.assertNotEqual(ka, kb)
        self.assertTrue((a["color"] == 255).all())
        self.assertFalse((b["color"] == 255).all())


if __name__ == "__main__":
    unittest.main()
