"""실사 선화 경로: 선화를 그대로 따라 그리고, 얼굴은 원본 해상도 조각으로 한 번 더 그린다."""

import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import lineart, stages  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402


class LineartEdgeTest(unittest.TestCase):
    def test_photo_lineart_uses_the_drawing_not_canny(self):
        color = np.full((40, 60, 3), 180, np.uint8)
        drawn = np.full_like(color, 255)
        mask = np.zeros((40, 60), np.uint8)
        mask[10:30, 20] = 255
        params = stages.default_params()
        params["lineart"] = True
        prev = {"color": color, "gray": color[:, :, 0], "prep_color": color, "prep_gray": color[:, :, 0]}
        with mock.patch("mirobot_sketch.lineart.ink_mask", return_value=(drawn, mask)) as ink:
            out = stages._run_edges(prev, params)
        ink.assert_called_once()
        self.assertEqual(int(out["edges"][15, 20]), 255)
        self.assertEqual(int(out["edges"][0, 0]), 0)
        self.assertTrue(out["lineart_used"])
        self.assertIs(out["prep_gray"], prev["prep_gray"])    # 명암 빗금 등 뒤 단계는 사진 밝기를 그대로 씀
        self.assertIs(out["lineart_art"], drawn)

    def test_close_parallel_lineart_lines_both_survive_dedupe(self):
        # 선화의 가까운 두 선(눈꺼풀·입술·머리카락)은 사이가 어두워도 하나로 지우지 않는다 (Canny 경계 쌍만 지움)
        gray = np.full((80, 800), 60, np.uint8)                 # 긴 변 800px → 쌍 간격 2% = 16px
        edges = np.zeros_like(gray)
        edges[30, 20:780] = 255
        edges[40, 20:780] = 255
        strokes = sp.trace_strokes(edges, 5)
        params = {**stages.default_params(), "box_mm": 100}
        prev = {"strokes": strokes, "edges": edges, "prep_gray": gray, "gray": gray, "region": mock.Mock(), "faces": []}
        canny = stages._run_dedupe(prev, params)
        art = stages._run_dedupe({**prev, "lineart_used": True}, params)
        total = lambda out: sum(sp.polyline_length(s) for s in out["strokes"])
        self.assertLess(total(canny), 1.2 * 760)  # Canny 경로: 사이가 먹인 쌍은 하나만
        self.assertGreater(total(art), 1.8 * 760)  # 선화 경로: 둘 다 남김

    def test_thick_blob_becomes_an_outline_but_thin_line_stays(self):
        mask = np.zeros((60, 80), np.uint8)
        mask[20:40, 10:30] = 255                 # 눈동자처럼 두꺼운 먹 덩어리
        mask[10:50, 60:62] = 255                 # 가는 선
        out = lineart.hollow(mask)
        self.assertEqual(int(out[30, 20]), 0)    # 속은 비우고
        self.assertEqual(int(out[20, 20]), 255)  # 테두리는 남김
        self.assertTrue((out[10:50, 60:62] == 255).all())

    def test_unavailable_model_falls_back_to_basic_edges_with_a_reason(self):
        color = np.full((40, 60, 3), 230, np.uint8)
        color[:, 30:] = 40                       # 세로 경계 하나
        gray = color[:, :, 0]
        params = {**stages.default_params(), "lineart": True}
        prev = {"color": color, "gray": gray, "prep_color": color, "prep_gray": gray}
        with mock.patch("mirobot_sketch.lineart.ink_mask", side_effect=lineart.LineartUnavailable("인터넷 없음.")):
            out = stages._run_edges(prev, params)
        self.assertFalse(out.get("lineart_used"))
        self.assertIn("인터넷 없음", out["lineart_missing"])
        self.assertTrue(out["edges"][:, 28:32].any())                    # 기본 선 검출이 경계를 찾음


class ModelDownloadTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.src = self.root / "release.onnx"
        self.src.write_bytes(b"model-bytes")
        patcher = mock.patch("mirobot_sketch.paths.cache_dir", return_value=self.root / "cache")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_verified_download_is_kept(self):
        good = hashlib.sha256(b"model-bytes").hexdigest()
        with mock.patch.object(lineart, "MODEL_SHA256", good):
            path = lineart.ensure_model(url=self.src.as_uri())
        self.assertEqual(path.read_bytes(), b"model-bytes")
        self.assertEqual(lineart.ensure_model(url="http://invalid.invalid/x"), path)   # 있으면 다시 받지 않음

    def test_corrupt_or_failed_download_leaves_no_file(self):
        with self.assertRaises(lineart.LineartUnavailable):
            lineart.ensure_model(url=self.src.as_uri())                 # SHA256 불일치
        with self.assertRaises(lineart.LineartUnavailable):
            lineart.ensure_model(url=(self.root / "missing.onnx").as_uri())
        self.assertEqual(list((self.root / "cache" / "models").iterdir()), [])


class LineartMergeTest(unittest.TestCase):
    def test_dashed_line_is_joined_and_stray_dash_dropped(self):
        dashes = [np.array([[x, 50.0] for x in range(x0, x0 + 20)]) for x0 in (0, 26, 52)]
        stray = np.array([[x, 10.0] for x in range(100, 104)])            # 홀로 떨어진 짧은 토막 (LINEART_MIN_PX 미만)
        params = stages.default_params()
        out = stages._run_merge({"strokes": dashes + [stray], "lineart_used": True}, {**params, "merge_join_px": 0})
        self.assertEqual(len(out["strokes"]), 1)
        self.assertGreater(sp.polyline_length(out["strokes"][0]), 65)


class LineartFaceTest(unittest.TestCase):
    def test_face_is_redrawn_from_the_crop_on_the_lineart_path_too(self):
        from mirobot_sketch import faces as fc
        face = fc.Face(np.array([20.0, 20.0, 40.0, 40.0]), np.array([[30, 35], [50, 35], [40, 45], [32, 55], [48, 55]], float), 1.0)
        inside = np.array([[35.0, 40.0], [45.0, 40.0]])
        outside = np.array([[2.0, 90.0], [30.0, 90.0]])
        fine = np.array([[33.0, 50.0], [47.0, 50.0]])
        prev = {"strokes": [inside, outside], "faces": [face], "face_crops": [{}], "lineart_used": True,
                "gray": np.zeros((100, 100), np.uint8), "region": None}
        params = {**stages.default_params(), "face_detail": True, "declutter": False}   # 잔선 정리는 test_declutter
        with mock.patch.object(stages, "_face_strokes", return_value=[fine]) as redo:
            out = stages._run_face(prev, params)
        redo.assert_called_once()
        self.assertEqual(out["faces_used"], 1)
        pts = np.vstack(out["strokes"])
        self.assertTrue(any(np.allclose(p, [33.0, 50.0]) for p in pts))          # 얼굴 조각에서 다시 딴 선
        self.assertFalse(any(np.allclose(p, [35.0, 40.0]) for p in pts))         # 얼굴 안의 원래 선은 교체
        self.assertTrue(any(np.allclose(p, [2.0, 90.0]) for p in pts))           # 얼굴 밖은 그대로


class JoinGapsTest(unittest.TestCase):
    def test_dashed_line_is_joined_but_a_corner_is_not(self):
        dashes = [np.array([[x, 50.0] for x in range(x0, x0 + 20)]) for x0 in (0, 26, 52)]
        self.assertEqual(len(sp.join_gaps(dashes)), 1)                     # 점선 -> 한 줄
        corner = [np.array([[x, 50.0] for x in range(0, 20)]), np.array([[24.0, y] for y in range(55, 80)])]
        self.assertEqual(len(sp.join_gaps(corner)), 2)                     # 직각으로 꺾이면 잇지 않음

    def test_dashed_circle_closes_into_one_loop_but_a_short_hook_stays_open(self):
        def arc(a0, a1, r=40.0):
            t = np.radians(np.arange(a0, a1, 3.0))
            return np.column_stack([100 + r * np.cos(t), 100 + r * np.sin(t)])
        circle = sp.join_gaps([arc(a, a + 80) for a in (0, 90, 180, 270)], 12)
        self.assertEqual(len(circle), 1)
        self.assertTrue(sp.is_closed(circle[0]))                          # 눈동자·얼굴 윤곽 같은 점선 고리
        hook = sp.join_gaps([arc(0, 330, r=4.0)], 12)
        self.assertFalse(sp.is_closed(hook[0]))                           # 짧은 갈고리는 억지로 닫지 않음


if __name__ == "__main__":
    unittest.main()
